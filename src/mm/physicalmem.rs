use alloc::vec::Vec;
use core::alloc::AllocError;
use core::fmt;
use core::sync::atomic::{AtomicUsize, Ordering};

use align_address::Align;
use free_list::{FreeList, PageLayout, PageRange};
use hermit_sync::InterruptTicketMutex;
use memory_addresses::VirtAddr;

#[cfg(all(target_arch = "x86_64", feature = "hermit-entry"))]
use crate::arch::mm::paging::PageTableEntryFlagsExt;
use crate::arch::mm::paging::{self, HugePageSize, LargePageSize, PageSize};
use crate::env::{self, MemmapType, StartInfo};
use crate::mm::device_alloc::DeviceAlloc;
use crate::mm::{PageRangeAllocator, PageRangeBox};

static PHYSICAL_FREE_LIST: InterruptTicketMutex<FreeList<16>> =
	InterruptTicketMutex::new(FreeList::new());
pub static TOTAL_MEMORY: AtomicUsize = AtomicUsize::new(0);

pub struct FrameAlloc;

impl PageRangeAllocator for FrameAlloc {
	unsafe fn init() {
		unsafe {
			init();
		}
	}

	fn allocate(layout: PageLayout) -> Result<PageRange, AllocError> {
		PHYSICAL_FREE_LIST
			.lock()
			.allocate(layout)
			.map_err(|_| AllocError)
	}

	fn allocate_at(range: PageRange) -> Result<(), AllocError> {
		PHYSICAL_FREE_LIST
			.lock()
			.allocate_at(range)
			.map_err(|_| AllocError)
	}

	unsafe fn deallocate(range: PageRange) {
		unsafe {
			PHYSICAL_FREE_LIST.lock().deallocate(range).unwrap();
		}
	}
}

impl fmt::Display for FrameAlloc {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		let free_list = PHYSICAL_FREE_LIST.lock();
		write!(f, "FrameAlloc free list:\n{free_list}")
	}
}

pub type FrameBox = PageRangeBox<FrameAlloc>;

pub fn total_memory_size() -> usize {
	TOTAL_MEMORY.load(Ordering::Relaxed)
}

#[cfg(feature = "hermit-entry")]
pub unsafe fn map_frame_range(frame_range: PageRange) {
	use memory_addresses::PhysAddr;

	use crate::arch::mm::paging::PageTableEntryFlags;

	cfg_select! {
		target_arch = "aarch64" => {
			type IdentityPageSize = paging::BasePageSize;
		}
		target_arch = "riscv64" => {
			type IdentityPageSize = HugePageSize;
		}
		target_arch = "x86_64" => {
			type IdentityPageSize = LargePageSize;
		}
	}

	let start = frame_range
		.start()
		.align_down(IdentityPageSize::SIZE.try_into().unwrap());
	let end = frame_range
		.end()
		.align_up(IdentityPageSize::SIZE.try_into().unwrap());

	(start..end)
		.step_by(IdentityPageSize::SIZE.try_into().unwrap())
		.map(|addr| PhysAddr::new(addr.try_into().unwrap()))
		.for_each(paging::identity_map::<IdentityPageSize>);

	// Map the physical memory again if DeviceAlloc operates at an offset
	if DeviceAlloc.phys_offset() != VirtAddr::zero() {
		let flags = {
			let mut flags = PageTableEntryFlags::empty();
			flags.normal().writable().execute_disable();
			flags
		};
		(start..end)
			.step_by(IdentityPageSize::SIZE.try_into().unwrap())
			.for_each(|addr| {
				let phys_addr = PhysAddr::new(addr.try_into().unwrap());
				let virt_addr = VirtAddr::from_ptr(DeviceAlloc.ptr_from::<()>(phys_addr));
				paging::map::<IdentityPageSize>(virt_addr, phys_addr, 1, flags);
			});
	}
}

/// End of the kernel image in memory: the linker's `_end`, or the
/// loader-reported extent when that is larger (the loader maps every
/// PT_LOAD, including segments appended to the ELF post-link, which the
/// linker never saw).
fn kernel_image_end() -> usize {
	let linked_end = elf_symbols::executable_end().addr();
	match env::loaded_image_end() {
		Some(loaded_end) => linked_end.max(loaded_end),
		None => linked_end,
	}
}

unsafe fn detect_from_start_info() {
	// On x86_64 with Linux boot params, build a list of non-RAM regions from
	// the E820 table so we can skip them when claiming memory-map entries.
	// The loader's FDT lists all physical address ranges (including MMIO
	// holes) as "/memory" nodes without distinguishing RAM from reserved, so
	// without this filter the allocator would identity-map and hand out
	// addresses in reserved MMIO space (e.g. Firecracker's hole at
	// 0xeec00000).
	#[cfg(all(target_arch = "x86_64", feature = "hermit-entry"))]
	#[allow(deprecated)]
	let e820_reserved: heapless::Vec<(usize, usize), 32> = {
		use hermit_entry::fc;

		/// Reads a `T` from the identity-mapped physical address `addr`.
		unsafe fn read<T: Copy>(addr: usize) -> T {
			unsafe { core::ptr::with_exposed_provenance::<T>(addr).read_unaligned() }
		}

		let mut v = heapless::Vec::new();
		if let Some(boot_params_addr) = env::boot_params_addr() {
			let bp = boot_params_addr.get();
			let nentries = unsafe { read::<u8>(bp + fc::E820_ENTRIES_OFFSET) } as usize;
			let table_base = bp + fc::E820_TABLE_OFFSET;
			for i in 0..nentries {
				let entry = table_base + i * 20;
				let start = unsafe { read::<u64>(entry) } as usize;
				let size = unsafe { read::<u64>(entry + 8) } as usize;
				let typ = unsafe { read::<u32>(entry + 16) };
				if typ != 1 && size > 0 {
					let _ = v.push((start, start + size));
				}
			}
		}
		v
	};
	#[cfg(not(all(target_arch = "x86_64", feature = "hermit-entry")))]
	let e820_reserved: heapless::Vec<(usize, usize), 32> = heapless::Vec::new();

	for memmap_entry in env::start_info().memmap() {
		if memmap_entry.ty != MemmapType::Ram {
			continue;
		}

		// Skip memory-map entries that fall entirely within an E820 non-RAM region.
		let entry_start = memmap_entry.phys_addr;
		let entry_end = entry_start + memmap_entry.len;
		if e820_reserved
			.iter()
			.any(|&(rs, re)| rs <= entry_start && entry_end <= re)
		{
			continue;
		}

		let mut start_addr = memmap_entry.phys_addr;
		let mut end_addr = start_addr + memmap_entry.len;

		// Do not free any address in the real mode addressable range.
		//
		// When claiming physical memory, ignore all addresses below this one. This ensures we
		// don't accidentally clash with hardcoded low addresses, such as `SMP_BOOT_CODE_ADDRESS`
		// in x86_64 with SMP enabled. We use a 2MIB size for now, but this is arbitrary, and could
		// likely be lowered.
		start_addr = start_addr.max(LargePageSize::SIZE as usize);

		// Do not claim any memory below the kernel image.
		//
		// The loader may place its own image, the kernel's boot stack, and the
		// page tables the kernel is running on anywhere below the kernel image
		// without announcing them as FDT memory reservations. hermit-loader
		// 0.5.6's x86_64 Linux-boot image is linked at 0x200000 and contains
		// the live root page table; claiming that region eventually hands the
		// running page tables out as allocations, which ends in a triple
		// fault. This also covers the recursive-page-table case of loader
		// 0.5.6 that was previously special-cased via `paging::is_recursive`.
		//
		// The image can end ABOVE the linker's `_end`: the loader maps every
		// PT_LOAD, including segments appended post-link (bundled artifacts),
		// and reports the true extent in its boot info. Respect both.
		start_addr = start_addr.max(kernel_image_end());

		start_addr = start_addr.align_up(0x1000);
		end_addr = end_addr.align_down(0x1000);

		if start_addr > end_addr {
			continue;
		}

		let range = PageRange::new(start_addr, end_addr).unwrap();
		unsafe {
			FrameAlloc::deallocate(range);
		}
	}

	let reserve = |reservation: PageRange| {
		debug!("Memory reservation: {reservation:#x?}");
		// While there are still overlaps between this reservation and any available ranges,
		// allocate that overlap to mark it as not available.
		while let Ok(reserved) = PHYSICAL_FREE_LIST
			.lock()
			.allocate_with(|range| reservation.and(range))
		{
			debug!("Reserved {reserved:#x?}");
		}
	};

	let kernel_start = elf_symbols::executable_start().addr();
	let kernel_end = kernel_image_end();
	let kernel_region = PageRange::containing(kernel_start, kernel_end).unwrap();
	reserve(kernel_region);

	for module in env::start_info().modules() {
		reserve(module.phys_frame_range());
	}

	#[cfg(feature = "hermit-entry")]
	{
		use crate::env::FdtStartInfo;

		let fdt = env::start_info().fdt().unwrap();

		for reservation in fdt.memory_reservations() {
			let start = reservation.address().addr();
			let end = start + reservation.size();
			let reservation = PageRange::new(start, end).unwrap();
			reserve(reservation);
		}

		let fdt_start = env::start_info().fdt_addr().unwrap().get();
		let fdt_end = fdt_start + fdt.total_size();
		let fdt_region = PageRange::containing(fdt_start, fdt_end).unwrap();
		reserve(fdt_region);
	}

	let frame_ranges = PHYSICAL_FREE_LIST.lock().iter().collect::<Vec<_>>();

	for frame_range in frame_ranges {
		#[cfg(feature = "hermit-entry")]
		unsafe {
			map_frame_range(frame_range);
		}
		debug!("Claimed physical memory: {frame_range:#x?}");
	}

	TOTAL_MEMORY.store(PHYSICAL_FREE_LIST.lock().free_space(), Ordering::Relaxed);
}

unsafe fn init() {
	if cfg!(target_arch = "x86_64") && DeviceAlloc.phys_offset() != VirtAddr::zero() {
		let start = DeviceAlloc.phys_offset();
		let count = DeviceAlloc.phys_offset().as_u64() / HugePageSize::SIZE;
		let count = usize::try_from(count).unwrap();
		paging::unmap::<HugePageSize>(start, count);
	}

	unsafe {
		detect_from_start_info();
	}
}
