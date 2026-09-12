//! Memory management syscalls.
//!
//! These system calls are similar to [sys/mman.h].
//!
//! <div class="warning">These system calls are not very POSIX-like yet!</div>
//!
//! [sys/mman.h]: https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_mman.h.html

use core::ffi::{c_int, c_void};

use align_address::Align;
use free_list::{FreeList, PageLayout, PageRange};
use hermit_sync::SpinMutex;
use memory_addresses::{PhysAddr, VirtAddr};

#[cfg(target_arch = "x86_64")]
use crate::arch::mm::paging::PageTableEntryFlagsExt;
use crate::arch::mm::paging::{self, BasePageSize, PageSize, PageTableEntryFlags};
use crate::errno::Errno;
use crate::mm::{FrameAlloc, PageAlloc, PageRangeAllocator};

bitflags! {
	#[repr(transparent)]
	#[derive(Debug, Copy, Clone, Default)]
	pub struct MemoryProtection: u32 {
		/// Pages may not be accessed.
		const None = 0;
		/// Indicates that the memory region should be readable.
		const Read = 1 << 0;
		/// Indicates that the memory region should be writable.
		const Write = 1 << 1;
		/// Indicates that the memory region should be executable.
		const Exec = 1 << 2;
	}
}

/// Virtual address ranges handed out by [`sys_mmap`] and not yet returned via
/// [`sys_munmap`].
///
/// [`sys_munmap`] only accepts ranges that are recorded here, so stray or
/// repeated unmaps cannot corrupt [`PageAlloc`] or unmap kernel mappings.
/// Adjacent ranges coalesce, so one `munmap` may span several `mmap`s, and a
/// `munmap` of a sub-range splits the recorded range accordingly.
static MMAP_RANGES: SpinMutex<FreeList<16>> = SpinMutex::new(FreeList::new());

fn page_table_flags(prot_flags: MemoryProtection) -> PageTableEntryFlags {
	let mut flags = PageTableEntryFlags::empty();
	flags.normal();
	if prot_flags.contains(MemoryProtection::Write) {
		flags.writable();
	}
	if !prot_flags.contains(MemoryProtection::Exec) {
		flags.execute_disable();
	}
	flags
}

/// Creates a new virtual memory mapping of the `size` specified with
/// protection bits specified in `prot_flags`.
#[hermit_macro::system(errno)]
#[unsafe(no_mangle)]
pub extern "C" fn sys_mmap(size: usize, prot_flags: MemoryProtection, ret: &mut *mut u8) -> i32 {
	let size = size.align_up(BasePageSize::SIZE as usize);
	if size == 0 {
		return -i32::from(Errno::Inval);
	}
	let layout = PageLayout::from_size(size).unwrap();
	let Ok(page_range) = PageAlloc::allocate(layout) else {
		return -i32::from(Errno::Nomem);
	};
	let virtual_address = VirtAddr::from(page_range.start());

	// `MemoryProtection::None` reserves address space only: no frames are
	// allocated and no pages are mapped.
	if !prot_flags.is_empty() {
		let frame_layout = PageLayout::from_size(size).unwrap();
		let Ok(frame_range) = FrameAlloc::allocate(frame_layout) else {
			unsafe {
				PageAlloc::deallocate(page_range);
			}
			return -i32::from(Errno::Nomem);
		};
		let physical_address = PhysAddr::from(frame_range.start());

		debug!("Mmap {physical_address:X} -> {virtual_address:X} ({size})");
		let count = size / BasePageSize::SIZE as usize;
		paging::map::<BasePageSize>(
			virtual_address,
			physical_address,
			count,
			page_table_flags(prot_flags),
		);
	}

	unsafe {
		MMAP_RANGES.lock().deallocate(page_range).unwrap();
	}

	*ret = virtual_address.as_mut_ptr();

	0
}

/// Unmaps memory at the specified `ptr` for `size` bytes.
#[hermit_macro::system(errno)]
#[unsafe(no_mangle)]
pub extern "C" fn sys_munmap(ptr: *mut u8, size: usize) -> i32 {
	let virtual_address = VirtAddr::from_ptr(ptr);
	if !virtual_address.is_aligned_to(BasePageSize::SIZE) {
		return -i32::from(Errno::Inval);
	}
	let size = size.align_up(BasePageSize::SIZE as usize);
	if size == 0 {
		return -i32::from(Errno::Inval);
	}
	let page_range = PageRange::from_start_len(virtual_address.as_usize(), size).unwrap();

	// Claim the range from the bookkeeping of live mappings. Failure means
	// (part of) the range was not obtained from `sys_mmap`, so unmapping it
	// could free frames or address space owned by someone else.
	if MMAP_RANGES.lock().allocate_at(page_range).is_err() {
		return -i32::from(Errno::Inval);
	}

	debug!("Munmap {virtual_address:X} ({size})");

	// Unmap and free page by page: the range may be backed by multiple
	// physically discontiguous frame allocations (e.g. committed piecewise
	// via `sys_mprotect`), and pages of a `MemoryProtection::None` reservation
	// have no frames at all.
	for offset in (0..size).step_by(BasePageSize::SIZE as usize) {
		let page_address = virtual_address + offset as u64;
		if let Some(physical_address) = paging::virtual_to_physical(page_address) {
			paging::unmap::<BasePageSize>(page_address, 1);
			let frame_range =
				PageRange::from_start_len(physical_address.as_usize(), BasePageSize::SIZE as usize)
					.unwrap();
			unsafe {
				FrameAlloc::deallocate(frame_range);
			}
		}
	}

	unsafe {
		PageAlloc::deallocate(page_range);
	}

	0
}

/// Configures the protections associated with a region of virtual memory
/// starting at `ptr` and going to `size`.
///
/// Returns 0 on success and an error code on failure.
#[hermit_macro::system(errno)]
#[unsafe(no_mangle)]
pub extern "C" fn sys_mprotect(ptr: *mut u8, size: usize, prot_flags: MemoryProtection) -> i32 {
	let count = size / BasePageSize::SIZE as usize;
	let flags = page_table_flags(prot_flags);

	let virtual_address = VirtAddr::from_ptr(ptr);

	debug!("Mprotect {virtual_address:X} ({size}) -> {prot_flags:?})");
	if let Some(physical_address) = paging::virtual_to_physical(virtual_address) {
		paging::map::<BasePageSize>(virtual_address, physical_address, count, flags);
		0
	} else {
		let frame_layout = PageLayout::from_size(size).unwrap();
		let frame_range = FrameAlloc::allocate(frame_layout).unwrap();
		let physical_address = PhysAddr::from(frame_range.start());
		paging::map::<BasePageSize>(virtual_address, physical_address, count, flags);
		0
	}
}

#[hermit_macro::system(errno)]
#[unsafe(no_mangle)]
pub extern "C" fn sys_mlock(_addr: *const c_void, _size: usize) -> i32 {
	// Hermit does not do any swapping yet.
	0
}

#[hermit_macro::system(errno)]
#[unsafe(no_mangle)]
pub extern "C" fn sys_munlock(_addr: *const c_void, _size: usize) -> i32 {
	// Hermit does not do any swapping yet.
	0
}

#[hermit_macro::system(errno)]
#[unsafe(no_mangle)]
pub extern "C" fn sys_mlockall(_flags: c_int) -> i32 {
	// Hermit does not do any swapping yet.
	0
}

#[hermit_macro::system(errno)]
#[unsafe(no_mangle)]
pub extern "C" fn sys_munlockall(_flags: c_int) -> i32 {
	// Hermit does not do any swapping yet.
	0
}
