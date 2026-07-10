use core::hint::spin_loop;

use hermit_sync::{OnceCell, without_interrupts};
use time::OffsetDateTime;
use x86_64::instructions::port::Port;

use crate::arch::kernel::processor;

const CMOS_COMMAND: Port<u8> = Port::new(0x70);
const CMOS_DATA: Port<u8> = Port::new(0x71);

const CMOS_DISABLE_NMI: u8 = 1 << 7;

const CMOS_SECOND_REGISTER: u8 = 0x00;
const CMOS_MINUTE_REGISTER: u8 = 0x02;
const CMOS_HOUR_REGISTER: u8 = 0x04;
const CMOS_DAY_REGISTER: u8 = 0x07;
const CMOS_MONTH_REGISTER: u8 = 0x08;
const CMOS_YEAR_REGISTER: u8 = 0x09;
const CMOS_STATUS_REGISTER_A: u8 = 0x0a;
const CMOS_STATUS_REGISTER_B: u8 = 0x0b;

const CMOS_UPDATE_IN_PROGRESS_FLAG: u8 = 1 << 7;
const CMOS_24_HOUR_FORMAT_FLAG: u8 = 1 << 1;
const CMOS_BINARY_FORMAT_FLAG: u8 = 1 << 2;
const CMOS_12_HOUR_PM_FLAG: u8 = 0x80;

struct Rtc {
	cmos_format: u8,
}

impl Rtc {
	fn new() -> Self {
		Self {
			cmos_format: Self::read_cmos_register(CMOS_STATUS_REGISTER_B),
		}
	}

	const fn is_24_hour_format(&self) -> bool {
		self.cmos_format & CMOS_24_HOUR_FORMAT_FLAG > 0
	}

	const fn is_binary_format(&self) -> bool {
		self.cmos_format & CMOS_BINARY_FORMAT_FLAG > 0
	}

	const fn time_is_pm(hour: u8) -> bool {
		hour & CMOS_12_HOUR_PM_FLAG > 0
	}

	/// Returns the binary value for a given value in BCD (Binary-Coded Decimal).
	const fn convert_bcd_value(value: u8) -> u8 {
		((value / 16) * 10) + (value & 0xf)
	}

	/// Returns the number of microseconds since the epoch from a given date.
	/// Inspired by Linux Kernel's mktime64(), see kernel/time/time.c.
	fn microseconds_from_date(
		year: u16,
		month: u8,
		day: u8,
		hour: u8,
		minute: u8,
		second: u8,
	) -> u64 {
		let (m, y) = if month > 2 {
			(u64::from(month - 2), u64::from(year))
		} else {
			(u64::from(month + 12 - 2), u64::from(year - 1))
		};

		let days_since_epoch =
			(y / 4 - y / 100 + y / 400 + 367 * m / 12 + u64::from(day)) + y * 365 - 719_499;
		let hours_since_epoch = days_since_epoch * 24 + u64::from(hour);
		let minutes_since_epoch = hours_since_epoch * 60 + u64::from(minute);
		let seconds_since_epoch = minutes_since_epoch * 60 + u64::from(second);

		seconds_since_epoch * 1_000_000u64
	}

	fn read_cmos_register(register: u8) -> u8 {
		let mut cmos_command = CMOS_COMMAND;
		let mut cmos_data = CMOS_DATA;
		unsafe {
			cmos_command.write(CMOS_DISABLE_NMI | register);
			cmos_data.read()
		}
	}

	fn read_datetime_register(&self, register: u8) -> u8 {
		let value = Self::read_cmos_register(register);

		// Every date/time register may either be in binary or in BCD format.
		// Convert BCD values if necessary.
		if self.is_binary_format() {
			value
		} else {
			Self::convert_bcd_value(value)
		}
	}

	fn read_all_values(&self) -> u64 {
		// Reading year, month, and day is straightforward.
		let year = u16::from(self.read_datetime_register(CMOS_YEAR_REGISTER)) + 2000;
		let month = self.read_datetime_register(CMOS_MONTH_REGISTER);
		let day = self.read_datetime_register(CMOS_DAY_REGISTER);

		// The hour register is a bitch.
		// On top of being in either binary or BCD format, it may also be in 12-hour
		// or 24-hour format.
		let mut hour = Self::read_cmos_register(CMOS_HOUR_REGISTER);
		let mut is_pm = false;

		// Check and mask off a potential PM flag if the hour is given in 12-hour format.
		if !self.is_24_hour_format() {
			is_pm = Self::time_is_pm(hour);
			hour &= !CMOS_12_HOUR_PM_FLAG;
		}

		// Now convert a BCD number to binary if necessary (after potentially masking off the PM flag above).
		if !self.is_binary_format() {
			hour = Self::convert_bcd_value(hour);
		}

		// If the hour is given in 12-hour format, do the necessary calculations to convert it into 24 hours.
		if !self.is_24_hour_format() {
			if hour == 12 {
				// 12:00 AM is 00:00 and 12:00 PM is 12:00 (see is_pm below) in 24-hour format.
				hour = 0;
			}

			if is_pm {
				// {01:00 PM, 02:00 PM, ...} is {13:00, 14:00, ...} in 24-hour format.
				hour += 12;
			}
		}

		// The minute and second registers are straightforward again.
		let minute = self.read_datetime_register(CMOS_MINUTE_REGISTER);
		let second = self.read_datetime_register(CMOS_SECOND_REGISTER);

		// Convert it all to microseconds and return the result.
		Self::microseconds_from_date(year, month, day, hour, minute, second)
	}

	pub fn get_microseconds_since_epoch(&self) -> u64 {
		loop {
			// If a clock update is currently in progress, wait until it is finished.
			while Self::read_cmos_register(CMOS_STATUS_REGISTER_A) & CMOS_UPDATE_IN_PROGRESS_FLAG
				> 0
			{
				spin_loop();
			}

			// Get the current time in microseconds since the epoch.
			let microseconds_since_epoch_1 = self.read_all_values();

			// If the clock is already updating the time again, the read values may be inconsistent
			// and we have to repeat this process.
			if Self::read_cmos_register(CMOS_STATUS_REGISTER_A) & CMOS_UPDATE_IN_PROGRESS_FLAG > 0 {
				continue;
			}

			// Get the current time again and verify that it's the same we last read.
			let microseconds_since_epoch_2 = self.read_all_values();
			if microseconds_since_epoch_1 == microseconds_since_epoch_2 {
				// Both times are identical, so we have read consistent values and can exit the loop.
				return microseconds_since_epoch_1;
			}
		}
	}
}

static BOOT_TIME: OnceCell<u64> = OnceCell::new();

// --- KVM paravirtualized clock (kvmclock) --------------------------------
//
// Hypervisors that strip the CMOS RTC (notably Firecracker) expose time
// through kvmclock:
//
// * `MSR_KVM_WALL_CLOCK_NEW` fills a guest-memory `pvclock_wall_clock` struct
//   with the wall-clock time at kvmclock's zero point, i.e. the boot instant.
//   KVM recomputes it on every MSR write (`host wall clock - current
//   kvmclock`), so re-reading it also picks up host-side clock changes --
//   most importantly the time a VM spent paused in a snapshot.
// * `MSR_KVM_SYSTEM_TIME_NEW` registers a `pvclock_vcpu_time_info` page that
//   the host keeps updated with an exact TSC-to-nanoseconds scaling
//   (`tsc_to_system_mul`/`tsc_shift`) and base. This gives drift-free
//   monotonic time (unlike our whole-MHz `CPU_FREQUENCY` calibration) that
//   stays continuous across snapshot/restore.
//
// Wall time is `epoch + system_time`, with the epoch re-derived at most once
// per second so that after a snapshot/restore the wall clock converges to the
// host's real time within a bounded interval instead of staying behind by the
// pause duration.
#[cfg(feature = "kvmclock")]
mod kvmclock {
	use core::sync::atomic::{
		AtomicBool, AtomicI8, AtomicU8, AtomicU32, AtomicU64, Ordering, fence,
	};

	use memory_addresses::VirtAddr;
	use x86_64::registers::model_specific::Msr;

	use crate::arch::kernel::processor;
	use crate::arch::x86_64::mm::paging::virtual_to_physical;

	const MSR_KVM_WALL_CLOCK_NEW: u32 = 0x4b56_4d00;
	const MSR_KVM_SYSTEM_TIME_NEW: u32 = 0x4b56_4d01;
	/// Bit 0 of the `MSR_KVM_SYSTEM_TIME_NEW` payload enables the clock.
	const MSR_KVM_SYSTEM_TIME_ENABLE: u64 = 1;

	// KVM paravirt CPUID: leaf 0x40000000 carries the "KVMKVMKVM" signature;
	// leaf 0x40000001 EAX bit 3 is KVM_FEATURE_CLOCKSOURCE2, which gates the
	// `_NEW` clock MSRs. We check these before touching the MSRs so a `wrmsr`
	// to an unimplemented MSR can't #GP on a non-KVM hypervisor.
	const KVM_CPUID_SIGNATURE: u32 = 0x4000_0000;
	const KVM_CPUID_FEATURES: u32 = 0x4000_0001;
	const KVM_FEATURE_CLOCKSOURCE2: u32 = 1 << 3;

	/// Set by the host while the KVM masterclock keeps every vCPU's pvclock
	/// parameters consistent. Only then is it valid to read the boot
	/// processor's time-info page from any CPU; KVM rewrites the page with the
	/// flag cleared if that ever stops holding, so it is checked on every read.
	const PVCLOCK_TSC_STABLE_BIT: u8 = 1 << 0;

	/// Re-derive the wall-clock epoch at most this often. This bounds how long
	/// wall time stays stale after a host-side clock change -- most importantly
	/// the pause duration of a snapshot/restore, which is invisible to the
	/// (deliberately continuous) monotonic clock. One `wrmsr` per second of
	/// wall-clock reads is negligible.
	const EPOCH_REFRESH_INTERVAL_MICROS: u64 = 1_000_000;

	/// `struct pvclock_wall_clock` (KVM ABI): a seqlock-guarded epoch. The host
	/// bumps `version` to an odd value while updating, then to an even value
	/// when the `sec`/`nsec` fields are stable.
	///
	/// `align(16)` keeps the 12-byte struct inside a single page, as the ABI
	/// requires.
	#[repr(C, align(16))]
	struct PvclockWallClock {
		version: AtomicU32,
		sec: AtomicU32,
		nsec: AtomicU32,
	}

	/// `struct pvclock_vcpu_time_info` (KVM ABI): seqlock-guarded TSC scaling
	/// parameters. `system_time` is the guest time in nanoseconds at the
	/// instant the TSC read `tsc_timestamp`; scaling the TSC delta by
	/// `tsc_to_system_mul`/`tsc_shift` extrapolates to "now".
	///
	/// `align(64)` keeps the 32-byte struct inside a single page.
	#[repr(C, align(64))]
	struct PvclockVcpuTimeInfo {
		version: AtomicU32,
		pad0: u32,
		tsc_timestamp: AtomicU64,
		system_time: AtomicU64,
		tsc_to_system_mul: AtomicU32,
		tsc_shift: AtomicI8,
		flags: AtomicU8,
		pad: [u8; 2],
	}

	// Lives in identity-or-known-mapped kernel memory; we translate its virtual
	// address to physical for the MSR. Written by the host, read here -- a
	// single global structure by ABI (the wall-clock MSR is VM-global).
	static WALL_CLOCK: PvclockWallClock = PvclockWallClock {
		version: AtomicU32::new(0),
		sec: AtomicU32::new(0),
		nsec: AtomicU32::new(0),
	};

	// Registered with the host on the boot processor only; reads from other
	// CPUs are gated on `PVCLOCK_TSC_STABLE_BIT` (see above).
	static VCPU_TIME: PvclockVcpuTimeInfo = PvclockVcpuTimeInfo {
		version: AtomicU32::new(0),
		pad0: 0,
		tsc_timestamp: AtomicU64::new(0),
		system_time: AtomicU64::new(0),
		tsc_to_system_mul: AtomicU32::new(0),
		tsc_shift: AtomicI8::new(0),
		flags: AtomicU8::new(0),
		pad: [0; 2],
	};

	/// Wall-clock microseconds at kvmclock's zero point (the boot instant);
	/// 0 = not yet derived.
	static EPOCH_MICROS: AtomicU64 = AtomicU64::new(0);
	/// kvmclock reading (microseconds) when the epoch was last derived.
	static EPOCH_READ_AT: AtomicU64 = AtomicU64::new(0);
	static EPOCH_REFRESHING: AtomicBool = AtomicBool::new(false);

	/// Whether the hypervisor is KVM and advertises the paravirt clock.
	fn available() -> bool {
		let sig = core::arch::x86_64::__cpuid(KVM_CPUID_SIGNATURE);
		// "KVMKVMKVM" in EBX/ECX/EDX as little-endian words:
		// EBX="KVMK", ECX="VMKV", EDX="M\0\0\0".
		let is_kvm = sig.ebx == 0x4b4d_564b && sig.ecx == 0x564b_4d56 && sig.edx == 0x0000_004d;
		if !is_kvm {
			return false;
		}
		let feat = core::arch::x86_64::__cpuid(KVM_CPUID_FEATURES);
		feat.eax & KVM_FEATURE_CLOCKSOURCE2 != 0
	}

	fn phys_of<T>(p: *const T) -> Option<u64> {
		let virt = VirtAddr::new(p.expose_provenance() as u64);
		virtual_to_physical(virt).map(|pa| pa.as_u64())
	}

	/// (Re-)derive the wall-clock epoch: the wall time (microseconds since the
	/// UNIX epoch) at kvmclock's zero point. KVM computes it freshly on every
	/// MSR write, so after a snapshot/restore or a host NTP step the returned
	/// epoch shifts to compensate.
	fn read_wall_clock_epoch() -> Option<u64> {
		let phys = phys_of(&raw const WALL_CLOCK)?;
		// SAFETY: CLOCKSOURCE2 is advertised, so this MSR is implemented; `phys`
		// points at our static, which the host may write.
		unsafe { Msr::new(MSR_KVM_WALL_CLOCK_NEW).write(phys) };

		for _ in 0..1000 {
			let v1 = WALL_CLOCK.version.load(Ordering::Acquire);
			if v1 & 1 != 0 {
				core::hint::spin_loop();
				continue;
			}
			// The acquire load of `version` orders the data loads after it.
			let sec = WALL_CLOCK.sec.load(Ordering::Relaxed);
			let nsec = WALL_CLOCK.nsec.load(Ordering::Relaxed);
			fence(Ordering::Acquire);
			let v2 = WALL_CLOCK.version.load(Ordering::Acquire);
			if v1 == v2 {
				if sec == 0 && nsec == 0 {
					return None;
				}
				// Microsecond resolution; sub-microsecond precision is dropped.
				return Some(u64::from(sec) * 1_000_000 + u64::from(nsec) / 1000);
			}
		}
		None
	}

	/// Guest monotonic time from the pvclock page, in microseconds, or `None`
	/// if the page is unregistered, not yet filled by the host, or the
	/// masterclock guarantee is off (in which case the caller falls back to the
	/// tick-based clock).
	fn system_time_micros() -> Option<u64> {
		for _ in 0..1000 {
			let v1 = VCPU_TIME.version.load(Ordering::Acquire);
			if v1 == 0 {
				// The host never filled the page: the MSR was not (successfully)
				// written, or the host ignored it.
				return None;
			}
			if v1 & 1 != 0 {
				core::hint::spin_loop();
				continue;
			}
			// The acquire load of `version` orders the data loads after it.
			let tsc_timestamp = VCPU_TIME.tsc_timestamp.load(Ordering::Relaxed);
			let system_time = VCPU_TIME.system_time.load(Ordering::Relaxed);
			let mul = VCPU_TIME.tsc_to_system_mul.load(Ordering::Relaxed);
			let shift = VCPU_TIME.tsc_shift.load(Ordering::Relaxed);
			let flags = VCPU_TIME.flags.load(Ordering::Relaxed);
			// The TSC must be sampled while the parameters are known-stable,
			// i.e. between the two version reads. `get_timestamp` is
			// lfence-serialized.
			let tsc = processor::get_timestamp();
			fence(Ordering::Acquire);
			let v2 = VCPU_TIME.version.load(Ordering::Acquire);
			if v1 != v2 {
				continue;
			}
			if flags & PVCLOCK_TSC_STABLE_BIT == 0 {
				return None;
			}
			// nanos = system_time + (((tsc - tsc_timestamp) << shift) * mul) >> 32
			// (u128 intermediates: a years-scale TSC delta times a ~2^30 `mul`
			// overflows u64).
			let delta = u128::from(tsc.saturating_sub(tsc_timestamp));
			let delta = if shift >= 0 {
				delta << u32::try_from(shift).unwrap()
			} else {
				delta >> u32::try_from(-i32::from(shift)).unwrap()
			};
			let nanos = system_time.wrapping_add(((delta * u128::from(mul)) >> 32) as u64);
			return Some(nanos / 1000);
		}
		None
	}

	/// Refresh the epoch if it is older than `EPOCH_REFRESH_INTERVAL_MICROS`.
	/// Serialized by a try-lock; a loser simply uses the current (still
	/// bounded-stale) epoch for this read.
	fn maybe_refresh_epoch(mono_micros: u64) {
		if mono_micros.saturating_sub(EPOCH_READ_AT.load(Ordering::Relaxed))
			< EPOCH_REFRESH_INTERVAL_MICROS
		{
			return;
		}
		if EPOCH_REFRESHING
			.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
			.is_err()
		{
			return;
		}
		if let Some(epoch) = read_wall_clock_epoch() {
			// `fetch_max`: never step the derived wall clock backwards. Forward
			// jumps (snapshot restore) pass through; small backward host
			// adjustments are absorbed. This keeps `now_micros` non-decreasing
			// -- callers compute `now - start` durations that must not
			// underflow.
			EPOCH_MICROS.fetch_max(epoch, Ordering::AcqRel);
		}
		EPOCH_READ_AT.store(mono_micros, Ordering::Relaxed);
		EPOCH_REFRESHING.store(false, Ordering::Release);
	}

	/// Current wall time in microseconds since the UNIX epoch, or `None` if
	/// kvmclock is not (fully) available.
	pub fn now_micros() -> Option<u64> {
		let mono = system_time_micros()?;
		if EPOCH_MICROS.load(Ordering::Relaxed) == 0 {
			return None;
		}
		maybe_refresh_epoch(mono);
		Some(EPOCH_MICROS.load(Ordering::Relaxed) + mono)
	}

	/// One-time initialization on the boot processor (pre-SMP): registers the
	/// system-time page and derives the boot epoch. Returns the boot instant in
	/// microseconds since the UNIX epoch, or `None` if kvmclock is unavailable.
	pub fn init() -> Option<u64> {
		if !available() {
			return None;
		}
		if let Some(phys) = phys_of(&raw const VCPU_TIME) {
			// SAFETY: CLOCKSOURCE2 is advertised, so this MSR is implemented;
			// `phys` points at our static, which the host writes from now on.
			unsafe { Msr::new(MSR_KVM_SYSTEM_TIME_NEW).write(phys | MSR_KVM_SYSTEM_TIME_ENABLE) };
		}
		let epoch = read_wall_clock_epoch()?;
		EPOCH_MICROS.store(epoch, Ordering::Relaxed);
		EPOCH_READ_AT.store(system_time_micros().unwrap_or(0), Ordering::Relaxed);
		Some(epoch)
	}
}

fn boot_time() -> OffsetDateTime {
	#[cfg(feature = "uhyve")]
	use crate::env::UhyveStartInfo;

	#[cfg(feature = "uhyve")]
	if let Some(boot_time) = crate::env::start_info().uhyve_boot_time() {
		return boot_time;
	}

	// Firecracker and other RTC-less hypervisors: kvmclock carries the real
	// wall clock. The epoch it reports is the wall time at kvmclock/TSC zero,
	// i.e. the boot instant itself -- no ticks subtraction. A host-provided
	// out-of-range value falls through to the RTC path instead of panicking.
	#[cfg(feature = "kvmclock")]
	if let Some(epoch_micros) = kvmclock::init()
		&& let Ok(boot_time) =
			OffsetDateTime::from_unix_timestamp_nanos(i128::from(epoch_micros) * 1000)
	{
		return boot_time;
	}

	// Get the current time in microseconds since the epoch (1970-01-01) from the x86 RTC.
	// Subtract the timer ticks to get the actual time when Hermit was booted.
	let current_time = without_interrupts(|| Rtc::new().get_microseconds_since_epoch());
	let boot_micros = current_time - processor::get_timer_ticks();
	let boot_nanos = i128::from(boot_micros) * 1000;
	OffsetDateTime::from_unix_timestamp_nanos(boot_nanos).unwrap()
}

pub fn init() {
	let boot_time = boot_time();
	info!("Hermit booted on {boot_time}");

	let micros = u64::try_from(boot_time.unix_timestamp_nanos() / 1000).unwrap();
	BOOT_TIME.set(micros).unwrap();
}

/// Returns the current time in microseconds since UNIX epoch.
pub fn now_micros() -> u64 {
	// Prefer kvmclock: host-provided TSC scaling (no whole-MHz calibration
	// drift) and a periodically re-derived wall epoch (converges to real time
	// after a snapshot/restore instead of staying behind by the pause
	// duration).
	#[cfg(feature = "kvmclock")]
	if let Some(now) = kvmclock::now_micros() {
		return now;
	}
	*BOOT_TIME.get().unwrap() + processor::get_timer_ticks()
}
