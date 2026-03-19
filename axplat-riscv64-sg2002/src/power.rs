use axplat::power::PowerIf;

struct PowerImpl;

#[impl_plat_interface]
impl PowerIf for PowerImpl {
    /// Bootstraps the given CPU core with the given initial stack (in physical
    /// address).
    ///
    /// Where `cpu_id` is the logical CPU ID (0, 1, ..., N-1, N is the number of
    /// CPU cores on the platform).
    #[cfg(feature = "smp")]
    fn cpu_boot(cpu_id: usize, stack_top_paddr: usize) {
        log::debug!("It should boot secondary cpu {}/{}", cpu_id, Self::cpu_num());
        use axplat::mem::{pa, phys_to_virt, va, virt_to_phys};
        unsafe extern "C" {
            fn _secondary_shim();
        }
        unsafe {
            let rst = phys_to_virt(pa!(0x3003024));
            rst.as_mut_ptr_of::<u32>().write_volatile(rst.as_ptr_of::<u32>().read_volatile() & !(1 << 6));
            let entry = virt_to_phys(va!(_secondary_shim as *const () as usize));
            let ass_en = phys_to_virt(pa!(0x20B0004));
            ass_en.as_mut_ptr_of::<u32>().write_volatile(ass_en.as_ptr_of::<u32>().read_volatile() | (1 << 13));
            phys_to_virt(pa!(0x20B0020)).as_mut_ptr_of::<u32>().write_volatile(entry.as_usize() as u32);
            phys_to_virt(pa!(0x20B0024)).as_mut_ptr_of::<u32>().write_volatile((entry.as_usize() >> 32) as u32);
            riscv::asm::fence_i();
            rst.as_mut_ptr_of::<u32>().write_volatile(rst.as_ptr_of::<u32>().read_volatile() | (1 << 6));
            riscv::asm::fence_i();
        }
    }

    /// Shutdown the whole system.
    fn system_off() -> ! {
        info!("Shutting down...");
        sbi_rt::system_reset(sbi_rt::Shutdown, sbi_rt::NoReason);
        warn!("It should shutdown!");
        loop {
            axcpu::asm::halt();
        }
    }

    /// Get the number of CPU cores available on this platform.
    fn cpu_num() -> usize {
        crate::config::plat::MAX_CPU_NUM
    }
}
