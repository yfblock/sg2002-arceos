use crate::config::plat::{BOOT_STACK_SIZE, PHYS_VIRT_OFFSET};
use axplat::mem::{Aligned4K, pa};

#[unsafe(link_section = ".bss.stack")]
static mut BOOT_STACK: [u8; BOOT_STACK_SIZE] = [0; BOOT_STACK_SIZE];

#[unsafe(link_section = ".data")]
static mut BOOT_PT_SV39: Aligned4K<[u64; 512]> = Aligned4K::new([0; 512]);

#[allow(clippy::identity_op)] // (0x0 << 10) here makes sense because it's an address
unsafe fn init_boot_page_table() {
    const DEVICE_FLAGS: u64 = 0b10011 << 59;
    const KERNEL_FLAGS: u64 = 0b01111 << 59;
    unsafe {
        // 0x0000_0000..0x4000_0000, VRWX_GAD, 1G block
        // BOOT_PT_SV39[0] = (0x0 << 10) | 0xef | (0x9 << 60);
        BOOT_PT_SV39[0] = (0x0 << 10) | 0xef | DEVICE_FLAGS;
        BOOT_PT_SV39[1] = (0x40000 << 10) | 0xef | DEVICE_FLAGS;
        // 0x8000_0000..0xc000_0000, VRWX_GAD, 1G block
        BOOT_PT_SV39[2] = (0x80000 << 10) | 0xef | KERNEL_FLAGS;
        // BOOT_PT_SV39[2] = (0x80000 << 10) | 0xef | (0x3 << 60);
        // 0xffff_ffc0_0000_0000..0xffff_ffc0_4000_0000, VRWX_GAD, 1G block
        // BOOT_PT_SV39[0x100] = (0x0 << 10) | 0xef | (0x9 << 60);
        BOOT_PT_SV39[0x100] = (0x0 << 10) | 0xef | DEVICE_FLAGS;
        BOOT_PT_SV39[0x101] = (0x40000 << 10) | 0xef | DEVICE_FLAGS;
        // 0xffff_ffc0_8000_0000..0xffff_ffc0_c000_0000, VRWX_GAD, 1G block
        BOOT_PT_SV39[0x102] = (0x80000 << 10) | 0xef | KERNEL_FLAGS;
        // BOOT_PT_SV39[0x102] = (0x80000 << 10) | 0xef | (0x3 << 60);
    }
}

unsafe fn init_mmu() {
    unsafe {
        // 这里需要注意，在写入 satp 之前，需要保证 PAGE TABLE 已经写入到内存中，而且指令已经完成。
        // 其他的问题也是同理
        core::arch::asm!("fence.i");
        axcpu::asm::write_kernel_page_table(pa!(&raw const BOOT_PT_SV39 as usize));
        axcpu::asm::flush_tlb(None);
    }
}

#[unsafe(naked)]
#[unsafe(no_mangle)]
unsafe extern "C" fn __trap_handler() {
    unsafe{
        core::arch::naked_asm!("
            .align 4
            csrr  a0, mcause
            csrr  a1, mepc
            csrr  a2, mtval
            csrr  a3, 0x7c0
            csrr  a4, satp
            j {handler}
        ",
        handler = sym handler)
    }
}

unsafe extern "C" fn handler(mcause: usize, mepc: usize, mtval: usize, mxstatus: usize, satp: usize) {
    // println!("mcause: {:#x}  mepc: {mepc:#x} mtval: {mtval:#x} mxstatus {mxstatus:#x} satp: {satp:#x}", mcause);
    loop {}
}

unsafe extern "C" fn test(mhcr: usize) {
    println!("mhcr: {:#x}", mhcr);
}

/// The earliest entry point for the primary CPU.
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.boot")]
unsafe extern "C" fn _start() -> ! {
    // PC = 0x8020_0000
    // a0 = hartid
    // a1 = dtb
    core::arch::naked_asm!("
        mv      s0, a0                  // save hartid
        mv      s1, a1                  // save DTB pointer
        la      sp, {boot_stack}
        li      t0, {boot_stack_size}
        add     sp, sp, t0              // setup boot stack

        call    {init_boot_page_table}
        call    {init_mmu}              // setup boot page table and enabel MMU

        li      s2, {phys_virt_offset}  // fix up virtual high address
        add     sp, sp, s2

        mv      a0, s0
        mv      a1, s1
        la      a2, {entry}
        add     a2, a2, s2
        jalr    a2                      // call_main(cpu_id, dtb)
        j       .",
        phys_virt_offset = const PHYS_VIRT_OFFSET,
        boot_stack_size = const BOOT_STACK_SIZE,
        boot_stack = sym BOOT_STACK,
        init_boot_page_table = sym init_boot_page_table,
        init_mmu = sym init_mmu,
        entry = sym axplat::call_main,
    )
}

#[cfg(feature = "smp")]
#[unsafe(naked)]
#[unsafe(no_mangle)]
unsafe extern "C" fn _secondary_shim() -> ! {
    core::arch::naked_asm!("
        # 1. 设置 PMP，允许 S 态访问所有内存
        // li      t0, 0x3fffffffffffff
        li      t0, -1
        csrw    pmpaddr0, t0
        li      t0, 0x1f                 # A=TOR, R=1, W=1, X=1
        csrw    pmpcfg0, t0

        # 2. 设置 mstatus.MPP = S-mode (01)
        #    MPP 位于 mstatus[12:11]
        li      t0, (1 << 11)
        csrw    mstatus, t0

        csrr    t0, mcounteren
        ori     t0, t0, (1 << 1)         # MCOUNTEREN.TM=1
        csrw    mcounteren, t0

        # 3. 设置返回地址 (mret 后跳转到这个地址)
        la      t0, {addr}
        csrw    mepc, t0

        # 4. 关闭 S 态分页 (使用物理地址)
        csrw    satp, zero

        # 5. 可选：委托异常和中断给 S 态
        // li      t0, 0xffff
        // csrw    medeleg, t0
        // csrw    mideleg, t0

        la      t0, __trap_handler
        # Mode = 0 (Direct 模式)，所有异常和中断都跳到同一个地址
        csrw    mtvec, t0

        li      sp, 0x0C00F000
        li      a0, 0x17f
        csrw    0x7c1, a0       # WRITE MHCR(IE, DE, WA, WB, RS, BPE, BTB, WBR)
        csrr    a0, 0x7c1       # MHCR
        call    {test}

        # 6. 执行 mret 跳转到 S 态
        mret
    ",
    test = sym test,
    addr = sym _start_secondary);
}

/// The earliest entry point for secondary CPUs.
#[cfg(feature = "smp")]
#[unsafe(naked)]
#[unsafe(no_mangle)]
unsafe extern "C" fn _start_secondary() -> ! {
    // a0 = hartid
    // a1 = SP
    core::arch::naked_asm!("
        li      sp, 0x0C00F000

        call    {init_mmu}              // setup boot page table and enabel MMU

        li      s1, {phys_virt_offset}  // fix up virtual high address
        add     sp, sp, s1

        la      a1, {entry}
        add     a1, a1, s1

        mv      a0, zero
        jalr    a1                      // call_secondary_main(cpu_id)
        j       .",
        phys_virt_offset = const PHYS_VIRT_OFFSET,
        init_mmu = sym init_mmu,
        entry = sym axplat::call_secondary_main,
    )
}
