//! DMAC → UART0 THR 演示。
//!
//! T-Head C906 的 DRAM 区域被页表映射为 **write-back cacheable**，因此 CPU 写入的数据可能
//! 停留在 D-cache 中，DMA 直接读物理内存会拿到 stale 值。必须在 DMA 启动前做 **dcache flush**
//!（clean+invalidate），DMA 完成后再 flush 一次以确保 CPU 看到 DMA 写入的结果。

use axhal::mem::{phys_to_virt, virt_to_phys, PhysAddr, VirtAddr};
use core::cell::UnsafeCell;
use core::ptr::{addr_of, read_volatile, write_volatile};
use core::sync::atomic::{fence, Ordering};
use sg200x_bsp::dma::{
    regs::{
        CTL_DST_FIX, CTL_DST_MSIZE_SHIFT, CTL_DST_STA_EN, CTL_DST_WIDTH_SHIFT, CTL_IOC_BLT_EN,
        CTL_LLI_LAST, CTL_LLI_VALID, CTL_SRC_INC, CTL_SRC_MSIZE_SHIFT, CTL_SRC_STA_EN,
        CTL_SRC_WIDTH_SHIFT, INT_ALL_ERR,
    },
    prepare_memcpy_lli,
    DmaController, DmaDirection, DmaLli, DmaMsize, DmaSlaveConfig, DmaWidth,
};

/// T-Head C906/C910: clean + invalidate **all** D-cache lines, then fence.
///
/// 编码 `0x0030_000b` = T-Head 自定义 `dcache.ciall` 指令（CUSTOM-0 opcode）。
/// 对于 DMA 场景：启动前调用保证 CPU 写已到 DRAM，完成后调用保证 CPU 读到 DMA 写入值。
#[inline(always)]
unsafe fn dcache_flush_all() {
    unsafe {
        core::arch::asm!(
            ".long 0x0030000b", // dcache.ciall
            "fence rw, rw",
        );
    }
}

const DMAC_BASE_PHYS: usize = 0x0433_0000;
const UART0_BASE_PHYS: usize = 0x0414_0000;
const DMA_CH: usize = 0;
const CLKGEN_BASE_PHYS: usize = 0x0300_2000;
const REG_CLK_EN_1: usize = 0x004;
const REG_CLK_EN_2: usize = 0x008;
const BIT_CLK_SDMA_AXI: u32 = 1 << 1;
const BIT_CLK_AXI4: u32 = 1 << 1;

/// 全部给 DMAC 用 **静态存储**，避免栈 VA/缓存与 `virt_to_phys` 组合踩坑
#[repr(C, align(64))]
struct DmaBuffers {
    /// UART 文本 + LLI 链
    lli_uart: [DmaLli; 40],
    src_uart: [u8; 160],
    /// RAM 自检
    lli_ram: [DmaLli; 4],
    tst_src: [u8; 64],
    tst_dst: [u8; 64],
}

struct DmaBuffersStatic(UnsafeCell<DmaBuffers>);
/// SAFETY：仅在启动早期单线程 `run_demo` 路径访问内部 `UnsafeCell`。
unsafe impl Sync for DmaBuffersStatic {}

static BUF: DmaBuffersStatic = DmaBuffersStatic(UnsafeCell::new(DmaBuffers {
    lli_uart: [DmaLli::new(); 40],
    src_uart: [0u8; 160],
    lli_ram: [DmaLli::new(); 4],
    tst_src: [0u8; 64],
    tst_dst: [0u8; 64],
}));

/// SAFETY：仅在启动早期单线程 `run_demo` 路径使用。
unsafe fn buf_mut() -> &'static mut DmaBuffers {
    unsafe { &mut *BUF.0.get() }
}

fn dmac_vaddr() -> usize {
    phys_to_virt(PhysAddr::from_usize(DMAC_BASE_PHYS)).as_usize()
}

fn va_to_phys<T>(p: *const T) -> u64 {
    virt_to_phys(VirtAddr::from(p as usize)).as_usize() as u64
}

fn enable_sdma_clock_gates() {
    let base = phys_to_virt(PhysAddr::from_usize(CLKGEN_BASE_PHYS)).as_usize();
    unsafe {
        let en2 = (base + REG_CLK_EN_2) as *mut u32;
        write_volatile(en2, read_volatile(en2) | BIT_CLK_AXI4);
        let en1 = (base + REG_CLK_EN_1) as *mut u32;
        write_volatile(en1, read_volatile(en1) | BIT_CLK_SDMA_AXI);
    }
}

fn wait_channel(dma: &DmaController, spins_max: usize) -> (usize, bool) {
    let mut spins = 0usize;
    while dma.is_channel_enabled(DMA_CH) {
        spins += 1;
        if spins >= spins_max {
            return (spins, false);
        }
        core::hint::spin_loop();
    }
    (spins, true)
}

/// RAM→RAM，验证 DMAC + master0 + 时钟 + 缓存一致性。成功则 `tst_dst` 全为 `0x5A`。
unsafe fn run_ram_self_test() -> bool {
    let b = buf_mut();
    b.tst_src.fill(0x5a);
    b.tst_dst.fill(0);
    fence(Ordering::SeqCst);

    let s = va_to_phys(b.tst_src.as_ptr());
    let d = va_to_phys(b.tst_dst.as_ptr());
    let lli_phys_base = va_to_phys(addr_of!(b.lli_ram[0]));
    // data_width = 4 字节，与 DTS `data-width = <4 4>` 一致（SG2002 最大 32 位通路）
    let n = prepare_memcpy_lli(&mut b.lli_ram, lli_phys_base, s, d, 64, 1024, 4);
    if n == 0 {
        println!("[dma_uart_tx] prepare_memcpy_lli returned 0");
        return false;
    }
    let l0 = lli_phys_base;

    let mut dma = DmaController::new(dmac_vaddr());
    let dma_id = dma.read_reg(0x00);
    println!(
        "[dma_uart_tx] DMAC id={:#x} src_pa={:#x} dst_pa={:#x} lli_pa={:#x} lli_cnt={}",
        dma_id, s, d, l0, n
    );

    dma.init();
    if dma.alloc_channel(DMA_CH).is_err() {
        println!("[dma_uart_tx] alloc_channel failed in self-test");
        return false;
    }
    dma.channels[DMA_CH].dst_id = 0;
    dma.channels[DMA_CH].src_id = 0;
    dma.channels[DMA_CH].m_master = 0;
    dma.channels[DMA_CH].p_master = 0;

    let cfg = DmaSlaveConfig {
        direction: DmaDirection::MemToMem,
        src_addr: 0,
        dst_addr: 0,
        src_addr_width: DmaWidth::Width32,
        dst_addr_width: DmaWidth::Width32,
        src_maxburst: 4,
        dst_maxburst: 4,
        device_fc: false,
    };
    if dma.configure_channel(DMA_CH, &cfg).is_err() {
        dma.free_channel(DMA_CH);
        println!("[dma_uart_tx] configure failed in self-test");
        return false;
    }

    // *** 关键：D-cache clean+invalidate，确保 LLI 与 src 数据刷到物理内存 ***
    dcache_flush_all();

    if dma.start_transfer(DMA_CH, l0).is_err() {
        dma.free_channel(DMA_CH);
        println!("[dma_uart_tx] start_transfer failed in self-test");
        return false;
    }

    let (spins, ok) = wait_channel(&dma, 50_000_000);
    let ist = dma.channels[DMA_CH].read_int_status();
    dma.free_channel(DMA_CH);

    // *** 关键：DMA 写完后 flush，CPU 才能看到 DMA 写入的数据 ***
    dcache_flush_all();

    if !ok {
        println!(
            "[dma_uart_tx] self-test timeout spins={} ch_int={:#x}",
            spins, ist
        );
        return false;
    }
    if (ist & INT_ALL_ERR) != 0 {
        println!("[dma_uart_tx] self-test DMA error ch_int={:#x}", ist);
        return false;
    }

    let pass = b.tst_dst.iter().all(|&x| x == 0x5a);
    if !pass {
        let bad = b.tst_dst.iter().position(|&x| x != 0x5a).unwrap_or(64);
        println!(
            "[dma_uart_tx] self-test mismatch at byte {}: dst[0..8]=[{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x}]",
            bad,
            b.tst_dst[0], b.tst_dst[1], b.tst_dst[2], b.tst_dst[3],
            b.tst_dst[4], b.tst_dst[5], b.tst_dst[6], b.tst_dst[7],
        );
    } else {
        println!("[dma_uart_tx] RAM self-test passed");
    }
    pass
}

/// DW APB UART LSR 寄存器（reg-shift=2，即 offset 5×4 = 0x14）
const UART_LSR_OFFSET: usize = 0x14;
/// LSR bit 6: Transmitter Empty（FIFO + 移位寄存器均空）
const UART_LSR_TEMT: u32 = 1 << 6;
/// UART FIFO 深度（DW APB UART 典型值 16 字节）
const UART_FIFO_DEPTH: usize = 16;

/// 等待 UART TX 完全空闲（FIFO + 移位寄存器）
fn wait_uart_tx_empty() {
    let lsr = phys_to_virt(PhysAddr::from_usize(UART0_BASE_PHYS + UART_LSR_OFFSET)).as_usize();
    unsafe {
        while read_volatile(lsr as *const u32) & UART_LSR_TEMT == 0 {
            core::hint::spin_loop();
        }
    }
}

/// 构建单条 LLI：8 位宽、源递增、目的固定为 UART THR。
fn build_uart_ctl() -> u64 {
    let mut ctl: u64 = 0;
    ctl |= (DmaMsize::Msize1 as u64) << CTL_DST_MSIZE_SHIFT;
    ctl |= (DmaMsize::Msize1 as u64) << CTL_SRC_MSIZE_SHIFT;
    ctl |= (DmaWidth::Width8 as u64) << CTL_DST_WIDTH_SHIFT;
    ctl |= (DmaWidth::Width8 as u64) << CTL_SRC_WIDTH_SHIFT;
    ctl |= CTL_SRC_INC | CTL_DST_FIX | CTL_SRC_STA_EN | CTL_DST_STA_EN;
    ctl |= CTL_LLI_VALID | CTL_LLI_LAST | CTL_IOC_BLT_EN;
    ctl
}

/// 分块 DMA 发送：每块 ≤ FIFO 深度，发完一块后等 UART TX Empty 再发下一块。
pub fn run_demo() {
    const PAYLOAD: &[u8] = b"\r\n[dma_uart_tx] hello from DMA -> UART0 (8-bit THR)\r\n";

    enable_sdma_clock_gates();

    let ram_ok = unsafe { run_ram_self_test() };
    if !ram_ok {
        println!("[dma_uart_tx] RAM self-test failed (check clk / AXI master)");
        println!("[dma_uart_tx] run_demo finished");
        return;
    }

    let ctl = build_uart_ctl();

    let mut dma = DmaController::new(dmac_vaddr());
    dma.init();
    if dma.alloc_channel(DMA_CH).is_err() {
        println!("[dma_uart_tx] alloc_channel failed");
        println!("[dma_uart_tx] run_demo finished");
        return;
    }
    dma.channels[DMA_CH].dst_id = 0;
    dma.channels[DMA_CH].src_id = 0;
    dma.channels[DMA_CH].m_master = 0;
    dma.channels[DMA_CH].p_master = 0;

    let cfg = DmaSlaveConfig {
        direction: DmaDirection::MemToMem,
        src_addr: 0,
        dst_addr: 0,
        src_addr_width: DmaWidth::Width8,
        dst_addr_width: DmaWidth::Width8,
        src_maxburst: 1,
        dst_maxburst: 1,
        device_fc: false,
    };
    if dma.configure_channel(DMA_CH, &cfg).is_err() {
        dma.free_channel(DMA_CH);
        println!("[dma_uart_tx] configure failed");
        println!("[dma_uart_tx] run_demo finished");
        return;
    }

    let mut offset = 0usize;
    let mut err = false;

    while offset < PAYLOAD.len() {
        let chunk_len = (PAYLOAD.len() - offset).min(UART_FIFO_DEPTH);

        // 等 UART FIFO 完全排空再灌下一块
        wait_uart_tx_empty();

        // 填充 src_uart 并更新 LLI
        unsafe {
            let b = buf_mut();
            b.src_uart[..chunk_len].copy_from_slice(&PAYLOAD[offset..offset + chunk_len]);

            let lli = &mut b.lli_uart[0];
            lli.sar = va_to_phys(b.src_uart.as_ptr());
            lli.dar = UART0_BASE_PHYS as u64;
            lli.block_ts = (chunk_len as u64) - 1;
            lli.llp = 0;
            lli.ctl = ctl;

            dcache_flush_all();
        }

        let lli_phys = unsafe { va_to_phys(addr_of!((*BUF.0.get()).lli_uart[0])) };

        if dma.start_transfer(DMA_CH, lli_phys).is_err() {
            println!("[dma_uart_tx] start_transfer failed at offset {}", offset);
            err = true;
            break;
        }

        let (_spins, done) = wait_channel(&dma, 100_000_000);
        if !done {
            let ist = dma.channels[DMA_CH].read_int_status();
            println!(
                "[dma_uart_tx] chunk timeout offset={} ch_int={:#x}",
                offset, ist
            );
            err = true;
            break;
        }

        offset += chunk_len;
    }

    dma.free_channel(DMA_CH);

    if !err {
        // 等最后一块字节全部从 UART 移位寄存器发出
        wait_uart_tx_empty();
        println!("[dma_uart_tx] UART DMA completed, {} bytes sent", PAYLOAD.len());
    }

    println!("[dma_uart_tx] run_demo finished");
}
