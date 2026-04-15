#![allow(dead_code)]
//! 摄像头帧与 DMAC 结合：UART 协议收齐 `frame` 后，用 **MemToMem** 拷入对齐静态缓冲并校验。
//!
//! 与 `dma_uart_tx.rs` 相同地依赖 `dcache.ciall`（见 T-Head 手册）与 `virt_to_phys`，
//! 行为与 Linux DTS 中 `dmac@4330000` 的 `data-width = <4 4>` 一致（32 位通路）。

use axhal::mem::{phys_to_virt, virt_to_phys, PhysAddr, VirtAddr};
use core::cell::UnsafeCell;
use core::ptr::addr_of;
use core::sync::atomic::{fence, Ordering};
use sg200x_bsp::dma::{
    prepare_memcpy_lli,
    regs::INT_ALL_ERR,
    DmaController, DmaDirection, DmaLli, DmaSlaveConfig, DmaWidth,
};

const DMAC_BASE_PHYS: usize = 0x0433_0000;
/// 与 `dma_uart_tx` 错开通道，避免将来并行扩展时冲突。
const DMA_CH: usize = 1;

/// 单帧拷贝上限（当前协议约 36KiB JPEG；留余量）。
const FRAME_CAP: usize = 384 * 1024;
const MAX_LLI: usize = 512;

#[inline(always)]
unsafe fn dcache_flush_all() {
    unsafe {
        core::arch::asm!(
            ".long 0x0030000b", // dcache.ciall
            "fence rw, rw",
        );
    }
}

fn dmac_vaddr() -> usize {
    phys_to_virt(PhysAddr::from_usize(DMAC_BASE_PHYS)).as_usize()
}

fn va_to_phys<T>(p: *const T) -> u64 {
    virt_to_phys(VirtAddr::from(p as usize)).as_usize() as u64
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

#[repr(C, align(64))]
struct DmaCameraBuffers {
    lli: [DmaLli; MAX_LLI],
    dst: [u8; FRAME_CAP],
}

struct DmaCameraStatic(UnsafeCell<DmaCameraBuffers>);
unsafe impl Sync for DmaCameraStatic {}

static CAM_BUF: DmaCameraStatic = DmaCameraStatic(UnsafeCell::new(DmaCameraBuffers {
    lli: [DmaLli::new(); MAX_LLI],
    dst: [0u8; FRAME_CAP],
}));

unsafe fn cam_buf_mut() -> &'static mut DmaCameraBuffers {
    unsafe { &mut *CAM_BUF.0.get() }
}

/// 将 `frame` 做一次 DMAC 内存拷贝，并与源数据逐字节比对。
///
/// 须在单线程路径调用；成功表示 DMAC 与摄像头缓冲协作正常。
pub fn memcopy_frame_verify(frame: &[u8]) -> bool {
    if frame.is_empty() {
        println!("[dma_camera] empty frame");
        return false;
    }
    if frame.len() > FRAME_CAP {
        println!(
            "[dma_camera] frame len {} > cap {}",
            frame.len(),
            FRAME_CAP
        );
        return false;
    }

    fence(Ordering::SeqCst);

    let b = unsafe { cam_buf_mut() };
    b.dst[..frame.len()].fill(0xA5);
    b.dst[frame.len()..].fill(0);

    let src_pa = va_to_phys(frame.as_ptr());
    let dst_pa = va_to_phys(b.dst.as_ptr());
    let lli_phys_base = va_to_phys(addr_of!(b.lli[0]));

    let n = prepare_memcpy_lli(
        &mut b.lli,
        lli_phys_base,
        src_pa,
        dst_pa,
        frame.len(),
        1024,
        4,
    );
    if n == 0 {
        println!("[dma_camera] prepare_memcpy_lli returned 0");
        return false;
    }

    let mut dma = DmaController::new(dmac_vaddr());
    dma.init();
    if dma.alloc_channel(DMA_CH).is_err() {
        println!("[dma_camera] alloc_channel failed");
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
        println!("[dma_camera] configure_channel failed");
        return false;
    }

    unsafe {
        dcache_flush_all();
    }

    if dma.start_transfer(DMA_CH, lli_phys_base).is_err() {
        dma.free_channel(DMA_CH);
        println!("[dma_camera] start_transfer failed");
        return false;
    }

    let (spins, ok) = wait_channel(&dma, 50_000_000);
    let ist = dma.channels[DMA_CH].read_int_status();
    dma.free_channel(DMA_CH);

    unsafe {
        dcache_flush_all();
    }

    if !ok {
        println!(
            "[dma_camera] DMA timeout spins={} ch_int={:#x}",
            spins, ist
        );
        return false;
    }
    if (ist & INT_ALL_ERR) != 0 {
        println!("[dma_camera] DMA error ch_int={:#x}", ist);
        return false;
    }

    let match_all = frame.iter().zip(b.dst[..frame.len()].iter()).all(|(a, d)| a == d);
    if !match_all {
        let off = frame
            .iter()
            .zip(b.dst[..frame.len()].iter())
            .position(|(a, d)| a != d)
            .unwrap_or(0);
        println!(
            "[dma_camera] mismatch at {} src={:#x} dst={:#x}",
            off, frame[off], b.dst[off]
        );
        return false;
    }

    let guard_end = (frame.len() + 64).min(FRAME_CAP);
    if b.dst[frame.len()..guard_end].iter().any(|&x| x != 0) {
        println!("[dma_camera] unexpected data past frame (guard)");
        return false;
    }

    println!(
        "[dma_camera] M2M ok: {} bytes, {} LLIs",
        frame.len(),
        n
    );
    true
}
