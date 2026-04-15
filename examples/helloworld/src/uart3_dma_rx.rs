//! UART3 RX: CPU spin-loop reader.
//!
//! UART3 DMA RX was attempted but the SG2002 SoC does not appear to have the
//! UART3 DMA request lines (14/15) physically connected to the SDMA controller.
//! The Linux kernel DTS for this SoC also does not use DMA for any UART.
//! We therefore use a tight LSR+RBR polling loop with interrupts disabled.

use axhal::mem::{phys_to_virt, PhysAddr};
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{fence, Ordering};

const UART3_PHYS: usize = 0x0417_0000;

pub const UART3_RX_HS_ID: u8 = 14;
pub const UART3_TX_HS_ID: u8 = 15;

const OFF_IER: usize = 0x04;
const OFF_FCR: usize = 0x08;
const OFF_LSR: usize = 0x14;

const FCR_FIFO_EN: u32 = 0x01;
const LSR_DR: u32 = 1;
const LSR_OE: u32 = 1 << 1;

static OE_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub fn overrun_count() -> u32 { OE_COUNT.load(core::sync::atomic::Ordering::Relaxed) }

fn uart3_va() -> usize {
    phys_to_virt(PhysAddr::from_usize(UART3_PHYS)).as_usize()
}

#[inline(always)]
fn ureg(off: usize) -> u32 {
    unsafe { read_volatile((uart3_va() + off) as *const u32) }
}
#[inline(always)]
fn uwreg(off: usize, v: u32) {
    unsafe { write_volatile((uart3_va() + off) as *mut u32, v) }
}

pub fn uart3_enable_dma_rx_mode() {
    uwreg(OFF_IER, 0);
    uwreg(OFF_FCR, FCR_FIFO_EN);
    fence(Ordering::SeqCst);
}

pub fn uart3_flush_and_clear() {
    uwreg(OFF_FCR, FCR_FIFO_EN | 0x02);
    fence(Ordering::SeqCst);
    uwreg(OFF_FCR, FCR_FIFO_EN);
    fence(Ordering::SeqCst);
    let _ = ureg(OFF_LSR);
    let _ = ureg(0x00);
}

/// No-op: DMA is not available on SG2002 UART3.
pub fn mark_dma_failed() {}

/// Always false on SG2002 (UART DMA not wired).
pub fn is_using_dma() -> bool { false }

/// Tight spin-loop reading UART3 RBR via LSR polling.
/// Disables interrupts during the critical section to prevent FIFO overflow.
pub fn read_bytes_dma_or_cpu(buf: &mut [u8]) -> usize {
    if buf.is_empty() {
        return 0;
    }

    let base = uart3_va();
    let mut n = 0usize;
    let mut idle = 0u32;

    let saved: usize;
    unsafe { core::arch::asm!("csrr {}, sstatus", out(reg) saved); }
    unsafe { core::arch::asm!("csrci sstatus, 2"); }

    while n < buf.len() {
        let lsr = unsafe { read_volatile((base + OFF_LSR) as *const u32) };
        if lsr & LSR_OE != 0 {
            OE_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        if lsr & LSR_DR != 0 {
            buf[n] = unsafe { read_volatile(base as *const u32) } as u8;
            n += 1;
            idle = 0;
        } else {
            idle += 1;
            let limit = if n == 0 { 5_000 } else if n < 32 { 8_000 } else { 50_000 };
            if idle > limit {
                break;
            }
        }
    }

    if saved & 2 != 0 {
        unsafe { core::arch::asm!("csrsi sstatus, 2"); }
    }
    n
}
