//! 32-bit MMIO helpers（后续 USB 寄存器与 DMA 属性会在此集中处理）。

use core::ptr::{read_volatile, write_volatile};

#[inline(always)]
pub unsafe fn read32(addr: usize) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}

#[inline(always)]
pub unsafe fn write32(addr: usize, val: u32) {
    unsafe { write_volatile(addr as *mut u32, val) }
}

#[inline(always)]
pub unsafe fn modify32(addr: usize, mask: u32, bits: u32) {
    let v = unsafe { read32(addr) };
    unsafe { write32(addr, (v & !mask) | (bits & mask)) }
}
