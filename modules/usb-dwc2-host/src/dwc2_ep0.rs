//! DWC2 主机：**通道 0** 仅 EP0 控制；**通道 1** 仅 Bulk（避免与部分 IP/QEMU 模型在单通道复用上的异常）。
//!
//! QEMU `usb-storage`：Bulk IN = EP **1**，Bulk OUT = EP **2**（见 `hw/usb/dev-storage.c`）。

use super::error::{UsbError, UsbResult};
use super::setup;
use crate::cache;
use crate::mmio;
use crate::platform;

#[inline]
fn base() -> usize {
    platform::dwc2_base_virt()
}

/// 每个主机通道寄存器块 0x20 字节；`HCCHARn` 基址 0x500。
const HC_BLK: usize = 0x500;
const HC_STRIDE: usize = 0x20;

const OFF_HCCHAR: usize = 0x00;
const OFF_HCSPLT: usize = 0x04;
const OFF_HCINT: usize = 0x08;
const OFF_HCINTMSK: usize = 0x0C;
const OFF_HCTSIZ: usize = 0x10;
const OFF_HCDMA: usize = 0x14;

// 全局寄存器（与 `dwc2.rs` 一致），仅调试转储用。
const R_GAHBCFG: usize = 0x008;
const R_GINTSTS: usize = 0x014;
const R_GINTMSK: usize = 0x018;
const R_GRSTCTL: usize = 0x010;
const R_GHWCFG2: usize = 0x048;
const R_GOTGCTL: usize = 0x000;
const R_HPRT0: usize = 0x440;

/// EP0 控制传输固定用通道 0。
const CH_CTL: u32 = 0;
/// Bulk 传输固定用通道 1（与控制分离）。
const CH_BULK: u32 = 1;

const HCCHAR_CHENA: u32 = 1 << 31;
const HCCHAR_CHDIS: u32 = 1 << 30;
const HCCHAR_EPDIR: u32 = 1 << 15;
const HCCHAR_EPTYPE_CONTROL: u32 = 0 << 18;
const HCCHAR_EPTYPE_BULK: u32 = 2 << 18;

const HCINT_CHHLTD: u32 = 1 << 1;
const HCINT_XFERCOMPL: u32 = 1 << 0;
const HCINT_STALL: u32 = 1 << 3;
const HCINT_NAK: u32 = 1 << 4;
const HCINT_XACTERR: u32 = 1 << 7;

const TSIZ_PID_SHIFT: u32 = 29;
const TSIZ_PKTCNT_SHIFT: u32 = 19;
pub const PID_DATA0: u32 = 0;
pub const PID_DATA1: u32 = 2;
const PID_SETUP: u32 = 3;

#[repr(C, align(256))]
struct DmaBuf {
    bytes: [u8; 1024],
}

static mut DMA_BUF: DmaBuf = DmaBuf { bytes: [0; 1024] };

const OFF_EP0: usize = 0;
pub const DMA_OFF_SECTOR: usize = 64;
pub const DMA_OFF_CSW: usize = 64 + 512;
pub const DMA_OFF_CBW: usize = 0;
/// EP0 小缓冲读（Hub 描述符、配置前缀、`GET_PORT_STATUS`），与 Bulk DMA 区错开。
const DMA_OFF_SMALL_IO: usize = 256;

#[inline]
fn hc_addr(ch: u32, reg_off: usize) -> usize {
    base() + HC_BLK + (ch as usize) * HC_STRIDE + reg_off
}

#[inline]
fn spin_delay(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

/// DMA 工作区基址（`static mut` 仅经裸指针访问，避免 `static_mut_refs`）。
#[inline]
fn dma_ptr() -> *mut u8 {
    core::ptr::addr_of_mut!(DMA_BUF).cast::<u8>()
}

fn dma_phys(off: usize) -> u32 {
    unsafe { crate::platform::usb_dma_phys_for(dma_ptr().add(off)) }
}

#[inline]
fn usb_bus_fence_before_dma() {
    #[cfg(target_arch = "riscv64")]
    unsafe {
        core::arch::asm!("fence rw, rw", options(nostack));
    }
}

/// 枚举前打印 EP0/DMA 窗口 VA→PA 与 `GHWCFG2.ARCH`（调试用）。
pub fn debug_log_ep0_dma_info() {
    let b = base();
    if b == 0 {
        crate::log::usb_log_fmt(format_args!("USB-DBG ep0_dma: DWC2 base not set"));
        return;
    }
    unsafe {
        let va = dma_ptr() as usize;
        let pa_base = crate::platform::usb_dma_phys_for(dma_ptr());
        let pa_ep0 = crate::platform::usb_dma_phys_for(dma_ptr().add(OFF_EP0));
        let g2 = mmio::read32(b + R_GHWCFG2);
        let arch = (g2 >> 3) & 3u32;
        crate::log::usb_log_fmt(format_args!(
            "USB-DBG ep0_dma va(buf)={:#010x} pa(buf)={:#010x} pa(setup)={:#010x}",
            va, pa_base, pa_ep0
        ));
        crate::log::usb_log_fmt(format_args!(
            "USB-DBG GHWCFG2={:#010x} ARCH={} (0=slave 1=ext-dma 2=int-dma)",
            g2, arch
        ));
        let snpsid = mmio::read32(b + 0x040);
        crate::log::usb_log_fmt(format_args!(
            "USB-DBG GSNPSID={:#010x} core_rev={:#06x}",
            snpsid,
            snpsid & 0xffff
        ));
        if arch == 2 {
            crate::log::usb_log_fmt(format_args!(
                "USB-DBG ARCH=2 为内部 DMA：主机通道必须用 HCDMA，不能关 DMA 改纯 FIFO/slave 枚举"
            ));
        }
        if pa_base as usize == va {
            crate::log::usb_log_fmt(format_args!(
                "USB-DBG ep0_dma: VA==PA（恒等映射），HCDMA 地址与 Linux phys-virt-offset=0 一致"
            ));
        }
    }
}

unsafe fn dump_channel_timeout_debug(ch: u32, phase: &'static str) {
    let b = base();
    if b == 0 {
        return;
    }
    unsafe {
        let hprt = mmio::read32(b + R_HPRT0);
        let gint = mmio::read32(b + R_GINTSTS);
        let gintm = mmio::read32(b + R_GINTMSK);
        let gahb = mmio::read32(b + R_GAHBCFG);
        let grst = mmio::read32(b + R_GRSTCTL);
        let gotg = mmio::read32(b + R_GOTGCTL);
        let hcchar = mmio::read32(hc_addr(ch, OFF_HCCHAR));
        let hcint = mmio::read32(hc_addr(ch, OFF_HCINT));
        let hcintm = mmio::read32(hc_addr(ch, OFF_HCINTMSK));
        let hctsiz = mmio::read32(hc_addr(ch, OFF_HCTSIZ));
        let hcdma = mmio::read32(hc_addr(ch, OFF_HCDMA));
        crate::log::usb_log_fmt(format_args!(
            "USB-TOUT [{}] ch={} HPRT0={:#010x} (CONNSTS={} SPD={})",
            phase,
            ch,
            hprt,
            (hprt & 1) != 0,
            (hprt >> 17) & 3
        ));
        crate::log::usb_log_fmt(format_args!(
            "USB-TOUT GINTSTS={:#010x} GINTMSK={:#010x} GAHBCFG={:#010x} GRSTCTL={:#010x}",
            gint, gintm, gahb, grst
        ));
        crate::log::usb_log_fmt(format_args!(
            "USB-TOUT GOTGCTL={:#010x} HCCHAR={:#010x} HCINT={:#010x} HCINTMSK={:#010x}",
            gotg, hcchar, hcint, hcintm
        ));
        crate::log::usb_log_fmt(format_args!(
            "USB-TOUT HCTSIZ={:#010x} HCDMA={:#010x}",
            hctsiz, hcdma
        ));
    }
}

unsafe fn ch_wait_disabled(ch: u32) -> UsbResult<()> {
    unsafe {
        for _ in 0..2_000_000u32 {
            let c = mmio::read32(hc_addr(ch, OFF_HCCHAR));
            if c & HCCHAR_CHENA == 0 {
                return Ok(());
            }
            spin_delay(8);
        }
        dump_channel_timeout_debug(ch, "ch_wait_disabled");
        Err(UsbError::Timeout)
    }
}

/// 若通道仍忙，按 Linux `dwc2_hc_halt` 同时置 `CHENA|CHDIS` 请求停止。
unsafe fn ch_halt(ch: u32) {
    unsafe {
        let c = mmio::read32(hc_addr(ch, OFF_HCCHAR));
        if c & HCCHAR_CHENA == 0 {
            return;
        }
        mmio::write32(hc_addr(ch, OFF_HCCHAR), c | HCCHAR_CHENA | HCCHAR_CHDIS);
        for _ in 0..500_000u32 {
            let v = mmio::read32(hc_addr(ch, OFF_HCCHAR));
            if v & HCCHAR_CHENA == 0 {
                return;
            }
            spin_delay(8);
        }
    }
}

unsafe fn ch_wait_halted(ch: u32) -> UsbResult<u32> {
    unsafe {
        for _ in 0..8_000_000u32 {
            let hi = mmio::read32(hc_addr(ch, OFF_HCINT));
            if hi & HCINT_CHHLTD != 0 {
                mmio::write32(hc_addr(ch, OFF_HCINT), hi);
                return Ok(hi);
            }
            spin_delay(8);
        }
        dump_channel_timeout_debug(ch, "ch_wait_halted");
        Err(UsbError::Timeout)
    }
}

unsafe fn ch_xfer(ch: u32, hcchar: u32, hctsiz: u32, dma_off: u32) -> UsbResult<u32> {
    unsafe {
        ch_wait_disabled(ch)?;
        ch_halt(ch);
        mmio::write32(hc_addr(ch, OFF_HCSPLT), 0);
        mmio::write32(hc_addr(ch, OFF_HCINT), 0x7FF);
        mmio::write32(hc_addr(ch, OFF_HCTSIZ), hctsiz);
        let dmap = dma_phys(dma_off as usize);
        usb_bus_fence_before_dma();
        mmio::write32(hc_addr(ch, OFF_HCDMA), dmap);
        usb_bus_fence_before_dma();
        mmio::write32(hc_addr(ch, OFF_HCCHAR), hcchar | HCCHAR_CHENA);
        let st = ch_wait_halted(ch)?;
        if st & HCINT_STALL != 0 {
            return Err(UsbError::Stall);
        }
        if st & HCINT_XACTERR != 0 || st & HCINT_NAK != 0 {
            return Err(UsbError::Protocol("ch xfer error (NAK/XACT)"));
        }
        if st & HCINT_XFERCOMPL == 0 {
            return Err(UsbError::Protocol("CHHLTD without XFERCOMPL"));
        }
        Ok(st)
    }
}

unsafe fn hcchar_control(dev: u32, ep: u32, mps: u32, dir_in: bool) -> u32 {
    let mut v = mps & 0x7ff;
    v |= (ep & 0xf) << 11;
    if dir_in {
        v |= HCCHAR_EPDIR;
    }
    v |= HCCHAR_EPTYPE_CONTROL;
    v |= (dev & 0x7f) << 22;
    v
}

unsafe fn hcchar_bulk(dev: u32, ep: u32, mps: u32, dir_in: bool) -> u32 {
    let mut v = mps & 0x7ff;
    v |= (ep & 0xf) << 11;
    if dir_in {
        v |= HCCHAR_EPDIR;
    }
    v |= HCCHAR_EPTYPE_BULK;
    v |= (dev & 0x7f) << 22;
    v
}

unsafe fn hctsiz(pid: u32, pktcnt: u32, xfersize: u32) -> u32 {
    (pid << TSIZ_PID_SHIFT) | (pktcnt << TSIZ_PKTCNT_SHIFT) | (xfersize & 0x7ffff)
}

pub fn usb_post_set_address_delay() {
    spin_delay(20_000_000);
}

/// Hub 下游端口 `PORT_RESET` 后给设备恢复时间（粗粒度忙等）。
pub fn usb_post_hub_port_reset_delay() {
    spin_delay(30_000_000);
}

#[inline]
fn normalize_ep0_mps(b: u8) -> u32 {
    match b {
        8 | 16 | 32 | 64 => b as u32,
        _ => 8,
    }
}

pub fn ep0_control_write_no_data(dev: u32, setup: [u8; 8], ep0_mps: u32) -> UsbResult<()> {
    unsafe {
        core::ptr::copy_nonoverlapping(setup.as_ptr(), dma_ptr().add(OFF_EP0), 8);
        cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

        let hc = hcchar_control(dev, 0, ep0_mps, false);
        ch_xfer(CH_CTL, hc, hctsiz(PID_SETUP, 1, 8), OFF_EP0 as u32)?;

        let hc = hcchar_control(dev, 0, ep0_mps, true);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, 0),
            OFF_EP0 as u32,
        )?;
        Ok(())
    }
}

pub fn set_usb_address(addr: u8, ep0_mps: u32) -> UsbResult<()> {
    ep0_control_write_no_data(0, setup::set_address(addr), ep0_mps)
}

pub fn set_configuration(dev: u32, cfg: u8, ep0_mps: u32) -> UsbResult<()> {
    ep0_control_write_no_data(dev, setup::set_configuration(cfg), ep0_mps)
}

pub fn mass_storage_bulk_only_reset(dev: u32, interface: u16, ep0_mps: u32) -> UsbResult<()> {
    ep0_control_write_no_data(dev, setup::mass_storage_reset(interface), ep0_mps)
}

#[allow(dead_code)]
pub fn get_configuration(dev: u32, ep0_mps: u32) -> UsbResult<u8> {
    unsafe {
        let setup_pkt = setup::get_configuration();
        core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
        cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

        let mut hc = hcchar_control(dev, 0, ep0_mps, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_SETUP, 1, 8),
            OFF_EP0 as u32,
        )?;

        hc = hcchar_control(dev, 0, ep0_mps, true);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, 1),
            OFF_EP0 as u32,
        )?;
        cache::dcache_invalidate_after_dma(dma_ptr().add(OFF_EP0), 1);
        let v = dma_ptr().add(OFF_EP0).read();

        hc = hcchar_control(dev, 0, ep0_mps, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, 0),
            OFF_EP0 as u32,
        )?;

        Ok(v)
    }
}

#[allow(dead_code)]
pub fn get_max_lun(dev: u32, interface: u16, ep0_mps: u32) -> UsbResult<u8> {
    unsafe {
        let setup_pkt = setup::get_max_lun(interface);
        core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
        cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

        let mut hc = hcchar_control(dev, 0, ep0_mps, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_SETUP, 1, 8),
            OFF_EP0 as u32,
        )?;

        hc = hcchar_control(dev, 0, ep0_mps, true);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, 1),
            OFF_EP0 as u32,
        )?;
        cache::dcache_invalidate_after_dma(dma_ptr().add(OFF_EP0), 1);
        let lun = dma_ptr().add(OFF_EP0).read();

        hc = hcchar_control(dev, 0, ep0_mps, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, 0),
            OFF_EP0 as u32,
        )?;

        Ok(lun)
    }
}

/// `GET_DESCRIPTOR(DEVICE, 18)` @ 地址 0；返回 VID、PID、EP0 MPS、`bDeviceClass`。
pub fn get_device_vid_pid_default_addr() -> UsbResult<(u16, u16, u32, u8)> {
    unsafe {
        let wlen: u16 = 18;
        let setup_pkt = setup::get_descriptor_device(wlen);
        core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
        cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

        let mut hc = hcchar_control(0, 0, 64, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_SETUP, 1, 8),
            OFF_EP0 as u32,
        )?;

        hc = hcchar_control(0, 0, 64, true);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, wlen as u32),
            OFF_EP0 as u32,
        )?;
        cache::dcache_invalidate_after_dma(dma_ptr().add(OFF_EP0), wlen as usize);

        let sl = core::slice::from_raw_parts(dma_ptr().add(OFF_EP0), wlen as usize);
        if sl.len() < 12 {
            return Err(UsbError::Protocol("short descriptor"));
        }
        let vid = u16::from_le_bytes([sl[8], sl[9]]);
        let pid = u16::from_le_bytes([sl[10], sl[11]]);
        let ep0_mps = normalize_ep0_mps(sl[7]);
        let b_device_class = sl[4];

        hc = hcchar_control(0, 0, 64, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, 0),
            OFF_EP0 as u32,
        )?;

        Ok((vid, pid, ep0_mps, b_device_class))
    }
}

/// 对 **已寻址** 设备发送 Hub `SET_PORT_FEATURE`（无数据阶段）。
pub fn hub_set_port_feature(dev: u32, port: u16, feature: u16, ep0_mps: u32) -> UsbResult<()> {
    ep0_control_write_no_data(dev, setup::hub_set_port_feature(port, feature), ep0_mps)
}

/// 控制传输：SETUP + 若干 IN 数据包（DATA1/DATA0 交替）+ STATUS OUT（ZLP，DATA1）。
///
/// 数据写入 `out`（总长度 = `out.len()`）。适用于 Hub 描述符、配置前缀、`GET_PORT_STATUS` 等。
pub fn ep0_control_read(dev: u32, setup_pkt: [u8; 8], ep0_mps: u32, out: &mut [u8]) -> UsbResult<()> {
    if out.is_empty() || out.len() > 512 {
        return Err(UsbError::Protocol("bad ep0 read len"));
    }
    let total = out.len() as u32;
    unsafe {
        core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
        cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

        let mut hc = hcchar_control(dev, 0, ep0_mps, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_SETUP, 1, 8),
            OFF_EP0 as u32,
        )?;

        let mut left = total;
        let mut out_off: usize = 0;
        let mut toggle = PID_DATA1;
        while left > 0 {
            let chunk = left.min(ep0_mps);
            let pkts = pktcnt_for(ep0_mps, chunk);
            hc = hcchar_control(dev, 0, ep0_mps, true);
            ch_xfer(
                CH_CTL,
                hc,
                hctsiz(toggle, pkts, chunk),
                DMA_OFF_SMALL_IO as u32,
            )?;
            cache::dcache_invalidate_after_dma(dma_ptr().add(DMA_OFF_SMALL_IO), chunk as usize);
            core::ptr::copy_nonoverlapping(
                dma_ptr().add(DMA_OFF_SMALL_IO),
                out.as_mut_ptr().add(out_off),
                chunk as usize,
            );
            out_off += chunk as usize;
            left -= chunk;
            toggle = if toggle == PID_DATA1 {
                PID_DATA0
            } else {
                PID_DATA1
            };
        }

        hc = hcchar_control(dev, 0, ep0_mps, false);
        ch_xfer(
            CH_CTL,
            hc,
            hctsiz(PID_DATA1, 1, 0),
            OFF_EP0 as u32,
        )?;
        Ok(())
    }
}

pub fn dma_copy_out(off: usize, dst: &mut [u8]) {
    unsafe {
        core::ptr::copy_nonoverlapping(dma_ptr().add(off), dst.as_mut_ptr(), dst.len());
    }
}

fn pktcnt_for(mps: u32, nbytes: u32) -> u32 {
    if mps == 0 {
        return 1;
    }
    (nbytes + mps - 1) / mps
}

pub fn bulk_out(dev: u32, ep: u32, mps: u32, pid: u32, data: &[u8], dma_off: usize) -> UsbResult<()> {
    if data.is_empty() || data.len() > 0x7ffff {
        return Err(UsbError::Protocol("bad bulk out len"));
    }
    unsafe {
        core::ptr::copy_nonoverlapping(data.as_ptr(), dma_ptr().add(dma_off), data.len());
        cache::dcache_clean_for_dma(dma_ptr().add(dma_off), data.len());
        let hc = hcchar_bulk(dev, ep, mps, false);
        let pkts = pktcnt_for(mps, data.len() as u32);
        ch_xfer(
            CH_BULK,
            hc,
            hctsiz(pid, pkts, data.len() as u32),
            dma_off as u32,
        )?;
        Ok(())
    }
}

pub fn bulk_in(dev: u32, ep: u32, mps: u32, pid: u32, len: usize, dma_off: usize) -> UsbResult<()> {
    if len == 0 || len > 0x7ffff {
        return Err(UsbError::Protocol("bad bulk in len"));
    }
    unsafe {
        let hc = hcchar_bulk(dev, ep, mps, true);
        let pkts = pktcnt_for(mps, len as u32);
        ch_xfer(
            CH_BULK,
            hc,
            hctsiz(pid, pkts, len as u32),
            dma_off as u32,
        )?;
        cache::dcache_invalidate_after_dma(dma_ptr().add(dma_off), len);
        Ok(())
    }
}
