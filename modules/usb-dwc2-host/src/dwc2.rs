//! Synopsys DWC2：探测、主机模式 bring-up（M1）、后续通道传输（M2+）。
//!
//! 寄存器名与位定义对齐 Linux `drivers/usb/dwc2/hw.h`（DesignWare OTG 2.0）。
//!
//! 启用 feature **`cv182x-host`** 时，主机初始化对齐 Linux
//! `dwc2_set_cv182x_params` + `dwc2_core_host_init` / `dwc2_config_fifos`（UTMI 16-bit、HS、动态 FIFO、
//! `GDFIFOCFG`、`PCGCTL`、`TOUTCAL`），见
//! [Sipeed LicheeRV-Nano `params.c`](https://github.com/sipeed/LicheeRV-Nano-Build/blob/d4003f15b35d43ad4842f427050ab2bba0114fa5/linux_5.10/drivers/usb/dwc2/params.c#L217)。

use super::error::{UsbError, UsbResult};
use crate::mmio;
use crate::platform;

#[inline]
fn base() -> usize {
    platform::dwc2_base_virt()
}

/// `dwc2_host_init` 内超时（`wait_ahb_idle` / 软复位 / FIFO flush）时转储；与 EP0 的 `USB-TOUT ch_*` 区分。
unsafe fn dbg_dwc2_init_timeout(phase: &'static str) {
    let b = base();
    if b == 0 {
        return;
    }
    unsafe {
        let grst = mmio::read32(b + GRSTCTL);
        let gint = mmio::read32(b + GINTSTS);
        let gahb = mmio::read32(b + GAHBCFG);
        let hprt = mmio::read32(b + HPRT0);
        let ahb_idle = (grst & GRSTCTL_AHBIDLE) != 0;
        let csftrst = (grst & GRSTCTL_CSFTRST) != 0;
        let rst_done = (grst & GRSTCTL_CSFTRST_DONE) != 0;
        let rx_flush = (grst & GRSTCTL_RXFFLSH) != 0;
        let tx_flush = (grst & GRSTCTL_TXFFLSH) != 0;
        crate::log::usb_log_fmt(format_args!(
            "USB-TOUT dwc2-init [{}] GRSTCTL={:#010x} AHBIDLE={} CSFTRST={} CSFTRST_DONE={} RXFFLSH={} TXFFLSH={}",
            phase, grst, ahb_idle, csftrst, rst_done, rx_flush, tx_flush
        ));
        crate::log::usb_log_fmt(format_args!(
            "USB-TOUT dwc2-init [{}] GINTSTS={:#010x} GAHBCFG={:#010x} HPRT0={:#010x}",
            phase, gint, gahb, hprt
        ));
    }
}

// --- 全局寄存器偏移（HSOTG_REG）---
#[cfg(feature = "cv182x-host")]
const GOTGCTL: usize = 0x000;
const GAHBCFG: usize = 0x008;
const GUSBCFG: usize = 0x00C;
const GRSTCTL: usize = 0x010;
const GINTSTS: usize = 0x014;
const GINTMSK: usize = 0x018;
const GRXFSIZ: usize = 0x024;
const GNPTXFSIZ: usize = 0x028;
const HPTXFSIZ: usize = 0x100;
const GSNPSID: usize = 0x040;
const GHWCFG1: usize = 0x044;
const GHWCFG2: usize = 0x048;
const GHWCFG3: usize = 0x04C;
const GHWCFG4: usize = 0x050;
const GDFIFOCFG: usize = 0x05C;
const PCGCTL: usize = 0xE00;

// --- Host ---
const HCFG: usize = 0x400;
const HPRT0: usize = 0x440;
const HAINTMSK: usize = 0x418;

// GUSBCFG
const GUSBCFG_FORCEHOSTMODE: u32 = 1 << 29;
const GUSBCFG_ULPI_UTMI_SEL: u32 = 1 << 4;
const GUSBCFG_PHYIF16: u32 = 1 << 3;
const GUSBCFG_TOUTCAL_MASK: u32 = 0x7;

// GRSTCTL
const GRSTCTL_AHBIDLE: u32 = 1 << 31;
const GRSTCTL_CSFTRST_DONE: u32 = 1 << 29;
const GRSTCTL_CSFTRST: u32 = 1 << 0;
const GRSTCTL_TXFFLSH: u32 = 1 << 5;
const GRSTCTL_RXFFLSH: u32 = 1 << 4;
const GRSTCTL_TXFNUM_SHIFT: u32 = 6;

// GAHBCFG
const GAHBCFG_GLBL_INTR_EN: u32 = 1 << 0;
const GAHBCFG_DMA_EN: u32 = 1 << 5;
const GAHBCFG_HBSTLEN_SHIFT: u32 = 1;
const GAHBCFG_HBSTLEN_INCR16: u32 = 7;

// GINTSTS / GINTMSK
const GINTSTS_CURMODE_HOST: u32 = 1 << 0;
const GINTMSK_HCHINT: u32 = 1 << 25;

// HCFG
const HCFG_FSLSSUPP: u32 = 1 << 2;
const HCFG_FSLSPCLKSEL_MASK: u32 = 0x3;
#[cfg(not(feature = "cv182x-host"))]
const HCFG_FSLSPCLKSEL_48_MHZ: u32 = 1;

// GHWCFG2 / GHWCFG4
const GHWCFG2_NUM_HOST_CHAN_SHIFT: u32 = 14;
const GHWCFG2_NUM_HOST_CHAN_MASK: u32 = 0xf;
const GHWCFG4_DED_FIFO_EN: u32 = 1 << 25;

// Linux `core.h`：`snpsid >= 0x4f54291a` 时配置 `GDFIFOCFG`（`hcd.c`）。
const DWC2_CORE_REV_2_91A: u32 = 0x4f54_291a;
/// 软复位序列分界：见 Linux `dwc2_core_reset()`（≥ 此版本用 `CSFTRST_DONE`，不再傻等 `CSFTRST` 自清）。
const DWC2_CORE_REV_4_20A: u32 = 0x4f54_420a;
const DWC2_CORE_REV_MASK: u32 = 0xffff;
const GDFIFOCFG_EPINFOBASE_SHIFT: u32 = 16;
const GDFIFOCFG_EPINFOBASE_MASK: u32 = 0xffff << 16;

// GOTGCTL（OTG：无 ID/VBUS 检测时由软件断言 A-session / VBUS，见 Linux `dwc2_ovr_avalid`）
#[cfg(feature = "cv182x-host")]
const GOTGCTL_DBNCE_FLTR_BYPASS: u32 = 1 << 15;
#[cfg(feature = "cv182x-host")]
const GOTGCTL_AVALOEN: u32 = 1 << 4;
#[cfg(feature = "cv182x-host")]
const GOTGCTL_AVALOVAL: u32 = 1 << 5;
#[cfg(feature = "cv182x-host")]
const GOTGCTL_VBVALOEN: u32 = 1 << 2;
#[cfg(feature = "cv182x-host")]
const GOTGCTL_VBVALOVAL: u32 = 1 << 3;

// HPRT0
const HPRT0_PWR: u32 = 1 << 12;
const HPRT0_RST: u32 = 1 << 8;
const HPRT0_CONNSTS: u32 = 1 << 0;
const HPRT0_CONNDET: u32 = 1 << 1;
const HPRT0_ENA: u32 = 1 << 2;

#[inline]
fn spin_delay(iterations: u32) {
    for _ in 0..iterations {
        core::hint::spin_loop();
    }
}

/// 读取硬件配置寄存器（上电后通常非零，用于 M0/M1「控制器是否可见」自检）。
pub unsafe fn dwc2_probe() -> UsbResult<(u32, u32, u32)> {
    unsafe {
        if base() == 0 {
            return Err(UsbError::Hardware("DWC2 base not set (call platform::set_dwc2_base_virt)"));
        }
        let h1 = mmio::read32(base() + GHWCFG1);
        let h2 = mmio::read32(base() + GHWCFG2);
        let h3 = mmio::read32(base() + GHWCFG3);
        if h2 == 0 && h3 == 0 {
            return Err(UsbError::Hardware(
                "DWC2 GHWCFG2/3 zero (no controller?)",
            ));
        }
        Ok((h1, h2, h3))
    }
}

unsafe fn wait_ahb_idle() -> UsbResult<()> {
    unsafe {
        for _ in 0..3_000_000u32 {
            if mmio::read32(base() + GRSTCTL) & GRSTCTL_AHBIDLE != 0 {
                return Ok(());
            }
            spin_delay(32);
        }
        dbg_dwc2_init_timeout("wait_ahb_idle");
        Err(UsbError::Timeout)
    }
}

unsafe fn core_soft_reset() -> UsbResult<()> {
    unsafe {
        wait_ahb_idle()?;
        let snpsid = mmio::read32(base() + GSNPSID);
        let core_rev = snpsid & DWC2_CORE_REV_MASK;
        let new_rst_seq = core_rev >= (DWC2_CORE_REV_4_20A & DWC2_CORE_REV_MASK);

        let mut greset = mmio::read32(base() + GRSTCTL);
        greset |= GRSTCTL_CSFTRST;
        mmio::write32(base() + GRSTCTL, greset);

        if !new_rst_seq {
            for _ in 0..3_000_000u32 {
                if mmio::read32(base() + GRSTCTL) & GRSTCTL_CSFTRST == 0 {
                    spin_delay(4096);
                    return Ok(());
                }
                spin_delay(32);
            }
            dbg_dwc2_init_timeout("core_soft_reset CSFTRST (legacy)");
            return Err(UsbError::Timeout);
        }

        // Linux `dwc2_core_reset`：Core ≥ 4.20a 时等 `CSFTRST_DONE`，再清 `CSFTRST` 并置位 `CSFTRST_DONE`。
        for _ in 0..3_000_000u32 {
            if mmio::read32(base() + GRSTCTL) & GRSTCTL_CSFTRST_DONE != 0 {
                greset = mmio::read32(base() + GRSTCTL);
                greset &= !GRSTCTL_CSFTRST;
                greset |= GRSTCTL_CSFTRST_DONE;
                mmio::write32(base() + GRSTCTL, greset);
                spin_delay(4096);
                return Ok(());
            }
            spin_delay(32);
        }
        dbg_dwc2_init_timeout("core_soft_reset CSFTRST_DONE");
        Err(UsbError::Timeout)
    }
}

unsafe fn force_host_mode() -> UsbResult<()> {
    unsafe {
        let mut v = mmio::read32(base() + GUSBCFG);
        v |= GUSBCFG_FORCEHOSTMODE;
        mmio::write32(base() + GUSBCFG, v);
        spin_delay(100_000);
        for _ in 0..500_000u32 {
            if mmio::read32(base() + GINTSTS) & GINTSTS_CURMODE_HOST != 0 {
                return Ok(());
            }
            spin_delay(32);
        }
        Err(UsbError::Hardware("CURMODE_HOST not set after FORCEHOSTMODE"))
    }
}

/// 配置 RX / NPTX / PTX FIFO（与 Linux `dwc2` 常见默认值同量级；QEMU `raspi3b` 可接受）。
#[cfg(not(feature = "cv182x-host"))]
unsafe fn init_fifos() {
    unsafe {
        const RX_DEPTH: u32 = 0x210; // 32-bit words
        const NPTX_DEPTH: u32 = 0x200;
        const PTX_DEPTH: u32 = 0x200;
        let nptx_start = RX_DEPTH;
        let ptx_start = nptx_start + NPTX_DEPTH;

        mmio::write32(base() + GRXFSIZ, RX_DEPTH);
        mmio::write32(base() + GNPTXFSIZ, (NPTX_DEPTH << 16) | nptx_start);
        mmio::write32(base() + HPTXFSIZ, (PTX_DEPTH << 16) | ptx_start);
    }
}

/// 依据 `GHWCFG2.ARCH` 决定是否置位 `DMA_EN`（内部 DMA 架构时必须开启，EP0 方能用 `HCDMA`）。
#[cfg(not(feature = "cv182x-host"))]
unsafe fn init_gahb() {
    unsafe {
        let g2 = mmio::read32(base() + GHWCFG2);
        let arch = (g2 >> 3) & 0x3;
        let mut v = mmio::read32(base() + GAHBCFG);
        v &= !(1 << 5);
        v |= GAHBCFG_GLBL_INTR_EN | (3 << 1);
        if arch == 2 {
            v |= 1 << 5;
        }
        mmio::write32(base() + GAHBCFG, v);
    }
}

#[cfg(not(feature = "cv182x-host"))]
unsafe fn init_hcfg_fs_ls() {
    unsafe {
        let mut v = mmio::read32(base() + HCFG);
        v |= HCFG_FSLSSUPP;
        v &= !HCFG_FSLSPCLKSEL_MASK;
        v |= HCFG_FSLSPCLKSEL_48_MHZ;
        mmio::write32(base() + HCFG, v);
    }
}

/// 读 `HPRT0`（调试与 M2 端口状态）。
pub unsafe fn dwc2_hprt0_read() -> u32 {
    unsafe { mmio::read32(base() + HPRT0) }
}

#[inline]
pub fn hprt_connsts(hprt: u32) -> bool {
    hprt & HPRT0_CONNSTS != 0
}

#[inline]
pub fn hprt_pwr(hprt: u32) -> bool {
    hprt & HPRT0_PWR != 0
}

#[inline]
#[allow(dead_code)]
pub fn hprt_enabled(hprt: u32) -> bool {
    hprt & HPRT0_ENA != 0
}

/// `HPRT0[18:17]`：0=High、1=Full、2=Low（与 Linux `dwc2` / Synopsys 位定义一致）。
#[inline]
pub fn hprt_speed_bits(hprt: u32) -> u32 {
    (hprt >> 17) & 3
}

/// 根据端口速度选择 Bulk MPS（与 QEMU `usb-storage` HS/FS 描述符一致）。
#[inline]
pub fn suggested_bulk_mps(hprt: u32) -> u32 {
    if hprt_speed_bits(hprt) == 0 {
        512
    } else {
        64
    }
}

unsafe fn port_power_on() {
    unsafe {
        let mut w = mmio::read32(base() + HPRT0);
        w |= HPRT0_PWR;
        // `CONNDET` 为写 1 清除
        if w & HPRT0_CONNDET != 0 {
            w |= HPRT0_CONNDET;
        }
        mmio::write32(base() + HPRT0, w);
    }
}

unsafe fn port_reset_pulse() {
    unsafe {
        let cur = mmio::read32(base() + HPRT0);
        mmio::write32(base() + HPRT0, cur | HPRT0_PWR | HPRT0_RST);
        spin_delay(2_000_000); // ~USB reset 10ms 量级：粗粒度忙等
        let cur2 = mmio::read32(base() + HPRT0);
        mmio::write32(base() + HPRT0, (cur2 | HPRT0_PWR) & !HPRT0_RST);
        spin_delay(500_000);
    }
}

/// `HPRT0[11:10]` 线路状态（Synopsys：`LNSTS`，用于无 `CONNSTS` 时粗判 D+/D- 是否像有上拉）。
#[inline]
pub fn hprt_lnsts(hprt: u32) -> u32 {
    (hprt >> 10) & 3
}

/// 在已检测到设备连接后发出 **USB 总线复位**（应在 `CONNSTS==1` 之后调用，符合主机枚举顺序）。
///
/// 会先对 `CONNDET` 做写 1 清除（若置位），再拉 `PRTRST`。
pub fn dwc2_host_root_bus_reset_pulse() -> UsbResult<()> {
    if base() == 0 {
        return Err(UsbError::Hardware("DWC2 base not set (call platform::set_dwc2_base_virt)"));
    }
    unsafe {
        let mut w = mmio::read32(base() + HPRT0);
        if w & HPRT0_CONNDET != 0 {
            w |= HPRT0_CONNDET;
            mmio::write32(base() + HPRT0, w);
        }
        port_reset_pulse();
    }
    Ok(())
}

/// 根口与片内 PHY 快照（`CONNSTS==0` 排障：LNSTS、GOTGCTL、PHY014 回读、DWC2 控制状态、PHY 全寄存器）。
pub fn debug_dump_root_port_hw(tag: &str) {
    let b = base();
    if b == 0 {
        return;
    }
    unsafe {
        let hprt = mmio::read32(b + HPRT0);
        let ln = hprt_lnsts(hprt);
        crate::log::usb_log_fmt(format_args!(
            "USB-DBG {} HPRT0={:#010x} LNSTS={} CONNSTS={} CONNDET={} RST={} PWR={} SPD={}",
            tag,
            hprt,
            ln,
            hprt & HPRT0_CONNSTS,
            (hprt & HPRT0_CONNDET) >> 1,
            (hprt & HPRT0_RST) >> 8,
            (hprt & HPRT0_PWR) >> 12,
            hprt_speed_bits(hprt),
        ));
        let gotg = mmio::read32(b + 0x000);
        let gusb = mmio::read32(b + GUSBCFG);
        let gahb = mmio::read32(b + GAHBCFG);
        let gint = mmio::read32(b + GINTSTS);
        let pcg  = mmio::read32(b + PCGCTL);
        let hcfg = mmio::read32(b + HCFG);
        crate::log::usb_log_fmt(format_args!(
            "USB-DBG {} GOTGCTL={:#010x} GUSBCFG={:#010x} GAHBCFG={:#010x}",
            tag, gotg, gusb, gahb
        ));
        crate::log::usb_log_fmt(format_args!(
            "USB-DBG {} GINTSTS={:#010x} PCGCTL={:#010x} HCFG={:#010x}",
            tag, gint, pcg, hcfg
        ));
        #[cfg(feature = "cv182x-host")]
        {
            let phy = CV182X_USB2_PHY_MMIO;
            crate::log::usb_log_fmt(format_args!(
                "USB-PHY {} 00={:#010x} 04={:#010x} 08={:#010x} 0c={:#010x}",
                tag,
                mmio::read32(phy + 0x00),
                mmio::read32(phy + 0x04),
                mmio::read32(phy + 0x08),
                mmio::read32(phy + 0x0c),
            ));
            crate::log::usb_log_fmt(format_args!(
                "USB-PHY {} 10={:#010x} 14={:#010x} 18={:#010x} 1c={:#010x}",
                tag,
                mmio::read32(phy + 0x10),
                mmio::read32(phy + 0x14),
                mmio::read32(phy + 0x18),
                mmio::read32(phy + 0x1c),
            ));
            crate::log::usb_log_fmt(format_args!(
                "USB-PHY {} 20={:#010x} 24={:#010x} 28={:#010x} 2c={:#010x}",
                tag,
                mmio::read32(phy + 0x20),
                mmio::read32(phy + 0x24),
                mmio::read32(phy + 0x28),
                mmio::read32(phy + 0x2c),
            ));
            crate::log::usb_log_fmt(format_args!(
                "USB-PHY {} 30={:#010x} 3c={:#010x} 40={:#010x} 48={:#010x} 4c={:#010x} 50={:#010x}",
                tag,
                mmio::read32(phy + 0x30),
                mmio::read32(phy + 0x3c),
                mmio::read32(phy + 0x40),
                mmio::read32(phy + 0x48),
                mmio::read32(phy + 0x4c),
                mmio::read32(phy + 0x50),
            ));
        }
    }
}

// --- CV182x / SG2002 主机（Linux `dwc2_set_cv182x_params` + `dwc2_core_host_init`）---

#[cfg(feature = "cv182x-host")]
unsafe fn wait_grstctl_handshake(bit: u32, set: bool) -> UsbResult<()> {
    unsafe {
        for _ in 0..3_000_000u32 {
            let on = mmio::read32(base() + GRSTCTL) & bit != 0;
            if on == set {
                spin_delay(64);
                return Ok(());
            }
            spin_delay(8);
        }
        let label = if bit == GRSTCTL_RXFFLSH {
            "wait_grstctl RXFFLSH clr"
        } else if bit == GRSTCTL_TXFFLSH {
            "wait_grstctl TXFFLSH clr"
        } else {
            "wait_grstctl other"
        };
        dbg_dwc2_init_timeout(label);
        Err(UsbError::Timeout)
    }
}

#[cfg(feature = "cv182x-host")]
unsafe fn flush_rx_fifo_host() -> UsbResult<()> {
    unsafe {
        wait_ahb_idle()?;
        mmio::write32(base() + GRSTCTL, GRSTCTL_RXFFLSH);
        wait_grstctl_handshake(GRSTCTL_RXFFLSH, false)?;
        spin_delay(2_000);
        Ok(())
    }
}

#[cfg(feature = "cv182x-host")]
unsafe fn flush_tx_fifo_host_all() -> UsbResult<()> {
    unsafe {
        wait_ahb_idle()?;
        let greset = GRSTCTL_TXFFLSH | (0x10 << GRSTCTL_TXFNUM_SHIFT);
        mmio::write32(base() + GRSTCTL, greset);
        wait_grstctl_handshake(GRSTCTL_TXFFLSH, false)?;
        spin_delay(2_000);
        Ok(())
    }
}

#[cfg(feature = "cv182x-host")]
#[inline]
unsafe fn total_dfifo_depth_words() -> u32 {
    unsafe { (mmio::read32(base() + GHWCFG3) >> 16) & 0xffff }
}

#[cfg(feature = "cv182x-host")]
#[inline]
unsafe fn host_channel_count() -> u32 {
    unsafe {
        let v = mmio::read32(base() + GHWCFG2);
        1 + ((v >> GHWCFG2_NUM_HOST_CHAN_SHIFT) & GHWCFG2_NUM_HOST_CHAN_MASK)
    }
}

/// 动态 FIFO：优先采用设备树常用值；超出 `GHWCFG3` 总深度时按 Linux `dwc2_calculate_dynamic_fifo` 收缩。
#[cfg(feature = "cv182x-host")]
unsafe fn init_host_fifos_cv182x() -> UsbResult<()> {
    unsafe {
        let total = total_dfifo_depth_words();
        let hc = host_channel_count();
        let mut rx: u32 = 536;
        let mut nptx: u32 = 32;
        let mut ptx: u32 = 768;

        if rx.saturating_add(nptx).saturating_add(ptx) > total {
            rx = 516 + hc;
            nptx = 256;
            ptx = 768;
        }
        let sum = rx.saturating_add(nptx).saturating_add(ptx);
        if sum > total {
            ptx = total.saturating_sub(rx).saturating_sub(nptx);
        }

        mmio::write32(base() + GRXFSIZ, rx & 0xffff);

        let nptxfsiz = (nptx << 16) & 0xffff_0000 | (rx & 0xffff);
        mmio::write32(base() + GNPTXFSIZ, nptxfsiz);

        let hptxfsiz = (ptx << 16) & 0xffff_0000 | ((rx + nptx) & 0xffff);
        mmio::write32(base() + HPTXFSIZ, hptxfsiz);

        let snpsid = mmio::read32(base() + GSNPSID);
        let ded = mmio::read32(base() + GHWCFG4) & GHWCFG4_DED_FIFO_EN != 0;
        if ded && snpsid >= DWC2_CORE_REV_2_91A {
            let mut df = mmio::read32(base() + GDFIFOCFG);
            df &= !GDFIFOCFG_EPINFOBASE_MASK;
            let epbase = rx.wrapping_add(nptx).wrapping_add(ptx);
            df |= (epbase << GDFIFOCFG_EPINFOBASE_SHIFT) & GDFIFOCFG_EPINFOBASE_MASK;
            mmio::write32(base() + GDFIFOCFG, df);
        }

        Ok(())
    }
}

/// `dr_mode=otg` 时常用：使能 override 并置位 A-session / VBUS valid，否则根口可能无电气活动。
#[cfg(feature = "cv182x-host")]
unsafe fn init_gotgctl_otg_host_session_overrides() {
    unsafe {
        let mut g = mmio::read32(base() + GOTGCTL);
        g |= GOTGCTL_DBNCE_FLTR_BYPASS;
        g |= GOTGCTL_AVALOEN | GOTGCTL_AVALOVAL;
        g |= GOTGCTL_VBVALOEN | GOTGCTL_VBVALOVAL;
        mmio::write32(base() + GOTGCTL, g);
        spin_delay(200_000);
    }
}

/// UTMI 16-bit、HS 超时校准；保持 `FORCEHOSTMODE`（与 `force_host_mode()` 一致）。
#[cfg(feature = "cv182x-host")]
unsafe fn init_gusbcfg_cv182x_utmi16_hs() {
    unsafe {
        let mut v = mmio::read32(base() + GUSBCFG);
        v |= GUSBCFG_FORCEHOSTMODE;
        v &= !GUSBCFG_ULPI_UTMI_SEL;
        v |= GUSBCFG_PHYIF16;
        v &= !GUSBCFG_TOUTCAL_MASK;
        v |= GUSBCFG_TOUTCAL_MASK;
        mmio::write32(base() + GUSBCFG, v);
    }
}

#[cfg(feature = "cv182x-host")]
unsafe fn init_gahb_dma_cv182x() {
    unsafe {
        let g2 = mmio::read32(base() + GHWCFG2);
        let arch = (g2 >> 3) & 0x3;
        let mut v = mmio::read32(base() + GAHBCFG);
        v &= !(0xfu32 << GAHBCFG_HBSTLEN_SHIFT);
        v |= GAHBCFG_HBSTLEN_INCR16 << GAHBCFG_HBSTLEN_SHIFT;
        v |= GAHBCFG_GLBL_INTR_EN;
        if arch == 2 {
            v |= GAHBCFG_DMA_EN;
        }
        #[cfg(feature = "usb-force-no-dma")]
        {
            v &= !GAHBCFG_DMA_EN;
            crate::log::usb_log_fmt(format_args!(
                "USB-DBG usb-force-no-dma: DMA_EN cleared (ARCH={arch}, expect fail on int-DMA IP)"
            ));
        }
        mmio::write32(base() + GAHBCFG, v);
    }
}

/// CV182x 片内 USB2 PHY（与 Linux `usb@04340000` 第二段 `reg` 一致）。`REG014` 见 `dwc2/platform.c`。
#[cfg(feature = "cv182x-host")]
const CV182X_USB2_PHY_MMIO: usize = 0x0300_6000;
#[cfg(feature = "cv182x-host")]
const CV182X_PHY_REG014: usize = 0x014;
/// 与厂商 Linux `platform.c` host 路径对齐：**不设 `UTMI_OVERRIDE`**。
///
/// DWC2 在 host 模式下通过 UTMI 接口自行驱动 `dp_pulldown` / `dm_pulldown` 信号；
/// 若 `UTMI_OVERRIDE`=1，PHY 忽略 DWC2 的 UTMI 信号，可能干扰控制器的连接检测。
///
/// 写 `REG014=0` 将控制权还给 DWC2（vendor kernel host 路径不碰 `REG014`；
/// `utmi_chgdet_prepare`/`utmi_reset` 仅在 `CONFIG_USB_DWC2_PERIPHERAL` 充电检测里使用）。
#[cfg(feature = "cv182x-host")]
unsafe fn cv182x_usb2_phy_host_clear_utmi_override() {
    unsafe {
        let old = mmio::read32(CV182X_USB2_PHY_MMIO + CV182X_PHY_REG014);
        mmio::write32(CV182X_USB2_PHY_MMIO + CV182X_PHY_REG014, 0);
        spin_delay(200_000);
        let now = mmio::read32(CV182X_USB2_PHY_MMIO + CV182X_PHY_REG014);
        crate::log::usb_log_fmt(format_args!(
            "USB-DBG REG014 {:#06x}->{:#06x} (UTMI_OVERRIDE cleared, DWC2 drives pulldowns)",
            old, now
        ));
    }
}

/// Linux 在 `speed == HS` 时**不**置 `HCFG_FSLSSUPP`；RPi/全速演示才需要 FSLS。
#[cfg(feature = "cv182x-host")]
unsafe fn hcfg_clear_fs_ls_for_high_speed() {
    unsafe {
        let mut h = mmio::read32(base() + HCFG);
        h &= !HCFG_FSLSSUPP;
        h &= !HCFG_FSLSPCLKSEL_MASK;
        mmio::write32(base() + HCFG, h);
    }
}

/// M1：软复位、强制 Host、FIFO、GAHB、HCFG、根口上电（及 CV182x PHY 下拉）。
///
/// **不在此处** 发 USB 总线复位：应在确认 [`hprt_connsts`] 后调用 [`dwc2_host_root_bus_reset_pulse`]。
///
/// 成功返回 Ok，不保证已有设备连接；请读 [`dwc2_hprt0_read`] 的 `CONNSTS`。
pub fn dwc2_host_init() -> UsbResult<()> {
    unsafe {
        if base() == 0 {
            return Err(UsbError::Hardware("DWC2 base not set (call platform::set_dwc2_base_virt)"));
        }
        // 清中断状态，避免上电残留影响轮询（后续 M2 再细化掩码）。
        mmio::write32(base() + GINTMSK, 0);
        mmio::write32(base() + GINTSTS, 0xFFFFFFFF);

        core_soft_reset()?;
        force_host_mode()?;
        core_soft_reset()?;

        #[cfg(feature = "cv182x-host")]
        {
            // 先断言 OTG host session，再配 `GUSBCFG`（对齐 Linux DRD override + host init 思路）
            init_gotgctl_otg_host_session_overrides();
            // 对齐 Linux `dwc2_core_host_init` 顺序：`GUSBCFG`/`PCGCTL` → `GAHB`/`HCFG` → FIFO → flush
            init_gusbcfg_cv182x_utmi16_hs();
            mmio::write32(base() + PCGCTL, 0);
            init_gahb_dma_cv182x();
            hcfg_clear_fs_ls_for_high_speed();
            init_host_fifos_cv182x()?;
            flush_tx_fifo_host_all()?;
            flush_rx_fifo_host()?;
        }
        #[cfg(not(feature = "cv182x-host"))]
        {
            init_fifos();
            init_gahb();
            init_hcfg_fs_ls();
        }

        // DMA 模式下部分 DWC2 仍要求主机通道中断在顶层可见，否则 `HCINT` 可能不置位。
        mmio::write32(base() + HAINTMSK, (1 << 0) | (1 << 1));
        let mut gintmsk = mmio::read32(base() + GINTMSK);
        gintmsk |= GINTMSK_HCHINT;
        mmio::write32(base() + GINTMSK, gintmsk);

        mmio::write32(base() + GINTSTS, 0xFFFFFFFF);

        port_power_on();

        #[cfg(feature = "cv182x-host")]
        {
            cv182x_usb2_phy_host_clear_utmi_override();
            // FIFO flush / 端口上电后部分芯片会清 `GOTGCTL` 位域；再断言一次 A/VBUS override。
            init_gotgctl_otg_host_session_overrides();
        }

        Ok(())
    }
}
