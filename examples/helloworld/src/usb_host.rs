//! SG2002 / CV181x 系 DWC2 USB 主机：时钟、TOP 层 PHY/复位、DWC2 MMIO，再递归打印 Hub 拓扑。
//!
//! 参考：U-Boot `board/cvitek/mars/board.c`（`cvi_usb_hw_init`）、Linux `clk-cv181x.c`（USB 门控）。
//!
//! Linux `cv181x_base.dtsi` 中 `usb@04340000` 的 **`vbus-gpio = <&portb 6 0>`**：须拉高 **GPIOB6** 才能给 Type-A 口下游供电，否则 `HPRT0.CONNSTS` 恒为 0。

use axhal::mem::{phys_to_virt, virt_to_phys, PhysAddr, VirtAddr};
use sg200x_bsp::gpio::{Direction, GPIO, GPIOPort, GPIO1_BASE};
use sg200x_bsp::pinmux::{FMUX_USB_VBUS_DET, Pinmux};
use tock_registers::interfaces::Writeable;
use usb_dwc2_host::{host, log, platform};

fn ep0_dma_virt_to_phys(p: *const u8) -> u32 {
    virt_to_phys(VirtAddr::from(p as usize)).as_usize() as u32
}

/// Linux 设备树 `usb@04340000`（`cvitek,cv182x-usb`）。
const USB_DWC2_PADDR: usize = 0x0434_0000;
/// `clock-controller`（`cvitek,cv181x-clk`），见 `clk-cv181x.c` `REG_CLK_EN_*`。
const CLKGEN_PADDR: usize = 0x0300_2000;
/// `top_misc` / TOP 域，USB PHY 控制与软复位与 `mars_reg.h` 一致。
const TOP_PADDR: usize = 0x0300_0000;
/// IOBLK G1：`usb_vbus_det` 引脚电气配置（`ioblk.rs` offset 0x20）。
const IOBLK_G1_PADDR: usize = 0x0300_1800;
const IOBLK_G1_USB_VBUS_DET_OFF: usize = 0x020;

/// 与 `cv181x_base.dtsi` 的 `vbus-gpio = <&portb 6 0>` 一致：GPIO1 = Port B。
const VBUS_GPIO_PORT: GPIOPort = GPIOPort::GPIO1;
const VBUS_GPIO_PIN: u8 = 6;
/// 常见负载开关为高有效；若原理图为低有效开 VBUS，改为 `false`。
const VBUS_GPIO_ACTIVE_HIGH: bool = true;

/// FMUX → GPIO + IOBLK 加强驱动（弱驱动时 FET 可能拉不动）。
fn pinmux_usb_vbus_det_gpio_output_prep() {
    let pinmux = Pinmux::new();
    pinmux.fmux().usb_vbus_det.write(FMUX_USB_VBUS_DET::FSEL::XGPIOB_6);
    let iob = phys_to_virt(PhysAddr::from_usize(IOBLK_G1_PADDR)).as_usize();
    let r = (iob + IOBLK_G1_USB_VBUS_DET_OFF) as *mut u32;
    unsafe {
        let v = core::ptr::read_volatile(r);
        // DS0|DS1|DS2：较大输出电流，便于驱动 VBUS 开关栅极
        core::ptr::write_volatile(r, v | (7 << 5));
    }
}

/// 打开 USB 口 VBUS 电源开关（板级若未接此 GPIO，可整段注释掉并自行供电）。
fn enable_usb_vbus_gpio_cv181x_dtsi() {
    let gpio_va = phys_to_virt(PhysAddr::from_usize(GPIO1_BASE)).as_usize();
    let gpio = unsafe { GPIO::from_base_address(gpio_va, VBUS_GPIO_PORT) };
    gpio.set_direction(VBUS_GPIO_PIN, Direction::Output);
    gpio.set(VBUS_GPIO_PIN, VBUS_GPIO_ACTIVE_HIGH);
}

fn usb_log_line(s: &str) {
    println!("{s}");
}

#[inline]
fn spin_udelay_approx(us: u32) {
    for _ in 0..us.saturating_mul(64) {
        core::hint::spin_loop();
    }
}

/// 打开 `clk_axi4_usb` … `clk_12m_usb`（`CLK_EN_1` bit28–31 + `CLK_EN_2` bit0）。
unsafe fn enable_usb_clocks_cv181x() {
    let b = phys_to_virt(PhysAddr::from_usize(CLKGEN_PADDR)).as_usize();
    let en1 = (b + 0x004) as *mut u32;
    let en2 = (b + 0x008) as *mut u32;
    let v1 = core::ptr::read_volatile(en1);
    let v2 = core::ptr::read_volatile(en2);
    core::ptr::write_volatile(en1, v1 | (0xFu32 << 28));
    core::ptr::write_volatile(en2, v2 | 1u32);
}

/// USB 软复位脉冲 + TOP `0x48`：对齐 Linux `dwc2_set_hw_id(..., is_dev=0)` 与 U-Boot `EXTVBUS`。
/// 返回 `TOP+0x48` 写后回读，便于串口核对是否与 Linux host+EXTVBUS 预期一致。
///
/// 关键：上游 Linux PHY 驱动 `phy-cv1800-usb.c` 指出——
/// > phy needs to change mode twice after initialization, otherwise
/// > the controller can not found devices attached to the phy.
///
/// 即 `TOP+0x48` bit[7:6] 必须先写 **device（0xC0）** 再写 **host（0x40）**。
unsafe fn cvitek_usb_top_host_bringup() -> u32 {
    let top = phys_to_virt(PhysAddr::from_usize(TOP_PADDR)).as_usize();
    let rst = (top + 0x3000) as *mut u32;
    let v = core::ptr::read_volatile(rst);
    core::ptr::write_volatile(rst, v & !(1 << 11));
    spin_udelay_approx(50);
    core::ptr::write_volatile(rst, v | (1 << 11));
    spin_udelay_approx(50);

    let usb_pin = (top + 0x48) as *mut u32;
    let x = core::ptr::read_volatile(usb_pin);

    // --- PHY ID pad toggle（Silicon workaround，见 phy-cv1800-usb.c probe()）---
    // 第 1 步：device 模式 override (bit7=1, bit6=1)
    let dev_mode = (x & !0xC0u32) | 0xC0u32 | 0x01u32;
    core::ptr::write_volatile(usb_pin, dev_mode);
    spin_udelay_approx(1_000);
    // 第 2 步：host 模式 override (bit7=0, bit6=1)
    let host_mode = (x & !0xC0u32) | 0x40u32 | 0x01u32;
    core::ptr::write_volatile(usb_pin, host_mode);
    spin_udelay_approx(1_000);

    let eco = (top + 0xB4) as *mut u32;
    core::ptr::write_volatile(eco, core::ptr::read_volatile(eco) | 0x80);
    core::ptr::read_volatile(usb_pin)
}

/// 探测 DWC2、初始化主机并打印 Hub/设备树（无 Mass Storage 亦可成功返回）。
pub fn init_and_dump_topology() {
    let top48 = unsafe {
        enable_usb_clocks_cv181x();
        cvitek_usb_top_host_bringup()
    };
    println!(
        "USB: TOP+0x48={top48:#010x} (Linux host: bit6; U-Boot EXTVBUS: bit0; 写入掩码后常含 0x41)"
    );
    pinmux_usb_vbus_det_gpio_output_prep();
    println!(
        "USB: FMUX+IOBLK usb_vbus_det -> XGPIOB[{}], DS=max",
        VBUS_GPIO_PIN
    );
    enable_usb_vbus_gpio_cv181x_dtsi();
    {
        let gpio_va = phys_to_virt(PhysAddr::from_usize(GPIO1_BASE)).as_usize();
        let gpio = unsafe { GPIO::from_base_address(gpio_va, VBUS_GPIO_PORT) };
        println!(
            "USB: VBUS GPIO1 pin {} drive {} (read_pin={})",
            VBUS_GPIO_PIN,
            if VBUS_GPIO_ACTIVE_HIGH { "high" } else { "low" },
            gpio.read(VBUS_GPIO_PIN)
        );
    }
    spin_udelay_approx(100_000);

    let vbase = phys_to_virt(PhysAddr::from_usize(USB_DWC2_PADDR)).as_usize();
    platform::set_dwc2_base_virt(vbase);
    platform::set_usb_dma_to_phys_fn(Some(ep0_dma_virt_to_phys));
    log::set_usb_log_fn(usb_log_line);
    usb_dwc2_host::debug_log_ep0_dma_info();

    match unsafe { usb_dwc2_host::dwc2::dwc2_probe() } {
        Ok((g1, g2, g3)) => {
            println!("USB DWC2 GHWCFG1={g1:#010x} GHWCFG2={g2:#010x} GHWCFG3={g3:#010x}");
        }
        Err(e) => {
            println!("USB DWC2 probe failed: {e:?}");
            return;
        }
    }

    match host::enumerate_topology_only() {
        Ok(()) => {}
        Err(e) => println!("USB topology enumeration: {e:?}"),
    }
}
