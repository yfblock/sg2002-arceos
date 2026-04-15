#![no_std]
#![no_main]

extern crate axplat_riscv64_sg2002;

#[macro_use]
extern crate axstd;

pub mod arm;
pub mod camera;

/// `arm` 等子模块通过 `use crate::pa` 使用 `pa!`。
pub use axstd::os::arceos::modules::axhal::mem::pa;

pub mod pwm_demo;
pub mod ssd1306;
pub mod sts3215;
pub mod utils;
mod dma_camera;
mod dma_uart_tx;
mod uart3_dma_rx;
mod usb_host;

use crate::arm::{arm_init, grab, release};
use crate::camera::UartTransport;
use crate::utils::hexdump;
use core::time::Duration;

use axhal::{
    asm::wait_for_irqs,
    mem::{PhysAddr, phys_to_virt},
};
use axstd::collections::vec_deque::VecDeque;
use axstd::println;
use axstd::sync::Mutex;
use axstd::thread::sleep;
use sg200x_bsp::{
    pinmux::{
        FMUX_IIC0_SCL, FMUX_IIC0_SDA, FMUX_JTAG_CPU_TCK, FMUX_JTAG_CPU_TMS, FMUX_SD1_D1,
        FMUX_SD1_D2, FMUX_UART0_RX, FMUX_UART0_TX, Pinmux,
    },
    pwm::{PwmChannel, PwmInstance, PwmMode},
};
use tock_registers::interfaces::Writeable;

static CAMERA_UART_BUF: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());
const UART3_ADDR: PhysAddr = PhysAddr::from_usize(0x04170000);

struct Uart3;

impl UartTransport for Uart3 {
    fn write_all(&mut self, data: &[u8]) -> Result<(), camera::CameraError> {
        let mut uart3 = dw_apb_uart::DW8250::new(phys_to_virt(UART3_ADDR).as_usize());
        data.iter().for_each(|x| uart3.putchar(*x));
        Ok(())
    }

    fn read_bytes(
        &mut self,
        buf: &mut [u8],
        _timeout_ms: u64,
    ) -> Result<usize, camera::CameraError> {
        sleep(Duration::from_millis(3));
        axhal::irq::set_enable(47, false);
        let mut cache_buf = CAMERA_UART_BUF.lock();
        let n = cache_buf.len().min(buf.len());
        if n == 0 {
            drop(cache_buf);
            sleep(Duration::from_millis(1));
            return Ok(0);
        }
        cache_buf.drain(..n).enumerate().for_each(|(i, x)| buf[i] = x);
        drop(cache_buf);
        axhal::irq::set_enable(47, true);
        Ok(n)
    }
}

/// 原先 `main` 里的 USB 主机与 DMA→UART 演示，保留供参考，不参与启动路径。
#[allow(dead_code)]
fn unused_usb_host_and_dma_uart_demo() {
    usb_host::init_and_dump_topology();
    dma_uart_tx::run_demo();
}

/// 原先 `main` 里的机械臂、PWM、按键、SDMMC 等演示，保留供参考，不参与启动路径。
#[allow(dead_code)]
fn unused_arm_pwm_sdmmc_demo() {
    let pinmux = Pinmux::new();

    pinmux.fmux().iic0_sda.write(FMUX_IIC0_SDA::FSEL::UART2_RX);
    pinmux.fmux().iic0_scl.write(FMUX_IIC0_SCL::FSEL::UART2_TX);

    pinmux
        .fmux()
        .jtag_cpu_tms
        .write(FMUX_JTAG_CPU_TMS::FSEL::PWM_7);
    pinmux
        .fmux()
        .jtag_cpu_tck
        .write(FMUX_JTAG_CPU_TCK::FSEL::PWM_6);
    pinmux.fmux().uart0_tx.write(FMUX_UART0_TX::FSEL::PWM_4);
    pinmux.fmux().uart0_rx.write(FMUX_UART0_RX::FSEL::PWM_5);

    pinmux.fmux().sd1_d2.write(FMUX_SD1_D2::FSEL::UART3_TX);
    pinmux.fmux().sd1_d1.write(FMUX_SD1_D1::FSEL::UART3_RX);

    arm_init();

    crate::arm::move_to_position(1, 2600);
    crate::arm::move_to_position(2, 2500);
    release();

    utils::button_init();
    utils::wait_button_press();

    let mut pwm_chip1 = sg200x_bsp::pwm::Pwm::new(PwmInstance::Pwm1);
    for i in 0..4 {
        if i % 2 == 0 {
            continue;
        }
        let channel = PwmChannel::from_u8(i).unwrap();
        pwm_chip1
            .configure_channel_raw(
                channel,
                10000,
                7000,
                sg200x_bsp::pwm::PwmPolarity::ActiveHigh,
            )
            .unwrap();
        pwm_chip1.set_mode(channel, PwmMode::Continuous);
        pwm_chip1.enable_output(channel);
        pwm_chip1.start(channel);
    }

    crate::arm::delay_ms(2000);

    pwm_chip1.disable_output(PwmChannel::Channel1);
    pwm_chip1.stop(PwmChannel::Channel1);
    pwm_chip1.disable_output(PwmChannel::Channel3);
    pwm_chip1.stop(PwmChannel::Channel3);

    grab();
    crate::arm::delay_ms(5000);
    release();

    let sdmmc = sg200x_bsp::sdmmc::init().unwrap();
    let mut buffer = [0; 512 * 6];
    sdmmc.clk_en(true);
    sdmmc.read_block(0, &mut buffer).unwrap();
    hexdump(&buffer, 0x0);
}

#[unsafe(no_mangle)]
fn main() {
    println!("Hello, world!");

    let pinmux = Pinmux::new();
    pinmux.fmux().sd1_d2.write(FMUX_SD1_D2::FSEL::UART3_TX);
    pinmux.fmux().sd1_d1.write(FMUX_SD1_D1::FSEL::UART3_RX);

    let mut uart0 = dw_apb_uart::DW8250::new(phys_to_virt(pa!(0x04140000)).as_usize());
    uart0.set_ier(true);

    axhal::irq::register(47, || {
        let mut uart3 = dw_apb_uart::DW8250::new(phys_to_virt(UART3_ADDR).as_usize());
        let mut buf = CAMERA_UART_BUF.lock();
        loop {
            if let Some(c) = uart3.getchar() {
                buf.push_back(c);
                continue;
            }
            break;
        }
        uart3.set_ier(true);
    });
    axhal::irq::set_enable(47, true);

    let mut uart3 = dw_apb_uart::DW8250::new(phys_to_virt(UART3_ADDR).as_usize());
    uart3.init_with_baud(1500000);
    uart3.set_ier(true);
    println!("get cpr: {:#x}", uart3.cpr());
    println!("UART3 initialized");
    let mut cam = crate::camera::CameraProtocol::new_default(Uart3);
    println!("camera initialized");
    cam.ping().unwrap();
    println!("camera ping");
    let info = cam.get_camera_info().unwrap();
    println!("camera Info: {:#x?}", info);
    let t0 = axstd::time::Instant::now();
    let frame = cam.get_frame().unwrap();
    let elapsed = t0.elapsed();

    println!(
        "get Camera len: {:#x} ({} bytes), time: {} ms",
        frame.len(), frame.len(), elapsed.as_millis()
    );
    loop {
        core::hint::spin_loop();
        wait_for_irqs();
    }
}
