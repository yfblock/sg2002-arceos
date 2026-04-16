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

use axhal::{
    asm::wait_for_irqs,
    mem::{PhysAddr, phys_to_virt},
};
use axstd::collections::vec_deque::VecDeque;
use axstd::println;
use axstd::sync::Mutex;
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
        axhal::irq::set_enable(47, false);
        let mut cache_buf = CAMERA_UART_BUF.lock();
        let n = cache_buf.len().min(buf.len());
        if n == 0 {
            drop(cache_buf);
            axhal::irq::set_enable(47, true);
            wait_for_irqs();
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
    println!("Starting HTTP server...");
    http_server::run();
}

mod http_server {
    use axstd::io::{self, prelude::*};
    use axstd::net::{TcpListener, TcpStream};
    use axstd::println;

    const LOCAL_PORT: u16 = 80;

    const RESPONSE: &[u8] = b"HTTP/1.1 200 OK\r\n\
Content-Type: text/html\r\n\
Content-Length: 569\r\n\
Connection: close\r\n\r\n\
<html>\
<head><title>Hello, SG2002 ArceOS</title>\
<style>\
body{font-family:sans-serif;background:#f0f4f8;margin:0}\
.c{max-width:600px;margin:80px auto;text-align:center}\
h1{color:#333}\
.info{background:#fff;border-radius:8px;padding:24px;box-shadow:0 2px 8px rgba(0,0,0,.1)}\
code{background:#eee;padding:2px 6px;border-radius:4px}\
</style></head>\
<body><div class=\"c\">\
<h1>Hello from SG2002!</h1>\
<div class=\"info\">\
<p>This page is served by <b>ArceOS</b> running on the <b>SG2002</b> RISC-V SoC.</p>\
<p>NIC driver: <code>cvitek-eth (DWMAC 3.70a)</code></p>\
</div></div></body></html>";

    fn handle(mut stream: TcpStream) -> io::Result<()> {
        let mut buf = [0u8; 1024];
        let _ = stream.read(&mut buf)?;
        stream.write_all(RESPONSE)?;
        Ok(())
    }

    pub fn run() {
        let listener = TcpListener::bind(("0.0.0.0", LOCAL_PORT)).expect("bind");
        println!("HTTP server listening on http://0.0.0.0:{}/", LOCAL_PORT);
        loop {
            match listener.accept() {
                Ok((stream, addr)) => {
                    println!("  client: {}", addr);
                    if let Err(e) = handle(stream) {
                        println!("  error: {:?}", e);
                    }
                }
                Err(e) => {
                    println!("accept error: {:?}", e);
                    break;
                }
            }
        }
    }
}
