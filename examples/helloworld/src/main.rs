#![cfg_attr(feature = "axstd", no_std)]
#![cfg_attr(feature = "axstd", no_main)]

extern crate axplat_riscv64_sg2002;

#[macro_use]
extern crate axstd;

pub mod arm;
pub mod gc4653;
pub mod ssd1306;
pub mod sts3215;
pub mod utils;
pub mod camera;
pub mod pwm_demo; 
// pub mod wifi;

use crate::arm::arm_init;
use crate::arm::release;
use crate::camera::UartTransport;
use crate::utils::hexdump;
use core::time::Duration;

use arm::grab;
use axhal::asm::wait_for_irqs;
use axhal::mem::PhysAddr;
use axhal::time::wall_time;
use axstd::os::arceos::modules::axhal::mem::pa;
use axstd::os::arceos::modules::axhal::mem::phys_to_virt;
use axstd::os::arceos::modules::axhal::mem::va;
use axstd::os::arceos::modules::axhal::mem::virt_to_phys;
use axstd::os::arceos::modules::axlog::debug;
use axstd::vec::Vec;
#[cfg(feature = "axstd")]
use axstd::{print, println};
use sg200x_bsp::i2c::I2cInstance;
use sg200x_bsp::i2c::I2cSpeed;
use sg200x_bsp::pinmux::FMUX_IIC0_SCL;
use sg200x_bsp::pinmux::FMUX_IIC0_SDA;
use sg200x_bsp::pinmux::FMUX_JTAG_CPU_TCK;
use sg200x_bsp::pinmux::FMUX_JTAG_CPU_TMS;
use sg200x_bsp::pinmux::FMUX_SD1_CLK;
use sg200x_bsp::pinmux::FMUX_SD1_CMD;
use sg200x_bsp::pinmux::FMUX_SD1_D0;
use sg200x_bsp::pinmux::FMUX_SD1_D1;
use sg200x_bsp::pinmux::FMUX_SD1_D2;
use sg200x_bsp::pinmux::FMUX_UART0_RX;
use sg200x_bsp::pinmux::FMUX_UART0_TX;
use sg200x_bsp::pinmux::Pinmux;
use sg200x_bsp::pwm::PwmInstance;
use sg200x_bsp::pwm::PwmChannel;
use sg200x_bsp::pwm::PwmMode;
use sg200x_bsp::{
    gpio::{Direction, GPIOPort},
    sdmmc,
};
use synopsys_dw_uart::SynopsysUart;
use tock_registers::interfaces::Writeable;

const UART3_ADDR: PhysAddr = PhysAddr::from_usize(0x04170000);
struct Uart3;

impl UartTransport for Uart3 {
    fn write_all(&mut self, data: &[u8]) -> Result<(), camera::CameraError> {
        let mut uart3 = dw_apb_uart::DW8250::new(phys_to_virt(UART3_ADDR).as_usize());
        data.iter().for_each(|x| uart3.putchar(*x));
        Ok(())
    }

    fn read_bytes(&mut self, buf: &mut [u8], timeout_ms: u64) -> Result<usize, camera::CameraError> {
        let start = wall_time();
        let mut rlen = 0;
        let mut uart3 = dw_apb_uart::DW8250::new(phys_to_virt(UART3_ADDR).as_usize());
        loop {
            if rlen >= buf.len() {
                break;
            }
            if start + Duration::from_millis(timeout_ms) < wall_time() {
                break;
            }
            if let Some(v) = uart3.getchar() {
                buf[rlen] = v;
                rlen += 1;
            }
        }
        println!("read len: {:#x}", rlen);
        Ok(rlen)
        // let start = wall_time();
        // let mut rlen = 0;
        // let mut uart3 = dw_apb_uart::DW8250::new(phys_to_virt(UART3_ADDR).as_usize());
        // loop {
        //     if rlen >= buf.len() {
        //         break;
        //     }
        //     if start + Duration::from_millis(timeout_ms) < wall_time() && rlen == 0 {
        //         return Err(camera::CameraError::Timeout);
        //     }
        //     if let Some(v) = uart3.getchar() {
        //         buf[rlen] = v;
        //         rlen += 1;
        //         continue;
        //     } else if rlen > 0 {
        //         break;
        //     }
        // }
        // println!("read len: {:#x}", rlen);
        // Ok(rlen)
    }
}

#[cfg_attr(feature = "axstd", unsafe(no_mangle))]
fn main() {
    println!("Hello, world!");
    let pinmux = Pinmux::new();
    // wifi::init();

    unsafe {
        pinmux.fmux().sd1_d2.write(FMUX_SD1_D2::FSEL::UART3_TX);
        pinmux.fmux().sd1_d1.write(FMUX_SD1_D1::FSEL::UART3_RX);
    }

    let mut uart3 = dw_apb_uart::DW8250::new(phys_to_virt(UART3_ADDR).as_usize());
    // uart3.init_with_baud(2500000);
    uart3.init_with_baud(1500000);
    // uart3.init();
    println!("get cpr: {:#x}", uart3.cpr());
    println!("UART3 initialized");
    let mut cam = crate::camera::CameraProtocol::new_default(Uart3);
    println!("camera initialized");
    cam.ping().unwrap();
    println!("camera ping");
    let info = cam.get_camera_info().unwrap();
    println!("camera Info: {:#x?}", info);
    let frame = cam.get_frame().unwrap();

    println!("get Camera len: {:#x}", frame.len());
    while true {
        core::hint::spin_loop();
        unsafe {
            wait_for_irqs();
        }
    }

    // gc4653::init();
    unsafe {
        // Set Uart 2 PINMUX
        // phys_to_virt(pa!(0x03001070)).as_mut_ptr().write_volatile(0x2);
        // phys_to_virt(pa!(0x03001074)).as_mut_ptr().write_volatile(0x2);
        pinmux.fmux().iic0_sda.write(FMUX_IIC0_SDA::FSEL::UART2_RX);
        pinmux.fmux().iic0_scl.write(FMUX_IIC0_SCL::FSEL::UART2_TX);


        // Set PWM PinMUX
        pinmux.fmux().jtag_cpu_tms.write(FMUX_JTAG_CPU_TMS::FSEL::PWM_7);
        pinmux.fmux().jtag_cpu_tck.write(FMUX_JTAG_CPU_TCK::FSEL::PWM_6);
        // pinmux.fmux().sd1_cmd.write(FMUX_SD1_CMD::FSEL::PWM_8);
        // pinmux.fmux().sd1_clk.write(FMUX_SD1_CLK::FSEL::PWM_9);
        pinmux.fmux().uart0_tx.write(FMUX_UART0_TX::FSEL::PWM_4);
        pinmux.fmux().uart0_rx.write(FMUX_UART0_RX::FSEL::PWM_5);

        // Set Uart 3 PINMUX
        // phys_to_virt(pa!(0x030010D4)).as_mut_ptr().write_volatile(0x5);
        // phys_to_virt(pa!(0x030010D8)).as_mut_ptr().write_volatile(0x5);
        pinmux.fmux().sd1_d2.write(FMUX_SD1_D2::FSEL::UART3_TX);
        pinmux.fmux().sd1_d1.write(FMUX_SD1_D1::FSEL::UART3_RX);
    }

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
        pwm_chip1.configure_channel_raw(
            channel,
            10000,
            7000,
            sg200x_bsp::pwm::PwmPolarity::ActiveHigh,
        );
        //
        pwm_chip1.set_mode(channel, PwmMode::Continuous);
        // 使能 IO 输出
        pwm_chip1.enable_output(channel);
        // 启动 PWM 输出
        pwm_chip1.start(channel);
    }
    // let channel = PwmChannel::Channel2;
    // pwm_chip1.configure_channel_raw(
    //     channel,
    //     10000,
    //     5000,
    //     sg200x_bsp::pwm::PwmPolarity::ActiveHigh,
    // );
    // //
    // pwm_chip1.set_mode(channel, PwmMode::Continuous);
    // // 使能 IO 输出
    // pwm_chip1.enable_output(channel);
    // // 启动 PWM 输出
    // pwm_chip1.start(channel);

    // let mut pwm_chip2 = sg200x_bsp::pwm::Pwm::new(PwmInstance::Pwm2);
    // for i in 0..2 {
    //     let channel = PwmChannel::from_u8(i).unwrap();
    //     pwm_chip2.configure_channel_raw(
    //         channel,
    //         10000,
    //         000,
    //         sg200x_bsp::pwm::PwmPolarity::ActiveHigh,
    //     );
    //     //
    //     pwm_chip2.set_mode(channel, PwmMode::Continuous);
    //     // 使能 IO 输出
    //     pwm_chip2.enable_output(channel);
    //     // 启动 PWM 输出
    //     pwm_chip2.start(channel);
    // }
    // pwm_chip2.configure_channel_raw(
    //     channel,
    //     10000,
    //     000,
    //     sg200x_bsp::pwm::PwmPolarity::ActiveHigh,
    // );
    // //
    // pwm_chip2.set_mode(channel, PwmMode::Continuous);
    // // 使能 IO 输出
    // pwm_chip2.enable_output(channel);
    // // 启动 PWM 输出
    // pwm_chip2.start(channel);
    // pwm_chip2.disable_output(PwmChannel::Channel0);
    // pwm_chip2.stop(PwmChannel::Channel0);

    crate::arm::delay_ms(2000);

    pwm_chip1.disable_output(PwmChannel::Channel1);
    pwm_chip1.stop(PwmChannel::Channel1);
    pwm_chip1.disable_output(PwmChannel::Channel3);
    pwm_chip1.stop(PwmChannel::Channel3);


    grab();
    crate::arm::delay_ms(5000);
    release();
    while true {
        core::hint::spin_loop();
        wait_for_irqs();
    }

    let sdmmc = sg200x_bsp::sdmmc::init().unwrap();
    let mut buffer = [0; 512 * 6];
    sdmmc.clk_en(true);
    sdmmc.read_block(0, &mut buffer);
    // pwm7.configure_channel_raw(
    //     sg200x_bsp::pwm::PwmChannel::Channel3,
    //     1000,
    //     400,
    //     sg200x_bsp::pwm::PwmPolarity::ActiveHigh,
    // );
    // pwm7.restart(PwmChannel::Channel3);
    // sdmmc.read_block_sdma(0, virt_to_phys(va!(buffer.as_ptr() as usize)).into(), (buffer.len() / 512) as _);
    // sg200x_bsp::sdmmc::read_block(0, &mut buffer);
    hexdump(&buffer, 0x0);
    // sg200x_bsp::sdmmc::read_block(1, &mut buffer);
    // hexdump(&buffer, 0x200);
}
