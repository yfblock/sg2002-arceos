use axstd::{print, println};
use core::iter::Iterator;
use sg200x_bsp::gpio::{Direction, GPIOPin, GPIOPort, GPIO};

pub fn hexdump(data: &[u8], mut start_addr: usize) {
    const PRELAND_WIDTH: usize = 70;
    println!("{:-^1$}", " hexdump ", PRELAND_WIDTH);
    for offset in (0..data.len()).step_by(16) {
        print!("{:08x} ", start_addr);
        start_addr += 0x10;
        for i in 0..16 {
            if offset + i < data.len() {
                print!("{:02x} ", data[offset + i]);
            } else {
                print!("{:02} ", "");
            }
        }

        print!("{:>6}", ' ');

        for i in 0..16 {
            if offset + i < data.len() {
                let c = data[offset + i];
                if c >= 0x20 && c <= 0x7e {
                    print!("{}", c as char);
                } else {
                    print!(".");
                }
            } else {
                print!("{:02} ", "");
            }
        }

        println!("");
    }
    println!("{:-^1$}", " hexdump end ", PRELAND_WIDTH);
}

pub fn button_init() {
    let gpio = GPIO::new(GPIOPort::GPIO0);
    let gpio_in = GPIOPin::new(&gpio, 30);
    gpio_in.set_direction(Direction::Input);
    gpio_in.set_debounce(true);
}

pub fn wait_button_press() {
    let gpio = GPIO::new(GPIOPort::GPIO0);
    let gpio_in = GPIOPin::new(&gpio, 30);
    while gpio_in.is_high() {
        core::hint::spin_loop();
    }
}
