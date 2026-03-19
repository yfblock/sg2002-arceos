use crate::config::devices::UART_PADDR;
use axplat::console::ConsoleIf;
use axplat::mem::{pa, phys_to_virt};

struct ConsoleIfImpl;

#[impl_plat_interface]
impl ConsoleIf for ConsoleIfImpl {
    /// Writes bytes to the console from input u8 slice.
    fn write_bytes(bytes: &[u8]) {
        // let mut uart = dw_apb_uart::DW8250::new(phys_to_virt(pa!(UART_PADDR)).into());
        let mut uart = dw_apb_uart::DW8250::new(UART_PADDR);
        uart.set_ier(false);
        bytes.iter().for_each(|x| {
            if *x == b'\n' {
                uart.putchar(b'\r');
                uart.putchar(*x)
            } else {
                uart.putchar(*x)
            }
        });
    }

    /// Reads bytes from the console into the given mutable slice.
    /// Returns the number of bytes read.
    fn read_bytes(bytes: &mut [u8]) -> usize {
        todo!()
    }
}

use core::fmt::Write;
impl Write for ConsoleIfImpl {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Self::write_bytes(s.as_bytes());
        Ok(())
    }
}

#[doc(hidden)]
pub fn __print_impl(args: core::fmt::Arguments) {
    ConsoleIfImpl.write_fmt(args).unwrap();
}

macro_rules! print {
    ($($arg:tt)*) => {
        $crate::console::__print_impl(format_args!($($arg)*));
    }
}

macro_rules! println {
    () => { $crate::ax_print!("\n") };
    ($($arg:tt)*) => {
        $crate::console::__print_impl(format_args!("{}\n", format_args!($($arg)*)));
    }
}
