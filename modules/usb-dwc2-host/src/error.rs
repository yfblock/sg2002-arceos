//! USB 栈统一错误类型（随 M2/M3 逐步填充变体）。

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsbError {
    NotImplemented,
    Timeout,
    Hardware(&'static str),
    Protocol(&'static str),
    Stall,
}

pub type UsbResult<T> = Result<T, UsbError>;
