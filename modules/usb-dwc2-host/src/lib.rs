//! Synopsys DWC2 USB 主机（枚举、Hub 递归拓扑），与板级 MMIO 基址解耦。
//!
//! 使用前须调用 [`platform::set_dwc2_base_virt`] 与 [`log::set_usb_log_fn`]。

#![no_std]

pub mod cache;
pub mod dwc2;
pub mod dwc2_ep0;

pub use dwc2_ep0::debug_log_ep0_dma_info;
pub mod error;
pub mod host;
pub mod log;
pub mod mmio;
pub mod platform;
pub mod setup;
pub mod topology;
