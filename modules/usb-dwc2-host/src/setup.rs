//! USB SETUP 数据包（小端）。

/// `GET_DESCRIPTOR`（Device），`wLength` 为本次希望读回的字节数（常见先读 8）。
#[inline]
pub fn get_descriptor_device(w_length: u16) -> [u8; 8] {
    [
        0x80, // bmRequestType: Dir IN, Type Standard, Recipient Device
        6,    // GET_DESCRIPTOR
        0x00,
        0x01, // wValue: DEVICE (high) index 0 (low)
        0x00,
        0x00, // wIndex
        w_length as u8,
        (w_length >> 8) as u8,
    ]
}

/// `SET_ADDRESS`（`addr` 1..127）。
#[inline]
pub fn set_address(addr: u8) -> [u8; 8] {
    [
        0x00,
        5, // SET_ADDRESS
        addr,
        0,
        0,
        0,
        0,
        0,
    ]
}

/// `GET_CONFIGURATION`（返回 1 字节 `bConfigurationValue`）。
#[inline]
pub fn get_configuration() -> [u8; 8] {
    [
        0x80,
        8, // GET_CONFIGURATION
        0x00,
        0x00,
        0x00,
        0x00,
        0x01,
        0x00,
    ]
}

/// `SET_CONFIGURATION`。
#[inline]
pub fn set_configuration(cfg: u8) -> [u8; 8] {
    [
        0x00,
        9, // SET_CONFIGURATION
        cfg,
        0,
        0,
        0,
        0,
        0,
    ]
}

/// Hub：`SET_PORT_FEATURE`（`bmRequestType=0x23` class+other，`bRequest=SET_FEATURE`）。
#[inline]
pub fn hub_set_port_feature(port: u16, feature: u16) -> [u8; 8] {
    [
        0x23,
        0x03, // USB_REQ_SET_FEATURE
        feature as u8,
        (feature >> 8) as u8,
        port as u8,
        (port >> 8) as u8,
        0,
        0,
    ]
}

/// Hub 端口特性：`PORT_RESET`（USB 2.0 hub）。
pub const HUB_PORT_FEATURE_RESET: u16 = 4;

/// `USB_DT_CONFIGURATION`（`GET_DESCRIPTOR` 高字节）。
pub const USB_DT_CONFIGURATION: u8 = 2;
/// Hub 类描述符类型（`GET_DESCRIPTOR` 高字节）。
pub const USB_DT_HUB: u8 = 0x29;

/// `GET_DESCRIPTOR(CONFIGURATION, index, wLength)` — 已寻址设备。
#[inline]
pub fn get_descriptor_configuration(cfg_index: u8, w_length: u16) -> [u8; 8] {
    [
        0x80,
        6, // GET_DESCRIPTOR
        cfg_index,
        USB_DT_CONFIGURATION,
        0x00,
        0x00,
        w_length as u8,
        (w_length >> 8) as u8,
    ]
}

/// `GET_DESCRIPTOR(HUB)` — Hub 已配置后由 Hub 设备返回（`bmRequestType` Device+Class+IN）。
#[inline]
pub fn get_descriptor_hub(w_length: u16) -> [u8; 8] {
    [
        0xA0,
        6,
        0x00,
        USB_DT_HUB,
        0x00,
        0x00,
        w_length as u8,
        (w_length >> 8) as u8,
    ]
}

/// Hub：`GET_PORT_STATUS`（`bmRequestType=0xA3` Class+IN+Other，`bRequest=GET_STATUS`）。
#[inline]
pub fn hub_get_port_status(port: u16) -> [u8; 8] {
    [
        0xA3,
        0, // GET_STATUS
        0,
        0,
        port as u8,
        (port >> 8) as u8,
        4,
        0,
    ]
}

/// `GET_MAX_LUN`（`bmRequestType=0xA1`，`wLength`=1）。
#[inline]
pub fn get_max_lun(interface: u16) -> [u8; 8] {
    [
        0xA1,
        0xFE,
        0x00,
        0x00,
        interface as u8,
        (interface >> 8) as u8,
        0x01,
        0x00,
    ]
}

/// USB MSC `Bulk-Only Mass Storage Reset`（`bmRequestType=0x21`）。
#[inline]
#[allow(dead_code)]
pub fn mass_storage_reset(interface: u16) -> [u8; 8] {
    [
        0x21,
        0xFF, // Mass Storage Reset
        0x00,
        0x00,
        interface as u8,
        (interface >> 8) as u8,
        0x00,
        0x00,
    ]
}
