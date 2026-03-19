use axstd::boxed::Box;
use sg2002_wifi_driver::{SecurityType, WifiConnectParams, WifiDriver};

pub fn init() -> Result<(), Box<dyn core::error::Error>> {
    // 初始化驱动
    // let config = WifiConfig::default();
    let mut wifi = WifiDriver::new();
    wifi.init()?;

    // 启动软中断处理线程（可选，需要 multitask feature）
    // #[cfg(feature = "multitask")]
    // sg2002_wifi_driver::start_interrupt_thread();

    // 扫描网络
    let results = wifi.scan(10000)?;
    for result in results {
        println!("SSID: {:?}, RSSI: {}", result.ssid, result.rssi);
    }

    // 连接网络
    let params = WifiConnectParams::new(b"MyWiFi")
        .with_psk(b"password")
        .with_security(SecurityType::WPA2);
    wifi.connect_with_params(&params)?;
    Ok(())
}
