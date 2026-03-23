use axstd::ptr;
use sg200x_bsp::gpio::{Direction, GPIOPin, GPIOPort};
use sg200x_bsp::i2c::{I2c, I2cInstance};
use sg200x_bsp::mipirx::{HdrMode, LaneMode, MipiRxDevAttr, RawDataType, SensorMode};
use sg200x_bsp::mipirx::{MipiRx, MipiRxCsi};
use sg200x_bsp::pinmux::Pinmux;
use tock_registers::interfaces::Readable;

const SNSR_I2C_ADDR: u8 = 0x29;
const GC4653_CHIP_ID_ADDR_H: u16 = 0x03f0;
const GC4653_CHIP_ID_ADDR_L: u16 = 0x03f1;

// pub static I2C4: LazyLock<Mutex<I2c>> = LazyLock::new(|| Mutex::new(I2c::new(I2cInstance::I2c4)));

// struct Clock {
//     sync_set: u32,
//     pre_div_sel: u32,
//     post_div_sel: u32,
//     sel_mode: u32,
//     div_sel: u32,
//     ictrl: u32,
// }

// const clk: Clock = Clock {
//     sync_set: todo!(),
//     pre_div_sel: todo!(),
//     post_div_sel: todo!(),
//     sel_mode: todo!(),
//     div_sel: todo!(),
//     ictrl: todo!(),
// };

pub fn enable_clk() {
    unsafe {
        let r = 0x030028C4 as *mut u32;
        let mut v = r.read_volatile();
        v &= !((0x3 << 8) | (0x3f) << 16);
        v |= (2 << 8) | (33 << 16);
        r.write_volatile(v);
    }
}

pub fn init() {
    let pinmux = Pinmux::new();
    pinmux.set_camera();
    // RTCSYS_GPIO(PWR_GPIO1)
    let rst_pin = GPIOPin::with_port(GPIOPort::RTCSysGPIO, 1);
    rst_pin.set_direction(Direction::Output);
    // https://github.com/sipeed/LicheeRV-Nano-Build/blob/d4003f15b35d43ad4842f427050ab2bba0114fa5/osdrv/interdrv/v2/cif/chip/mars/cif.c#L2349C12-L2349C31
    rst_pin.set(true);

    // CAMPLL_FREQ_24M
    let clk_div = 33;
    // 使用 unsafe 块包裹所有硬件操作
    unsafe {
        // 1. 设置 0x03002800 的第 16 位
        mmio_set_bit(0x03002800 as *mut u32, 16);

        // udelay(100);

        // 2. 清除 0x03002030 的第 29 位 (enable camp0pll clk source)
        mmio_clear_bit(0x03002030 as *mut u32, 29);

        // 3. 写入分频系数到 0x030020F8
        let div_val = (clk_div << 16) | 0x09;
        mmio_write(0x030020F8 as *mut u32, div_val);

        // 4. 设置 0x03002840 的第 5 位 (set sync source en)
        mmio_set_bit(0x03002840 as *mut u32, 5);
    }

    // 如果开启 pll
    unsafe {
        // 1. 写入 Sync Set
        // mmio_write(0x03002884 as *mut u32, clk.sync_set);
        mmio_write(0x03002884 as *mut u32, 0x24AAAAAB);

        // 2. 翻转 Sync SW 的第 0 位 (value ^= 0x01)
        mmio_toggle_bit(0x03002880 as *mut u32, 0);

        // 3. 组合并写入 CSR 寄存器
        // let csr_value = clk.pre_div_sel
        //     | (clk.post_div_sel << 8)
        //     | (clk.sel_mode << 15)
        //     | (clk.div_sel << 17)
        //     | (clk.ictrl << 24);
        // mmio_write(0x03002818, csr_value);
        // /* set csr */
        // value = clk->pre_div_sel |
        //     (clk->post_div_sel << 8) |
        //     (clk->sel_mode << 15) |
        //     (clk->div_sel << 17) |
        //     (clk->ictrl << 24);
        // iowrite32(value, ioremap(0x03002818, 0x4));
        mmio_write(0x03002818 as *mut u32, 0x00168101);

        // 4. 清除 PWD 的第 12 位 (value &= ~(1<<16))
        mmio_clear_bit(0x03002800 as *mut u32, 16);

        // 5. 延时
        // udelay(100);
    }

    let ref mut buf = [0u8; 2];
    let mut i2c = I2c::new(I2cInstance::I2c4);
    i2c.init(sg200x_bsp::i2c::I2cSpeed::Fast);
    i2c.read(SNSR_I2C_ADDR, &mut buf[..1]).unwrap();
    println!("{:#x?}", GC4653_CHIP_ID_ADDR_H.to_be_bytes());
    // i2c.write(SNSR_I2C_ADDR,   &GC4653_CHIP_ID_ADDR_H.to_be_bytes()).unwrap();
    i2c.write_read(
        SNSR_I2C_ADDR,
        &GC4653_CHIP_ID_ADDR_H.to_be_bytes(),
        &mut buf[..1],
    )
    .unwrap();
    i2c.write_read(
        SNSR_I2C_ADDR,
        &GC4653_CHIP_ID_ADDR_L.to_be_bytes(),
        &mut buf[1..2],
    )
    .unwrap();
    let sensor_id: u16 = u16::from_be_bytes(buf[..2].try_into().unwrap());
    println!("session id: {}", sensor_id);
    linear_1440p30_init();
    write_default_regs();
    let mut mipirx = unsafe { MipiRx::new() };
    mipirx.reset(0).unwrap();
    // 配置设备属性
    let attr = MipiRxDevAttr {
        devno: 0,
        sensor_mode: SensorMode::Csi,
        lane_mode: LaneMode::Lane4,
        data_type: RawDataType::Raw10,
        hdr_mode: HdrMode::None,
        lane_id: [0, 1, 2, 3],
        clk_lane_sel: 0,
        pn_swap: [false; 5],
        img_width: 1920,
        img_height: 1080,
    };

    mipirx.configure(&attr).unwrap();

    // 使能接收
    mipirx.enable(0).unwrap();

    // MAC TOP:
    let mipirxcsi = MipiRxCsi::new(0).unwrap();
    println!("mipirx csi: {:#x?}", mipirxcsi.regs.reg_00.get());
    println!("mipirx csi reg14: {:#x?}", mipirxcsi.regs.reg_14.get());
    println!("mipirx csi reg18: {:#x?}", mipirxcsi.regs.reg_18.get());
    println!("mipirx csi reg40: {:#x?}", mipirxcsi.regs.reg_40.get());

    let addr = 0x0A0C2000 as *mut u32;
    let addr1 = 0x0A0C4000 as *mut u32;
    unsafe {
        addr.write_volatile(addr.read_volatile() | (1 << 4) | 0x1);
        println!("value: {:#x}", addr.read_volatile());
        println!("value1: {:#x}", addr1.read_volatile());
    }
    loop {
        // println!("GC4653 init start: {:#x?}", mipirx.get_interrupt_status(0));
    }
    println!("sensor id: {:#x}", sensor_id);
    // let vi = sg200x_bsp::vi::Vi::new(sg200x_bsp::vi::ViDevno::Vi2);
    // 配置文件: /home/yfblock/Code/chenloong/LicheeRV-Nano-Build/osdrv/interdrv/v2/cif/chip/mars/drv/cif_drv.c
}

pub fn gc4653_write_register(reg_addr: u16, val: u8) {
    let mut buf = [0u8; 3];
    buf[..2].copy_from_slice(&reg_addr.to_be_bytes());
    buf[2] = val;
    let i2c = I2c::new(I2cInstance::I2c4);
    i2c.write(SNSR_I2C_ADDR, &buf).unwrap();
}

pub fn write_default_regs() {
    gc4653_write_register(0x202, 0x0);
    gc4653_write_register(0x203, 0x5D);
    gc4653_write_register(0x2B3, 0x0);
    gc4653_write_register(0x2B4, 0x0);
    gc4653_write_register(0x2B8, 0x1);
    gc4653_write_register(0x2B9, 0x0);
    gc4653_write_register(0x515, 0x30);
    gc4653_write_register(0x519, 0x1E);
    gc4653_write_register(0x2D9, 0x5C);
    gc4653_write_register(0x20E, 0x1);
    gc4653_write_register(0x20F, 0x0);
    gc4653_write_register(0x340, 0x7);
    gc4653_write_register(0x341, 0x8);
    gc4653_write_register(0x31D, 0x2D);
    gc4653_write_register(0x101, 0x0);
    gc4653_write_register(0x31D, 0x28);
}

pub fn linear_1440p30_init() {
    gc4653_write_register(0x03fe, 0xf0);
    gc4653_write_register(0x03fe, 0x00);
    gc4653_write_register(0x0317, 0x00);
    gc4653_write_register(0x0320, 0x77);
    gc4653_write_register(0x0324, 0xc8);
    gc4653_write_register(0x0325, 0x06);
    gc4653_write_register(0x0326, 0x6c);
    gc4653_write_register(0x0327, 0x03);
    gc4653_write_register(0x0334, 0x40);
    gc4653_write_register(0x0336, 0x6c);
    gc4653_write_register(0x0337, 0x82);
    gc4653_write_register(0x0315, 0x25);
    gc4653_write_register(0x031c, 0xc6);
    gc4653_write_register(0x0287, 0x18);
    gc4653_write_register(0x0084, 0x00);
    gc4653_write_register(0x0087, 0x50);
    gc4653_write_register(0x029d, 0x08);
    gc4653_write_register(0x0290, 0x00);
    gc4653_write_register(0x0340, 0x05);
    gc4653_write_register(0x0341, 0xdc);
    gc4653_write_register(0x0345, 0x06);
    gc4653_write_register(0x034b, 0xb0);

    gc4653_write_register(0x0352, 0x06);
    gc4653_write_register(0x0354, 0x07);
    gc4653_write_register(0x02d1, 0xe0);
    gc4653_write_register(0x0223, 0xf2);
    gc4653_write_register(0x0238, 0xa4);
    gc4653_write_register(0x02ce, 0x7f);
    gc4653_write_register(0x0232, 0xc4);
    gc4653_write_register(0x02d3, 0x05);
    gc4653_write_register(0x0243, 0x06);
    gc4653_write_register(0x02ee, 0x30);
    gc4653_write_register(0x026f, 0x70);
    gc4653_write_register(0x0257, 0x09);
    gc4653_write_register(0x0211, 0x02);
    gc4653_write_register(0x0219, 0x09);
    gc4653_write_register(0x023f, 0x2d);
    gc4653_write_register(0x0518, 0x00);
    gc4653_write_register(0x0519, 0x1e);
    gc4653_write_register(0x0515, 0x30);
    gc4653_write_register(0x02d9, 0x5c);
    gc4653_write_register(0x02da, 0x02);
    gc4653_write_register(0x02db, 0xe8);
    gc4653_write_register(0x02e6, 0x20);
    gc4653_write_register(0x021b, 0x10);
    gc4653_write_register(0x0252, 0x22);
    gc4653_write_register(0x024e, 0x22);
    gc4653_write_register(0x02c4, 0x01);
    gc4653_write_register(0x021d, 0x17);
    gc4653_write_register(0x024a, 0x01);
    gc4653_write_register(0x02ca, 0x02);
    gc4653_write_register(0x0262, 0x10);
    gc4653_write_register(0x029a, 0x20);
    gc4653_write_register(0x021c, 0x0e);
    gc4653_write_register(0x0298, 0x03);
    gc4653_write_register(0x029c, 0x00);
    gc4653_write_register(0x027e, 0x14);
    gc4653_write_register(0x02c2, 0x10);
    gc4653_write_register(0x0540, 0x20);
    gc4653_write_register(0x0546, 0x01);
    gc4653_write_register(0x0548, 0x01);
    gc4653_write_register(0x0544, 0x01);
    gc4653_write_register(0x0242, 0x1b);
    gc4653_write_register(0x02c0, 0x1b);
    gc4653_write_register(0x02c3, 0x20);
    gc4653_write_register(0x02e4, 0x10);
    gc4653_write_register(0x022e, 0x00);
    gc4653_write_register(0x027b, 0x3f);
    gc4653_write_register(0x0269, 0x0f);
    gc4653_write_register(0x02d2, 0x40);
    gc4653_write_register(0x027c, 0x08);
    gc4653_write_register(0x023a, 0x2e);
    gc4653_write_register(0x0245, 0xce);
    gc4653_write_register(0x0530, 0x20);
    gc4653_write_register(0x0531, 0x02);
    gc4653_write_register(0x0228, 0x50);
    gc4653_write_register(0x02ab, 0x00);
    gc4653_write_register(0x0250, 0x00);
    gc4653_write_register(0x0221, 0x50);
    gc4653_write_register(0x02ac, 0x00);
    gc4653_write_register(0x02a5, 0x02);
    gc4653_write_register(0x0260, 0x0b);
    gc4653_write_register(0x0216, 0x04);
    gc4653_write_register(0x0299, 0x1C);
    gc4653_write_register(0x02bb, 0x0d);
    gc4653_write_register(0x02a3, 0x02);
    gc4653_write_register(0x02a4, 0x02);
    gc4653_write_register(0x021e, 0x02);
    gc4653_write_register(0x024f, 0x08);
    gc4653_write_register(0x028c, 0x08);
    gc4653_write_register(0x0532, 0x3f);
    gc4653_write_register(0x0533, 0x02);
    gc4653_write_register(0x0277, 0xc0);
    gc4653_write_register(0x0276, 0xc0);
    gc4653_write_register(0x0239, 0xc0);
    gc4653_write_register(0x0202, 0x05);
    gc4653_write_register(0x0203, 0xd0);
    gc4653_write_register(0x0205, 0xc0);
    gc4653_write_register(0x02b0, 0x68);
    gc4653_write_register(0x0002, 0xa9);
    gc4653_write_register(0x0004, 0x01);
    gc4653_write_register(0x021a, 0x98);
    gc4653_write_register(0x0266, 0xa0);
    gc4653_write_register(0x0020, 0x01);
    gc4653_write_register(0x0021, 0x03);
    gc4653_write_register(0x0022, 0x00);
    gc4653_write_register(0x0023, 0x04);

    gc4653_write_register(0x0342, 0x06);
    gc4653_write_register(0x0343, 0x40);
    gc4653_write_register(0x03fe, 0x10);
    gc4653_write_register(0x03fe, 0x00);
    gc4653_write_register(0x0106, 0x78);
    gc4653_write_register(0x0108, 0x0c);
    gc4653_write_register(0x0114, 0x01);
    gc4653_write_register(0x0115, 0x10); //0x12 coninute mode, 0x10 no-continue mode
    gc4653_write_register(0x0180, 0x46);
    gc4653_write_register(0x0181, 0x30);
    gc4653_write_register(0x0182, 0x05);
    gc4653_write_register(0x0185, 0x01);
    gc4653_write_register(0x03fe, 0x10);
    gc4653_write_register(0x03fe, 0x00);
    gc4653_write_register(0x000f, 0x00);
    gc4653_write_register(0x0100, 0x09); //stream on
    //otp
    gc4653_write_register(0x0080, 0x02);
    gc4653_write_register(0x0097, 0x0a);
    gc4653_write_register(0x0098, 0x10);
    gc4653_write_register(0x0099, 0x05);
    gc4653_write_register(0x009a, 0xb0);
    gc4653_write_register(0x0317, 0x08);
    gc4653_write_register(0x0a67, 0x80);
    gc4653_write_register(0x0a70, 0x03);
    gc4653_write_register(0x0a82, 0x00);
    gc4653_write_register(0x0a83, 0x10);
    gc4653_write_register(0x0a80, 0x2b);
    gc4653_write_register(0x05be, 0x00);
    gc4653_write_register(0x05a9, 0x01);
    gc4653_write_register(0x0313, 0x80);
    gc4653_write_register(0x05be, 0x01);
    gc4653_write_register(0x0317, 0x00);
    gc4653_write_register(0x0a67, 0x00);

    gc4653_write_register(0x0000, 0x00); // end flag
}

pub fn standby() {
    gc4653_write_register(0x0100, 0x00);
    gc4653_write_register(0x031c, 0xc7);
    gc4653_write_register(0x0317, 0x01);
}

pub fn restart() {
    gc4653_write_register(0x0317, 0x00);
    gc4653_write_register(0x031c, 0xc6);
    gc4653_write_register(0x0100, 0x09);
}

// 辅助函数：封装 unsafe 操作
unsafe fn mmio_set_bit(addr: *mut u32, bit: u8) {
    unsafe {
        let val = ptr::read_volatile(addr);
        ptr::write_volatile(addr, val | (1 << bit));
    }
}

unsafe fn mmio_clear_bit(addr: *mut u32, bit: u8) {
    unsafe {
        let val = ptr::read_volatile(addr);
        ptr::write_volatile(addr, val & !(1 << bit));
    }
}

unsafe fn mmio_write(addr: *mut u32, val: u32) {
    unsafe {
        ptr::write_volatile(addr, val);
    }
}

unsafe fn mmio_toggle_bit(addr: *mut u32, bit: u8) {
    unsafe {
        let mut val = ptr::read_volatile(addr);
        val = val ^ (1 << bit);
        mmio_write(addr, val);
    }
}
