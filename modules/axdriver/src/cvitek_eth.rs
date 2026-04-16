//! CVitek/Sophgo SG2002 Ethernet driver (Synopsys DWMAC 3.70a)
//!
//! A bare-metal driver for the DesignWare MAC 10/100 controller found
//! on the SG2002 SoC at MMIO address 0x0407_0000.
//! Uses normal DMA descriptors in ring mode with polling (no IRQ).

extern crate alloc;

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{fence, Ordering};

use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use axdriver_net::{EthernetAddress, NetBuf, NetBufBox, NetBufPool, NetBufPtr, NetDriverOps};

// ---------------------------------------------------------------------------
// Hardware constants
// ---------------------------------------------------------------------------

const ETH_BASE: usize = 0x0407_0000;

// MAC registers
const GMAC_CONTROL: usize = 0x0000;
const GMAC_FRAME_FILTER: usize = 0x0004;
const GMAC_HASH_HIGH: usize = 0x0008;
const GMAC_HASH_LOW: usize = 0x000C;
const GMAC_MII_ADDR: usize = 0x0010;
const GMAC_MII_DATA: usize = 0x0014;
const GMAC_INT_MASK: usize = 0x003C;
const GMAC_ADDR0_HIGH: usize = 0x0040;
const GMAC_ADDR0_LOW: usize = 0x0044;

// DMA registers (base + 0x1000)
const DMA_BUS_MODE: usize = 0x1000;
const DMA_TX_POLL: usize = 0x1004;
const DMA_RX_POLL: usize = 0x1008;
const DMA_RX_BASE_ADDR: usize = 0x100C;
const DMA_TX_BASE_ADDR: usize = 0x1010;
const DMA_STATUS: usize = 0x1014;
const DMA_OPERATION: usize = 0x1018;
const DMA_INTR_ENA: usize = 0x101C;

// GMAC_CONTROL bits
const GMAC_CTL_RE: u32 = 1 << 2;
const GMAC_CTL_TE: u32 = 1 << 3;

// DMA_BUS_MODE bits
const DMA_BUS_SWR: u32 = 1 << 0;
const DMA_BUS_DSL_SHIFT: u32 = 2;
const DMA_BUS_PBL_SHIFT: u32 = 8;
const DMA_BUS_FB: u32 = 1 << 16;
const DMA_BUS_AAL: u32 = 1 << 25;
// DSL = 12 words (48 bytes) skip -> stride = 16+48 = 64 bytes (cache line)
const DMA_BUS_DSL: u32 = 12 << DMA_BUS_DSL_SHIFT;

// DMA_OPERATION bits
const DMA_OP_SR: u32 = 1 << 1;
const DMA_OP_OSF: u32 = 1 << 2;
const DMA_OP_ST: u32 = 1 << 13;
const DMA_OP_FTF: u32 = 1 << 20;
const DMA_OP_TSF: u32 = 1 << 21;
const DMA_OP_RSF: u32 = 1 << 25;

// DMA_STATUS bits
const DMA_STS_RI: u32 = 1 << 6;
const DMA_STS_NIS: u32 = 1 << 16;

// TX descriptor TDES0 (normal mode: only OWN is host-writable)
const TDES0_OWN: u32 = 1 << 31;

// TX descriptor TDES1 (normal mode: control + buffer size)
const TDES1_IC: u32 = 1 << 31;
const TDES1_LS: u32 = 1 << 30;
const TDES1_FS: u32 = 1 << 29;
const TDES1_TER: u32 = 1 << 25;
const TDES1_TBS1_MASK: u32 = 0x7FF;

// RX descriptor RDES0
const RDES0_OWN: u32 = 1 << 31;
const RDES0_FL_MASK: u32 = 0x3FFF << 16;
const RDES0_FL_SHIFT: u32 = 16;
const RDES0_ES: u32 = 1 << 15;

// RX descriptor RDES1
const RDES1_RBS1_MASK: u32 = 0x7FF;
const RDES1_RER: u32 = 1 << 25;

// MDIO
const MII_BUSY: u32 = 1 << 0;
const MII_WRITE: u32 = 1 << 1;
const MII_CLK_CSR: u32 = 0x4 << 2; // CSR 60-100 MHz -> /42

const TX_RING_SIZE: usize = 32;
const RX_RING_SIZE: usize = 32;
const BUF_SIZE: usize = 2048;

const PHY_ADDR: u32 = 0;

// ---------------------------------------------------------------------------
// DMA Descriptor (16 bytes, aligned to cache line to avoid coherency issues)
// ---------------------------------------------------------------------------

#[repr(C, align(64))]
struct DmaDesc {
    des0: u32,
    des1: u32,
    des2: u32,
    des3: u32,
}

// ---------------------------------------------------------------------------
// Cache helpers for T-Head C906
// ---------------------------------------------------------------------------

const CACHE_LINE: usize = 64;

#[inline(always)]
unsafe fn dcache_cva(va: usize) {
    core::arch::asm!(".insn i 0x0b, 0, x0, {0}, 0x025", in(reg) va);
}

#[inline(always)]
unsafe fn dcache_iva(va: usize) {
    core::arch::asm!(".insn i 0x0b, 0, x0, {0}, 0x026", in(reg) va);
}

#[inline(always)]
unsafe fn dcache_clean_range(start: usize, size: usize) {
    let mut addr = start & !(CACHE_LINE - 1);
    let end = start + size;
    while addr < end {
        dcache_cva(addr);
        addr += CACHE_LINE;
    }
    core::arch::asm!("fence iorw, iorw");
}

#[inline(always)]
unsafe fn dcache_invalidate_range(start: usize, size: usize) {
    let mut addr = start & !(CACHE_LINE - 1);
    let end = start + size;
    while addr < end {
        dcache_iva(addr);
        addr += CACHE_LINE;
    }
    core::arch::asm!("fence iorw, iorw");
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct CvitekEthNic {
    base: usize,
    mac_addr: [u8; 6],
    tx_pool: Arc<NetBufPool>,
    rx_pool: Arc<NetBufPool>,
    tx_descs: Vec<DmaDesc>,
    rx_descs: Vec<DmaDesc>,
    tx_bufs: Vec<Option<NetBufPtr>>,
    rx_bufs: Vec<Option<NetBufBox>>,
    tx_head: usize,
    tx_tail: usize,
    rx_cur: usize,
}

unsafe impl Send for CvitekEthNic {}
unsafe impl Sync for CvitekEthNic {}

impl CvitekEthNic {
    #[inline]
    fn read_reg(&self, off: usize) -> u32 {
        unsafe { core::ptr::read_volatile((self.base + off) as *const u32) }
    }

    #[inline]
    fn write_reg(&self, off: usize, val: u32) {
        unsafe { core::ptr::write_volatile((self.base + off) as *mut u32, val) }
    }

    fn mdio_wait(&self) {
        let mut t = 100_000u32;
        while self.read_reg(GMAC_MII_ADDR) & MII_BUSY != 0 {
            t = t.wrapping_sub(1);
            if t == 0 {
                break;
            }
        }
    }

    fn mdio_read(&self, phy: u32, reg: u32) -> u16 {
        self.mdio_wait();
        let v = (phy << 11) | (reg << 6) | MII_CLK_CSR | MII_BUSY;
        self.write_reg(GMAC_MII_ADDR, v);
        self.mdio_wait();
        self.read_reg(GMAC_MII_DATA) as u16
    }

    fn mdio_write(&self, phy: u32, reg: u32, data: u16) {
        self.mdio_wait();
        self.write_reg(GMAC_MII_DATA, data as u32);
        let v = (phy << 11) | (reg << 6) | MII_CLK_CSR | MII_WRITE | MII_BUSY;
        self.write_reg(GMAC_MII_ADDR, v);
        self.mdio_wait();
    }

    fn dma_reset(&self) {
        self.write_reg(DMA_BUS_MODE, DMA_BUS_SWR);
        let mut t = 100_000u32;
        while self.read_reg(DMA_BUS_MODE) & DMA_BUS_SWR != 0 {
            t = t.wrapping_sub(1);
            if t == 0 {
                log::warn!("cvitek-eth: DMA reset timeout");
                break;
            }
        }
    }

    fn read_mac_from_hw(&self) -> [u8; 6] {
        let hi = self.read_reg(GMAC_ADDR0_HIGH);
        let lo = self.read_reg(GMAC_ADDR0_LOW);
        [
            (lo & 0xFF) as u8,
            ((lo >> 8) & 0xFF) as u8,
            ((lo >> 16) & 0xFF) as u8,
            ((lo >> 24) & 0xFF) as u8,
            (hi & 0xFF) as u8,
            ((hi >> 8) & 0xFF) as u8,
        ]
    }

    fn set_mac_hw(&self, m: &[u8; 6]) {
        let lo = (m[0] as u32)
            | ((m[1] as u32) << 8)
            | ((m[2] as u32) << 16)
            | ((m[3] as u32) << 24);
        let hi = (m[4] as u32) | ((m[5] as u32) << 8) | (1u32 << 31);
        self.write_reg(GMAC_ADDR0_LOW, lo);
        self.write_reg(GMAC_ADDR0_HIGH, hi);
    }

    fn flush_desc(desc: &DmaDesc) {
        let addr = desc as *const DmaDesc as usize;
        unsafe { dcache_clean_range(addr, core::mem::size_of::<DmaDesc>()) };
    }

    fn invalidate_desc(desc: &DmaDesc) {
        let addr = desc as *const DmaDesc as usize;
        unsafe { dcache_invalidate_range(addr, core::mem::size_of::<DmaDesc>()) };
    }

    fn setup_tx_ring(&mut self) {
        for i in 0..TX_RING_SIZE {
            let d = &mut self.tx_descs[i];
            d.des0 = 0;
            d.des1 = if i == TX_RING_SIZE - 1 { TDES1_TER } else { 0 };
            d.des2 = 0;
            d.des3 = 0;
            Self::flush_desc(d);
        }
    }

    fn setup_rx_ring(&mut self) {
        for i in 0..RX_RING_SIZE {
            let buf = self.rx_pool.alloc_boxed().expect("RX buf alloc");
            let data_pa = buf.raw_buf().as_ptr() as u32;

            let d = &mut self.rx_descs[i];
            d.des2 = data_pa;
            d.des1 = {
                let mut v = (BUF_SIZE as u32).min(RDES1_RBS1_MASK);
                if i == RX_RING_SIZE - 1 {
                    v |= RDES1_RER;
                }
                v
            };
            d.des3 = 0;
            fence(Ordering::Release);
            d.des0 = RDES0_OWN;
            Self::flush_desc(d);
            self.rx_bufs[i] = Some(buf);
        }
    }

    pub fn init(base: usize) -> DevResult<Self> {
        let tx_pool = NetBufPool::new(TX_RING_SIZE + 16, BUF_SIZE)?;
        let rx_pool = NetBufPool::new(RX_RING_SIZE + 16, BUF_SIZE)?;

        let mut tx_descs = Vec::with_capacity(TX_RING_SIZE);
        let mut rx_descs = Vec::with_capacity(RX_RING_SIZE);
        for _ in 0..TX_RING_SIZE {
            tx_descs.push(DmaDesc { des0: 0, des1: 0, des2: 0, des3: 0 });
        }
        for _ in 0..RX_RING_SIZE {
            rx_descs.push(DmaDesc { des0: 0, des1: 0, des2: 0, des3: 0 });
        }

        let mut tx_bufs: Vec<Option<NetBufPtr>> = Vec::with_capacity(TX_RING_SIZE);
        let mut rx_bufs: Vec<Option<NetBufBox>> = Vec::with_capacity(RX_RING_SIZE);
        for _ in 0..TX_RING_SIZE {
            tx_bufs.push(None);
        }
        for _ in 0..RX_RING_SIZE {
            rx_bufs.push(None);
        }

        let mut nic = Self {
            base,
            mac_addr: [0; 6],
            tx_pool,
            rx_pool,
            tx_descs,
            rx_descs,
            tx_bufs,
            rx_bufs,
            tx_head: 0,
            tx_tail: 0,
            rx_cur: 0,
        };

        let ver = nic.read_reg(0x0020);
        log::info!("cvitek-eth: DWMAC version {:#x}", ver);

        nic.mac_addr = nic.read_mac_from_hw();
        if nic.mac_addr == [0; 6] || nic.mac_addr == [0xFF; 6] {
            nic.mac_addr = [0x00, 0x50, 0x43, 0x02, 0x02, 0x02];
        }
        log::info!(
            "cvitek-eth: MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            nic.mac_addr[0], nic.mac_addr[1], nic.mac_addr[2],
            nic.mac_addr[3], nic.mac_addr[4], nic.mac_addr[5],
        );

        let uboot_mac_ctl = nic.read_reg(GMAC_CONTROL);

        // Enable Ethernet clocks (U-Boot may disable them on shutdown)
        unsafe {
            let clk_en0 = 0x0300_2000 as *mut u32;
            let old = core::ptr::read_volatile(clk_en0);
            core::ptr::write_volatile(clk_en0, old | (1 << 25) | (1 << 26));
        }
        // Release Ethernet MAC reset
        unsafe {
            let soft_rstn0 = 0x0300_3000 as *mut u32;
            let old = core::ptr::read_volatile(soft_rstn0);
            core::ptr::write_volatile(soft_rstn0, old | (1 << 12));
        }
        // Deassert EPHY resets
        unsafe {
            let soft_rstn3 = 0x0300_300C as *mut u32;
            let old = core::ptr::read_volatile(soft_rstn3);
            core::ptr::write_volatile(soft_rstn3, old | (1 << 0) | (1 << 1));
        }
        for _ in 0..2_000_000u32 { core::hint::spin_loop(); }

        // DMA soft-reset
        nic.dma_reset();

        // Build descriptor rings
        nic.setup_tx_ring();
        nic.setup_rx_ring();

        // DMA bus mode: PBL=8, DSL=12 (64-byte desc stride), fixed burst, AAL
        nic.write_reg(DMA_BUS_MODE,
            (8u32 << DMA_BUS_PBL_SHIFT) | DMA_BUS_DSL | DMA_BUS_FB | DMA_BUS_AAL);

        nic.write_reg(DMA_TX_BASE_ADDR, nic.tx_descs.as_ptr() as u32);
        nic.write_reg(DMA_RX_BASE_ADDR, nic.rx_descs.as_ptr() as u32);

        // Disable DMA interrupts (polling mode)
        nic.write_reg(GMAC_INT_MASK, 0x60F);
        nic.write_reg(DMA_INTR_ENA, 0);

        nic.set_mac_hw(&nic.mac_addr);

        // Receive all frames for now
        nic.write_reg(GMAC_FRAME_FILTER, 1 | (1 << 31));
        nic.write_reg(GMAC_HASH_HIGH, 0xFFFF_FFFF);
        nic.write_reg(GMAC_HASH_LOW, 0xFFFF_FFFF);

        // PHY init: soft-reset, auto-negotiate
        nic.mdio_write(PHY_ADDR, 0, 0x8000);
        for _ in 0..1_000_000u32 { core::hint::spin_loop(); }
        for _ in 0..100u32 {
            if nic.mdio_read(PHY_ADDR, 0) & 0x8000 == 0 { break; }
            for _ in 0..100_000u32 { core::hint::spin_loop(); }
        }
        nic.mdio_write(PHY_ADDR, 0, 0x3300);

        // Wait for link up
        let mut link_up = false;
        for i in 0..300u32 {
            let _ = nic.mdio_read(PHY_ADDR, 1);
            let bmsr = nic.mdio_read(PHY_ADDR, 1);
            if bmsr & 4 != 0 {
                log::info!("cvitek-eth: link UP after {} polls", i);
                link_up = true;
                break;
            }
            for _ in 0..200_000u32 { core::hint::spin_loop(); }
        }
        if !link_up {
            log::warn!("cvitek-eth: link still DOWN, continuing anyway");
        }

        // Restore U-Boot MAC control + enable TX/RX
        nic.write_reg(GMAC_CONTROL, uboot_mac_ctl | GMAC_CTL_TE | GMAC_CTL_RE);

        // Reset MMC counters
        nic.write_reg(0x0100, 0x01);

        // DMA operation: store-and-forward, flush TX FIFO, start TX/RX
        nic.write_reg(DMA_OPERATION, DMA_OP_TSF | DMA_OP_RSF | DMA_OP_OSF | DMA_OP_FTF);
        let mut t = 100_000u32;
        while nic.read_reg(DMA_OPERATION) & DMA_OP_FTF != 0 {
            t = t.wrapping_sub(1);
            if t == 0 { break; }
        }
        nic.write_reg(DMA_OPERATION, nic.read_reg(DMA_OPERATION) | DMA_OP_ST | DMA_OP_SR);
        nic.write_reg(DMA_RX_POLL, 1);

        log::info!("cvitek-eth: initialized OK");
        Ok(nic)
    }
}

impl BaseDriverOps for CvitekEthNic {
    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }
    fn device_name(&self) -> &str {
        "cvitek-eth"
    }
}

impl NetDriverOps for CvitekEthNic {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress(self.mac_addr)
    }

    fn can_transmit(&self) -> bool {
        Self::invalidate_desc(&self.tx_descs[self.tx_head]);
        let des0 = unsafe { core::ptr::read_volatile(&self.tx_descs[self.tx_head].des0) };
        des0 & TDES0_OWN == 0
    }

    fn can_receive(&self) -> bool {
        Self::invalidate_desc(&self.rx_descs[self.rx_cur]);
        let des0 = unsafe { core::ptr::read_volatile(&self.rx_descs[self.rx_cur].des0) };
        des0 & RDES0_OWN == 0
    }

    fn rx_queue_size(&self) -> usize {
        RX_RING_SIZE
    }
    fn tx_queue_size(&self) -> usize {
        TX_RING_SIZE
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        unsafe {
            let _ = NetBuf::from_buf_ptr(rx_buf);
        }
        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        while self.tx_tail != self.tx_head {
            Self::invalidate_desc(&self.tx_descs[self.tx_tail]);
            let des0 = unsafe { core::ptr::read_volatile(&self.tx_descs[self.tx_tail].des0) };
            if des0 & TDES0_OWN != 0 {
                break;
            }
            if let Some(buf) = self.tx_bufs[self.tx_tail].take() {
                unsafe {
                    let _ = NetBuf::from_buf_ptr(buf);
                }
            }
            self.tx_tail = (self.tx_tail + 1) % TX_RING_SIZE;
        }
        Ok(())
    }

    fn transmit(&mut self, mut tx_buf: NetBufPtr) -> DevResult {
        let idx = self.tx_head;

        Self::invalidate_desc(&self.tx_descs[idx]);
        let des0 = unsafe { core::ptr::read_volatile(&self.tx_descs[idx].des0) };
        if des0 & TDES0_OWN != 0 {
            return Err(DevError::Again);
        }

        let mut len = tx_buf.packet_len();
        if len < 60 {
            let end = tx_buf.packet().as_ptr() as usize + len;
            unsafe { core::ptr::write_bytes(end as *mut u8, 0, 60 - len) };
            len = 60;
        }
        let data = tx_buf.packet().as_ptr() as usize;
        unsafe { dcache_clean_range(data, len) };

        let d = &mut self.tx_descs[idx];

        let mut tdes1 = TDES1_IC | TDES1_FS | TDES1_LS | ((len as u32) & TDES1_TBS1_MASK);
        if idx == TX_RING_SIZE - 1 {
            tdes1 |= TDES1_TER;
        }

        unsafe {
            core::ptr::write_volatile(&mut d.des2, data as u32);
            core::ptr::write_volatile(&mut d.des1, tdes1);
        }

        fence(Ordering::Release);
        unsafe { core::ptr::write_volatile(&mut d.des0, TDES0_OWN) };
        Self::flush_desc(d);
        unsafe { dcache_clean_range(data, len) };

        self.tx_bufs[idx] = Some(tx_buf);
        self.tx_head = (self.tx_head + 1) % TX_RING_SIZE;

        self.write_reg(DMA_TX_POLL, 1);
        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        let idx = self.rx_cur;

        let dma_sts = self.read_reg(DMA_STATUS);
        if dma_sts & DMA_STS_RI != 0 {
            self.write_reg(DMA_STATUS, DMA_STS_RI | DMA_STS_NIS);
        }

        Self::invalidate_desc(&self.rx_descs[idx]);
        let des0 = unsafe { core::ptr::read_volatile(&self.rx_descs[idx].des0) };

        if des0 & RDES0_OWN != 0 {
            return Err(DevError::Again);
        }

        if des0 & RDES0_ES != 0 {
            unsafe { core::ptr::write_volatile(&mut self.rx_descs[idx].des0, RDES0_OWN) };
            Self::flush_desc(&self.rx_descs[idx]);
            self.rx_cur = (self.rx_cur + 1) % RX_RING_SIZE;
            self.write_reg(DMA_RX_POLL, 1);
            return Err(DevError::Again);
        }

        let frame_len = ((des0 & RDES0_FL_MASK) >> RDES0_FL_SHIFT) as usize;
        let frame_len = if frame_len >= 4 { frame_len - 4 } else { frame_len };

        let mut buf = self.rx_bufs[idx].take().ok_or(DevError::Again)?;
        unsafe { dcache_invalidate_range(buf.raw_buf().as_ptr() as usize, frame_len) };

        buf.set_packet_len(frame_len);
        let buf_ptr = buf.into_buf_ptr();

        if let Some(new_buf) = self.rx_pool.alloc_boxed() {
            let pa = new_buf.raw_buf().as_ptr() as u32;
            let d = &mut self.rx_descs[idx];
            unsafe {
                core::ptr::write_volatile(&mut d.des2, pa);
                let mut rdes1 = (BUF_SIZE as u32).min(RDES1_RBS1_MASK);
                if idx == RX_RING_SIZE - 1 {
                    rdes1 |= RDES1_RER;
                }
                core::ptr::write_volatile(&mut d.des1, rdes1);
                core::ptr::write_volatile(&mut d.des3, 0);
                fence(Ordering::Release);
                core::ptr::write_volatile(&mut d.des0, RDES0_OWN);
            }
            Self::flush_desc(d);
            self.rx_bufs[idx] = Some(new_buf);
        } else {
            log::warn!("cvitek-eth: RX buf alloc failed");
        }

        self.rx_cur = (self.rx_cur + 1) % RX_RING_SIZE;
        self.write_reg(DMA_STATUS, DMA_STS_RI | DMA_STS_NIS);
        self.write_reg(DMA_RX_POLL, 1);

        Ok(buf_ptr)
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        let mut buf = self.tx_pool.alloc_boxed().ok_or(DevError::NoMemory)?;
        buf.set_packet_len(size);
        Ok(buf.into_buf_ptr())
    }
}
