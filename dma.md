# sg200x-bsp DMA 驱动使用说明

`sg200x_bsp::dma` 模块为 SG2002/CV181x 系列芯片提供 DMA 控制器驱动，基于 Synopsys DesignWare AXI DMA IP。

## 硬件概述

| 参数 | 值 |
|---|---|
| 控制器基址 | `0x0433_0000` |
| 最大通道数 | 8 |
| Master 数据通路宽度 | 4 字节（32 位） |
| LLI 描述符大小 | 64 字节，`#[repr(C, align(64))]` |
| 多块传输方式 | Linked List（LLI 链表） |

## 核心类型

### `DmaController`

DMA 控制器实例，管理所有通道。

```rust
use sg200x_bsp::dma::DmaController;

// 传入 DMAC 寄存器的 **虚拟地址**（MMU 环境下需做 phys_to_virt 转换）
let mut dma = DmaController::new(dmac_virt_addr);
```

### `DmaLli`

链表描述符（Linked List Item），每项 64 字节，与硬件严格对齐。

```rust
use sg200x_bsp::dma::DmaLli;

static mut LLI_ARRAY: [DmaLli; 4] = [DmaLli::new(); 4];
```

### `DmaSlaveConfig`

通道配置结构体，描述传输方向、地址宽度、突发长度等。

```rust
use sg200x_bsp::dma::{DmaSlaveConfig, DmaDirection, DmaWidth};

let config = DmaSlaveConfig {
    direction: DmaDirection::MemToMem,
    src_addr: 0,
    dst_addr: 0,
    src_addr_width: DmaWidth::Width32,
    dst_addr_width: DmaWidth::Width32,
    src_maxburst: 4,
    dst_maxburst: 4,
    device_fc: false,
};
```

### 枚举类型

| 类型 | 说明 | 常用值 |
|---|---|---|
| `DmaDirection` | 传输方向 | `MemToMem`, `MemToDev`, `DevToMem` |
| `DmaWidth` | 传输位宽 | `Width8`(1B), `Width16`(2B), `Width32`(4B) |
| `DmaMsize` | 突发大小 | `Msize1`, `Msize4`, `Msize8`, `Msize32` |
| `DmaFlowControl` | 流控类型 | `DmaM2M`, `DmaM2P`, `DmaP2M` |

## 使用流程

### 1. 使能时钟

DMA 控制器使用前需确保 SDMA AXI 时钟已打开：

```rust
// CLKGEN 基址 0x0300_2000
// CLK_EN_1 (offset 0x004) bit 1: CLK_SDMA_AXI
// CLK_EN_2 (offset 0x008) bit 1: CLK_AXI4
unsafe {
    let base = phys_to_virt(0x0300_2000);
    let en2 = (base + 0x008) as *mut u32;
    write_volatile(en2, read_volatile(en2) | (1 << 1));
    let en1 = (base + 0x004) as *mut u32;
    write_volatile(en1, read_volatile(en1) | (1 << 1));
}
```

### 2. 初始化控制器

```rust
let mut dma = DmaController::new(dmac_virt_addr);
dma.init();  // reset → disable → 清中断 → 清通道使能
```

### 3. 分配并配置通道

```rust
let ch = 0;
dma.alloc_channel(ch).unwrap();

// 设置外设/内存 master 接口
dma.channels[ch].m_master = 0;
dma.channels[ch].p_master = 0;
dma.channels[ch].src_id = 0;
dma.channels[ch].dst_id = 0;

let config = DmaSlaveConfig {
    direction: DmaDirection::MemToMem,
    src_addr_width: DmaWidth::Width32,
    dst_addr_width: DmaWidth::Width32,
    src_maxburst: 4,
    dst_maxburst: 4,
    ..Default::default()
};
dma.configure_channel(ch, &config).unwrap();
```

### 4. 准备 LLI 链

**关键：** `prepare_memcpy_lli` 的 `lli_array_phys` 参数必须传入 LLI 数组首元素的 **物理地址**，而非虚拟地址。在 MMU 环境下 VA != PA，DMAC 通过 LLP 取下一描述符时只认物理地址。

```rust
use sg200x_bsp::dma::prepare_memcpy_lli;

let lli_phys_base = virt_to_phys(&lli_array[0] as *const _ as usize) as u64;
let src_phys = virt_to_phys(src_buf.as_ptr() as usize) as u64;
let dst_phys = virt_to_phys(dst_buf.as_ptr() as usize) as u64;

let lli_count = prepare_memcpy_lli(
    &mut lli_array,
    lli_phys_base,   // LLI 数组的物理基址
    src_phys,        // 源物理地址
    dst_phys,        // 目的物理地址
    transfer_len,    // 传输字节数
    1024,            // block_size
    4,               // data_width（字节，SG2002 最大 4）
);
```

### 5. D-Cache 刷新（T-Head C906 必需）

T-Head C906 的 DRAM 区域被页表映射为 write-back cacheable，CPU 写入的数据可能停留在 D-cache 中。**必须在 DMA 启动前 flush D-cache**，DMA 完成后再 flush 一次以确保 CPU 看到 DMA 写入的结果。

```rust
#[inline(always)]
unsafe fn dcache_flush_all() {
    core::arch::asm!(
        ".long 0x0030000b", // T-Head dcache.ciall (clean + invalidate all)
        "fence rw, rw",
    );
}

// DMA 启动前
unsafe { dcache_flush_all(); }

dma.start_transfer(ch, lli_phys_base).unwrap();

// 等待完成...

// DMA 完成后
unsafe { dcache_flush_all(); }
```

### 6. 启动传输并等待完成

```rust
dma.start_transfer(ch, lli_phys_base).unwrap();

// 轮询等待通道禁用（传输完成后硬件自动清 CH_EN）
while dma.is_channel_enabled(ch) {
    core::hint::spin_loop();
}

// 检查错误
let int_status = dma.channels[ch].read_int_status();
if (int_status & sg200x_bsp::dma::regs::INT_ALL_ERR) != 0 {
    // 处理错误
}
```

### 7. 释放通道

```rust
dma.free_channel(ch);
```

## 完整示例：内存到内存拷贝

```rust
use sg200x_bsp::dma::*;

fn dma_memcpy(dma_vaddr: usize) {
    let mut lli = [DmaLli::new(); 4];
    let mut src = [0xA5u8; 64];
    let mut dst = [0u8; 64];

    let lli_pa = virt_to_phys(&lli[0] as *const _ as usize) as u64;
    let src_pa = virt_to_phys(src.as_ptr() as usize) as u64;
    let dst_pa = virt_to_phys(dst.as_ptr() as usize) as u64;

    let n = prepare_memcpy_lli(&mut lli, lli_pa, src_pa, dst_pa, 64, 1024, 4);
    assert!(n > 0);

    let mut dma = DmaController::new(dma_vaddr);
    dma.init();
    dma.alloc_channel(0).unwrap();

    dma.channels[0].m_master = 0;
    dma.channels[0].p_master = 0;

    let cfg = DmaSlaveConfig {
        direction: DmaDirection::MemToMem,
        src_addr_width: DmaWidth::Width32,
        dst_addr_width: DmaWidth::Width32,
        src_maxburst: 4,
        dst_maxburst: 4,
        ..Default::default()
    };
    dma.configure_channel(0, &cfg).unwrap();

    unsafe { dcache_flush_all(); }
    dma.start_transfer(0, lli_pa).unwrap();

    while dma.is_channel_enabled(0) {
        core::hint::spin_loop();
    }
    unsafe { dcache_flush_all(); }

    dma.free_channel(0);
    assert_eq!(dst, src);
}
```

## 完整示例：DMA 向 UART 发送数据

UART 没有 DMA 硬件流控，需要按 FIFO 深度（16 字节）分块发送，每块之间轮询 UART LSR 的 TEMT 位等待 FIFO 排空。

```rust
use sg200x_bsp::dma::*;
use sg200x_bsp::dma::regs::*;

const UART0_BASE_PHYS: u64 = 0x0414_0000;
const UART_LSR_OFFSET: usize = 0x14;
const UART_LSR_TEMT: u32 = 1 << 6;
const UART_FIFO_DEPTH: usize = 16;

fn wait_uart_tx_empty(uart_lsr_vaddr: usize) {
    unsafe {
        while core::ptr::read_volatile(uart_lsr_vaddr as *const u32) & UART_LSR_TEMT == 0 {
            core::hint::spin_loop();
        }
    }
}

fn dma_uart_send(dma_vaddr: usize, payload: &[u8]) {
    let mut lli = [DmaLli::new(); 1];
    let mut src_buf = [0u8; 16];

    // 构建 CTL：8 位宽，源递增，目的固定（UART THR）
    let ctl = (DmaMsize::Msize1 as u64) << CTL_DST_MSIZE_SHIFT
        | (DmaMsize::Msize1 as u64) << CTL_SRC_MSIZE_SHIFT
        | (DmaWidth::Width8 as u64) << CTL_DST_WIDTH_SHIFT
        | (DmaWidth::Width8 as u64) << CTL_SRC_WIDTH_SHIFT
        | CTL_SRC_INC | CTL_DST_FIX
        | CTL_SRC_STA_EN | CTL_DST_STA_EN
        | CTL_LLI_VALID | CTL_LLI_LAST | CTL_IOC_BLT_EN;

    let mut dma = DmaController::new(dma_vaddr);
    dma.init();
    dma.alloc_channel(0).unwrap();
    dma.channels[0].m_master = 0;
    dma.channels[0].p_master = 0;

    let cfg = DmaSlaveConfig {
        direction: DmaDirection::MemToMem,
        src_addr_width: DmaWidth::Width8,
        dst_addr_width: DmaWidth::Width8,
        src_maxburst: 1,
        dst_maxburst: 1,
        ..Default::default()
    };
    dma.configure_channel(0, &cfg).unwrap();

    let uart_lsr = phys_to_virt(UART0_BASE_PHYS as usize + UART_LSR_OFFSET);
    let mut offset = 0;

    while offset < payload.len() {
        let chunk = (payload.len() - offset).min(UART_FIFO_DEPTH);
        wait_uart_tx_empty(uart_lsr);

        src_buf[..chunk].copy_from_slice(&payload[offset..offset + chunk]);

        let src_pa = virt_to_phys(src_buf.as_ptr() as usize) as u64;
        lli[0] = DmaLli {
            sar: src_pa,
            dar: UART0_BASE_PHYS,
            block_ts: (chunk as u64) - 1,
            llp: 0,
            ctl,
            ..Default::default()
        };

        let lli_pa = virt_to_phys(&lli[0] as *const _ as usize) as u64;
        unsafe { dcache_flush_all(); }
        dma.start_transfer(0, lli_pa).unwrap();

        while dma.is_channel_enabled(0) {
            core::hint::spin_loop();
        }
        offset += chunk;
    }

    dma.free_channel(0);
    wait_uart_tx_empty(uart_lsr);
}
```

## 注意事项

### 物理地址 vs 虚拟地址

- `DmaController::new()` 接收 DMAC 寄存器的 **虚拟地址**（CPU 通过 MMU 访问）。
- `DmaLli` 中的 `sar`、`dar`、`llp` 以及 `start_transfer()` 的 `lli_phys` 参数必须是 **物理地址**（DMAC 直接走 AXI 总线，不经过 MMU）。
- `prepare_memcpy_lli` 的 `lli_array_phys` 参数必须是 LLI 数组首元素的物理地址。

### D-Cache 一致性（T-Head C906/C910）

DRAM 区域映射为 write-back cacheable，**每次 DMA 传输前后必须 flush D-cache**：
- 传输前：确保 CPU 写入的 LLI 和源数据已刷到物理内存。
- 传输后：确保 CPU 读到 DMA 写入的目的缓冲区数据。
- 使用 T-Head 自定义指令 `dcache.ciall`（编码 `0x0030000b`）执行全局 clean+invalidate。

### data_width 参数

SG2002 的 DMAC master 数据通路宽度为 4 字节（DTS `data-width = <4 4>`），`prepare_memcpy_lli` 的 `data_width` 参数应传 `4`，不要传 `32`（会被解释为 256 位宽度，超出硬件能力）。

### UART DMA 发送的 FIFO 限制

SG2002 UART 没有 DMA 硬件握手流控，DMA 以总线最大速度灌数据会导致 FIFO 溢出、数据丢失。解决方案：
- 每次 DMA 传输数据量不超过 UART FIFO 深度（通常 16 字节）。
- 每块传输前轮询 UART LSR 寄存器 bit 6（TEMT）确认发送器完全空闲。

### 静态缓冲区

建议将 LLI 数组和数据缓冲区放在 `static` 存储中（而非栈上），以确保 `virt_to_phys` 转换的地址稳定可靠，且生命周期覆盖整个 DMA 传输过程。

## API 速览

| 方法 | 说明 |
|---|---|
| `DmaController::new(base)` | 创建控制器实例（传入虚拟地址） |
| `dma.init()` | 复位并初始化控制器 |
| `dma.alloc_channel(ch)` | 分配通道（首次分配时自动 enable 控制器） |
| `dma.configure_channel(ch, &config)` | 配置通道传输参数 |
| `dma.start_transfer(ch, lli_phys)` | 启动 LLI 链式传输 |
| `dma.is_channel_enabled(ch)` | 检查通道是否仍在传输 |
| `dma.free_channel(ch)` | 停止并释放通道（最后一个通道释放时自动 disable 控制器） |
| `dma.handle_interrupt()` | 处理全部通道中断，返回已处理通道掩码 |
| `dma.get_residue(ch)` | 获取通道剩余未传输字节数 |
| `prepare_memcpy_lli(...)` | 构建 M2M LLI 链（自动计算传输宽度和分块） |
| `build_ctl_m2m(...)` | 构建 M2M 传输的 CTL 寄存器值 |
| `build_ctl_slave(...)` | 构建外设传输的 CTL 寄存器值 |
