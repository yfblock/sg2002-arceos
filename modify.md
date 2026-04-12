# DMA UART TX 功能开发 — 修改记录

## 目标

在 SG2002（T-Head C906）平台上，通过 DesignWare AXI DMAC 将数据写入 UART0 THR，实现 DMA 串口输出。

## 涉及文件

| 文件 | 操作 |
|------|------|
| `examples/helloworld/src/dma_uart_tx.rs` | **新增** |
| `examples/helloworld/src/main.rs` | 修改（引入 `dma_uart_tx` 模块并调用） |
| `examples/helloworld/Cargo.toml` | 修改（添加 `cvitek-dma` 依赖） |
| `LicheeRV/cvitek-dma-rs/src/lib.rs` | 修改（修复 5 处 bug） |
| `LicheeRV/cvitek-dma-rs/src/regs.rs` | 修改（修正握手常量命名） |
| `LicheeRV/cvitek-dma-rs/README.md` | 修改（同步 API 变更） |

## 修复的 Bug（共 5 个）

### Bug 1：`DmaLli` 结构体大小 128 字节，硬件要求 64 字节

**文件**：`LicheeRV/cvitek-dma-rs/src/lib.rs`

旧版 `DmaLli` 有 9 个 `u64` 字段（72 字节）加 `align(64)` 导致数组步长为 128 字节。
DesignWare AXI DMAC 的 LLI 描述符固定 64 字节，128 字节步长使 LLP 链全部错位，
DMA 读取第二条 LLI 起全部是垃圾数据。

**修改**：去掉多余的 `reserved` 字段（8 个字段 × 8 字节 = 64 字节），
添加编译期断言 `const _: () = assert!(core::mem::size_of::<DmaLli>() == 64);`。

### Bug 2：`prepare_memcpy_lli` 的 LLP 使用虚拟地址

**文件**：`LicheeRV/cvitek-dma-rs/src/lib.rs`

原实现用 `&lli_array[i + 1] as *const DmaLli as u64` 作为下一条 LLI 地址，
这是虚拟地址。DMAC 通过 AXI 总线用物理地址取描述符，MMU 下 VA≠PA 时会读错。

**修改**：`prepare_memcpy_lli` 新增 `lli_array_phys: u64` 参数（`lli_array[0]` 的物理地址），
用 `lli_array_phys + (i + 1) * size_of::<DmaLli>()` 计算每条 LLP。

### Bug 3：D-cache 未刷新（T-Head C906 write-back cacheable）

**文件**：`examples/helloworld/src/dma_uart_tx.rs`

SG2002 的启动页表将 DRAM 区域映射为 `KERNEL_FLAGS = 0b01111 << 59`（C=1, B=1），
即 write-back cacheable。CPU 写入 DMA 缓冲区的数据留在 D-cache 中，
DMA 从物理内存读到全零的 stale 值；DMA 写完后 CPU 也读到缓存中的旧值。

**修改**：添加 `dcache_flush_all()` 函数（T-Head 自定义 `dcache.ciall` 指令，编码 `0x0030000b`），
在 DMA 启动前调用确保 CPU 写入到达物理内存，DMA 完成后再调用确保 CPU 读到 DMA 写入的数据。

### Bug 4：`data_width` 参数传 32 应为 4（字节）

**文件**：`examples/helloworld/src/dma_uart_tx.rs`、`LicheeRV/cvitek-dma-rs/src/lib.rs`

`prepare_memcpy_lli` 调用传 `data_width = 32`，经 `trailing_zeros` 计算得到
`trans_width = 5` → `DmaWidth::Width256`（256 位 / 32 字节通路）。
但 SG2002 DTS 写明 `data-width = <4 4>`（4 字节 = 32 位最大通路），
硬件只有 4 字节数据通路，Width256 下只有部分字节正确传输。

**修改**：调用处改为 `data_width = 4`；`DmaController` 默认 `data_width` 从 `[32,32,32,32]` 改为 `[4,4,4,4]`。

### Bug 5：`static mut` 在 Rust 2024 edition 被拒绝

**文件**：`examples/helloworld/src/dma_uart_tx.rs`

Rust 2024 对 `static mut` 引用会触发 `static_mut_refs` deny 错误。

**修改**：改用 `UnsafeCell<DmaBuffers>` 包装 + `DmaBuffersStatic` newtype + `unsafe impl Sync`，
通过 `buf_mut()` 函数在 `unsafe` 块中访问。

## 额外修改

### 握手常量命名修正

**文件**：`LicheeRV/cvitek-dma-rs/src/regs.rs`

原 `CFG_HS_SEL_DST_HW = 1 << 36` 与 Linux/DesignWare 定义相反
（位为 1 表示**软件**握手，0 表示**硬件**握手）。
重命名为 `CFG_HS_SEL_SRC_SW` / `CFG_HS_SEL_DST_SW` 并添加注释。
`init_channel` 中去掉对该常量的错误 `cfg |=`，使 MemToDev/DevToMem 下位 35/36 保持 0 = 硬件握手。

### UART DMA 发送改为 8 位宽 + 分块发送

**文件**：`examples/helloworld/src/dma_uart_tx.rs`

UART THR 是 8 位寄存器，对 APB 桥做 32 位写可能异常。
改为 `DmaWidth::Width8` + `DmaMsize::Msize1`，源递增、目的固定。

SG2002 的 UART 没有 DMA request 线连接到 DMAC（DTS 中 UART 节点无 `dmas` 属性），
只能用 `MemToMem` 模式无流控发送。DMA 以总线速度灌 THR，超过 16 字节 FIFO 深度后会丢字节。

解决方案：**分块发送**，每块 ≤16 字节（UART FIFO 深度），发完一块后轮询 UART LSR 寄存器的
TEMT 位（bit 6 = Transmitter Empty，FIFO + 移位寄存器均空）再发下一块。
DMA channel 只分配/配置一次，循环内仅更新 LLI 的 `block_ts` 和 `src_uart` 内容、刷新 D-cache、
重新启动传输。

## 最终效果

串口完整输出 DMA payload 文本，无丢字节：

```
[dma_uart_tx] RAM self-test passed

[dma_uart_tx] hello from DMA -> UART0 (8-bit THR)
[dma_uart_tx] UART DMA completed, 53 bytes sent
[dma_uart_tx] run_demo finished
```

## 已知限制

- **无硬件流控**：由于 UART 未连接 DMA handshake 线，只能用 MemToMem + 软件分块方式。
  若未来需要更高效的 DMA UART，需确认 SoC 是否有 UART DMA request 线路并使用 `MemToDev` 模式。
