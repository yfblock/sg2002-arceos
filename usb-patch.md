# USB 主机栈：`dwc-rust-test` → ArceOS `usb-dwc2-host` 变更摘要

本文档对照仓库内原始参考实现 **`dwc-rust-test/usb-bare-dri/`**（树莓派 3B + QEMU `raspi3b` 验证），总结迁入 **`modules/usb-dwc2-host/`** 及 **`examples/helloworld`** 板级胶水后的主要差异。路径均以仓库根为基准。

---

## 1. 工程结构与依赖

| 项目 | `dwc-rust-test` | ArceOS 当前 |
|------|-----------------|-------------|
| Crate 名 | `usb-bare-dri` | `usb-dwc2-host` |
| HAL | 依赖 `hal`（`mmio`、RPi3 `DWC2_USB_BASE`、`cache`、UART） | 自带 `mmio`、`cache`，**不再依赖 `hal`** |
| 日志 | `uart_log::UsbUartWriter`（Mini UART） | `log` 模块：`set_usb_log_fn` 回调（如接到 `println!`） |
| 基址 | 编译期常量 `hal::rpi3::DWC2_USB_BASE` | 运行时 `platform::set_dwc2_base_virt`（`AtomicUsize`） |
| DMA 地址 | `HCDMA` 直接写 `指针 as u32`（假设 VA=PA） | `platform::set_usb_dma_to_phys_fn` 可选；默认 `va as u32` |
| 上层功能 | 含 `mass_storage`、`fat_disk`、`fatfs`（M3/M3.5） | **crate 内未包含** MSC/FAT；仅保留枚举与拓扑 |
| Cargo feature | 无 | `cv182x-host`（SG2002/CV182x 主机初始化）；`usb-force-no-dma`（实验） |

---

## 2. `dwc2.rs`（控制器 bring-up）

### 2.1 通用增强（相对 RPi 路径）

- **`dwc2_probe`**：未设基址时返回明确错误；返回值含 **`GHWCFG1`**（原为 `GHWCFG2/3` 三元组，现 `(GHWCFG1,GHWCFG2,GHWCFG3)`）。
- **`wait_ahb_idle` / 软复位等**：轮询上限加大；超时时 **`dbg_dwc2_init_timeout`** 转储 `GRSTCTL`、`GINTSTS`、`GAHBCFG`、`HPRT0` 等（`USB-TOUT dwc2-init`）。
- **`core_soft_reset`**：按 **`GSNPSID` 低 16 位** 区分 Synopsys 版本；**≥ 4.20a** 时对齐 Linux `dwc2_core_reset`：等待 **`CSFTRST_DONE`**，再清 **`CSFTRST`** 并处理 **`CSFTRST_DONE`**，避免旧版「傻等 `CSFTRST` 自清」在 SG2002 上卡死。
- **新增导出**：`hprt_lnsts`、`dwc2_host_root_bus_reset_pulse`（根口 USB 总线复位）、`debug_dump_root_port_hw`（失败时转储 `HPRT0`、`GOTGCTL`、`GUSBCFG`、`GAHBCFG`、`GINTSTS`、`PCGCTL`、`HCFG` 及 **PHY 0x03006000 多寄存器**）。

### 2.2 双路径：`#[cfg(feature = "cv182x-host")]`（默认 RPi 风格为 `not`）

启用 **`cv182x-host`** 时，`dwc2_host_init` 走 **Linux `dwc2_set_cv182x_params` / `dwc2_core_host_init` 思路**，与 RPi 演示路径显著不同：

| 方面 | RPi（`not(cv182x-host)`） | CV182x / SG2002 |
|------|---------------------------|-----------------|
| `GUSBCFG` | 主要 `FORCEHOSTMODE` | **UTMI 16-bit**、清 ULPI、**`TOUTCAL=7`** 等 |
| `GOTGCTL` | 不配置 | **A-session / VBUS valid 软件 override** + 去抖动旁路；端口上电后 **再写一次**（防部分芯片清位） |
| `PCGCTL` | 未单独强调 | 写 **0** 解除 PHY 时钟门控类默认 |
| `GAHBCFG` | `HBSTLEN=3`（INCR8）、按 `ARCH` 开 DMA | **`HBSTLEN=INCR16`**、同样按 `ARCH==2` 开 **`DMA_EN`** |
| `HCFG` | **置 `FSLSSUPP` + 48MHz FSLS 时钟**（全速友好） | **清 `FSLSSUPP`**（高速主机路径，与 Linux HS 一致） |
| FIFO | 固定深度 `init_fifos` | 按 **`GHWCFG3`** 总深度 **`dwc2_calculate_dynamic_fifo` 风格** 配置 RX/NPTX/PTX；满足版本时写 **`GDFIFOCFG`** |
| FIFO flush | 无独立 host flush | **`flush_tx_fifo_host_all` / `flush_rx_fifo_host`**（带 `GRSTCTL` 握手等待） |
| 片内 PHY `0x03006000` | 无 | **`REG014` 写 0**：清除 **`UTMI_OVERRIDE`**，由 **DWC2 UTMI** 驱动下拉/收发（对齐 vendor Linux host 路径；曾用 `0xC1` 软件下拉已弃用） |
| 根口复位 | `port_power_on` 后 **立即 `port_reset_pulse`** | **`dwc2_host_init` 内不发总线复位**；由 **`host.rs`** 在 **`CONNSTS==1`** 后调用 **`dwc2_host_root_bus_reset_pulse`**（含 `CONNDET` W1C） |

未启用 `cv182x-host` 时，逻辑与 `dwc-rust-test` 的 **`init_fifos` + `init_gahb` + `init_hcfg_fs_ls` + 上电后立即总线复位** 仍属同一类 RPi/QEMU 演示序列（基址与超时/软复位新版行为除外）。

---

## 3. `dwc2_ep0.rs` + `cache.rs`

- **基址**：`BASE` 常量 → `platform::dwc2_base_virt()`。
- **`dma_phys`**：经 **`usb_dma_phys_for`**，支持 MMU 下 VA≠PA。
- **RISC-V**：DMA 前后增加 **`fence rw, rw`**（`usb_bus_fence_before_dma`）。
- **通道掩码**：可读 **`HCINTMSK`** 用于超时 **`USB-TOUT`** 调试行。
- **`debug_log_ep0_dma_info`**：打印 EP0 缓冲区 VA/PA、`GHWCFG2` ARCH、`GSNPSID` 等（供板级在枚举前调用）。
- **`cache.rs`**：保留 AArch64 按行 clean/invalid；**新增 `riscv64`** 路径使用 **T-Head C906 `dcache.ciall`**（全 cache 清洗+无效，与项目 `dma.md` 思路一致），供 SG2002 内部 DMA 一致性。

---

## 4. `host.rs`

- 增加 **`check_root_device_connected`**：长轮询 **`HPRT0.CONNSTS`**，失败时打印 **`LNSTS`** 并调用 **`debug_dump_root_port_hw`**。
- **`enumerate_root_port` / `enumerate_topology_only`**：`dwc2_host_init` → **连接检测** → **`dwc2_host_root_bus_reset_pulse`** → `topology::...`（总线复位从 `dwc2_host_init` 末尾 **移出**，符合「先见连接再复位」顺序）。

`dwc-rust-test` 中 `enumerate_root_port` 仅为 **`dwc2_host_init` → `topology`**，无根口连接检查与延迟复位。

---

## 5. `topology.rs` 与日志

- 输出从 **`UsbUartWriter`** 改为 **`LineBufferedUsbLog` + `usb_log_fmt`**；扫描结束处配合 **`usb_log_flush_residual`**，避免行缓冲残留。
- 其余 Hub/MSC 识别、递归端口扫描逻辑与参考实现 **同源演进**，接口上仍可向 `enumerate_root_port` 返回 MSC 四元组（若未找到 MSC 则按原错误语义）。

---

## 6. 未迁入 `usb-dwc2-host` 的 `dwc-rust-test` 内容

以下仍在 **`dwc-rust-test/`** 内，**未**作为 ArceOS workspace 成员与 `usb-dwc2-host` 依赖引入：

- **`mass_storage.rs`**、**`fat_disk.rs`**、`fatfs` 依赖及 **`firmware/examples/demos/usb_fat_host_demo.rs`** 演示主程序。
- **`hal/`** 整 crate（RPi3 板级、Mini UART 等）。
- **`tools/gen-fat-img/`** FAT 镜像生成工具。

若需在 ArceOS 上读 U 盘，需在应用或新 crate 中 **重新接线** MSC/BOT/FAT 层，并依赖本 `usb-dwc2-host` 的 EP0/拓扑 API。

---

## 7. 板级集成（`examples/helloworld`）

新增/使用 **`src/usb_host.rs`**（示意职责，非 crate 内代码）：

- **时钟**：`CLKGEN` `CLK_EN_1` bit28–31、`CLK_EN_2` bit0（对齐 Linux `clk-cv181x` 命名习惯）。
- **TOP**：USB 控制器软复位寄存器；**`TOP+0x48`** 写 **host + EXTVBUS**；**`0xB4` ECO** bit7（与既有 bring-up 笔记一致）。
- **LicheeRV Nano / DTS**：**FMUX `usb_vbus_det` → XGPIOB6**、**IOBLK 加强驱动**、**GPIO1 pin6** 作为 VBUS 相关 GPIO（极性可配）；**注意**：官方 Wiki 要求 **Host 时排针 VBUS/VSYS 外部 5V**，仅靠 Type-C 供电时下游可能无 VBUS，与 `CONNSTS=0` 现象一致。
- **`phys_to_virt` / `virt_to_phys`**：设置 `set_dwc2_base_virt`、`set_usb_dma_to_phys_fn`（SG2002 常见 **phys-virt-offset=0**）。
- 调用 **`usb_dwc2_host::dwc2::dwc2_probe`**、**`host::enumerate_topology_only`** 等。

---

## 8. 小结

- **架构**：由「RPi 专用 `hal` + 固定基址」改为 **可移植 crate**：**平台回调 + feature 分岔**。
- **硅片**：针对 **Synopsys 4.20a+ 软复位** 与 **CV182x/SG2002 UTMI/动态 FIFO/内部 DMA** 做了完整主机初始化路径。
- **行为**：根口 **连接检测**、**总线复位时机**、**PHY REG014 交给 DWC2**、**调试转储** 均为相对 `dwc-rust-test` 的增量。
- **范围**：ArceOS 侧 **刻意不包含** FAT/MSC 应用层，聚焦 **DWC2 主机枚举与拓扑**；板级供电与 Type-C 走线需按硬件文档单独满足。

---

*文档生成目的：便于审阅与后续将 MSC/FAT 或更多 SoC 迁回时对照基准。*
