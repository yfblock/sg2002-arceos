# 将 `dwc-rust-test` USB 主机驱动集成到 `examples/helloworld` 的计划

## 1. 目标

- 在 ArceOS `examples/helloworld` 启动流程中初始化 **Synopsys DWC2 USB 主机**。
- 启动后执行与 `dwc-rust-test/usb-bare-dri` 中 **`topology`** 模块等价的逻辑：**递归遍历 Hub 下游端口**，并通过串口/`println!` 输出设备树（VID/PID、端口连接状态、Hub 端口数等）。

## 2. 现有代码资产（`dwc-rust-test`）

| 组件 | 作用 |
|------|------|
| `usb-bare-dri` | `no_std` DWC2 主机栈：`dwc2`（控制器初始化）、`dwc2_ep0`（控制/批量传输）、`setup`（标准请求）、`topology`（Hub 递归枚举与打印）、`host::enumerate_root_port`（入口） |
| `hal`（同目录） | **RPi3 专用**：`rpi3::DWC2_USB_BASE`、`mmio`、`uart_mini`、`cache`（AArch64 DMA 维护；RISC-V 上为空操作） |

**拓扑与打印**：`topology.rs` 中 `visit_default_depth` 已实现根口 → Hub → 子端口的递归；`enumerate_bus_print_tree()` 在完成扫描后**若未发现 Mass Storage 会返回 `Err`**，而 helloworld 仅需「打印树」时，需要 **拆分或新增 API**（见 §4）。

Linux 中 usb 设备树代码如下：

```plain
	usb: usb@04340000 {
		compatible = "cvitek,cv182x-usb";
		reg = <0x0 0x04340000 0x0 0x10000>,
			<0x0 0x03006000 0x0 0x58>;	//USB 2.0 PHY
		dr_mode = "otg";
		g-use-dma;
		g-rx-fifo-size = <536>;
		g-np-tx-fifo-size = <32>;
		g-tx-fifo-size = <768 512 512 384 128 128>;
		clocks = <&clk CV181X_CLK_AXI4_USB>,
				<&clk CV181X_CLK_APB_USB>,
				<&clk CV181X_CLK_125M_USB>,
				<&clk CV181X_CLK_33K_USB>,
				<&clk CV181X_CLK_12M_USB>;
		clock-names = "clk_axi", "clk_apb", "clk_125m", "clk_33k", "clk_12m";
		vbus-gpio = <&portb 6 0>;
		status = "okay";
	};

```

Linux 中 usb 驱动代码，对应 `cvitek,cv182x-usb`

https://github.com/sipeed/LicheeRV-Nano-Build/blob/d4003f15b35d43ad4842f427050ab2bba0114fa5/linux_5.10/drivers/usb/dwc2/params.c#L217

## 3. 主要差距与风险

### 3.1 平台耦合

- `dwc2.rs` / `dwc2_ep0.rs` 通过 `hal::rpi3::DWC2_USB_BASE` 写死 **BCM2837 外设窗口**（`0x3F000000 + 0x980000`）。
- **helloworld 当前目标为 SG2002（RISC-V）**（`axplat-riscv64-sg2002`）。DWC2 的 **MMIO 物理基址、时钟/复位、可选 PHY/引脚** 必须与 RPi3 不同，不能直接链接现有 `hal::rpi3`。

### 3.2 日志输出

- `topology` / `uart_log` 使用 `hal::uart_mini::mini_puts`。
- helloworld 已使用 **`axstd::println!`** 与 `dw_apb_uart`。集成时应提供 **`core::fmt::Write` 适配**（或注入 writer），避免再依赖 Mini UART。

### 3.3 DMA 与缓存

- `dwc2_ep0` 使用片上 `static` 缓冲 + `HCDMA` 寄存器写入地址；`hal::cache` 在 AArch64 上做 clean/invalidate，**RISC-V 目标当前为空实现**。
- 若 SG2002 上 USB 主控 DMA 与 CPU 缓存不一致，需在 **RISC-V 侧补缓存维护**（或保证缓冲区位于 **非缓存映射** 区域——需对照 ArceOS/平台内存属性）。

### 3.4 板级外设使能

- `sg200x-bsp` 中存在 **USB IP 软复位** 等符号（如 `rstc` 中与 USB 相关位），集成前需确认：**时钟门控、复位释放、VBUS/PHY 模式** 是否与 Linux 设备树一致，否则 `dwc2_probe()` 可能读寄存器全 0 或超时。

## 4. 建议的软件架构调整

### 4.1 将 `usb-bare-dri` 变为「可移植 crate」（推荐）

在 **ArceOS 工作区内** 增加 crate（例如 `usb-dwc2-host`，或由 `helloworld` 以 `path` 依赖一份 fork 后的 `usb-bare-dri`），做最小侵入式重构：

1. **平台常量接口**  
   - 用 `trait UsbPlatform` 或 `cfg` 分模块提供：`fn dwc2_base() -> usize`（或 `PhysAddr` + `phys_to_virt`）。  
   - RPi3 保留现有地址；SG2002 在 **单独模块** 中填入从 **数据手册 / 设备树 / 厂商 BSP** 确认的基址（**此数值必须在计划中列为待验证项**，不可沿用 RPi）。

2. **日志**  
   - 将 `UsbUartWriter` 改为泛型或 `&mut dyn Write`，或提供 `set_usb_log_writer`（若需 `no_std` 且避免 alloc，可用静态 `Option<&'static mut dyn Write>` 等模式，需评估线程与生命周期）。

3. **拓扑 API 拆分**（满足「只遍历 Hub 并输出」）  
   - 新增例如 `enumerate_bus_print_tree_only() -> UsbResult<()>`：逻辑与现 `enumerate_bus_print_tree` 相同，但 **不要求 MSC**，扫描结束即 `Ok(())`。  
   - 保留原函数供 FAT/MSC 演示使用；`host::enumerate_root_port` 可改为调用带 MSC 的版本或分别暴露两个入口。

### 4.2 依赖 `fatfs`（可选）

- 若 helloworld **仅**做 Hub 树打印，可在该构建中 **用 feature 关闭** `fat_disk` / `mass_storage`，减小体积与 `git` 依赖解析成本。  
- 若后续要读 U 盘，再打开 feature 并接 `rust-fatfs`。

## 5. `examples/helloworld` 集成步骤

1. **Cargo**  
   - 在 `examples/helloworld/Cargo.toml` 增加对移植后 crate 的 `path` 依赖；如需 workspace 成员，在根 `Cargo.toml` 中注册该 crate（遵循现有 workspace 约定）。

2. **新增模块**  
   - 例如 `src/usb_host.rs`：封装  
     - 平台基址获取（`phys_to_virt` 与 `axhal::mem` 一致）；  
     - （可选）USB 时钟/复位调用 `sg200x-bsp`；  
     - 调用 `dwc2_probe` → `dwc2_host_init` → `enumerate_bus_print_tree_only`（或等价路径）。

3. **`main` 调用时机**  
   - 在 `println!("Hello, world!");` 之后、或其它外设占用相同资源之前，插入 **`usb_host::init_and_dump_topology()`**。  
   - 若 USB 与现有 UART/DMA 演示有总线竞争，**串行化**或加简单错误提示（probe 失败时打印原因）。

4. **配置**  
   - 在 `axplat-riscv64-sg2002/axconfig.toml` 或平台代码中记录 **USB MMIO 区域**（若平台模型需要），便于后续 IRQ/设备树对齐（当前枚举可仍用轮询，与 `dwc-rust-test` 一致）。

## 6. 验证策略

| 阶段 | 内容 |
|------|------|
| 寄存器级 | 上电后 `GHWCFG2/3` 非零、`HPRT0` 连接位符合线缆/Hub 状态 |
| 功能 | 日志中出现 `topology: recursive hub scan`、各 Hub 端口 `CONNECTED`/`empty`、下游设备 VID/PID |
| 回归 | 原有 helloworld 功能（相机/UART 等）在关闭 USB 初始化或 USB 失败分支时仍可运行 |

若在 QEMU 无对应 SG2002 USB 模型，**实机或厂商仿真环境**为主验证手段。

## 7. 任务清单（实施顺序）

1. ~~从数据手册/设备树 **确认 SG2002 DWC2 USB 主机 MMIO 基址与复位/时钟序列**~~（已按设备树采用 `0x04340000`；完整时钟树待实机核对）。  
2. ~~Fork/拷贝 `usb-bare-dri`，去掉对 `hal::rpi3` / `uart_mini` 的硬编码，引入 **平台抽象 + 可注入 `Write`**~~（见 workspace 内 `modules/usb-dwc2-host`）。  
3. ~~调整 `topology`：**无 MSC 时也成功返回** 的 API~~（`topology::enumerate_bus_print_tree_only` + `host::enumerate_topology_only`）。  
4. ~~在 `helloworld` 中接线 `phys_to_virt`、可选 `rstc`/`clk`，在 `main` 调用拓扑打印~~（`src/usb_host.rs`，`main` 在 `Hello` 后调用）。  
5. **待办**：实机插拔 Hub/U 盘验证；若 DMA 与缓存不一致，在 RISC-V 上补 `cache` 或非常规映射缓冲区。

## 8. 交付物

- 可编译运行的 `arceos-helloworld`，启动后打印 **Hub 递归设备树**。  
- 移植后的 USB crate 与清晰的分层（平台 / 控制器 / 拓扑），便于后续接 MSC、HID 等。

---

## 9. 实施记录（与代码同步）

| 项 | 说明 |
|----|------|
| Crate | `modules/usb-dwc2-host`（workspace 成员），`#![no_std]`，自含 `mmio` / `cache`（AArch64 按行；`riscv64` 为 C906 全 cache 维护） |
| 基址 | 运行时 `platform::set_dwc2_base_virt(phys_to_virt(0x04340000))`（与 §2 设备树一致） |
| 复位 / TOP | `usb_host`：`TOP+0x3000` USB 位软复位脉冲 + `TOP+0x48` PHY（主机 ID）+ `TOP+0xB4` ECO（对齐 U-Boot `cvi_usb_hw_init`） |
| 时钟 | `usb_host`：`0x03002000` 上 `CLK_EN_1` bit28–31、`CLK_EN_2` bit0（对齐 Linux `clk-cv181x.c` USB 门控） |
| D-Cache | `usb-dwc2-host`：`target_arch=riscv64` 时在 DMA 前后执行 C906 `dcache.ciall`（修复 EP0 `Timeout`） |
| `cv182x-host` | Cargo feature：DWC2 主机 init 对齐 Linux `dwc2_set_cv182x_params`（见上节链接）；`helloworld` 已启用 |
| 日志 | `log::set_usb_log_fn` + `LineBufferedUsbLog`（Rust 2024 下避免 `static_mut` 共享引用问题，行缓冲容量 `LOG_CAP`） |
| 应用入口 | `examples/helloworld/src/usb_host.rs::init_and_dump_topology`，`main` 中先于 `dma_uart_tx` 调用 |
| 编译验证 | `make MYPLAT=axplat-riscv64-sg2002 build` 已通过（生成 `helloworld_sg2002.elf` / `.bin`） |

**已补充（针对实机 `Timeout`）**  
- **D-Cache**：`usb-dwc2-host` 在 `riscv64` 上对 DMA 区使用 T-Head C906 `dcache.ciall`（与 `dma.md` 一致），否则 EP0 通道会无限等 `XFERCOMPL`。  
- **时钟 + TOP**：`usb_host` 依 `clk-cv181x.c` 置位 `CLK_EN_1` bit28–31 与 `CLK_EN_2` bit0；依 U-Boot `mars/board.c` 做 USB 软复位脉冲、`TOP+0x48` PHY（主机 ID 覆盖）、`TOP+0xB4` ECO。  
- **DWC2 与 Linux `cvitek,cv182x-usb` 对齐**：`helloworld` 依赖 `usb-dwc2-host` 的 **`cv182x-host`** feature。`dwc2_host_init` 在该模式下对齐 Sipeed 树中 [`params.c` 的 `dwc2_set_cv182x_params`](https://github.com/sipeed/LicheeRV-Nano-Build/blob/d4003f15b35d43ad4842f427050ab2bba0114fa5/linux_5.10/drivers/usb/dwc2/params.c#L217) 及 `dwc2_core_host_init` / `dwc2_config_fifos`：**UTMI 16-bit**（`GUSBCFG_PHYIF16`、清 `ULPI_UTMI_SEL`）、**`TOUTCAL=7`**、**`PCGCTL=0`** 恢复 PHY 时钟、**`GAHBCFG` HBSTLEN=INCR16 + DMA**、**高速主机下清除 `HCFG_FSLSSUPP`**（此前误按 FS/LS 配置会导致枚举超时）、按 `GHWCFG3` 总深度配置 **动态 RX/NPTX/PTX FIFO**（默认 536/32/768，过大时按 Linux `dwc2_calculate_dynamic_fifo` 收缩）、满足 `GSNPSID`/`GHWCFG4` 时写 **`GDFIFOCFG`**，并 **flush 全部 TX / RX FIFO**。  
- **OTG 会话**：在 `cv182x-host` 下于配 `GUSBCFG` 前写 **`GOTGCTL`**：使能 `AVAL`/`VBVAL` override 并置位 valid + `DBNCE_FLTR_BYPASS`（对齐 Linux `dwc2_ovr_avalid` 一类软件会话，避免根口无电气状态）。  
- **`HCDMA` 地址**：`platform::set_usb_dma_to_phys_fn` + `axhal::virt_to_phys`，保证 EP0 静态缓冲写入 DMA 的为 **物理地址**（与 `dma_uart_tx` 一致）。  
- **根口检测**：`host::enumerate_*` 在发 EP0 前若 **`HPRT0 CONNSTS=0`** 直接返回 `Hardware(...)`，区分「未接设备」与传输层 `Timeout`。  

**仍可能需核对**：第二路寄存器 `0x03006000`（USB 2.0 PHY）、`vbus-gpio`、以及 SoC 若与 CV181x 时钟位不完全一致时的门控位调整。

---

*本文档仅描述计划；具体寄存器地址与时钟位以 SG2002 官方文档及你当前 BSP 为准。*
