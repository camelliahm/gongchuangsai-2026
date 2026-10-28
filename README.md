# STM32F407 CAN 底盘固件

四轮麦轮小车固件。主控 STM32F407VETx，FreeRTOS，通过 CAN1 与 Emm_V5 / ZDT（张大头）
步进闭环驱动器通信，并同时在该总线上接收上位机指令。

> 本文只讲**代码架构**：分层、模块契约、消息通路、状态机、协议、扩展落点。
> 代码注释只讲"这个函数干什么、参数怎么传、有什么坑"；模块定位、分层规则、扩展指引这类说明性内容写在这里。

---

## 0. 构建（三条链路）

三套配置互相独立，任一套能编不代表另两套能编。

| 链路 | 配置文件 | 编译器 | 产物 |
| --- | --- | --- | --- |
| Keil | `PRJ/STM32_CAN_CMD.uvprojx` | ARMCC 5 | `PRJ/Objects/STM32F4.axf` |
| EIDE | `PRJ/.eide/eide.yml` | arm-none-eabi-gcc | `PRJ/build/V1.0/` |
| **GCC + CMake** | 根目录 `CMakeLists.txt` + `CMakePresets.json` | arm-none-eabi-gcc | `build-gcc/{Debug,Release}/` |

```bash
# Debug：-Og -g3 -gdwarf-2，日常开发与单步调试
cmake --preset debug   && cmake --build --preset debug
# Release：-Os，体积优先
cmake --preset release && cmake --build --preset release
```

工具链默认取 `C:/Program Files (x86)/Arm/GNU Toolchain mingw-w64-i686-arm-none-eabi/bin`，
换机器或换系统用 `-DARM_TOOLCHAIN_DIR=<工具链 bin 目录>` 覆盖；Linux 下留空即从 PATH 查找。

CMake 侧的支撑文件都在 `cmake/`：`arm-none-eabi.cmake`（工具链）、`STM32F407VETx_FLASH.ld`（链接脚本）、
`startup_stm32f40_41xxx.S`（GCC 语法启动文件，向量表逐项对齐 ARMCC 版）、`syscalls.c`（newlib 裸机桩）。

三处**只能在 CMake 侧**处理的点：

- `-funsigned-char`：ARMCC 的 `char` 默认无符号，GCC 默认有符号。不加这条，`oled_font` 与字符串处理会出现行为差异。
- `-lm` 必须走 `target_link_libraries`：放进 `target_link_options` 会排在目标文件之前，
  ld 单遍扫描时 `cosf`/`sinf` 尚未被引用，`libm` 成员不会被拉入 → `undefined reference`。
- 链接脚本**不能**保留 ST 模板的 `/DISCARD/ { libm.a(*) }`：那条会把整个 `libm` 丢掉，同样导致 `cosf` 未定义。

新增 `.c` 文件时除 Keil/EIDE 的四处登记外，还要加进 `CMakeLists.txt` 对应的 `*_SOURCES` 列表。

---

## 1. 分层结构

| 文件 | 职责 | 不该放什么 |
| --- | --- | --- |
| `CONTRACT/contract.h` | **公共契约层**：消息码分段、地址宏、状态/事件枚举、`Chassis_t`、任务参数、`contract_*` 接口 | 各层私有类型与宏 |
| `CONTRACT/contract.c` | **状态机 + 系统级调度**：迁移表、`contract_OnEnter`、消息路由派发、三个任务体、控制环编排 | 任何控制算法 |
| `CONTRACT/oled.c/.h`<br>`CONTRACT/oled_font.h` | **显示层**：SSD1306 128x64 状态屏（软件 I2C，只读黑板，不参与控制） | 状态机迁移、任何控制逻辑 |
| `CONTRACT/PID_Mod.c/.h` | **通用算法层**：PID 控制器（外部移植，只引用不修改） | 状态机迁移、硬件访问 |
| `DEVICE/chassis_drive.c/.h` | **底盘驱动层**：四轮动作 / 反馈 / 看门狗 / 停稳判断 + 里程计 / 麦轮逆解 / 姿态环 | 状态机迁移、升降轴 |
| `DEVICE/lift_axis.c/.h` | **升降轴层**：T8 丝杆 + Emm_V5 步进闭环（CAN `0x05`）的指令下发、反馈解析、看门狗与到位判断 | 状态机迁移、底盘四轮 |
| `DEVICE/grip_seq.c/.h` | **夹持机构层**：旋转轴 / 夹子 / 车载物料盘三路舵机 + **装载 / 卸载 / 取回**三条动作序列（`GripPhase_t` 状态机） | 状态机迁移、底盘四轮 |
| `DEVICE/input.c/.h` | **输入汇聚层**：三条串口的硬件配置、DMA 与字节管道（USART1 视觉1 / USART3 视觉2 / USART2 串口屏）+ **摁键（自启动，板内自举的唯一入口）** | 帧解析、执行机构、状态机迁移 |
| `DEVICE/vision.c/.h` | **视觉层**：两个视觉模块的帧解析（同一种匿名协议精简帧；视觉1 用 `0xF5` 报物块颜色 + 对准相对距离，视觉2 用 `0xF3` / `0xF4`） | 串口硬件、执行机构、状态机迁移 |
| `DEVICE/servo.c/.h` | **执行机构（PWM）**：3 路 50Hz 舵机（TIM3 CH1/CH2/CH3 = PC6/PC7/PC8）；**角度语义由 `grip_seq.c` 的夹持机构定义**（CH1 = 旋转轴、CH2 = 夹子、CH3 = 车载物料盘）；量程按通道给（CH3 是 270° 舵机） | 状态机迁移、底盘四轮、舵机角度语义 |
| `APP_LOG/nav.c/.h` | **编排层（任务层）· 调度与入口**：航段序列驱动、段起点与判据复位、超时重试、对外接口（引擎部分） | 段内控制律、地图数据 |
| `APP_LOG/nav_priv.h` | **nav 内部头**：`s_nav` 状态结构、内部宏、跨文件函数原型（仅供 `nav*.c` 使用） | 对外接口（在 `nav.h`） |
| `APP_LOG/nav_seg.c` | **编排层 · 航段实现**：平移 / 横移到位判据、对准段、装载段、取放段 | 状态机迁移、地图数据 |
| `APP_LOG/nav_align.c` | **编排层 · 对准与挑色**：视觉1 闭环对准、任务码解析、目标色 / 位号查询 | 状态机迁移、直接下发 CAN |
| `APP_LOG/nav_util.c` | **编排层 · 工具与读接口**：角度折算、绝对值、`nav_Load` / `nav_GetState` / `nav_GetStats` / `nav_GetTaskCode` 等只读接口 | 控制律、状态迁移 |
| `APP_LOG/nav_map.h` | **场地地图**：初赛赛场的几何 / 区域 / 停靠点坐标 + **默认路线表**（纯数据，无逻辑） | 任何逻辑代码、里程计状态 |
| `APP_LOG/avoid.c/.h` | **避障层**：读视觉2 的障碍横向偏移，**在 `nav_Step()` 之后**往黑板 `chassis.vx` 上叠加让行分量（只叠加不覆盖） | 状态机迁移、航段序列、直接下发 CAN |
| `IFACE/auxiliary.c/.h` | **辅助机构接入层**：对上层只暴露 `aux_Init()` / `aux_HandleCmd()` / `aux_Step()` 三个入口，对内聚合 `DEVICE/` 下机构层；`CFG_HOST_CAN_ENABLED` 的指令闸门也在此 | 状态机迁移、底盘四轮 |
| `IFACE/screen.c/.h` | **串口屏协议层**：TJC（淘晶驰）"ASCII 指令 + `FF FF FF`"的组帧、发送与事件解析；USART2 硬件配置在 `input.c` | 其他串口通道、状态机迁移 |

> **目录约定**：自研代码按四层分目录 —— `CONTRACT/`（契约 / 调度 / 显示 / 通用算法）、`DEVICE/`（硬件抽象与机构执行）、
> `APP_LOG/`（应用决策，即虚拟上位机 `nav` / 避障层 `avoid`）、`IFACE/`（接入与人机：`auxiliary` / `screen`）。
> `APP/` 只保留 `main.c` 与 ST 模板（`stm32f4xx_conf.h`、`stm32f4xx_it.c/.h`）。

三层之间**不直接互相调用**，通过两条通道交互：

- **向上报事件**：`contract_PostEvent(EV_FAULT, ...)` 投给状态机
- **向下读状态**：只读 `chassis.state`

---

## 2. 模块间通信：模拟 ROS 方案

**一句话：用「消息」连接模块，而不是用函数调用。**

这不是要把 ROS 搬进单片机，而是把「**统一接口形状 + 一张注册表**」定为新增模块的标准接入方式。
目的只有两个：**加模块不用改已有模块；多人可以按冻结的契约并行开发。**

> **现状**：接口形状、**路由注册表**（L2）与**编排层 `nav`** 均已就位，`nav` 已接入控制环。

### 2.1 五个要素

| 要素 | 内容 | 现状 |
| --- | --- | --- |
| **契约** | 地址宏、消息码枚举、消息结构体 —— 集中在 `CONTRACT/contract.h` | 已有 |
| **节点** | 每个机构 = 一个 `xxx_HandleCmd()` + 一个 `xxx_Step()`，**不新开任务** | 已有（`auxiliary.c` 的 `aux_HandleCmd()` / `aux_Step()` 就是这个形状） |
| **路由** | 一张 `{ 消息码段 → handler }` 注册表 | **已有**（`contract.c` 的 `s_routeTable[]`，`contract_HandleCmd()` 按段查表派发） |
| **黑板** | 全局 `chassis` 结构体：编排层写目标，模块写状态 | 已有 |
| **编排** | 任务层 `nav`：决定机构的组合与先后顺序 | **已建**（已接入控制环） |

### 2.2 消息码分段

沿用现有功能码（在代码与本节中统一称"消息码"），但**按段划分归属**：

| 段 | 归属 |
| --- | --- |
| `0x00` ~ `0x0F` | 系统（`CMD_ENABLE` / `CMD_START` / `CMD_STOP` / `CMD_ESTOP` / `CMD_CLEAR_FAULT`） |
| `0x10` ~ `0x1F` | 底盘（`CMD_SET_VEL`） |
| `0x20` ~ `0x2F` | 升降轴（`CMD_LIFT_*`） |
| `0x30` ~ `0x3F` | 夹持机构（`CMD_GRIP_*`，走舵机） |
| `0x40` ~ `0x4F` | 任务层 / 地图（规划） |
| `0x50` ~ `0x5F` | 视觉 / 二维码（规划） |

分段的意义：新模块申请一个段就不会与别人撞码；路由表按段匹配，一张表即可路由到所有模块。

### 2.3 投递：双模，同步为主

| 场景 | 机制 | 延迟量级 |
| --- | --- | --- |
| 同任务、同控制周期内 | **同步派发**（直接调用 handler） | 亚微秒 |
| 跨执行上下文（CAN 接收任务 → 状态机任务） | **RTOS 队列** | 见下 |

**为什么这里的异步队列几乎不慢**（这是"能不能用异步"的依据）：

- 载荷只有几个字节，`xQueueSend` 的拷贝成本是几十个周期；
- 本工程所有投递都是"高优先级 → 低优先级"（`CAN_RX`(3)→`STATE`(2)、`CTRL`(4)→`STATE`(2)），
  `xQueueSend` **不会**触发立即切换；
- 因此延迟上界是 **发送方本周期的剩余执行时间**，而不是 1 个 tick（1 ms）：
  `CAN_RX` 投完立刻回去阻塞 → 几十 µs；`CTRL` 投完跑完本周期 → 百 µs 量级；
- 即便退到最坏的 1 个 tick，**事件层是低频语义事件**（`EV_ENABLE` / `EV_FAULT` / `EV_ARRIVED`），
  1 ms 抖动无害 —— 它**不落在 5 ms 控制环的数据通路上**。

> 结论：**异步不是需要回避的成本**。但仍建议"同步为主"——同步可以直接读调用栈调试，
> 异步只在"跨越执行上下文"这一种情况下才别无选择。

### 2.4 六条边界

1. **总线只传请求，不绕过安全闸门。** handler 可以拒绝请求；"允不允许动"始终由状态机决定
   （`CMD_SET_VEL` 被 `RUNNING` 闸住就是范例）。
2. **不新增任务。** 一个控制任务按序 tick 各机构 —— `contract_ControlStep()` 已经是这个形状。
   16 KB 堆撑不住 N 个机构各一个任务。
3. **同步派发必须同一拍内处理完**，不许留待下一拍。
4. **持续状态走黑板，不走消息。** 位姿、位置、电机状态这类"随时可读"的量放结构体；
   消息只承载"瞬时意图"。混用会让队列要么丢帧、要么被状态刷新刷爆。
5. **消息载荷用结构体，不用裸字节。** CAN 帧在入口解析/校验后转成结构体再派发，
   让编译器帮忙查参数错误。
6. **不做动态分配。** 消息是定长副本、值传递，不存在"谁申请谁释放"的问题。

### 2.5 新增模块的四个动作

1. **申请消息码** —— 在对应段里加一个枚举（`contract.h`）
2. **实现 handler + step** —— `xxx_HandleCmd()` / `xxx_Step()`，**新建设备模块文件**（`DEVICE/` 下），再由 `IFACE/auxiliary.c` 这类接入层聚合
3. **注册到路由表** —— 在 `contract.c` 的 `s_routeTable[]` 加一行（段基址 + 段掩码 + 该模块的 handler）；不属于任何已注册段的消息会被忽略并累加 `chassis.evtIgnored`
4. **在 `contract_OnEnter()` 登记状态动作** —— IDLE 失能、FAULT/ESTOP 安全停
   （**与接口无关，这一步一直都要做**）

### 2.6 收益与代价（如实）

**收益**

- 新增模块**不改已有模块**：加一行注册，而不是改一处 `switch` / `else if`；
- **多人可并行**：冻结契约后各自实现自己的 handler，互不阻塞；
- 接口形状统一，新人看一个例子就能照做。

**代价**

- 调试路径变长：调用栈多跨一层派发器；
- 跨任务的那部分时序不可复现（同步部分不受影响）；
- **注册表要维护**：漏注册 = 模块静默不响应 —— 建议注册表带计数校验或逐个断言；
- 若用裸字节传参会丢失编译期类型检查 —— 因此第 5 条边界要求用结构体。

> **诚实的边界**：这套方案买到的便利集中在「**加模块**」和「**多人分工**」，
> 它不会让单个模块的调试变简单，也不会提升运行时性能。

### 2.7 与真 ROS 的差别（划清边界）

| | 真 ROS / ROS 2 | 本方案 |
| --- | --- | --- |
| 传输 | DDS over UDP | 直接函数调用 + 少量 RTOS 队列 |
| 节点发现 | 运行期动态发现 | 编译期静态注册 |
| 序列化 | IDL 生成 | 不需要（同一地址空间） |
| 跨节点 | 支持多机 | 仅单芯片 |
| 服务 / RPC | 有 | 无 |

**所以本方案的本质是"统一接口形状 + 一张注册表"，不是"把 ROS 搬进单片机"。**

### 2.8 演进路线

| 级 | 内容 | 触发条件 / 状态 |
| --- | --- | --- |
| L1 | 接口形状（handler + step） | 已具备 —— 每个机构一个 handler 一个周期函数 |
| L2 | 注册表 + 同步派发 | **现状** —— `s_routeTable[]` 已建（系统 / 底盘 / 升降轴 / 夹持机构 **4 段**，全部指向已有模块） |
| L3 | 异步主题（队列 + 多订阅者） | 出现真正的多对多（如视觉反过来订阅底盘位姿做闭环） |

---

## 3. 启动与任务

`main()` 只做两件事：集中初始化 + 启动调度器，不含业务逻辑。

```
ALL_Init()            ① 全部初始化（顺序见下表）
vTaskStartScheduler() ② 启动调度器（不返回）
```

`ALL_Init()` 内部按依赖顺序展开：

| 序 | 调用 | 作用 |
| --- | --- | --- |
| 1 | `board_init()` | 硬件初始化（NVIC、时钟、GPIO、CAN） |
| 2 | `servo_Init()` | 三路舵机 PWM（TIM3 CH1~CH3）— 三路语义都已由夹持机构下发（旋转轴 / 夹子 / 车载物料盘） |
| 3 | `input_Init()` | 三条串口（USART1 视觉1 / USART3 视觉2 / USART2 串口屏）+ 摁键 GPIO；**只配硬件与字节管道，不做任何帧解析** — 三条通道的解析都已接入（`vision1_Step()` / `vision_Step()` / `screen_Step()`） |
| 4 | `vision_Init()` | 视觉 1 / 2 的帧槽位与组帧状态归零（须在 `input_Init()` 之后） |
| 5 | `screen_Init()` | 串口屏上报事件槽位与收帧状态归零（USART2 硬件在 `input_Init()`） |
| 6 | `oled_Init()` | OLED 状态屏（软件 I2C）；周期刷新由 STATE 任务调 `oled_Step()` |
| 7 | `nav_Init()` | 编排层：装载默认地图，不自动启动（启动由进入 RUNNING 触发） |
| 8 | `can_RtosInit()` | 建 CAN 接收队列 + 发送互斥锁 |
| 9 | `contract_Init()` | 建事件队列 → `chassis_DriveInit()` + `aux_Init()` + `avoid_Init()` |
| 10 | `App_TaskCreate()` | 按任务表 `s_tasks[]` 建任务 |

第 1 项须最先执行 —— 后序模块的时钟与 GPIO 由它打开；第 8~10 项须在建任务之前
—— 任务一起来就立刻用队列和互斥锁。

| 任务 | 优先级 | 栈（字） | 入口 | 职责 |
| --- | --- | --- | --- | --- |
| `CTRL` | 4 | 512 | `contract_CtrlTask` | 5 ms 绝对节拍，每拍调 `contract_ControlStep()` |
| `CAN_RX` | 3 | 256 | `contract_CanRecvTask` | 阻塞 `can_RecvFrame()` 取帧 → `contract_HandleFrame()` 分流 |
| `STATE` | 2 | 256 | `contract_StateTask` | 带超时等事件 → `contract_Step()` 查表迁移；超时走 `contract_UiStep()` 刷屏 + `oled_Step()` 刷状态页（本任务兼作低速后台） |

急停为什么快：`CAN_RX` 优先级高于 `STATE`，帧一到就抢占运行，投完事件 `STATE`
立即就绪；`CTRL` 每 5 ms 只占极短时间。最坏等待 = 一次控制脉冲（几十 µs 量级）。

控制环周期定 5 ms（200 Hz）而非 1 ms：四轮闭环在驱动器内部完成，MCU 无需高频闭合；
且 1 ms × 4 台电机 = 4000 帧/s，已超出 500 kbit/s 总线约 3817 帧/s 的物理上限。
**改周期必须同步重算总线负载并重新整定 PID。**

---

## 4. CAN 地址分配

遵循 ZDT 帧布局：`ExtId[28:8]` = 地址，`ExtId[7:0]` = 包序号。

| 地址 | 归属 | 处理 |
| --- | --- | --- |
| `0x01` ~ `0x04` | 四台 Emm_V5 驱动器（麦轮） | `chassis_HandleFeedback()`，更新电机槽位 |
| `0x05` | 升降轴驱动器（丝杆） | `aux_HandleLiftFeedback()` |
| `0xFE` | 本机（上位机发给底盘的指令帧） | `contract_HandleCmd()` |
| `0x00` | ZDT **广播地址** | **禁止占用**，收到一律忽略 |
| 其它 | 无关帧 | 忽略 |

> **本表只描述"挂在 CAN1 上"的设备。** 目前只有步进类（四轮 + 升降丝杆）走 CAN。
> 后续若新增非 CAN 机构（PWM / GPIO / 串口），不占这里的地址，也不进
> `contract_HandleFrame()` 的分流。

**升降轴为什么挂在同一条总线上**：四轮下发 800 帧/s + 反馈 200 帧/s，加升降轴约
225 帧/s，合计约 1225 帧/s ≈ 32%，远低于物理上限，无需第二路 CAN 分流。上 CAN2 要改
`can.c` 里全部硬编码的 `CAN1`、划分过滤器 bank、加第二路收发器与终端电阻，收益仅为
"物理隔离"，不划算。**若日后升降大功率动作导致总线误码（观察 `can_TxDropCount` 异常
增长），再考虑拆分独立总线。**

### 消息码表（`Data[0]` = 消息码，`Data[1..]` = 参数；即原"上位机指令功能码"）

| 功能码 | 名称 | 参数 | 效果 |
| --- | --- | --- | --- |
| `0x01` | `CMD_ENABLE` | 无 | IDLE → READY |
| `0x02` | `CMD_DISABLE` | 无 | 任意 → IDLE |
| `0x03` | `CMD_START` | 无 | READY → RUNNING |
| `0x04` | `CMD_STOP` | 无 | RUNNING → BRAKING |
| `0x05` | `CMD_ESTOP` | 无 | 任意 → ESTOP（最高优先） |
| `0x06` | `CMD_CLEAR_FAULT` | 无 | FAULT → READY；ESTOP → IDLE |
| `0x10` | `CMD_SET_VEL` | `vx, vy, w` 各 int16 大端（6 字节，DLC ≥ 7） | 仅 RUNNING 下生效 |
| `0x20` | `CMD_LIFT_POS` | 见下 | 升降轴相对位移（**脉冲**单位，单帧上限 ≈ 41 mm） |
| `0x21` | `CMD_LIFT_HOME` | 无 | 触发回零（模式 3，限位开关） |
| `0x22` | `CMD_LIFT_STOP` | 无 | 立即停止升降轴 |
| `0x23` | `CMD_LIFT_SETZERO` | 无 | 当前位置清零（需先卸负载） |
| `0x24` | `CMD_LIFT_CLEARF` | 无 | 解除升降轴堵转保护 |
| `0x25` | `CMD_LIFT_MM` | 同 `0x20`，位移量单位换成 **0.1 mm** | **推荐上位机用这条**：单帧可到 6553.5 mm，目标高度自动夹到行程内 |
| `0x30` | `CMD_GRIP_LOAD` | 无 | 跑一轮装载（取料 → 转 180° → 落盘 → 回 90°）；前置条件见下 |
| `0x31` | `CMD_GRIP_ABORT` | 无 | 中止装载动作序列（保持当前姿态）；任何状态受理 |
| `0x32` | `CMD_GRIP_SAFE` | 无 | 安全姿态（旋转 90° + 夹子张开 + 中止序列）；任何状态受理 |
| `0x33` / `0x34` | `CMD_GRIP_OPEN` / `CMD_GRIP_CLOSE` | 无 | 夹子张开 / 夹紧（标定用） |
| `0x35` / `0x36` / `0x37` | `CMD_GRIP_ROTPICK` / `ROTPLACE` / `ROTSAFE` | 无 | 旋转轴 → 取料位(0°) / 落盘位(180°) / 安全位(90°)（标定用） |

> 夹持机构的**非安全指令只在 READY / RUNNING 下受理**（`0x31` / `0x32` 除外），
> 与升降轴同一条闸门；`CMD_GRIP_LOAD` 另需 `lift.homed = 1`，否则拒绝并计入 `evtIgnored`。

`CMD_LIFT_POS`(0x20) / `CMD_LIFT_MM`(0x25) 帧格式（DLC = 9，单帧塞满，无需分包）：

```
Data[0]     功能码 0x20 / 0x25
Data[1]     方向：0 = 上升(正)  1 = 下降(负)
Data[2..3]  位移量（uint16 大端）
            0x20 → 单位「脉冲」；0x25 → 单位「0.1 mm」(120 = 12.0 mm)
Data[4..5]  速度（RPM，uint16 大端）
Data[6]     加速度（0-255，0 = 直接启动）
Data[7]     保留填 0
```

位移量是**相对当前位置**而非绝对位置 —— 这样 8 字节恰好塞进单帧，避开
`Emm_V5` 多包分包的字节序风险。

> **⚠ 为什么要有 0x25**：导程 2 mm + 16 细分 = 1600 脉冲/mm，`0x20` 的 uint16
> 脉冲字段单帧最多 `65535` 脉冲 ≈ **41 mm**，走满行程要连发好几条。`0x25` 把单位
> 换成 0.1 mm，一次到位。

**C 侧控制接口**（`DEVICE/lift_axis.c`，两套并存）：

| 函数 | 单位 | 说明 |
| --- | --- | --- |
| `lift_MoveToMM(mm, vel, acc)` | mm | 升降到绝对高度（0 = 原点，向上为正），目标夹到 `[0, LIFT_STROKE_MM]` |
| `lift_MoveByMM(deltaMM, vel, acc)` | mm | 相对当前高度升降（正升 / 负降） |
| `lift_GetHeightMM()` | mm | 读当前高度 |
| `lift_MoveTo(clk, dir, vel, acc)` / `lift_MoveBy(delta, vel, acc)` | 脉冲 | 底层接口，mm 接口内部换算后转调它们 |

### 升降轴参数与换算（`lift_axis.h`）

| 字段 / 宏 | 含义 |
| --- | --- |
| `lift.online` | 反馈未超时（`LIFT_FB_TIMEOUT_MS = 200 ms`） |
| `lift.homed` / `lift.homing` | 本次上电已完成回零或软件标定原点 / 回零进行中（收到 `S_OFLAG`(`0x3B`) 且值为 0 时清除并置 `homed`） |
| `lift.moving` | 已下发位置指令且尚未判到位 |
| `lift.pos` / `lift.target` | 实时位置 / 目标位置（`int32_t`，脉冲，**有符号**） |
| `LIFT_FB_PERIOD_MS = 50` | 位置上报周期（比四轮慢，丝杆响应本就慢，省带宽） |
| `LIFT_HOME_TIMEOUT_MS = 15000` | 回零总超时，防止限位开关失效时一直撞到底 |
| `LIFT_SCREW_LEAD_MM = 2.0f`（**已实测确认**）/ `LIFT_SCREW_MICROSTEP = 16`（**占位**）/ `LIFT_SCREW_STEP_PER_REV = 200` | T8 丝杆规格：导程 **2 mm/圈**、驱动器细分、整步数/圈 |
| `LIFT_PULSE_PER_MM` | = `200 × 细分 / 导程` = `200 × 16 / 2` = **1600 脉冲/mm** |
| `LIFT_MM_TO_PULSE(mm)` / `LIFT_PULSE_TO_MM(clk)` | mm ↔ 脉冲 换算宏（四舍五入到整脉冲） |
| `LIFT_RPM_TO_MMPS(rpm)` / `LIFT_MMPS_TO_RPM(mmps)` | RPM ↔ 直线速度(mm/s)：`mm/s = RPM / 60 × 导程`。默认 300 RPM = 10 mm/s |
| `LIFT_STROKE_MM = 300.0f` | 丝杆有效行程(mm)，**⚠ 占位待实测**；mm 接口会把目标高度夹到 `[0, 行程]` |
| `LIFT_ARRIVE_LEN_MM = 0.05` / `LIFT_ARRIVE_WINDOW` | 到位窗口：先给物理位移(mm)，再 × `LIFT_PULSE_PER_MM` 换算成脉冲（= **80**） |
| `LIFT_ARRIVE_VEL_TH = 30` / `LIFT_ARRIVE_HOLD_MS = 200` | 到位判据：转速阈值(RPM) + 持续时长 |

- 速度想按 mm/s 给，用 `LIFT_MMPS_TO_RPM(10)` 换算（导程 2 mm 时 10 mm/s = 300 RPM）。
- **夹行程只在 `homed = 1`（回零或 `lift_SetZero()` 之后）才有意义**——未回零时零点不在行程最低点。
- 对外接口（均在 `lift_axis.h` 声明，内部统一用 `can_TxLock/Unlock` 包住发送）：
  `lift_SetEnable()`、`lift_EnableFeedback()`、`lift_ClearOnline()`、`lift_MoveTo()`、`lift_MoveBy()`、
  `lift_MoveToMM()`、`lift_MoveByMM()`、`lift_GetHeightMM()`、`lift_Home()`、`lift_StopNow()`、
  `lift_SetZero()`、`lift_ClearClog()`、`lift_IsMoving()`。

### 夹持机构（`grip_seq.c`，旋转轴 + 夹子 + 车载物料盘）

**无水平伸缩机构**：水平那一维没有机构可挂，只能由**底盘全向移动 + 视觉1 距离闭环**代偿
（落在编排层的对准段）；机构侧只有**升降**（复用升降轴）与**180° 旋转**两个自由度。

| 项 | 取值 / 说明 |
| --- | --- |
| 旋转轴 | `servo_SetAngle(0, …)` = TIM3 **CH1**（PC6） |
| 夹子 | `servo_SetAngle(1, …)` = TIM3 **CH2**（PC7） |
| 端点角 | `GRIP_ROT_PICK_DEG = 0°`（取料位，朝原料区）、`GRIP_ROT_PLACE_DEG = 180°`（落盘位，朝车上物料盘） |
| 安全位 | `GRIP_ROT_SAFE_DEG = 90°`（两工位中间）——上电姿态与**急停 / 故障 / IDLE 姿态** |
| 斜坡 | 两轴**都必须限速**：`grip_RotTo()` / `grip_TrayTo()` 只记目标，`rot_Step()` / `tray_Step()` 每控制周期各逼近一步（`GRIP_ROT_SLEW_DEG` / `GRIP_TRAY_SLEW_DEG`，占位 1.0°/周期 → 180° 约 0.9 s、240° 约 1.2 s），**不可一次给大目标差** |
| 车载物料盘 | `servo_SetAngle(2, …)` = TIM3 **CH3**（PC8），**270° 舵机** —— 量程按通道单独标定（`servo.h` 的 `SERVO_*_CH3`），否则 240° 会被截断到 180° |
| 三个槽位 | `GRIP_TRAY_POS1/2/3_DEG = 0° / 120° / 240°`；**槽位角即指令角**，机械上把槽位 k 装成该角正对落料点 —— 任务码顺序第 k 位就落进槽位 k |
| 换批复位 | 装满 `GRIP_LOAD_TOTAL` 轮后自动转回位置一（`GRIP_TRAY_HOME` 阶段）；装载段入口的 `grip_ResetLoad()` 也会顺带归位 |

**一轮装载动作序列**（`grip_LoadOnce()` 启动，`grip_Step()` 由 `aux_Step()` 每 5 ms 推进）：

```
旋转→取料位(0°)  下降→抓取高度  夹紧  抬起→安全高度
   → 旋转→落盘位(180°) ‖ 物料盘→本轮槽位   下降→落盘高度  松开  升高让开料盘  旋转→安全位(90°)
   （第 3 轮装完再多一步：物料盘→位置一）
```

- 每个升降阶段等 `lift_IsMoving()` 归零，每个旋转阶段等角度进入 `GRIP_ROT_TOL_DEG`，
  **两者都有超时兜底**（`GRIP_MOVE_TIMEOUT_MS` / `GRIP_ROT_TIMEOUT_MS`），超时置 `grip.fault` 并中止。
- **落盘旋转是两轴并行**：进入该阶段时旋转轴与物料盘**同时起步**，各自按自己的斜坡逼近；
  该阶段的判据要求**两轴都到位**才允许下降放料。
- **物料盘不在 `grip_Safe()` 内**：盘上可能已有物料，急停时转动会把它们甩出去，故安全姿态只动旋转轴与夹子。
- 一轮结束 `grip.loadCount++`；**共 `GRIP_LOAD_TOTAL = 3` 轮**，全部完成才 `grip_AllLoaded()`。
- 每轮都回到安全位（90°），下一轮从 90° 再回取料位 —— 与"车停原处、每轮视觉微调后取"一致。
- **序列 ≠ 编排**：本层只管机构动作，不含底盘移动，也不判断"什么时候该装"；
  对准与"三轮何时结束"由编排层的 `NAV_SEG_ALIGN` / `NAV_SEG_LOAD` 决定（见 §6）。

**卸载 / 取回动作序列**（`grip_UnloadOnce(slot)` / `grip_ReloadOnce(slot)`）：

| 序列 | 方向 | 动作链 |
| --- | --- | --- |
| 卸载 | 物料盘 → 车外目标区 | 转车内位(180°) ‖ 物料盘→槽位 → 下降→槽内高度 → 夹紧 → 抬升 → 转车外位(0°) → 下降→放置高度 → 松开 → 抬升 → 回安全位 |
| 取回 | 车外目标区 → 物料盘 | **与装载同向、复用装载的阶段**：转车外位(0°) → 下降→圆环取料高度 → 夹紧 → 抬升 → 转车内位(180°) ‖ 物料盘→槽位 → 下降→落盘高度 → 松开 → 抬升 → 回安全位 |

- 两者都**复用 0° / 180° 两个端点角**，只是角色对调（0° = 车外、180° = 车内）。
- **取回复用装载阶段**，与装载只差两点：**抓取高度**（`GRIP_H_R_PICK_MM` 地面圆环 ≠
  `GRIP_H_PICK_MM` 原料区转盘）与**完成动作**（取回是一次性的，不计 `grip.loadCount`）。
  为此装载阶段被参数化：`GRIP_LOWER_PICK` 用 `Grip_t.pickH`，`GRIP_ROT_PLACE` 用 `Grip_t.traySlot`，
  收尾按 `Grip_t.seqLoad` 分流（`= 0` 即取回，直接 `grip_Abort()`）。
- **车停在哪由编排层负责**：物料盘只能把槽位转到同一个落料点，覆盖不了间距 150 mm 的三个圆环，
  故"每放一个 → 横移一段"由 `nav` 的平移段表达，本层只管一次取放（与装载侧分工一致）。

| 对外接口 | 说明 |
| --- | --- |
| `grip_LoadOnce()` | 启动一轮装载；状态不符 / 序列忙 / **升降轴未回零**都返回 false |
| `grip_UnloadOnce(slot)` | 启动一次卸载（物料盘第 slot 槽 → 车外）；前置条件同上，另加 slot 越界检查 |
| `grip_ReloadOnce(slot)` | 启动一次取回（车外 → 物料盘第 slot 槽）；前置条件同 `grip_UnloadOnce()` |
| `grip_Safe()` | 安全姿态：旋转 90° + 夹子张开 + 中止序列（`contract_OnEnter()` 调用） |
| `grip_Abort()` / `grip_RotTo()` / `grip_SetJaw()` | 中止序列 / 设旋转目标角 / 夹子开合 |
| `grip_TrayTo()` / `grip_TrayHome()` | 物料盘转到第 pos 个槽位（1~3）/ 回位置一 |
| `grip_ResetLoad()` | 装载计数清零（**每批装载开始必须调**，否则装载段秒过） |
| `grip_IsBusy()` / `grip_AllLoaded()` | 序列进行中 / 三轮是否装完（由 `nav_LoadStep()` 调用） |

> ⚠ **占位值待实测**：`GRIP_H_PICK_MM` / `GRIP_H_PLACE_MM` / `GRIP_H_SAFE_MM`（三个高度）、
> `GRIP_JAW_OPEN_DEG` / `GRIP_JAW_CLOSE_DEG`（夹子开合角）、`GRIP_ROT_SLEW_DEG` / `GRIP_TRAY_SLEW_DEG`（两条斜坡速率）、
> `GRIP_JAW_SETTLE_MS`（夹子动作等待）、`GRIP_TRAY_POS1/2/3_DEG`（槽位角，须按实车落料点核对）、
> `SERVO_PULSE_MIN/MAX_US_CH3`（270° 舵机的脉宽量程）。

---

## 5. 状态机

六个状态：`IDLE` / `READY` / `RUNNING` / `BRAKING` / `FAULT` / `ESTOP`。
纯数据驱动，迁移规则在 `contract.c` 的 `s_transTable[]`。

| 当前状态 | 事件 | 目标状态 |
| --- | --- | --- |
| IDLE | `EV_ENABLE` | READY |
| READY | `EV_START` | RUNNING |
| READY | `EV_DISABLE` | IDLE |
| READY | `EV_FAULT` | FAULT |
| RUNNING | `EV_STOP` | BRAKING |
| RUNNING | `EV_FAULT` | FAULT |
| RUNNING | `EV_DISABLE` | IDLE |
| BRAKING | `EV_ARRIVED` | READY |
| BRAKING | `EV_FAULT` | FAULT |
| BRAKING | `EV_DISABLE` | IDLE |
| FAULT | `EV_CLEAR_FAULT` | READY |
| FAULT | `EV_DISABLE` | IDLE |
| ESTOP | `EV_CLEAR_FAULT` | IDLE |

规则：

- **`EV_ESTOP` 不在表里**。它在 `contract_Step()` 查表之前单独拦截，任何非 ESTOP
  状态一律直接进 ESTOP。
- **三个监视态（READY / RUNNING / BRAKING）都必须有 `EV_FAULT` 出口** —— 否则
  看门狗投出的故障会被当作"该状态不接受的事件"静默丢弃。
- `EV_ARRIVED` / `EV_FAULT` 是**内部事件**，由停稳判断与看门狗投递，上位机指令不产生它们。
- **`EV_ENABLE` / `EV_START` 有两个来源**：板内摁键（自启动，见 §6「摁键自启动」）与 CAN 后门
  （`CMD_ENABLE` / `CMD_START`）。摁键只在 `IDLE` / `READY` 下受理，**不改变本表的迁移规则**。
- 表里没有的组合 = "该状态不接受此事件"，被忽略并累加 `chassis.evtIgnored`（调试可观察）。

### 进入各状态的动作（`contract_OnEnter()`）

| 状态 | 四轮 | 升降轴 | 夹持机构 |
| --- | --- | --- | --- |
| IDLE | 断使能 + 清在线标志 | 断使能 + 清在线 | **安全姿态**（`grip_Safe()`：旋转 90° + 夹子张开 + 中止序列） |
| READY | 使能 + 开 `S_VEL` 上报（20 ms）+ 重置心跳 | 使能 + 开 `S_CPOS` 上报（50 ms） | —— |
| RUNNING | 清 `vx/vy/w` | —— | —— |
| BRAKING | 清 `vx/vy/w`；控制环逐周期下发零速软停（**不是**急停） | —— | —— |
| FAULT / ESTOP | `Stop_Now` 立即停止 + `faultCnt++` | 断使能（**不是** `Stop_Now`，见 §7） | **安全姿态**（同上） |

> ⚠ **BRAKING 的减速由控制环执行，不在 `contract_OnEnter()` 里**：`contract_ControlStep()`
> 在 BRAKING 分支里每周期调 `chassis_BrakeWheels()` 下发零速。只发一次是不够的 —— 状态机任务
> 优先级低于控制环，存在"控制环已进入 `chassis_DriveStep()` 下发旧速度、状态机随后改成
> BRAKING"的交叠窗口，那帧旧速度会盖掉先发的零速。

### 两条自动保护

| 保护 | 判据 | 事件 |
| --- | --- | --- |
| `chassis_DriveWatchdog()` | READY/RUNNING/BRAKING 下某电机超 100 ms 没回帧 | `EV_FAULT` |
| `chassis_CheckBraking()` | BRAKING 下四轮转速 < 30 RPM 且持续 200 ms | `EV_ARRIVED` |
| `lift_Watchdog()` | READY/RUNNING 下升降轴超 200 ms 没回帧 | `EV_FAULT` |
| `lift_CheckArrive()` | ① 回零超时 15 s → 中断回零 + 报故障<br>② 到位：位置窗口 + 转速 + 持续 200 ms → 清 `moving`（**不投事件**） | ① `EV_FAULT`<br>② 无 |

> 看门狗用**跳变沿判据**：只在"在线 → 掉线"那一拍投递一次。若每 5 ms 都投，会刷爆
> 8 深的事件队列并淹没真正的事件。

---

## 6. 后续开发在哪写代码

| 要做什么 | 改哪个文件 | 具体位置 |
| --- | --- | --- |
| **加新的执行机构**<br>接口不限 CAN：PWM / GPIO / 串口都行 | 新建 `DEVICE/xxx.c` / `xxx.h`（照 `lift_axis.c` / `grip_seq.c` 的形状：`xxx_Init()` + `xxx_Step()` + 私有状态） | ① 新模块内实现 `xxx_Init()` / `xxx_Step()`<br>② 由 `IFACE/auxiliary.c` 的 `aux_Init()` / `aux_Step()` 聚合调用<br>③ 在 `contract_OnEnter()` 补该机构的使能 / 失能调用<br>④ 仅当受上位机 **CAN** 指令控制时，才在 `aux_HandleCmd()` 加 `case` |
| **加底盘相关逻辑**<br>新运动模式、IMU 融合、更好的里程计 | `DEVICE/chassis_drive.c` | `chassis_UpdateOdom()` / `chassis_InverseKinematics()` / `chassis_DriveStep()` |
| **改状态迁移**<br>加新状态、改转移条件 | `CONTRACT/contract.c` | 只动 `s_transTable[]` 和 `contract_OnEnter()` 的分支 |
| **改协议**<br>新指令码、新地址 | `CONTRACT/contract.h` + `BSP/Emm_V5.c` | 功能码枚举在 `contract.h`，协议字节组装在 `Emm_V5.c` |
| **改默认路线**<br>动作序列、航段表 | `APP_LOG/nav_map.h` | `s_defaultMap[]`；段值由同文件场地常量推出，启停方式与航向基准见下文。⚠ 段类型是 `nav.h` 的 `NavSegType_t`，加新类型要同时改 `nav_SegStep()` |
| **改对准 / 装载策略**<br>对准窗口与增益、重试次数、抓取轮数 | `APP_LOG/nav.h` 的 `NAV_ALIGN_*` / `NAV_LOAD_*` / `NAV_HOME_*` | 控制律在 `nav_align.c` 的 `nav_AlignCtrl()`；重试与超时框架在 `nav.c` 的 `nav_TryTimeout()`。⚠ 三个高度与夹子角度属机构，仍在 `grip_seq.h` 的 `GRIP_*` |
| **改场地几何**<br>区域位置、尺寸 | `APP_LOG/nav_map.h` | 纯数据（场地坐标系下的几何、停靠点、路线表），无逻辑；`nav*.c` 只做引擎 |
| **加数据源**<br>第二个相机、新传感器串口 | `input.c` + `vision.c` | `InputCh_t` 加通道 + `input_Init()` 补引脚与 DMA；解析入口放进各自的 `xxx_Step()`（视觉见 `vision.c` 的 `vision_Step()`）。⚠ `input.c` 另有「摁键」小节，新增通道时不要混进去 |
| **改串口屏协议**<br>换屏型号、加控件指令、加事件码 | `screen.c` + `screen.h` | 发送/事件宏（`SCREEN_*`）在 `screen.h`；帧长与起始码的分发在 `screen_Dispatch()` |
| **改摁键引脚 / 延时**<br>换脚、换成板载键、改倒计时 | `input.h` | `KEY_PORT` 与 `KEY_CLK` 必须同步改，再改 `KEY_PIN`；有效电平改 `KEY_ACTIVE_LOW`；时序与接线见下文「摁键自启动」 |
| **接避障层**<br>反应式优先级抑制 | 新建 `avoid.c`（建议） | 排在 `nav_Step()` 之后：后写的覆盖先写的即得优先级，无需新仲裁机制 |
| **改状态屏内容** | `oled.c` | `oled_ShowStatus()` |
| **换舵机 / 标定行程** | `servo.h` | `SERVO_PULSE_MIN_US` / `SERVO_PULSE_MAX_US` |
| **改夹持机构的角度 / 高度 / 斜坡** | `grip_seq.h` | `GRIP_ROT_*`（三个端点角与斜坡速率）、`GRIP_JAW_*`（开合角）、`GRIP_H_*`（抓取 / 落盘 / 安全 / 卸载 / 取回 / 码垛各高度）、`GRIP_TRAY_*`（物料盘通道 / 三槽位 / 斜坡）、`GRIP_*_TIMEOUT_MS` |

### 场地地图与默认路线（`nav_map.h`）

`APP_LOG/nav_map.h` 是**智能搬运初赛赛场的地图文件**（纯数据，无逻辑），取自
`DOC/07_智能搬运赛项命题要求.md` 与「图 1 初赛赛场示意图」；**场地几何与默认路线表都在该文件**，
`nav*.c` 只保留引擎逻辑。

- **坐标系**：原点 = 场地左上角（俯视，与图 1 同向），+X 向右，+Y 向下，单位 mm。
  它与里程计坐标系（上电点为原点、随车体）**不是一个东西**；两者的换算取决于上电摆放位姿。
- **几何**（场地 2400×2400）：十字灰色行车道宽 400（中心线 1200）；四块 450×450 淡黄区
  对称居中，夹出十字；启停区 1/2 各 300×300，位于右上 / 右下角；原料区为顶边 φ300 转盘
  （圆心在顶边线上）；暂存区 580×150 贴左边、竖向居中；粗加工区 580×150 贴底边、横向居中；
  二维码板在右边墙，A4 横放，QR 80×80。
- **停靠点**（`NAV_MAP_DOCK_*`）：机器人中心应到达的场地坐标，一律"车身外缘贴住区域边界"，
  故与边界的间距 = 车体半宽 `NAV_MAP_DOCK_HALF_MM`（150）。
- ⚠ **现场随机项**：原料区圆心 X ∈ [1100,1300]、二维码板中心 Y ∈ [1100,1300]（命题原文
  "位置在规定范围内随机摆放，以现场为准"）。本表取中值；实车必须用视觉模块 2 的识别结果
  修正，不能只靠本表开环跑。

`nav_map.h` 末尾的 `s_defaultMap[]` 是**初赛全程路线**（68 段 + END），**段值全部写成同文件
场地常量的表达式**，表里不出现裸数字。全程 = 读码 → 第一批（原料区取 → 粗加工区放 → 取回 →
暂存区放）→ 回原料区取第二批（粗加工区放 → 取回 → 暂存区码垛）→ 回启停区 1；两批各 4 次
取放动作，位置号一律由任务码给出。三个前提写在表上方的注释里：起点位姿（启停区 1 中心，
机头朝 +X）、转向正方向（逆时针为正，实车反向则 TURN 段取负）、随机项取中值。运行时由
`nav_Init()` 装载（`nav_Load()` 可换任意表）。

> 转向段显式给 `timeout = NAV_TURN_TIMEOUT_MS`：姿态环一阶时间常数约 5 s，**180° 约需 24 s**，
> 超过默认的 20 s 单段超时。平移/停留段用默认 `NAV_SEG_TIMEOUT_MS`，保持较紧的卡死检测。
> 回零 / 对准 / 装载 / 取放 / 横移段同样**必须显式给**（`NAV_HOME_TIMEOUT_MS` /
> `NAV_ALIGN_TIMEOUT_MS` / `NAV_LOAD_TIMEOUT_MS` / `NAV_UNLOAD_TIMEOUT_MS` /
> `NAV_RELOAD_TIMEOUT_MS` / `NAV_RING_TIMEOUT_MS`）—— 它们比默认的 20 s 长，段超时只作兜底。
> 段数 68 已接近 `NAV_SEG_MAX`(96) 的上限，删改路线时留意 `nav_Load()` 的长度校验。

### 摁键自启动（板内自举）

**本工程没有上位机**，CAN 指令层（`CMD_ENABLE` / `CMD_START`）已降级为
**调试后门 + 唯一外部急停**。把状态机推到 `RUNNING` 的正规入口就是这颗摁键 ——
它是 `nav_Start()` 之前那道闸门在板内的唯一开关。

| 项 | 取值 | 宏 |
| --- | --- | --- |
| 引脚 | `PA0`（当前工程未占用；`GPIOA` 时钟已由 `board.c` 开启） | `KEY_PORT` / `KEY_PIN` / `KEY_CLK` |
| 有效电平 | 低电平按下（内部上拉） | `KEY_ACTIVE_LOW` / `KEY_PUPD` |
| 消抖 | 连续 4 次同电平（控制环 5 ms → 20 ms） | `KEY_DEBOUNCE_CNT` |
| 延时 | 3000 ms | `KEY_START_DELAY_MS` |
| 受理状态 | 仅 `CHS_IDLE` / `CHS_READY` | `input_KeyStep()` |

**接线**：`PA0` 与 `GND` 之间接一个常开摁键，内部上拉已开、无需外部电阻。
⚠ 换到板载摁键且它是"按下接 VDD"的 WK_UP 型时，把 `KEY_ACTIVE_LOW` 改成 `0`
（`KEY_PUPD` 会跟着变成下拉）；换端口则 `KEY_PORT` 与 `KEY_CLK` 必须同步改。

**时序与语义**

1. 按下 → 消抖 20 ms 确认 → **开始计时**。这 3 s 内车仍在 `IDLE`，**电机失能、可自由推动**。
2. 到点**重新判定一次状态**再投事件：`IDLE` → 投 `EV_ENABLE` + `EV_START`（一步到 `RUNNING`）；
   `READY` → 只投 `EV_START`；其余状态（3 s 内被后门改动过）**放弃本次启动**。
3. 进入 `RUNNING` 的动作里带 `nav_Start()`，于是车开始跑 `nav_map.h` 的默认路线 ——
   **摁键的实质是"开始跑那条 68 段路线"**，不是"上电就走"。

- **按下即锁存**，中途松手不取消；计时窗口内再按不生效。
- **上电时以当前电平为基准**，所以"摁键被压住时上电"不产生按下边沿 —— 避免上电即跑车。
- 摁键与 CAN 后门是两个平等的启动源，都只走到 `RUNNING` 这一步；`CMD_ESTOP` 的全局最高优先级
  （`contract_Step()` 里"查表之前单独处理"）不因摁键而改变。

> 为什么"3 s 后再判定状态"而不是"按下就投 `EV_ENABLE`"：`EV_ENABLE` 会让四轮**立即抱死**，
> 摆位与对准就没法做了。这 3 s 是刻意留给操作者的摆放窗口。
>
> 摁键的扫描与倒计时都按"被调用次数/时刻"推进，因此 `input_Step()` **必须每周期被调用** ——
> 它已挂在 `contract_ControlStep()` 编排序列的第 1 步。

### 已接入控制环的模块

| 模块 | 接入形态 | 残留前置条件 |
| --- | --- | --- |
| `nav`（编排层） | `nav_Step()` 在 `contract_ControlStep()` 的 RUNNING 闸门**之前**调用；启动由 `contract_OnEnter(CHS_RUNNING)` 里的 `nav_Start()` 触发；姿态环给定经黑板 `chassis.yawTarget` 传递 | **里程计标定**（`chassis_drive.h` 的轮径 / 轮距 / 轮序 / 符号）与**姿态环 PID 参数**——仍是占位值 |
| `input`（摁键部分） | `input_Step()` 在编排序列第 1 步调用，内含摁键消抖与 3 s 自启动倒计时 | 无。引脚见「摁键自启动」表 |
| `视觉模块 1`（颜色 + 对准帧解析，`vision.c`） | `vision1_Step()` 同样挂在编排序列第 1 步；`0xF5` 帧经共用组帧 / 校验后落进 `vision1Slot` | 无。消费方是**对准段**（`nav_AlignStep()`）与**装载段的挑色步**（`nav_ColorWait()`），但 `NAV_ALIGN_KP` / 窗口 / 限速 / `NAV_COLOR_HOLD_FRAMES` 全是**占位值** |
| `夹持机构`（`grip_seq.c`，经 `auxiliary.c` 聚合） | `aux_Step()` 在编排序列第 1 步调用 `grip_Step()`，其中含 `rot_Step()` / `tray_Step()`（旋转轴与物料盘两条斜坡，无条件推进）与 `grip_SeqStep()`（装载 / 卸载 / 取回动作序列）；安全姿态由 `contract_OnEnter()` 的 IDLE / FAULT / ESTOP 登记为 `grip_Safe()`（**不含物料盘**） | ⚠ 触发方是**装载段**（`nav_LoadStep()`，三轮 + 失败重试）；抓取 / 落盘 / 安全三个高度、夹子开合角、两条斜坡速率、三个槽位角全为**占位值** |
| **对准 / 装载 / 回零 / 取放 / 横移段**（`nav_seg.c`，经 `nav.c` 分派） | 七段类型 `NAV_SEG_ALIGN` / `NAV_SEG_LOAD` / `NAV_SEG_HOME` / `NAV_SEG_UNLOAD` / `NAV_SEG_RELOAD` / `NAV_SEG_RING` / `NAV_SEG_STACK`（外加码垛用的 `NAV_SEG_STACKED`），在 `nav_SegStep()` 里各自管住超时与重试（取放三段共用 `nav_GripStep()`，横移两段共用平移判据） | ⚠ `NAV_SEG_HOME` 依赖限位开关已接线；`NAV_SEG_LOAD` 的挑色判据与横移段的位号都依赖视觉模块 2 的任务码（取不到即判故障） |
| **避障层**（`avoid.c`） | `avoid_Step()` 在编排序列里紧跟 `nav_Step()` 之后；读视觉2 的 `0xF3` 障碍横向偏移，只在 `RUNNING` 且本段属**平移族**（`FORWARD` / `STRAFE` / `RING` / `STACK`）时，把 `±AVOID_STRAFE_VEL` **叠加**到黑板 `chassis.vx` 上（不改 `vy`，不覆盖 nav 本段的横移目标） | ⚠ `AVOID_STRAFE_VEL` / `AVOID_HOLD_MS` / `AVOID_DEADBAND_CM` 为**占位值**；视觉侧须按 `0xF3` 定稿格式发帧（障碍存在期间持续发，MCU 靠帧超时判消失） |

> **`nav_Step()` 必须放在 RUNNING 闸门之前，而不是之后。** 它自带"离开 RUNNING 即中止并交还
> 控制权"的归位逻辑（停车 / 故障 / 急停后不再拿着失效的段基准接着往下跑），该逻辑只有
> "每周期都被调用"才成立；放到闸门之后就变成死代码。
> 保护并未因此削弱 —— `nav_Step()` 只写黑板 `chassis.vx/vy/w` 与 `chassis.yawTarget`，不碰车轮，
> 真正的下发仍被闸门挡在 RUNNING 之内。

> **`avoid_Step()` 必须排在 `nav_Step()` 之后，且只叠加不覆盖。** 它要在导航本周期算完速度后
> 再往 `vx` 上加让行分量；若改成直接赋值，`STRAFE` / `RING` / `STACK` 段的横移目标会被整段抹掉，
> 车会变成"只让行不前进"。转向 / 对准 / 取放段不介入，否则会把机构动作冲掉。

> 另一个配套约定：**航向误差一律在连续域上算**。`chassisDrive.odomYawCont` 单调累加、
> 跨 ±180 不跳变，导航到位判据与姿态环 PID 都读它；`odomYaw` 只是它的归一化副本，
> 供内部的泰勒展开近似使用。若拿归一化值相减，179° → -180° 会算出几百度的假误差。

### 各模块的接入形态（**已全部接入**）

各模块均按"**硬件就绪、逻辑自洽、只差一行调用**"的形态交付，当前**已无未接入模块**：

| 模块 | 接入动作 | 前置条件 |
| --- | --- | --- |
| `input`（三条串口 + 摁键） | `input_Init()` 只配硬件与字节管道；`input_Step()` **已接入**（编排序列第 1 步） | 无。串口接收走 DMA + IDLE 中断，**不开新任务**。三条通道的字节均有消费方（`screen_Step()` 已接） |
| `vision`（两个视觉模块） | **两个都已接入**（编排序列第 1 步）：`vision1_Step()` 解析 `0xF5`（颜色 + 对准距离）、`vision_Step()` 解析 `0xF3`（障碍横向偏移）/ `0xF4`（任务码）；`vision_Init()` 在 `ALL_Init()` 归零槽位 | 无。帧解析靠 `input_Read()` 拉字节，须在**任务上下文**调用 |
| `avoid`（避障层） | **已接入**：`avoid_Step()` 排在编排序列 `nav_Step()` **之后**；消费 `vision_TakeFrame()` 的障碍横向偏移 | 无。⚠ 让行速度 / 失效时长 / 死区三个宏为**占位值**，待实测 |
| `screen`（串口屏协议） | **已接入**：`screen_Step()` 收屏上报（编排序列第 1 步）；上屏（任务码 / 任务统计）走 `screen_ShowTask()` / `screen_ShowGet()` / `screen_ShowPut()` / `screen_ShowErr()`，**调用点在 STATE 任务**（`contract_UiStep()`）—— 屏指令是阻塞发送（115200 下最长约 6 ms），**不能放进 5 ms 控制环**；且**每个 200 ms 超时周期最多发一条**，把连发阻塞摊到多个周期。⚠ 控件名宏 `SCREEN_OBJ_*` 是**占位值**，按 TJC 工程实际控件名改 | 屏上报的字节经 `input_Read(INPUT_SCREEN)` 拉取，须在**任务上下文**调用 |
| `oled` | **已接入**：`oled_Step()` 在 STATE 任务里紧跟 `contract_UiStep()`（内部已按 `OLED_REFRESH_MS` 限速） | 无。注意刷新是阻塞式软件 I2C，**不能放进 5 ms 控制环**；放在 STATE 是为了让阻塞与屏发送**在同一任务内串行**，不与控制环叠加 |
| `servo` | **三路都已由 `grip_seq.c` 的夹持机构接管**（CH1 旋转轴 / CH2 夹子 / CH3 车载物料盘），无需额外接入；标定行程走 `CMD_GRIP_*`（`0x33`~`0x37`）或直接调 `servo_SetPulse()` | 无。⚠ **不要绕过夹持机构直接改这三路的角度** —— 会与旋转轴 / 物料盘斜坡和装载序列打架 |

> 新模块的周期处理挂进 `contract_ControlStep()` 的编排序列里；**插入位置由该模块自身的
> 归位需求决定** —— 需要"每周期都被调用才能自保"的（如 `nav_Step()`）放在闸门之前，
> 纯执行类、不需要自保的放在闸门之后。

### 视觉模块 1（`vision.c`）：物块颜色 + 对准帧已定稿并接入

**只用一个功能码 `0xF5`，DATA 固定 5 字节**，帧壳与校验和视觉模块 2 完全相同：

| 帧头 | 功能码 | 数据长度 | 数据内容 | 和校验 | 附加校验 |
| --- | --- | --- | --- | --- | --- |
| `0xAA` | `0xF5` | `5` | `color`(u8) + `front`(int16 小端) + `left`(int16 小端) | `SC` | `AC` |

- `color` = **物块颜色数字代号**，取值与赛题一致：**1 红 / 2 黄 / 3 蓝 / 4 绿 / 5 黑 / 6 浅蓝**，`0` = 未识别（`VISION1_COLOR_*`）。
- `front` = **前后相对距离(mm)，正 = 车需前进**；`left` = **左右相对距离(mm)，正 = 车需左移**。
- 语义是"以**物料中心为原点**，视觉中心相对物料中心的矢量"，**相对量、带符号**。
- 落到控制量上：`vy = +Kp·front`、`vx = −Kp·left`（`chassis.vx` 右移为正，所以左移取负）。
- 校验不过 / 功能码不是 `0xF5` / 长度不是 5，都只累加 `vision1Slot.errCnt`，**不更新 color / front / left**
  （三者始终来自同一帧，不会新旧混用）。
- `online = 0` 时 `color` / `front` / `left` **都不可信**（与避障同一条 fail-safe 铁律），对准逻辑不得据此动作。

> **颜色与距离同帧同码**：车对准的同时就把颜色读到了，不需要额外请求；代价是二者无法单独刷新
> （视觉模块若某一项没识别出来，整帧照发、该项填 0，接收侧不做额外校验）。
>
> 消费方是编排层的**对准段**（`NAV_SEG_ALIGN`）与**装载段的挑色步**（`NAV_SEG_LOAD` 每轮先比 `color` 与任务码目标色），见下节。

### 原料区动作：回零 / 对准 / 装载三个航段（`nav_seg.c` + `nav_align.c`）

原料区那两段（默认路线 06·07·08 与 37·38·39，两批各一次）由**三个新段类型**承载，
都是"车停原地、只动机构或只微调底盘"：

| 段类型 | 做什么 | 判完成 | 段超时 |
| --- | --- | --- | --- |
| `NAV_SEG_HOME` | 进入本段时若 `lift.homed = 0` 就 `lift_Home()`，之后等反馈 | `lift.homed != 0` | `NAV_HOME_TIMEOUT_MS` |
| `NAV_SEG_ALIGN` | 读 `vision1_GetSlot()`，`vy = Kp·front`、`vx = −Kp·left`，两轴同时进窗口后停住保持 | 两轴 \|误差\| ≤ `NAV_ALIGN_TOL_MM` 且持续 `NAV_ALIGN_HOLD_MS` | `NAV_ALIGN_TIMEOUT_MS` |
| `NAV_SEG_LOAD` | 段入口按段值定的**颜色组**（第 1 组 / 第 3 组）取任务码并清装载计数；每轮**先等本轮目标色**（`nav_ColorWait()`：原地泊住，同色连续 `NAV_COLOR_HOLD_FRAMES` 帧算停稳）→ 再对准 → **抓取前复查颜色**（`nav_ColorMatched()`，不符退回等色步重来）→ 再 `grip_LoadOnce()` —— 落盘时物料盘同步转到本轮槽位；机构忙就等 | `grip_AllLoaded()` **且机构空闲**（等物料盘归位的收尾动作） | `NAV_LOAD_TIMEOUT_MS` |

- **重试框架**：`nav_TryTimeout()` 管单次尝试的时限与重试计数。`ALIGN` 用
  `NAV_ALIGN_TRY_TIMEOUT_MS` × `NAV_ALIGN_RETRY`；`LOAD` 里对准与抓取**共用** `s_nav.retry`。
  重试用尽才 `contract_PostEvent(EV_FAULT)` + `nav_Abort()`，收敛到 FAULT，本层不自行复位。
- **fail-safe**：`vision1Slot.online = 0` 时对准段**不动作**（速度清零，等恢复或超时），
  与避障同一条铁律 —— 距离不可信就不许据它动底盘。
- ⚠ `NAV_SEG_LOAD` 的前置条件是 `lift.homed = 1`，故默认路线把 `NAV_SEG_HOME` 排在它前面；
  `grip_LoadOnce()` 返回 false（状态不符 / 未回零）**不重试，直接判故障**。
- **装载计数每段清零**（`grip_ResetLoad()`）：不清则跑完一批再启动导航时 `grip_AllLoaded()` 恒真，
  本段会**秒过**。清零在段入口做，不在 `aux_Init()` —— 后者一次上电只执行一次。
- **物料盘与夹持轴并行**：进入落盘旋转阶段时两轴同时起斜坡（各自按 `GRIP_*_SLEW_DEG`），
  下降到放料高度前要求**两轴都到位**。第 k 轮的目标槽位 = 已装轮数 + 1（顺序 1 → 位置一 = 0°）。
- **挑色抓取（颜色策略 B1）**：转盘会停但**停位随机**，不能保证首次停的就是任务码要的第一个颜色 ——
  故每轮**先等目标色**（`task[loadGroup][loadCount]`，`loadGroup` = 本段段值定的颜色组）转到取料点、
  连续若干帧同色确认停稳，再对准抓取。
  等色期间车原地泊住（`vx/vy = 0`），颜色不可信（`online = 0` 或 `color = 0`）时同样不动作。
  **槽位仍由轮次决定**（`loadCount + 1`），**不是**按颜色查表。无任务码（`taskGot = 0`）时本段判故障。
- **抓取前复查颜色**（`nav_ColorMatched()`）：等色判据是"对准之前"的快照，而对准本身要耗时，
  期间转盘可能又转过一颗 —— 那时对准收敛到的已不是目标色物料。故在 `grip_LoadOnce()` 之前
  按当前帧再比一次颜色，不符就复位 `loadColored` / `loadAligned` / `retry` 与 `tryTick`，退回等色步重来。
  复查只取**当前帧**，判据同样是"在线且颜色等于目标色"；长期对不上由 `NAV_COLOR_TIMEOUT_MS` 与段超时兜底。
- ⚠ **段超时字段是 `uint32_t`**：一轮装载按当前的 `LIFT_VEL_DEFAULT`（300 RPM = 10 mm/s）估算
  约 40 s，三轮"等色 + 对准 + 动作"已远超 `uint16_t` 的 65.5 s 上限（现给 240 s）。抬高升降速度后可以调小这些常量。

### 目标区动作：横移 + 卸料 / 取回 / 码垛（`nav_seg.c`）

车外目标区（粗加工区 / 暂存区）的动作分两类：**横移对准**（底盘）与**取放**（机构），
各自占一个段类型，同名段可重复出现（**每件一个段**）：

| 段类型 | 做什么 | 判完成 | 段超时 |
| --- | --- | --- | --- |
| `NAV_SEG_RING` | `value` = 组号×10 + 件序号；取 `task[组][件]` 得位号，横移到该位 | 位移投影到位 | `NAV_RING_TIMEOUT_MS` |
| `NAV_SEG_STACK` | `value` = 第二批件序号；取**第一批同色物料**所在位，横移到该位 | 同上 | `NAV_RING_TIMEOUT_MS` |
| `NAV_SEG_UNLOAD` | `value` = 物料盘槽位；调 `grip_UnloadOnce(slot)`，机构动作期间车原地泊住 | 机构空闲且 `grip.fault = 0` | `NAV_UNLOAD_TIMEOUT_MS` |
| `NAV_SEG_RELOAD` | `value` = 物料盘槽位；调 `grip_ReloadOnce(slot)`，其余同上 | 同上 | `NAV_RELOAD_TIMEOUT_MS` |
| `NAV_SEG_STACKED` | `value` = 物料盘槽位；调 `grip_StackOnce(slot)`，放置高度抬一个料高 | 同上 | `NAV_UNLOAD_TIMEOUT_MS` |

- 横移段与平移段共用到位判据，只是位移由**位号差**折算：
  `(目标位号 − 当前位号) × NAV_MAP_RING_PITCH_MM × NAV_MAP_RING_DIR`。
  **当前位号在"取放族"段之间保持**（区域内逐件横移不必每次回到入口基准），
  一旦遇到非取放族段（平移 / 转向 / 停留 / 对准 / 装载 / 回零）就复位到 `NAV_MAP_RING_DOCK_NUM`。
- 三个取放段共用一个实现 `nav_GripStep(slot, mode)`，只靠模式参数区分
  （`NAV_GRIP_UNLOAD` / `NAV_GRIP_RELOAD` / `NAV_GRIP_STACK`）。
- 前置条件是**车已由上游平移段停在目标区前**；不满足（状态不符 / 升降轴未回零 / 槽位越界）
  **不重试，直接判故障**，与 `NAV_SEG_LOAD` 同一条口径。
- ⚠ **位号越界即判故障**：任务码的位置组不做范围校验（现场圆环数未定），取值落在 1~3 之外、
  或第二批某色在第一批里找不到同色物料，横移段入口就 `nav_Fault()`，不猜。
- ⚠ `NAV_MAP_RING_DOCK_NUM` / `NAV_MAP_RING_DIR`（`nav_map.h`）是**占位值**：
  入口停靠点正对哪个位号、位号增大朝车体哪一侧，取决于现场两区的摆位与编号方向，需实测核定。

### 视觉模块 2（`vision.c`）：任务码 + 避障帧均已定稿并接入

**通信帧格式 = 匿名（ANO）通信协议 V7 的精简帧 —— 去掉了目标地址 `D_ADDR`**，
其余字段与校验算法原样保留：

| 帧头 | 功能码 | 数据长度 | 数据内容 | 和校验 | 附加校验 |
| --- | --- | --- | --- | --- | --- |
| `0xAA`（固定） | `ID` | `LEN` | `DATA[LEN]`（小端） | `SC` | `AC` |

- 总长 `LEN + 5`；`SC` 在下标 `LEN+3`、`AC` 在下标 `LEN+4`。
- `SC` = 从帧头起逐字节累加取低 8 位，**累加范围是帧头到 DATA 结束共 `LEN+3` 字节**；
  `AC` = 每累加一字节就把当时的 `SC` 再累加一次，取低 8 位。
- 帧由 `vision_FeedByte()` 逐字节拼装，收满即校验；**校验不过只计 `errCnt`，不进槽位**。
- 功能码：`0xF3` 避障信息、`0xF4` 任务码（二维码）。`0xF4` 的数据内容是**十进制数字串原文** ——
  如任务码 `156+123+516+231` 则 DATA 发 `156123516231`（12 字节 ASCII，**不带 `+`**）；
  槽位按原文存进 `VisionTask_t.code[16]` + `len`，**分组解析留给消费方** ——
  编排层 `nav_ParseTask()` 抽出全部数字、**按固定位序每 3 位切一组**（`'+'` 等分隔符一律跳过，
  故带分隔符的原文也能解析），一次填满 `s_nav.task[4][3]`：

| 组 | `NAV_TASK_G_*` | 含义 | 取值范围 |
| --- | --- | --- | --- |
| 1 | `NAV_TASK_G_ORDER1` | 第一批三个物料的颜色与搬运顺序 | 1~6（颜色编号） |
| 2 | `NAV_TASK_G_POS1` | 第一批物料在粗加工区 / 暂存区的放置位号 | 环号，**不校验范围**（现场圆环数未定） |
| 3 | `NAV_TASK_G_ORDER2` | 第二批三个物料的颜色与搬运顺序 | 1~6 |
| 4 | `NAV_TASK_G_POS2` | 第二批物料在粗加工区的放置位号 | 环号，**不校验范围** |

  位数不足 12 位、或颜色组出现越界值 → **整码作废**（保留上次结果，不置 `taskGot`），
  避免视觉短暂丢帧把任务清空。颜色编号 1 红 / 2 黄 / 3 蓝 / 4 绿 / 5 黑 / 6 浅蓝。
- **位置组的消费方（已确认，2026-10-06）**：
  - 第 2 组 = **第一批**物料在**粗加工区 + 暂存区**的放置位号，**两区同号** ——
    第一批第 k 个 → 粗加工区 = 暂存区 = `task[NAV_TASK_G_POS1][k]`。
    粗加工区放完就地按**同一组位号**取回，运至暂存区再按同一组位号放下。
  - 第 4 组 = **第二批**物料在**粗加工区**的放置位号 → `task[NAV_TASK_G_POS2][k]`，取回同号。
  - 第二批在**暂存区**不读位置码：它是把第二批**码垛**在第一批物料之上，位号由**同色反查**得到 ——
    `nav_StackRing(k)` 拿 `task[ORDER2][k]` 的颜色去 `task[ORDER1][*]` 里找同色件，
    取其 `task[POS1][*]`（命题原文：暂存区码垛、颜色一致）。
  - 位号换算成底盘横移：`(目标位号 − 当前位号) × NAV_MAP_RING_PITCH_MM × NAV_MAP_RING_DIR`；
    入口基准位号与方向见 `NAV_MAP_RING_DOCK_NUM` / `NAV_MAP_RING_DIR`（**占位值，待实测**）。

**避障帧（`0xF3`，2026-10-06 定稿）**：DATA 固定 **2 字节**，`int16` 小端，只有一个字段 ——

| 偏移 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- |
| `[0:1]` | `x` | `int16` 小端 | 障碍**横向偏移(cm)**：**正 = 障碍在车前方偏左**，负 = 偏右 |

- **不报纵向距离**：视觉只在障碍进入警戒区时发帧，**"收到这一帧"本身就是触发信号**；
  MCU 侧靠**帧超时**（`AVOID_HOLD_MS`）判断障碍消失 —— 障碍在场期间视觉须持续发帧。
- 单帧单条，落进 `visionSlot.obst[0]` + `count = 1` + `obstFresh = 1`；
  `vision_TakeFrame()` 取走后清 `obstFresh`，避障层在两次取帧之间沿用槽位里的最后偏移。
- 障碍物实体是 **φ50×100 mm 黑色圆柱**（模拟障碍），帧里不带尺寸。
- ⚠ 发置信度、发多条、发纵向距离都**不在协议里**，不要自行加字段。

**视距已实测**：`VISION_RANGE_MM = 500`（50 cm）。车速上限由它反推 ——
`净空 = 车速×总延迟 + 车速²/(2×减速度) + 车体半径 + 障碍半径`。
取延迟 100 ms、减速度 800 mm/s²、车体+障碍 200 mm：300 mm/s → 约 286 mm（余量充足）；
600 mm/s → 约 485 mm（**已逼近 500 mm 边界**）；900 mm/s → 约 796 mm（**超出视距，不可用**）。
**即 50 cm 视距把车速上限压在约 600 mm/s 附近 —— 这是物理限制，不是算法限制。**

> ⚠ `vision_GetRangeMM()` 返回 0 时的语义是"**不可信**"，避障层不得据此放行任何速度。
> 这与"视觉无数据 ≠ 无障碍"是同一条 fail-safe 铁律。

### 三条串口的分工

| | 视觉模块 1 | 视觉模块 2 | 串口屏 |
| --- | --- | --- | --- |
| 通道 | `INPUT_VISION1` | `INPUT_VISION2` | `INPUT_SCREEN` |
| 串口 | USART1（PA9/PA10，DMA2 Stream5） | USART3（PB10/PB11，DMA1 Stream3） | USART2（PA2/PA3，DMA1 Stream5） |
| 承载内容 | 物块颜色 + 对准相对距离（前后 / 左右） | 障碍物（`0xF3`）+ 任务码（二维码，`0xF4`），视距 50 cm | 控件指令下发 + 触摸 / 变量事件上报 |
| 协议实现 | `vision.c`（**匿名协议精简帧**，与视觉 2 共用组帧 / 校验） | `vision.c`（**匿名协议精简帧**） | `screen.c`（TJC 协议） |
| 状态 | 协议已落地**并接入控制环**（`0xF5`，DATA 5 字节）；消费方为对准段 | 协议已落地**并接入控制环**：`0xF4`（任务码）由装载段消费，`0xF3`（障碍横向偏移，DATA 2 字节）由避障层 `avoid_Step()` 消费 | 协议已落地**并接入控制环**（收：`screen_Step()`；发：STATE 任务里的 `contract_UiStep()`） |

### 新机构接入系统要做三件事

**只有第 ③ 项和 CAN 有关** —— 走 PWM 的机构直接跳过第 ③ 步即可。

1. **动作登记** → `contract_OnEnter()` 里补 IDLE 失能 / FAULT-ESTOP 安全停（**与接口无关**）
2. **周期处理** → 挂进 `aux_Step()`（**与接口无关**）
3. **指令分发** → `aux_HandleCmd()` 加 `case`（**仅当它受上位机 CAN 指令控制时**）

### 两条红线

1. **`contract_ControlStep()` 不许塞算法。** 它是骨架编排，只负责按顺序调用别人：
   `chassis_DriveWatchdog()` → `aux_Step()` → `input_Step()` → `vision1_Step()` → `vision_Step()` →
   `screen_Step()` → `chassis_CheckBraking()` → `nav_Step()` → `RUNNING ? chassis_DriveStep() : BRAKING ? chassis_BrakeWheels()`。
   新模块的周期处理挂进 `vision_Step()` 之前那一组，不要改动这个顺序。
2. **加新机构时别碰 `contract.c` 的状态机逻辑。** 只需在 `contract_OnEnter()` 里加一行调用。
   状态机永远只认"系统级"的事，机构细节全留在各自的机构层文件（`lift_axis.c` / `grip_seq.c`）。

---

## 7. 安全设计要点（务必保留）

- **升降轴急停用"断使能"，不是 `Stop_Now`。** 两者区别：断使能（`0xF3/0xAB`）撤掉保持电流、
  立即停转并断电；`Stop_Now`（`0xFE/0x98`）是**保持使能**的减速停止。选断使能是为了彻底断电，
  负载则靠 **T8 梯形丝杆自锁**停住。**换成不自锁的丝杆（如滚珠丝杆）时不能照搬**——
  断使能会让负载下滑，须改成"保持使能 + `Stop_Now`"并加抱闸。
- **`lift.pos` / `lift.target` 用 `int32_t`**：驱动器位置是有符号的，无符号跨零点比较会出错。
- **升降轴不进 `chassis_CheckBraking()` 的停稳判据**：丝杆动作与底盘停车无关，算进来会让
  底盘永远到不了 READY。
- **`homed` 不参与移动放行**：未接限位开关的轴也能用 `CMD_LIFT_SETZERO` 手动建立基准；
  若强行要求先回零，未接限位开关的轴将完全无法使用。

---

## 8. `PID_Mod` 模块行为要点（2026-10-01 逐条核对源码）

> 本节只记这个模块自身的实现行为 —— 它在什么情况下不会按直觉工作。

- **`Improve = NONE`（0）并非"什么都不做"**：`f_Output_Limit` / `f_Proportion_Limit`
  在非死区分支里是**无条件调用**的，输出限幅关不掉；`Fallback_Diff` 必须显式开启才生效。
- **误差恰好等于死区时走"保持"分支**（判断是 `ABS(Err) > DeadBand`，不是 `>=`）。
- **未开 `Fallback_Diff` 时，死区内 `Output` 保持上一周期值**——位置环里意味着"命令保持"，
  接 Emm_V5 位置模式（驱动器自带闭环）没问题；若改成直接驱动 PWM 会因此"抖不起来/卸力"。
- **开了 `Fallback_Diff` 则死区内会清零 `Iout`/`Pout`**，输出只剩微分项，测量值一停就趋于 0；
  同时该分支的微分用回"误差微分"（`Kd*(Err-Last_Err)`）而非测量值微分，
  **在死区边界上会重新引入设定值突变的微分冲击**。
- **`ErrorHandle`（`0x80`）慎用**：① `(ABS(Target-Measure) / Target)` 在 `Target == 0` 时除零；
  ② 判定用的是**上一周期的 `Output`**，且 `Output < MaxOut*0.01f` 这个"保护门"对负向大输出无效；
  ③ 一旦置为 `Motor_Blocked` 就**永久锁死**，唯一解除办法是重新 `PID_Init()`。建议自己重写这段。
- **无控制周期无关性**：Kp/Ki/Kd 未按采样时间归一化，结构体里的 `ControlPeriod` / `thistime` /
  `dtime` / `MaxErr` / `LastNoneZeroTarget` 全是预留未用字段。**改控制周期会静默改变等效积分增益，
  必须重新整定。**
- **`ABS` 宏会重复求值**（`((x) > 0 ? (x) : -(x))`），只可传无副作用的表达式。
- 参数名 `intergral_limit`（及枚举 `Trapezoid_Intergral`）是 `integral` 的**拼写错误**，
  属既成事实，改名会破坏调用方，暂按原样。
- **结构体无任何互斥保护**，同一实例只能由单个任务访问。

---

## 附：架构图

见 `DOC/`（全部为宽幅矢量 `.svg`，1600×1000，可直接缩放）：

- `DOC/01_函数调用链与开发落点.svg` —— 启动链 / 三任务 / 控制环编排序列 / CAN 收帧分流 / 开发落点速查
- `DOC/02_状态机迁移图.svg` —— 六态迁移 13 条 + 进入动作 + 自动事件与摁键来源
- `DOC/03_代码架构全景图.svg` —— 一页看全：启动链 → 三任务 → CAN 上/下行 → L2 路由表 → 黑板 → 文件分层
- `DOC/04_main初始化收敛到ALL_Init.svg` —— `ALL_Init()` 八步重构对比
- `DOC/05_工程进度全景_已建与未建.svg` —— 已接入 / 已建未接入 / 未开始 三列盘点
- `DOC/06_引脚分配对应图.svg` —— 引脚与外设分配
- `DOC/07_智能搬运赛项命题要求.md` —— 赛项比赛流程与运行要求（`nav_map.h` 地图数据的依据，非图）
