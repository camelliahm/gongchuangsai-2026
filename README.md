# STM32F407 CAN 底盘固件

四轮麦轮小车固件。主控 STM32F407VETx，FreeRTOS，通过 CAN1 与 Emm_V5 / ZDT（张大头）
步进闭环驱动器通信，并同时在该总线上接收上位机指令。

## 本文件只负责什么

**本文件是「工程知识层」——讲"这工程是什么、为什么这么设计"。**

| 文件 | 角色 | 只放什么 | 谁读 |
| --- | --- | --- | --- |
| **`README.md`**（本文件） | **知识层：为什么这么设计** | 分层结构与扩展指引、消息架构完整论证、状态迁移与协议表、升降轴安全设计、待硬件确认清单 | 按需查阅，面向**人** |
| `../../AGENTS.md` | **规范层：做什么、怎么做才对** | 注释规范 / 编码换行 / 目录命名 / 构建收尾等**硬约束**；构建烧录步骤；硬件与协议事实表 | AI **每轮注入** |
| `../../.workbuddy/memory/MEMORY.md` | **状态层：现在到哪一步了** | 构建基线**数字**、未接入模块**清单**、Git 纳管现状、工具链路径、**踩坑教训**、用户已拍板的决策 | 按需查阅，面向 **AI** |

- **规范条文只写在 `AGENTS.md` 开头的「⛔ 硬约束」节**，本文件不重复，只留指针。
- **架构论证只写在本文件**，`AGENTS.md` 里只保留摘要 + 指针。
- 三处不一致时：规范以 `AGENTS.md` 为准，架构以本文件为准。

**代码里的注释只讲"这个函数干什么、参数怎么传、有什么坑"。**
模块定位、分层规则、扩展指引、接口约定这类"说明性"内容统一写在本文件，
**不要再塞进 `.c` / `.h` 里**。改架构时改这里，改函数行为时才改代码注释。

---

## 1. 分层结构

| 文件 | 职责 | 不该放什么 |
| --- | --- | --- |
| `BASE/chassis.h` | **公共契约层**：消息码分段、地址宏、状态/事件枚举、`Chassis_t`、任务参数、`chassis_*` 接口 | 各层私有类型与宏 |
| `BASE/chassis.c` | **状态机 + 系统级调度**：迁移表、`chassis_OnEnter`、消息路由派发、三个任务体、控制环编排 | 任何控制算法 |
| `BASE/oled.c/.h`<br>`BASE/oled_font.h` | **显示层**：SSD1306 128x64 状态屏（软件 I2C，只读黑板，不参与控制） | 状态机迁移、任何控制逻辑 |
| `BASE/PID_Mod.c/.h` | **通用算法层**：PID 控制器（外部移植，只引用不修改） | 状态机迁移、硬件访问 |
| `Mycode/chassis_drive.c/.h` | **底盘驱动层**：四轮动作 / 反馈 / 看门狗 / 停稳判断 + 里程计 / 麦轮逆解 / 姿态环 | 状态机迁移、升降轴 |
| `Mycode/auxiliary.c/.h` | **辅助机构层**：底盘四轮之外的所有执行机构（现为升降轴） | 状态机迁移、底盘四轮 |
| `Mycode/nav.c/.h` | **编排层（任务层）**：航段序列 → 逐周期的底盘速度与目标航向 | 状态机迁移、直接下发 CAN |
| `Mycode/input.c/.h` | **输入汇聚层**：三条串口通道（USART1 视觉1 物块颜色 / USART3 视觉2 障碍物+任务号 / USART2 串口屏）+ 视觉2 帧槽位；⚠ 帧格式留空 | 执行机构、状态机迁移 |
| `Mycode/servo.c/.h` | **执行机构（PWM）**：3 路 50Hz 舵机（TIM3 CH1/CH2/CH3 = PC6/PC7/PC8） | 状态机迁移、底盘四轮 |

> **目录约定**：**底层基础（契约 / 调度 / 显示 / 通用算法）放 `BASE/`；应用层与机构层新增的
> `.c` / `.h` 一律放 `Mycode/`**，`APP/` 只保留 `main.c` 与 ST 模板
> （`stm32f4xx_conf.h`、`stm32f4xx_it.c/.h`）。加入新文件后 Keil 与 EIDE 两侧的
> include 路径都已包含 `../BASE` 与 `../Mycode`，无需再改搜索路径。

三层之间**不直接互相调用**，通过两条通道交互：

- **向上报事件**：`chassis_PostEvent(EV_FAULT, ...)` 投给状态机
- **向下读状态**：只读 `chassis.state`

> 这是最小形态。扩展到"所有模块统一按消息接入"的完整约定，见 **§2 模块间通信**。

> **⚠ 新增的模块尚未接入控制环。**
> `nav` / `input` / `servo` / `oled` 的硬件已在 `APP/main.c` 里初始化，但
> **`nav_Step()` / `input_Step()` / `vision_Step()` / `oled_Step()` 都没有调用点，舵机也无人下达角度**。
> 它们现在是"装好了但没接线的部件"——接入位置与前置条件见 **§6**。
>
> **`input` 内的视觉模块 2 仍未定稿**（见 §6 与 §10）：通信帧格式、任务号承载位置与视场角 / 安装姿态。

---

## 2. 模块间通信：模拟 ROS 方案

**一句话：用「消息」连接模块，而不是用函数调用。**

这不是要把 ROS 搬进单片机，而是把「**统一接口形状 + 一张注册表**」定为新增模块的标准接入方式。
目的只有两个：**加模块不用改已有模块；多人可以按冻结的契约并行开发。**

> **现状**：接口形状与**路由注册表**均已就位（L2）；编排层 `nav` 已建，**尚未接入控制环**。

### 2.1 五个要素

| 要素 | 内容 | 现状 |
| --- | --- | --- |
| **契约** | 地址宏、消息码枚举、消息结构体 —— 集中在 `BASE/chassis.h` | 已有 |
| **节点** | 每个机构 = 一个 `xxx_HandleCmd()` + 一个 `xxx_Step()`，**不新开任务** | 已有（`auxiliary.c` 就是这个形状） |
| **路由** | 一张 `{ 消息码段 → handler }` 注册表 | **已有**（`chassis.c` 的 `s_routeTable[]`，`chassis_HandleCmd()` 按段查表派发） |
| **黑板** | 全局 `chassis` 结构体：编排层写目标，模块写状态 | 已有 |
| **编排** | 任务层 `nav`：决定机构的组合与先后顺序 | **已建**（未接入控制环） |

### 2.2 消息码分段

沿用现有功能码（在代码与本节中统一称"消息码"），但**按段划分归属**：

| 段 | 归属 |
| --- | --- |
| `0x00` ~ `0x0F` | 系统（`CMD_ENABLE` / `CMD_START` / `CMD_STOP` / `CMD_ESTOP` / `CMD_CLEAR_FAULT`） |
| `0x10` ~ `0x1F` | 底盘（`CMD_SET_VEL`） |
| `0x20` ~ `0x2F` | 升降轴（`CMD_LIFT_*`） |
| `0x30` ~ `0x3F` | 舵机（规划） |
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
2. **不新增任务。** 一个控制任务按序 tick 各机构 —— `chassis_ControlStep()` 已经是这个形状。
   16 KB 堆撑不住 N 个机构各一个任务。
3. **同步派发必须同一拍内处理完**，不许留待下一拍。
4. **持续状态走黑板，不走消息。** 位姿、位置、电机状态这类"随时可读"的量放结构体；
   消息只承载"瞬时意图"。混用会让队列要么丢帧、要么被状态刷新刷爆。
5. **消息载荷用结构体，不用裸字节。** CAN 帧在入口解析/校验后转成结构体再派发，
   让编译器帮忙查参数错误。
6. **不做动态分配。** 消息是定长副本、值传递，不存在"谁申请谁释放"的问题。

### 2.5 新增模块的四个动作

1. **申请消息码** —— 在对应段里加一个枚举（`chassis.h`）
2. **实现 handler + step** —— `xxx_HandleCmd()` / `xxx_Step()`，放 `auxiliary.c`（或新建模块文件）
3. **注册到路由表** —— 在 `chassis.c` 的 `s_routeTable[]` 加一行（段基址 + 段掩码 + 该模块的 handler）；不属于任何已注册段的消息会被忽略并累加 `chassis.evtIgnored`
4. **在 `chassis_OnEnter()` 登记状态动作** —— IDLE 失能、FAULT/ESTOP 安全停
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
| L2 | 注册表 + 同步派发 | **现状** —— `s_routeTable[]` 已建（系统 / 底盘 / 升降 3 段；`servo` 尚未注册） |
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
| 2 | `servo_Init()` | 三路舵机 PWM（TIM3 CH1~CH3）— ⚠ 未接入控制环 |
| 3 | `input_Init()` | 三条串口（USART1 视觉1 / USART3 视觉2 / USART2 串口屏）— ⚠ 未接入控制环，帧格式留空 |
| 4 | `oled_Init()` | OLED 状态屏（软件 I2C）— ⚠ 未接入控制环 |
| 5 | `nav_Init()` | 编排层：装载默认地图，不自动启动 — ⚠ 未接入控制环 |
| 6 | `can_RtosInit()` | 建 CAN 接收队列 + 发送互斥锁 |
| 7 | `chassis_Init()` | 建事件队列 → `chassis_DriveInit()` + `aux_Init()` |
| 8 | `App_TaskCreate()` | 按任务表 `s_tasks[]` 建任务 |

第 1 项须最先执行 —— 后序模块的时钟与 GPIO 由它打开；第 6~8 项须在建任务之前
—— 任务一起来就立刻用队列和互斥锁。

| 任务 | 优先级 | 栈（字） | 入口 | 职责 |
| --- | --- | --- | --- | --- |
| `CTRL` | 4 | 512 | `chassis_CtrlTask` | 5 ms 绝对节拍，每拍调 `chassis_ControlStep()` |
| `CAN_RX` | 3 | 256 | `chassis_CanRecvTask` | 阻塞 `can_RecvFrame()` 取帧 → `chassis_HandleFrame()` 分流 |
| `STATE` | 2 | 256 | `chassis_StateTask` | 阻塞等事件 → `chassis_Step()` 查表迁移 |

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
| `0xFE` | 本机（上位机发给底盘的指令帧） | `chassis_HandleCmd()` |
| `0x00` | ZDT **广播地址** | **禁止占用**，收到一律忽略 |
| 其它 | 无关帧 | 忽略 |

> **本表只描述"挂在 CAN1 上"的设备。** 目前只有步进类（四轮 + 升降丝杆）走 CAN。
> 后续若新增非 CAN 机构（PWM / GPIO / 串口），不占这里的地址，也不进
> `chassis_HandleFrame()` 的分流。

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

**C 侧控制接口**（`Mycode/auxiliary.c`，两套并存）：

| 函数 | 单位 | 说明 |
| --- | --- | --- |
| `lift_MoveToMM(mm, vel, acc)` | mm | 升降到绝对高度（0 = 原点，向上为正），目标夹到 `[0, LIFT_STROKE_MM]` |
| `lift_MoveByMM(deltaMM, vel, acc)` | mm | 相对当前高度升降（正升 / 负降） |
| `lift_GetHeightMM()` | mm | 读当前高度 |
| `lift_MoveTo(clk, dir, vel, acc)` / `lift_MoveBy(delta, vel, acc)` | 脉冲 | 底层接口，mm 接口内部换算后转调它们 |

### 升降轴参数与换算（`auxiliary.h`）

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
- 对外接口（均在 `auxiliary.h` 声明，内部统一用 `can_TxLock/Unlock` 包住发送）：
  `lift_SetEnable()`、`lift_EnableFeedback()`、`lift_ClearOnline()`、`lift_MoveTo()`、`lift_MoveBy()`、
  `lift_MoveToMM()`、`lift_MoveByMM()`、`lift_GetHeightMM()`、`lift_Home()`、`lift_StopNow()`、
  `lift_SetZero()`、`lift_ClearClog()`、`lift_IsMoving()`。

---

## 5. 状态机

六个状态：`IDLE` / `READY` / `RUNNING` / `BRAKING` / `FAULT` / `ESTOP`。
纯数据驱动，迁移规则在 `chassis.c` 的 `s_transTable[]`。

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

- **`EV_ESTOP` 不在表里**。它在 `chassis_Step()` 查表之前单独拦截，任何非 ESTOP
  状态一律直接进 ESTOP。
- **三个监视态（READY / RUNNING / BRAKING）都必须有 `EV_FAULT` 出口** —— 否则
  看门狗投出的故障会被当作"该状态不接受的事件"静默丢弃。
- `EV_ARRIVED` / `EV_FAULT` 是**内部事件**，由停稳判断与看门狗投递，上位机指令不产生它们。
- 表里没有的组合 = "该状态不接受此事件"，被忽略并累加 `chassis.evtIgnored`（调试可观察）。

### 进入各状态的动作（`chassis_OnEnter()`）

| 状态 | 四轮 | 升降轴 |
| --- | --- | --- |
| IDLE | 断使能 + 清在线标志 | 断使能 + 清在线 |
| READY | 使能 + 开 `S_VEL` 上报（20 ms）+ 重置心跳 | 使能 + 开 `S_CPOS` 上报（50 ms） |
| RUNNING | 清 `vx/vy/w` | —— |
| BRAKING | 只清目标速度，由控制环逐周期减速（**不是**急停） | —— |
| FAULT / ESTOP | `Stop_Now` 立即停止 + `faultCnt++` | 断使能（**不是** `Stop_Now`，见 §7） |

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
| **加新的执行机构**<br>接口不限 CAN：PWM / GPIO / 串口都行 | `auxiliary.c` / `auxiliary.h` | ① `aux_Init()` 初始化<br>② `aux_Step()` 加周期处理<br>③ 在 `chassis_OnEnter()` 补该机构的使能 / 失能调用<br>④ 仅当受上位机 **CAN** 指令控制时，才在 `aux_HandleCmd()` 加 `case` |
| **加底盘相关逻辑**<br>新运动模式、IMU 融合、更好的里程计 | `chassis_drive.c` | `chassis_UpdateOdom()` / `chassis_InverseKinematics()` / `chassis_DriveStep()` |
| **改状态迁移**<br>加新状态、改转移条件 | `chassis.c` | 只动 `s_transTable[]` 和 `chassis_OnEnter()` 的分支 |
| **改协议**<br>新指令码、新地址 | `chassis.h` + `BSP/Emm_V5.c` | 功能码枚举在 `chassis.h`，协议字节组装在 `Emm_V5.c` |
| **改任务编排**<br>地图、动作序列 | `nav.c` | `s_defaultMap[]` 换成赛题路线；接入点见 §6 |
| **加数据源**<br>第二个相机、新传感器串口 | `input.c` | `InputCh_t` 加通道 + `input_Init()` 补引脚与 DMA；**解析定稿后加在 `input_Step()` 内**（视觉模块 2 的解析负责填 `visionSlot`） |
| **接避障层**<br>反应式优先级抑制 | 新建 `avoid.c`（建议） | 排在 `nav_Step()` 之后：后写的覆盖先写的即得优先级，无需新仲裁机制 |
| **改状态屏内容** | `oled.c` | `oled_ShowStatus()` |
| **换舵机 / 标定行程** | `servo.h` | `SERVO_PULSE_MIN_US` / `SERVO_PULSE_MAX_US` |

### 未接入模块怎么接进来

四个新模块都按"**硬件就绪、逻辑自洽、只差一行调用**"的形态交付：

| 模块 | 接入动作 | 前置条件 |
| --- | --- | --- |
| `nav` | 在 `chassis_ControlStep()` 的 `if (state != CHS_RUNNING) return;` **之后**、`chassis_DriveStep()` **之前**加 `nav_Step()` | **里程计必须先标定**（`chassis_drive.h` 的轮径 / 轮距 / 轮序 / 符号仍是占位值）；另需把 `chassis_DriveStep()` 姿态环的给定 `0.0f` 换成 `nav_GetTargetYaw()`，转向才会生效 |
| `input`（三条串口） | 在控制环里加 `input_Step()`（200 Hz 已高于任何相机帧率），另按需加 `vision_Step()` | 无。串口接收走 DMA + IDLE 中断，**不开新任务**。⚠ 帧格式未定稿前字节只停在环形缓冲、由 `input_Read()` 取用；无人取用时 `lockCnt` 会持续增长 |
| `oled` | 加 `oled_Step()`（内部已按 `OLED_REFRESH_MS` 限速） | 无。注意刷新是阻塞式软件 I2C，**不要放进对时序敏感的临界区** |
| `servo` | 由上层直接调 `servo_SetAngle()` / `servo_SetPulse()` | 无。若要受安全闸门约束，需在 `chassis_OnEnter()` 的 IDLE / FAULT / ESTOP 分支登记失能 |

> `nav_Step()` 放在 RUNNING 闸门**之后**是关键：非 RUNNING 时它根本不会被执行，
> 于是急停、故障、停车的保护全部免费继承，任务层不需要任何额外的安全判断。

### 视觉模块 2（`input.c` 的视觉段）：三项未定稿

视觉模块 2 以"**接口与流程完整、内容格式留空**"的形态落地。留空的是**内容**，不是结构：

| 留空项 | 位置 | 现状 | 填写时机 |
| --- | --- | --- | --- |
| **通信帧格式** | `input.h` 视觉段 + `input_Step()` 内的解析（**尚未实现**） | 帧头 `0xA5` / 帧尾 `0x5A` / `VisionObst_t` 字段量纲，**全是占位约定** | 视觉模块 2 规格确定后 |
| **任务号承载位置** | `VisionTask_t` | 语义已定（整数任务号，`uint8_t`，`VISION_TASK_ID_NONE` = 本帧无任务），**帧内字节位置未定** | 视觉模块 2 规格确定后 |
| **视场角 / 安装姿态** | `VISION_FOV_DEG` / `VISION_MOUNT_H_MM` / `VISION_MOUNT_PITCH_DEG` | `0` = 未标定 | 实测后 |

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
| 承载内容 | 物块颜色 | 障碍物列表 + 任务号（二维码），视距 50 cm | 待定（本次只搭骨架） |
| 状态 | 已落地，未接入控制环；帧格式留空 | 同左 | 硬件已就绪，协议未定 |

### 新机构接入系统要做三件事

> 完整版（含消息码申请、路由表注册）见 **§2.5 新增模块的四个动作**。

**只有第 ③ 项和 CAN 有关** —— 走 PWM 的机构直接跳过第 ③ 步即可。

1. **动作登记** → `chassis_OnEnter()` 里补 IDLE 失能 / FAULT-ESTOP 安全停（**与接口无关**）
2. **周期处理** → 挂进 `aux_Step()`（**与接口无关**）
3. **指令分发** → `aux_HandleCmd()` 加 `case`（**仅当它受上位机 CAN 指令控制时**）

### 两条红线

1. **`chassis_ControlStep()` 不许塞算法。** 它是骨架编排，只负责按顺序调用别人：
   `chassis_DriveWatchdog()` → `aux_Step()` → `chassis_CheckBraking()` →
   `if (state != RUNNING) return;` → `chassis_DriveStep()`。
   新模块的周期处理挂在 ①② 之间，不要改动这个顺序。
2. **加新机构时别碰 `chassis.c` 的状态机逻辑。** 只需在 `chassis_OnEnter()` 里加一行调用。
   状态机永远只认"系统级"的事，机构细节全留在 `auxiliary.c`。

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

## 8. 编码约定

> **本节只讲"为什么这么定"，规范条文（含完整注释模板）以 `../../AGENTS.md` 开头的
> 「⛔ 硬约束」节为准，本文件不重复条文。** 下面只补充不在那节里的工程侧说明。

- **注释风格**：函数详细注释放 `.c` 定义上方（`@brief` / `@param` / `@note` / `@author camelliahm`），
  调用处只允许行尾短注，`.h` 声明处最多一行行尾注。**模块级"说明性"内容放本 README。**
- **精简凝练**：`@note` 一般 1~2 行，只留"坑"与调用前提；同一信息不在 `@note` 与函数体内重复。
  **分区标题统一一行** `/*---- 标题 ----*/`（填到总宽 54 列），不分四行、不用 `/*====*/`。
  ⚠ 历史遗留：`nav.c` / `input.c` / `oled.c` / `main.c` 里曾有一批四行分隔框，
  已于 2026-10-03 全部压成一行。
- **语义必须准确**：写进注释的命令行为要与 `BSP/Emm_V5.c` 实现核对过。
  易错一对——`Emm_V5_Stop_Now`（`0xFE/0x98`）是**保持使能**的减速停止；
  **撤保持电流的是 `Emm_V5_En_Control(false)`（`0xF3/0xAB`）**。
- **排除范围**：`PID_Mod.*`（用户排除）、`Emm_V5.c`（张大头官方协议库）、
  `stm32f4xx_it.c`（ST 官方模板）不改其注释。
- **头文件**：include guard + `.c` 首行包含自身头文件。
- **文件编码 / 换行**：`APP/`、`BSP/`、`DRIVERS/`、`BASE/`、`Mycode/` 下均为 **UTF-8 无 BOM + CRLF**。
  唯一例外是 `FreeRTOS/FreeRTOSConfig.h`（无 BOM + 纯 LF）。
  ⚠ **不要对源码做批量换行符 / 编码转换**——历史上曾有脚本把所有 `n` 换成 `rn`，
  整片文件报废（详见 `MEMORY.md` §5.1）。
- **共享状态**：模块级全局用 `__IO` 修饰并经 `extern` 在头文件暴露。
  `__IO` 只保证"每次访问都真正读写内存"，**不是同步手段** —— 结构体多字段的组合读取
  仍可能被抢占切成半新半旧，需要一致快照时用队列传副本或临界区。
- **编译约束**：ARMCC AC5 + `--c99` + `-O0` + `--MICROLIB`。不要用 GCC 语法，不要假设
  `printf` 可用。
- **禁用**：调度器启动后禁止 `delay_ms()` / `delay_us()`（它们会重配并关闭 SysTick），
  改用 `vTaskDelay()`。

### 8.1 `PID_Mod` 使用避坑（2026-10-01 逐条核对源码）

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

## 9. 构建

> **操作步骤、五处同步清单、命令行与"两条链路都要跑"的纪律见 `../../AGENTS.md` 的
> 「改完配置要自己收尾」与「构建与烧录」两节**，本文件只记**架构侧要点与产物现状**。

两套配置**必须同步维护**（Keil 与 EIDE 是独立配置）：

- **Keil µVision 5 + ARM Compiler 5**：`PRJ/STM32_CAN_CMD.uvprojx`，目标 `V1.0`。
- **VSCode + EIDE**：`PRJ/.eide/eide.yml`。

**新增编译单元时两处都要加。** 输出目录 `PRJ/Objects/`（Keil，产物 `STM32F4.hex`）、
`PRJ/build/V1.0/`（EIDE，产物 `STM32_CAN_CMD.hex`）。

**当前基线**（2026-10-04，`input` / `vision` 合并为 `input` 之后，Keil 全量重建）：
**0 Error / 0 Warning**，
`Code = 22868`、`RO-data = 692`、`RW-data = 232`、`ZI-data = 23856`
→ **Flash 23.2 KB / 512 KB (4.5%)，RAM 23.5 KB / 128 KB (18.4%)**。

- 合并净变化（对比 `23492/676/228/23484`）：Code **−624 B**、RO **+16 B**、RW **+4 B**、ZI **+372 B**。
  Code 下降来自移除文本行解析与二进制帧解析；ZI 上升来自第三条串口并入同一汇聚层
  （多 1 路 DMA 缓冲 256 B + 环形缓冲 512 B，同时去掉了旧的 `vision` 独立缓冲）。
- 更早基线依次为 `22748`（`ALL_Init()` 重构）/ `22740`（四模块落地）/ `19428`（路由表落地）。
- **完整历史序列见 `.workbuddy/memory/MEMORY.md` §1。**

> `ZI-data` 里的 16384 B 是 `heap_4` 的 `ucHeap`，**所有任务栈与内核对象都从这 16 KB
> 动态分配**（不在 ZI 里单独体现）。增删任务 / 队列 / 信号量前先核算堆占用。

---

## 10. 待硬件确认

> 这份清单只记**"填什么值、什么时候填"**；各占位值在代码里的确切位置与换算关系见
> 上文对应小节。

**参数标定类**

- 麦轮几何参数（轮径 60 mm、轴距 / 轮距 200 mm、LX / LY 100 mm）全是**占位值**，必须实测标定。
- 轮序（0 = 左前 1 = 右前 2 = 左后 3 = 右后）与正负号需与实际装配核对。
- 丝杆驱动器细分 `LIFT_SCREW_MICROSTEP = 16` 仍是**占位值**，必须按实车实测后改
  （`LIFT_PULSE_PER_MM`、`LIFT_ARRIVE_WINDOW` 会随之自动换算）。
  **导程已确认为 2 mm/圈**，不必再改 `LIFT_SCREW_LEAD_MM`。
- 有效行程 `LIFT_STROKE_MM = 300.0f` 也是**占位值**，需按实际丝杆长度标定。
- 舵机脉宽范围 `SERVO_PULSE_MIN_US = 500` / `SERVO_PULSE_MAX_US = 2500` 与
  `SERVO_ANGLE_MAX = 180` 是**通用值**，需按实际舵机型号标定（数字舵机常见 500~2500 µs，
  部分型号只有 1000~2000 µs）。
- 姿态环 PID 无控制周期无关性，**改 `CHASSIS_CTRL_PERIOD_MS` 必须重新整定**。

**协议核对类**

- 驱动器回包字节序（`S_VEL` `0x35` / `S_CPOS` `0x36` / `S_FLAG` `0x3A` / `S_OFLAG` `0x3B`）
  目前**按 ZDT 手册推测**，需上硬件实测。**若字节序或功能码不符，`vel` 会显示为乱值，
  进而让停稳判断失效。**
- 三条串口波特率 `INPUT_BAUD = 115200` 是**约定值**，需与视觉 1 / 视觉 2 / 串口屏的实际输出核对。
  `VISION_RANGE_MM = 500` 已实测，但视场角与安装姿态（`VISION_FOV_DEG` 等）仍留空。
- **`Emm_V5_Auto_Return_Sys_Params_Timed()` 的关闭方式未验证**：进入 READY 时会打开四轮的
  20 ms 定时上报（`S_VEL`），但代码里没有"关闭上报"的路径——退出 READY 后驱动器仍会继续上报。
  手册上 `time_ms = 0` 应可关闭（未实测），**离开 READY 时请补上，否则会白占约 5% 总线带宽**。
- **三条串口的解析都尚未实现**：当前只把到达字节搬进环形缓冲。旧的"帧头哨兵 `'\n'` +
  `resyncCnt` 重同步"那套无校验兜底已随 `input` / `vision` 合并一并移除。
  定稿后建议直接按"**按长度收齐 + 校验通过才交付**"实现（而不是无校验拼行），
  并保留丢弃计数 `lockCnt` 作为诊断。
- **升降轴限位开关必须已接线**，否则回零会一直撞到底（15 s 超时保护兜底）。
- OLED 从机地址按 7 位 `0x3C` 左移 1 位写成 `OLED_I2C_ADDR = 0x78`；
  若模块实际用 `0x3D`，需同步改这个宏。

**视觉模块 2（USART3）的留空项**

| 留空项 | 位置 | 现状 | 填写时机 |
| --- | --- | --- | --- |
| **通信帧格式** | `input.h` 视觉段 + `input_Step()` 内的解析（尚未实现） | 帧头 `0xA5` / 帧尾 `0x5A` / `VisionObst_t` 字段量纲，**全是占位约定** | 视觉模块 2 规格确定后 |
| **任务号承载位置** | `VisionTask_t` | 语义已定（整数任务号，`uint8_t`），**帧内字节位置未定** | 视觉模块 2 规格确定后 |
| **视场角 / 安装姿态** | `VISION_FOV_DEG` / `VISION_MOUNT_H_MM` / `VISION_MOUNT_PITCH_DEG` | `0` = 未标定 | 实测后 |

**有效视距已实测**：`VISION_RANGE_MM = 500`（50 cm）。
**帧格式与视场角填完前避障层不得放行任何速度。**（视距如何决定车速上限，见 §6。）

---

## 附：架构图

见 `DOC/`：

- `DOC/01_函数调用链与开发落点.html`
- `DOC/02_状态机迁移图.html`
- `DOC/03_代码架构全景图.html`（另有 `.svg` / `.png` 两份同图）
- `DOC/04_main初始化收敛到ALL_Init.svg`
- `DOC/index.html`（索引）
