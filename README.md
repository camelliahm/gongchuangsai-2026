# STM32F407 CAN 底盘固件

四轮麦轮小车固件。主控 STM32F407VETx，FreeRTOS，通过 CAN1 与 Emm_V5 / ZDT（张大头）
步进闭环驱动器通信，并同时在该总线上接收上位机指令。

**代码里的注释只讲"这个函数干什么、参数怎么传、有什么坑"。**
模块定位、分层规则、扩展指引、接口约定这类"说明性"内容统一写在本文件，
**不要再塞进 `.c` / `.h` 里**。改架构时改这里，改函数行为时才改代码注释。

---

## 1. 分层结构

| 文件 | 职责 | 不该放什么 |
| --- | --- | --- |
| `Mycode/chassis.h` | **公共契约层**：地址宏、功能码枚举、状态/事件枚举、`Chassis_t`、任务参数、`chassis_*` 接口 | 各层私有类型与宏 |
| `Mycode/chassis.c` | **状态机 + 系统级调度**：迁移表、`chassis_OnEnter`、CAN 帧地址分流、三个任务体、控制环编排 | 任何控制算法 |
| `Mycode/chassis_drive.c/.h` | **底盘驱动层**：四轮动作 / 反馈 / 看门狗 / 停稳判断 + 里程计 / 麦轮逆解 / 姿态环 | 状态机迁移、升降轴 |
| `Mycode/aux.c/.h` | **辅助机构层**：底盘四轮之外的所有执行机构（现为升降轴），后续新机构也放这里 | 状态机迁移、底盘四轮 |

> **目录约定**：**新增的 `.c` / `.h` 一律放 `Mycode/`**，`APP/` 只保留 `main.c` 与 ST 模板
> （`stm32f4xx_conf.h`、`stm32f4xx_it.c/.h`）。加入新文件后 Keil 与 EIDE 两侧的
> include 路径都已包含 `../Mycode`，无需再改搜索路径。

三层之间**不直接互相调用**，通过两条通道交互：

- **向上报事件**：`chassis_PostEvent(EV_FAULT, ...)` 投给状态机
- **向下读状态**：只读 `chassis.state`

---

## 2. 启动与任务

`main()` 只做声明式装配，不含业务逻辑：

```
board_init()          ① 硬件初始化（NVIC、时钟、GPIO、CAN）
can_RtosInit()        ② 建 CAN 接收队列 + 发送互斥锁
chassis_Init()        ③ 建事件队列 → chassis_DriveInit() + aux_Init()
App_TaskCreate()      ④ 按任务表 s_tasks[] 建任务
vTaskStartScheduler() ⑤ 启动调度器（不返回）
```

前三个必须在建任务之前 —— 任务一起来就会立刻用队列和互斥锁。

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

## 3. CAN 地址分配

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

### 上位机指令功能码（`Data[0]` = 功能码，`Data[1..]` = 参数）

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

**C 侧控制接口**（`Mycode/aux.c`，两套并存）：

| 函数 | 单位 | 说明 |
| --- | --- | --- |
| `lift_MoveToMM(mm, vel, acc)` | mm | 升降到绝对高度（0 = 原点，向上为正），目标夹到 `[0, LIFT_STROKE_MM]` |
| `lift_MoveByMM(deltaMM, vel, acc)` | mm | 相对当前高度升降（正升 / 负降） |
| `lift_GetHeightMM()` | mm | 读当前高度 |
| `lift_MoveTo(clk, dir, vel, acc)` / `lift_MoveBy(delta, vel, acc)` | 脉冲 | 底层接口，mm 接口内部换算后转调它们 |

---

## 4. 状态机

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
| FAULT / ESTOP | `Stop_Now` 立即停止 + `faultCnt++` | 断使能（**不是** `Stop_Now`，见 §6） |

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

## 5. 后续开发在哪写代码

| 要做什么 | 改哪个文件 | 具体位置 |
| --- | --- | --- |
| **加新的执行机构**<br>接口不限 CAN：PWM / GPIO / 串口都行 | `aux.c` / `aux.h` | ① `aux_Init()` 初始化<br>② `aux_Step()` 加周期处理<br>③ 在 `chassis_OnEnter()` 补该机构的使能 / 失能调用<br>④ 仅当受上位机 **CAN** 指令控制时，才在 `aux_HandleCmd()` 加 `case` |
| **加底盘相关逻辑**<br>新运动模式、IMU 融合、更好的里程计 | `chassis_drive.c` | `chassis_UpdateOdom()` / `chassis_InverseKinematics()` / `chassis_DriveStep()` |
| **改状态迁移**<br>加新状态、改转移条件 | `chassis.c` | 只动 `s_transTable[]` 和 `chassis_OnEnter()` 的分支 |
| **改协议**<br>新指令码、新地址 | `chassis.h` + `BSP/Emm_V5.c` | 功能码枚举在 `chassis.h`，协议字节组装在 `Emm_V5.c` |

### 新机构接入系统要做三件事

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
   状态机永远只认"系统级"的事，机构细节全留在 `aux.c`。

---

## 6. 安全设计要点（务必保留）

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

## 7. 编码约定

- **注释风格**：函数详细注释放 `.c` 定义上方（`@brief` / `@param` / `@note` / `@author`），
  调用处只允许行尾短注，`.h` 声明处最多一行行尾注。**模块级"说明性"内容放本 README。**
- **排除范围**：`PID_Mod.*`（用户排除）、`Emm_V5.c`（张大头官方协议库）、
  `stm32f4xx_it.c`（ST 官方模板）不改其注释。
- **头文件**：include guard + `.c` 首行包含自身头文件。
- **文件编码 / 换行**：`APP/`、`BSP/`、`DRIVERS/`、`Mycode/` 下均为 **UTF-8 无 BOM + CRLF**。
  唯一例外是 `FreeRTOS/FreeRTOSConfig.h`（无 BOM + 纯 LF）。
  **不要对源码做批量换行符 / 编码转换。**
- **共享状态**：模块级全局用 `__IO` 修饰并经 `extern` 在头文件暴露。
  `__IO` 只保证"每次访问都真正读写内存"，**不是同步手段** —— 结构体多字段的组合读取
  仍可能被抢占切成半新半旧，需要一致快照时用队列传副本或临界区。
- **编译约束**：ARMCC AC5 + `--c99` + `-O0` + `__MICROLIB`。不要用 GCC 语法，不要假设
  `printf` 可用。
- **禁用**：调度器启动后禁止 `delay_ms()` / `delay_us()`（它们会重配并关闭 SysTick），
  改用 `vTaskDelay()`。

---

## 8. 构建

两套配置**必须同步维护**（Keil 与 EIDE 是独立配置）：

- **Keil µVision 5 + ARM Compiler 5**：`PRJ/STM32_CAN_CMD.uvprojx`，目标 `V1.0`。
- **VSCode + EIDE**：`PRJ/.eide/eide.yml`。

**新增编译单元时两处都要加。** 命令行编译验证：

```bash
cd PRJ && "/d/Keil_v5/UV4/UV4.exe" -b STM32_CAN_CMD.uvprojx -t V1.0 -o build_out.txt
```

输出目录 `PRJ/Objects/`（Keil，产物 `STM32F4.hex`）、`PRJ/build/V1.0/`（EIDE）。

当前状态（2026-10-01，Keil 全量重建）：**0 Error / 0 Warning**，
`Code = 19312`、`RO-data = 584`、`RW-data = 192`、`ZI-data = 20352`
→ **Flash 19.54 KB / 512 KB (3.8%)，RAM 20.06 KB / 128 KB (15.7%)**。

> `ZI-data` 里的 16384 B 是 `heap_4` 的 `ucHeap`，**所有任务栈与内核对象都从这 16 KB
> 动态分配**（不在 ZI 里单独体现）。增删任务 / 队列 / 信号量前先核算堆占用。

---

## 9. 待硬件确认

- 驱动器回包字节序（`S_VEL` `0x35` / `S_CPOS` `0x36` / `S_FLAG` `0x3A` / `S_OFLAG` `0x3B`）
  目前**按 ZDT 手册推测**，需上硬件实测。
- 麦轮几何参数（轮径 60 mm、轴距 / 轮距 200 mm、LX / LY 100 mm）全是**占位值**，必须实测标定。
- 轮序（0 = 左前 1 = 右前 2 = 左后 3 = 右后）与正负号需与实际装配核对。
- 驱动器细分 `LIFT_SCREW_MICROSTEP = 16` 仍是**占位值**，必须按实车实测后改
  （`LIFT_PULSE_PER_MM`、`LIFT_ARRIVE_WINDOW` 会随之自动换算）。
  **导程已确认为 2 mm/圈**（转一圈升 2 mm），不必再改 `LIFT_SCREW_LEAD_MM`。
- 有效行程 `LIFT_STROKE_MM = 300.0f` 也是**占位值**，需按实际丝杆长度标定 ——
  mm 接口（`lift_MoveToMM` / `lift_MoveByMM`）会把目标高度夹到 `[0, 行程]`。
- 升降轴限位开关必须已接线，否则回零会一直撞到底（15 s 超时保护兜底）。
- 姿态环 PID 无控制周期无关性，改 `CHASSIS_CTRL_PERIOD_MS` 必须重新整定。

---

## 附：架构图

见 `DOC/`：

- `DOC/01_函数调用链与开发落点.html`
- `DOC/02_状态机迁移图.html`
- `DOC/index.html`（索引）
