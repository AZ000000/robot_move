# robot_move

履带式底盘（差速驱动）的**原理说明 → 嵌入式固件 → 浏览器仿真验证**全链路工程。

一句话概括：**上位机只发两个数（线速度 `v`、角速度 `ω`），STM32 把这两个数解算成左右两条履带各自的目标速度，再用双路 PID 闭环把目标速度变成 PWM 占空比输出给电机；转向本质就是让左右两侧的速度不一样。**

本仓库包含三份可独立使用的交付物：

| 交付物 | 形式 | 用途 | 入口 |
|--------|------|------|------|
| 机械原理技术文档 | 单页 HTML（7 章） | 讲清履带"为什么会动"：结构、接地压强、静止铺地模型、转向机构 | [tracked-chassis-tech/tracked-chassis-tech.html](tracked-chassis-tech/tracked-chassis-tech.html) |
| 嵌入式控制白皮书 | 单页 HTML（9 章） | 讲清"怎么控制"：芯片选型、管脚、协议、运动学、PID、状态机、安全保护 | [tracked-chassis-control/tracked-chassis-control.html](tracked-chassis-control/tracked-chassis-control.html) |
| 固件 + 实时仿真器 | C 代码 + 浏览器 Canvas | 固件可直接烧板；仿真器与固件逻辑 1:1，先在电脑上看效果 | [tracked-chassis-control/sim/simulator.html](tracked-chassis-control/sim/simulator.html) |

---

## 目录结构

```
robot_move/
├── README.md
│
├── tracked-chassis-tech/                 # ① 机械原理文档
│   ├── tracked-chassis-tech.html
│   └── _shared/js/mermaid.min.js
│
└── tracked-chassis-control/              # ② 控制白皮书 + ③ 固件 + 仿真器
    ├── tracked-chassis-control.html      #    控制白皮书（9 章）
    ├── _shared/                          #    字体 + Mermaid（离线可用，无需联网）
    │
    ├── firmware/                         #    STM32 HAL 固件（C，约 1000 行）
    │   ├── main.c / main.h               #      入口 + HAL 回调 + 主循环调度
    │   ├── core/                         #      纯业务逻辑，不依赖 HAL，可单元测试
    │   │   ├── chassis_config.h          #      ★ 所有可调参数集中于此
    │   │   ├── chassis_types.h           #      全局类型 + g_chassis 运行时状态
    │   │   ├── kinematics.c / .h         #      逆运动学 + 三种转向模式选择
    │   │   ├── speed_planner.c / .h      #      加减速平滑（S 曲线简化版）
    │   │   └── pid.c / .h                #      增量式 PID（积分抗饱和）
    │   ├── drivers/                      #      HAL 对接层
    │   │   ├── encoder.c / .h            #      正交解码 → 线速度换算
    │   │   └── motor.c / .h              #      PWM + 方向 + 制动 + 变化率限制
    │   └── proto/                        #      通信协议层
    │       ├── ringbuf.c / .h            #      UART RX 环形缓冲 + 帧检测
    │       ├── cmd_parser.c / .h         #      ASCII + 二进制双协议解析
    │       └── crc16.c / .h              #      Modbus CRC-16
    │
    └── sim/                              #    浏览器仿真器
        ├── simulator.html                #    入口页面（Canvas 俯视图 + 控制面板）
        └── js/
            ├── kinematics.js             #    ← 对应 firmware/core/kinematics.c
            ├── speed_planner.js          #    ← 对应 firmware/core/speed_planner.c
            ├── pid.js                    #    ← 对应 firmware/core/pid.c
            ├── physics.js                #    被控对象模型（真机里由电机+地面代替）
            ├── cmd_parser.js             #    ← 对应 firmware/proto/cmd_parser.c
            ├── simulator.js              #    ← 对应 firmware/main.c 的主循环调度
            ├── render.js                 #    Canvas 绘制
            └── ui.js                     #    面板交互
```

**分层原则**：`core/` 不认识 HAL，只做数学；`drivers/` 只跟寄存器打交道；`proto/` 只做字节流到命令的翻译。换芯片只需要重写 `drivers/` 和 `main.c` 的初始化，业务逻辑一行不用改。

---

## 快速开始

### 1. 看文档（零依赖）

直接用浏览器打开两个 HTML 即可，字体和 Mermaid 均已本地化，断网也能正常渲染。

### 2. 跑仿真器（推荐先做这一步）

仿真器是纯静态页面（无模块加载、无网络请求），**双击 `sim/simulator.html` 也能直接跑**。推荐用本地服务器打开，便于测试和避免部分浏览器的 `file://` 字体/缓存策略差异：

```bash
cd tracked-chassis-control
python3 -m http.server 8766
# 浏览器打开 http://localhost:8766/sim/simulator.html
```

> 如果改了 `sim/js/*.js`，页面可能仍用缓存版本。`simulator.html` 里的 `<script>` 已带 `?v=` 版本号，改动后把版本号递增或强制刷新（Ctrl+Shift+R）。

**界面说明**

| 区域 | 内容 |
|------|------|
| 左侧 Canvas | 俯视图：网格、底盘车体、左右履带（条纹颜色随实际速度方向变化，绿=正转 / 红=反转）、朝向箭头、黄色速度矢量、蓝色轨迹残影 |
| 右上状态表 | 每 10 ms 刷新一次：位姿 `x/y/θ`、实际 `v`/`ω`、左右履带实际 / 目标速度、PWM 输出、转向模式、tick 计数 |
| 右下控制区 | `START / STOP / RESET`、6 个预设轨迹按钮、ASCII 命令输入框、命令日志 |

**6 个预设按钮**（与固件协议完全相同）

| 按钮 | 命令 | 预期现象 |
|------|------|----------|
| 直行 | `CMD:MOV,v=0.40,ω=0.00` | 模式 `DIFF`，两条履带同速，沿箭头方向直线前进 |
| 左转弧线 | `CMD:MOV,v=0.40,ω=0.80` | 模式 `BRAKE`，左履带停、右履带加速到 0.80，走小半径左弧 |
| 右转弧线 | `CMD:MOV,v=0.40,ω=-0.80` | 模式 `BRAKE`，右停左加速，走小半径右弧 |
| 原地逆时针 | `CMD:MOV,v=0.00,ω=1.50` | 模式 `PIVOT`，左右履带反向对转，原地旋转、半径 = 0 |
| 原地顺时针 | `CMD:MOV,v=0.00,ω=-1.50` | 同上，反向 |
| 后退 | `CMD:MOV,v=-0.30,ω=0.00` | 两条履带同时反转，倒退 |

也可以在输入框直接敲命令（回车发送），例如 `CMD:MOV,v=0.50,ω=1.20`、`CMD:STOP`、`CMD:RESET`。

### 3. 烧固件到实机

见下文「固件移植到实机」。

---

## 工作原理速览

### 控制链路（自下而上 4 层）

```
① 上位机（你 / ROS / 遥控器）
      只发两个数：v（m/s，负=后退）   ω（rad/s，正=左转）
                │
                ▼  UART 115200
② MCU 运动学层  kinematics_full(v, ω)
      算左右履带目标速度，并按 |ω| 自动选转向模式
                │
                ▼
③ 控制层  速度平滑 + PID × 2（100 Hz）
      目标速度 → PWM 计数值（±65535）
                │
                ▼
④ 驱动层  PWM + DIR 电平 → H 桥 → 电机
      电机转 → 履带卷带铺地 → 地面反力 → 车体前进
                │
                └──→ 编码器 tick 反馈回 ③（闭环）
```

### 逆运动学（核心公式）

```c
vL = v - ω * WHEELBASE_M / 2;   // 左履带目标线速度
vR = v + ω * WHEELBASE_M / 2;   // 右履带目标线速度
```

`WHEELBASE_M = 0.32 m` 为两侧履带中心距。直行时 `ω = 0` 两轮同速；`ω` 越大左右差越大，转弯越急。

### 三种转向模式（自动切换，无需上位机指定）

| 模式 | 触发条件（按判断顺序） | 固件做法 | 效果 |
|------|------------------------|----------|------|
| `STEER_PIVOT` 原地反转 | `\|v\| < 0.05` 且 `\|ω\| > 0.5` | `vL = -ωL/2`，`vR = +ωL/2` | 转向半径 = 0，原地打转（电流最大、履带磨损最重，不宜长时间保持） |
| `STEER_BRAKE` 单边制动 | 不满足上条，且 `\|ω\| > 0.5` 且 `\|v\| > 0.05` | 内侧 `vL=0`，外侧 `vR=2v` | 小半径急转，响应快 |
| `STEER_DIFF` 平滑差速 | 其余情况（`\|ω\| ≤ 0.5`） | 两侧同向不同速 | 大半径弧线，最平顺 |

判断顺序很重要：先判 PIVOT（`v` 几乎为零时才可能原地转），再判 BRAKE（`v` 不为零时用单边制动），否则差速。

> 注意：`chassis_config.h` 里的 `STEER_DIFF_MAX_OMEGA` 和 `STEER_BRAKE_MAX_OMEGA` 目前只作为文档性常量定义，`kinematics_full()` 实际判断用的是 `STEER_BRAKE_MIN_OMEGA` 加硬编码的 `0.05`。若想让单边制动在 `|ω| > 1.0` 时自动升级为原地反转，需要在 `kinematics.c` 里补这条判断（仿真器同样）。

阈值常量集中在 [chassis_config.h](tracked-chassis-control/firmware/core/chassis_config.h)，可按实际车体调整（`|v| > 0.05` 这个 0.05 目前在代码里是硬编码，调整时记得两处一起改：`kinematics.c` 与 `sim/js/kinematics.js`）。

### 三个并发执行路径

| 路径 | 触发 | 频率 | 干什么 |
|------|------|------|--------|
| ① UART RX 中断 | 每收到 1 字节 | 事件驱动 | 环形缓冲入队 → 检测完整帧 → 解析 → 写 `g_chassis` 目标值 → 置 `cmd_new` |
| ② 主循环 | `while(1)` + `HAL_Delay(1)` | 约 1 kHz | 检查 `cmd_new` → `kinematics_full()` 解算 → 命令超时保护（500 ms 无命令自动软停车） |
| ③ TIM2 中断 | 定时器溢出 | 100 Hz（10 ms） | 读编码器 → 速度平滑 → PID × 2 → 写 PWM（这一段禁止阻塞） |

---

## 命令协议参考

### ASCII 行协议（调试 / 手动控制，人类可读）

行尾必须带 `\r\n`。

| 命令 | 说明 |
|------|------|
| `CMD:MOV,v=0.30,ω=0.50` | 设置目标线速度 0.30 m/s、角速度 0.50 rad/s |
| `CMD:MOV,v=0.40` | `ω` 可省略，默认沿用／置 0 |
| `CMD:STOP` | 目标清零，状态切 `STATE_IDLE`，软停车（靠减速斜坡） |
| `CMD:BRAKE` | 立即短路制动（H 桥双下桥导通），用于紧急场景 |
| `CMD:QUERY:ENCODER` | 请求编码器数据（返回 `1`，由上层组织应答帧） |
| `CMD:QUERY:STATUS` | 请求运行状态（返回 `2`） |

### 二进制帧协议（高频 / ROS 场景）

固定 **13 字节**：

```
偏移  长度  字段
 0     1    SOF    固定 0xA5
 1     1    CMD    0x01=MOV, 0x02=STOP
 2     4    v      float32（小端）
 6     4    ω      float32（小端）
10     2    CRC16  Modbus CRC-16，计算范围 = 前 10 字节
12     1    EOF    固定 0x5A
```

`CRC` 不匹配时**静默丢弃**（返回 -2），不回复错误帧，避免噪声干扰时反复重传。

### 限幅（固件内部强制执行，上位机无法越界）

```c
|v|     ≤ V_MAX_MPS     = 0.8 m/s
|ω|     ≤ OMEGA_MAX_RADPS = 2.0 rad/s
```

---

## 硬件方案与接线

**推荐配置**：STM32F103C8T6（"蓝板"）+ TB6612（代码注释里写作 MTB6612）/ L298N 双 H 桥 + 带霍尔编码器的减速电机 × 2。

| 管脚 | 功能 | 用途 |
|------|------|------|
| PA0 / PA1 | TIM2 CH1/CH2 正交解码 | 左编码器 A / B 相 |
| PA6 / PA7 | TIM3 CH1/CH2 正交解码 | 右编码器 A / B 相 |
| PB0 | PWM 输出 | 左电机驱动 PWM |
| PB1 | PWM 输出 | 右电机驱动 PWM |
| PB3 | GPIO 输出 | 左电机方向 DIR |
| PB4 | GPIO 输出 | 右电机方向 DIR |
| PB5 | GPIO 输出 | 总使能 EN（拉低 = 紧急停，**必做**） |
| PA9 / PA10 | USART1 TX / RX，115200 | 上位机 USB-TTL 或蓝牙 |
| PB6 / PB7 | I²C1 SCL / SDA | MPU6050 航向（可选，用于航向闭环） |
| PA4 | ADC_IN4 | 履带张力模拟量（可选，防脱轨） |

> 注意：PWM 与编码器不能共用同一个定时器通道。上面的方案里 TIM2 只做左编码器、TIM3 只做右编码器、PWM 用独立定时器通道——实际分配以你的 `.ioc` 为准，只要改 `main.c` 的 `MX_*_Init()` 和 `motor_apply_raw()` 即可。

---

## 固件移植到实机

固件是**框架代码**，HAL 初始化部分故意留空（`main.c` 末尾的 `SystemClock_Config()` / `MX_GPIO_Init()` 等是空桩），需要 CubeMX 生成真实实现。步骤：

1. **新建 CubeMX 工程**（STM32F103C8T6），配置：
   - TIM2 / TIM3 的 Encoder Mode（TI1 and TI2）
   - 一个定时器输出 PWM（16 bit，`PWM_RES = 65535`）
   - 一个定时器做 100 Hz 更新中断（`CONTROL_HZ = 100`）
   - USART1 异步 115200，开全局中断
   - PB3 / PB4 / PB5 推挽输出
2. 生成代码（Toolchain = Makefile 或 MDK-ARM）。
3. 把 `firmware/` 下所有 `.c/.h` 拷进生成工程的 `Core/Src`、`Core/Inc`（或新建 `Application` 分组）。
4. 用 CubeMX 生成的 `SystemClock_Config()` / `MX_*_Init()` **替换** `main.c` 末尾的五个空桩函数。
5. 在 `main.c` 里把 `htim_pwm_left` / `htim_pwm_right` / `htim_ctrl` / `huart1` 指向 CubeMX 生成的句柄。
6. 补齐 `motor_set_dir()` 里 `MOTOR_RIGHT` 分支的 GPIO 操作（PB4 那一路）。
7. 编译烧录，串口发 `CMD:MOV,v=0.20,ω=0.00\r\n`，观察是否前进。

> 上电顺序建议：先只接电机不接履带，确认方向和 PWM 正确；再接履带空转；最后落地。

---

## 参数标定

**所有硬件相关参数集中在 [chassis_config.h](tracked-chassis-control/firmware/core/chassis_config.h)，换板子只需要改这一个文件。**

| 参数 | 默认值 | 含义 / 怎么标定 |
|------|--------|----------------|
| `WHEELBASE_M` | 0.32 | 两侧履带中心距，卷尺量 |
| `DRIVE_WHEEL_R_M` | 0.05 | 驱动链轮节圆半径 |
| `V_MAX_MPS` | 0.8 | 实际满载最大速度，跑一次测 |
| `PULSE_PER_REV` | 390 | 电机单相 PPR，查电机规格书 |
| `GEAR_RATIO` | 98 | 减速箱减速比 |
| `ENCODER_MULT` | 4 | 正交解码倍频（4×） |
| `PWM_MIN_STATIONARY` | 800 | 电机刚开始转的最小 PWM（死区标定），否则 PID 小误差时电机不转 |
| `PWM_SLEW_RATE` | 200 | 每周期 PWM 最大变化量，防电流尖峰 |
| `MAX_ACCEL_MPS2` | 0.5 | 加速度上限，太大起步打滑 |
| `MAX_DECEL_MPS2` | 1.0 | 减速度上限，可比加速更猛 |
| `PID_KP/KI/KD` | 0.8 / 0.02 / 0.15 | 见下方调参 |
| `CMD_TIMEOUT_MS` | 500 | 上位机失联后自动停车时间 |

**编码器速度换算**（`encoder.c`）：

```
v = 2π·R · ticks / (PULSE_PER_REV × ENCODER_MULT × GEAR_RATIO × dt)
```

### PID 调参口诀

1. **先只开 P**（`KI=KD=0`），从小到大加 `KP`，直到速度能跟上但开始轻微振荡，然后退回 70%。
2. **加 I** 消静差：缓慢加大 `KI`，直到稳态误差在 1% 以内且不产生低频摆动。
3. **最后加 D**（可选）：`KD` 只用来抑制超调，通常取 `KP` 的 1/10 ~ 1/20。在低成本履带底盘上 D 项极易被编码器量化噪声放大而发散，**宁可不要**。
4. 两个轮子必须**分别调**，因为左右负载、摩擦不一致。

> ⚠️ 仿真器里的 PID 默认参数（`Kp=0.35 / Ki=1.2 / Kd=0.01`，输出归一化到 ±1.0）是为 JS 模型标定的，**不能直接抄到 C 固件**。固件用的是计数值量纲（±65535），增益量级完全不同。

---

## 安全与保护

三级递进保护（设计方案见白皮书第 9 章；**阈值已定义在 `chassis_config.h`，但电流采样 / 张力采样的判断逻辑尚未写入固件**，接传感器后需补在 `ctrl_tick_100hz()` 里）：

| 等级 | 触发条件 | 动作 |
|------|----------|------|
| L1 降载 | 电流 > 1.5× 额定（`OVERCURR_L1_RATIO`） | PWM 降到 70%，继续运行并上报 |
| L2 停车 | 电流 > 2.0× 额定 | 保护性停车，状态切 `STATE_FAULT` |
| L3 硬切断 | 编码器 50 ms 无 tick（堵转）/ 张力低于 `TENSION_FAULT_N`（脱轨风险） | 关闭 EN，H 桥高阻 |

另外两条软件保护已在主循环实现：

- **命令超时**：当状态为 `STATE_RUN` 时，500 ms 没收到新命令 → 目标清零软停车（防上位机崩溃后小车一直跑）。
- **变化率限制**：`motor_apply_slew()` 限制每周期 PWM 变化量，避免突变电流。

**硬件急停必须独立**：EN 脚串联一个常闭急停按钮直接拉低，不经过软件。

---

## 仿真器与固件的一致性与差异

| 模块 | 固件 | 仿真器 | 是否一致 |
|------|------|--------|----------|
| 逆运动学 + 转向模式 | `core/kinematics.c` | `sim/js/kinematics.js` | ✅ 同阈值、同公式 |
| 速度平滑 | `core/speed_planner.c` | `sim/js/speed_planner.js` | ✅ 同加减速上限 |
| PID | `core/pid.c` | `sim/js/pid.js` | ✅ 同结构（增益量纲不同，见上文） |
| 命令解析 | `proto/cmd_parser.c` | `sim/js/cmd_parser.js` | ✅ 同 ASCII 语法、同限幅 |
| 主循环调度 | `main.c` | `sim/js/simulator.js` | ✅ 同为 100 Hz 固定步长 |
| 被控对象 | 真电机 + 地面 | `sim/js/physics.js` | ❌ 仿真用一阶惯性 + 运动方程近似 |

**仿真器不能替代示波器和实车标定**，它的价值在于：验证运动学解算正确、转向模式切换逻辑正确、协议格式正确，并且能快速试出"什么样的 v/ω 组合能得到想要的轨迹"。

---

## 常见问题排查

| 现象 | 可能原因 | 排查 |
|------|----------|------|
| 仿真器点按钮车不动 | ① 浏览器缓存了旧 JS ② `START` 未点 ③ PID/量纲配置错误 | 强制刷新（Ctrl+Shift+R）；确认状态表 `tick` 在增长；检查 `motorL_out` 是否接近 0 |
| 仿真器速度爆炸 / 轨迹乱飞 | PID 的 D 项在高增益下发散 | 把 `Kd` 设为 0，降低 `Kp`；确认 PID 输出与 `physics.js` 的 `motorToSpeed` 量纲匹配 |
| 实车发命令不动 | ① EN 脚未拉高 ② PWM 定时器未 `HAL_TIM_PWM_Start` ③ DIR 脚接反 ④ 串口参数不对 | 先发 `CMD:BRAKE` 听是否有咔哒声；用万用表量 EN 和 PWM 脚；串口 115200 8N1 |
| 实车走不直 | 左右电机/摩擦不一致 ② PID 未分别标定 ③ 履带张紧不一致 | 分别标定 `pid_L` / `pid_R`；检查两侧履带张紧装置；必要时加 MPU6050 航向闭环 |
| 转向时电机堵转 / 电流大 | 原地反转模式在地面摩擦大时负载最高 | 避免长时间 PIVOT；调低 `OMEGA_MAX_RADPS`；确认电流保护阈值（`OVERCURR_*`）与实际电机匹配 |
| 履带脱轨 | 张力不足 | 调整张紧装置；接入 PA4 张力传感器做 L3 保护 |

---

## 已知限制 / 后续可做

- 固件的 `SystemClock_Config()` 等 5 个 HAL 初始化函数是空桩，必须由 CubeMX 生成替换。
- `motor_set_dir()` 里 `MOTOR_RIGHT` 分支未实现（占位注释），移植时需补 PB4 的逻辑。
- `encoder.c` 中"读 TIMx->CNT 求增量"的部分是占位（`enc_delta_this_cycle[]` 需由 HAL 层填充），速度换算常数 `K` 本身是完整可用的。
- 三级电流 / 张力保护只有阈值宏，没有采样与判断代码。
- `STEER_DIFF_MAX_OMEGA` / `STEER_BRAKE_MAX_OMEGA` 两个宏当前未被引用（见「三种转向模式」的说明）。
- 没有 IMU 航向闭环：目前是纯速度闭环，长距离直行会因左右偏差累积而跑偏。
- 没有里程计发布（ROS `odom` / `tf`），只有编码器原始 tick。
- 仿真器为 2D 俯视运动学模型，不含打滑、接地压强、履带形变等力学细节。

---

## 参考资料

- 履带底盘机械原理与结构：[tracked-chassis-tech/tracked-chassis-tech.html](tracked-chassis-tech/tracked-chassis-tech.html)
- 嵌入式控制白皮书（含完整公式推导、状态机、协议帧图）：[tracked-chassis-control/tracked-chassis-control.html](tracked-chassis-control/tracked-chassis-control.html)
- STM32F103C8T6 数据手册 / HAL 库文档（意法半导体官网）
