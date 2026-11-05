# 主从协同实现指南：Geomagic Touch → 上位机 → STM32F407 → 三轴机台

> 适用对象：本工作区的 Geomagic Touch（HID-USB，SN 22119000410）+ 野火 F407 霸天虎 + 三台 SRX02-E + 机台三自由度（直线推进 d、平面旋转 θ、夹爪 g）。
> 依据：`主从遥操作-下位机与桥接-交付说明.docx`（下称"交付说明"）、`A-4-3-MCU-LCD/` 工程、`A-4-3-MCU-LCD/三电机接线说明.md`、本机实测环境。
> 本文只讲"怎么把主从跑起来、怎么验证、哪里会翻车"，不重复 `Geomagic_Touch_连接与调试手册.md` 里的驱动安装细节（那份是主手单机调试）。

---

## 0. 先定“谁是准”：一切以 `A-4-3-MCU-LCD/` 的代码与实物为准

本项目的事实源是上一个交付成员的**实际搭建记录**：`A-4-3-MCU-LCD/User/main.c`、`Project/Output/Fire_F407ZG.hex`、`Project/Listings/Fire_F407ZG.map`、`三电机接线说明.md`。
后来的 `主从遥操作-下位机与桥接-交付说明.docx` 是 AI 处理产物，其中的行数、编译尺寸、协议细节、驱动器型号都可能有误。**凡冲突，以代码和实物为准。**

### 0.1 代码 vs 文档：冲突清单（左列为准）

| 项 | 代码 / 构建产物（准） | docx（AI 稿，可能出错） |
|---|---|---|
| `User/main.c` | **815 行** | 1227 行 |
| 串口遥控层 | **没有**：`grep Remote_` 0 处，`PING/HOME/STATUS` 均无；map 里也无 `Remote_*` 符号 | 完整 ASCII 协议 + 看门狗 + 回零 |
| 编译尺寸 | Code=18504 RO=7212 RW=188 ZI=4628；hex 72,935 B | Code=18848 RO=7320 RW=324 ZI=4540；hex 73,970 B |
| M3 夹爪 | `65×400 = 26000` 脉冲，2400 Hz，≈10.8 s | 90° = 800 脉冲，≈2 s |
| M1 默认速度 | **1600 Hz**（30 r/min） | 400 Hz（与代码不符） |
| 驱动器 | `三电机接线说明.md`：SRX02-E ×3，PC0–PC5，共阴 5V | 同为 SRX02-E（但照片里出现 DM542J + DM1-422C，需现场核对） |
| 构建时间 | 2026-11-04 20:36，`main.c` 与 hex/axf 同一时间戳 | — |

**结论：板子里就是这份“屏幕 6 键”固件，没有任何串口遥控层。** docx 描述的是一个设计意图，工作区里没有对应的可编译产物，也可能从未真正编译过——所以别按 docx 的行数/尺寸去核对，会越核越乱。

### 0.2 上板 10 秒复核（确认板子没被另刷过）

先确认没有别的程序占着串口（Touch 自己也会占一个虚拟 COM，见 §3.4），然后：

```powershell
python -c "import serial,time; s=serial.Serial('COM5',115200,timeout=0.5); s.write(b'PING\r\n'); time.sleep(0.2); print(repr(s.read(200))); s.close()"
```

- 无回复，或只回开机横幅（`欢迎使用野火开发板` / `三电机触摸控制程序启动`）→ 与代码一致，按本文从零加遥控层（§7）。
- 回 `b'OK PONG\r\n'` → 有人另烧过别的东西，先找回那份源码，别用 §7 重写。
- 回乱码 → 波特率不对，或打开的是 Touch 的虚拟串口。

### 0.3 落地主从还差什么（按缺口排序）

1. **串口遥控层（要自己写）**：现有固件只有屏幕按键，协议解析、位置计数、回零、斜坡、通信死区全都没有。§7 给了在现有工程上的最小增量；docx 里的 `Remote_*` 代码只能当设计参考，不能当已验证实现。
2. **上位机三个脚本**：`pc/bridge.py`、`pc/fake_board.py`、`pc/motor_console.py` 在工作区不存在（`**/*.py` 只搜到我自己的 `_read_docx.py`）。docx 附录 B/C/D 有全文，可照着重建，但这些代码**没上过真机**，重建后必须按 §8 修 4 处再验证。
3. **OpenHaptics SDK**：本机只装了 Touch 驱动（`C:\Program Files\3D Systems\Touch Device Drivers`，2023.10.11.1213），**没有 hd.lib / hd.h / 系统 hd.dll**；只有 Unity 演示里带了一个 `OpenHaptics_Demos\HapticDemo\HapticDemo_Data\Plugins\x86_64\hd.dll`，那是插件私有副本，别当 SDK 用。
4. **标定数据**：`三电机接线说明.md` §3 给了 **3200 脉冲/圈**（→ M2 电机轴 8.89 脉冲/度）；**M1 每毫米缺丝杠导程，只能实测**；M2 输出轴减速比未给；§4 的“90°/800 脉冲”与代码 26000 脉冲冲突，**以代码为准**。详见 §5。
5. **pyserial**：本机 Python 3.9 没装。

---

## 1. 系统全貌

```
   ┌──────────────┐  USB(CDC虚拟串口)   ┌───────────────────────────┐
   │ Geomagic     │  1 kHz 位姿/按键    │ Windows PC                │
   │ Touch 主手   │ ──────────────────► │  OpenHaptics(hd.dll)      │
   │ 6DOF 传感    │                     │  bridge.py  50 Hz 映射    │
   │ 3DOF 力反馈  │ ◄────────────────── │  （力反馈本版不做）       │
   └──────────────┘                     └────────────┬──────────────┘
                                                     │ USB-TTL(CH340) 115200 8N1
                                                     │ ASCII 一行一条，\r\n
   ┌─────────────────────────────────────────────────▼──────────────────┐
   │ 野火 F407 霸天虎  STM32F407ZGT6   USART1: PA9(TX)/PA10(RX)          │
   │  ├ 主循环 10 ms：收串口 → 解析指令 → 设目标；看门狗 1.5 s          │
   │  ├ TIM6 20 kHz 中断：相位累加器发 STEP 脉冲（1 脉冲 = 1 中断）      │
   │  └ 屏幕 6 个软按键：与串口共用同一套 Motor_* 运动函数（模式互斥）   │
   └───────────────┬──────────────────────────────┬────────────────────┘
                   │ PC0..PC5                     │ PC7（光电，上拉）
        ┌──────────▼──────────┐         ┌─────────▼─────────┐
        │ 高速光耦隔离板      │         │ 槽型光电 / NPN    │
        │ 3.3V 入 / 5V 出     │         │ 24V 供电，OC 输出 │
        └──────────┬──────────┘         └───────────────────┘
                   │ STEP±/DIR±（共阴）
        ┌──────────▼──────────┐
        │ SRX02-E ×3  3200/圈 │  24V 独立供电
        └──────────┬──────────┘
                   │ A±/B±
              M1 直线推进 / M2 旋转 / M3 夹爪
```

### 1.1 频率预算（决定"能不能跟得上"的硬约束）

| 环节 | 频率/时标 | 含义 |
|---|---|---|
| Touch 采样 | ~1000 Hz | 主手位姿更新率 |
| `bridge.py` 主循环 | 50 Hz（20 ms） | 每 20 ms 下发一次增量 |
| UART | 115200 8N1 | 一条 `M1 MOVE 1234 800\r\n` 约 20 B ≈ 1.7 ms |
| 板子主循环 | 10 ms | 收指令、看门狗、状态机 |
| TIM6 脉冲引擎 | 20 kHz | 每 50 µs 最多 1 个脉冲 → 理论上限 10 kHz（占空比 50%） |
| M1 原版固件默认速度 | 1600 Hz | = 30 r/min（3200 脉冲/圈） |
| M2/M3 默认 | 400 / 2400 Hz | 7.5 r/min / 45 r/min |
| 通信死区 | 1.5 s | 上位机 1.5 s 不发指令，板子自停 M1/M2 |
| 回零超时 | 15 s | 光电一直不触发就报错退出 |

**结论：整条链的瓶颈不是 USB、不是串口，是步进电机的可用转速。** 手速一旦超过 §6.1 算出的上限，指令就会在板子里排队，表现出来就是"越走越偏"。

---

## 2. 接线复核（照这张表逐根量，别信照片）

依据 `A-4-3-MCU-LCD/三电机接线说明.md`（这份是真实接线，和代码一致）：

### 2.1 F407 → 隔离板输入侧（3.3V 档）

| F407 | 隔离板 | 含义 |
|---|---|---|
| PC0 | X1+ | M1 STEP |
| PC1 | X2+ | M1 DIR |
| PC2 | X3+ | M2 STEP |
| PC3 | X4+ | M2 DIR |
| PC4 | X5+ | M3 STEP |
| PC5 | X6+ | M3 DIR |
| 任一 GND | Xn− / 公共负端 | 共地 |

### 2.2 隔离板 → SRX02-E（5V 档，共阴）

```
隔离板输出 GND ──► 三台驱动器的 STEP−、DIR−   （共阴）
Y1 → M1 STEP+    Y2 → M1 DIR+
Y3 → M2 STEP+    Y4 → M2 DIR+
Y5 → M3 STEP+    Y6 → M3 DIR+
```
`STEP+` 与 `DIR+` **不要短接**（那是下拉型 NPN 模块的接法，这块板不是）。

### 2.3 驱动器侧

| 项 | 设置 | 备注 |
|---|---|---|
| 供电 | 24V+ → V+，24V− → V− | 与信号线分开走，别共线束 |
| 电机 | A+/A−、B+/B− | 两相 |
| SW11 | OFF | STEP/DIR 模式 |
| 细分 | 3200 pulse/rev | 1.8° 电机 16 细分 |
| EN | 当前程序不用 | **不要拿 ENA 当急停**：竖轴会掉 |

### 2.4 三个必须实测的点

1. **方向**：原版固件里 `MOTOR1_DIR_INVERTED=1`、`MOTOR2_DIR_INVERTED=1`、`MOTOR3_DIR_INVERTED=0`（`User/main.c:41-43`）。换线、换驱动器后方向会反，先按 §4-L2 用屏幕按键确认，再动串口。
2. **光电类型**：交付说明明确写了"光电棕线不得接到 STM32 引脚"。先万用表量：
   - 棕 = +24V，蓝 = 0V，黑 = 输出。
   - 黑对蓝：遮挡时 0V、不遮挡 24V → **PNP**，绝不能直接进 PC7（会灌 24V 烧引脚），必须经光耦/分压。
   - 黑对蓝：遮挡时 24V、不遮挡 0V（或反之但高电平只有 3.3/5V）→ NPN 开集，可上拉到 3.3V 进 PC7。
   - 代码里 `SENSOR1_ACTIVE_LOW` 就是给这个极性留的开关，接之前先定死。
3. **共地**：隔离板输入侧 GND 必须与 F407 GND 同点；输出侧 5V 必须是隔离的独立电源，不要从板子 5V 取。

### 2.5 与照片不一致的地方（现场必须确认）

- 照片里有 **Leadshine DM542J + DM1-422C**（PUL/DIR/ENA/ALM 端子、5–24V 差分输入），而接线说明和固件都按 **SRX02-E ×3** 写。
- 两者接法不同：DM542J 是差分输入（PUL+/PUL−，可共阳/共阴），SRX02-E 按本文是共阴 5V。
- **先确认每根轴到底是哪台驱动器**，再决定隔离板输出档位（3.3V/5V）和共阳/共阴。这一条搞错，轻则不动，重则烧输出级。

---

## 3. 前置条件清单（缺一项就卡住）

### 3.1 下位机

- Keil MDK5 + 工程 `A-4-3-MCU-LCD/Project/Fire_F407ZG.uvprojx`
- DAP 调试器（本机 USB 枚举到 `VID_0483&PID_3748`，ST-Link/V2 兼容）
- 烧录：F8（Download）。烧完按复位，屏幕应出现 6 个按钮。
- **保留回滚**：`Project/Output/Fire_F407ZG.hex` 先复制一份，改坏了能刷回去。

### 3.2 上位机

```powershell
# 本机 Python 3.9 已装，缺 pyserial
& "C:\Users\ASUS\AppData\Local\Programs\Python\Python39\python.exe" -m pip install pyserial matplotlib
```

### 3.3 OpenHaptics SDK（读主手的前提）

- 本机**没装 SDK**，只有驱动。`--master touch` 会直接失败。
- 装 OpenHaptics 3.4（3D Systems 下载页），装完确认：
  - 环境变量 `OH_SDK_BASE` 指向安装目录
  - `hd.dll` 在 `%OH_SDK_BASE%\bin` 或 System32
  - `hd.lib`、`HD/hd.h` 存在（C++ 路线用）
- 只跑 Python 桥接的话，至少要让 `hd.dll` 能被 `ctypes.WinDLL("hd.dll")` 找到（把 SDK 的 bin 加进 PATH）。

### 3.4 串口识别（最容易踩的坑）

设备管理器里会同时出现两个 COM：

| 设备 | VID/PID | 用途 |
|---|---|---|
| Geomagic Touch Virtual COM Port | `VID_2988&PID_0302` | **主手**，给 OpenHaptics 用，别用串口助手打开 |
| USB-SERIAL CH340 | `VID_1A86&PID_7523` | **STM32 板**，桥接用的就是它 |

插拔时看"哪个消失了"来区分；或者在 PowerShell 里查注册表：

```powershell
Get-ChildItem 'HKLM:\SYSTEM\CurrentControlSet\Enum\USB' |
  Where-Object { $_.PSChildName -match 'VID_2988|VID_1A86' } |
  Select-Object PSChildName
```

---

## 4. 分级联调 L0–L7（每级都有通过标准，不通过不许进下一级）

### L0 只读主手（不接板子）

目的：确认 Touch 能被读到，且坐标系方向与预期一致。

```powershell
# 装好 SDK 后，用交付说明里的探针（工作区没有，需向同事要 check_device.py）
python check_device.py
```

通过标准：打印 6DOF 位姿、按键位、`HD_INPUT_DOF=6`、`HD_OUTPUT_DOF=3`；手推笔尖，X 向右、Y 向上、Z 向自己为正（面对底座）。

失败：`hdInitDevice` 失败 → 先跑 `Touch_Diagnostic.exe`（见主手手册 §4）；设备名要和 "Geomagic Touch Setup" 里一字不差。

### L1 串口握手（不接电机电源）

目的：确认板子、CH340、波特率、指令格式。

```powershell
python motor_console.py --list          # 找到 CH340 的 COM
python motor_console.py --port COM5     # 交互
> PING
OK PONG
> STATUS
OK POS=0,0,0 RUN=000 SENS=0 HOME=0
```

通过标准：`PING` 往返 < 50 ms；`STATUS` 字段齐全。

失败：
- 无回复 → 板子里是原版固件（§0），或者 RX/TX 接反（PA9 是 TX，要接 CH340 的 RX）。
- 收到 `欢迎使用野火开发板` → 原版固件，正常现象。
- 收到 `x: 123 y: 456` 之类 → 打开了 Touch 的虚拟串口。

### L2 单轴点动（先空载！先空载！）

目的：确认方向、速度、有没有堵转。

1. **先用屏幕按键**走一遍：按"直线推进"，看机构往哪边走，记下来；按"直线回退"；再按"顺时针旋转""逆时针旋转"。这一步是后面判断串口方向对不对的基准。
2. 再上串口（同样是空载或脱开负载）：

```
M1 FWD 300        # 2 秒
M1 STOP
M1 REV 300
M1 STOP
M2 FWD 300
M2 STOP
```

通过标准：串口 `FWD` 的方向与屏幕"推进"一致；300 Hz 下声音平稳、不丢步。

失败：
- 方向反 → 改 `MOTOR1_DIR_INVERTED`（`User/main.c:41`）或调换 A+/A−；**不要**在桥接里用负号硬掰，否则回零和限位全反。
- 一启动就堵转/尖叫 → 速度太高。原版 M1 默认 **1600 Hz**（`User/main.c:37`），且**没有加减速斜坡**；带负载时先降到 300–400 Hz 试。
- 只嗡不转 → 细分/电流拨码、相序、24V 没上。

### L3 定位与标定（这一步产出 ppm）

目的：把"脉冲"和"毫米/度"对上。

```
ZERO
M1 MOVE 800 800      # 正转 800 脉冲，800 Hz
STATUS               # 看 POS=800
```

用卡尺/百分表量实际位移 Δ（mm）：

```
ppm = 800 / Δ        # 脉冲/毫米
```

重复 3 次（正反各一次），取平均；三次偏差 > 2% 说明有丢步或反向间隙，先解决机械再标。

> 交付说明里 `--ppm 80` 只是占位默认值。按 3200 脉冲/圈推算，80 脉冲/mm 对应 **40 mm/圈**的导程——普通滚珠丝杠是 5/10/20 mm，所以这个默认值几乎肯定不对，必须实测。

### L4 回零（只对 M1 有效）

```
HOME
OK HOME START
# 等
OK HOME DONE POS=0
```

通过标准：15 s 内停到光电处，位置归零；手动挡一下光电能立刻停。

失败：
- `ERR HOME TIMEOUT` → 光电没接/极性反/遮挡片没到位/M1 方向反（`HOME_DIR`）。
- 冲过光电才停 → 光电是 PNP 或上拉不对，见 §2.4。
- 注意：**只有 M1 装了光电**（PC7 一路），M2、M3 没有硬件限位，只能靠软限位和行程标定。
- 更要注意：**原版固件里根本没有读光电的代码**（`Sensor_GPIO_Config()` 是 Remote 版新增的），所以在当前板子上发 `HOME` 只会回 `ERR UNKNOWN`。光电要按 §2.4 接到 PC7（上拉、开集），并把极性写进 `SENSOR1_ACTIVE_LOW`，`HOME` 才有意义。

### L5 三轴 + 夹爪

```
M2 FWD 200 / M2 STOP
M3 CLAMP        # 65 × 400 脉冲，约 10.8 s
M3 OPEN
```

注意：`MOTOR3_TRAVEL_PULSES = 65*400 = 26000`（`User/main.c:39`），3200 脉冲/圈 → **8.1 圈**，2400 Hz 下约 **10.8 s**。而 `三电机接线说明.md` 写的是"90°，800 脉冲，约 2 秒"——**文档和代码不一致，以代码为准**。夹爪没有位置反馈、没有力反馈，靠"走到行程末端堵转"来夹紧，这是当前设计的最大隐患（见 §9）。

通过标准：夹爪能夹住工件且不打滑；连续 20 次 CLAMP/OPEN 后行程不漂。

### L6 桥接（仿真主手 + 真板子）

```powershell
python bridge.py --port COM5 --master sim --seconds 30 --plot track.png
```

通过标准：`最终误差 < 1 mm`，`最大误差 < 5 mm`（受速度限制），曲线无台阶、无单向漂移。

失败：
- 越走越偏 → 看 §6.3 与 §8 的"指令积压"。
- 报 `ERR DEADMAN` → 桥接循环卡住超过 1.5 s（matplotlib 画图、GC、串口阻塞都会），把 `--plot` 关掉再试。

### L7 真主手

```powershell
python bridge.py --port COM5 --master touch
```

**先读 §8.1**：当前 `bridge.py` 在 touch 模式下第一帧会按主手绝对坐标发一次大位移（`last_x, last_y = 0.0, 0.0` 那两行）。上真机前必须改掉，否则一按运行机台就窜出去几十毫米。

通过标准：手推慢速（< 10 mm/s）时从动跟随，松手停；按急停/关桥接，1.5 s 内停。

---

## 5. 标定清单（做完打勾，数据写进下表）

### 5.0 先分清：文档给了什么、没给什么

`三电机接线说明.md` 的"4. 程序参数"只写了**速度和耗时**，不是当量。它隐含的只有两个已知量：

| 量 | 来源 | 值 | 说明 |
|---|---|---|---|
| 电机每圈脉冲 | §3 驱动器设置：`细分 → 3200 pulse/rev` | **3200 脉冲/圈** | 1.8° 电机 16 细分；换细分则整表重算 |
| M2 每度脉冲（电机轴） | §4：`90° = 3200×90/360 = 800 脉冲` | **8.89 脉冲/°** | 只是**电机轴**；M2 若有同步带/蜗轮/减速机，输出轴要乘减速比 i |
| M1 每毫米脉冲 | **文档没有** | **未知** | 缺一个关键参数：丝杠导程 L。`ppm = 3200 / L`，只能实测 |
| M3 行程 | 代码 `MOTOR3_TRAVEL_PULSES` | 65×400 = **26000 脉冲**（≈8.1 圈） | 与同文档"90°/800 脉冲"矛盾，见 §0 的表 |

几个常见导程对照（3200 脉冲/圈）：

| 丝杠导程 L | 4 mm | 5 mm | 10 mm | 20 mm | 40 mm |
|---|---|---|---|---|---|
| ppm（脉冲/mm） | 800 | 640 | 320 | 160 | 80 |

> 桥接默认 `--ppm 80` 正对应 40 mm 导程，是很少见的规格——这就是为什么它几乎肯定要改。

### 5.1 实测表

| 项 | 方法 | 记录 | 通过标准 |
|---|---|---|---|
| M1 ppm | `M1 MOVE 800 800`，量 Δ | ____ 脉冲/mm（= 3200/L 校核） | 3 次偏差 < 2% |
| M1 行程 | 回零后 `M1 MOVE 大数` 到机械端 | ____ mm | 软限位留 5 mm 余量 |
| M1 最高可用速度 | 从 400 Hz 往上试到丢步 | ____ Hz | 取 70% 作上限 |
| M2 减速比 i | 数同步带齿/蜗轮齿，或转 10 圈量输出角 | ____ | — |
| M2 脉冲/度 | `M2 MOVE 800`，量角度 | ____ 脉冲/°（电机轴 8.89×i） | 3 次偏差 < 2% |
| M2 零位 | 目视/标记对中 | ____ | 上电位置可复现 |
| M3 夹持行程 | CLAMP 到打滑，数脉冲 | ____ 脉冲 | 留 10% 余量 |
| M3 夹持力 | 弹簧秤拉工件 | ____ N | 满足工艺，不压伤 |
| 光电触发点 | 手动挡片，看 SENS 翻转 | ____ mm | 重复性 < 0.2 mm |


---

## 6. 映射与协同策略（这是"主从"的核心，不是接线）

### 6.1 先算速度预算：决定比例系数 k 的上限

从动手到机台，链路里最硬的两个数：`Hz_max`（电机实际可用脉冲频率）和 `ppm`（脉冲/mm）。

```
机台最大线速度   v_slave_max = Hz_max / ppm                [mm/s]
主手最大手速     v_hand_max  = v_slave_max / k             [mm/s]
```

举例（M1 导程 5 mm，3200 脉冲/圈）：

```
ppm = 3200 / 5 = 640 脉冲/mm
Hz_max = 1600 Hz（原版默认）
v_slave_max = 1600 / 640 = 2.5 mm/s
k = 1 时，手速超过 2.5 mm/s 就跟不上
```

人手正常推拉是 **20–100 mm/s**，也就是说 **k 必须小到 0.03–0.1**，否则永远追不上。这不是调参问题，是物理上限。

**推论：任何"1:1 直接映射"的方案在这台机器上都不成立。** 必须做三件事之一：
1. 缩小 k（位移缩放，手感变"重"）；
2. 提高 `Hz_max`（换更高频的驱动/细分降一档，但要先确认不丢步）；
3. 改"增量位置"为"速度指令"（主手偏移量 → 从动速度，松手即停），这是遥操作的常规做法，也最适合本链路。

第 3 种最稳：主手偏移 > 死区才动，偏移越大速度越快，回中就停。它对指令延迟、丢步都不敏感。

### 6.2 三自由度映射（当前实现 vs 建议）

| 主手输入 | 机台自由度 | 现在的 bridge.py | 建议 v1 |
|---|---|---|---|
| X（左右） | d 直线推进（M1） | ✅ x→M1，增量 20 ms | 保留，k_d 按 §6.1 定 |
| Y（上下） | θ 平面旋转（M2） | ✅ y→M2，`--rot-ppm 100` | 保留，但 100 脉冲/mm ≈ 11.25°/mm 太大，改 10–20 脉冲/mm |
| Z（前后） | g 夹爪（M3） | ❌ 未接 | 用 Z + 死区 + 最小间隔，或直接按钮 B3/B4 → CLAMP/OPEN |
| 按钮 1 | 使能（dead-man） | ❌ 未接 | 按住才动，松开停 |
| 按钮 2 | 离合（clutch） | ❌ 未接 | 按下重设零点，松开继续 |

> `--rot-ppm 100` 的含义是"主手每 1 mm → M2 走 100 脉冲"，3200 脉冲/圈 → **11.25°/mm**。主手行程 ±60 mm 就是 ±675°，明显不合理。建议先按"主手 30 mm ↔ 机台 90°"定：`rot_ppm = 90/30 × 8.89 ≈ 27 脉冲/mm`。

### 6.3 增量指令为什么会漂（交付说明里踩过的坑，还会再踩）

当前架构是"每 20 ms 发一条 `M1 MOVE <增量脉冲>`"：

```
t=0   主手走了 2 mm → 发 M1 MOVE 1280 800   （需要 1.6 s 走完）
t=20ms 主手又走 2 mm → 发 M1 MOVE 1280 800  （板子累加，需要再 1.6 s）
...
```

板子会把同向增量累加（`pulses_remaining += count`），所以**不会丢**，但会**无限滞后**。滞后本身不可怕，可怕的是：
- 主手回退时，板子还在执行前面的正向余量，先继续往前冲；
- 你看 `STATUS` 的 POS 与"主手位置×ppm"对不上，误判成丢步/反向间隙，于是去调机械——方向就错了。

**判据：** 跟随误差 `e = cmd_pulses − actual_pulses`。稳态下 e 应该收敛到 0；如果 e 单调增大，就是速度预算不够（§6.1），不是机械问题。

**根治：** 固件加绝对位置指令（§7.3 `GOTO`），桥接每 20 ms 发"目标位置"而不是"增量"，板子自己算差值并按自己的速度上限走。这样滞后有界、可观测、可限幅。

### 6.4 建议的协同状态机（上位机侧）

```
IDLE ──(按钮1按下 & 标定有效)──► ENABLED
ENABLED ──(按钮2按下)──► CLUTCH（记录 p_m0, p_s0）
CLUTCH ──(按钮2松开)──► ENABLED（用新的 p_m0 重算，不产生跳变）
ENABLED ──(按钮1松开 / 串口断 / STATUS 超时)──► IDLE（发 STOP）
任意 ──(急停 / ERR)──► FAULT（锁存，人工复位）
```

要点：
- **离合必须在"松开按钮"的瞬间重设基准**，否则手一松再按，机台会追一个 Δ。
- `STATUS` 每 200 ms 一次，连续 3 次没回 → 判通信故障，本地停 + 报警，别等板子的 1.5 s 死区。

---

## 7. 固件扩展：把原版 LCD 工程改成 Remote 版（最小增量）

如果你拿不到 `-Remote` 源码，就按下面 6 步自己加。全部改动都在 `User/main.c`，不需要动 HAL 库。

### 7.1 加位置与斜坡字段

`Motor_t`（`User/main.c:66-76`）里加：

```c
volatile int32_t  position;      /* 带符号脉冲计数 */
volatile uint32_t speed_target;  /* 目标频率 Hz */
volatile uint32_t speed_now;     /* 当前频率 Hz */
volatile uint8_t  ramp;          /* 1 = 正在加减速 */
```

### 7.2 ISR 里累加位置、做斜坡

`Motor_Service()`（`User/main.c:444` 起）里，把 `motor->pulse_frequency_hz` 换成 `speed_now`：

```c
/* 20000 Hz/s ÷ 20000 tick/s = 每 tick 加 1 Hz，正好是交付说明里的斜坡 */
if (motor->speed_now < motor->speed_target)      motor->speed_now += 1U;
else if (motor->speed_now > motor->speed_target) motor->speed_now -= 1U;

motor->phase_accumulator += motor->speed_now;
if (motor->phase_accumulator >= MOTOR_TIMER_TICK_HZ) {
    motor->phase_accumulator -= MOTOR_TIMER_TICK_HZ;
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT, motor->step_pin, GPIO_PIN_SET);
    motor->step_high = 1U;
    motor->position += (motor->direction == MOTOR_DIR_CW) ? 1 : -1;  /* 带 DIR_INVERTED 修正 */
    ...
}
```

注意 `MOTOR1_DIR_INVERTED=1` 时，"CW"在物理上是反的，`position` 的符号要和最终机构方向一致——**在 L3 标定时用 `M1 MOVE 800` 的实际位移方向核对一次**。

### 7.3 加绝对定位与软限位

> `Motor_StartMove()` 是交付说明里 Remote 版新增的函数（原版没有），签名 `(uint8_t index, int32_t pulses, uint32_t hz)`，内部做同向累加。下面的 `AXIS_MIN_PULSES/AXIS_MAX_PULSES` 是标定数组，按 §5 填。

```c
static uint8_t Motor_Goto(uint8_t i, int32_t target, uint32_t hz)
{
    int32_t delta;
    if (target < AXIS_MIN_PULSES[i] || target > AXIS_MAX_PULSES[i]) return 0U;  /* 软限位 */
    delta = target - motors[i].position;
    if (delta == 0) return 1U;
    return Motor_StartMove(i, delta, hz);   /* 同向累加逻辑保留 */
}
```

`AXIS_MIN/MAX` 由 L3/L4 标定得出，M1 下限就是回零后的 0。

### 7.4 加串口服务（放在主循环，10 ms 节拍）

`while(1)` 里 `HAL_Delay(10)` 之前加：

```c
if (u1_get_rx_flag() != 0U) {
    Remote_Execute((char *)u1_get_rx_data());   /* strtok 解析，一行一条 */
    u1_clean_rx_data();
}
Remote_Service();   /* 看门狗、DONE 回包、回零状态机 */
```

协议按交付说明实现即可（`PING / STATUS / HOME / ZERO / M1|M2 MOVE|FWD|REV|STOP / M3 CLAMP|OPEN|STOP / STOP`），**再加两条**：

| 指令 | 回复 | 用途 |
|---|---|---|
| `M1 GOTO <pulses> [Hz]` | `OK M1 GOTO ...` / `ERR LIMIT` | 绝对定位，桥接首选 |
| `M1 LIMIT <min> <max>` | `OK LIMIT` | 现场标定后写软限位，掉电不丢（可存 Flash） |

### 7.5 回零状态机

```c
/* state 1: 以 400 Hz 朝 HOME_DIR 走，直到 Sensor1_Active() */
/* 触发：Motor_Stop(0); position = 0; state = 2; 回 OK HOME DONE POS=0 */
/* 超时 15 s：Motor_Stop(0); state = 0; 回 ERR HOME TIMEOUT */
```

### 7.6 模式互斥（屏幕 / 串口）

原版屏幕按键是"按住走、松开停"（`UI_ProcessTouch` + `Motor_Service` 里 300 ms 无触摸就停 M1/M2，`User/main.c:448`、`User/main.c:651`）。加串口后必须定义优先级：

- 建议：**串口模式下屏幕只读**（显示状态，按键无效），或屏幕任意按键 = 本地接管 + 串口模式退出。
- 不要让两边同时调 `Motor_StartContinuous`，否则方向会被来回改。

---

## 8. 上位机桥接改造（`bridge.py` 有 4 个必须修的问题）

### 8.1 真主手首帧大跳（**上真机前必改**）

```python
last_x, last_y = master.read()
last_x, last_y = 0.0, 0.0          # ← 这两行把基准清零了
```

`--master touch` 分支里 `mx = x - last_x`，而 `last_x=0`，于是第一帧 `mx = 主手绝对 X`（可能 20–30 mm），直接变成 `M1 MOVE 1600+` 脉冲。**开机就窜**。

改法：

```python
last_x, last_y = master.read()      # 以当前位置为基准，删掉清零那行
```

### 8.2 没有速度预算，指令无限积压

```python
n1 = int(round(dx * args.ppm))
```

`dx` 是 20 ms 内的主手位移，手快时 `n1` 可达数千脉冲，而板子只能用 800 Hz 慢慢走。加两层限幅：

```python
max_step = args.speed / LOOP_HZ          # 每 20 ms 允许的脉冲数
n1 = int(round(dx * args.ppm + run._res1))
n1 = max(-max_step, min(max_step, n1))
run._res1 = dx * args.ppm + run._res1 - n1   # 余量带到下一拍，别丢
```

### 8.3 没有闭环校验（只解析 M1 的 POS，且不回看）

`parse_pos` 只取 `values[0]`，M2/M3 从不校验；也不判断跟随误差。加：

```python
if abs(cmd_pulses - pos) > 5 * args.ppm:   # 5 mm
    board.send("STOP"); raise SystemExit("跟随误差超限，已停")
```

### 8.4 没有按钮、没有离合、没有 M3

`TouchMaster.read()` 只返回 `position[0], position[1]`。要读按钮得让 `touch_hd` 暴露 `HD_CURRENT_BUTTONS`，否则"松手停"只能靠桥接进程被杀（1.5 s 死区兜底）。这是**安全级缺口**，不是功能缺口。

---

## 9. 安全设计（这张表比任何代码都重要）

| 风险 | 后果 | 措施 | 怎么验证 |
|---|---|---|---|
| 桥接卡死/断线 | 机台继续跑 | 板端 1.5 s 死区 + 上位机 STATUS 超时 | 运动中拔 USB，量停止时间 |
| 主手漂移/手抖 | 机台乱走 | 按钮使能 + 死区（< 0.5 mm 不发指令） | 按住使能不动手，看 POS 不变 |
| 超行程 | 撞机械端 | 软限位（标定值）+ 硬限位块 | 故意发超程指令，应回 `ERR LIMIT` |
| 光电失效 | 回零撞死 | 回零超时 + 机械挡块 + 回零速度降到 200–400 Hz | 拔光电，看是否 `ERR HOME TIMEOUT` |
| 夹爪过力 | 压坏工件/烧电机 | 夹持脉冲上限 + 电流拨码 + 弹簧秤标定 | 连续夹 20 次，测力不上升 |
| 丢步 | 位置漂移 | 关键轴加编码器；没有就每次上电回零 | 100 次往返后回零，看偏差 |
| 24V 误接信号 | 烧 F407 | 光耦隔离 + 先量电平再接线 | 万用表量 PC7 对地 ≤ 3.3V |
| 急停 | — | **独立硬回路**切 24V，不经过 MCU | 按下即断电，与软件无关 |

三条铁律：
1. `ENA` 不是急停。竖轴/夹爪断使能会掉。
2. 光电不是安全器件，只是定位开关。
3. 步进是开环，`POS` 是"发出去的脉冲数"，不是"实际位置"。没有编码器就必须靠回零复位。

---

## 10. 验收测试表（交付/上线前逐条打勾）

| # | 测试 | 方法 | 通过标准 |
|---|---|---|---|
| 1 | 串口链路 | 100 次 `PING` | 100% 回 `OK PONG`，平均 RTT < 50 ms |
| 2 | 单轴重复定位 | `M1 MOVE 800` ×10，百分表 | 极差 < 0.1 mm |
| 3 | 高速不丢步 | 400→1600 Hz 逐档，各 2000 脉冲 | 回零后偏差 < 0.2 mm |
| 4 | 回零重复性 | `HOME` ×10 | 零点极差 < 0.2 mm |
| 5 | 死区 | 运动中杀桥接进程 | < 1.6 s 停，无过冲 |
| 6 | 跟随（仿真） | `--master sim --seconds 30 --plot` | 最终误差 < 1 mm，无单调漂移 |
| 7 | 跟随（真手） | 慢推 ±20 mm，0.2 Hz | 滞后 < 3 mm，松手停 |
| 8 | 夹爪 | CLAMP/OPEN ×20 | 行程不漂，工件不滑 |
| 9 | 软限位 | 发超程 `GOTO` | `ERR LIMIT`，不动 |
| 10 | 断电恢复 | 运行中断电再上电 | 必须重新 `HOME` 才能动 |

---

## 11. 待补物料 / 现场待确认

**必须拿到**
1. `A-4-3-MCU-LCD-Remote` 完整工程（或确认板内固件版本后，按 §7 自己加）。
2. `pc/bridge.py`、`pc/fake_board.py`、`pc/motor_console.py`、`串口协议.md`、`改动说明.md`。
3. `05_3dtouch主手\3dtouch-main\code\touchpy\touch_hd.py`（`bridge.py` 依赖它读主手）。

**现场确认**
4. 三根轴各是哪台驱动器（SRX02-E / DM542J / DM1-422C 混装？），隔离板输出档位与共阳/共阴。
5. 光电是 NPN 还是 PNP，棕色线现在接到哪（绝不能直连 PC7）。
6. M1 导程、M2 减速比、M3 夹爪螺距 —— 决定 ppm，缺一不可。
7. M2/M3 是否有第二路限位（目前代码只有 PC7 一路）。
8. 夹爪是否有扭矩限制或力矩传感器；没有就限制夹持脉冲并定期复标。
9. 机台急停是否是硬回路，能否同时切断三台驱动器 24V。

**已知文档/代码不一致（以代码为准，但要让同事确认）**
10. `三电机接线说明.md` 写 M3 "90°/800 脉冲/2 秒"，代码是 `65×400=26000` 脉冲、2400 Hz、约 10.8 s。
11. 交付说明写驱动器 SRX02-E ×3，照片里是 DM542J + DM1-422C + SRX02-E。
12. `bridge.py` 默认 `--ppm 80`、`--rot-ppm 100`，与 3200 脉冲/圈推算不符，必须实测替换。

---

## 附：最短路径（如果今天就要动起来）

```text
① 上电、屏幕按键点动三轴，确认方向与声音            （10 min）
② python motor_console.py --port COMx，PING/STATUS  （5 min，先确认版本）
③ 空载 M1 MOVE 800 800，量位移，算出 ppm            （20 min）
④ HOME，确认光电与回零                              （10 min）
⑤ bridge.py --master sim --seconds 30 --plot        （10 min，看误差）
⑥ 改 §8.1 首帧大跳 → --master touch，手速 < 5 mm/s  （30 min）
⑦ 按 §6.1 调 k，先把"跟得上"跑顺，再谈手感          （半天）
```

**别跳步**：②没过就调 ⑤，只会在"越走越偏"里浪费一整天。
