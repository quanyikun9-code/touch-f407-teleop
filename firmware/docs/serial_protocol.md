# 串口遥控协议（v1）

固件在原有“屏幕 6 键”基础上增加了 ASCII 串口遥控层，两者共用同一套电机函数。
串口参数：**115200 8N1**，一行一条命令，以 `\r\n` 或 `\n` 结束。

## 命令

| 命令 | 回复 | 说明 |
|---|---|---|
| `PING` | `OK PONG` | 链路探测，不抢占控制权 |
| `BEEP [ms]` | `OK BEEP 200` | 蜂鸣器响一下，测硬件用（默认 200 ms，1–5000 ms） |
| `STATUS` | `OK POS=x,y,z RUN=000 REMOTE=0` | 当前位置（脉冲）、运行位、远端标志 |
| `ZERO` | `OK ZERO` | 三轴命令计数清零 |
| `STOP` | `OK STOP` | 三轴全停并交回屏幕 |
| `LOCAL` | `OK LOCAL` | 交回屏幕控制 |
| `M1|M2 FWD [Hz]` | `OK M1 FWD 800` | 连续正转，频率可省略（M1 默认 4800，M2 默认 400） |
| `M1|M2 REV [Hz]` | `OK M1 REV 800` | 连续反转 |
| `M1|M2 MOVE <pulses> [Hz]` | `OK M1 MOVE 800 800` | 定量运动，正数 CW、负数 CCW |
| `M1|M2 GOTO <pos> [Hz]` | `OK M1 GOTO 1234` / `ERR LIMIT` | 绝对定位（按 `position` 算差值） |
| `M1|M2 STOP` | `OK M1 STOP` | 只停该轴，**不退出远端** |
| `M1|M2 LIMIT <min> <max>` | `OK M1 LIMIT -1000 10000` | 软限位，掉电丢失 |
| `M3 CLAMP` / `M3 OPEN` | `OK M3 CLAMP 26000` / `ERR M3 BUSY` | 65×45° 固定行程，夹紧/松开必须交替 |
| `M3 STOP` | `OK M3 STOP` | 停夹爪 |
| `SENSOR` | `OK SENSOR=1 RAW=0 EDGES=42 MS=12345 PERIOD=1000` | 光电传感器状态与边沿计数，不抢控制权 |
| `SENSOR RESET` | `OK SENSOR RESET` | 边沿计数清零，不改当前电平 |
| 其它 | `ERR UNKNOWN` / `ERR ARG` | |

## 控制权（重要）

- **只有运动命令**（`FWD/REV/MOVE/GOTO/CLAMP/OPEN`）会设置 `REMOTE=1`；
  `PING/STATUS/ZERO` 只刷新看门狗，不抢控制权；`SENSOR` **连看门狗都不刷新**，
  这样桥接挂掉后还有一个只管看光电的脚本时，M1/M2 仍会按 1.5 s 超时停下。
- `REMOTE=1` 时，屏幕的“无触摸 300 ms 自动停 M1/M2”不再生效，避免和上位机打架。
- **屏幕是本地接管**：远端运行时触摸屏幕任意按钮会立刻 `REMOTE=0`，交回屏幕。
- **看门狗 1.5 s**：远端模式下连续 1.5 s 没有任何命令 → 停 M1/M2 并 `REMOTE=0`。
- 单轴 `M1/M2 STOP` 不清除 `REMOTE`，所以一轴停止不会误停另一轴；
  要交回控制权请用 `STOP` 或 `LOCAL`。

## 光电传感器（PC7）

固件把 PC7 配成上拉输入，在 TIM6 的 20 kHz 中断里每 20 个 tick（1 kHz）采一次样，
连续 3 个采样一致才认这个电平——主循环刷屏时也不会漏掉边沿。

| 字段 | 含义 |
|---|---|
| `SENSOR` | 1 = 光电被遮挡（有效），0 = 常开 |
| `RAW` | 引脚原始电平（1 = 高）。它和 `SENSOR` 相反；若与实物相反，说明极性接反 |
| `EDGES` | 上电（或 `SENSOR RESET`）以来"进入遮挡"的次数 |
| `MS` | 最近一次边沿的系统时间（`HAL_GetTick()`，毫秒，上电起算） |
| `PERIOD` | 与上一沿的间隔（µs）；第一次边沿为 0 |

- 采样只在正常模式下跑：`ISOLATOR_STATIC_TEST=1` 或 `LCD_DIAGNOSTIC_MODE=1` 时
  TIM6 不启动，`SENSOR` 的数值不会刷新。
- 1 kHz 采样 + 3 ms 去抖意味着窄于约 3 ms 的脉冲会丢。要数更快的齿/缝，
  得改用 EXTI 计数或定时器输入捕获，别指望这条命令。
- 电气：NPN 集电极开路，遮挡时把线拉低；PC7 内部上拉约 40 kΩ，线长超过约 30 cm
  建议外部 4.7k–10k 上拉到 3.3 V。PC7 是 5 V 容忍脚，**不能接 24 V**。
- 遮挡提示：默认闪红灯 + 蜂鸣器断续响（`main.c` 顶部 `PHOTO_ALERT_MODE`：1 = 只闪灯，
  2 = 只响，3 = 两者，0 = 关；频率 `PHOTO_ALERT_HZ` 默认 4 Hz）。它和采样共用 1 kHz
  节拍，刷屏时不会卡顿；遮挡时顺手把常亮的绿灯关掉，所以看到的是"红/灭"而不是"红/绿"。
- PC 侧读法见 `pc/sensor_log.py`（轮询 + CSV）。

## 位置语义

`POS` 是**已发出的脉冲计数**（在 TIM6 20 kHz 中断里累加），不是编码器反馈。
步进开环，没有回零代码、默认软限位为 ±∞，上电后必须先 `ZERO` 或 `GOTO` 前自行确认行程。

## 示例

```
PING                    -> OK PONG
STATUS                  -> OK POS=0,0,0 RUN=000 REMOTE=0
M1 MOVE 200 400         -> OK M1 MOVE 200 400
M1 GOTO 0 400           -> OK M1 GOTO 0
M1 FWD 800              -> OK M1 FWD 800
M1 STOP                 -> OK M1 STOP
SENSOR                  -> OK SENSOR=0 RAW=1 EDGES=0 MS=0 PERIOD=0
STOP                    -> OK STOP
```

## 上位机注意

野火 F407 霸天虎板载 CH340 的 **DTR/RTS 接在 MCU 复位上**：串口一打开就会把板子按在复位里
（屏幕变黑、收不到任何数据）。Python 打开串口后必须立刻：

```python
s = serial.Serial("COM6", 115200, timeout=0.1)
s.dtr = False
s.rts = False
```

等约 1.2 s 再发命令。

## 接线

见 [wiring.md](wiring.md)：GPIOC PC0–PC5 接隔离板，驱动器 SRX02-E 共阳极、3200 脉冲/圈。
