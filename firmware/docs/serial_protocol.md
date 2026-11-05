# 串口遥控协议（v1）

固件在原有“屏幕 6 键”基础上增加了 ASCII 串口遥控层，两者共用同一套电机函数。
串口参数：**115200 8N1**，一行一条命令，以 `\r\n` 或 `\n` 结束。

## 命令

| 命令 | 回复 | 说明 |
|---|---|---|
| `PING` | `OK PONG` | 链路探测，不抢占控制权 |
| `STATUS` | `OK POS=x,y,z RUN=000 REMOTE=0` | 当前位置（脉冲）、运行位、远端标志 |
| `ZERO` | `OK ZERO` | 三轴命令计数清零 |
| `STOP` | `OK STOP` | 三轴全停并交回屏幕 |
| `LOCAL` | `OK LOCAL` | 交回屏幕控制 |
| `M1|M2 FWD [Hz]` | `OK M1 FWD 800` | 连续正转，频率可省略（M1 默认 1600，M2 默认 400） |
| `M1|M2 REV [Hz]` | `OK M1 REV 800` | 连续反转 |
| `M1|M2 MOVE <pulses> [Hz]` | `OK M1 MOVE 800 800` | 定量运动，正数 CW、负数 CCW |
| `M1|M2 GOTO <pos> [Hz]` | `OK M1 GOTO 1234` / `ERR LIMIT` | 绝对定位（按 `position` 算差值） |
| `M1|M2 STOP` | `OK M1 STOP` | 只停该轴，**不退出远端** |
| `M1|M2 LIMIT <min> <max>` | `OK M1 LIMIT -1000 10000` | 软限位，掉电丢失 |
| `M3 CLAMP` / `M3 OPEN` | `OK M3 CLAMP 26000` / `ERR M3 BUSY` | 65×45° 固定行程，夹紧/松开必须交替 |
| `M3 STOP` | `OK M3 STOP` | 停夹爪 |
| 其它 | `ERR UNKNOWN` / `ERR ARG` | |

## 控制权（重要）

- **只有运动命令**（`FWD/REV/MOVE/GOTO/CLAMP/OPEN`）会设置 `REMOTE=1`；
  `PING/STATUS/ZERO` 只刷新看门狗，不抢控制权。
- `REMOTE=1` 时，屏幕的“无触摸 300 ms 自动停 M1/M2”不再生效，避免和上位机打架。
- **屏幕是本地接管**：远端运行时触摸屏幕任意按钮会立刻 `REMOTE=0`，交回屏幕。
- **看门狗 1.5 s**：远端模式下连续 1.5 s 没有任何命令 → 停 M1/M2 并 `REMOTE=0`。
- 单轴 `M1/M2 STOP` 不清除 `REMOTE`，所以一轴停止不会误停另一轴；
  要交回控制权请用 `STOP` 或 `LOCAL`。

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
