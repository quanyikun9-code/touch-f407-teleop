# 采集数据 / 上位机脚本 / 固件

本目录是主从调试的上位机侧 + 固件快照。
固件的原始仓库仍保留在 **https://github.com/quanyikun9-code/STM32F407-**，
本仓库里的 `firmware/` 是**加了串口遥控层之后的完整工程快照**，可以直接用 Keil 打开
`firmware/Project/Fire_F407ZG.uvprojx` 编译。

## 目录

| 文件 | 作用 |
|---|---|
| `STARTUP.txt` | **每次启动的完整操作说明**（接线、检查、干跑、正式运行、故障判断），给第一次用的人 |
| `pc/bridge.py` | 主手 → F407 桥接。按钮 1 只控 M1（主手 Z），按钮 2 只控 M2（绕 Y 转角），M3 只由板子屏幕控制；默认干跑，`--live` 才发指令 |
| `pc/analyze_master.py` | 采集 CSV 体检：速率、噪声、跳变、按键、量程 |
| `pc/touch_probe.cpp` | C++ 只读采集器（备用，需要 MSVC + OpenHaptics SDK） |
| `pc/sensor_log.py` | 读 PC7 光电传感器：遮挡状态 + 边沿计数 + 周期，可存 CSV |
| `pc/serial_console.py` | 最小串口助手（自动处理 CH340 的 DTR/RTS 复位），交互式或 `--cmd` 单发 |
| `pc/limit_test.py` | M1 限位测距：走到光电触发 → 记脉冲 → 原路退回，报圈数/角度/毫米 |
| `pc/README.md` | 环境、采集、桥接、踩坑的速查 |
| `firmware/` | STM32F407 固件工程快照（串口遥控 + 上下两组触屏按键） |
| `firmware/docs/serial_protocol.md` | 串口协议（命令、看门狗、控制权） |
| `firmware/docs/wiring.md` | 三电机接线说明 |
| `主从协同实现指南_Touch到三轴机台.md` | 从接线到验收的完整落地指南 |

## 快速开始

```bat
:: 1) 主手工具链（touch_hd.py 等，按需 clone 到 tools/3dtouch）
git clone https://github.com/hopezzzz/3dtouch tools\3dtouch

:: 2) Python 依赖
python -m pip install -r requirements.txt

:: 3) 确认主手（先装 Touch 驱动；只用 Python 读数据不需要 SDK）
python tools\3dtouch\code\touchpy\check_device.py
python tools\3dtouch\code\touchpy\calibrate.py

:: 4) 干跑，确认手柄动作与指令
python pc\bridge.py

:: 5) 真机
python pc\bridge.py --live
```

默认串口 `COM6`（板载 CH340），可用 `--port` 改。

## 控制策略

| 按钮 | 采集 | 控制 |
|---|---|---|
| 按钮 1 | 主手 Z 位移 | M1 直线，速度 = `(|dz| - 2 mm) * 40 Hz/mm`，上限 1600 |
| 按钮 2 | 绕 Y 轴转角（gimbal1） | M2 旋转，速度 = `(|dθ| - 1.5°) * 15 Hz/deg`，上限 400 |
| 两个都按 | — | 全停 |

每次按下重新取基准，松开即停。M3（夹爪）不接主手，只由屏幕按钮控制。

## 双器械触屏

480×800 竖屏分上下两组：上半屏控制 M1/M2/M3，下半屏控制 M4/M5/M6。每组按键位置一致：直线推进/回退、顺/逆时针旋转、夹紧/松开，以及右上角的回位。直线和旋转按住才动；夹爪点按走固定行程，夹紧和松开交替。两组互不抢占，支持在触摸控制器的触点数以内并行操作；夹爪点按启动后无需继续按住。

新增电机的实际接线须写入 `firmware/User/second_instrument.h`。先核对 M4/M5/M6 的 STEP、DIR 引脚、方向和 M6 夹爪行程，再把 `SECOND_INSTRUMENT_ENABLED` 设为 `1`。不要复用触屏、LCD、串口、调试口或其他板载外设的引脚。未启用时，下半屏灰显，上半屏照常使用。M4 回位另需独立光电输入和实测的返程脉冲数，不能套用 M1 的数值。回位功能不是自动寻零，也不是硬件限位。

原单主手桥接 `pc/bridge.py` 只控制 M1/M2。双主手桥接 `pc/bridge_dual.py` 分别控制 M1/M2 和 M4/M5，M3/M6 仍由触屏控制。双主手启动与测试步骤见 [STARTUP.txt](STARTUP.txt)。

## 安全

- 固件有 **1.5 s 通信看门狗**：上位机不再发命令就停 M1/M2；
- `POS` 是发出去的脉冲计数，**开环**；没有回零、没有默认软限位；
- 第一次上电/改线后先空载低速、手放急停；
- 急停必须是切断驱动器动力电的独立回路，不能只靠软件。

## 第三方与授权

- `tools/3dtouch`（https://github.com/hopezzzz/3dtouch）不随本仓库分发，按需 clone。
  其 README 声明 `code/` 可自由使用；驱动和 OpenHaptics SDK 版权归 3D Systems，**不在本仓库内**。
- OpenHaptics Developer Edition 仅限科研/教学，商业分发需向 3D Systems 购买授权。
- 本仓库不包含任何厂商安装包或 PDF 文档。
