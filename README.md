# 采集数据 / 上位机脚本

本目录是主从调试的上位机侧，配套固件仓库：
**https://github.com/quanyikun9-code/STM32F407-**（串口协议见该仓库 `docs/serial_protocol.md`）。

## 目录

| 文件 | 作用 |
|---|---|
| `pc/bridge.py` | 主手 → F407 桥接。按钮 1 只控 M1（主手 Z），按钮 2 只控 M2（绕 Y 转角），M3 只由板子屏幕控制；默认干跑，`--live` 才发指令 |
| `pc/analyze_master.py` | 采集 CSV 体检：速率、噪声、跳变、按键、量程 |
| `pc/touch_probe.cpp` | C++ 只读采集器（备用，需要 MSVC + OpenHaptics SDK） |
| `pc/README.md` | 环境、采集、桥接、踩坑的速查 |
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
