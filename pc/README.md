# pc/ — 主端（Geomagic Touch）数据采集与控制准备

## 已经装好的东西（2026-11-05）

| 项 | 状态 |
|---|---|
| `tools/3dtouch` | 已 clone（hopezzzz/3dtouch），Python 接口 + 安装脚本 |
| Touch 驱动 | 已装 **2025.12.12.644**，`C:\Program Files\3D Systems\Phantom Device Drivers\hd.dll` |
| OpenHaptics SDK | 已装 **3.5.0** 到 `C:\OpenHaptics\Developer\3.5.0`，`OH_SDK_BASE` 已设 |
| INF 注册 | `gtouchusb.inf` / `touchusb.inf` / `touchxusb.inf` 均已进 Windows 驱动库 |
| Python 绑定 | 3.14 x64 下已验证：`hd.dll` 能加载，常量正确（`HD_CURRENT_POSITION=0x2050`） |
| pyserial | 已装 3.5（给下位机串口用） |

Python 读数据**不需要 SDK**，驱动里的 `hd.dll` 就够了；SDK 是给 C++/官方演示用的，装上也留着。

## 目录

```
tools/3dtouch/            上游仓库（勿改）
  code/touchpy/           Python HDAPI 封装（touch_hd.py）+ 诊断/校准/示例
  install/                下载与安装脚本
  vendor/installers/      驱动 exe、SDK zip（都校验过 SHA-256）
  OpenHaptics/            未使用（SDK 装在 C:\OpenHaptics）
pc/
  analyze_master.py       采集数据体检（两种 CSV 都认）
  touch_probe.cpp         C++ 备用采集器（要 MSVC + SDK，一般用不到）
```

## 三步拿到数据

在仓库根目录（本文件夹）执行。**先把 Touch 的 USB 和电源都接好**，再开新终端。

```bat
:: 1. 诊断：驱动 -> USB -> HDAPI
python tools\3dtouch\code\touchpy\check_device.py

:: 2. 校准（笔杆放进墨盒后执行；每次设备上电都要做）
python tools\3dtouch\code\touchpy\calibrate.py

:: 3. 采 30 秒，200 Hz，写 CSV（Ctrl+C 结束）
python tools\3dtouch\code\touchpy\demo_read.py --rate 200 --csv captures\touch_30s.csv
```

采集动作：**前 10 秒完全不动**（噪声基线）→ X/Y/Z 各慢推到底再回中 → 手腕转三轴 → 按两个按钮。

## 体检

```bat
python pc\analyze_master.py captures\touch_30s.csv --plot captures\touch_30s.png
```

合格线：速率偏差 <10%、σ < 0.1 mm、相邻跳变 < 5 mm（200 Hz）、按键有 0→1、量程约 X/Y ±80 mm、Z ±35 mm。
没有硬件时可先跑 `python pc\analyze_master.py --demo demo.csv` 验证工具链。

## 桥接：主手 → 机台（速度控制）

固件已加串口遥控层（`PING/STATUS/FWD/REV/STOP`，1.5 s 看门狗，屏幕触摸可本地接管），
烧录后 `PING` 应回 `OK PONG`。桥接脚本是 `pc/bridge.py`：

```bat
python pc\bridge.py                       :: 干跑，只打印要发的指令
python pc\bridge.py --live                :: 真的动，按住按钮 1 才动
python pc\bridge.py --live --hz1-per-mm 15
python pc\bridge.py --yaw gimbal2         :: 矩阵解算的 Y 轴角不灵时换万向节角
```

**互斥双模式（按住才动，松开即停）**：

| 按钮 | 采集 | 控制 | 速度 |
|---|---|---|---|
| 按钮 1 | 主手 Z 位移 | M1 直线 | `Hz = (|dz| - 2 mm) * 40`，最大 1600 |
| 按钮 2 | gimbal1 转角 | M2 旋转 | `Hz = (|dθ| - 1.5°) * 15`，最大 400 |
| 两个都按 | — | 全停 | — |

每个模式只发对应轴的指令，另一轴保持 `STOP`，所以：
推 Z 时手腕的小转动不会影响 M1；转腕时 Z 的机械耦合也不会影响 M1。
M3（夹爪）**不接主手**，只由板子屏幕按钮控制；桥接只发 `M1 STOP`/`M2 STOP`，
**从不发全局 `STOP`**，不会把屏幕正在动的 M3 停掉。

方向反了用 `--invert1/--no-invert1`、`--invert2`；速度用 `--hz1-per-mm`、`--hz2-per-deg` 调。

操作：**按钮 1 = 使能（按住才动，松开停）**，**按钮 2 = 离合（按住停，松开以当前姿态为新基准）**。

- 干跑不发电机指令；`--live` 才会发。
- 远端模式下屏幕任一按钮都会夺回本地控制；`STOP` 或 1.5 s 无命令也会交回。
- `POS` 是发出去的脉冲数（开环），没有回零/软限位，第一次动之前先确认方向与行程。

## 坑

- **CH340 的 DTR/RTS 接了板子的复位**：pyserial 一 `open()` 就把 NRST 拉低，板子被按在复位里——屏幕变黑、串口一个字节都没有。开串口后立刻 `s.dtr = False; s.rts = False`，等 1.5 s 让板子启动。这也是“按 RESET 没用、关掉脚本又好了”的原因。
- 板子实测开机横幅：`GT ID : 911` / `Max ponts num : 5`（LCD 触摸芯片 GT911 的初始化打印），收到它就说明 CH340→USART1→115200 链路正常。
- **设备一次只能一个程序用**：跑 demo_read 时关掉 Touch Diagnostic / 官方演示。
- **每次上电都要校准**，否则坐标和力方向都是错的。
- USB 报 `设备描述符请求失败`（Code 43）→ 先插电源适配器、换 USB 2.0 直连口、换线；这不是驱动问题。
- `demo_read.py --csv` 是**追加**写；重复采集前先删旧文件。
