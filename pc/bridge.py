#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bridge.py — Geomagic Touch -> F407（互斥双模式）。

模式（按住才动，松开即停）：
    按钮 1：只采集主手 Z 位移   -> 速度控制 M1（直线）
    按钮 2：只采集绕 Y 轴转角   -> 速度控制 M2（旋转）
    M3（夹爪）：不接主手，只由板子屏幕按钮控制。

每个模式下只发对应轴的指令，另一轴保持停止，所以推 Z 时手腕的小转动
不会影响 M1，转腕时 Z 的机械耦合也不会影响 M1。

默认干跑；--live 才发运动指令。

用法：
    python pc/bridge.py                     # 干跑
    python pc/bridge.py --live              # 真动
    python pc/bridge.py --live --hz1-per-mm 15
    python pc/bridge.py --live --yaw gimbal2
"""

import argparse
import csv
import math
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools" / "3dtouch" / "code" / "touchpy"))

try:
    from touch_hd import HDError, TouchDevice
except RuntimeError as exc:
    print("加载 hd.dll 失败：%s" % exc, file=sys.stderr)
    sys.exit(2)

import serial

POS_RE = re.compile(r"POS=(-?\d+),(-?\d+),(-?\d+)")

MODE_NONE, MODE_M1, MODE_M2 = 0, 1, 2
MODE_NAME = {MODE_NONE: "off", MODE_M1: "M1/Z", MODE_M2: "M2/Yaw"}


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def rel_yaw_deg(r_now, r0):
    """相对初始姿态、绕设备 Y 轴的转角（度），用于 --yaw matrix。"""
    if not r_now or not r0:
        return 0.0
    rel = [[sum(r_now[i][k] * r0[j][k] for k in range(3)) for j in range(3)]
           for i in range(3)]
    return math.degrees(math.atan2(rel[0][2] - rel[2][0],
                                   rel[0][0] + rel[2][2]))


def slew(cur, target, accel, dt):
    step = accel * dt
    if target > cur:
        return min(target, cur + step)
    if target < cur:
        return max(target, cur - step)
    return cur


def rate_hz(offset, deadband, gain, max_hz, sign):
    excess = abs(offset) - deadband
    if excess <= 0.0:
        return 0.0
    speed = min(excess * gain, max_hz)
    return sign * speed if offset >= 0.0 else -sign * speed


def axis_cmd(name, hz):
    hz = int(round(hz))
    if hz == 0:
        return "%s STOP" % name
    return "%s %s %d" % (name, "FWD" if hz > 0 else "REV", abs(hz))


class Board:
    def __init__(self, port, live):
        self.live = live
        self.ser = serial.Serial(port, 115200, timeout=0.04)
        # CH340 的 DTR/RTS 接在板子复位上，打开串口必须立刻放开
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(1.2)
        self.ser.reset_input_buffer()
        self.pos = (0, 0, 0)

    def query(self, line):
        self.drain()
        self.ser.write((line + "\r\n").encode("ascii"))
        try:
            return self.ser.readline().decode("ascii", "replace").strip()
        except (serial.SerialException, OSError):
            return ""

    def send(self, line):
        if not self.live:
            return
        self.ser.write((line + "\r\n").encode("ascii"))

    def drain(self):
        try:
            self.ser.read(self.ser.in_waiting or 0)
        except (serial.SerialException, OSError):
            pass

    def update_status(self):
        reply = self.query("STATUS")
        match = POS_RE.search(reply)
        if match:
            self.pos = tuple(int(v) for v in match.groups())
        return reply

    def stop_axes(self):
        # 只停 M1/M2，绝不发全局 STOP，避免把屏幕正在控制的 M3 也停掉
        if self.live:
            self.send("M1 STOP")
            self.send("M2 STOP")

    def close(self):
        try:
            self.stop_axes()
            if self.live:
                self.ser.flush()
        finally:
            try:
                self.ser.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(description="Touch -> F407（按钮1=M1/Z，按钮2=M2/Yaw，默认干跑）")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=0.0)
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--auto-enable", action="store_true",
                    help="干跑自检：1 秒后视为按住按钮 1")
    ap.add_argument("--axis1", choices=["x", "y", "z"], default="z")
    ap.add_argument("--yaw", choices=["matrix", "gimbal1", "gimbal2", "gimbal3"],
                    default="gimbal1")
    ap.add_argument("--invert1", action=argparse.BooleanOptionalAction, default=True,
                    help="M1 方向取反；本机实测默认取反")
    ap.add_argument("--invert2", action=argparse.BooleanOptionalAction, default=False)
    ap.add_argument("--hz1-per-mm", type=float, default=40.0)
    ap.add_argument("--hz2-per-deg", type=float, default=15.0)
    ap.add_argument("--deadband1", type=float, default=2.0, help="M1 死区 mm")
    ap.add_argument("--deadband2", type=float, default=1.5, help="M2 死区 deg")
    ap.add_argument("--max-hz1", type=int, default=1600)
    ap.add_argument("--max-hz2", type=int, default=400)
    ap.add_argument("--accel1", type=float, default=3000.0)
    ap.add_argument("--accel2", type=float, default=1200.0)
    ap.add_argument("--log", default="")
    args = ap.parse_args()

    if args.live and args.auto_enable:
        ap.error("--auto-enable 只能干跑用")

    sign1 = -1.0 if args.invert1 else 1.0
    sign2 = -1.0 if args.invert2 else 1.0
    axis1 = {"x": 0, "y": 1, "z": 2}[args.axis1]
    gimbal_index = None if args.yaw == "matrix" else int(args.yaw[-1]) - 1

    try:
        dev = TouchDevice().open()
    except (HDError, RuntimeError) as exc:
        print("打开主手失败：%s" % exc, file=sys.stderr)
        return 2

    try:
        board = Board(args.port, args.live)
    except serial.SerialException as exc:
        dev.close()
        print("打开串口 %s 失败：%s" % (args.port, exc), file=sys.stderr)
        return 2

    info = dev.info()
    state = dev.read()
    if dev.calibration_status() != "ok":
        print("警告：主手未标定，数据不可用", file=sys.stderr)

    writer = None
    log_file = None
    if args.log:
        log_file = open(args.log, "w", newline="", encoding="utf-8")
        writer = csv.writer(log_file)
        writer.writerow(["t", "mode", "offset", "hz", "cmd"])

    print("主手: %s  s/n %s   串口: %s   模式: %s" % (
        info.get("model"), info.get("serial_number"), args.port,
        "LIVE" if args.live else "DRY-RUN"))
    print("按钮1: M1 <- %s, %.0f Hz/mm   按钮2: M2 <- %s, %.1f Hz/deg" % (
        args.axis1, args.hz1_per_mm, args.yaw, args.hz2_per_deg))
    print("M3 只由板子屏幕控制，桥接不碰。按住按钮才动，松开即停。\n")

    mode = MODE_NONE
    p0 = 0.0
    g0 = 0.0
    r0 = ()
    v1 = v2 = 0.0
    last_status = 0.0
    last_print = 0.0
    last_cmd = ""
    started = time.perf_counter()
    last_tick = started

    board.update_status()

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - started
            dt = max(1e-3, now - last_tick)
            last_tick = now
            if args.seconds > 0 and elapsed >= args.seconds:
                break

            state = dev.read()
            b1 = state.button(1) or (args.auto_enable and elapsed > 1.0)
            b2 = state.button(2)

            if now - last_status > 0.5:
                board.update_status()
                last_status = now

            # 两个按钮都按 -> 全停；否则按住的按钮决定唯一模式
            if b1 and b2:
                want = MODE_NONE
            elif b1:
                want = MODE_M1
            elif b2:
                want = MODE_M2
            else:
                want = MODE_NONE

            if want != mode:
                v1 = v2 = 0.0
                board.stop_axes()
                if want == MODE_M1:
                    p0 = state.position[axis1]
                elif want == MODE_M2:
                    g0 = state.gimbal_angles[gimbal_index] if gimbal_index is not None else 0.0
                    r0 = state.orientation
                mode = want

            offset = 0.0
            hz = 0.0

            if mode == MODE_M1:
                offset = state.position[axis1] - p0
                target = rate_hz(offset, args.deadband1, args.hz1_per_mm, args.max_hz1, sign1)
                v1 = slew(v1, target, args.accel1, dt)
                v2 = 0.0
                hz = v1
                last_cmd = axis_cmd("M1", v1) + " | M2 STOP"
                board.send(axis_cmd("M1", v1))
                board.send("M2 STOP")

            elif mode == MODE_M2:
                if gimbal_index is None:
                    offset = rel_yaw_deg(state.orientation, r0)
                else:
                    offset = math.degrees(state.gimbal_angles[gimbal_index] - g0)
                    offset = (offset + 180.0) % 360.0 - 180.0
                target = rate_hz(offset, args.deadband2, args.hz2_per_deg, args.max_hz2, sign2)
                v2 = slew(v2, target, args.accel2, dt)
                v1 = 0.0
                hz = v2
                last_cmd = "M1 STOP | " + axis_cmd("M2", v2)
                board.send("M1 STOP")
                board.send(axis_cmd("M2", v2))

            board.drain()

            if writer:
                writer.writerow(["%.4f" % elapsed, MODE_NAME[mode],
                                 "%.3f" % offset, "%.0f" % hz, last_cmd])

            if now - last_print >= 0.5:
                print("%6.1fs %-8s offset=%+8.3f  hz=%+6.0f  %s" % (
                    elapsed, MODE_NAME[mode], offset, hz,
                    last_cmd if mode != MODE_NONE else ""))
                last_print = now

            sleep = (1.0 / max(1.0, args.rate)) - (time.perf_counter() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n停止")
    finally:
        try:
            board.stop_axes()
        finally:
            board.close()
            dev.close()
            if log_file:
                log_file.close()
                print("CSV 已写: %s" % args.log)
    return 0


if __name__ == "__main__":
    sys.exit(main())
