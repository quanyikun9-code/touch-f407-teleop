#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""serial_console.py — 最小串口助手，专门处理 CH340 的 DTR/RTS 复位坑。

板载 CH340 的 DTR/RTS 接在 MCU 的 NRST 上：任何按默认参数打开 COM6 的工具
（SSCOM、PuTTY、miniterm……）都会把板子按在复位里，屏幕不亮、一个字节都收不到。
本脚本打开后立刻把 DTR/RTS 拉低，再等 1.2 s 让板子启动。

用法：
    python pc/serial_console.py                        # COM6，交互式
    python pc/serial_console.py --port COM7
    python pc/serial_console.py --cmd PING --cmd SENSOR  # 发完就退，适合脚本/排查
    python pc/serial_console.py --raw                  # 不发命令，只看板子输出

交互式里直接敲命令回车发送；:q 退出；Ctrl+C 也退。
"""

import argparse
import sys
import time

import serial


def open_port(port, baud):
    ser = serial.Serial(port, baud, timeout=0.2)
    # 必须先做这两行，否则板子一直处于复位
    ser.dtr = False
    ser.rts = False
    time.sleep(1.2)
    ser.reset_input_buffer()
    return ser


def collect(ser, seconds):
    """把 seconds 内收到的行都打出来，返回是否收到东西。"""
    deadline = time.perf_counter() + seconds
    got = False
    while time.perf_counter() < deadline:
        line = ser.readline()
        if not line:
            continue
        got = True
        print("<- " + line.decode("ascii", "replace").rstrip("\r\n"))
    return got


def main():
    ap = argparse.ArgumentParser(description="最小串口助手（DTR/RTS 安全）")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cmd", action="append", default=[],
                    help="发送一条命令后退出，可重复")
    ap.add_argument("--raw", action="store_true", help="只看板子输出，不发送")
    args = ap.parse_args()

    try:
        ser = open_port(args.port, args.baud)
    except serial.SerialException as exc:
        print("打开 %s 失败：%s" % (args.port, exc), file=sys.stderr)
        print("端口被别的程序占着？（bridge.py、厂家的 Touch 程序、另一个终端）", file=sys.stderr)
        print("用 python -m serial.tools.list_ports -v 看当前有哪些口", file=sys.stderr)
        return 2

    print("%s @ %d 8N1   Ctrl+C 退出" % (args.port, args.baud))

    try:
        if args.raw:
            while True:
                collect(ser, 0.2)

        if args.cmd:
            for cmd in args.cmd:
                print("-> " + cmd)
                ser.write((cmd + "\r\n").encode("ascii"))
                if not collect(ser, 0.5):
                    print("<- (0.5 s 内没有回复)")
            return 0

        # 先看看开机横幅/残留输出
        collect(ser, 0.3)

        while True:
            try:
                text = input("> ").strip()
            except EOFError:
                break
            if text in (":q", "quit", "exit"):
                break
            if not text:
                continue
            ser.write((text + "\r\n").encode("ascii"))
            if not collect(ser, 0.5):
                print("<- (0.5 s 内没有回复)")
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
