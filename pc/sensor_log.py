#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sensor_log.py — 读 F407 上的光电传感器（PC7），打印并可选存 CSV。

固件把 PC7 配成上拉输入（NPN 集电极开路，遮挡时拉低），在 TIM6 的 20 kHz
中断里 1 kHz 采样、3 ms 去抖，用 `SENSOR` 命令一次给出全部信息：

    SENSOR -> OK SENSOR=1 RAW=0 EDGES=42 MS=12345 PERIOD=1000

用法：
    python pc/sensor_log.py                          # COM6, 20 Hz
    python pc/sensor_log.py --hz 50
    python pc/sensor_log.py --seconds 60
    python pc/sensor_log.py --csv captures/sensor.csv
    python pc/sensor_log.py --port COM7
    python pc/sensor_log.py --no-reset               # 不清零固件边沿计数

CSV 列：t_s,state,raw,edges,edge_delta,period_us
    state       1 = 光电被遮挡（有效）
    raw         引脚原始电平，和 state 相反说明极性接反
    edges       固件累计的"进入遮挡"次数
    edge_delta  相对上一行新增的边沿数（两次轮询之间可能 >1）
    period_us   与上一沿的间隔，第一次为 0

固件必须已烧入带 SENSOR 命令的版本；老固件只会回 ERR UNKNOWN。
"""

import argparse
import csv
import re
import sys
import time

import serial

SENSOR_RE = re.compile(
    r"SENSOR=(\d+)\s+RAW=(\d+)\s+EDGES=(\d+)\s+MS=(\d+)\s+PERIOD=(\d+)")


class Link:
    def __init__(self, port, timeout=0.05):
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        # CH340 的 DTR/RTS 接在板子复位上，打开串口必须立刻放开
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(1.2)
        self.ser.reset_input_buffer()

    def ask(self, line, tries=2, wait=0.3):
        """发一行、等一条 OK/ERR 回复。中途的开机横幅和空行都跳过。"""
        for _ in range(tries):
            self.ser.reset_input_buffer()
            self.ser.write((line + "\r\n").encode("ascii"))
            deadline = time.perf_counter() + wait
            while time.perf_counter() < deadline:
                reply = self.ser.readline().decode("ascii", "replace").strip()
                if not reply:
                    continue
                if reply.startswith("OK") or reply.startswith("ERR"):
                    return reply
        return ""

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="读 F407 PC7 光电传感器")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--hz", type=float, default=20.0, help="轮询频率，默认 20")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = 一直跑")
    ap.add_argument("--csv", default="", help="把每次轮询写进 CSV")
    ap.add_argument("--no-reset", action="store_true", help="不清零固件的边沿计数")
    args = ap.parse_args()

    try:
        link = Link(args.port)
    except serial.SerialException as exc:
        print("打开串口 %s 失败：%s" % (args.port, exc), file=sys.stderr)
        print("用 python -m serial.tools.list_ports -v 找 CH340 的端口号", file=sys.stderr)
        return 2

    writer = None
    log_file = None

    try:
        pong = link.ask("PING")
        if "PONG" not in pong:
            print("板子没回 PONG（收到 %r）：检查串口号、波特率、USB 线"
                  % pong, file=sys.stderr)
            return 2

        if not args.no_reset:
            reply = link.ask("SENSOR RESET")
            if "RESET" not in reply:
                print("固件不认 SENSOR 命令（收到 %r）：先烧入带光电支持的新固件"
                      % reply, file=sys.stderr)
                return 2

        probe = link.ask("SENSOR")
        if not SENSOR_RE.search(probe):
            print("SENSOR 回复解析失败：%r" % probe, file=sys.stderr)
            return 2

        if args.csv:
            log_file = open(args.csv, "w", newline="", encoding="utf-8")
            writer = csv.writer(log_file)
            writer.writerow(["t_s", "state", "raw", "edges", "edge_delta", "period_us"])

        print("串口 %s  @%.0f Hz   CSV: %s" % (
            args.port, args.hz, args.csv if args.csv else "不写"))
        print("Ctrl+C 结束。state=1 表示光电被遮挡（NPN 拉低）。\n")

        started = time.perf_counter()
        last_state = None
        last_edges = 0
        last_print = 0.0
        polls = 0
        bad = 0

        while True:
            now = time.perf_counter()
            elapsed = now - started
            if args.seconds > 0 and elapsed >= args.seconds:
                break

            reply = link.ask("SENSOR")
            match = SENSOR_RE.search(reply)
            polls += 1
            if not match:
                bad += 1
                time.sleep(max(0.0, 1.0 / args.hz - (time.perf_counter() - now)))
                continue

            state, raw, edges, _ms, period = (int(v) for v in match.groups())
            delta = edges - last_edges
            last_edges = edges

            if writer:
                writer.writerow(["%.4f" % elapsed, state, raw, edges, delta, period])

            changed = state != last_state
            if changed or (now - last_print >= 2.0):
                print("%8.3fs  %-6s raw=%d  edges=%6d (+%d)  period=%7.1f ms" % (
                    elapsed,
                    "遮挡" if state else "常开",
                    raw, edges, delta,
                    period / 1000.0 if period else 0.0))
                last_print = now
            last_state = state

            time.sleep(max(0.0, 1.0 / args.hz - (time.perf_counter() - now)))

        print("\n%d 次轮询，%d 次没拿到有效回复，共 %d 个遮挡边沿"
              % (polls, bad, last_edges))

    except KeyboardInterrupt:
        print("\n停止")
    finally:
        if log_file:
            log_file.close()
            print("CSV 已写: %s" % args.csv)
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
