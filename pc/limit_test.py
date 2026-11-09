#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""limit_test.py — 量 M1 从"初始位置"走到"限位（光电触发）"有多远。

流程（全自动，不用手按屏幕）：
    1. PING / SENSOR 自检：确认光电现在是通的（没被遮挡）
    2. ZERO：把当前位置定义为初始位置 0
    3. M1 以 --hz 朝 --dir 方向走
    4. 光电一被遮挡，立刻 M1 STOP，读 STATUS 记下 POS
    5. 默认停在限位不回退，由板子屏幕上的【回位】按钮退回去（--return 可自动回退）
    6. 打印脉冲数 / 圈数 / 角度（给了 --lead-mm 再折算毫米）

用法：
    python pc/limit_test.py                       # COM6，正向，1600 Hz
    python pc/limit_test.py --hz 800
    python pc/limit_test.py --dir rev
    python pc/limit_test.py --hz 800 --lead-mm 5
    python pc/limit_test.py --dry-run             # 只自检，不动电机
    python pc/limit_test.py --log captures/limit.csv

安全：
    * 光电不走固件看门狗，所以轮询循环里每 0.3 s 发一次 STATUS 喂狗，
      否则 1.5 s 后固件会把 M1 停掉（这是协议设计，不是 bug）；
    * 如果 M1 中途自己停了（屏幕被碰 / 看门狗 / 驱动器报警），脚本会认出来并
      打印 REMOTE 标志，不会傻等到超时；
    * 急停仍然要靠硬件回路，软件只负责发 STOP。
"""

import argparse
import csv
import os
import re
import sys
import time

import serial

POS_RE = re.compile(r"POS=(-?\d+),(-?\d+),(-?\d+)")
RUN_RE = re.compile(r"RUN=(\d)(\d)(\d)")
SENSOR_RE = re.compile(
    r"SENSOR=(\d+)\s+RAW=(\d+)\s+EDGES=(\d+)\s+MS=(\d+)\s+PERIOD=(\d+)")


class Board:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        # CH340 的 DTR/RTS 接在板子复位上
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(1.2)
        self.ser.reset_input_buffer()

    def _write(self, line):
        self.ser.write((line + "\r\n").encode("ascii"))

    def ask(self, line, wait=0.5, match=None):
        self.ser.reset_input_buffer()
        self._write(line)
        deadline = time.perf_counter() + wait
        while time.perf_counter() < deadline:
            reply = self.ser.readline().decode("ascii", "replace").strip()
            if not reply:
                continue
            if not (reply.startswith("OK") or reply.startswith("ERR")):
                continue
            if match is None or match.search(reply):
                return reply
        return ""

    def send(self, line):
        self._write(line)

    def sensor(self):
        # 20 ms 足够：固件主循环 10 ms 一拍
        match = SENSOR_RE.search(self.ask("SENSOR", wait=0.2, match=SENSOR_RE))
        if not match:
            return None
        state, raw, edges, ms, period = (int(v) for v in match.groups())
        return {"state": state, "raw": raw, "edges": edges,
                "ms": ms, "period_us": period}

    def status(self):
        reply = self.ask("STATUS", wait=0.3, match=POS_RE)
        pos = POS_RE.search(reply)
        run = RUN_RE.search(reply)
        if not pos or not run:
            return None
        return {"pos": tuple(int(v) for v in pos.groups()),
                "run": tuple(int(v) for v in run.groups()),
                "remote": "REMOTE=1" in reply,
                "raw": reply}

    def close(self):
        try:
            self.send("M1 STOP")
            self.ser.flush()
        finally:
            self.ser.close()


def stop_all(board):
    """多发几次 STOP：固件以前会丢命令，单发有一次不中的风险。"""
    try:
        for _ in range(3):
            board.send("M1 STOP")
        board.send("M2 STOP")
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="量 M1 初始位置到限位之间的距离")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--dir", choices=["fwd", "rev"], default="fwd",
                    help="朝哪个方向找限位，默认 fwd")
    ap.add_argument("--hz", type=int, default=1600,
                    help="去程速度 Hz，默认 1600（= 30 rpm）")
    ap.add_argument("--back-hz", type=int, default=1600,
                    help="回程速度 Hz，默认 1600（仅 --return 时用）")
    ap.add_argument("--return", dest="do_return", action="store_true",
                    help="测完后自动退回。默认不回退，交给板子屏幕上的【回位】按钮")
    ap.add_argument("--ppr", type=int, default=3200,
                    help="驱动器细分，默认 3200 脉冲/圈")
    ap.add_argument("--lead-mm", type=float, default=0.0,
                    help="丝杠导程 mm/圈，给了才折算毫米")
    ap.add_argument("--max-pulses", type=int, default=0,
                    help="走这么多脉冲还没触发就放弃，0 = 不设上限（默认）")
    ap.add_argument("--timeout", type=float, default=300.0,
                    help="去程最长时间 s，默认 300，只防跑飞；0 = 完全不限")
    ap.add_argument("--log", default="captures/limit.csv",
                    help="过程 CSV，默认 captures/limit.csv")
    ap.add_argument("--yes", action="store_true", help="跳过确认提示")
    ap.add_argument("--dry-run", action="store_true", help="只自检，不动电机")
    args = ap.parse_args()

    if args.hz <= 0 or args.back_hz <= 0:
        ap.error("--hz / --back-hz 必须大于 0")
    if args.timeout > 0:
        timeout = args.timeout
    elif args.max_pulses > 0:
        timeout = args.max_pulses / args.hz + 30.0
    else:
        timeout = 0.0

    try:
        board = Board(args.port)
    except serial.SerialException as exc:
        print("打开串口 %s 失败：%s" % (args.port, exc), file=sys.stderr)
        return 2

    log_file = None
    log_writer = None

    try:
        pong = board.ask("PING")
        if "PONG" not in pong:
            print("板子没回 PONG（收到 %r）" % pong, file=sys.stderr)
            return 2

        sta = board.status()
        sen = board.sensor()
        if sta is None or sen is None:
            print("固件不支持 STATUS/SENSOR，先烧入当前版本", file=sys.stderr)
            return 2

        print("板子: %s" % sta["raw"])
        print("光电: SENSOR=%d RAW=%d EDGES=%d" % (
            sen["state"], sen["raw"], sen["edges"]))

        if sen["state"] != 0:
            print("\n光电现在就是被遮挡的。先把 M1 挪开（让 SENSOR=0）再来测，",
                  file=sys.stderr)
            print("否则一出发就会判定到达限位。", file=sys.stderr)
            return 2

        if args.dry_run:
            print("\n[dry-run] 自检通过，未发任何运动指令。")
            print("[dry-run] 正式跑：python pc\\limit_test.py --dir %s --hz %d"
                  % (args.dir, args.hz))
            return 0

        if not args.yes:
            print("\n即将让 M1 朝 %s 方向以 %d Hz 走，直到光电被遮挡。"
                  % (args.dir.upper(), args.hz))
            if args.max_pulses > 0:
                print("   上限 %d 脉冲（%.1f 圈）/ %.0f s。"
                      % (args.max_pulses, args.max_pulses / float(args.ppr), timeout))
            elif timeout > 0:
                print("   不设脉冲上限，最长 %.0f s（只防跑飞，可用 --timeout 改）。"
                      % timeout)
            else:
                print("   **完全不设上限**：如果它停住不动却一直嗡嗡响，按 Ctrl+C。")
            print("确认机台运动范围内没人、急停在手边，然后回车（Ctrl+C 放弃）。")
            input()

        if args.log:
            log_path = os.path.abspath(args.log)
            parent = os.path.dirname(log_path)
            if parent and not os.path.isdir(parent):
                os.makedirs(parent)
            log_file = open(log_path, "w", newline="", encoding="utf-8")
            log_writer = csv.writer(log_file)
            log_writer.writerow(["t_s", "sensor", "pos", "run", "remote"])
            print("过程记录: %s" % log_path)

        # --- 1. 记下初始位置 ---------------------------------------------
        sta = None
        for _ in range(4):
            board.ask("ZERO")
            sta = board.status()
            if sta is not None and sta["pos"] == (0, 0, 0):
                break
        else:
            print("ZERO 没生效（POS=%s），检查串口链路"
                  % (sta["pos"] if sta else "无回复",), file=sys.stderr)
            return 2
        print("\n初始位置已清零。开始走 %s @ %d Hz ..." % (args.dir.upper(), args.hz))

        # --- 2. 出发 ------------------------------------------------------
        for _ in range(4):
            board.send("M1 %s %d" % (args.dir.upper(), args.hz))
            time.sleep(0.05)
            sta = board.status()
            if sta is not None and sta["run"][0] != 0:
                break
        else:
            print("M1 起不来（RUN=%s），检查驱动器使能 / 急停 / 24V"
                  % ("".join(str(v) for v in sta["run"]) if sta else "无回复",),
                  file=sys.stderr)
            return 2
        started = time.perf_counter()
        last_feed = 0.0
        last_report = 0.0
        polls = 0
        pos = 0
        motion_seen = False
        last_sen = {"state": 0}
        last_run = "?"
        last_remote = "?"
        trigger_state = None

        while True:
            now = time.perf_counter()
            elapsed = now - started
            if timeout > 0 and elapsed > timeout:
                stop_all(board)
                print("\n超时（%.0f s）仍未触发，已停 M1。" % timeout, file=sys.stderr)
                print("可能原因：方向反了、光电没装在行程里（或装在另一头）、"
                      "机台已经顶到机械限位。", file=sys.stderr)
                return 3

            # 先喂狗：STATUS 同时给出位置和运行状态。SENSOR 不喂狗，只发它的
            # 话 1.5 s 后固件会自己把 M1 停掉。
            if now - last_feed >= 0.3:
                last_feed = now
                sta = board.status()
                if sta is not None:
                    pos = sta["pos"][0]
                    last_run = "".join(str(v) for v in sta["run"])
                    last_remote = 1 if sta["remote"] else 0
                    if sta["run"][0] != 0:
                        motion_seen = True
                    elif motion_seen and last_sen["state"] == 0:
                        stop_all(board)
                        print("\nM1 自己停了，不是本程序停的，光电也还没被遮挡。",
                              file=sys.stderr)
                        print("  POS=%d  RUN=%s  REMOTE=%d"
                              % (pos, "".join(str(v) for v in sta["run"]),
                                 1 if sta["remote"] else 0), file=sys.stderr)
                        print("  REMOTE=0 说明控制权交回了板子：屏幕被碰了一下"
                              "（或触摸芯片漂移），或者 1.5 s 看门狗超时。",
                              file=sys.stderr)
                        print("  排查：a) 运行时别碰屏幕；b) 复现就加 --log 再看。",
                              file=sys.stderr)
                        return 3
                    if args.max_pulses > 0 and abs(pos) >= args.max_pulses:
                        stop_all(board)
                        print("\n已走 %d 脉冲仍未触发，超过 --max-pulses，已停。"
                              % pos, file=sys.stderr)
                        return 3

            sen = board.sensor()
            polls += 1
            if sen is not None:
                last_sen = sen
                if sen["state"] != 0:
                    trigger_state = sen
                    break

            if log_writer:
                log_writer.writerow(["%.3f" % elapsed, last_sen["state"], pos,
                                     last_run, last_remote])

            if now - last_report >= 1.0:
                last_report = now
                print("  %6.1fs  POS=%8d  光电=%d" % (elapsed, pos, last_sen["state"]))

        forward_time = time.perf_counter() - started

        # --- 3. 停住并读确切位置 ------------------------------------------
        stop_all(board)
        time.sleep(0.05)
        sta = board.status()
        if sta is None:
            print("停止后读不到 STATUS", file=sys.stderr)
            return 2

        position = sta["pos"][0]
        pulses = abs(position)
        avg_poll_ms = forward_time * 1000.0 / max(1, polls)

        print("光电已触发（EDGES=%d），M1 停在 POS=%d（用时 %.2f s）"
              % (trigger_state["edges"], position, forward_time))

        if pulses == 0:
            print("触发得太快，脉冲数为 0：光电可能就压在起点上。", file=sys.stderr)
            return 3

        # --- 4. 原路退回（默认不做，交给板子上的【回位】按钮） -------------
        back_time = None
        final_pos = None
        back_sen = None

        if args.do_return:
            print("原路退回 %d 脉冲 @ %d Hz ..." % (pulses, args.back_hz))
            sta = None
            for _ in range(4):
                board.send("M1 MOVE %d %d" % (-position, args.back_hz))
                time.sleep(0.05)
                sta = board.status()
                if sta is not None and (sta["run"][0] != 0 or sta["pos"][0] == 0):
                    break

            back_started = time.perf_counter()
            back_deadline = back_started + pulses / args.back_hz + 10.0
            while time.perf_counter() < back_deadline:
                time.sleep(0.05)
                sta = board.status()
                if sta is None:
                    continue
                if sta["run"][0] == 0:
                    break
            else:
                stop_all(board)
                print("\n回程超时，已停 M1。POS=%s"
                      % (sta["pos"] if sta else "读不到",), file=sys.stderr)
                return 3

            back_time = time.perf_counter() - back_started
            sta = board.status()
            final_pos = sta["pos"][0] if sta else None
            back_sen = board.sensor()

        # --- 5. 结果 ------------------------------------------------------
        revs = pulses / float(args.ppr)
        print("\n" + "=" * 52)
        print(" 测量结果")
        print("=" * 52)
        print(" 方向            : %s @ %d Hz" % (args.dir.upper(), args.hz))
        print(" 去程用时        : %.2f s" % forward_time)
        print(" 初始位置 → 限位  : %d 脉冲" % pulses)
        print("                 : %.3f 圈" % revs)
        print("                 : %.1f 度" % (revs * 360.0))
        if args.lead_mm > 0:
            print(" 导程 %.3g mm/圈   : %.2f mm" % (args.lead_mm, revs * args.lead_mm))
        if args.do_return:
            print(" 回程            : %d 脉冲 @ %d Hz，%.2f s" % (
                pulses, args.back_hz, back_time))
            print(" 回程后 POS      : %s" % (final_pos,))
            if final_pos not in (None, 0):
                print("  [!] 回程没回到 0，差 %d 脉冲：MOVE 被丢或中途丢步" % final_pos,
                      file=sys.stderr)
            if back_sen is not None:
                print(" 回程后光电      : SENSOR=%d %s" % (
                    back_sen["state"],
                    "（已离开限位，正常）" if back_sen["state"] == 0
                    else "（仍被遮挡！可能丢步或回程不够）"))
        else:
            print(" 回程            : 未执行（交给板子上的【回位】按钮）")
            print(" 提示            : 光电计数刚 +1，此时屏幕右上角的【回位】应为黄色；")
            print("                   按一下它就会退回初始位置，退完自动变灰。")
        print("-" * 52)
        overshoot = max(1, int(args.hz * avg_poll_ms / 1000.0))
        print(" 轮询间隔 %.1f ms → 触发判定延迟约 %d 脉冲（%.4f 圈）"
              % (avg_poll_ms, overshoot, overshoot / float(args.ppr)))
        print(" 真实距离约比读数小 %d 脉冲。" % overshoot)
        if args.log:
            print(" 过程已记录: %s" % os.path.abspath(args.log))
        print("=" * 52)
        return 0

    except KeyboardInterrupt:
        print("\n中断，停 M1")
        stop_all(board)
        return 130
    finally:
        if log_file:
            log_file.close()
        board.close()


if __name__ == "__main__":
    sys.exit(main())
