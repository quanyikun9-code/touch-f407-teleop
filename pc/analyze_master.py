#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_master.py — 分析 touch_probe 采到的主手 CSV，判断这份数据能不能拿去做主从。

用法：
    python analyze_master.py touch_30s.csv
    python analyze_master.py touch_30s.csv --plot touch_30s.png
    python analyze_master.py --demo demo.csv        # 先造一份演示数据
    python analyze_master.py demo.csv               # 再分析（不接硬件也能验证工具链）

看什么：
    * 实测速率是否接近设定值（掉到一半以上，说明调度器/USB 有问题）
    * 静止 1 s 的噪声 std（> 0.1 mm 就要查标定/线缆/供电）
    * 相邻样本最大跳变（几 mm 的瞬跳 = 标定丢失或干扰，不能用于控制）
    * 按键能不能正常 0→1，墨水井状态
"""

import argparse
import math
import os
import random
import statistics
import sys

DEFAULT_COLS = ["t_ms", "x", "y", "z", "vx", "vy", "vz",
                "gx", "gy", "gz", "btn1", "btn2", "inkwell"]


def load_csv(path):
    cols = None
    data = {}
    with open(path, "r", encoding="utf-8-sig", errors="replace") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if cols is None:
                try:
                    float(parts[0])
                except ValueError:
                    cols = parts
                    data = {c: [] for c in cols}
                    continue
                cols = DEFAULT_COLS[:len(parts)]
                data = {c: [] for c in cols}
            if len(parts) != len(cols):
                raise ValueError("第 %d 行列数 %d，表头是 %d" % (lineno, len(parts), len(cols)))
            for c, p in zip(cols, parts):
                data[c].append(float(p))
    if not data or not (data.get("t_ms") or data.get("t")):
        raise ValueError("没读到数据行：%s" % path)
    return data


def normalize(data):
    """同时接受本项目 CSV（t_ms,x,...）和 3dtouch demo_read.py 的 CSV（t,x_mm,...）。"""
    def pick(*names):
        for n in names:
            v = data.get(n)
            if v:
                return v
        return None

    t_ms = pick("t_ms")
    if t_ms is None:
        t = pick("t")
        if t is None:
            raise ValueError("CSV 里既没有 t_ms 也没有 t 列")
        t_ms = [v * 1000.0 for v in t]

    out = {"t_ms": t_ms}
    for a, alt in (("x", "x_mm"), ("y", "y_mm"), ("z", "z_mm")):
        v = pick(a, alt)
        if v is None:
            raise ValueError("CSV 缺少 %s 列" % a)
        out[a] = v

    for v in ("vx", "vy", "vz"):
        out[v] = pick(v)

    btn1, btn2 = pick("btn1"), pick("btn2")
    if btn1 is None:
        raw = pick("buttons")          # demo_read.py: 位掩码
        if raw is not None:
            btn1 = [float(int(x) & 1) for x in raw]
            btn2 = [float(int(x) & 2) for x in raw]
    out["btn1"], out["btn2"] = btn1, btn2
    out["inkwell"] = pick("inkwell")
    return out


def col(data, name):
    return data.get(name, [])


def speed_series(data):
    n = len(data["t_ms"])
    out = [0.0] * n
    if data.get("vx"):
        for i in range(n):
            vx, vy, vz = data["vx"][i], data["vy"][i], data["vz"][i]
            out[i] = math.sqrt(vx * vx + vy * vy + vz * vz)
        return out
    for i in range(1, n):
        dt = max(1e-6, (data["t_ms"][i] - data["t_ms"][i - 1]) / 1000.0)
        d = [data[a][i] - data[a][i - 1] for a in ("x", "y", "z")]
        out[i] = math.sqrt(sum(v * v for v in d)) / dt
    out[0] = out[1] if n > 1 else 0.0
    return out


def noise_sigma(vals):
    """白噪声估计：位置二阶差分的方差 = 6*sigma^2，对匀速/慢加速运动不敏感。"""
    if len(vals) < 3:
        return 0.0
    d2 = [vals[i + 1] - 2.0 * vals[i] + vals[i - 1] for i in range(1, len(vals) - 1)]
    return statistics.pstdev(d2) / math.sqrt(6.0)


def max_jump(data):
    n = len(data["t_ms"])
    worst, worst_i = 0.0, 0
    for i in range(1, n):
        d = math.sqrt(sum((data[a][i] - data[a][i - 1]) ** 2 for a in ("x", "y", "z")))
        if d > worst:
            worst, worst_i = d, i
    return worst, worst_i


def button_edges(data, name):
    vals = data.get(name) or []
    edges = []
    for i in range(1, len(vals)):
        if vals[i] > 0.5 >= vals[i - 1]:
            edges.append(data["t_ms"][i] / 1000.0)
    return edges, (sum(1 for v in vals if v > 0.5) / len(vals) if vals else 0.0)


def stats(vals, lo, hi):
    part = vals[lo:hi]
    if not part:
        return (0.0, 0.0, 0.0, 0.0)
    return (min(part), max(part), statistics.fmean(part), statistics.pstdev(part))


def plot(data, speed, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [v / 1000.0 for v in data["t_ms"]]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for a in ("x", "y", "z"):
        axes[0].plot(t, data[a], label=a, linewidth=0.8)
    axes[0].set_ylabel("position (mm)")
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, speed, color="tab:red", linewidth=0.8)
    axes[1].set_ylabel("speed (mm/s)")
    axes[1].grid(alpha=0.3)

    if data.get("btn1"):
        axes[2].step(t, data["btn1"], label="btn1", where="post")
    if data.get("btn2"):
        axes[2].step(t, data["btn2"], label="btn2", where="post")
    if data.get("inkwell"):
        axes[2].step(t, data["inkwell"], label="inkwell", where="post", alpha=0.6)
    axes[2].set_ylabel("buttons")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print("图已保存: %s" % path)


def analyze(path, expect_hz, max_jump_mm, make_plot):
    data = normalize(load_csv(path))
    n = len(data["t_ms"])
    rate = (n - 1) / ((data["t_ms"][-1] - data["t_ms"][0]) / 1000.0) if n > 1 else 0.0
    duration = (data["t_ms"][-1] - data["t_ms"][0]) / 1000.0

    speed = speed_series(data)
    jump, jump_i = max_jump(data)

    print("=" * 72)
    print("文件      : %s" % os.path.abspath(path))
    print("样本/时长 : %d 行 / %.2f s" % (n, duration))
    print("实测速率  : %.1f Hz   (期望 %.0f Hz)" % (rate, expect_hz))
    print("-" * 72)
    print("位姿范围 (mm):")
    for a in ("x", "y", "z"):
        lo, hi, mean, std = stats(data[a], 0, n)
        print("  %s : [%8.2f, %8.2f]  均值 %7.2f  全程std %.3f" % (a, lo, hi, mean, std))
    print("-" * 72)
    for a in ("x", "y", "z"):
        print("  噪声估计 %s : sigma %.4f mm" % (a, noise_sigma(data[a])))
    print("-" * 72)
    print("最大相邻跳变: %.2f mm @ %.2f s" % (jump, data["t_ms"][jump_i] / 1000.0))
    if data.get("btn1"):
        e1, frac1 = button_edges(data, "btn1")
        print("btn1 按下次数: %d  (占空 %.0f%%)" % (len(e1), frac1 * 100))
        if e1:
            print("  首次按下 @ %.2f s" % e1[0])
    if data.get("btn2"):
        e2, frac2 = button_edges(data, "btn2")
        print("btn2 按下次数: %d  (占空 %.0f%%)" % (len(e2), frac2 * 100))
    if data.get("inkwell"):
        print("墨水井在井内占比: %.0f%%" % (100 * sum(1 for v in data["inkwell"] if v > 0.5) / n))

    print("-" * 72)
    bad = []
    if rate < expect_hz * 0.9:
        bad.append("实测速率 %.0f Hz 低于期望 %.0f Hz 的 90%%：调度器被阻塞或 USB 带宽不足" % (rate, expect_hz))
    if rate > expect_hz * 1.1:
        bad.append("实测速率 %.0f Hz 高于期望 %.0f Hz 10%%：时间戳/节拍有问题" % (rate, expect_hz))
    for a in ("x", "y", "z"):
        sigma = noise_sigma(data[a])
        if sigma > 0.1:
            bad.append("%s 轴噪声 sigma %.3f mm > 0.1 mm：先查标定、线缆、供电" % (a, sigma))
            break
    if jump > max_jump_mm:
        bad.append("存在 %.2f mm 的瞬跳（阈值 %.1f mm）：标定丢失或干扰，别拿去控制电机" % (jump, max_jump_mm))

    if bad:
        print("判定: 不建议直接用于控制")
        for b in bad:
            print("  [!] " + b)
    else:
        print("判定: 数据质量 OK，可以进入映射/下发阶段")
    print("=" * 72)

    if make_plot:
        plot(data, speed, make_plot)
    return 0


def make_demo(path, seconds=30.0, hz=200.0, seed=7):
    rnd = random.Random(seed)
    n = int(seconds * hz)
    lines = ["t_ms,x,y,z,vx,vy,vz,gx,gy,gz,btn1,btn2,inkwell"]
    for i in range(n):
        t = i / hz
        x = 25.0 * math.sin(2 * math.pi * 0.15 * t) + rnd.gauss(0, 0.02)
        y = 15.0 * math.sin(2 * math.pi * 0.10 * t + 1.0) + rnd.gauss(0, 0.02)
        z = 10.0 * math.sin(2 * math.pi * 0.20 * t) + rnd.gauss(0, 0.02)
        vx = 25.0 * 2 * math.pi * 0.15 * math.cos(2 * math.pi * 0.15 * t)
        vy = 15.0 * 2 * math.pi * 0.10 * math.cos(2 * math.pi * 0.10 * t + 1.0)
        vz = 10.0 * 2 * math.pi * 0.20 * math.cos(2 * math.pi * 0.20 * t)
        gx = 0.3 * math.sin(2 * math.pi * 0.1 * t)
        gy = 0.2 * math.sin(2 * math.pi * 0.12 * t)
        gz = 0.9 * math.sin(2 * math.pi * 0.08 * t)
        btn1 = 1 if 5.0 <= t < 25.0 else 0
        btn2 = 1 if 12.0 <= t < 12.5 else 0
        lines.append("%.3f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%d,%d,%d" % (
            i * 1000.0 / hz, x, y, z, vx, vy, vz, gx, gy, gz, btn1, btn2, 0))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")
    print("已生成演示数据: %s  (%d 行, %.0f Hz, %.0f s)" % (path, n, hz, seconds))


def main():
    ap = argparse.ArgumentParser(description="分析 Geomagic Touch 采集 CSV")
    ap.add_argument("csv", nargs="?", help="touch_probe 输出的 CSV")
    ap.add_argument("--demo", metavar="OUT", help="生成一份演示 CSV 后退出")
    ap.add_argument("--expect-hz", type=float, default=200.0, help="期望采样率，默认 200")
    ap.add_argument("--max-jump", type=float, default=5.0, help="允许的最大相邻跳变 mm，默认 5")
    ap.add_argument("--plot", metavar="PNG", help="把曲线画到 PNG")
    args = ap.parse_args()

    if args.demo:
        make_demo(args.demo)
        return 0
    if not args.csv:
        ap.print_help()
        return 2
    try:
        return analyze(args.csv, args.expect_hz, args.max_jump, args.plot)
    except (OSError, ValueError) as exc:
        print("[错误] %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
