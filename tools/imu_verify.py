#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
IMU 装回后的验证与编码器联合标定（配合 Task3_YawPID 固件的串口命令行）。

为什么需要这个脚本：

  1. 陀螺自己的 yaw 在 STAT 里看不到。STAT 的 yaw/cont/rate 是「当前反馈源」的
     值：fb=ENC 时它们全是编码器量。陀螺 yaw 只能换算出来：

         gyro = edeg + dIE          (dIE = imu - enc，STAT 的 dIE 字段)

     本脚本所有姿态都用这个换算，避免把「编码器没动」误判成「陀螺没输出」。

  2. CPD（编码器每转盘度的计数）之前是按 13.6 估的，没标定过。开环短脉冲让
     转盘转一个角度，同一段机械转角既被编码器数成 Δedeg，又被陀螺积成 Δgyro：

         ratio  = Δedeg / Δgyro = cpd_true / cpd_fw
         cpd_new = cpd_fw * ratio

     顺带验掉符号：接反时 Δedeg 与 Δgyro 反号，ratio < 0。

  3. 每次脉冲都带角度预算，中途 COAST，且左右交替，不会单向累积撞限位。
     FWD/REV 默认 400pm（折合 487‰），200ms 就能转 60° 以上，所以这里只用
     DUTY=<pm> 的小占空比 + 毫秒级脉冲。

用法：
  python imu_verify.py pose                      # 打印一次姿态（含换算出的陀螺 yaw）
  python imu_verify.py pose --sec 10 --interval 1  # 连续观察，看漂移
  python imu_verify.py pulse --pm 250 --ms 120   # 单次脉冲，看实际转了多少
  python imu_verify.py cpd --n 4                 # 4 次交替脉冲 -> CPD 与符号结论
  python imu_verify.py cpd --n 6 --pm 300 --ms 160 --budget 25

写在最后的永远是 COAST（松手滑行），不是 BRK：
BRK 会让转盘抱死，看起来像「电机不响应」。
"""
import argparse
import csv
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tune_yaw import Link, stat_fields  # noqa: E402

DEFAULT_CPD = 13.6


def _f(d, k, default=float("nan")):
    try:
        return float(d.get(k, ""))
    except (TypeError, ValueError):
        return default


def read_cpd(link):
    """从 ENC 行里取固件当前的 cpd（STAT 里没有这个字段，别用 STAT 猜）"""
    for ln in link.command("ENC", 0.5):
        if "cpd=" in ln:
            for kv in ln.replace("|", " ").split():
                if kv.startswith("cpd="):
                    try:
                        return float(kv[4:])
                    except ValueError:
                        pass
    return DEFAULT_CPD


def read_pose(link):
    """读一条 STAT，返回编码器角、陀螺 yaw（换算值）和状态字段"""
    f = stat_fields(link)
    edeg = _f(f, "edeg")
    die = _f(f, "dIE")
    return {
        "edeg": edeg,                       # 编码器角（cpd 折算后）
        "die": die,                         # imu - enc
        "gyro": edeg + die,                 # 陀螺积分 yaw（与 fb 无关）
        "fb_yaw": _f(f, "yaw"),             # 反馈源显示角（±180）
        "fb_cont": _f(f, "cont"),           # 反馈源连续角（限位用它）
        "rate": _f(f, "rate"),
        "left": f.get("left", "?"),
        "fb": f.get("fb", "?"),
        "zt": f.get("zt", "?"),
        "raw": f,
    }


def fmt(v, nd=2):
    if v is None:
        return "n/a"
    try:
        if v != v:                      # NaN
            return "n/a"
    except TypeError:
        return "n/a"
    return ("%." + str(nd) + "f") % v


def pulse(link, pm, ms, budget_deg, poll=0.010):
    """发一个毫秒级脉冲；转够 budget_deg 立刻 COAST。

    返回 (实际转动角度, 是否被预算截断)。

    ⚠ 这里必须用 record_raw（非阻塞，先看 in_waiting）而不是 link.record()：
    pyserial 的 read(8192) 会一直等满 8192 字节或超时（50ms），高速转动时
    一次轮询就是 25~60 度，30 度的预算实测冲到过 122 度，很危险。
    record_raw 的返回值是 (t, vals)，vals[1] 才是 yaw（vals[0] 是目标）。
    """
    chunk = record_raw(link, 0.08)
    y0 = chunk[-1][1][1] if chunk else 0.0

    link.sp.reset_input_buffer()
    link.send("DUTY=%d" % pm)
    t0 = time.time()
    moved = 0.0
    cut = False
    while (time.time() - t0) * 1000.0 < ms:
        chunk = record_raw(link, poll)
        if chunk:
            moved = chunk[-1][1][1] - y0
            if abs(moved) >= budget_deg:
                cut = True
                break
    link.send("COAST")
    return moved, cut


def guard(link, pose, direction, limit_margin):
    """朝 direction 转之前先看还剩多少度，不够就拒绝"""
    if pose["fb_cont"] != pose["fb_cont"]:
        return True
    return abs(pose["fb_cont"]) < limit_margin


def cmd_pose(link, args):
    link.set_stream(True)
    print("%6s %9s %9s %9s %9s %6s %s" %
          ("t(s)", "edeg", "dIE", "gyro", "fb_cont", "zt", "left"))
    t0 = time.time()
    while True:
        p = read_pose(link)
        print("%6.1f %9s %9s %9s %9s %6s %s" %
              (time.time() - t0, fmt(p["edeg"], 3), fmt(p["die"], 3),
               fmt(p["gyro"], 3), fmt(p["fb_cont"], 3), p["zt"], p["left"]))
        if args.sec <= 0 or (time.time() - t0) >= args.sec:
            break
        time.sleep(max(0.0, args.interval - 0.5))


def cmd_pulse(link, args):
    link.set_stream(True)
    p0 = read_pose(link)
    print("前: edeg=%s gyro=%s fb=%s left=%s"
          % (fmt(p0["edeg"], 3), fmt(p0["gyro"], 3), p0["fb"], p0["left"]))
    moved, cut = pulse(link, args.pm, args.ms, args.budget)
    time.sleep(args.settle / 1000.0)
    p1 = read_pose(link)
    print("脉冲: pm=%d ms=%d 预算截断=%s 实际转 %.2f°"
          % (args.pm, args.ms, cut, moved))
    print("后: edeg=%s gyro=%s fb=%s left=%s"
          % (fmt(p1["edeg"], 3), fmt(p1["gyro"], 3), p1["fb"], p1["left"]))
    print("Δedeg=%s  Δgyro=%s  比值=%s"
          % (fmt(p1["edeg"] - p0["edeg"], 3), fmt(p1["gyro"] - p0["gyro"], 3),
             fmt((p1["edeg"] - p0["edeg"]) / (p1["gyro"] - p0["gyro"])
                 if abs(p1["gyro"] - p0["gyro"]) > 0.5 else float("nan"), 4)))


def record_raw(link, sec, t_base=0.0):
    """非阻塞地解析数据流，返回 [(t, [ch0, ch1, ch2...])]，保留所有通道。

    注意不能用 sp.read(N)：pyserial 的 read 会一直等满 N 字节或等到超时，
    100Hz 的小流上它一次就吃掉整个 timeout（50ms），脉冲窗口会被拉长到
    几百毫秒，时间戳全歪。这里改成先看 in_waiting 再读。
    t_base 是调用前已经过去的时间，用于拼出连续的时间轴。
    """
    t0 = time.time()
    data = []
    pending = b""
    while time.time() - t0 < sec:
        n = link.sp.in_waiting
        if n > 0:
            pending += link.sp.read(n)
        else:
            time.sleep(0.002)
        while b"\n" in pending:
            line, pending = pending.split(b"\n", 1)
            s = line.decode("utf-8", "replace").strip()
            if not s.startswith("yaw:"):
                continue
            try:
                vals = [float(x) for x in s[4:].split(",")]
            except ValueError:
                continue
            data.append((t_base + time.time() - t0, vals))
    return data


def cmd_trace(link, args):
    """时间过程采样：脉冲期间同时看陀螺角（数据流）与编码器角（STAT 前后对比）。

    FB=IMU 时数据流的 ch1 就是陀螺 yaw，于是 100Hz 能看到陀螺的时间过程；
    编码器角只能在脉冲前后各读一次 STAT。两者覆盖同一段机械转角，比值就是
    CPD 的修正系数，符号则直接暴露「编码器与陀螺反向」。
    """
    link.command("STREAM=ON", 0.3)
    link.command("FB=%s" % args.fb, 0.5)
    if args.raw:
        link.command("RAW=ON", 0.4)
    p0 = read_pose(link)
    print("起始: edeg=%s gyro=%s fb=%s raw=%s"
          % (fmt(p0["edeg"], 3), fmt(p0["gyro"], 3), p0["fb"], args.raw))

    link.sp.reset_input_buffer()
    t_start = time.time()
    data = []
    t_pulse = None
    t_coast = None
    t_pulse_at = args.delay
    t_coast_at = args.delay + args.ms / 1000.0
    while (time.time() - t_start) < args.sec:
        elapsed = time.time() - t_start
        # 用绝对时刻触发，别指望循环体耗时稳定
        if (t_pulse is None) and (elapsed >= t_pulse_at):
            link.send("DUTY=%d" % args.pm)
            t_pulse = time.time() - t_start
            print("--- t=%.3fs DUTY=%d 通电 ---" % (t_pulse, args.pm))
        if (t_coast is None) and (elapsed >= t_coast_at):
            link.send("COAST")
            t_coast = time.time() - t_start
            print("--- t=%.3fs COAST 断电（通了 %.0f ms）---"
                  % (t_coast, (t_coast - t_pulse) * 1000.0))
        data.extend(record_raw(link, 0.005, elapsed))
    if t_coast is None:
        link.send("COAST")

    print("%7s %10s %10s %10s" % ("t(s)", "tgt", "ch1", "rate"))
    last_print = -1e9
    for t, v in data:
        if (t - last_print) < (1.0 / max(0.1, args.print_hz)):
            continue
        last_print = t
        mark = "*" if (t_pulse is not None and t_coast is not None and t_pulse <= t <= t_coast) else " "
        print("%6.3f%s %10s %10s %10s"
              % (t, mark, fmt(v[0], 3), fmt(v[1], 3),
                 fmt(v[2], 2) if len(v) > 2 else "  -"))

    time.sleep(args.settle / 1000.0)
    p1 = read_pose(link)
    d_enc = p1["edeg"] - p0["edeg"]
    d_gyro_hw = p1["gyro"] - p0["gyro"]
    d_gyro_stream = (data[-1][1][1] - data[0][1][1]) if data else float("nan")
    print()
    print("编码器 Δedeg      = %s" % fmt(d_enc, 3))
    print("陀螺   Δgyro(STAT) = %s" % fmt(d_gyro_hw, 3))
    print("陀螺   Δch1(流)    = %s（±180 卷绕，仅当转动小于半圈才可比）"
          % fmt(d_gyro_stream, 3))
    cpd_fw = read_cpd(link)
    if abs(d_gyro_hw) > 1.0:
        ratio = d_enc / d_gyro_hw
        print("比值 Δedeg/Δgyro  = %.4f  ->  CPD 建议 %.2f（当前 %.2f）%s"
              % (ratio, abs(cpd_fw * ratio), cpd_fw,
                 "  ※ 符号相反，编码器与陀螺反向" if ratio < 0 else ""))

    if args.raw == 1:
        link.command("RAW=OFF", 0.3)
    link.command("FB=ENC", 0.4)


def cmd_cpd(link, args):
    link.set_stream(True)
    start = read_pose(link)
    dfw = read_cpd(link)
    if dfw != dfw or dfw <= 0.0:
        dfw = DEFAULT_CPD
    print("起始: edeg=%s gyro=%s fb=%s cpd_fw=%.3f limit±%s"
          % (fmt(start["edeg"], 3), fmt(start["gyro"], 3), start["fb"], dfw,
             start["raw"].get("left", "?")))

    # 第一次脉冲当探针，先确定「pm 为正」对应 yaw 往哪个方向走
    pol = 1
    rows = []
    for i in range(args.n):
        p0 = read_pose(link)
        direction = 1 if i % 2 == 0 else -1
        if not guard(link, p0, direction, args.guard):
            direction = -direction          # 快撞限位，这拍反向
        pm = direction * pol * args.pm

        moved, cut = pulse(link, pm, args.ms, args.budget)
        time.sleep(args.settle / 1000.0)
        p1 = read_pose(link)

        d_enc = p1["edeg"] - p0["edeg"]
        d_gyro = p1["gyro"] - p0["gyro"]
        if i == 0 and abs(moved) > 1.0:
            pol = 1 if moved > 0 else -1
            print("探针: DUTY=%+d -> yaw 变化 %+.2f°（极性 pol=%+d）" % (pm, moved, pol))
        ratio = (d_enc / d_gyro) if abs(d_gyro) > 0.5 else float("nan")
        rows.append((d_enc, d_gyro, ratio, cut))
        print("第 %d 次: pm=%+5d 截断=%-5s Δedeg=%+8s Δgyro=%+8s 比值=%s"
              % (i + 1, pm, cut, fmt(d_enc, 3), fmt(d_gyro, 3), fmt(ratio, 4)))

    good = [r for (_e, g, r, _c) in rows
            if abs(g) > 5.0 and r == r]
    print()
    if not good:
        print("结论: 有效样本不足（每次转动要 >5°），把 --ms 或 --pm 加大重测")
        return
    ratio = statistics.fmean(good)
    cpd_new = dfw * ratio
    print("有效样本 %d 个，平均比值 %.4f" % (len(good), ratio))
    if ratio < 0.0:
        print("结论: 编码器与陀螺**符号相反** —— 用 SIG=-1（或 CSIGN=-1）翻转编码器方向")
    print("结论: CPD 建议 %.2f（当前 %.3f，改 %.1f%%）"
          % (abs(cpd_new), dfw, (abs(cpd_new) / dfw - 1.0) * 100.0))
    print("      下发:  CPD=%.2f" % abs(cpd_new))
    print("      校验:  再跑一次 cpd，比值应接近 1.0000")

    end = read_pose(link)
    print("累计: Δedeg=%s Δgyro=%s（同一段机械转角，两者应相等）"
          % (fmt(end["edeg"] - start["edeg"], 3),
             fmt(end["gyro"] - start["gyro"], 3)))


AX_KEYS = ("ax", "ay", "az", "gx", "gy", "gzraw", "gzs", "temp")


def capture_lines(link, sec, prefix=""):
    """非阻塞收集串口行，返回 [(t, line)]。

    理由同 record_raw：不能用 sp.read(N)，pyserial 会一直等满 N 字节或超时，
    短窗口里时间戳会整段失真。
    """
    t0 = time.time()
    out = []
    pending = b""
    while time.time() - t0 < sec:
        n = link.sp.in_waiting
        if n > 0:
            pending += link.sp.read(n)
        else:
            time.sleep(0.002)
        while b"\n" in pending:
            line, pending = pending.split(b"\n", 1)
            s = line.decode("utf-8", "replace").strip()
            if s and (not prefix or s.startswith(prefix)):
                out.append((time.time() - t0, s))
    return out


def parse_imu_line(s):
    """把 'IMU ax=.. ay=..' 解析成 dict；字段不全或解析失败返回 None"""
    f = {}
    for kv in s.split()[1:]:
        if "=" not in kv:
            continue
        k, v = kv.split("=", 1)
        try:
            f[k] = float(v)
        except ValueError:
            return None
    for k in AX_KEYS:
        if k not in f:
            return None
    return f


def cmd_axes(link, args):
    """六轴静态检查：换安装位置/换模块之后先跑这个，别急着上闭环。

    三轴加速度就是重力在芯片坐标里的投影：哪根轴 |a| 接近 1g，哪根就是竖直轴；
    符号决定芯片面朝上还是朝下。这个结论直接决定 GYRO_SIGN 取 +1 还是 -1，
    以及 Z 轴是否真的还对准转盘轴——装歪了 yaw 速度会漏一部分到 gx/gy，
    积分出来的角度还会按 cos(倾角) 缩小。
    """
    link.command("STREAM=OFF", 0.4)
    link.command("IMU=OFF", 0.2)
    link.sp.reset_input_buffer()
    link.command("IMU=ON", 0.3)

    rows = []
    for _t, s in capture_lines(link, args.sec, "IMU "):
        f = parse_imu_line(s)
        if f is not None:
            rows.append(f)

    link.command("IMU=OFF", 0.2)
    link.command("STREAM=ON", 0.3)

    if len(rows) < 5:
        print("样本太少（%d 条）——IMU 没上线，或者 dump 没打开" % len(rows))
        return

    print("六轴静态采样 %.1f s，有效样本 %d 条" % (args.sec, len(rows)))
    print("%7s %10s %8s %10s %10s" % ("", "mean", "std", "min", "max"))
    mean = {}
    sd = {}
    for k in AX_KEYS:
        vs = [r[k] for r in rows]
        mean[k] = statistics.fmean(vs)
        sd[k] = statistics.pstdev(vs) if len(vs) > 1 else 0.0
        print("%7s %10.3f %8.3f %10.3f %10.3f"
              % (k, mean[k], sd[k], min(vs), max(vs)))

    ax, ay, az = mean["ax"], mean["ay"], mean["az"]
    tot = math.sqrt(ax * ax + ay * ay + az * az)
    axis, val = max((("X", ax), ("Y", ay), ("Z", az)), key=lambda kv: abs(kv[1]))
    tilt_z = math.degrees(math.acos(min(1.0, abs(az) / tot))) if tot > 1e-6 else float("nan")

    print()
    print("重力模长 |a| = %.3f g（正常 0.95~1.05，偏差大说明标度或数据有问题）" % tot)
    print("竖直轴 = %s（%.3f g） -> 芯片面朝%s"
          % (axis, val, "下（芯片 Z 轴朝下）" if val < 0 else "上（芯片 Z 轴朝上）"))
    print("芯片 Z 轴与竖直方向夹角 = %.2f 度（<5 度算装平）" % tilt_z)
    print("陀螺零偏: gx=%+8.3f gy=%+8.3f gzraw=%+8.3f dps" % (mean["gx"], mean["gy"], mean["gzraw"]))
    print("陀螺噪声: gx=%8.3f gy=%8.3f gzraw=%8.3f dps (std)"
          % (sd["gx"], sd["gy"], sd["gzraw"]))
    if abs(mean["gzraw"]) > 1e-6:
        print("符号: gzs/gzraw = %+.3f（固件当前 GYRO_SIGN）" % (mean["gzs"] / mean["gzraw"]))
    print("温度: %.1f C" % mean["temp"])

    print()
    if axis != "Z":
        print("警告: 竖直轴不是 Z —— 固件按 gz 积分，这个装法转盘转动几乎读不到，"
              "必须把模块摆正，或改固件取 %s 轴" % axis.lower())
    elif tilt_z > 5.0:
        print("警告: Z 轴与竖直方向差 %.1f 度，转盘转动会有 %.0f%% 漏到 gx/gy，"
              "积分角度按 cos 缩小" % (tilt_z, 100.0 * math.sin(math.radians(tilt_z))))
    else:
        print("结论: 竖直轴是 Z 且装得平，可以用 gz 积分")
    if abs(mean["gzraw"]) > 2.0:
        print("注意: gz 静态零偏 %.2f dps 偏大，静态漂移会明显（CAL 能扣掉）" % mean["gzraw"])
    if sd["gzraw"] > 0.5:
        print("注意: gz 噪声 %.2f dps 偏大，可能是 I2C 时序或供电不稳" % sd["gzraw"])
    print("下一步: 手转转盘 ~90 度，用 pose 前后对比 Δedeg / Δgyro 验符号与比例")


def cmd_hand(link, args):
    """手转核对：全程不用电机，纯靠人把转盘转一个角度。

    陀螺角和编码器角都是固件里的绝对积分，所以只要「两次读数之间净转过一次」，
    转动发生在窗口的哪一刻都不影响 Delta，不需要精确同步——这比打脉冲安全，
    也不会碰限位。符号和比例一次全验完：
        d_gyro / d_edeg ~ +1  -> 符号对、比例对
        d_gyro / d_edeg ~ -1  -> 符号反，GYRO_SIGN 取反
        |比值| 明显偏离 1     -> CPD 或陀螺量程（FS）有问题

    为什么不用固定窗口：上位机和人的反应之间有随机延时，固定 18 秒很容易
    「命令跑完了人还没开始转」。这里改成主动轮询编码器，真的动了、并且停稳了
    才收尾，操作者想什么时候转都行。
    """
    link.set_stream(True)
    p0 = read_pose(link)
    if p0["edeg"] != p0["edeg"]:
        print("读不到 STAT，先确认串口和固件")
        return
    print("起始: edeg=%s gyro=%s fb=%s left=%s"
          % (fmt(p0["edeg"], 3), fmt(p0["gyro"], 3), p0["fb"], p0["left"]))
    print(">>> 现在把转盘转约 %.0f 度，随时开始都行，转完停住不动即可 <<<" % args.deg)
    print(">>> 最多等 %.0f 秒 <<<" % args.timeout)

    e0 = p0["edeg"]
    prev = e0
    last_move = time.time()
    t_start = time.time()
    started = False
    while (time.time() - t_start) < args.timeout:
        f = {}
        for ln in link.command("STAT", 0.10):
            if ln.startswith("STAT "):
                for kv in ln.split()[1:]:
                    if "=" in kv:
                        k, v = kv.split("=", 1)
                        f[k] = v
        try:
            e = float(f.get("edeg", "nan"))
        except ValueError:
            continue
        if e != e:
            continue
        if not started:
            if abs(e - e0) >= args.min_deg:
                started = True
                prev = e
                last_move = time.time()
                print("--- 检测到转动：已转 %.1f 度，等你停稳 ---" % (e - e0))
            continue
        if abs(e - prev) > 0.3:
            prev = e
            last_move = time.time()
        elif (time.time() - last_move) >= args.settle:
            print("--- 停稳了（%.0f 度）---" % (e - e0))
            break

    if not started:
        print("结论: %.0f 秒内没检测到转动（编码器几乎没变）— 重跑一次"
              % args.timeout)
        return

    p1 = read_pose(link)
    d_enc = p1["edeg"] - p0["edeg"]
    d_gyro = p1["gyro"] - p0["gyro"]
    print("结束: edeg=%s gyro=%s left=%s"
          % (fmt(p1["edeg"], 3), fmt(p1["gyro"], 3), p1["left"]))
    print()
    print("d_edeg = %+8.3f   (编码器)" % d_enc)
    print("d_gyro = %+8.3f   (陀螺积分，已含 GYRO_SIGN)" % d_gyro)

    if abs(d_enc) < 5.0:
        print("结论: 编码器只动了 %.2f 度，太小 — 重跑，转够 30 度以上" % d_enc)
        return
    if abs(d_gyro) < 1.0:
        print("结论: 陀螺几乎没动但编码器动了 %.1f 度 — 陀螺没在积分或轴不对" % d_enc)
        return

    ratio = d_gyro / d_enc
    print("比值 d_gyro/d_edeg = %+.4f" % ratio)
    if abs(ratio) > 0.02:
        print("符号: %s" % ("一致（当前 GYRO_SIGN 方向正确）" if ratio > 0
                            else "相反（把 GYRO_SIGN 取反：1 改 -1，-1 改 1）"))
    scale_err = abs(abs(ratio) - 1.0) * 100.0
    print("比例误差: %.2f%%  %s"
          % (scale_err, "OK" if scale_err <= 5.0 else "偏大，查 CPD 与陀螺量程"))
    print("提示: 手转速度不均匀只会让瞬时读数抖动，不影响积分总量")


def cmd_rec(link, args):
    """纯录制：不改任何状态（不 ZERO / 不 SET / 不 START），只把数据流写盘。

    为什么需要它：测"人手拨开再松手"这类外部事件时，如果录制命令自己带
    ZERO，就会在松开前把当时的位置当成新的 0 —— 实测踩到过：人推了 40 度，
    step 的 ZERO 直接把那个位置取成 0，轨迹里只剩一条平线。
    所以流程必须是：先用别的命令把环路设好（SET/START 并确认 st=RUN），
    再单独启动本命令只录不改，最后才请人去拨。
    """
    link.set_stream(True)
    if args.raw:
        link.command("RAW=ON", 0.4)
    f = stat_fields(link)
    print("录制前状态: st=%s fb=%s tgt=%s yaw=%s out=%s duty=%s"
          % (f.get("st", "?"), f.get("fb", "?"), f.get("tgt", "?"),
             f.get("yaw", "?"), f.get("out", "?"), f.get("duty", "?")))
    if f.get("st") != "RUN":
        print("⚠ 当前不在 RUN，人手拨开不会有任何对抗 —— 先 SET + START")

    data = []
    pend = b""
    t0 = time.time()
    try:
        while (time.time() - t0) < args.sec:
            n = link.sp.in_waiting
            if n > 0:
                pend += link.sp.read(n)
            else:
                time.sleep(0.002)
            while b"\n" in pend:
                line, pend = pend.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if not s.startswith("yaw:"):
                    continue
                parts = s[4:].split(",")
                if len(parts) < 2:
                    continue
                try:
                    vals = [float(x) for x in parts]
                except ValueError:
                    continue
                rate = vals[2] if len(vals) > 2 else float("nan")
                data.append((time.time() - t0, vals[0], vals[1], rate))
    finally:
        if args.raw:
            link.command("RAW=OFF", 0.3)

    if len(data) < 20:
        print("数据不足 n=%d" % len(data))
        return
    ys = [d[2] for d in data]
    print("录制完成: n=%d  yaw min=%.2f max=%.2f" % (len(data), min(ys), max(ys)))

    name = args.out or ("rec_%s.csv" % time.strftime("%H%M%S"))
    path = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..",
        "docs", "tuning", name))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "target_deg", "yaw_deg", "rate_dps"])
        for t, g, y, r in data:
            w.writerow(["%.4f" % t, "%.3f" % g, "%.3f" % y,
                        "" if r != r else "%.2f" % r])
    print("  csv:", path)


def cmd_recover(link, args):
    """扰动恢复（任务书发挥部分第 2 项）：

        "转盘被外力拨开 30 度以上后，松手能无超调回到目标角度"

    人手推属于外力，自动化测试里用开环脉冲代替：同样是把转盘推到目标外，
    只是扰动波形不同（脉冲更陡）。流程：
        1) ZERO + SET=0，让目标就是当前位置；
        2) 开环 DUTY 脉冲把转盘推开 --disp 度（带角度预算，超了立刻 COAST）；
        3) 松手 -> START，闭环把转盘拉回 0，全程 100Hz 记录；
        4) 统计超调（往反方向越过 0 的最大幅度）、稳定时间、静差。

    判据按任务书：无超调（过冲 <= 0.5 度）、最终静差 <= 1.5 度。
    """
    link.command("STOP", 0.3)
    link.command("CLEAR", 0.3)
    link.command("FB=%s" % args.fb, 0.5)
    if args.stall_off:
        link.command("STALL=OFF", 0.3)
    link.command("ZERO", 0.4)
    link.command("LIMIT=%d" % args.limit, 0.4)
    link.command("PID=%g,%g,%g" % (args.kp, args.ki, args.kd), 0.5)
    link.command("SET=0", 0.4)
    link.set_stream(True)

    # 1) 开环推开
    d = 1.0 if args.dir >= 0 else -1.0
    moved, cut = pulse(link, int(d * args.pm), args.ms, args.disp)
    time.sleep(args.settle / 1000.0)
    p = read_pose(link)
    disp = p["fb_cont"]
    print("推开: 目标 %.0f 度  实际 %.2f 度  (预算截断=%s)" % (args.disp, disp, cut))
    if abs(disp) < args.disp * 0.8:
        print("结论: 推开不足 %.0f 度，重跑（加大 --pm 或 --ms）" % args.disp)
        return

    # 2) 松手，闭环拉回
    link.sp.reset_input_buffer()
    link.send("START")
    data = []
    pend = b""
    t0 = time.time()
    try:
        while (time.time() - t0) < args.sec:
            n = link.sp.in_waiting
            if n > 0:
                pend += link.sp.read(n)
            else:
                time.sleep(0.002)
            while b"\n" in pend:
                line, pend = pend.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if not s.startswith("yaw:"):
                    continue
                parts = s[4:].split(",")
                if len(parts) < 2:
                    continue
                try:
                    data.append((time.time() - t0, float(parts[0]), float(parts[1])))
                except ValueError:
                    pass
    finally:
        link.command("STOP", 0.3)

    if len(data) < 20:
        print("数据不足 n=%d" % len(data))
        return

    ys = [x[2] for x in data]
    ts = [x[0] for x in data]
    # 超调：位移是正的，拉回时越过 0 往负走就是超调
    if disp > 0:
        worst = min(ys)
        over = -worst if worst < 0.0 else 0.0
    else:
        worst = max(ys)
        over = worst if worst > 0.0 else 0.0
    tail = ys[-max(20, len(ys) // 5):]
    sse = statistics.fmean(tail)
    settle = None
    for i in range(len(ys)):
        if all(abs(y) <= args.tol for y in ys[i:]):
            settle = ts[i]
            break
    t90 = next((t for t, y in zip(ts, ys)
                if abs(y) <= abs(disp) * 0.1), None)

    print("回程: 起点 %+.2f 度 -> 终点 %+.3f 度  (n=%d)" % (disp, sse, len(ys)))
    print("  超调(越过 0) = %.3f 度   反向极值 %+.2f 度" % (over, worst))
    print("  回到 10%% 以内用时 = %s" % (fmt(t90, 3) + "s" if t90 else "n/a"))
    print("  稳定(±%.1f度)用时 = %s"
          % (args.tol, fmt(settle, 3) + "s" if settle else "n/a"))
    print("  最终静差 = %+.3f 度" % sse)
    print("  判据: 超调 <= %.1f 度 -> %s ; 静差 <= %.1f 度 -> %s"
          % (args.max_over, "通过" if over <= args.max_over else "不通过",
             args.tol, "通过" if abs(sse) <= args.tol else "不通过"))

    name = "recover_%s_kp%g_ki%g_kd%g_d%g.csv" % (
        args.fb.lower(), args.kp, args.ki, args.kd, args.disp)
    path = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..",
        "docs", "tuning", name))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "target_deg", "yaw_deg"])
        for t, g, y in data:
            w.writerow(["%.4f" % t, "%.3f" % g, "%.3f" % y])
    print("  csv:", path)


def cmd_step(link, args):
    """闭环阶跃：整段实验中用"非阻塞 + 自打时间戳"的方式采数据流。

    为什么不用 tune_yaw.record()：那个函数用 sp.read(8192)，pyserial 会一直
    等到凑满 8192 字节才返回，100Hz 的流上要等 3 秒多，结果整段采样的时间戳
    全是同一个值（第一版实测踩到），超调/上升时间根本算不出来。
    """
    link.command("STOP", 0.3)
    link.command("CLEAR", 0.3)
    link.command("FB=%s" % args.fb, 0.5)
    if args.stall_off:
        link.command("STALL=OFF", 0.3)
    link.command("ZERO", 0.4)
    link.command("LIMIT=%d" % args.limit, 0.4)
    link.command("OMAX=%d" % args.omax, 0.4)
    link.command("PID=%g,%g,%g" % (args.kp, args.ki, args.kd), 0.5)
    link.set_stream(True)
    if args.raw:
        link.command("RAW=ON", 0.4)
    link.command("SET=%g" % args.target, 0.4)
    # 先发不读：command() 会先排水 0.4s，阶跃在头 0.4s 里就跑到一半了，
    # 那样测不到真正的起点（实测 y0 直接是 28.5°）。
    link.sp.reset_input_buffer()
    link.send("START")
    data = []
    pend = b""
    t0 = time.time()
    try:
        while (time.time() - t0) < args.sec:
            n = link.sp.in_waiting
            if n > 0:
                pend += link.sp.read(n)
            else:
                time.sleep(0.002)
            while b"\n" in pend:
                line, pend = pend.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if not s.startswith("yaw:"):
                    continue
                p = s[4:].split(",")
                if len(p) < 2:
                    continue
                try:
                    vals = [float(x) for x in p]
                except ValueError:
                    continue
                rate = vals[2] if len(vals) > 2 else float("nan")
                data.append((time.time() - t0, vals[0], vals[1], rate))
    finally:
        link.command("STOP", 0.3)
        if args.raw:
            link.command("RAW=OFF", 0.3)

    if len(data) < 20:
        print("数据不足 n=%d（检查 STREAM 是否 ON）" % len(data))
        return
    ts = [d[0] for d in data]
    ys = [d[2] for d in data]
    tgt = args.target
    y0 = ys[0]
    peak = max(ys)
    peak = peak if (tgt >= y0) else min(ys)
    over = peak - tgt
    t90 = next((t for t, y in zip(ts, ys)
                if (y - y0) * (1.0 if tgt >= y0 else -1.0) >= 0.9 * abs(tgt - y0)), None)
    settle = None
    for i in range(len(ys)):
        if all(abs(y - tgt) <= args.tol for y in ys[i:]):
            settle = ts[i]
            break
    tail = ys[-max(20, len(ys) // 5):]
    print("fb=%s Kp=%g Ki=%g Kd=%g target=%g" % (args.fb, args.kp, args.ki, args.kd, tgt))
    print("  n=%d y0=%.2f peak=%.2f overshoot=%.2f° rise90=%s settle(±%.1f°)=%s sse=%.3f jitter=%.3f"
          % (len(data), y0, peak, over, fmt(t90, 3) + "s" if t90 else "n/a",
             args.tol, fmt(settle, 3) + "s" if settle else "n/a",
             statistics.fmean(tail) - tgt, statistics.pstdev(tail)))

    # 角速度通道：区分「转盘真的在来回撞」和「陀螺数据被污染」。
    # 电机满速约 79dps，出现几百 dps 的尖峰只能是坏数据或机械撞击。
    rt = [d[3] for d in data if d[3] == d[3]]
    if rt:
        rt_tail = [d[3] for d in data[-max(20, len(data) // 5):] if d[3] == d[3]]
        spikes = [r for r in rt if abs(r) > 200.0]
        print("  rate: max|r|=%.1f dps  tail mean=%.2f std=%.2f dps  |r|>200dps 样本 %d/%d"
              % (max(abs(r) for r in rt),
                 statistics.fmean(rt_tail) if rt_tail else float("nan"),
                 statistics.pstdev(rt_tail) if len(rt_tail) > 1 else 0.0,
                 len(spikes), len(rt)))

    name = "step_%s_kp%g_ki%g_kd%g_t%g.csv" % (args.fb.lower(), args.kp, args.ki, args.kd, tgt)
    path = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..",
        "docs", "tuning", name))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "target_deg", "yaw_deg", "rate_dps"])
        for t, g, y, r in data:
            w.writerow(["%.4f" % t, "%.3f" % g, "%.3f" % y,
                        "" if r != r else "%.2f" % r])
    print("  csv:", path)
    if args.trace:
        last = -1e9
        print("%7s %9s %9s" % ("t(s)", "yaw", "rate"))
        for t, _g, y, r in data:
            if t - last < 0.1:
                continue
            last = t
            print("%7.2f %9.2f %9.2f" % (t, y, r))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM6")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("pose")
    p.add_argument("--sec", type=float, default=0.0, help="0 = 只打一次")
    p.add_argument("--interval", type=float, default=1.0)

    p = sub.add_parser("pulse")
    p.add_argument("--pm", type=int, default=250)
    p.add_argument("--ms", type=int, default=120)
    p.add_argument("--budget", type=float, default=20.0)
    p.add_argument("--settle", type=int, default=700)

    p = sub.add_parser("cpd")
    p.add_argument("--n", type=int, default=4)
    p.add_argument("--pm", type=int, default=250)
    p.add_argument("--ms", type=int, default=140)
    p.add_argument("--budget", type=float, default=20.0)
    p.add_argument("--settle", type=int, default=700)
    p.add_argument("--guard", type=float, default=140.0, help="离限位小于它就反向")

    p = sub.add_parser("trace")
    p.add_argument("--pm", type=int, default=250)
    p.add_argument("--ms", type=int, default=120)
    p.add_argument("--delay", type=float, default=0.8)
    p.add_argument("--sec", type=float, default=3.5)
    p.add_argument("--settle", type=int, default=700)
    p.add_argument("--fb", default="IMU", help="数据流 ch1 用谁的角，默认 IMU 便于看陀螺")
    p.add_argument("--raw", type=int, default=1, help="1 = RAW=ON，帧里带 rate 通道")
    p.add_argument("--print-hz", dest="print_hz", type=float, default=20.0)

    p = sub.add_parser("axes")
    p.add_argument("--sec", type=float, default=5.0)

    p = sub.add_parser("hand")
    p.add_argument("--deg", type=float, default=90.0, help="打算转多少度（只用于提示）")
    p.add_argument("--min-deg", dest="min_deg", type=float, default=20.0,
                   help="编码器变化超过它才认为开始转了")
    p.add_argument("--settle", type=float, default=1.5, help="判定停稳所需的静止秒数")
    p.add_argument("--timeout", type=float, default=90.0, help="最长等待秒数")

    p = sub.add_parser("rec")
    p.add_argument("--sec", type=float, default=40.0)
    p.add_argument("--out", default="")
    p.add_argument("--raw", type=int, default=1)

    p = sub.add_parser("recover")
    p.add_argument("--disp", type=float, default=30.0, help="要推开的度数")
    p.add_argument("--dir", type=float, default=1.0, help=">0 往正方向推开，<0 往负方向")
    p.add_argument("--pm", type=int, default=250)
    p.add_argument("--ms", type=int, default=400)
    p.add_argument("--settle", type=int, default=600)
    p.add_argument("--sec", type=float, default=6.0)
    p.add_argument("--tol", type=float, default=1.5, help="静差与稳定判据（度）")
    p.add_argument("--max-over", dest="max_over", type=float, default=0.5,
                   help="允许的超调（度），任务书要求无超调")
    p.add_argument("--kp", type=float, default=12.0)
    p.add_argument("--ki", type=float, default=1.5)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--limit", type=int, default=120)
    p.add_argument("--fb", default="IMU")
    p.add_argument("--stall-off", dest="stall_off", action="store_true", default=True)
    p.add_argument("--stall-on", dest="stall_off", action="store_false")

    p = sub.add_parser("step")
    p.add_argument("--target", type=float, default=30.0)
    p.add_argument("--kp", type=float, default=30.0)
    p.add_argument("--ki", type=float, default=1.5)
    p.add_argument("--kd", type=float, default=2.0)
    p.add_argument("--sec", type=float, default=8.0)
    p.add_argument("--tol", type=float, default=1.5, help="settle 判据 ±度")
    p.add_argument("--omax", type=int, default=300)
    p.add_argument("--limit", type=int, default=165)
    p.add_argument("--fb", default="IMU")
    p.add_argument("--stall-off", dest="stall_off", action="store_true", default=True)
    p.add_argument("--stall-on", dest="stall_off", action="store_false")
    p.add_argument("--raw", type=int, default=1,
                   help="1 = RAW=ON，帧里带角速度通道（默认开，用来识别坏数据）")
    p.add_argument("--trace", action="store_true")

    args = ap.parse_args()
    link = Link(args.port)
    try:
        {"pose": cmd_pose, "pulse": cmd_pulse, "cpd": cmd_cpd,
         "trace": cmd_trace, "axes": cmd_axes, "hand": cmd_hand,
         "recover": cmd_recover, "rec": cmd_rec, "step": cmd_step}[args.cmd](link, args)
    finally:
        try:
            link.command("COAST", 0.2)
            link.set_stream(True)
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    main()
