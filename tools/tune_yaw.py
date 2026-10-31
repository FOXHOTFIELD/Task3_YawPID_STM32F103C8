#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Yaw 转盘调参上位机（配合 Task3_YawPID 固件的串口命令行）。

做的事：
  1. 读固件的 FireWater 数据流  yaw:<Target>,<Yaw>  作为采样源（100Hz）；
  2. 发命令做开环/闭环实验，按时间打时间戳；
  3. 自动算指标：稳态误差、超调、上升时间、调节时间、稳态抖动；
  4. 每次实验把原始波形存成 CSV，方便事后画图或复现。

用法示例：
  python tune_yaw.py info
  python tune_yaw.py openloop --duty 200 --sec 1.5
  python tune_yaw.py deadzone --duties 40,60,80,120,160,200,300
  python tune_yaw.py step --target 30 --kp 20 --ki 0 --kd 0 --sec 8
  python tune_yaw.py sweep --target 30 --kp 12,16,20,24 --ki 5 --kd 3
  python tune_yaw.py raw "STOP"

注意：转盘排线余量有限，脚本里所有运动都先限幅，且每次实验结束自动 STOP。
"""
import argparse
import csv
import os
import statistics
import sys
import time

import serial

DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 115200
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "docs", "tuning")


class Link(object):
    def __init__(self, port, baud=DEFAULT_BAUD):
        self.sp = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.3)
        self.sp.reset_input_buffer()
        self.sp.reset_output_buffer()

    def close(self):
        try:
            self.sp.close()
        except Exception:
            pass

    def send(self, cmd):
        self.sp.write((cmd + "\r\n").encode("ascii", "ignore"))
        self.sp.flush()

    def drain(self, sec=0.3):
        out = []
        t_end = time.time() + sec
        pending = b""
        while time.time() < t_end:
            pending += self.sp.read(4096)
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                out.append(line.decode("utf-8", "replace").strip())
        return out

    def command(self, cmd, wait=0.5):
        """发一条命令并收集回显（丢掉数据流行）"""
        self.drain(0.05)
        self.send(cmd)
        lines = self.drain(wait)
        return [ln for ln in lines if not ln.startswith("yaw:")]

    def set_stream(self, on=True):
        self.command("STREAM=%s" % ("ON" if on else "OFF"), 0.4)
        time.sleep(0.2)
        self.sp.reset_input_buffer()

    def record(self, sec, settle=0.05):
        """采集 sec 秒的数据流，返回 [(t, target, yaw)]"""
        t0 = time.time()
        data = []
        pending = b""
        while time.time() - t0 < sec:
            pending += self.sp.read(8192)
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if not s.startswith("yaw:"):
                    continue
                body = s[4:]
                parts = body.split(",")
                if len(parts) < 2:
                    continue
                try:
                    tg = float(parts[0])
                    yw = float(parts[1])
                except ValueError:
                    continue
                data.append((time.time() - t0, tg, yw))
            if sec - (time.time() - t0) < settle:
                time.sleep(0.001)
        return data


# ----------------------------------------------------------------------
# 指标
# ----------------------------------------------------------------------
def metrics(data, target, tol=1.0):
    """给一段阶跃响应算指标（data 是 [(t,target,yaw)]）"""
    if len(data) < 10:
        return None
    ys = [d[2] for d in data]
    ts = [d[0] for d in data]
    y0 = ys[0]
    step = target - y0
    if abs(step) < 1.0:
        return None
    sgn = 1.0 if step > 0 else -1.0
    peak = max(ys) if sgn > 0 else min(ys)
    overshoot = (peak - target) * sgn
    # 90% 上升时间
    thr = y0 + 0.9 * step
    t90 = None
    for t, y in zip(ts, ys):
        if (y - thr) * sgn >= 0:
            t90 = t
            break
    # 进入并保持在 ±tol 内的时间
    t_settle = None
    for i in range(len(ys)):
        if all(abs(y - target) <= tol for y in ys[i:]):
            t_settle = ts[i]
            break
    tail = ys[-max(20, int(len(ys) * 0.2)):]
    sse = statistics.fmean(tail) - target
    jitter = statistics.pstdev(tail)
    return {
        "y_start": y0,
        "target": target,
        "peak": peak,
        "overshoot": overshoot,
        "rise_90": t90,
        "settle_1deg": t_settle,
        "sse": sse,
        "jitter": jitter,
        "samples": len(ys),
    }


def dump_csv(name, data):
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.normpath(os.path.join(LOG_DIR, name))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "target_deg", "yaw_deg"])
        for row in data:
            w.writerow(["%.4f" % row[0], "%.3f" % row[1], "%.3f" % row[2]])
    return path


def fmt(v, unit="", nd=2):
    if v is None:
        return "n/a"
    return ("%." + str(nd) + "f%s") % (v, unit)


# ----------------------------------------------------------------------
# 子命令
# ----------------------------------------------------------------------
def cmd_info(link, args):
    for ln in link.command("INFO", 0.8):
        print(ln)


def cmd_raw(link, args):
    for ln in link.command(args.text, 0.8):
        print(ln)


def stat_fields(link):
    """读一条 STAT，返回 {键: 值} 字典（cont/left/yaw 都能用）"""
    out = {}
    for ln in link.command("STAT", 0.5):
        if ln.startswith("STAT "):
            for kv in ln.split()[1:]:
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    out[k] = v
    return out


def pulse_move(link, duty, total_ms, pulse_ms=200, max_deg=20.0):
    """短脉冲驱动：每 200ms 一个脉冲，中间 COAST，实时看 yaw 变化，超预算立刻停。

    为什么要这样：长命令 DUTY=xxx 一旦上位机异常退出，电机会一直转
    （固件现在有 1.5s 点动超时兜底，但双保险更好）。脉冲式最多多转 200ms。
    """
    t0 = time.time()
    y_start = None
    data = []
    while (time.time() - t0) * 1000.0 < total_ms:
        link.send("DUTY=%d" % duty)
        chunk = link.record(pulse_ms / 1000.0)
        link.send("COAST")
        data.extend(chunk)
        if chunk:
            if y_start is None:
                y_start = chunk[0][2]
            moved = abs(chunk[-1][2] - y_start)
            if moved > max_deg:
                print("  [ABORT] moved %.1f deg > budget %.1f, motor off" % (moved, max_deg))
                break
    return data, y_start


def wait_still(link, rate_thr=2.0, timeout=12.0):
    """等转盘停下来（|rate| 小），最多等 timeout 秒。返回最终 rate。"""
    t0 = time.time()
    r = 999.0
    while time.time() - t0 < timeout:
        f = stat_fields(link)
        try:
            r = abs(float(f.get("rate", "999")))
        except ValueError:
            r = 999.0
        if r < rate_thr:
            return r
        time.sleep(0.3)
    return r


def ramp_set(link, target, dps=15.0, step_deg=1.0):
    """把目标角缓慢爬过去，而不是一步阶跃（转盘惯性大，阶跃会甩飞）。"""
    f = stat_fields(link)
    try:
        cur_goal = float(f.get("tgt", "0"))
    except ValueError:
        cur_goal = 0.0
    delta = target - cur_goal
    n = max(1, int(abs(delta) / step_deg))
    dt = step_deg / dps
    for i in range(1, n + 1):
        link.send("SET=%.2f" % (cur_goal + delta * i / n))
        time.sleep(dt)
    link.send("SET=%.2f" % target)


def home(link, timeout=8.0):
    """先回标定零点：每次实验前都回零，保证物理扭转量不会越攒越多"""
    link.command("SET=0", 0.3)
    link.command("START", 0.3)
    t0 = time.time()
    while time.time() - t0 < timeout:
        f = stat_fields(link)
        try:
            if abs(float(f.get("err", "999"))) < 1.5:
                break
        except ValueError:
            pass
        time.sleep(0.4)
    link.command("STOP", 0.3)
    f = stat_fields(link)
    print("  home: yaw=%s cont=%s left=%s" % (f.get("yaw"), f.get("cont"), f.get("left")))
    return f


def prepare(link, kp, ki, kd, zero=True, do_home=True):
    link.set_stream(True)
    link.command("STOP", 0.3)
    if do_home:
        home(link)
    if zero:
        link.command("ZERO", 0.4)
    if kp is not None:
        link.command("PID=%g,%g,%g" % (kp, ki, kd), 0.5)


def cmd_openloop(link, args):
    """开环点动：脉冲给力，测 Δyaw 与编码器增量，判断方向/是否卡死"""
    link.set_stream(True)
    link.command("STOP", 0.3)
    st0 = stat_fields(link)
    print("before: yaw=%s cont=%s left=%s enc=%s"
          % (st0.get("yaw"), st0.get("cont"), st0.get("left"), st0.get("enc")))
    enc0 = link.command("ENC", 0.5)
    print("        %s" % (enc0[0] if enc0 else ""))

    data, y0 = pulse_move(link, args.duty, args.ms, max_deg=args.max_deg)
    st1 = stat_fields(link)
    enc1 = link.command("ENC", 0.5)
    y1 = data[-1][2] if data else (y0 or 0.0)
    dy = (y1 - y0) if y0 is not None else 0.0
    print("after : yaw=%s cont=%s left=%s enc=%s"
          % (st1.get("yaw"), st1.get("cont"), st1.get("left"), st1.get("enc")))
    print("        %s" % (enc1[0] if enc1 else ""))
    print("delta : %.2f deg in %dms (duty=%d)" % (dy, args.ms, args.duty))
    if abs(dy) < 0.5:
        print("        -> 几乎没动：力度低于死区，或电机/电源没接")
    path = dump_csv("openloop_duty%d.csv" % args.duty, data)
    print("csv:", path)


def cmd_deadzone(link, args):
    """扫力度找启动死区：每档 0.6s 脉冲，测完反向同样时间回位"""
    link.set_stream(True)
    link.command("STOP", 0.3)
    link.command("DZ=0", 0.3)
    duties = [int(x) for x in args.duties.split(",")]
    print("duty(permille)  d_yaw(deg)  rate(dps)  moved?")

    rows = []
    for d in duties:
        st = stat_fields(link)
        data, y0 = pulse_move(link, d, args.ms, max_deg=args.max_deg)
        y1 = data[-1][2] if data else (y0 or 0.0)
        # 反向回位（时间一致，近似回原点）
        pulse_move(link, -d, args.ms, max_deg=args.max_deg * 2)
        dy = (y1 - y0) if y0 is not None else 0.0
        rows.append((d, dy, dy / (args.ms / 1000.0)))
        print("%6d          %8.2f   %8.2f   %s"
              % (d, dy, dy / (args.ms / 1000.0), "YES" if abs(dy) > 1.0 else "-"))
        time.sleep(0.2)

    moved = [r for r in rows if abs(r[1]) > 1.0]
    if moved:
        print("\n建议 DZ ≈ %d（第一档能转的力度 - 10~20）" % max(0, moved[0][0] - 15))
    print("当前 left=%s" % stat_fields(link).get("left"))


def run_step(link, target, kp, ki, kd, sec, use_zero=True, do_home=True):
    """闭环阶跃：返回 (metrics, data)"""
    prepare(link, kp, ki, kd, zero=use_zero, do_home=do_home)
    link.command("SET=%g" % target, 0.4)
    link.command("START", 0.4)
    data = link.record(sec)
    link.command("STOP", 0.3)
    m = metrics(data, target)
    return m, data


def cmd_step(link, args):
    m, data = run_step(link, args.target, args.kp, args.ki, args.kd, args.sec,
                       use_zero=(not args.no_zero), do_home=(not args.no_home))
    print("Kp=%s Ki=%s Kd=%s target=%g" % (args.kp, args.ki, args.kd, args.target))
    if m:
        print("  sse=%s  overshoot=%s  rise90=%s  settle(±1°)=%s  jitter=%s  n=%d"
              % (fmt(m["sse"], "°"), fmt(m["overshoot"], "°"), fmt(m["rise_90"], "s", 3),
                 fmt(m["settle_1deg"], "s"), fmt(m["jitter"], "°", 3), m["samples"]))
    else:
        print("  数据不足，检查 START 是否成功")
    path = dump_csv("step_kp%g_ki%g_kd%g_t%g.csv" % (args.kp, args.ki, args.kd, args.target), data)
    print("  csv:", path)


def cmd_sweep(link, args):
    kps = [float(x) for x in args.kp.split(",")]
    print("target=%g Ki=%g Kd=%g" % (args.target, args.ki, args.kd))
    print("   Kp   sse(deg)  overshoot(deg)  rise90(s)  settle(s)  jitter(deg)  verdict")
    best = None
    for kp in kps:
        m, data = run_step(link, args.target, kp, args.ki, args.kd, args.sec,
                           use_zero=(not args.no_zero), do_home=(not args.no_home))
        if m is None:
            print("  %5.1f   (no data)" % kp)
            continue
        verdict = "ok"
        if m["jitter"] > 1.5:
            verdict = "oscillating"
        elif m["overshoot"] > 5.0:
            verdict = "overshoot"
        elif m["rise_90"] is None or m["rise_90"] > args.sec * 0.6:
            verdict = "too slow"
        print("  %5.1f   %8.2f   %14.2f   %8s   %8s   %10.3f  %s"
              % (kp, m["sse"], m["overshoot"], fmt(m["rise_90"], "", 2),
                 fmt(m["settle_1deg"], "", 2), m["jitter"], verdict))
        score = abs(m["sse"]) + 0.5 * abs(m["overshoot"]) + m["jitter"]
        if best is None or score < best[0]:
            best = (score, kp, m)
        dump_csv("sweep_kp%g.csv" % kp, data)
    if best:
        print("\n建议 Kp=%g（综合 sse+overshoot+jitter 最小）" % best[1])


def cmd_kick(link, args):
    """抗扰测试：闭环锁定后请人手拨开 target，再放开，看回复过程"""
    prepare(link, args.kp, args.ki, args.kd, zero=True)
    link.command("SET=%g" % args.target, 0.4)
    link.command("START", 0.4)
    print("已锁定 %g°，请在 3 秒内用手把转盘拨开 %g° 后松手 ..." % (args.target, args.push))
    time.sleep(args.delay)
    data = link.record(args.sec)
    link.command("STOP", 0.3)
    path = dump_csv("kick_%g_%g.csv" % (args.target, args.push), data)
    if data:
        ys = [d[2] for d in data]
        print("yaw 范围 %.2f ~ %.2f，末尾 %.2f（目标 %g）"
              % (min(ys), max(ys), ys[-1], args.target))
        tail = ys[-max(20, len(ys) // 5):]
        print("回复后均值 %.2f，抖动 %.2f°" % (statistics.fmean(tail), statistics.pstdev(tail)))
    print("csv:", path)


def cmd_cal(link, args):
    """标定零偏并自动重试，直到 static quality 达标（QUALITY 是唯一可靠的静止判据）"""
    link.set_stream(False)
    best = None
    for attempt in range(1, args.tries + 1):
        link.command("CAL", 0.3)
        time.sleep(1.9)
        info = link.command("INFO", 0.8)
        q = None
        bias = None
        for ln in info:
            if "cal_quality" in ln:
                for kv in ln.split():
                    if kv.startswith("bias="):
                        bias = kv[5:]
                    elif kv.startswith("cal_quality="):
                        q = int(kv.split("=")[1])
        print("  第 %d 次: quality=%s bias=%s dps" % (attempt, q, bias))
        if q is not None and (best is None or q > best[0]):
            best = (q, bias)
        if q is not None and q >= args.min_quality:
            break
        time.sleep(0.6)
    link.command("ZERO", 0.5)
    st = link.command("STAT", 0.6)
    print("  最终: quality=%s bias=%s  |  %s" % (best[0] if best else "?",
                                                 best[1] if best else "?",
                                                 st[0] if st else ""))


def cmd_watch(link, args):
    """按固定间隔打印 STAT，用来观察漂移/静止保持/手转编码器"""
    link.set_stream(False)
    print("t(s)   yaw     cont     left   rate   zt  enc      duty")
    t0 = time.time()
    while time.time() - t0 < args.sec:
        f = stat_fields(link)
        print("%5.1f  %7s %8s %5s %7s %4s %8s %5s"
              % (time.time() - t0, f.get("yaw", "?"), f.get("cont", "?"),
                 f.get("left", "?"), f.get("rate", "?"), f.get("zt", "?"),
                 f.get("enc", "?"), f.get("duty", "?")))
        time.sleep(args.interval)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("info")

    p = sub.add_parser("openloop")
    p.add_argument("--duty", type=int, default=200)
    p.add_argument("--ms", type=int, default=400, help="总通电时间（按 200ms 脉冲发）")
    p.add_argument("--max-deg", type=float, default=15.0, help="允许转动的角度预算")

    p = sub.add_parser("deadzone")
    p.add_argument("--duties", default="40,60,80,100,120,160,200")
    p.add_argument("--ms", type=int, default=600)
    p.add_argument("--max-deg", type=float, default=12.0)

    p = sub.add_parser("step")
    p.add_argument("--target", type=float, default=30.0)
    p.add_argument("--kp", type=float, default=20.0)
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--sec", type=float, default=8.0)
    p.add_argument("--no-home", action="store_true")
    p.add_argument("--no-zero", action="store_true")

    p = sub.add_parser("sweep")
    p.add_argument("--target", type=float, default=30.0)
    p.add_argument("--kp", default="12,16,20,24,28")
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--sec", type=float, default=8.0)
    p.add_argument("--no-home", action="store_true")
    p.add_argument("--no-zero", action="store_true")

    p = sub.add_parser("kick")
    p.add_argument("--target", type=float, default=0.0)
    p.add_argument("--push", type=float, default=30.0)
    p.add_argument("--kp", type=float, default=20.0)
    p.add_argument("--ki", type=float, default=5.0)
    p.add_argument("--kd", type=float, default=3.0)
    p.add_argument("--delay", type=float, default=2.0)
    p.add_argument("--sec", type=float, default=10.0)

    p = sub.add_parser("cal")
    p.add_argument("--min-quality", type=int, default=80)
    p.add_argument("--tries", type=int, default=3)

    p = sub.add_parser("watch")
    p.add_argument("--sec", type=float, default=10.0)
    p.add_argument("--interval", type=float, default=1.0)

    p = sub.add_parser("raw")
    p.add_argument("text")

    args = ap.parse_args()
    link = Link(args.port)
    try:
        {"info": cmd_info, "openloop": cmd_openloop, "deadzone": cmd_deadzone,
         "step": cmd_step, "sweep": cmd_sweep, "kick": cmd_kick, "watch": cmd_watch, "cal": cmd_cal,
         "raw": cmd_raw}[args.cmd](link, args)
    finally:
        try:
            link.command("STOP", 0.2)
        except Exception:
            pass
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
