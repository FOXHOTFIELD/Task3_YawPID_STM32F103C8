#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
STM32F1 串口 ISP 烧录工具（AN3155 协议，不依赖 ST-Link / FlyMcu）

用法：
    python serial_isp.py --detect                      扫描串口并尝试进入 ISP 同步
    python serial_isp.py --flash Task1.hex --port COM3  烧写（擦除 -> 写入 -> 读回校验 -> 跳转）
    python serial_isp.py --monitor --port COM3 --seconds 5   按 8N1 抓应用输出

前提：板子 BOOT0=1（3V3）、BOOT1=0（GND），复位后停在 System Memory bootloader。
      ISP 协议固定 8 数据位 + 偶校验 + 1 停止位（8E1），115200。
      应用运行时是 8N1，所以 monitor 用 8N1。

协议要点（AN3155）：
    ACK = 0x79, NACK = 0x1F；同步字节 0x7F。
    GET 0x00 / GET_ID 0x02 / READ 0x11 / GO 0x21 / WRITE 0x31 / ERASE 0x43。
    地址与数据都以"字节 + 异或校验"成对发送。
"""
import argparse
import os
import sys
import time

import serial
import serial.tools.list_ports

ACK = 0x79
NACK = 0x1F
SYNC = 0x7F

CMD_GET = 0x00
CMD_GET_VERSION = 0x01
CMD_GET_ID = 0x02
CMD_READ = 0x11
CMD_GO = 0x21
CMD_WRITE = 0x31
CMD_ERASE = 0x43

FLASH_BASE = 0x08000000
BLOCK = 256


# ----------------------------------------------------------------- 基础工具
def xor_bytes(data):
    x = 0
    for b in data:
        x ^= b
    return x


def find_port(preferred=None):
    ports = list(serial.tools.list_ports.comports())
    if preferred:
        for p in ports:
            if p.device.upper() == preferred.upper():
                return p
        return None
    for p in ports:
        desc = (p.description or '').upper()
        if 'CH340' in desc or 'USB-SERIAL' in desc or 'CP210' in desc:
            return p
    return None


class Bootloader:
    def __init__(self, port, baud=115200, timeout=2.0):
        self.sp = serial.Serial()
        self.sp.port = port
        self.sp.baudrate = baud
        self.sp.bytesize = serial.EIGHTBITS
        self.sp.parity = serial.PARITY_EVEN      # ISP 要 8E1
        self.sp.stopbits = serial.STOPBITS_ONE
        self.sp.timeout = timeout
        self.sp.write_timeout = timeout
        self.sp.dtr = False
        self.sp.rts = False

    def open(self):
        self.sp.open()
        # CH340 刚打开时第一次发送容易丢/变形；bootloader 又只用第一个 0x7F 测速，
        # 所以这里留 150ms 让驱动和线路稳定下来。
        time.sleep(0.15)

    def close(self):
        if self.sp.is_open:
            self.sp.close()

    # ---- 低层收发 ----
    def _wait_ack(self, timeout=None):
        if timeout is not None:
            self.sp.timeout = timeout
        b = self.sp.read(1)
        if not b:
            return None
        return b[0]

    def sync(self, tries=6):
        for i in range(tries):
            self.sp.reset_input_buffer()
            self.sp.write(bytes([SYNC]))
            self.sp.flush()
            r = self._wait_ack(0.4)
            if r == ACK:
                return True
            time.sleep(0.1)
        return False

    def command(self, cmd, timeout=1.0):
        time.sleep(0.006)          # 命令之间留一点间隙：劣质线路/慢边沿时更稳
        self.sp.reset_input_buffer()
        self.sp.write(bytes([cmd]))
        self.sp.flush()
        return self._wait_ack(timeout)

    # ---- 具体命令 ----
    def get(self):
        if self.command(CMD_GET) != ACK:
            return None
        n = self._wait_ack(1.0)
        if n is None:
            return None
        data = self.sp.read(n + 1)
        ack = self._wait_ack(1.0)
        if len(data) != n + 1 or ack != ACK:
            return None
        return {'version': data[0], 'commands': list(data[1:])}

    def get_id(self):
        if self.command(CMD_GET_ID) != ACK:
            return None
        n = self._wait_ack(1.0)
        if n is None:
            return None
        data = self.sp.read(n + 1)
        ack = self._wait_ack(1.0)
        if len(data) != n + 1 or ack != ACK:
            return None
        pid = (data[0] << 8) | data[1]
        return pid

    def erase_all(self, timeout=25.0):
        if self.command(CMD_ERASE) != ACK:
            return False
        # 0x43 全局擦除：0xFF + 校验 0x00
        self.sp.write(bytes([0xFF, 0x00]))
        self.sp.flush()
        return self._wait_ack(timeout) == ACK

    def write_block(self, addr, data):
        assert len(data) == BLOCK
        if self.command(CMD_WRITE) != ACK:
            return False
        addr_bytes = bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
                            (addr >> 8) & 0xFF, addr & 0xFF])
        self.sp.write(addr_bytes + bytes([xor_bytes(addr_bytes)]))
        self.sp.flush()
        if self._wait_ack(1.0) != ACK:
            return False
        time.sleep(0.004)          # 地址帧与数据帧之间留间隙
        payload = bytes([BLOCK - 1]) + data
        self.sp.write(payload + bytes([xor_bytes(payload)]))
        self.sp.flush()
        return self._wait_ack(3.0) == ACK

    def read_mem(self, addr, length):
        if self.command(CMD_READ) != ACK:
            return None
        addr_bytes = bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
                            (addr >> 8) & 0xFF, addr & 0xFF])
        self.sp.write(addr_bytes + bytes([xor_bytes(addr_bytes)]))
        self.sp.flush()
        if self._wait_ack(1.0) != ACK:
            return None
        payload = bytes([length - 1])
        self.sp.write(payload + bytes([xor_bytes(payload)]))
        self.sp.flush()
        if self._wait_ack(1.0) != ACK:
            return None
        data = self.sp.read(length)
        if len(data) != length:
            return None
        if self._wait_ack(1.0) != ACK:
            return None
        return data

    def go(self, addr):
        if self.command(CMD_GO) != ACK:
            return False
        addr_bytes = bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
                            (addr >> 8) & 0xFF, addr & 0xFF])
        self.sp.write(addr_bytes + bytes([xor_bytes(addr_bytes)]))
        self.sp.flush()
        return self._wait_ack(1.0) == ACK


# ----------------------------------------------------------------- HEX 解析
def hex_to_image(path):
    """把 Intel HEX 解析成 (起始地址, 字节串)，未覆盖区域填 0xFF，按 256 对齐补齐。"""
    mem = {}
    base = 0
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(':'):
                raise ValueError('第%d行不是 Intel HEX 记录: %s' % (lineno, line[:20]))
            raw = bytes.fromhex(line[1:])
            count = raw[0]
            addr = (raw[1] << 8) | raw[2]
            rtype = raw[3]
            data = raw[4:4 + count]
            # Intel HEX 的校验是"所有字节累加和取二的补码"，不是异或
            if ((sum(raw[:-1]) + raw[-1]) & 0xFF) != 0:
                raise ValueError('第%d行校验和错误' % lineno)
            if rtype == 0x00:
                for i, b in enumerate(data):
                    mem[base + addr + i] = b
            elif rtype == 0x01:
                break
            elif rtype == 0x04:
                base = ((data[0] << 8) | data[1]) << 16
            elif rtype == 0x02:
                base = ((data[0] << 8) | data[1]) << 4
    if not mem:
        raise ValueError('HEX 里没有数据记录')
    lo = min(mem)
    hi = max(mem)
    lo = lo & ~(BLOCK - 1)
    hi = (hi + BLOCK) & ~(BLOCK - 1)
    buf = bytearray(b'\xFF' * (hi - lo))
    for a, b in mem.items():
        buf[a - lo] = b
    return lo, bytes(buf)


# ----------------------------------------------------------------- 各种模式
def do_detect(args):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print('没有找到任何串口。')
        return 1
    print('可用串口：')
    for p in ports:
        print('  %-6s %s' % (p.device, p.description))

    cand = find_port(args.port)
    if cand is None:
        print('\n没找到 CH340 之类的 USB 串口。')
        return 1
    print('\n尝试在 %s 上同步（8E1 @ %d）……' % (cand.device, args.baud))

    combos = [(False, False), (True, False), (False, True), (True, True)]
    for dtr, rts in combos:
        bl = Bootloader(cand.device, args.baud)
        try:
            bl.sp.dtr = dtr
            bl.sp.rts = rts
            bl.open()
        except Exception as e:
            print('  打开失败：%s' % e)
            return 1
        ok = bl.sync(tries=2)
        if ok:
            print('  同步成功（DTR=%s RTS=%s）' % (dtr, rts))
            info = bl.get()
            pid = bl.get_id()
            if info:
                print('  Bootloader 版本：0x%02X，命令数：%d' % (info['version'], len(info['commands'])))
            if pid is not None:
                print('  PID：0x%04X（0x0410 = STM32F10x Medium Density）' % pid)
            bl.close()
            print('\n结论：板子已经在 ISP bootloader 里，可以直接烧写。')
            return 0
        bl.close()
        time.sleep(0.15)

    print('\n同步失败：没有收到 ACK(0x79)。')
    print('请检查：')
    print('  1) BOOT0 是否接 3V3（跳线插 1 侧），BOOT1 接 GND；')
    print('  2) 改完跳线后按一下复位键（或拔插一次供电）；')
    print('  3) CH340 的 TX 接 PA10、RX 接 PA9、GND 与板子共地；')
    print('  4) 板子有电（灯亮）。')
    return 2


def do_flash(args):
    port = args.port
    if not port:
        p = find_port(None)
        if p is None:
            print('没找到串口，请用 --port 指定（如 --port COM3）')
            return 1
        port = p.device

    lo, image = hex_to_image(args.flash)
    print('HEX：%s' % args.flash)
    print('映像：0x%08X 起，%d 字节（%.1f KB），按 %d 字节对齐' % (lo, len(image), len(image) / 1024.0, BLOCK))
    if len(image) > 64 * 1024:
        print('警告：超过 64KB，F103C8 装不下')

    bl = Bootloader(port, args.baud, timeout=2.0)
    bl.open()
    try:
        if args.no_sync:
            # 芯片已经同步过（比如刚才握手成功过），再发 0x7F 只会被当未知命令 NACK
            pid = bl.get_id()
            info = bl.get()
            print('跳过同步：PID 0x%04X，bootloader 版本 0x%02X' %
                  (pid or 0, (info['version'] if info else 0)))
            if pid is None and info is None:
                print('芯片对 GET_ID/GET 都没有响应，无法在"已同步"状态下继续')
                return 2
        elif args.wait > 0:
            deadline = time.time() + args.wait
            ok = False
            print('等待板子进入 ISP 模式（最长 %d 秒）：请把 BOOT0 接 3V3，然后按一下复位键……' % args.wait)
            n = 0
            while time.time() < deadline:
                if bl.sync(tries=1):
                    ok = True
                    break
                # 如果芯片已经处于"已同步"状态（上电后没复位过），
                # 再发 0x7F 只会被当成未知命令回 NACK；这时直接探测 GET_ID
                pid = bl.get_id()
                if pid is not None:
                    print('  芯片已处于已同步状态（PID 0x%04X），不再重新同步' % pid)
                    ok = True
                    break
                n += 1
                if n % 5 == 0:
                    print('  …仍在等待（%d 次尝试，剩余 %.0fs）' % (n, deadline - time.time()))
                    sys.stdout.flush()
                time.sleep(0.1)
            if not ok:
                print('等待超时：一直没收到 ACK(0x79)。')
                print('  → 确认 BOOT0 = 3V3（1 侧）、BOOT1 = GND、按过复位、板子有电、PA9 接到 CH340 RX')
                return 2
            print('  收到 ACK，已进入 ISP 模式')
        elif not bl.sync():
            print('同步失败：板子不在 ISP 模式（BOOT0 未置高或未复位）')
            return 2
        info = bl.get()
        pid = bl.get_id()
        print('已连上 bootloader：版本 0x%02X，PID 0x%04X' % (info['version'] if info else 0, pid or 0))

        t0 = time.time()
        if not bl.erase_all():
            print('整片擦除失败')
            return 3
        print('整片擦除完成（%.2fs）' % (time.time() - t0))

        t0 = time.time()
        total = len(image) // BLOCK
        for i in range(total):
            addr = lo + i * BLOCK
            chunk = image[i * BLOCK:(i + 1) * BLOCK]
            if not bl.write_block(addr, chunk):
                print('写入失败 @0x%08X（第 %d/%d 块）' % (addr, i + 1, total))
                return 4
            if (i + 1) % 16 == 0 or (i + 1) == total:
                print('  写入 %3d/%3d 块（%5.1f%%）' % (i + 1, total, 100.0 * (i + 1) / total))
        dt = time.time() - t0
        print('写入完成：%d 字节，用 %.2fs（%.1f KB/s）' % (len(image), dt, len(image) / 1024.0 / dt))

        # ---- 读回校验 ----
        bad = 0
        for i in range(total):
            addr = lo + i * BLOCK
            chunk = image[i * BLOCK:(i + 1) * BLOCK]
            back = bl.read_mem(addr, BLOCK)
            if back is None:
                print('读回失败 @0x%08X' % addr)
                return 5
            if back != chunk:
                diff = sum(1 for a, b in zip(back, chunk) if a != b)
                print('校验不一致 @0x%08X，%d 字节不同' % (addr, diff))
                bad += 1
        if bad:
            print('读回校验失败：%d 块不一致' % bad)
            return 6
        print('读回校验通过：%d 块逐字节一致' % total)

        if args.no_go:
            print('按 --no-go 要求不跳转，请手动复位。')
        else:
            if bl.go(lo):
                print('已发送 GO 0x%08X：程序开始运行（注意：BOOT0=1 时仅本次有效）' % lo)
            else:
                print('GO 命令未获应答（有些 bootloader 会直接跳走，属正常）')
        return 0
    finally:
        bl.close()


def do_monitor(args):
    port = args.port
    if not port:
        p = find_port(None)
        port = p.device if p else None
    if not port:
        print('没找到串口')
        return 1
    sp = serial.Serial(port, args.baud, bytesize=8, parity=serial.PARITY_NONE,
                       stopbits=1, timeout=0.2)
    print('监听 %s @ %d 8N1，共 %d 秒……' % (port, args.baud, args.seconds))
    end = time.time() + args.seconds
    buf = b''
    while time.time() < end:
        chunk = sp.read(64)
        if chunk:
            buf += chunk
            sys.stdout.write(chunk.decode('utf-8', errors='replace'))
            sys.stdout.flush()
    sp.close()
    print('\n--- 共收到 %d 字节 ---' % len(buf))
    return 0


def do_loopback(args):
    """回环测试：把 CH340 的 TX 和 RX 用杜邦线短接，看发出去的字节能原样回来。

    这一步用来区分"模块/线/驱动坏了"和"板子那侧没接好"：
      回环正常 → 模块、驱动、USB 都没问题，故障在板子侧（供电/接线/BOOT0）；
      回环失败 → 模块或杜邦线/驱动有问题，先换一根线或换个模块。
    """
    port = args.port
    if not port:
        p = find_port(None)
        port = p.device if p else None
    if not port:
        print('没找到串口')
        return 1

    payload = bytes([0x55, 0xAA, 0x00, 0xFF] + list(range(0x41, 0x41 + 28)))
    sp = serial.Serial(port, args.baud, bytesize=8, parity=serial.PARITY_NONE,
                       stopbits=1, timeout=0.5)
    sp.reset_input_buffer()
    sp.write(payload)
    sp.flush()
    got = sp.read(len(payload) + 32)
    sp.close()

    print('回环测试 %s @ %d 8N1' % (port, args.baud))
    print('  发送 %d 字节: %s' % (len(payload), ' '.join('%02X' % b for b in payload[:12]) + ' ...'))
    print('  收到 %d 字节: %s' % (len(got), (' '.join('%02X' % b for b in got[:12]) + ' ...') if got else '(无)'))

    if not got:
        print('\n结果：一个字节都没回来 → TX/RX 没有短接，或模块/线有问题。')
        return 3
    n = min(len(payload), len(got))
    same = sum(1 for i in range(n) if payload[i] == got[i])
    print('  逐字节相同：%d / %d' % (same, n))
    if same >= n - 1 and len(got) >= len(payload) - 1:
        print('\n结果：回环正常 → CH340 模块、驱动、USB 都没问题，故障在板子那一侧。')
        return 0
    print('\n结果：回来的数据不对 → 模块或杜邦线有问题（也可能是短接没接牢）。')
    return 4


BAUD_CANDIDATES = [9600, 19200, 38400, 57600, 115200]


def do_auto_flash(args):
    """等复位 + 自动找一个能用的波特率，然后烧写。

    为什么要这样：STM32 的串口 bootloader 只在复位后用同步字节 0x7F 自动测速，
    一旦被错误波特率的字节骗进去，它就会一直锁在错误状态，只有复位能重来。
    慢速（9600）对接触不良/长线的容忍度远高于 115200，所以先试慢的。
    """
    port = args.port or (find_port(None) or type('X', (), {'device': None})).device
    if not port:
        print('没找到串口')
        return 1

    lo, image = hex_to_image(args.flash)
    print('HEX：%s -> %d 字节（%.1f KB）' % (args.flash, len(image), len(image) / 1024.0))
    print('\n请在下面倒计时内给板子按一次复位键（BOOT0 保持接 3V3）：')
    deadline = time.time() + args.wait
    bl = None
    baud_ok = None
    rounds = 0
    while time.time() < deadline:
        rounds += 1
        for baud in BAUD_CANDIDATES:
            if time.time() >= deadline:
                break
            b = Bootloader(port, baud, timeout=0.4)
            try:
                b.open()
            except Exception as e:
                print('  打开 %s 失败：%s' % (port, e))
                return 1
            got = b.sync(tries=1)
            if got:
                info = b.get()
                if info is not None:
                    print('\n★ 在 %d bps 同步成功，bootloader 版本 0x%02X' % (baud, info['version']))
                    bl, baud_ok = b, baud
                    break
                print('  在 %d bps 收到 ACK 但 GET 无响应，继续找' % baud)
            b.close()
            print('  第 %d 轮：%6d bps 无有效响应（剩 %.0fs）' % (rounds, baud, deadline - time.time()))
            sys.stdout.flush()
        if bl is not None:
            break
    if bl is None:
        print('\n超时：%d 轮 × %d 种波特率都没拿到有效 bootloader 响应。' %
              (rounds, len(BAUD_CANDIDATES)))
        return 2

    try:
        pid = bl.get_id()
        print('PID：0x%04X（0x0410 = STM32F10x Medium Density）' % (pid or 0))
        t0 = time.time()
        if not bl.erase_all():
            print('整片擦除失败')
            return 3
        print('整片擦除完成（%.2fs）' % (time.time() - t0))

        t0 = time.time()
        total = len(image) // BLOCK
        for i in range(total):
            addr = lo + i * BLOCK
            if not bl.write_block(addr, image[i * BLOCK:(i + 1) * BLOCK]):
                print('写入失败 @0x%08X（第 %d/%d 块）' % (addr, i + 1, total))
                return 4
            if (i + 1) % 16 == 0 or (i + 1) == total:
                print('  写入 %3d/%3d 块（%5.1f%%）' % (i + 1, total, 100.0 * (i + 1) / total))
        dt = time.time() - t0
        print('写入完成：%d 字节，%.2fs（%.1f KB/s）' % (len(image), dt, len(image) / 1024.0 / dt))

        bad = 0
        for i in range(total):
            addr = lo + i * BLOCK
            back = bl.read_mem(addr, BLOCK)
            if back is None or back != image[i * BLOCK:(i + 1) * BLOCK]:
                print('校验不一致 @0x%08X' % addr)
                bad += 1
        if bad:
            print('读回校验失败：%d 块不一致' % bad)
            return 6
        print('读回校验通过：%d 块逐字节一致' % total)

        if not args.no_go:
            if bl.go(lo):
                print('已发送 GO 0x%08X：程序开始运行' % lo)
        return 0
    finally:
        bl.close()


def do_slow_flash(args):
    """慢速救急烧写：串口只开一次，等用户按复位，然后在低速下反复发同步字节。

    为什么这样做：
      - CH340 刚打开时第一个字节容易丢/变形，而 bootloader 只用复位后的第一个
        0x7F 自动测速，所以"重开串口 + 立刻发 0x7F"这种写法在接触不良时几乎必败；
      - 慢速（默认 9600）对长线/劣质杜邦线/接地不良的容忍度远高于 115200。
    """
    port = args.port or (find_port(None) or type('X', (), {'device': None})).device
    if not port:
        print('没找到串口')
        return 1
    lo, image = hex_to_image(args.flash)
    print('HEX：%s -> %d 字节（%.1f KB）' % (args.flash, len(image), len(image) / 1024.0))
    print('\n>>> 现在给板子按一次复位键（BOOT0 必须接 3V3）<<<')
    print('    我会在 %d bps 下持续等 %d 秒，串口全程不重开。' % (args.baud, args.wait))

    bl = Bootloader(port, args.baud, timeout=0.3)
    bl.open()
    try:
        time.sleep(0.25)
        bl.sp.reset_input_buffer()
        bl.sp.write(b'\x7f')              # 预热，丢掉驱动开工时的第一个字节
        bl.sp.flush()
        time.sleep(0.1)
        bl.sp.reset_input_buffer()

        deadline = time.time() + args.wait
        ok = False
        n = 0
        got_bytes = {}
        echo = 0
        while time.time() < deadline:
            n += 1
            bl.sp.write(b'\x7f')
            bl.sp.flush()
            r = bl._wait_ack(0.2)
            if r == ACK:
                ok = True
                print('\n★ 第 %d 次同步成功（%d bps）：芯片收到干净的 0x7F' % (n, args.baud))
                break
            if r is not None:
                got_bytes[r] = got_bytes.get(r, 0) + 1
                if r == SYNC:
                    echo += 1
                    if echo == 30:
                        print('\n⚠ 连续 %d 次收到自己发出去的 0x7F —— 这是回环！' % echo)
                        print('  → 说明 CH340 的 TX 与 RX 仍然短接着（或 RX 直接收到了自己的发送）。')
                        print('  → 请拆掉短接线，把模块 TXD 接 PA10、RXD 接 PA9、GND 接 GND，再按复位。')
                        print('  继续等待，直到你改好接线……')
                        sys.stdout.flush()
            # 芯片可能已经处于"已同步"状态（那就不会 ACK 0x7F），探一下 GET
            if (n % 4) == 0:
                info = bl.get()
                if info is not None:
                    print('\n★ 芯片已处于已同步状态（bootloader 版本 0x%02X），直接开始烧写' % info['version'])
                    ok = True
                    break
            if n % 10 == 0:
                top = ', '.join('%02X×%d' % (k, v) for k, v in sorted(got_bytes.items())[:4])
                print('  已试 %d 次，收到过的字节：%s（剩 %.0fs）' % (n, top or '无', deadline - time.time()))
                sys.stdout.flush()
        if not ok:
            print('\n%d 秒内一直没同步成功（%d 次尝试）。' % (args.wait, n))
            print('→ 高度怀疑 CH340 TX → PA10 这条线/接地有问题，请先做回环测试：')
            print('   把模块的 TX 与 RX 直接短接，然后运行：')
            print('   python serial_isp.py --loopback --port %s' % port)
            return 2

        info = bl.get()
        if info is not None:
            print('  bootloader 版本 0x%02X，命令数 %d' % (info['version'], len(info['commands'])))
        pid = bl.get_id()
        print('  PID 0x%04X' % (pid or 0))

        t0 = time.time()
        if not bl.erase_all():
            print('整片擦除失败')
            return 3
        print('  整片擦除完成（%.2fs）' % (time.time() - t0))

        t0 = time.time()
        total = len(image) // BLOCK
        for i in range(total):
            addr = lo + i * BLOCK
            if not bl.write_block(addr, image[i * BLOCK:(i + 1) * BLOCK]):
                print('写入失败 @0x%08X（第 %d/%d 块）' % (addr, i + 1, total))
                return 4
            if (i + 1) % 8 == 0 or (i + 1) == total:
                print('  写入 %3d/%3d 块（%5.1f%%）' % (i + 1, total, 100.0 * (i + 1) / total))
                sys.stdout.flush()
        dt = time.time() - t0
        print('  写入完成：%d 字节，%.1fs（%.1f KB/s）' % (len(image), dt, len(image) / 1024.0 / dt))

        bad = 0
        for i in range(total):
            addr = lo + i * BLOCK
            back = bl.read_mem(addr, BLOCK)
            if back is None or back != image[i * BLOCK:(i + 1) * BLOCK]:
                print('  校验不一致 @0x%08X' % addr)
                bad += 1
        if bad:
            print('读回校验失败：%d 块不一致' % bad)
            return 6
        print('  读回校验通过：%d 块逐字节一致' % total)

        if not args.no_go:
            if bl.go(lo):
                print('  已发送 GO 0x%08X' % lo)
        return 0
    finally:
        bl.close()


def do_watch_flash(args):
    """看门狗烧写：串口只开一次，在 9600 / 115200 之间轮询等复位。

    为什么这样设计：
      - bootloader 只在复位后的第一个 0x7F 上测速，所以必须"人按复位"与
        "我们正在发 0x7F"同时发生；
      - 9600 的位宽是 115200 的 12 倍，对接触不良/长线宽容得多，所以以 9600 为主；
      - 串口全程不重开，避免 CH340 打开后第一个字节丢失。
    """
    port = args.port or (find_port(None) or type('X', (), {'device': None})).device
    if not port:
        print('没找到串口')
        return 1
    lo, image = hex_to_image(args.flash)
    print('HEX：%s -> %d 字节（%.1f KB）' % (args.flash, len(image), len(image) / 1024.0))

    bl = Bootloader(port, 9600, timeout=0.25)
    bl.open()
    try:
        # 固定波特率时不要掺杂其它速率：bootloader 只认复位后第一个 0x7F，
        # 掺着发会让它锁到"恰好那一拍"的速率上，后面全对不上。
        if args.baud == 115200:
            pattern = [115200] * 4
        else:
            pattern = [args.baud, args.baud, args.baud, 115200]
        deadline = time.time() + args.wait
        print('\n>>> 现在按一下板子的 RESET 键（或拔插一次 USB 给板子重新上电）<<<')
        print('    窗口 %d 秒；以 9600 为主轮询，偶尔试 115200。' % args.wait)
        i = 0
        baud_ok = None
        last_r = None
        while time.time() < deadline and baud_ok is None:
            baud = pattern[i % len(pattern)]
            i += 1
            bl.sp.baudrate = baud
            time.sleep(0.03)
            bl.sp.reset_input_buffer()
            bl.sp.write(b'\x7f')
            bl.sp.flush()
            last_r = bl._wait_ack(0.25)
            if last_r == ACK:
                baud_ok = baud
                print('\n★ 第 %d 次尝试在 %d bps 同步成功' % (i, baud))
                break
            if last_r == SYNC:
                print('  ⚠ 收到自己的回环，TX/RX 可能又短接了')
            if (i % 10) == 0:
                pid = bl.get_id()
                if pid is not None:
                    baud_ok = baud
                    print('\n★ 芯片已处于已同步状态（PID 0x%04X，%d bps）' % (pid, baud))
                    break
            if (i % 25) == 0:
                print('  …第 %d 次未握手（剩 %.0fs，最近回应 %s）' %
                      (i, deadline - time.time(),
                       ('%02X' % last_r) if last_r is not None else '无'))
                sys.stdout.flush()
        if baud_ok is None:
            print('\n%d 秒内没等到复位/同步（共 %d 次）' % (args.wait, i))
            return 2

        bl.sp.baudrate = baud_ok
        info = bl.get()
        pid = bl.get_id()
        print('  bootloader 版本 0x%02X，PID 0x%04X' %
              ((info['version'] if info else 0), (pid or 0)))

        t0 = time.time()
        if not bl.erase_all():
            print('整片擦除失败')
            return 3
        print('  整片擦除完成（%.2fs）' % (time.time() - t0))

        t0 = time.time()
        total = len(image) // BLOCK
        for i in range(total):
            addr = lo + i * BLOCK
            if not bl.write_block(addr, image[i * BLOCK:(i + 1) * BLOCK]):
                print('写入失败 @0x%08X（第 %d/%d 块）' % (addr, i + 1, total))
                return 4
            if (i + 1) % 8 == 0 or (i + 1) == total:
                print('  写入 %3d/%3d 块（%5.1f%%）' % (i + 1, total, 100.0 * (i + 1) / total))
                sys.stdout.flush()
        dt = time.time() - t0
        print('  写入完成：%d 字节，%.1fs（%.1f KB/s）' % (len(image), dt, len(image) / 1024.0 / dt))

        bad = 0
        for i in range(total):
            addr = lo + i * BLOCK
            back = bl.read_mem(addr, BLOCK)
            if back is None or back != image[i * BLOCK:(i + 1) * BLOCK]:
                print('  校验不一致 @0x%08X' % addr)
                bad += 1
        if bad:
            print('读回校验失败：%d 块不一致' % bad)
            return 6
        print('  读回校验通过：%d 块逐字节一致' % total)

        if not args.no_go:
            if bl.go(lo):
                print('  已发送 GO 0x%08X：程序开始运行' % lo)
        return 0
    finally:
        bl.close()


def do_daemon_flash(args):
    """守护式烧写：扛得住 USB 拔插。

    状态机：
      串口不在 -> 每 2 秒找一次 CH340，找到就打开
      串口在   -> 持续发 0x7F 等复位后的同步
      同步成功 -> 擦除/写入/读回校验/GO
      中途掉线 -> 关掉句柄，回到"找串口"，不退出
    """
    lo, image = hex_to_image(args.flash)
    print('HEX：%s -> %d 字节（%.1f KB）' % (args.flash, len(image), len(image) / 1024.0))
    print('守护模式：%d 秒内插好 CH340、BOOT0 接 3V3、按一次复位即可。' % args.wait)
    print('（中途拔插 USB 不会让脚本退出，它会自己重连）\n')

    deadline = time.time() + args.wait
    bl = None
    i = 0
    while time.time() < deadline:
        # ---------- 确保串口打开 ----------
        if bl is None or not bl.sp.is_open:
            p = find_port(args.port) if args.port else find_port(None)
            if p is None:
                print('  [%.0fs] 没找到 CH340，2 秒后再找…' % (deadline - time.time()))
                sys.stdout.flush()
                time.sleep(2)
                continue
            try:
                bl = Bootloader(p.device, args.baud, timeout=0.3)
                bl.open()
                print('  已打开 %s（%s）' % (p.device, p.description))
            except Exception as e:
                print('  打开 %s 失败：%s' % (p.device, e))
                bl = None
                time.sleep(2)
                continue

        # ---------- 同步 ----------
        i += 1
        try:
            bl.sp.baudrate = args.baud
            bl.sp.reset_input_buffer()
            bl.sp.write(b'\x7f')
            bl.sp.flush()
            r = bl._wait_ack(0.3)
        except serial.SerialException as e:
            print('  串口断开（%s），准备重连…' % e)
            try:
                bl.close()
            except Exception:
                pass
            bl = None
            time.sleep(1)
            continue
        except OSError as e:
            print('  串口异常（%s），准备重连…' % e)
            bl = None
            time.sleep(1)
            continue

        if r == ACK:
            print('\n★ 第 %d 次在 %d bps 同步成功，开始烧写' % (i, args.baud))
        else:
            if (i % 25) == 0:
                print('  [%.0fs] 第 %d 次未握手（最近回应 %s）' %
                      (deadline - time.time(), i,
                       ('%02X' % r) if r is not None else '无'))
                sys.stdout.flush()
            continue

        # ---------- 烧写 ----------
        try:
            info = bl.get()
            pid = bl.get_id()
            print('  bootloader 版本 0x%02X，PID 0x%04X' %
                  ((info['version'] if info else 0), (pid or 0)))
            if not bl.erase_all():
                print('  整片擦除失败，回到等待状态（按复位后可重来）')
                bl.close()
                bl = None
                continue
            print('  擦除完成')

            t0 = time.time()
            total = len(image) // BLOCK
            for k in range(total):
                addr = lo + k * BLOCK
                if not bl.write_block(addr, image[k * BLOCK:(k + 1) * BLOCK]):
                    raise IOError('写入失败 @0x%08X（第 %d/%d 块）' % (addr, k + 1, total))
                if (k + 1) % 8 == 0 or (k + 1) == total:
                    print('  写入 %3d/%3d 块（%5.1f%%）' % (k + 1, total, 100.0 * (k + 1) / total))
                    sys.stdout.flush()
            dt = time.time() - t0
            print('  写入完成：%d 字节，%.1fs（%.1f KB/s）' % (len(image), dt, len(image) / 1024.0 / dt))

            bad = 0
            for k in range(total):
                addr = lo + k * BLOCK
                back = bl.read_mem(addr, BLOCK)
                if back is None or back != image[k * BLOCK:(k + 1) * BLOCK]:
                    print('  校验不一致 @0x%08X' % addr)
                    bad += 1
            if bad:
                raise IOError('读回校验失败：%d 块不一致' % bad)
            print('  读回校验通过：%d 块逐字节一致' % total)

            if not args.no_go:
                if bl.go(lo):
                    print('  已发送 GO 0x%08X：程序开始运行' % lo)
            print('\n★★★ 烧写完成 ★★★')
            return 0
        except (serial.SerialException, OSError, IOError) as e:
            print('  烧写中断：%s' % e)
            print('  → 回到等待状态；请确认接线后按复位，脚本会重来一次')
            try:
                bl.close()
            except Exception:
                pass
            bl = None
            time.sleep(1)

    print('\n守护窗口 %d 秒用尽，未完成烧写。' % args.wait)
    return 2


def main():
    ap = argparse.ArgumentParser(description='STM32F1 串口 ISP 烧录工具（AN3155）')
    ap.add_argument('--detect', action='store_true', help='扫描并尝试同步')
    ap.add_argument('--flash', metavar='HEX', help='要烧写的 Intel HEX 文件')
    ap.add_argument('--monitor', action='store_true', help='按 8N1 监听应用输出')
    ap.add_argument('--loopback', action='store_true',
                    help='回环测试（需先把 CH340 的 TX 与 RX 短接）')
    ap.add_argument('--port', help='串口号，如 COM3；不填则自动找 CH340')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=int, default=5)
    ap.add_argument('--auto', action='store_true',
                    help='等复位并自动选择波特率后烧写（配合 --flash/--wait）')
    ap.add_argument('--slow', action='store_true',
                    help='慢速救急模式：串口只开一次，等复位后在低速下反复同步并烧写')
    ap.add_argument('--no-sync', action='store_true',
                    help='跳过同步握手（芯片已处于已同步状态时用）')
    ap.add_argument('--watch', action='store_true',
                    help='看门狗模式：单串口 + 9600/115200 轮询等复位，抓到就烧')
    ap.add_argument('--daemon', action='store_true',
                    help='守护模式：扛得住 USB 拔插，插好/复位后自动烧写')
    ap.add_argument('--no-go', action='store_true', help='烧完不跳转')
    ap.add_argument('--wait', type=int, default=0,
                    help='烧写前先等待 N 秒让板子进入 ISP 模式（期间可拨 BOOT0/按复位）')
    args = ap.parse_args()

    if args.detect:
        return do_detect(args)
    if args.flash and args.auto:
        return do_auto_flash(args)
    if args.flash and args.slow:
        return do_slow_flash(args)
    if args.flash and args.watch:
        return do_watch_flash(args)
    if args.flash and args.daemon:
        return do_daemon_flash(args)
    if args.flash:
        return do_flash(args)
    if args.monitor:
        return do_monitor(args)
    if args.loopback:
        return do_loopback(args)
    ap.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
