#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
生成任务三（TB6612 + 水平转盘 Yaw 闭环）的实现说明 docx。

文档里的引脚、定时器参数、PID 默认值、编译体积、单元测试结论全部取自
本工程的源码与真实编译/测试输出；"未验证"的部分在文末明确列出。

用法：python make_doc3.py
"""
import os
import shutil

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

# 路径由脚本自身位置推导：本脚本在 <仓库>/tools/ 下，仓库根就是上一级。
# 这样整个仓库可以整体搬走，不用改代码。
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.dirname(PROJ)
OUT = os.path.join(PROJ, 'docs', '26级第三期_Yaw闭环转盘_实现说明.docx')
OUT2 = os.path.join(BASE, '文档', '26级第三期_Yaw闭环转盘_实现说明.docx')


def set_style_font(style, cn, en, size, bold=False):
    style.font.name = en
    style.font.size = Pt(size)
    style.font.bold = bold
    rpr = style.element.get_or_add_rPr()
    rfonts = rpr.find(qn('w:rFonts'))
    if rfonts is None:
        rfonts = OxmlElement('w:rFonts')
        rpr.append(rfonts)
    rfonts.set(qn('w:ascii'), en)
    rfonts.set(qn('w:hAnsi'), en)
    rfonts.set(qn('w:eastAsia'), cn)


def shade(cell, color):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:val'), 'clear')
    shd.set(qn('w:color'), 'auto')
    shd.set(qn('w:fill'), color)
    tcPr.append(shd)


def new_doc():
    doc = Document()
    sec = doc.sections[0]
    sec.left_margin = Cm(2.2)
    sec.right_margin = Cm(2.2)
    sec.top_margin = Cm(2.2)
    sec.bottom_margin = Cm(2.0)

    set_style_font(doc.styles['Normal'], '宋体', 'Times New Roman', 10.5)
    doc.styles['Normal'].paragraph_format.space_after = Pt(3)
    doc.styles['Normal'].paragraph_format.line_spacing = 1.25

    set_style_font(doc.styles['Heading 1'], '黑体', 'Arial', 15, bold=True)
    set_style_font(doc.styles['Heading 2'], '黑体', 'Arial', 12.5, bold=True)
    set_style_font(doc.styles['Heading 3'], '黑体', 'Arial', 11, bold=True)
    for name, color in (('Heading 1', 0x1F3864), ('Heading 2', 0x2E5496), ('Heading 3', 0x33589C)):
        doc.styles[name].font.color.rgb = RGBColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF)
    return doc


def para(doc, text, bold=False, size=10.5, align=None):
    p = doc.add_paragraph()
    run = p.add_run(text)
    run.bold = bold
    run.font.size = Pt(size)
    if align == 'center':
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    return p


def bullets(doc, items):
    for it in items:
        p = doc.add_paragraph(style='List Bullet')
        p.paragraph_format.space_after = Pt(2)
        run = p.add_run(it)
        run.font.size = Pt(10.5)


def code(doc, lines):
    for ln in lines:
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.left_indent = Cm(0.6)
        run = p.add_run(ln)
        run.font.name = 'Consolas'
        run.font.size = Pt(9)
        run.font.color.rgb = RGBColor(0x33, 0x33, 0x33)


def table(doc, headers, rows, widths=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = ''
        run = hdr[i].paragraphs[0].add_run(h)
        run.bold = True
        run.font.size = Pt(9.5)
        shade(hdr[i], 'DCE6F1')
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = ''
            run = cells[i].paragraphs[0].add_run(str(v))
            run.font.size = Pt(9.5)
    if widths:
        for row in t.rows:
            for i, w in enumerate(widths):
                row.cells[i].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)
    return t


def build():
    doc = new_doc()

    # ---------------- 封面 ----------------
    para(doc, '26 级第三期考核任务', bold=True, size=12, align='center')
    para(doc, 'TB6612 电机驱动与水平转盘偏航角闭环控制', bold=True, size=20, align='center')
    para(doc, '实现说明', bold=False, size=13, align='center')
    para(doc, '')
    table(doc, ['项目', '内容'], [
        ['主控', 'STM32F103C8T6 最小系统板，HSE 8MHz × PLL9 = 72MHz'],
        ['执行机构', 'TB6612FNG 双 H 桥 + 直流减速电机（带正交编码器）'],
        ['反馈', 'MPU6050 陀螺 Z 轴积分 Yaw（上电相对角）+ TIM2 编码器；两者可在线切换，'
                 '整定就是先在编码器环上做完再切回陀螺'],
        ['工具链', 'Keil MDK 5.43a / ARMCC V5.06 / MicroLIB / -O2 / C99'],
        ['版本', 'v1.2.8（陀螺反馈完成整定；切换反馈源自动套用增益与 ZUPT）'],
        ['构建结果', '0 Error, 0 Warning；Code 30628 B, RO 896 B, RW 260 B, ZI 1836 B；HEX 89450 B'],
        ['单元测试', '主机端 55 项断言全部通过'],
        ['实机指标', '陀螺反馈：60 秒锁定静差 -0.059 度；±30 度阶跃超调 <=3.18 度、上升 0.18~0.20s；'
                 '拨开 45 度后超调 0.55~0.80 度、0.26s 稳定；±180 最短路径用 err 字段验证走短边'],
    ], widths=[3.0, 12.5])
    doc.add_page_break()

    # ---------------- 1 方案概述 ----------------
    doc.add_heading('1  任务与方案概述', level=1)
    para(doc, '考核要求用 STM32 HAL 库完成 TB6612 电机驱动、定时器节拍控制、PID 闭环、串口命令与 VOFA+ 波形，'
              '使水平转盘锁定目标角度、抗外力扰动、跨 ±180° 走最短路径，并用软件限位保护排线。')
    para(doc, '本工程的方案要点：')
    bullets(doc, [
        '执行层：TIM3_CH1(PA6) 输出 20kHz 硬件 PWM，TB6612 的 AIN1/AIN2 决定方向，AIN1=AIN2=1 实现短路刹车，STBY 由 PB0 软件控制，上电默认待机。',
        '调度层：TIM4 每 1ms 进一次中断，中断里只累加计数并置标志；主循环在标志到来时执行 200Hz 控制：读传感器 → 读编码器 → 选反馈源 → PID → 写 PWM。全程无阻塞延时。',
        '感知层：MPU6050 贴在转盘上，陀螺 Z 轴与电机轴平行；用 DWT 周期计数器测量真实 dt，对 gz 去零偏后积分得到 Yaw。电机自带正交编码器经 TIM2 四倍频计数，折算成转盘角度后同样可以作为闭环反馈。',
        '控制层：位置式/增量式可选 PID，带输出限幅、积分限幅与条件积分抗饱和，微分项对测量值求导（微分先行），并支持用干净的测速代替位置差分。',
        '交互层：串口命令行（SET/START/STOP/CAL/ZERO/KP/KI/KD/FB/CPD/…）+ VOFA+ FireWater 双通道波形；短按按键启停、长按重新标定。',
        '安全层：连续累计角 ±165° 软件限位，接近限位只允许回收方向出力，越限立即刹车并报故障；编码器可选堵转检测；开环点动限位守卫 + 1.5s 超时断电。',
    ])

    # ---------------- 2 硬件 ----------------
    doc.add_heading('2  硬件接线与电源', level=1)
    doc.add_heading('2.1  引脚分配', level=2)
    table(doc, ['功能', '引脚', '模式', '接到'], [
        ['PWMA', 'PA6', 'TIM3_CH1，AF 推挽，20kHz', 'TB6612 PWMA'],
        ['AIN1 / AIN2', 'PA4 / PA5', 'GPIO 推挽输出', 'TB6612 AIN1 / AIN2'],
        ['STBY', 'PB0', 'GPIO 推挽，上电默认低', 'TB6612 STBY'],
        ['编码器 A / B', 'PA15 / PB3', 'TIM2_CH1/CH2（部分重映射），上拉输入，4 倍频', '编码器 C1 / C2'],
        ['PWMB（预留）', 'PA7', 'TIM3_CH2，本工程不发波', '第二路电机备用'],
        ['I2C1_SCL / SDA', 'PB6 / PB7', 'AF 开漏，400kHz', 'MPU6050 SCL / SDA'],
        ['USART1_TX / RX', 'PA9 / PA10', '115200 8N1', 'CH340 RX / TX'],
        ['状态灯', 'PC13', 'GPIO 推挽（板载，低电平点亮）', '—'],
        ['按键', 'PA0', 'EXTI0 下降沿 + 内部上拉', '按钮 → GND'],
        ['SWD', 'PA13 / PA14', '—', '烧写调试，占用不解'],
    ], widths=[3.2, 2.6, 6.2, 3.8])

    doc.add_heading('2.2  为什么这样选引脚', level=2)
    bullets(doc, [
        'PA6/PA7 是 TIM3_CH1/CH2，且 PA4/PA5/PA6 与 PB0 在开发板同一侧相邻，四根电机控制线可以走同一侧，接线短、干扰小。',
        'STBY 用 GPIO 而非直接接 3V3：限位触发、串口 STOP、按键急停时可以立刻让 TB6612 进待机；PWM 置 0 但方向位仍有效时，电机是"短路刹车"状态，会持续发热。',
        '编码器用 TIM2 部分重映射到 PA15/PB3：TIM3 已做 PWM，TIM4 的 CH1/CH2 正好是 I2C 用的 PB6/PB7，TIM2 直连的 PA0/PA1 又被按键占用；而 PA15/PB3 是 JTAG 脚，工程只保留 SWD 正好释放，且两脚都是 5V 容忍脚。',
        'USART1（PA9/PA10）与第二期保持一致，接线不用改，CH340 插上即可看波形。',
    ])

    doc.add_heading('2.3  TB6612 电源与地', level=2)
    code(doc, [
        ' VM  <- 电机电源 6~12V（独立电池/电源，绝不可从开发板 5V 取）',
        ' VCC -> 3V3   ★必须 3.3V：VIH = 0.7×VCC，VCC=5V 时门限 3.5V，3.3V 逻辑推不动',
        ' GND -> 与开发板、电机电源负极共地',
        ' VM-GND 就近并 100uF 电解 + 100nF 瓷片，抑制换向火花干扰 I2C',
        ' AO1/AO2 -> 电机两根线',
    ])
    para(doc, '注意：不要开 TIM3 部分重映射（TIM3_REMAP），否则 CH1~CH4 会被搬到 PB4/PB5/PB0/PB1，'
              '直接和本工程的 STBY(PB0)、编码器/JTAG 脚冲突。')

    # ---------------- 3 软件结构 ----------------
    doc.add_heading('3  软件结构与调度时序', level=1)
    table(doc, ['模块', '职责'], [
        ['app.c', '1ms 节拍分发、200Hz 控制调度、VOFA 上报、状态行、按键处理、状态灯映射'],
        ['app_ctl.c', '控制状态机 BOOT/CALIB/IDLE/RUN/FAULT；采样→积分→PID→输出；限位与堵转保护'],
        ['app_pid.c', '位置式/增量式 PID；输出限幅、积分限幅、条件积分抗饱和、微分先行、微分低通'],
        ['app_yaw.c', 'Yaw 积分（去零偏、可冻结）、±180 折返、最短路径误差'],
        ['app_tb6612.c', '电机驱动：正转/反转/刹车/滑行/待机、死区补偿、可选换向"踢一脚"'],
        ['app_enc.c', 'TIM2 编码器 4 倍频计数、增量与低通速度'],
        ['app_cli.c', '串口 1 字节中断 + 环形缓冲 + 逐行解析；命令回显与 ERR 码'],
        ['app_vofa.c', 'FireWater / JustFloat 组帧；浮点定点化（不用 printf 的 %f）'],
        ['app_key.c / app_led.c', '按键消抖与长短按、状态灯闪烁（均为非阻塞状态机）'],
        ['app_mpu6050.c / app_i2c_bus.c', 'MPU6050 寄存器驱动与 I2C 双后端（沿用第二期实现）'],
    ], widths=[4.6, 11.0])

    doc.add_heading('3.1  时序', level=2)
    table(doc, ['事件', '周期', '执行者与内容'], [
        ['TIM4 更新中断', '1 ms', '中断：s_ctrl_ticks++，累到 5 置 s_ctrl_due'],
        ['控制周期', '5 ms（200Hz）', '主循环：读 MPU6050 → 读编码器 → 静止判断 → Yaw 积分 → PID → 写 PWM'],
        ['VOFA+ 上报', '10 ms（100Hz）', '主循环，STREAM_DIV = 200/100 = 2'],
        ['状态行', '1 s', '主循环，STAT=ON 时打印'],
    ], widths=[3.6, 3.4, 8.6])
    para(doc, '主循环中没有任何 HAL_Delay()：标定、日志、闪灯、按键消抖全部由节拍驱动的状态机完成，'
              '满足"主循环不得使用阻塞延时"的要求。唯一的阻塞调用是 MPU6050 初始化里的数据手册要求的等待时间，'
              '发生在 app_init() 中、节拍启动之前。')

    # ---------------- 4 TB6612 ----------------
    doc.add_heading('4  TB6612 驱动：方向、刹车与死区补偿', level=1)
    table(doc, ['操作', 'AIN1', 'AIN2', 'PWM', 'STBY', '现象'], [
        ['正转', '1', '0', '占空比', '1', '按占空比转速'],
        ['反转', '0', '1', '占空比', '1', '反方向转'],
        ['短路刹车', '1', '1', '100%', '1', '电机两端被短接，明显阻力'],
        ['滑行', '0', '0', '0', '1', '电机不受力，惯性自由转'],
        ['待机', '×', '×', '×', '0', '输出高阻，最省电最安全'],
    ], widths=[2.6, 1.4, 1.4, 1.8, 1.4, 6.6])
    para(doc, '死区处理（评分点 1"处理电机启动死区"）：直流减速电机在低占空比下只发热不转。'
              '驱动层把 PID 的力度 u（千分比，-1000~1000）映射成实际占空比：')
    code(doc, [
        'mag  = |u|',
        'duty = 死区 + mag × (1000 - 死区) / 1000        // 死区默认 145‰，可用 DZ= 改',
        'mag <= 1‰ 时直接滑行，避免零附近反复换向抖动',
    ])
    para(doc, '本机实测死区（转盘自由、DZ=0 直接给占空比，每次 0.4 秒）：')
    table(doc, ['占空比', '0.4 秒转过', '说明'], [
        ['60‰ / 90‰ / 120‰', '0.00° / 0.44° / 0.15°', '基本不动'],
        ['150‰', '12.2°', '起动门限，≈30dps'],
        ['180‰', '16.8°', '≈40dps'],
        ['300‰', '38.8°', '≈100dps'],
    ], widths=[4.0, 5.2, 6.4])
    para(doc, '所以死区取 145‰，高于它的任何非零力度都能推动电机；代价是小力度对应更大占空比，'
              '所以 Kp 不能调得太大，否则会出现"一跳一跳"的极限环。若仍启动困难，可用 KICK=<‰>,<ms> 打开'
              '换向瞬间的"踢一脚"补偿（默认关闭，避免与 PID 抢控制）。')
    para(doc, '两个必须注意的点：')
    bullets(doc, [
        '死区强依赖机构状态。MPU6050 装在转盘上时，它的排线会拖住转盘，那时 200‰ 都推不动，'
        '死区看起来像 250‰；拆掉之后 150‰ 就能起转。换机构一定要重测，否则闭环会"小误差也给满功率"。',
        '电机两根线本机是接反的，MOTOR_DIR_INVERT 必须为 1。若为 0，"正输出"会让编码器角减小，'
        '闭环变成正反馈一路加速。上电后先用 DUTY=160 打一个 0.2s 小脉冲确认 edeg 是增大的。',
    ])

    # ---------------- 5 Yaw ----------------
    doc.add_heading('5  反馈源与 Yaw 解算', level=1)
    doc.add_heading('5.1  反馈源：先编码器、后陀螺', level=2)
    para(doc, '闭环用的角度有三个来源，可以在线切换（运行中不允许切，切换时会清 PID 状态）：')
    table(doc, ['反馈源', '角度从哪来', '特性', '用途'], [
        ['FB=ENC', '编码器累计角 edeg，Δ = Δcount × SIG / CPD', '不漂、分辨率 0.07°，含减速箱齿隙', '调参主力'],
        ['FB=IMU', '陀螺 Z 轴积分', '平滑、无机械耦合，但零偏会漂', '考核要求的主反馈'],
        ['FB=FUSE', '互补滤波：陀螺出高频，编码器用 2s 时间常数把低频拉回', '兼顾平滑与不漂', '最终形态'],
    ], widths=[2.2, 6.0, 4.4, 3.0])
    para(doc, '为什么先调编码器：编码器不漂、没有振动噪声，闭环问题（方向反、死区不准、增益不对）'
              '在它上面暴露得干净；陀螺环里零偏与积分漂移会把同样的现象搅在一起，很难定位。'
              '编码器装在电机轴上，减速比把回差折算到输出端只有 0.01° 量级，所以它确实能代表转盘角度——'
              '但刻度是 CPD（每转盘度多少计数），必须标定，否则限位和增益的单位全是错的。')
    code(doc, [
        'edeg += (enc_delta() * SIG) / CPD;          // CPD 实测 6.933，SIG 为 ±1',
        'FB=ENC  时  fb = edeg,        rate = 编码器测速',
        'FB=FUSE 时  fb += rate_g*dt;  fb += dt/2000ms * (edeg - fb)',
    ])

    doc.add_heading('5.2  为什么用 DWT 计时', level=2)
    para(doc, 'Yaw = ∫(gz - bias)dt。控制周期 5ms，如果 dt 取 HAL_GetTick() 的 1ms 整数，'
              '量化误差最高 20%，积分出来的角度会系统性偏大或偏小。Cortex-M3 的 DWT 周期计数器'
              '以 72MHz 计数，dt 精度足够；无符号相减还能自动处理 59.6 秒的回绕。')
    code(doc, [
        'd      = DWT->CYCCNT - last_cyc;          // 无符号，回绕自动正确',
        'dt     = d / 72000000.0f;                 // 秒',
        'rate   = gz_dps - bias;                   // 去零偏',
        'cont  += rate * dt;                       // 连续累计角',
        'yaw    = wrap180(cont);                   // 显示角 [-180,180)',
    ])

    doc.add_heading('5.3  连续角与显示角', level=2)
    para(doc, '维护两个量是关键设计：cont 是连续累计角（不折返），yaw 是折返到 [-180,180) 的显示角。'
              '限位必须按 cont 判断——排线拧了多少是物理量；波形按 yaw 显示——用户看的是方向。'
              '从 +170° 走最短路径到 -170° 时 cont 会到 190°，这就是"允许跨 ±180°"与"限位按连续角算"'
              '能同时成立的原因。')

    doc.add_heading('5.4  零偏标定', level=2)
    bullets(doc, [
        '上电后 1.5 秒（非阻塞）以 200Hz 采样陀螺，累加均值与方差，均值作为零偏。',
        '标准差用来给"静止质量"打分：接近 0dps 得 100 分，超过 2.5dps 得 0 分并提示标定期间动了。',
        '标定结束后 yaw 归零、目标角归零、编码器计数清零，进入 IDLE。',
        '运行中可用 CAL 命令或长按按键重新标定。',
    ])

    doc.add_heading('5.5  ZUPT 静止保持', level=2)
    para(doc, '零偏不可能校准到绝对 0，残余 0.1~0.3dps 就是 6~18°/min 的漂移，长时间保持必然超差。'
              '做法是：角速度很小、且（闭环时）输出也很小时，连续 500ms 判定为静止，冻结 Yaw 积分；'
              '静止超过 1 秒后再用 0.002 的系数在线微调零偏。冻结只停积分、不停测量，所以曲线会平但角度仍可读。')

    doc.add_heading('5.6  最短路径（发挥项 3）', level=2)
    code(doc, [
        'err = wrap180(target - yaw);              // +170° 看 -170° -> +20°',
        'u   = pid_update(cont + err, cont, dt);   // 用连续角做 PID 输入输出，避免 ±180 跳变',
    ])
    para(doc, 'PID 的等效目标取 cont + err：误差是短的，但反馈量是连续的，两者都不在 ±180 处跳变。')

    # ---------------- 6 PID ----------------
    doc.add_heading('6  PID 设计与参数', level=1)
    doc.add_heading('6.1  位置式与增量式', level=2)
    table(doc, ['对比项', '位置式', '增量式'], [
        ['公式', 'u = Kp·e + ∫Ki·e·dt + Kd·de/dt', 'Δu = Kp·Δe + Ki·e·dt + Kd·Δ²e/dt'],
        ['输出含义', '直接是控制量', '是"本次要加多少"，靠外部累加'],
        ['积分', '显式积分项，需要抗饱和', '输出累加天然带积分，无积分饱问题'],
        ['限制', '需处理积分饱和', '输出限幅会丢掉增量信息，目标突变冲击大'],
        ['本工程', '默认（保持效果好）', 'MODE=INC 现场对比用'],
    ], widths=[3.0, 6.6, 6.6])

    doc.add_heading('6.2  工程细节（每条都是真机踩出来的）', level=2)
    bullets(doc, [
        '微分先行：微分项对测量值求导，u = Kp·e - Kd·d(meas)/dt。改目标角时不会出现微分冲击（setpoint kick），'
        '这也是本工程能做到"改 SET 不抖一下"的原因。',
        '条件积分抗饱和：输出顶到限幅、且误差还在往饱和方向推时，暂停积分累加；误差反向时立刻退出饱和。'
        '叠加积分项自身 ±400‰ 的硬限幅，双重保险。',
        '★ 限幅必须同步给 PID 自己：OMAX 早期只砍外层输出，PID 内部仍按 ±1000 判断饱和，积分在被砍掉的'
        '时间段里继续涨到 ±400，误差一反向就冲出一大截。把 OMAX 写进 pid->out_max/out_min 之后超调才降下来——'
        '这是"Ki 越大反而越飘"的真正根因。',
        '★ 微分改用外部测速：位置差分在 5ms 周期上的量化是"编码器 1 个计数 = 14.7dps"的台阶，'
        'Kd 一放大微分项就被量化噪声打死。代码用 pid_set_rate_source() + pid_set_rate() 把编码器/陀螺的'
        '干净测速直接喂给微分项。',
        '微分低通：d_filt = 0.7·d_filt + 0.3·d_raw，再压一层噪声。',
        '★ 小输出刹车：|u| < 6‰ 时不滑行而是切 TB6612 短路刹车。实测同样脉冲后，'
        '刹车比滑行少走 56~58%（350‰/0.1s：3.53° vs 8.46°；500‰/0.08s：5.73° vs 13.16°）。',
        '★ 制动增强（非对称出力）：加速能力与制动能力需求相反——加速大了会满力正反打（±25° 振荡），'
        '制动了小则手一推就顶不住（手推瞬时 500dps、401‰ 占空比无解）。所以正常出力受 OMAX 限制，'
        '**被外力推离目标（误差×速度<0）时放大 3 倍**到 1000‰。判据必须用误差而不是"输出与速度反向"，'
        '后者会在正常减速时误放大、刹过头反向冲出去（实测反向超调 14°）。',
    ])

    doc.add_heading('6.3  参数对响应的影响（评分点 3 现场可讲）', level=2)
    table(doc, ['参数', '作用', '过大的表现', '过小的表现'], [
        ['Kp', '决定响应速度，误差到力度的比例', '振荡、来回摆、电机一跳一跳', '追不上目标、爬行慢'],
        ['Ki', '消除稳态误差（克服摩擦与死区）', '过冲后缓慢摆回、出现低频摆动', '停在离目标几度的位置不动'],
        ['Kd', '阻尼，抑制速度、减少超调', '高频抖动、电机发热、怕噪声', '接近目标时冲过头'],
    ], widths=[1.6, 5.2, 4.8, 4.0])
    para(doc, '默认值 Kp=60、Ki=3、Kd=4（单位：‰/度、‰/(度·s)、‰/(度/s)），是真机整定结果'
              '（FB=ENC、死区 145‰、OMAX=300 + 制动增强 3×），实测：')
    table(doc, ['工况', '超调', '稳态误差', '进入 ±1.5° 用时'], [
        ['30° 阶跃（位置式）', '0', '0.66°', '0.45s'],
        ['等效扰动（目标脉冲 40°）', '反向超调 0', '—', '0.33s 回位'],
        ['等效扰动（更狠，60°/200ms）', '反向超调 0', '—', '0.32s 回位'],
        ['增量式对照 Kp=5 Ki=10 Kd=0', '3.2°', '2.5°', '2.35s'],
    ], widths=[5.4, 3.4, 3.4, 3.4])
    para(doc, '为什么 Ki 不能贪大：死区补偿把 |u|=1 都映射到 145‰，积分只要涨过"起动门限对应值"'
              '（约 12‰）电机就动。Ki 越大，积分冲过这个阈值越快，超调越大。实测 Ki=8 时 30° 阶跃超调'
              '涨到 2.7°，反而不如 Ki=3。静差的大小则由 Kp 决定：静差 ≈ 起动门限 / (Kp × 0.86)。')

    # ---------------- 7 安全 ----------------
    doc.add_heading('7  软件限位与安全策略', level=1)
    bullets(doc, [
        '限位默认 ±165°（连续角），可用 LIMIT= 在 30~300 之间调整；目标角允许范围是 ±(限位-5°)，超出回 ERR:LIMIT。',
        '接近限位 10° 内，只允许往回收方向的出力，继续往外推会被裁成 0。',
        '越过限位立刻刹车进入 FAULT，PC13 双闪、串口打印 FAULT:LIMIT；必须 STOP（或按键）解除。',
        '开环点动（DUTY/FWD/REV）同样受限位守卫：剩余角度不足且方向朝外直接拒绝并回 ERR:LIMIT；'
        '点动超过 1.5s 自动断电，防止上位机脚本异常退出后电机一直转。',
        '★ 守卫必须按"电机实际转向"判断：早期直接拿命令符号比，没考虑 INV 取反，导致反向脉冲被全部拒绝、'
        '只剩单向累积；实测把转盘一路推到 +121° 撞限位才暴露。现在用取反后的方向判断。',
        'STAT 行的 left= 实时显示距离限位还剩多少度，调参前先看它。',
        '±180° 最短路径演示：本机排线已拆除，直接 LIMIT=300，SET=150 后 SET=-150 即可（实测只走 61°）。'
        '若排线还在，则需先手动负偏 40° 再 ZERO，把 cont 的绝对偏移让出来。',
        '★ 堵转检测（STALL=ON）：力度超门限而编码器计数不变超时判定。FB=ENC 时会自动打开作为'
        '"编码器线松了"的兜底，但**用手按住转盘制造扰动时它会误判**，所以超时放宽到 2s，做手推测试前先 STALL=OFF。',
        'IMU 读取失败时：FB=ENC 下不报故障（编码器闭环不需要陀螺，当成陀螺=0 继续跑）；'
        'FB=IMU/FUSE 下连续 100 拍失败才判故障并刹车。',
    ])

    # ---------------- 8 串口 ----------------
    doc.add_heading('8  串口协议与 VOFA+', level=1)
    table(doc, ['命令', '说明'], [
        ['SET=<角度>', '设定目标角（也接受 "SET 30"），超出限位回 ERR:LIMIT'],
        ['START / STOP', '进入 / 退出闭环；STOP 后电机滑行断电'],
        ['CAL / ZERO', '重新标定零偏 / 当前角度置零（编码器角、融合角一并清零）'],
        ['KP= / KI= / KD=', '单项整定，立即生效并回显（发挥项 1）'],
        ['PID=<kp>,<ki>,<kd> / PID', '三项一起设 / 打印当前值'],
        ['MODE=POS|INC', '位置式 / 增量式'],
        ['FB=IMU|ENC|FUSE', '反馈源切换（仅非 RUN 状态）；切到 ENC 自动开堵转检测，'
                           '并自动套用该反馈源的增益与 ZUPT（ENC 30/1.5/2+ZUPT 开，IMU/FUSE 12/0.8/0+ZUPT 关）'],
        ['CPD=<n> / SIG=±1 / GSIG=±1', '编码器每转盘度计数 / 编码器计数方向 / 陀螺安装方向（面朝下 -1）'],
        ['IMU / IMU=ON|OFF', '打印一条 / 连续 20Hz 输出六轴原始值（ax ay az gx gy gzraw gzs temp）'],
        ['IMUINIT', '重新探测 MPU6050（线松过、I2C 卡死后免重启板子），成功会自动重做零偏标定'],
        ['LIMIT= / DZ= / OMAX=', '限位、死区补偿、闭环输出上限（OMAX 同时写进 PID 限幅）'],
        ['KICK= / INV=', '换向"踢一脚" / 电机方向取反'],
        ['ZUPT= / BIASAUTO= / STALL=', '静止冻结 / 怠速零偏跟踪 / 堵转检测'],
        ['FWD / REV / BRK / COAST / DUTY=<‰>', '开环验证正转、反转、刹车、滑行（仅 IDLE）'],
        ['STREAM= / RAW= / PROTO=', '数据流、第三通道、协议切换'],
        ['STAT= / INFO / ENC / RESET / HELP', '状态行、详细转储、编码器、恢复默认、帮助'],
    ], widths=[5.4, 10.2])
    para(doc, 'STAT 一行样例（fb= 当前反馈源，edeg= 编码器角，dIE= 陀螺角−编码器角，可用于估计漂移）：')
    code(doc, [
        'STAT st=IDLE fb=ENC tgt=0.000 yaw=0.000 cont=0.000 err=0.000 left=120 out=0 duty=0',
        '     rate=0.000 zt=1 enc=0 edeg=0.000 dIE=0.000',
    ])
    para(doc, 'VOFA+：数据引擎选 FireWater，帧格式 yaw:<Target>,<Yaw>\\r\\n，100Hz；RAW=ON 追加第三通道角速度；'
              'PROTO=JUST 切换为 JustFloat（小端 float + 帧尾 00 00 80 7F）。建议 Y 轴 ±180°，X 轴 20 秒。')

    # ---------------- 9 调参与验收 ----------------
    doc.add_heading('9  调参与验收步骤', level=1)
    doc.add_heading('9.1  整定流程（实机版）', level=2)
    code(doc, [
        '1) 确认方向：DUTY=160 打 0.2s 脉冲，看 edeg 是否增大；减小就 INV=ON',
        '   （方向反着进闭环 = 正反馈，会一路加速，必须最先做）',
        '2) 测死区：DZ=0，DUTY=60/90/120/150/180 逐个试，DZ 取起转的最小值（本机 145）',
        '3) 调 P  ：KI=0 KD=0，SET=30 START；静差 ≈ 起动门限/(KP×0.86)',
        '4) 调 I  ：KI 从 0 加到 3 就能把静差压到 0.2°；再大反而超调（Ki=8 → 超调 2.7°）',
        '5) 调 D  ：KD=3~4，压接近目标时的冲过头',
        '6) 手推 30° 验证回位；LIMIT=300 后 SET=150 → SET=-150 验证最短路径',
    ])
    para(doc, '整定过程中踩到的坑，都已在代码里修掉：')
    table(doc, ['现象', '根因', '修法'], [
        ['目标一改转盘纹丝不动，随机复现', '堵转检测参考时刻只在 RUN 里更新，START 时沿用旧时刻，'
                                          'now-旧值早已超时，第一拍就 FAULT:STALL', 'ctl_start() 重新打点；超时 600ms→2s'],
        ['Ki 越大反而越飘', 'OMAX 只砍外层输出，PID 内部仍按 ±1000 判饱和，积分涨到 ±400', 'OMAX 同步写 pid->out_max/min'],
        ['闭环一上就满功率', '死区 250‰ 是 MPU 线拖住转盘时测的，实际只有 145‰', '重测死区；换机构必须重测'],
        ['反向脉冲被拒、单向累积撞限位', 'ctl_manual() 守卫没考虑 INV 取反', '按取反后的实际方向判断'],
        ['增量式照搬位置式增益会乱抖', 'Kd·Δ²e/dt 在 dt=5ms 上放大 200 倍', '增量式用 Kd=0 和自己的增益量级'],
        ['手推扰动时闭环放弃回位', '堵转检测把"手按住"当成堵转，600ms 就刹车', '超时放宽到 2s；测试前 STALL=OFF'],
    ], widths=[4.2, 6.6, 4.8])

    doc.add_heading('9.2  评分点验收对照', level=2)
    table(doc, ['评分点', '操作', '期望', '实测'], [
        ['1 电机驱动（15）', 'FWD / REV / BRK', '正转、反转、刹车三态明显不同', '通过'],
        ['2 定时器节拍（15）', '运行中敲 HELP、看 VOFA 波形', '命令即时响应、波形不平坦、无卡顿', '200Hz 节拍稳定'],
        ['3 PID（20）', '现场改 KP/KI/KD，切 MODE=INC 对比', '能说出三者对响应的影响', '见 6.3 数据'],
        ['4 锁定 + 限位（20）', 'SET=0 → START 锁 60 秒，读 STAT 的 err', '|err| ≤ 2.5°；越限刹车',
         '静差 -0.059°；越限后朝外出力被拦'],
        ['5 VOFA+（10）', '打开 VOFA+ 看两条曲线', 'Target 阶跃、Yaw 跟随', '通过'],
        ['发挥 1（8）', 'KP= 等在线修改并回显', '|err| ≤ 1.5°', '+30°: 0.13~0.43°；-30°: 0.21~0.29°'],
        ['发挥 2（7）', '手拨 30~45° 后松手', '无超调回到目标',
         '拨开 44.6~45.4°：超调 0.55~0.80°，0.26s 稳定'],
        ['发挥 3（5）', '转盘在 +20° 时 SET=-179，读 err', '只走短边',
         'err=+160.831°（朴素实现会给 -199.17°）'],
    ], widths=[3.4, 5.6, 4.4, 3.2])

    # ---------------- 10 验证记录 ----------------
    doc.add_heading('10  验证记录', level=1)
    table(doc, ['项目', '结果'], [
        ['Keil 全量编译', '0 Error, 0 Warning（ARMCC V5.06 update 5）'],
        ['程序体积', 'Code 30628 B / RO 896 B / RW 260 B / ZI 1836 B'],
        ['产物', 'MDK-ARM/Objects/Task3_YawPID.hex（89450 B）'],
        ['单元测试', '55 项断言全部通过（app_pid / app_yaw / app_util）'],
    ], widths=[4.0, 11.6])
    para(doc, '实机验证（FB=ENC、死区 145‰、OMAX=300 + 制动增强 3×、位置式 PID）：')
    table(doc, ['项目', '实测结果'], [
        ['电机正反转', 'FWD 使角度增大、REV 使角度减小，各 66°'],
        ['短路刹车', '同样脉冲突发后，刹车比滑行少走 56~58%'
                    '（350‰/0.1s：3.53° vs 8.46°；500‰/0.08s：5.73° vs 13.16°）'],
        ['死区曲线', '60/90/120‰ 不动，150‰ 起转（12.2°/0.4s），180‰≈40dps，300‰≈100dps'],
        ['编码器刻度', 'CPD = 6.933 counts/转盘度（13 线 ×4 ×48:1 = 2496 counts/rev）；'
                    '早期按 13.6 算是它的 2 倍，所有角度都差一倍'],
        ['编码器跟随', '手转 60.1°，edeg 全程同步，无打滑'],
        ['30° 阶跃', '超调 0，静差 0.66°，进入 ±1.5° 约 0.45s'],
        ['等效扰动', '目标脉冲 40°/60°：峰值偏离 16~22°，反向超调 0，0.33s 回位'],
        ['±180 最短路径', '+150°→−150°：cont 走 149→171→210，只转 42°；反向 45°，都过 ±180'],
        ['限位保护', 'LIMIT=30 时开环推过 30° → FAULT @ edeg=34.78 并刹车；超限时 CLEAR 被拒'],
        ['位置式 vs 增量式', '同条件增量式超调 3.2°（Kp=5 Ki=10 Kd=0），位置式 0.15°'],
        ['VOFA+ FireWater', '100Hz，格式 yaw:<Target>,<Yaw>，实测 yaw:146.99,16.03'],
        ['VOFA+ JustFloat', '12 字节/帧（2×float + 00 00 80 7F），49 帧连续无错位'],
        ['命令错误码', 'UNKNOWN_CMD / SYNTAX / RANGE / STATE / LIMIT 全部命中'],
    ], widths=[3.8, 11.8])
    para(doc, '单元测试覆盖：角度回绕与最短路径、Yaw 积分（去零偏/冻结/跨 ±180°）、位置式 PID 的限幅、'
              '积分限幅、条件积分抗饱和、微分先行无冲击、微分低通、增量式的 Ki 累加特性与限幅、'
              '浮点解析与 Newton 开方，以及在"死区+库仑摩擦+惯性"对象上的闭环收敛、30° 抗扰无超调。')
    para(doc, '陀螺反馈版本（v1.2.6~v1.2.8，FB=IMU、Kp=12/Ki=0.8/Kd=0、OMAX=300、ZUPT=OFF）实机实测：')
    table(doc, ['项目', '实测结果'], [
        ['60 秒锁定', '静差 -0.059 度，抖动 0.004 度，角速度峰值 0.2dps（6000 帧无一帧超 200dps）'],
        ['±30 度阶跃', '+30: 超调 0.76~3.18 度、上升 0.18~0.20s、稳定 0.44~0.67s；'
                    '-30: 超调 0.30~0.55 度、上升 0.20s、稳定 0.21s'],
        ['扰动恢复', '拨开 44.6~45.4 度后松手：超调 0.55~0.80 度、0.26s 稳定、静差 0.05~0.15 度'],
        ['±180 最短路径', '转盘在 +20.169 度时 SET=-179：STAT 回 err=+160.831（= wrap(-179-20.169)），'
                       '朴素实现会给 -199.17；SET=+179 时 err=+158.856'],
        ['FB=FUSE', '+30 超调 4.01 度/稳定 0.512s；-30 超调 0.70 度/稳定 0.190s；静差 0.27~0.33 度'],
        ['陀螺静态', '竖直轴 Z、面朝下（az=-1.02g）、倾角 2.7~4.8 度；噪声 0.052~0.066dps；标定质量 98'],
        ['陀螺动态', '手转 97.5 度（ZUPT=OFF）：陀螺/编码器比值 0.991，符号一致'],
        ['回程间隙', '换向后多出 5.58 度、同向只多 0.66 度；FB=ENC 换向后转盘实际少转 3.6 度'],
    ], widths=[3.8, 11.8])
    para(doc, '未验证：±180 的 340 度实转演示（排线限制行程，改用 err 字段验证算法）；'
              '长期（小时级）温漂；MODE=INC 在陀螺反馈下的整定。')

    # ---------------- 11 已知限制 ----------------
    doc.add_heading('11  已知限制与后续改进', level=1)
    bullets(doc, [
        '★ 静摩擦让"静态顶住"力有上限：out = Kp×误差，误差超过约 25 度就被 OMAX=300（40% 占空比）钳住不再增长，'
        '所以用手慢慢推时感觉不到力随角度变大；一松手速度上来，制动增强接管，回位力才充分（45 度回位 0.26s）。',
        '★ 回程间隙约 5 度。编码器在电机侧，换向要先空走这段间隙，FB=ENC 换向后转盘实际少转 3.6 度而编码器显示精确到位。'
        '要根治只能改机械（预紧弹簧 / 消隙齿轮 / 提高传动刚度）。',
        '★ 陀螺零偏随温度漂（几分钟内约 0.1dps/℃），必须"上电预热 2~3 分钟再 CAL"。'
        '标定后静止漂移可压到 0.0135dps（0.8 度/分）；BIASAUTO 只能在怠速时慢跟。',
        '★ ZUPT 与慢速转动冲突：判静止期间积分 dt 置 0，慢转时编码器很多拍是 0 增量、角速度又低于 1dps 门限，'
        '会被整段误判成静止——实测手转 97.5 度只积到 72%。所以 FB=IMU/FUSE 默认关 ZUPT。',
        '★ 死区强依赖机构状态（MPU 的线是否拖着转盘差一倍以上），换机构必须重测，否则闭环会"小误差也给满功率"。',
        '★ 堵转检测与"手推扰动"天然冲突：电机给力而编码器不动，两者在固件眼里一样。已把超时放宽到 2s，'
        '做手推测试前请先 STALL=OFF。',
        '增量式 PID 的限幅会丢掉部分增量信息，且 Kd/dt 在 5ms 上放大 200 倍，所以增量式必须用 Kd=0；'
        '默认仍是位置式，MODE=INC 用于现场对比讲解。',
        'Yaw 是上电后的相对角（纯陀螺积分），没有绝对基准，长时间运行会缓慢漂移；ZUPT + 怠速零偏跟踪能压住'
        '静止漂移，温漂需重新 CAL。',
        '速度内环（串级）没有启用：app_enc.c 已提供 enc_speed_cps()，需要时可把位置环输出当速度指令再加一层 PI。',
        'MPU6050 与电机同电源时，换向火花可能干扰 I2C；已要求 VM 就近加 100uF + 100nF，'
        '若仍有丢包可把 DLPF 调大或改用软件 I2C 后端对比。',
    ])

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    doc.save(OUT)
    print('生成 %s' % OUT)
    if os.path.isdir(os.path.dirname(OUT2)):
        shutil.copyfile(OUT, OUT2)
        print('同步 %s' % OUT2)


if __name__ == '__main__':
    build()
