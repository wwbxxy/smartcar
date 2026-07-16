from machine import *
from smartcar import *
from seekfree import *
import time, gc, math, os, io

from pid_module import PID_CLASS
from imu_module import IMU_FILTER
from motor_module import MOTOR
from display_module import DISPLAY, MenuItem, Menu
from ccd_module import CCD
from debug_module import DEBUG

# ===== 全局对象 =====
# 屏幕可能坏了, 初始化加容错
display = None
try:
    display = DISPLAY()
    imu_filter = IMU_FILTER(lcd=display.lcd, beep=display.beep)
except Exception as e:
    print("[WARN] 屏幕初始化失败: {}, 将使用串口调试".format(e))
    # 无屏幕时IMU单独初始化 (不依赖LCD/蜂鸣器)
    imu_filter = IMU_FILTER()
motor = MOTOR()

# ===== CCD 初始化 =====

ccd_hw = TSL1401(1)
ccd_hw.set_resolution(TSL1401.RES_8BIT)
ccd_near = CCD(ccd_hw, 0)   # CCD1接口 (主巡线)
ccd_mid  = None              # 没有第二个CCD, 置None

# ===== PID 控制器 =====
# 角度环（外环）：误差=目标角度-当前角度，输出=目标角速度(dps)
balance_angle_pid = PID_CLASS(kp=-4.0, ki=0.0, kd=-8.0,  ei_max=500, output_max=2000)
# 角速度环（内环）：误差=目标角速度-当前角速度，输出=PWM
balance_gyro_pid  = PID_CLASS(kp=-17.0,  ki=-0.5, kd=0.0,  ei_max=800, output_max=2000)
# 方向环（PD）：误差=CCD中点偏差，输出=转向PWM差值
direction_pid = PID_CLASS(kp=0.0, ki=0.0, kd=0.0, ei_max=0, output_max=2000)

# ===== 运行状态 =====
running = False          # 平衡控制是否运行
current_output = 0.0     # 当前平衡PWM（供显示）
turn_pwm = 0             # 当前转向PWM（方向环输出, ticker读取）
display_cnt = 0          # 显示刷新计数
ccd_flag = False         # CCD 10ms采集完成标志
ccd_enable = False       # 方向控制是否启用 (run模式下自动开启)

# 屏幕模式: "menu" / "status" / "run" / "ccd"
screen = "menu"

# ===== 参数系统 =====
params = {
    "T_Angle": 0.0,      # 目标角度（机械零点）
    "A_Kp":    -4.0,    # 角度环 Kp
    "A_Ki":    0.0,      # 角度环 Ki
    "A_Kd":    -8.0,     # 角度环 Kd
    "G_Kp":    -17.0,     # 角速度环 Kp
    "G_Ki":    -0.5,      # 角速度环 Ki
    "G_Kd":    0.0,      # 角速度环 Kd
    "D_Kp":    80.0,      # 方向环 Kp (CCD误差→转向PWM)
    "D_Kd":    0.0,       # 方向环 Kd
    "Turn_Max": 3000,     # 转向PWM限幅
    "CCD_ThMax": 100,      # CCD二值化阈值上限
    "CCD_ThMin": 10,      # CCD二值化阈值下限
    "CCD_Prot":  12,      # CCD保护阈值(低于此值冲出赛道)
}

TARGET_ANGLE = 0.0


# ===== 参数 ↔ PID 同步 =====
def update_pid_params():
    """将 params 同步到 PID 控制器和 TARGET_ANGLE"""
    global TARGET_ANGLE
    TARGET_ANGLE = params["T_Angle"]
    balance_angle_pid.kp = params["A_Kp"]
    balance_angle_pid.ki = params["A_Ki"]
    balance_angle_pid.kd = params["A_Kd"]
    balance_gyro_pid.kp = params["G_Kp"]
    balance_gyro_pid.ki = params["G_Ki"]
    balance_gyro_pid.kd = params["G_Kd"]
    # 方向环
    direction_pid.kp = params["D_Kp"]
    direction_pid.kd = params["D_Kd"]
    direction_pid.output_max = params["Turn_Max"]
    # CCD参数 (只更新存在的CCD)
    for c in (ccd_near, ccd_mid):
        if c is not None:
            c.threshold_max = int(params["CCD_ThMax"])
            c.threshold_min = int(params["CCD_ThMin"])
            c.protect_value = int(params["CCD_Prot"])


# ===== 动作回调 (菜单节点用) =====
def action_run():
    """启动平衡控制"""
    global screen, running, turn_pwm, ccd_enable
    balance_angle_pid.reset()
    balance_gyro_pid.reset()
    direction_pid.reset()
    turn_pwm = 0
    ccd_enable = True       # run模式下开启CCD方向控制
    running = True
    screen = "run"


def action_status():
    """进入状态查看界面 (电机不转)"""
    global screen, running
    running = False
    motor.set_duty(0, 0)
    screen = "status"


def action_save():
    try:
        os.chdir("/flash")
        with io.open("params.txt", "w") as f:
            for k, v in params.items():
                f.write("{}={}\n".format(k, v))
        display.show_message("Saved OK!", display.GREEN)
        display.beep_ok()
        time.sleep_ms(500)
    except:
        display.show_message("Save FAIL!", display.RED)
        display.beep_error()
        time.sleep_ms(500)
    menu.need_redraw = True


def action_load():
    try:
        os.chdir("/flash")
        with io.open("params.txt", "r") as f:
            for line in f:
                line = line.strip()
                if line and "=" in line and not line.startswith('#'):
                    k, v = line.split("=", 1)
                    try:
                        params[k.strip()] = float(v.strip())
                    except:
                        pass
        update_pid_params()
        display.show_message("Loaded OK!", display.GREEN)
        display.beep_ok()
        time.sleep_ms(500)
    except:
        display.show_message("No Save File", display.YELLOW)
        display.beep_error()
        time.sleep_ms(500)
    menu.need_redraw = True


def action_cal_imu():
    global running
    running = False
    motor.set_duty(0, 0)
    imu_filter.calibrate(5000)
    menu.need_redraw = True


def action_ccd():
    """进入CCD查看界面 (电机不转, 实时看CCD波形)"""
    global screen, running, ccd_enable
    running = False
    ccd_enable = False
    motor.set_duty(0, 0)
    screen = "ccd"


# ===== 菜单树定义 (改菜单只改这里) =====
menu_root = MenuItem("MAIN", children=[
    MenuItem("Run",     on_enter=action_run),
    MenuItem("Status",  on_enter=action_status),
    MenuItem("CCD",     on_enter=action_ccd),
    MenuItem("Params",  children=[
        MenuItem("T_Angle", param_key="T_Angle", step=0.1),
        MenuItem("A_Kp",    param_key="A_Kp",    step=0.5),
        MenuItem("A_Ki",    param_key="A_Ki",    step=0.1),
        MenuItem("A_Kd",    param_key="A_Kd",    step=0.5),
        MenuItem("G_Kp",    param_key="G_Kp",    step=0.5),
        MenuItem("G_Ki",    param_key="G_Ki",    step=0.1),
        MenuItem("G_Kd",    param_key="G_Kd",    step=0.5),
    ]),
    MenuItem("DirParams", children=[
        MenuItem("D_Kp",     param_key="D_Kp",     step=5.0),
        MenuItem("D_Kd",     param_key="D_Kd",     step=1.0),
        MenuItem("Turn_Max", param_key="Turn_Max", step=100),
    ]),
    MenuItem("CCDParams", children=[
        MenuItem("CCD_ThMax", param_key="CCD_ThMax", step=1.0),
        MenuItem("CCD_ThMin", param_key="CCD_ThMin", step=1.0),
        MenuItem("CCD_Prot",  param_key="CCD_Prot",  step=1.0),
    ]),
    MenuItem("Save",    on_enter=action_save),
    MenuItem("Load",    on_enter=action_load),
    MenuItem("CalIMU",  on_enter=action_cal_imu),
])

menu = Menu(display, menu_root, params)


# ===== 上电初始化 =====
# 1. 加载IMU零漂校准
if not imu_filter.load_calibration():
    imu_filter.calibrate(5000)

# 2. 加载参数（失败则用默认值）
try:
    os.chdir("/flash")
    with io.open("params.txt", "r") as f:
        for line in f:
            line = line.strip()
            if line and "=" in line and not line.startswith('#'):
                k, v = line.split("=", 1)
                try:
                    params[k.strip()] = float(v.strip())
                except:
                    pass
except:
    pass
update_pid_params()


# ===== ticker 回调（5ms 周期）=====
def pit_callback(pit_obj):
    global display_cnt, current_output

    # IMU 解算（始终运行）
    imu_filter.update()

    if running:
        # --- 角度环（外环）---
        angle_err = TARGET_ANGLE - imu_filter.angle
        target_gyro = balance_angle_pid.pid_standard_integral(angle_err)

        # --- 角速度环（内环）---
        gyro_dps = imu_filter.gyro_y_filt * imu_filter.gyro_scale
        gyro_err = target_gyro - gyro_dps
        output = balance_gyro_pid.pid_standard_integral(gyro_err)

        # 安全限幅（±10000 对应电机满量程）
        if output > 10000:
            output = 10000
        elif output < -10000:
            output = -10000

        current_output = output
        # 平衡PWM叠加转向PWM (左减右加实现差速转向, 符号需按实际接线调试)
        motor.set_duty(int(output - turn_pwm), int(output + turn_pwm))
    else:
        current_output = 0.0

    display_cnt += 1


# ===== CCD 10ms ticker 回调 =====
def ccd_callback(pit_obj):
    global ccd_flag
    ccd_flag = True


# ===== 初始化 ticker =====
# 5ms: IMU + 平衡控制 (+ 按键采集, 屏幕可用时)
pit = ticker(1)
pit.callback(pit_callback)
if display is not None:
    pit.capture_list(imu_filter.imu, display.key)
else:
    pit.capture_list(imu_filter.imu)
pit.start(5)

# 10ms: CCD自动采集 (数据处理在主循环, 避免阻塞中断)
pit_ccd = ticker(2)
pit_ccd.callback(ccd_callback)
pit_ccd.capture_list(ccd_hw)
pit_ccd.start(10)


# ===== 串口调试系统初始化 =====
debug = DEBUG(params, update_cb=update_pid_params)
debug.attach_imu(imu_filter)
debug.attach_motor(motor)
debug.attach_pid(balance_angle_pid, balance_gyro_pid, direction_pid)
debug.attach_ccd(ccd_near, ccd_mid)  # ccd_mid=None, debug内部已处理


def sync_debug_running():
    """同步debug的running状态到全局running"""
    global running, turn_pwm, ccd_enable
    if debug.running != running:
        running = debug.running
        if running:
            turn_pwm = 0
            ccd_enable = True
        else:
            turn_pwm = 0
            ccd_enable = False
            motor.set_duty(0, 0)


print("=" * 50)
print("平衡车调试系统已启动")
print("屏幕状态: {}".format("正常" if display else "不可用(串口模式)"))
print("输入 help 查看可用命令")
print("=" * 50)


# ===== 屏幕刷新辅助 =====
def refresh_status(running_flag):
    global display_cnt
    if display_cnt < 20:
        return
    display_cnt = 0
    gyro_dps = imu_filter.gyro_y_filt * imu_filter.gyro_scale
    display.show_status(
        imu_filter.angle, gyro_dps, current_output, running_flag,
        TARGET_ANGLE,
        (imu_filter.acc_x, imu_filter.acc_y, imu_filter.acc_z),
        (imu_filter.gyro_x, imu_filter.gyro_y, imu_filter.gyro_z),
        imu_filter.calibrated
    )


def handle_status_back():
    """status/run/ccd 界面按返回键"""
    global screen, running, ccd_enable, turn_pwm
    kd = display.key.get()
    if kd[3] > 0:
        display.key.clear(4)
        running = False
        ccd_enable = False
        turn_pwm = 0
        motor.set_duty(0, 0)
        screen = "menu"
        menu.need_redraw = True


# ===== CCD 颜色字典 (供 CCD.draw 使用) =====
ccd_colors = {
    'YELLOW': display.YELLOW,
    'RED':    display.RED,
    'GREEN':  display.GREEN,
    'BLUE':   display.BLUE,
    'WHITE':  display.WHITE,
}


def refresh_ccd():
    """CCD查看界面: 实时显示CCD波形与边界"""
    lcd = display.lcd
    lcd.clear(display.BLACK)
    lcd.str16(0, 0, "-- CCD VIEW --", display.CYAN)

    # CCD1 (上方)
    ccd_near.draw(lcd, 20, "CCD1", ccd_colors)

    # 第二路CCD (下方, 若存在)
    if ccd_mid is not None:
        ccd_mid.draw(lcd, 120, "CCD2", ccd_colors)

    # 底部汇总
    err = ccd_near.err
    valid = "OK" if ccd_near.is_valid() else "LOST!"
    lcd.str16(0, 230, "Err:{:3d}  {}".format(err, valid), display.YELLOW)
    lcd.str16(0, 250, "[Back] Menu", display.WHITE)


# ===== 主循环 =====
while True:
    # --- CCD 数据处理 (10ms周期, 所有模式都更新方便查看) ---
    if ccd_flag:
        ccd_flag = False
        ccd_near.update()
        if ccd_mid is not None:
            ccd_mid.update()
        # 方向控制 (仅run模式且CCD有效时)
        if ccd_enable and running:
            if ccd_near.is_valid():
                turn_pwm = direction_pid.pid_standard_integral(ccd_near.err)
            else:
                turn_pwm = 0   # 冲出赛道, 停止转向
        else:
            turn_pwm = 0

    # --- 串口调试 (始终运行) ---
    debug.process()                  # 处理命令行输入
    sync_debug_running()             # 同步running状态
    debug.set_running(running)       # 回同步状态给debug
    debug.print_status()             # 周期打印状态 (10Hz)
    debug.print_ccd()                # 周期打印CCD (5Hz, 需ccd命令开启)

    # --- 屏幕交互 (屏幕可用时) ---
    if display is not None:
        if screen == "menu":
            menu.process()
            # 参数有改动 → 同步到 PID 控制器
            if menu.params_dirty:
                update_pid_params()
                menu.params_dirty = False
        elif screen == "status":
            refresh_status(False)
            handle_status_back()
        elif screen == "run":
            refresh_status(True)
            handle_status_back()
        elif screen == "ccd":
            refresh_ccd()
            handle_status_back()

    gc.collect()
