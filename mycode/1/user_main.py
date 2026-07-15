from machine import *
from smartcar import *
from seekfree import *
import time, gc, math, os, io

from pid_module import PID_CLASS
from imu_module import IMU_FILTER
from motor_module import MOTOR
from display_module import DISPLAY, MenuItem, Menu

# ===== 全局对象 =====
display = DISPLAY()
imu_filter = IMU_FILTER(lcd=display.lcd, beep=display.beep)
motor = MOTOR()

# ===== PID 控制器 =====
# 角度环（外环）：误差=目标角度-当前角度，输出=目标角速度(dps)
# ei_max 限制积分累积，防止积分饱和（外环输出限到±500 dps）
balance_angle_pid = PID_CLASS(kp=300.0, ki=0.0, kd=3.0,  ei_max=500, output_max=2000)
# 角速度环（内环）：误差=目标角速度-当前角速度，输出=PWM
# ei_max 限制积分累积（内环输出限到±800 PWM）
balance_gyro_pid  = PID_CLASS(kp=50.0,  ki=0.0, kd=0.5,  ei_max=800, output_max=2000)

# ===== 运行状态 =====
running = False          # 平衡控制是否运行
current_output = 0.0     # 当前电机输出（供主循环显示）
display_cnt = 0          # 显示刷新计数

# 屏幕模式: "menu" / "status" / "run"
screen = "menu"

# ===== 参数系统 =====
params = {
    "T_Angle": 0.0,      # 目标角度（机械零点）
    "A_Kp":    300.0,    # 角度环 Kp
    "A_Ki":    0.0,      # 角度环 Ki
    "A_Kd":    3.0,      # 角度环 Kd
    "G_Kp":    50.0,     # 角速度环 Kp
    "G_Ki":    0.0,      # 角速度环 Ki
    "G_Kd":    0.5,      # 角速度环 Kd
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


# ===== 动作回调 (菜单节点用) =====
def action_run():
    """启动平衡控制"""
    global screen, running
    balance_angle_pid.reset()
    balance_gyro_pid.reset()
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


# ===== 菜单树定义 (改菜单只改这里) =====
menu_root = MenuItem("MAIN", children=[
    MenuItem("Run",     on_enter=action_run),
    MenuItem("Status",  on_enter=action_status),
    MenuItem("Params",  children=[
        MenuItem("T_Angle", param_key="T_Angle", step=0.1),
        MenuItem("A_Kp",    param_key="A_Kp",    step=0.5),
        MenuItem("A_Ki",    param_key="A_Ki",    step=0.1),
        MenuItem("A_Kd",    param_key="A_Kd",    step=0.5),
        MenuItem("G_Kp",    param_key="G_Kp",    step=0.5),
        MenuItem("G_Ki",    param_key="G_Ki",    step=0.1),
        MenuItem("G_Kd",    param_key="G_Kd",    step=0.5),
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
        gyro_dps = imu_filter.gyro_y * imu_filter.gyro_scale
        gyro_err = target_gyro - gyro_dps
        output = balance_gyro_pid.pid_standard_integral(gyro_err)

        # 安全限幅（±10000 对应电机满量程）
        if output > 10000:
            output = 10000
        elif output < -10000:
            output = -10000

        current_output = output
        motor.set_duty(int(output), int(output))
    else:
        current_output = 0.0

    display_cnt += 1


# ===== 初始化 ticker =====
pit = ticker(1)
pit.callback(pit_callback)
pit.capture_list(imu_filter.imu, display.key)
pit.start(5)


# ===== 屏幕刷新辅助 =====
def refresh_status(running_flag):
    global display_cnt
    if display_cnt < 20:
        return
    display_cnt = 0
    gyro_dps = imu_filter.gyro_y * imu_filter.gyro_scale
    display.show_status(
        imu_filter.angle, gyro_dps, current_output, running_flag,
        TARGET_ANGLE,
        (imu_filter.acc_x, imu_filter.acc_y, imu_filter.acc_z),
        (imu_filter.gyro_x, imu_filter.gyro_y, imu_filter.gyro_z),
        imu_filter.calibrated
    )


def handle_status_back():
    """status/run 界面按返回键"""
    global screen, running
    kd = display.key.get()
    if kd[3] > 0:
        display.key.clear(4)
        running = False
        motor.set_duty(0, 0)
        screen = "menu"
        menu.need_redraw = True


# ===== 主循环 =====
while True:
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
    gc.collect()
