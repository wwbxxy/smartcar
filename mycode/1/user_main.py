from machine import *
from smartcar import *
from seekfree import *
import time, gc, math, os, io

from pid_module import PID_CLASS
from imu_module import IMU_FILTER
from motor_module import MOTOR
from display_module import DISPLAY

# ===== 全局对象 =====
display = DISPLAY()
imu_filter = IMU_FILTER(lcd=display.lcd, beep=display.beep)
motor = MOTOR()

# ===== PID 控制器 =====
# 角度环（外环）：误差=目标角度-当前角度，输出=目标角速度(dps)
balance_angle_pid = PID_CLASS(kp=300.0, ki=0.0, kd=3.0,  ei_max=0, output_max=2000)
# 角速度环（内环）：误差=目标角速度-当前角速度，输出=PWM
balance_gyro_pid  = PID_CLASS(kp=50.0,  ki=0.0, kd=0.5,  ei_max=0, output_max=3000)

# ===== 运行状态 =====
running = False          # 平衡控制是否运行
current_output = 0.0     # 当前电机输出（供主循环显示）
display_cnt = 0          # 显示刷新计数

# ===== 参数系统 =====
params = {
    "T_Angle": 0.0,      # 目标角度（机械零点）
    "A_Kp":    300.0,    # 角度环 Kp
    "A_Kd":    3.0,      # 角度环 Kd
    "G_Kp":    50.0,     # 角速度环 Kp
    "G_Kd":    0.5,      # 角速度环 Kd
}

# 参数调节步长
steps = {
    "T_Angle": 0.5,
    "A_Kp":    10.0,
    "A_Kd":    0.5,
    "G_Kp":    5.0,
    "G_Kd":    0.1,
}

# ===== 菜单系统 =====
MENU_MAIN   = 0
MENU_PARAMS = 1
MENU_STATUS = 2
MENU_RUN    = 3

menu_state = MENU_MAIN
menu_index = 0
menu_edit = False
menu_need_redraw = True

main_items  = ["Run", "Params", "Status", "Save", "Load", "CalIMU"]
param_items = ["T_Angle", "A_Kp", "A_Kd", "G_Kp", "G_Kd", "Back"]

TARGET_ANGLE = 0.0


# ===== 参数 ↔ PID 同步 =====
def update_pid_params():
    """将 params 字典同步到 PID 控制器和 TARGET_ANGLE"""
    global TARGET_ANGLE
    TARGET_ANGLE = params["T_Angle"]
    balance_angle_pid.kp = params["A_Kp"]
    balance_angle_pid.kd = params["A_Kd"]
    balance_gyro_pid.kp = params["G_Kp"]
    balance_gyro_pid.kd = params["G_Kd"]


def save_params():
    """保存参数到 /flash/params.txt"""
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


def load_params():
    """从 /flash/params.txt 加载参数"""
    global params
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
pit.start(5)  # 5ms


# ===== 菜单处理函数 =====
def process_menu():
    global menu_state, menu_index, menu_edit, menu_need_redraw
    global running

    display.key.capture()
    key_data = display.key.get()

    # ===== 主菜单 =====
    if menu_state == MENU_MAIN:
        if key_data[0] > 0:  # 上
            menu_index = (menu_index - 1) % len(main_items)
            display.key.clear(1)
            menu_need_redraw = True
        elif key_data[1] > 0:  # 下
            menu_index = (menu_index + 1) % len(main_items)
            display.key.clear(2)
            menu_need_redraw = True
        elif key_data[2] > 0:  # 确认
            display.key.clear(3)
            item = main_items[menu_index]
            if item == "Run":
                # 启动平衡控制
                balance_angle_pid.reset()
                balance_gyro_pid.reset()
                running = True
                menu_state = MENU_RUN
                menu_need_redraw = True
            elif item == "Params":
                menu_state = MENU_PARAMS
                menu_index = 0
                menu_need_redraw = True
            elif item == "Status":
                menu_state = MENU_STATUS
                menu_need_redraw = True
            elif item == "Save":
                save_params()
                menu_need_redraw = True
            elif item == "Load":
                load_params()
                menu_need_redraw = True
            elif item == "CalIMU":
                running = False
                motor.set_duty(0, 0)
                imu_filter.calibrate(5000)
                menu_need_redraw = True
        elif key_data[3] > 0:  # 返回
            display.key.clear(4)

        if menu_need_redraw:
            display.show_menu("main", menu_index, main_items)
            menu_need_redraw = False
            time.sleep_ms(50)

    # ===== 参数页面 =====
    elif menu_state == MENU_PARAMS:
        items = param_items

        if key_data[3] > 0:  # 返回 → 回主菜单
            display.key.clear(4)
            menu_state = MENU_MAIN
            menu_index = 0
            menu_edit = False
            menu_need_redraw = True
        elif menu_edit:
            param_key = items[menu_index]
            current_value = params.get(param_key, 0)
            step = steps.get(param_key, 1.0)
            if key_data[0] > 0:  # 上 → 增大
                params[param_key] = current_value + step
                display.key.clear(1)
                update_pid_params()
                menu_need_redraw = True
            elif key_data[1] > 0:  # 下 → 减小
                params[param_key] = current_value - step
                display.key.clear(2)
                update_pid_params()
                menu_need_redraw = True
            elif key_data[2] > 0:  # 确认 → 退出编辑
                menu_edit = False
                display.key.clear(3)
                menu_need_redraw = True
        else:
            if key_data[0] > 0:  # 上
                menu_index = (menu_index - 1) % len(items)
                display.key.clear(1)
                menu_need_redraw = True
            elif key_data[1] > 0:  # 下
                menu_index = (menu_index + 1) % len(items)
                display.key.clear(2)
                menu_need_redraw = True
            elif key_data[2] > 0:  # 确认
                display.key.clear(3)
                if menu_index == len(items) - 1:  # Back
                    menu_state = MENU_MAIN
                    menu_index = 0
                    menu_need_redraw = True
                else:
                    menu_edit = True
                    menu_need_redraw = True

        if menu_need_redraw:
            display.show_menu("params", menu_index, items, params, menu_edit)
            menu_need_redraw = False
            time.sleep_ms(50)

    # ===== 状态查看（电机不转）=====
    elif menu_state == MENU_STATUS:
        if key_data[3] > 0:  # 返回
            display.key.clear(4)
            menu_state = MENU_MAIN
            menu_index = 0
            menu_need_redraw = True

        if display_cnt >= 20:  # 每100ms刷新
            display_cnt = 0
            gyro_dps = imu_filter.gyro_y * imu_filter.gyro_scale
            display.show_status(
                imu_filter.angle, gyro_dps, current_output, False,
                TARGET_ANGLE,
                (imu_filter.acc_x, imu_filter.acc_y, imu_filter.acc_z),
                (imu_filter.gyro_x, imu_filter.gyro_y, imu_filter.gyro_z),
                imu_filter.calibrated
            )

    # ===== 运行界面（平衡控制中）=====
    elif menu_state == MENU_RUN:
        if key_data[3] > 0:  # 返回 → 停车
            display.key.clear(4)
            running = False
            motor.set_duty(0, 0)
            menu_state = MENU_MAIN
            menu_index = 0
            menu_need_redraw = True

        if display_cnt >= 20:  # 每100ms刷新
            display_cnt = 0
            gyro_dps = imu_filter.gyro_y * imu_filter.gyro_scale
            display.show_status(
                imu_filter.angle, gyro_dps, current_output, True,
                TARGET_ANGLE,
                (imu_filter.acc_x, imu_filter.acc_y, imu_filter.acc_z),
                (imu_filter.gyro_x, imu_filter.gyro_y, imu_filter.gyro_z),
                imu_filter.calibrated
            )


# ===== 主循环 =====
while True:
    process_menu()
    gc.collect()
