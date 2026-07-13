from machine import *
from smartcar import *
from seekfree import *
import time, gc, math

from pid_module import PID_CLASS
from imu_module import IMU_FILTER
from motor_module import MOTOR
from display_module import DISPLAY

# ===== 全局对象 =====
imu_filter = IMU_FILTER()
motor = MOTOR()
display = DISPLAY()

# 角度环（外环）：误差=目标角度-当前角度，输出=目标角速度
balance_angle_pid = PID_CLASS(kp=300.0, ki=0.0, kd=3.0,  ei_max=0, output_max=2000)
# 角速度环（内环）：误差=目标角速度-当前角速度，输出=PWM
balance_gyro_pid  = PID_CLASS(kp=50.0,  ki=0.0, kd=0.5,  ei_max=0, output_max=3000)
# 速度环（预留，无编码器暂不启用，kp/ki/kd 全 0）
speed_pid         = PID_CLASS(kp=0.0,   ki=0.0, kd=0.0,  ei_max=0, output_max=0)

# 目标角度（机械零点，需调试填入）
TARGET_ANGLE = 0.0

# 速度环对目标角度的修正量（预留接口，暂为 0）
speed_angle_out = 0.0

# 调试计数
display_cnt = 0

# ===== 上电 IMU 校准（车必须静止）=====
if not imu_filter.load_calibration():
    imu_filter.calibrate(5000)

# ===== ticker回调 =====
def pit_callback(pit_obj):
    global display_cnt
    # IMU解算
    imu_filter.update()

    # --- 角度环（外环）---
    angle_err = (TARGET_ANGLE - speed_angle_out) - imu_filter.angle
    target_gyro = balance_angle_pid.pid_standard_integral(angle_err)

    # --- 角速度环（内环）---
    gyro_dps = imu_filter.gyro_y * imu_filter.gyro_scale
    gyro_err = target_gyro - gyro_dps
    output = balance_gyro_pid.pid_standard_integral(gyro_err)

    # 限幅
    if output > 10000:
        output = 10000
    elif output < -10000:
        output = -10000

    # 电机输出
    motor.set_duty(int(output), int(output))

    display_cnt += 1


# ===== 初始化ticker =====
pit = ticker(1)
pit.callback(pit_callback)
pit.capture_list(imu_filter.imu)
pit.start(5)  # 5ms

# ===== 主循环（显示调试） =====
while True:
    if display_cnt >= 40:  # 每200ms更新一次显示
        display_cnt = 0
        display.show(imu_filter.angle, imu_filter.gyro_y * imu_filter.gyro_scale, 0)
    gc.collect()
