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

# 平衡环参数（需要调试）
BALANCE_KP = 3000.0
BALANCE_KD = 30.0

# 目标角度（机械零点偏移，需要调试）
TARGET_ANGLE = 0.0

# 调试计数
display_cnt = 0


# ===== ticker回调 =====
def pit_callback(pit_obj):
    global display_cnt
    # IMU解算
    imu_filter.update()

    # 平衡控制（PD）
    angle = imu_filter.angle - TARGET_ANGLE
    gyro_dps = imu_filter.gyro_y * imu_filter.gyro_scale
    output = BALANCE_KP * angle + BALANCE_KD * gyro_dps

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
