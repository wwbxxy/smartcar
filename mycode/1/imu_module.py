from seekfree import IMU660RX
import math
import os
import io
import time


class IMU_FILTER:
    def __init__(self, lcd=None, beep=None):
        self.imu = IMU660RX()
        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

        # 角度解算结果
        self.angle = 0.0  # 互补滤波角度（度）
        self.angle_acc = 0.0  # 加速度计角度（度）

        # 转换系数（需要根据实际调试确认）
        # IMU660RB: 加速度计±8g → 4096 LSB/g, 陀螺仪±2000dps → 16.384 LSB/dps
        self.gyro_scale = 1.0 / 16.384  # 原始值转dps
        self.acc_scale = 1.0 / 4096.0  # 原始值转g

        # 互补滤波系数
        self.K = 0.98  # 陀螺仪权重
        self.dt = 0.005  # 采样周期(s)，需与ticker周期一致

        # 零漂校准值
        self.gyro_offset_x = 0
        self.gyro_offset_y = 0
        self.gyro_offset_z = 0

        # 死区阈值（小于这个值视为 0）
        self.dead_zone = 8

        # 校准是否完成
        self.calibrated = False

        self.lcd = lcd
        self.beep = beep

    def load_calibration(self):
        """从文件加载零漂校准值"""
        try:
            os.chdir("/flash")
            with io.open("IMU660.txt", "r") as f:
                for line in f:
                    line = line.strip()
                    if line and '=' in line:
                        k, v = line.split('=', 1)
                        k = k.strip()
                        v = int(v.strip())
                        if k == "gx":
                            self.gyro_offset_x = v
                        elif k == "gy":
                            self.gyro_offset_y = v
                        elif k == "gz":
                            self.gyro_offset_z = v
            self.calibrated = True
            return True
        except:
            return False

    def save_calibration(self):
        """保存零漂校准值到文件"""
        try:
            os.chdir("/flash")
            with io.open("IMU660.txt", "w") as f:
                f.write("gx={}\n".format(self.gyro_offset_x))
                f.write("gy={}\n".format(self.gyro_offset_y))
                f.write("gz={}\n".format(self.gyro_offset_z))
            return True
        except:
            return False

    def calibrate(self, samples=5000):
        """上电校准：车必须静止平放"""
        if self.lcd:
            self.lcd.str16(0, 0, "Calibrating...", 0x07E0)

        sum_x = 0
        sum_y = 0
        sum_z = 0

        for i in range(samples):
            data = self.imu.get()
            sum_x += data[3]
            sum_y += data[4]
            sum_z += data[5]
            time.sleep_ms(1)

        self.gyro_offset_x = sum_x // samples
        self.gyro_offset_y = sum_y // samples
        self.gyro_offset_z = sum_z // samples

        self.save_calibration()
        self.calibrated = True

        if self.beep:
            self.beep.value(1)
            time.sleep_ms(100)
            self.beep.value(0)

        if self.lcd:
            self.lcd.str16(0, 20, "Done", 0x07E0)
            time.sleep_ms(500)

    def update(self):
        """在ticker回调中调用，更新IMU数据并进行角度解算"""
        data = self.imu.get()
        self.acc_x = data[0]
        self.acc_y = data[1]
        self.acc_z = data[2]
        self.gyro_x = data[3]
        self.gyro_y = data[4]
        self.gyro_z = data[5]

        # 减零漂
        self.gyro_x = data[3] - self.gyro_offset_x
        self.gyro_y = data[4] - self.gyro_offset_y
        self.gyro_z = data[5] - self.gyro_offset_z

        # 死区滤波
        if abs(self.gyro_x) <= self.dead_zone:
            self.gyro_x = 0
        if abs(self.gyro_y) <= self.dead_zone:
            self.gyro_y = 0
        if abs(self.gyro_z) <= self.dead_zone:
            self.gyro_z = 0

        #互补滤波

        # 加速度计计算角度（atan2返回弧度，转为度）
        # 注意：这里假设车模前进方向的倾角由acc_x和acc_z决定
        # 具体轴的选取取决于IMU安装方向，需要实际调试
        angle_acc_rad = math.atan2(self.acc_x, self.acc_z)
        self.angle_acc = angle_acc_rad * 180.0 / math.pi

        # 陀螺仪积分（gyro_y转dps，再乘dt得到角度增量）
        gyro_dps = self.gyro_y * self.gyro_scale
        self.angle = self.K * (self.angle + gyro_dps * self.dt) + (1.0 - self.K) * self.angle_acc