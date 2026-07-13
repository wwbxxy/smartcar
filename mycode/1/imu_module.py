from seekfree import IMU660RX
import math


class IMU_FILTER:
    def __init__(self):
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

    def update(self):
        """在ticker回调中调用，更新IMU数据并进行角度解算"""
        data = self.imu.get()
        self.acc_x = data[0]
        self.acc_y = data[1]
        self.acc_z = data[2]
        self.gyro_x = data[3]
        self.gyro_y = data[4]
        self.gyro_z = data[5]

        # 加速度计计算角度（atan2返回弧度，转为度）
        # 注意：这里假设车模前进方向的倾角由acc_x和acc_z决定
        # 具体轴的选取取决于IMU安装方向，需要实际调试
        angle_acc_rad = math.atan2(self.acc_x, self.acc_z)
        self.angle_acc = angle_acc_rad * 180.0 / math.pi

        # 陀螺仪积分（gyro_y转dps，再乘dt得到角度增量）
        gyro_dps = self.gyro_y * self.gyro_scale
        self.angle = self.K * (self.angle + gyro_dps * self.dt) + (1.0 - self.K) * self.angle_acc