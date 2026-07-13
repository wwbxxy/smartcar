from machine import *
from smartcar import *
from seekfree import *
from display import *
import time
import io
import gc
import os


switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP)
state2 = switch2.value()
cs = Pin('B29', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
cs.high()
cs.low()
rst = Pin('B31', Pin.OUT, value=True)
dc = Pin('B5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(2)
lcd.clear(0x0000)
key = KEY_HANDLER(6)
key_data = key.get()

beep = Pin('D24', Pin.OUT, value=False)

encoder_left = encoder("C2", "C3", False)
encoder_right = encoder("C0", "C1", True)
encoder_sum = 0

imu = IMU660RX()
imu_data = imu.get()

motor_L = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 5000, duty=0, invert=False)
motor_R = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 5000, duty=0, invert=False)
motor_L.duty(0)
motor_R.duty(0)

uart = UART(5)
uart.init(460800)

# wireless = WIRELESS_UART(460800)

ccd = TSL1401(1)
ccd.set_resolution(TSL1401.RES_8BIT)

def pit_5ms(ticker):
    global pit_5ms_flag
    pit_5ms_flag = True

def pit_10ms(ticker):
    global pit_10ms_flag
    pit_10ms_flag = True

def pit_50ms(ticker):
    global pit_50ms_flag
    pit_50ms_flag = True

def pit_20ms(ticker):
    global pit_20ms_flag
    pit_20ms_flag = True

pit1 = ticker(0)
pit1.capture_list(imu,key)
pit1.callback(pit_5ms)
pit1.start(5)

pit2 = ticker(1)
pit2.capture_list(ccd)
pit2.callback(pit_10ms)
pit2.start(10)


pit3 = ticker(2)
pit3.capture_list(encoder_left, encoder_right)
pit3.callback(pit_20ms)
pit3.start(20)

pit4 = ticker(3)
pit4.capture_list()
pit4.callback(pit_50ms)
pit4.start(50)

pit_5ms_flag = True
pit_10ms_flag = True
pit_50ms_flag = True
pit_20ms_flag = True


class IMUProcessor:
    """IMU姿态解算类,负责处理IMU数据和姿态估计"""

    def __init__(self, imu):

        self.imu = imu
        self.imu_data = [0] * 9  # IMU原始数据

        # 姿态角参数
        self.gyro_ration = 4.0  # 角速度环比例系数
        self.acc_ration = 4.0  # 角度环比例系数
        self.angle_temp = 0.0  # 角度环中间变量
        self.filter_angle = 0  # 一阶滤波后的角度
        self.cycle = 0.01  # 控制周期(秒)

        # IMU校准值
        self.reset_imu_data3 = 0  # X轴角速度校准值
        self.reset_imu_data4 = 0  # Y轴角速度校准值
        self.reset_imu_data5 = 0  # Z轴角速度校准值

        # 加载IMU校准数据
        self.load_calibration()

    def update(self):
        """更新IMU数据并处理"""
        # 获取最新IMU数据
        self.imu_data = self.imu.get()

        # 应用校准
        self.calibrate_data()

        # 姿态解算
        self.first_order_filtering()

    def load_calibration(self):
        """加载IMU校准数据"""
        try:
            os.chdir("/flash")
            with io.open("IMU.txt", "r") as f:
                for line in f:
                    line = line.strip()
                    if line and '=' in line:
                        k, value = line.split('=', 1)
                        k = k.strip()
                        if k == "i3":
                            self.reset_imu_data3 = int(value)
                        elif k == "i4":
                            self.reset_imu_data4 = int(value)
                        elif k == "i5":
                            self.reset_imu_data5 = int(value)
        except:
            # 文件不存在或读取失败
            beep_NO()

    def calibrate_imu(self):
        beep_OK()
        lcd.clear(0x0000)
        lcd.str24(20, 100, "Calibrating IMU", 0x07E0)
        """校准IMU数据"""
        # 长时间静止收集数据进行校准
        self.reset_imu_data3 = 0
        self.reset_imu_data4 = 0
        self.reset_imu_data5 = 0

        # 收集10000个样本
        samples_count = 10000
        for i in range(samples_count):
            imu_data = self.imu.get()
            self.reset_imu_data3 += imu_data[3]
            self.reset_imu_data4 += imu_data[4]
            self.reset_imu_data5 += imu_data[5]
            time.sleep_ms(1)

        # 计算平均值
        self.reset_imu_data3 = (int)(self.reset_imu_data3 / samples_count)
        self.reset_imu_data4 = (int)(self.reset_imu_data4 / samples_count)
        self.reset_imu_data5 = (int)(self.reset_imu_data5 / samples_count)

        # 保存校准数据
        self.save_calibration()

    def save_calibration(self):
        """保存IMU校准数据到文件"""
        try:
            os.chdir("/flash")
            with io.open("IMU.txt", "w") as f:
                f.write(f"i3={self.reset_imu_data3}\n")
                f.write(f"i4={self.reset_imu_data4}\n")
                f.write(f"i5={self.reset_imu_data5}\n")
            beep_OK()
        except:
            beep_NO()

    def calibrate_data(self):
        """应用校准值并过滤微小噪声"""
        # 减去校准零点
        self.imu_data[3] = self.imu_data[3] - self.reset_imu_data3
        self.imu_data[4] = self.imu_data[4] - self.reset_imu_data4
        self.imu_data[5] = self.imu_data[5] - self.reset_imu_data5

        # 过滤微小噪声
        if abs(self.imu_data[3]) <= 8:
            self.imu_data[3] = 0
        if abs(self.imu_data[4]) <= 8:
            self.imu_data[4] = 0
        if abs(self.imu_data[5]) <= 8:
            self.imu_data[5] = 0

    def first_order_filtering(self):
        """角度一阶互补滤波"""
        # 加速度计角度（弧度）
        gyro_temp = self.imu_data[1] * self.acc_ration

        # 陀螺仪角速度积分
        acc_temp = (self.imu_data[3] - self.angle_temp) * self.gyro_ration

        # 互补滤波
        self.angle_temp += ((gyro_temp + acc_temp) * self.cycle)
        self.filter_angle = self.angle_temp