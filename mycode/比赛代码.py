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

imu = IMU963RX()
imu_data = imu.get()

motor_L = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 5000, duty=0, invert=False)
motor_R = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 5000, duty=0, invert=False)

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

motor_L.duty(0)
motor_R.duty(0)

raw_encoder_left = 0
raw_encoder_right = 0
kal_data_encoder_left = 0
kal_data_encoder_right = 0
motor_ALL_speed = 0

kal_data_imu_4 = 0


def beep_OK():
    beep.value(1)
    time.sleep_ms(200)
    beep.value(0)
    time.sleep_ms(200)

def beep_NO():
    for i in range(5):
        beep.value(1)
        time.sleep_ms(100)
        beep.value(0)
        time.sleep_ms(100)
    time.sleep_ms(1000)
    
class SimplePID:
    def __init__(self, P=0.0, I=0.0, D=0.0):
        # PID参数
        self.P = P
        self.I = I
        self.D = D
        
        # 错误记录
        self.error = 0.0
        self.error_last = 0.0
        self.error_sum = 0.0
        
        # 输出分量
        self.P_out = 0.0
        self.I_out = 0.0
        self.D_out = 0.0
        
        # 限制参数
        self.output_limit = None  # 输出限幅
        self.integral_limit = None  # 积分限幅
    
    # 在SimplePID类中添加此方法
    def set_params(self, P, I, D, output_limit, integral_limit):
        """设置PID参数"""
        self.P = P
        self.I = I
        self.D = D
        self.output_limit = output_limit
        self.integral_limit = integral_limit
    
     
    def calculate(self, target, current):
        """计算PID输出"""
        # 计算误差
        self.error = target - current
        
        # 计算比例项
        self.P_out = self.P * self.error
        
        # 计算积分项
        self.error_sum += self.error
        if self.integral_limit is not None:
            self.error_sum = max(-self.integral_limit, min(self.error_sum, self.integral_limit))
        self.I_out = self.I * self.error_sum
        
        # 计算微分项
        self.D_out = self.D * (self.error - self.error_last)
        
        # 更新上一次误差
        self.error_last = self.error
        
        # 计算总输出
        output = self.P_out + self.I_out + self.D_out
        
        # 输出限幅
        if self.output_limit is not None:
            output = max(-self.output_limit, min(output, self.output_limit))
            
        return output
    
    def clear(self):
        """清除状态"""
        self.error = 0.0
        self.error_last = 0.0
        self.error_sum = 0.0
        self.P_out = 0.0
        self.I_out = 0.0
        self.D_out = 0.0

class IMUProcessor:
    """IMU姿态解算类,负责处理IMU数据和姿态估计"""
    
    def __init__(self,imu):

        self.imu = imu
        self.imu_data = [0] * 9  # IMU原始数据
        
        # 姿态角参数
        self.gyro_ration = 4.0     # 角速度环比例系数
        self.acc_ration = 4.0      # 角度环比例系数
        self.angle_temp = 0.0      # 角度环中间变量
        self.filter_angle = 0      # 一阶滤波后的角度
        self.cycle = 0.01         # 控制周期(秒)

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

class ElementParser:
    """简化的元素解析器 - 只处理元素包"""
    
    def __init__(self, uart):
        self.uart = uart
        self.buffer = bytearray(10)  # 缓冲区
        self.element = 0
    
    def update(self):
        """解析新的元素数据"""
        # 快速检查是否有数据
        available = self.uart.any()
        if not available:
            return False
        
        # 读取数据
        bytes_read = self.uart.readinto(self.buffer, min(available, len(self.buffer)))
        
        # 查找有效数据包
        i = 0
        while i <= bytes_read - 5:  # 元素包长度为5
            # 检查包头和包类型
            if (self.buffer[i] == 0xAA and 
                self.buffer[i+1] == 0x02 and 
                i + 4 < bytes_read and 
                self.buffer[i+4] == 0x55):
                
                element = self.buffer[i+2]
                checksum = self.buffer[i+3]
                
                # 验证校验和
                if checksum == (0x02 ^ element):
                    self.element = element
                    return True
                
                i += 5
            else:
                i += 1
        
        return False
    
    def get_element(self):
        """获取最新的元素类型"""
        return self.element
    
    def clear_element(self):
        """清零元素值"""
        self.element = 0

class DirectionController:
    """转向控制器 - 实现 err*P1 + abs(err)*err*P2 + (err - err_last)*D1 + gyro*D2"""
    
    def __init__(self, P1=0.0, P2=0.0, D1=0.0, D2=0.0):
        self.P1 = P1  # 误差比例项
        self.P2 = P2  # 误差平方项系数
        self.D1 = D1  # 误差微分项
        self.D2 = D2  # 陀螺仪项
        self.output_limit = None
        
        # 内部状态变量
        self.err_last = 0.0  # 存储上次误差
    
    def set_params(self, P1, P2, D1, D2, output_limit=10000):
        """设置控制参数"""
        self.P1 = P1
        self.P2 = P2
        self.D1 = D1
        self.D2 = D2
        self.output_limit = output_limit
    
    def calculate(self, err, gyro):
        """
        计算转向控制输出
        err: 当前误差
        gyro: 陀螺仪数据
        """
        # 计算各项
        p1_term = err * self.P1                          # 基础比例项
        p2_term = abs(err) * err * self.P2               # 误差平方项(带符号)
        d1_term = (err - self.err_last) * self.D1        # 误差微分项
        d2_term = gyro * self.D2                         # 陀螺仪项
        
        # 总输出
        output = p1_term + p2_term + d1_term + d2_term
        
        # 更新上次误差
        self.err_last = err
        
        # 输出限幅
        if self.output_limit is not None:
            output = max(-self.output_limit, min(output, self.output_limit))
        
        return output
    
    def clear(self):
        """清除状态"""
        self.err_last = 0.0  # 重置上次误差

class KalmanFilter:
    """一维卡尔曼滤波器"""
    
    def __init__(self, Q=0.1, R=1.0):
        self.Q = Q          # 过程噪声
        self.R = R          # 测量噪声
        self.P = 1.0        # 估计误差
        self.x = 0.0        # 状态估计值

    def update(self, measurement):
        """输入测量值，返回滤波后的值"""
        # 预测步骤
        self.P += self.Q
        # 更新步骤
        K = self.P / (self.P + self.R)              # 卡尔曼增益
        self.x += K * (measurement - self.x)        # 状态更新
        self.P *= (1 - K)                           # 误差协方差更新
        return self.x

# 各个环参数全局变量
Balance_Gyro_P = 0.0
Balance_Gyro_I = 0.0
Balance_Gyro_D = 0.0
Balance_Angle_P = 0.0
Balance_Angle_I = 0.0
Balance_Angle_D = 0.0
Balance_Speed_P = 0.0
Balance_Speed_I = 0.0
Balance_Speed_D = 0.0
Direction_Err_P1 = 0.0
Direction_Err_P2 = 0.0
Direction_Err_D1 = 0.0
Direction_Gyro_D = 0.0
min_angle = 0            # 最小角度(限制速度环输出)
target_gyro = 0             # 目标陀螺仪角速度 角度环输出
target_angle = 0            # 目标角度 设定值
angle_out = 0               # 速度环的输出 加减这个值为目标角度
mid_angle = 0               # 平衡角度
target_speed = 0            # 目标速度 设定值
circle_speed_k = 0            # 速度环加速系数
target_gyro_horizontal = 0  # 目标陀螺仪角速度 水平环输出
basic_pwm = 0               # 平衡的PWM值
turn_pwm = 0                # 转向的PWM值
turn_pwm_max = 0            # 最大转向PWM值
CCD_Protect = 12
ccd0_threshold_max = 21
ccd0_threshold_min = 8
ccd1_threshold_max = 28
ccd1_threshold_min = 13
# 菜单系统全局变量
menu_page = "main"       # 当前菜单页面: main, params, status
menu_index = 0           # 当前选中项索引
menu_edit = False        # 编辑模式标志
key_data = [0, 0, 0, 0]  # 按键数据
# 参数集(从文件加载)
params = {}

# 菜单项定义
main_menu = ["Cargo", "Params", "Status", "LoadS1", "LoadS2", "LoadS3", "LoadBackUp", "SaveS1", "SaveS2", "SaveS3", "Cal"]
param_menu = [
    "T_Speed", "Mid_Angle", # 目标角度和速度
    "B_G_P", "B_G_I", "B_G_D",
    "B_A_P", "B_A_I", "B_A_D",
    "B_S_P", "B_S_I", "B_S_D",
    "D_E_P1", "D_E_P2", "D_E_D1","D_G_D2",
    "min_angle", "f_angle_out","circle_speed_k", "s_change_ccd","circle_err","r_circle_left_point","l_circle_right_point",
    "CCD_Protect", "ccd0_threshold_max", "ccd0_threshold_min", 
    "ccd1_threshold_max", "ccd1_threshold_min","turn_pwm_max","zebra_stop_time",
    "Back"
]


# 步长参数字典 - 从文件加载
step_params = {}

# 默认步长配置
default_steps = {
    "T_Speed": 25.0,
    "Mid_Angle": 100.0,
    "B_G_P": 0.1,
    "B_G_I": 0.002,
    "B_G_D": 0.01,
    "B_A_P": 0.1,
    "B_A_I": 0.01,
    "B_A_D": 0.1,
    "B_S_P": 0.1,
    "B_S_I": 0.001,
    "B_S_D": 0.05,
    "D_E_P1": 0.5,
    "D_E_P2": 0.1,
    "D_E_D1": 0.5,
    "D_G_D2": 0.1,
    "min_angle": 100.0,
    "f_angle_out": 50.0,
    "circle_speed_k": 0.05,
    "s_change_ccd": 100.0,
    "circle_err": 1,
    "r_circle_left_point":2,
    "l_circle_right_point": 2,
    "CCD_Protect": 1,
    "ccd0_threshold_max": 1,
    "ccd0_threshold_min": 1,
    "ccd1_threshold_max": 1,
    "ccd1_threshold_min": 1,
    "turn_pwm_max": 100,
    "zebra_stop_time": 2,
}

menu_need_redraw = True
first_angle_out = 0


# 声明一个变量 用于记录一开始是否到达目标速度 一旦到达 置1
speed_flag = 0
ccd0_real_data = [0] * 128  # 最近CCD原始数据
ccd1_real_data = [0] * 128  # 中CCD原始数据

ccd0_binary_data = [0] * 128  # 最近CCD二值化数据
ccd1_binary_data = [0] * 128  # 中CCD二值化数据

ccd0_max_value = 0     # 最近CCD最大值
ccd1_max_value = 0     # 中CCD最大值
ccd0_min_value = 0     # 最近CCD最小值
ccd1_min_value = 0     # 中CCD最小值
ccd0_threshold = 0     # 最近CCD二值化阈值
ccd1_threshold = 0     # 中CCD二值化阈值

ccd0_left_point = 63        # 最近CCD左边界点
ccd0_right_point = 65     # 最近CCD右边界点
ccd0_mid_point = 64        # 最近CCD中点位置
ccd0_track_width = 0       # 最近CCD赛道宽度
ccd0_left_lost = False     # 最近CCD左边界丢失标志
ccd0_right_lost = False    # 最近CCD右边界丢失标志
ccd0_left_point_last = 63        # 最近CCD左边界点
ccd0_right_point_last = 65     # 最近CCD右边界点
ccd0_mid_point_last = 64        # 最近CCD中点位置
ccd0_track_width_last = 0       # 最近CCD赛道宽度
ccd0_left_lost_last = False     # 最近CCD左边界丢失标志
ccd0_right_lost_last = False    # 最近CCD右边界丢失标志

ccd1_left_point = 63        # 中CCD左边界点
ccd1_right_point = 65     # 中CCD右边界点
ccd1_mid_point = 64        # 中CCD中点位置
ccd1_track_width = 0       # 中CCD赛道宽度
ccd1_left_lost = False     # 中CCD左边界丢失标志
ccd1_right_lost = False    # 中CCD右边界丢失标志
ccd1_left_point_last = 63        # 中CCD左边界点
ccd1_right_point_last = 65     # 中CCD右边界点
ccd1_mid_point_last = 64        # 中CCD中点位置
ccd1_track_width_last = 0       # 中CCD赛道宽度
ccd1_left_lost_last = False     # 中CCD左边界丢失标志
ccd1_right_lost_last = False    # 中CCD右边界丢失标志


ccd0_err = 0           # 最近CCD误差值
ccd1_err = 0           # 中CCD误差值

ccd_expect_mid_point = 64      # 期望中线位置

ccd0_avg_brightness = 0  # 最近CCD平均亮度
ccd1_avg_brightness = 0  # 中CCD平均亮度

ccd_inv_range = 10  # 非法的边界范围
ccd0_normal_width = 60   # 最近CCD正常宽度
ccd1_normal_width = 40   # 中CCD正常宽度

ccd0_good_mid = 0 # 距离最近的有效中点
ccd1_good_mid = 0 # 距离中间的有效中点


ccd_err_last = 0  # 上一次CCD误差值
follow_which = 0  # 
circle_err = 0
roadblock_err = 15
ccd_err = 0
element = 0 # 0普通 1圆环 ->10左圆环 ->11右圆环
openart = 0    # 红色检测标志
angle_state = 0 
ramp_start_ticks = 0  # 坡道开始时间戳
circle_flag = 0
circle_gyro_i = 0
circle_gyro_i_out = 1300
circle_gyro_i_enable = 0
r_circle_left_point = 64
l_circle_right_point = 64
ccd0_nomal_width = 45
ccd1_nomal_width = 38

def load_step_params():
    """从文件加载步长参数"""
    global step_params
    
    try:
        os.chdir("/flash")
        with io.open("steps.txt", "r") as f:
            step_params.clear()
            for line in f:
                line = line.strip()
                if line and "=" in line and not line.startswith('#'):
                    key, value_str = line.split("=", 1)
                    try:
                        step_params[key.strip()] = float(value_str.strip())
                    except:
                        pass
        
        # 补充缺失的默认步长
        for key, default_value in default_steps.items():
            if key not in step_params:
                step_params[key] = default_value
                
    except:
        # 文件不存在，使用默认步长并创建文件
        step_params = default_steps.copy()
        create_default_steps_file()

def create_default_steps_file():
    """创建默认步长配置文件"""
    try:
        os.chdir("/flash")
        with io.open("steps.txt", "w") as f:
            f.write("# 步长配置文件 - 每个参数对应的调整步长\n")
            f.write("# 格式: 参数名=步长值\n")
            f.write("# 注意 三套参数S1/S2/S3共用这一套步长配置\n\n")
            
            for param_name, step_value in default_steps.items():
                f.write(f"{param_name}={step_value}\n")
    except:
        pass

def get_step(param_name):
    """根据参数名称从步长文件返回对应的步长"""
    return step_params.get(param_name, 1.0)  # 如果找不到对应步长，默认返回1.0

def draw_menu():
    lcd.clear(0x0000)  # 黑色背景
    
    # 显示标题
    if menu_page == "main":
        lcd.str16(5, 5, "MAIN MENU", 0xFFFF)  # 使用str16显示标题
    elif menu_page == "params":
        title_color = 0xF800 if menu_edit else 0xFFFF  # 红色编辑模式标题，否则白色
        lcd.str16(5, 5, "PARAMETERS", title_color)
    else:
        lcd.str16(5, 5, menu_page.upper(), 0xFFFF)
    
    # 显示菜单项
    items = main_menu if menu_page == "main" else param_menu
    y_start = 25  # 调整起始位置，为标题留出空间
    y_step = 16   # str16的行间距，通常是16像素
    
    # 计算屏幕可以显示的最大项数（基于str16的高度）
    max_visible_items = 14  # 根据屏幕高度320和str16字体大小调整
    
    # 计算显示起始索引，确保选中项可见
    start_idx = 0
    if menu_index >= max_visible_items:
        start_idx = menu_index - max_visible_items + 1
        
    # 限制结束索引不超过项目总数
    end_idx = min(start_idx + max_visible_items, len(items))
    
    # 只显示可见范围内的菜单项
    for i in range(start_idx, end_idx):
        y = y_start + (i - start_idx) * y_step
        text_color = 0xFFE0 if i == menu_index else 0xFFFF  # 黄色选中项，否则白色
        
        if menu_page == "params" and i < len(items) - 1:  # 参数条目
            param_key = items[i]
            value = params.get(param_key, 0)
            
            # 根据参数类型选择显示格式 - 添加图像参数为整数显示
            if param_key in ["T_Speed", "Mid_Angle", "min_angle", "f_angle_out", "s_change_ccd", "enter_yuanhuan", 
                           "CCD_Protect", "ccd0_threshold_max", "ccd0_threshold_min", "ccd1_threshold_max", "ccd1_threshold_min","zebra_stop_time"]:
                format_str = "{:.0f}"
            else:
                format_str = "{:.3f}"
            
            # 显示参数名和值 - 使用str16并优化布局
            prefix = "*" if menu_edit and i == menu_index else ">" if i == menu_index else " "
            
            # 参数名称显示 - 限制长度避免遮挡
            param_display_name = param_key
            if len(param_key) > 12:  # 如果参数名过长，进行缩写
                # 对常见的长参数名进行缩写
                if param_key == "ccd0_threshold_max":
                    param_display_name = "ccd0_th_max"
                elif param_key == "ccd0_threshold_min":
                    param_display_name = "ccd0_th_min"
                elif param_key == "ccd1_threshold_max":
                    param_display_name = "ccd1_th_max"
                elif param_key == "ccd1_threshold_min":
                    param_display_name = "ccd1_th_min"
                elif param_key == "enter_yuanhuan":
                    param_display_name = "enter_circle"
                elif param_key == "s_change_ccd":
                    param_display_name = "speed_ch_ccd"
                else:
                    # 通用缩写：取前12个字符
                    param_display_name = param_key[:12]
            
            # 使用str16显示参数名（左侧）
            lcd.str16(5, y, f"{prefix}{param_display_name}", text_color)
            
            # 编辑中的值显示红色，否则显示绿色（右侧对齐）
            value_color = 0xF800 if menu_edit and i == menu_index else 0x07E0
            value_text = format_str.format(value)
            
            # 数值右对齐显示在屏幕右侧
            lcd.str16(160, y, value_text, value_color)
            
        else:
            # 显示普通菜单项 - 使用str16
            prefix = ">" if i == menu_index else " "
            
            # 主菜单项名称可能也需要处理长度
            menu_display_name = items[i]
            if len(items[i]) > 18:  # 主菜单项名称长度限制
                menu_display_name = items[i][:18]
            
            lcd.str16(5, y, f"{prefix}{menu_display_name}", text_color)

def process_menu():
    global menu_page, menu_index, menu_edit, menu_need_redraw, key_data
    key.capture()
    key_data = key.get()
    
    # 处理按键4返回功能(适用于所有菜单页面)
    if key_data[3] > 0:
        key.clear(4)
        if menu_page == "main":
            menu_index = 0
            menu_need_redraw = True
        else:
            menu_page = "main"
            menu_index = 0
            menu_edit = False
            menu_need_redraw = True
        return
    
    # 主菜单处理
    if menu_page == "main":
        items = main_menu
        if key_data[0]>0:
            menu_index = (menu_index - 1) % len(items)
            key.clear(1)
            menu_need_redraw = True
        elif key_data[1]>0:
            menu_index = (menu_index + 1) % len(items)
            key.clear(2)
            menu_need_redraw = True
        elif key_data[2]>0:
            key.clear(3)
            item = items[menu_index]
            if item == "Cargo":
                start_cargo()
                menu_need_redraw = True
            elif item == "Params":
                menu_page = "params"
                menu_index = 0
                menu_need_redraw = True
            elif item == "Status":
                show_status()
                menu_index = 0
                menu_need_redraw = True
            elif item == "LoadS1":
                load_save_file("S1.txt")
                update_current_controllers()
                menu_need_redraw = True
            elif item == "LoadS2":
                load_save_file("S2.txt")
                update_current_controllers()
                menu_need_redraw = True
            elif item == "LoadS3":
                load_save_file("S3.txt")
                update_current_controllers()
                menu_need_redraw = True
            elif item == "SaveS1":
                save_to_file("S1.txt")
                menu_need_redraw = True
            elif item == "SaveS2":
                save_to_file("S2.txt")
                menu_need_redraw = True
            elif item == "SaveS3":
                save_to_file("S3.txt")
                menu_need_redraw = True
            elif item == "LoadBackUp":
                load_save_file("BackUp.txt")
                update_current_controllers()
                menu_need_redraw = True
            elif item == "Cal":
                imu_processor.calibrate_imu()
                menu_need_redraw = True

            
            # elif item == "Gyro_I":
            #     get_circle_gyro_i()
            #     menu_need_redraw = True



                
    # 参数页面处理
    elif menu_page == "params":
        items = param_menu
        if menu_edit:
            if menu_index < len(items) - 1:
                param_key = items[menu_index]
                current_value = params.get(param_key, 0)
                step = get_step(param_key)
                if key_data[0]>0:
                    params[param_key] = current_value + step
                    key.clear(1)
                    menu_need_redraw = True
                elif key_data[1]>0:
                    if items[menu_index] not in ["T_Speed", "Mid_Angle","min_angle","f_angle_out"]:
                        params[param_key] = max(0, current_value - step)
                    else:
                        params[param_key] = current_value - step
                    key.clear(2)
                    menu_need_redraw = True
                elif key_data[2]>0:
                    menu_edit = False
                    key.clear(3)
                    update_current_controllers()
                    menu_need_redraw = True
            else:
                menu_edit = False
                key.clear(3)
                menu_need_redraw = True
        else:
            if key_data[0]>0:
                menu_index = (menu_index - 1) % len(items)
                key.clear(1)
                menu_need_redraw = True
            elif key_data[1]>0:
                menu_index = (menu_index + 1) % len(items)
                key.clear(2)
                menu_need_redraw = True
            elif key_data[2]>0:
                key.clear(3)
                if menu_index == len(items) - 1:
                    menu_page = "main"
                    menu_index = 0
                    menu_need_redraw = True
                else:
                    menu_edit = True
                    menu_need_redraw = True
    
    # 状态页面处理
    elif menu_page == "status":
        if Key_Click():
            menu_page = "main"
            menu_index = 0
            key.clear(1)
            key.clear(2)
            key.clear(3)
            key.clear(4)
            menu_need_redraw = True

    if menu_need_redraw:
        draw_menu()
        menu_need_redraw = False
        time.sleep_ms(100)
        gc.collect()

# def update_controllers(sta):
#     global target_angle, target_speed, mid_angle
#     global Balance_Gyro_P, Balance_Gyro_I, Balance_Gyro_D
#     global Balance_Angle_P, Balance_Angle_I, Balance_Angle_D
#     global Direction_Err_P1, Direction_Err_P2, Direction_Err_D1
#     global Balance_Speed_P, Balance_Speed_I, Balance_Speed_D, min_angle, first_angle_out, speed_change_ccd, enter_yuanhuan
#     # 更新目标值
#     target_speed = params.get("T_Speed", 0)
#     mid_angle = params.get("Mid_Angle", 0)
#     # 更新PID参数
#     Balance_Gyro_P = params.get("B_G_P", 0.0)
#     Balance_Gyro_I = params.get("B_G_I", 0.0)
#     Balance_Gyro_D = params.get("B_G_D", 0.0)
#     Balance_Angle_P = params.get("B_A_P", 0.0)
#     Balance_Angle_I = params.get("B_A_I", 0.0)
#     Balance_Angle_D = params.get("B_A_D", 0.0)
#     Balance_Speed_P = params.get("B_S_P", 0.0)
#     Balance_Speed_I = params.get("B_S_I", 0.0)
#     Balance_Speed_D = params.get("B_S_D", 0.0)
#     Direction_Err_P1 = params.get("D_E_P1", 0.0)
#     Direction_Err_P2 = params.get("D_E_P2", 0.0)
#     Direction_Err_D1 = params.get("D_E_D1", 0.0)
#     min_angle = params.get("min_angle", 0)
#     first_angle_out = params.get("f_angle_out", 500)
#     speed_change_ccd = params.get("s_change_ccd", 2000)
#     enter_yuanhuan = params.get("enter_yuanhuan", 0)
#     # 更新控制器
#     balance_gyro_controller.set_params(Balance_Gyro_P, Balance_Gyro_I, Balance_Gyro_D, 10000, 10000)
#     balance_angle_controller.set_params(Balance_Angle_P, Balance_Angle_I, Balance_Angle_D, 10000, 10000)
#     speed_controller.set_params(Balance_Speed_P, Balance_Speed_I, Balance_Speed_D, 10000, 10000)

def save_to_file(filename):
    try:
        os.chdir("/flash")
        with io.open(filename, "w") as f:
            # 按照param_menu的顺序保存，方便查看
            for param_name in param_menu:
                if param_name != "Back":
                    value = params.get(param_name, 0)
                    f.write(f"{param_name}={value}\n")
        beep_OK()
    except:
        beep_NO()

def load_save_file(filename):
    global params
    
    try:
        os.chdir("/flash")
        with io.open(filename, "r") as f:
            for line in f:
                if "=" in line and not line.startswith('#'):
                    key, value_str = line.split("=", 1)
                    try:
                        params[key.strip()] = float(value_str.strip())
                    except:
                        beep_NO()
        beep_OK()
    except:
        # 文件不存在，创建默认参数
        create_default_params()
        save_to_file(filename)
        beep_NO()

def create_default_params():
    global params
    params.clear()
    for param_name in param_menu:
        if param_name != "Back":
            # 设置图像参数的默认值
            if param_name == "CCD_Protect":
                params[param_name] = 12
            elif param_name == "ccd0_threshold_max":
                params[param_name] = 21
            elif param_name == "ccd0_threshold_min":
                params[param_name] = 8
            elif param_name == "ccd1_threshold_max":
                params[param_name] = 28
            elif param_name == "ccd1_threshold_min":
                params[param_name] = 13
            elif param_name == "zebra_stop_time":
                params[param_name] = 2
            else:
                params[param_name] = 0

def init_params():
    load_step_params()        # 加载步长参数
    load_save_file("BackUp.txt")  # 上电默认读取BackUp.txt
    update_current_controllers()

def update_current_controllers():
    global target_angle, target_speed, mid_angle
    global Balance_Gyro_P, Balance_Gyro_I, Balance_Gyro_D
    global Balance_Angle_P, Balance_Angle_I, Balance_Angle_D
    global Direction_Err_P1, Direction_Err_P2, Direction_Err_D1
    global Balance_Speed_P, Balance_Speed_I, Balance_Speed_D, min_angle, first_angle_out, circle_gyro_i_out
    global CCD_Protect, ccd0_threshold_max, ccd0_threshold_min, ccd1_threshold_max, ccd1_threshold_min
    global Direction_Gyro_D,zebra_stop_time,circle_err, turn_pwm_max
    global circle_speed_k,r_circle_left_point,l_circle_right_point

    # 更新目标值
    target_speed = params.get("T_Speed", 0)
    mid_angle = params.get("Mid_Angle", 0)
    Balance_Gyro_P = params.get("B_G_P", 0.0)
    Balance_Gyro_I = params.get("B_G_I", 0.0)
    Balance_Gyro_D = params.get("B_G_D", 0.0)
    Balance_Angle_P = params.get("B_A_P", 0.0)
    Balance_Angle_I = params.get("B_A_I", 0.0)
    Balance_Angle_D = params.get("B_A_D", 0.0)
    Balance_Speed_P = params.get("B_S_P", 0.0)
    Balance_Speed_I = params.get("B_S_I", 0.0)
    Balance_Speed_D = params.get("B_S_D", 0.0)
    Direction_Err_P1 = params.get("D_E_P1", 0.0)
    Direction_Err_P2 = params.get("D_E_P2", 0.0)
    Direction_Err_D1 = params.get("D_E_D1", 0.0)
    Direction_Gyro_D = params.get("D_G_D2", 0.0)
    min_angle = params.get("min_angle", 0)
    first_angle_out = params.get("f_angle_out", 500)
    circle_speed_k = params.get("circle_speed_k", 1.2)
    circle_err = params.get("circle_err", 10)
    r_circle_left_point = params.get("r_circle_left_point", 64)
    l_circle_right_point = params.get("l_circle_right_point", 64)
    # circle_gyro_i_out = params.get("circle_gyro_i_out", 0)
    CCD_Protect = int(params.get("CCD_Protect", 12))
    ccd0_threshold_max = int(params.get("ccd0_threshold_max", 40))
    ccd0_threshold_min = int(params.get("ccd0_threshold_min", 10))
    ccd1_threshold_max = int(params.get("ccd1_threshold_max", 28))
    ccd1_threshold_min = int(params.get("ccd1_threshold_min", 13))
    zebra_stop_time = int(params.get("zebra_stop_time", 2))
    turn_pwm_max = int(params.get("turn_pwm_max", 4000))
    balance_gyro_controller.set_params(Balance_Gyro_P, Balance_Gyro_I, Balance_Gyro_D, 10000, 10000)
    balance_angle_controller.set_params(Balance_Angle_P, Balance_Angle_I, Balance_Angle_D, 10000, 10000)
    speed_controller.set_params(Balance_Speed_P, Balance_Speed_I, Balance_Speed_D, 10000, 10000)
    direction_controller.set_params(Direction_Err_P1, Direction_Err_P2, Direction_Err_D1, Direction_Gyro_D, 10000)

def save_params(filename):
    try:
        with io.open(f"/flash/{filename}", "w") as f:
            for key, value in params.items():
                f.write(f"{key}={value}\n")
        beep_OK()
    except:
        beep_NO()

def load_params(filename):
    global params
    params.clear()  # 清空当前参数
    
    try:
        with io.open(f"/flash/{filename}", "r") as f:
            for line in f:
                if "=" in line and not line.startswith('#'):
                    key, value_str = line.split("=", 1)
                    try:
                        params[key.strip()] = float(value_str.strip())
                    except:
                        beep_NO()
        # beep_OK()
    except:
        beep_NO()

def Key_Click():
    global key_data
    key_data = key.get()
    return key_data[0] or key_data[1] or key_data[2] or key_data[3]

def SetMotor(right_speed,left_speed):
    left_speed = 8000 if left_speed > 8000 else left_speed
    right_speed = 8000 if right_speed > 8000 else right_speed
    left_speed = -8000 if left_speed < -8000 else left_speed
    right_speed = -8000 if right_speed < -8000 else right_speed
    motor_L.duty(-left_speed)
    motor_R.duty(-right_speed)

def reset_car_state():
    """重置所有系统状态和标志位，恢复到默认状态"""
    # === CCD数据相关变量 ===
    global ccd0_real_data, ccd1_real_data
    global ccd0_binary_data, ccd1_binary_data
    global ccd0_max_value, ccd1_max_value
    global ccd0_min_value, ccd1_min_value
    global ccd0_threshold, ccd1_threshold
    global ccd0_left_point, ccd0_right_point, ccd0_mid_point, ccd0_track_width
    global ccd1_left_point, ccd1_right_point, ccd1_mid_point, ccd1_track_width
    global ccd0_left_lost, ccd0_right_lost, ccd1_left_lost, ccd1_right_lost
    global ccd0_left_point_last, ccd0_right_point_last, ccd0_mid_point_last
    global ccd1_left_point_last, ccd1_right_point_last, ccd1_mid_point_last
    global ccd0_track_width_last, ccd1_track_width_last
    global ccd0_left_lost_last, ccd0_right_lost_last
    global ccd1_left_lost_last, ccd1_right_lost_last
    global ccd0_err, ccd1_err, ccd_err
    global ccd0_avg_brightness, ccd1_avg_brightness
    # === 元素识别相关变量 ===
    global element, circle_flag_inner, circle_flag_outer, circle_direction
    global count_time_left, count_time_right
    global ccd_circle_err_cnt, ccd_circle_err_ave, ccd_circle_err_sum
    global err_ave, err_sum, err_cnt, circle_flag, follow_which
    # === 编码器和运动控制 ===
    global  encoder_sum_circle, encoder_sum
    global basic_pwm, turn_pwm, speed_flag
    global target_angle, target_speed, target_gyro, target_gyro_horizontal
    global angle_out, mid_angle
    # === 定时器标志 ===
    global pit_5ms_flag, pit_10ms_flag, pit_50ms_flag, pit_20ms_flag
    global zebra_pass_time, zebra_detected_time, zebra_is_standby

    # 重置斑马线相关变量
    zebra_pass_time = 0
    zebra_detected_time = 0
    zebra_is_standby = False

    # 重置CCD相关数据
    for i in range(128):
        ccd0_real_data[i] = 0
        ccd1_real_data[i] = 0

        ccd0_binary_data[i] = 0
        ccd1_binary_data[i] = 0

    
    # 重置CCD阈值和最大最小值
    ccd0_max_value = 0
    ccd1_max_value = 0

    ccd0_min_value = 0
    ccd1_min_value = 0

    ccd0_threshold = 0
    ccd1_threshold = 0

    ccd0_avg_brightness = 0
    ccd1_avg_brightness = 0

    
    # 重置CCD边界和状态信息
    ccd0_left_point = 63
    ccd0_right_point = 65
    ccd0_mid_point = 64
    ccd1_left_point = 63
    ccd1_right_point = 65
    ccd1_mid_point = 64

    ccd0_track_width = 0
    ccd1_track_width = 0

    
    ccd0_left_lost = False
    ccd0_right_lost = False
    ccd1_left_lost = False
    ccd1_right_lost = False
    
    # 重置上一次的CCD数据
    ccd0_left_point_last = 63
    ccd0_right_point_last = 65
    ccd0_mid_point_last = 64
    ccd1_left_point_last = 63
    ccd1_right_point_last = 65
    ccd1_mid_point_last = 64

    ccd0_track_width_last = 0
    ccd1_track_width_last = 0
    
    ccd0_left_lost_last = False
    ccd0_right_lost_last = False
    ccd1_left_lost_last = False
    ccd1_right_lost_last = False

    # 重置误差值
    ccd0_err = 0
    ccd1_err = 0

    ccd_err = 0
    ccd_circle_err_sum = 0  # 圆环误差和

    
    # 重置元素识别相关变量
    element = 0
    circle_flag_inner = 0
    circle_flag_outer = 0
    circle_direction = 0
    count_time_left = 0
    count_time_right = 0
    circle_flag = 0
    # 重置圆环误差相关变量
    ccd_circle_err_cnt = 0
    ccd_circle_err_ave = 0
    ccd_circle_err_sum = 0

    err_ave = 0
    err_sum = 0
    err_cnt = 0
    circle_flag = 0
    follow_which = 0

    # 重置编码器相关变量
    encoder_sum_circle = 0
    encoder_sum = 0

    # 重置电机和运行标志
    basic_pwm = 0
    turn_pwm = 0
    speed_flag = 0
    
    # 重置目标值
    target_angle = mid_angle
    target_gyro = 0
    target_gyro_horizontal = 0
    angle_out = 0

    element_parser.update()
    element_parser.clear_element()


    # 清除PID控制器状态
    balance_gyro_controller.clear()
    balance_angle_controller.clear()
    direction_controller.clear()
    speed_controller.clear()
    
    # 重置定时器标志位
    pit_5ms_flag = True
    pit_10ms_flag = True
    pit_50ms_flag = True
    pit_20ms_flag = True
    
    # 停止电机
    motor_L.duty(0)
    motor_R.duty(0)
    
    # 重置其他可能的状态变量
    beep.value(0)  # 确保蜂鸣器关闭

    # 更新屏幕，给用户反馈
    lcd.clear(0x0000)
    lcd.str24(40, 120, "System Reset", 0x07E0)
    time.sleep_ms(100)

def show_status():
    """CCD data visualization interface"""
    global element,circle_flag,follow_which,openart
    # Color definitions
    WHITE = 0xFFFF
    RED = 0xF800
    GREEN = 0x07E0
    BLUE = 0x001F
    YELLOW = 0xFFE0
    CYAN = 0x07FF
    # Clear screen
    lcd.clear(0x0000)
    reset_car_state()
    global key_data
    key_data = key.get()
    while key_data[3] == 0:
        beep.value(0)  # 确保蜂鸣器关闭
        imu_processor.update()
        element_parser.update()
        openart = element_parser.get_element()                                                # 获取元素识别数据
        # print(f"{openart}")
        update_ccd()
        update_circle()
        update_roadblock()
        update_err()
        check_zebra()
        # update_err()
        # check_element()
        # update_err()
        key_data = key.get()
        # if key_data[0] >= 1:
        #     time.sleep_ms(500)
        #     if encoder_start_flag:
        #         print("Encoder stopped")
        #         print("Total encoder", encoder_sum_all)
        #         encoder_sum = 0
        #         encoder_sum_all = 0
        #         encoder_start_flag = False
        #     else:
        #         encoder_start_flag = True
        #         print("Encoder started")

        # if encoder_start_flag:
        #     encoder_sum = encoder_left.get() + encoder_right.get() + (int)(8* (imu_processor.imu_data[3])/100)
        #     encoder_sum_all += encoder_sum

        # # Update data
        # encoder_sum = encoder_left.get() + encoder_right.get() + (int)(8* (imu_processor.imu_data[3])/100)

        # === HEADER SECTION ===
        lcd.str16(0, 0, f"Angle:{(int)(-imu_processor.filter_angle):5d}", CYAN)

        # === EMCODER SECTION ===
        lcd.str16(0, 40, f"Encoder_L:{(int)(encoder_left.get()):5d}", CYAN)
        lcd.str16(0, 25, f"Encoder_R:{(int)(encoder_right.get()):5d}", CYAN)

        # === CCD0 (NEAR) SECTION ===
        y_pos = 180
        lcd.str16(0, y_pos, "CCD0-NEAR:", YELLOW)
        lcd.wave(0, y_pos+16, 128, 40, ccd0_real_data, max=255)
        
        # Boundary lines
        lcd.line(ccd0_left_point, y_pos+16, ccd0_left_point, y_pos+56, color=RED, thick=2)     # Left edge
        lcd.line(ccd0_right_point, y_pos+16, ccd0_right_point, y_pos+56, color=GREEN, thick=2) # Right edge
        lcd.line(ccd0_mid_point, y_pos+16, ccd0_mid_point, y_pos+56, color=BLUE, thick=1)      # Mid point
        lcd.line(ccd_expect_mid_point, y_pos+16, ccd_expect_mid_point, y_pos+56, color=YELLOW, thick=1) # Expected
        
        # Parameters (right side)
        lcd.str16(130, y_pos, f"Err{ccd0_err:2d}", WHITE)
        lcd.str16(130, y_pos+15, f"W:{ccd0_track_width:3d}", WHITE)
        lcd.str16(130, y_pos+30, f"Th:{ccd0_threshold:3d}", WHITE)
        lcd.str16(130, y_pos+45, f"Avg:{ccd0_avg_brightness:2d}", WHITE)
        lcd.str16(180, y_pos, f"Max:{ccd0_max_value:3d}", WHITE)
        lcd.str16(180, y_pos+15, f"Min:{ccd0_min_value:3d}", WHITE)
        lcd.str16(180, y_pos+30, f"LEFT{ccd0_left_point:3d}", WHITE)
        lcd.str16(180, y_pos+45, f"RIGH{ccd0_right_point:3d}", WHITE)
        # Edge status
        status0 = "L:" + ("LOST" if ccd0_left_lost else "OK  ")
        status1 = "R:" + ("LOST" if ccd0_right_lost else "OK  ")
        lcd.str16(0, y_pos+58, status0, RED if ccd0_left_lost else GREEN)
        lcd.str16(64, y_pos+58, status1, RED if ccd0_right_lost else GREEN)
        
        # === CCD1 (MID) SECTION ===
        y_pos = 100
        lcd.str16(0, y_pos, "CCD1-MID:", YELLOW)
        lcd.wave(0, y_pos+16, 128, 40, ccd1_real_data, max=255)
        
        # Boundary lines
        lcd.line(ccd1_left_point, y_pos+16, ccd1_left_point, y_pos+56, color=RED, thick=2)     # Left edge
        lcd.line(ccd1_right_point, y_pos+16, ccd1_right_point, y_pos+56, color=GREEN,thick= 2) # Right edge
        lcd.line(ccd1_mid_point, y_pos+16, ccd1_mid_point, y_pos+56,color= BLUE, thick=1)      # Mid point
        lcd.line(ccd_expect_mid_point, y_pos+16, ccd_expect_mid_point, y_pos+56, color=YELLOW,thick= 1) # Expected
        # Parameters (right side)
        lcd.str16(130, y_pos, f"Err{ccd1_err:2d}", WHITE)
        lcd.str16(130, y_pos+15, f"W:{ccd1_track_width:3d}", WHITE)
        lcd.str16(130, y_pos+30, f"Th:{ccd1_threshold:3d}", WHITE)
        lcd.str16(130, y_pos+45, f"Avg:{ccd1_avg_brightness:2d}", WHITE)
        lcd.str16(180, y_pos, f"Max:{ccd1_max_value:3d}", WHITE)
        lcd.str16(180, y_pos+15, f"Min:{ccd1_min_value:3d}", WHITE)
        lcd.str16(180, y_pos+30, f"LEFT{ccd1_left_point:3d}", WHITE)
        lcd.str16(180, y_pos+45, f"RIGH{ccd1_right_point:3d}", WHITE)
        # Edge status
        status0 = "L:" + ("LOST" if ccd1_left_lost else "OK  ")
        status1 = "R:" + ("LOST" if ccd1_right_lost else "OK  ")
        lcd.str16(0, y_pos+58, status0, RED if ccd1_left_lost else GREEN)
        lcd.str16(64, y_pos+58, status1, RED if ccd1_right_lost else GREEN)
        
        y_pos = 20
        # lcd.str16(0, y_pos+45, f"CNT:{zebra_count}", WHITE)
        lcd.str16(100, y_pos+45, f"Zebra:{zebra_pass_time}", WHITE)
        # === CCD2 (FAR) SECTION ===
        # y_pos = 20
        # lcd.str16(0, y_pos, "CCD2-FAR:", YELLOW)
        # lcd.wave(0, y_pos+16, 128, 40, ccd2_real_data, max=60)
        
        # # Boundary lines
        # lcd.line(ccd2_left_point, y_pos+16, ccd2_left_point, y_pos+56, color=RED, thick=2)     # Left edge
        # lcd.line(ccd2_right_point, y_pos+16, ccd2_right_point, y_pos+56, color=GREEN, thick=2) # Right edge
        # lcd.line(ccd2_mid_point, y_pos+16, ccd2_mid_point, y_pos+56, color=BLUE, thick=1)      # Mid point
        # lcd.line(ccd_expect_mid_point, y_pos+16, ccd_expect_mid_point, y_pos+56, color=YELLOW, thick=1) # Expected
        
        # # Parameters (right side)
        # lcd.str16(130, y_pos, f"Err:{ccd2_err:2d}", WHITE)
        # lcd.str16(130, y_pos+15, f"W:{ccd2_track_width:3d}", WHITE)
        # lcd.str16(130, y_pos+30, f"Th:{ccd2_threshold:3d}", WHITE)
        # lcd.str16(130, y_pos+45, f"Avg:{ccd2_avg_brightness:2d}", WHITE)
        # lcd.str16(180, y_pos, f"Max:{ccd2_max_value:3d}", WHITE)
        # lcd.str16(180, y_pos+15, f"Min:{ccd2_min_value:3d}", WHITE)
        # lcd.str16(180, y_pos+30, f"LEFT{ccd2_left_point:3d}", WHITE)
        # lcd.str16(180, y_pos+45, f"RIGH{ccd2_right_point:3d}", WHITE)

        # # Edge status
        # status0 = "L:" + ("LOST" if ccd2_left_lost else "OK  ")
        # status1 = "R:" + ("LOST" if ccd2_right_lost else "OK  ")
        # lcd.str16(0, y_pos+58, status0, RED if ccd2_left_lost else GREEN)
        # lcd.str16(64, y_pos+58, status1, RED if ccd2_right_lost else GREEN)
        
        # === FOOTER STATISTICS ===
        y_pos = 250
        # Display CCD_ERR
        lcd.str16(0, y_pos, f"CCD_ERR:{(int)(ccd_err):4d}", WHITE)
        # Display element status
        if openart == 0:
            lcd.str16(0, y_pos+15, "Non        ", WHITE)
        elif element == 1:
            lcd.str16(0, y_pos+15, "Red        ", RED)
        elif element == 2:
            lcd.str16(0, y_pos+15, "Green      ", GREEN)
        elif element == 3:
            lcd.str16(0, y_pos+15, "Yellow Left", YELLOW)
        elif element == 4:
            lcd.str16(0, y_pos+15, "Yellow Right", YELLOW)
        # 显示 circle_flag 和 follow_which
        lcd.str16(0, y_pos+30, f"Cir:{circle_flag}", WHITE)
        lcd.str16(80, y_pos+30, f"Fol:{follow_which}", WHITE)
        # # Display circle_flag_inner and circle_flag_outer
        # circle_status = "Cir_Inner"+str(circle_flag_inner) + " Cir_Outer" + str(circle_flag_outer)
        # lcd.str16(0, y_pos+30, circle_status, WHITE)
        # print(f"Element: {element}","CCD0_Left_Lost:", ccd0_left_lost, "CCD0_Right_Lost:", ccd0_right_lost)
        


        # Memory management
        gc.collect()
    
    # Exit beep
    beep_OK()

def get_speed():
    """获取电机速度"""
    global raw_encoder_left, raw_encoder_right, motor_ALL_speed, kal_data_encoder_left, kal_data_encoder_right
    raw_encoder_left = encoder_left.get()
    raw_encoder_right = encoder_right.get()
    kal_data_encoder_left = (int)(kal_encoder_left.update(raw_encoder_left))
    kal_data_encoder_right = (int)(kal_encoder_right.update(raw_encoder_right))
    motor_ALL_speed = (int)((kal_data_encoder_left + kal_data_encoder_right) / 2)

def update_ccd_last():
    global ccd0_left_point_last, ccd1_left_point_last, ccd0_right_point_last, ccd1_right_point_last, ccd0_mid_point_last, ccd1_mid_point_last, ccd0_track_width_last, ccd1_track_width_last, ccd0_left_lost_last, ccd1_left_lost_last, ccd0_right_lost_last, ccd1_right_lost_last
    global ccd0_left_point, ccd1_left_point, ccd0_right_point, ccd1_right_point, ccd0_mid_point, ccd1_mid_point, ccd0_track_width, ccd1_track_width, ccd0_left_lost, ccd1_left_lost, ccd0_right_lost, ccd1_right_lost
    # 更新上次的CCD数据
    ccd0_left_point_last = ccd0_left_point
    ccd1_left_point_last = ccd1_left_point
    ccd0_right_point_last = ccd0_right_point
    ccd1_right_point_last = ccd1_right_point
    ccd0_mid_point_last = ccd0_mid_point
    ccd1_mid_point_last = ccd1_mid_point
    ccd0_track_width_last = ccd0_track_width
    ccd1_track_width_last = ccd1_track_width
    ccd0_left_lost_last = ccd0_left_lost
    ccd1_left_lost_last = ccd1_left_lost
    ccd0_right_lost_last = ccd0_right_lost
    ccd1_right_lost_last = ccd1_right_lost

def update_ccd():
    """更新CCD数据 - 内存优化版"""
    global ccd0_real_data, ccd1_real_data, ccd0_binary_data, ccd1_binary_data
    global ccd0_max_value, ccd1_max_value, ccd0_min_value, ccd1_min_value
    global ccd0_threshold, ccd1_threshold, ccd0_left_point, ccd1_left_point
    global ccd0_right_point, ccd1_right_point, ccd0_mid_point, ccd1_mid_point
    global ccd0_track_width, ccd1_track_width, ccd0_left_lost, ccd1_left_lost
    global ccd0_right_lost, ccd1_right_lost, ccd0_avg_brightness, ccd1_avg_brightness
    global ccd0_err, ccd1_err, ccd0_left_point_last, ccd1_left_point_last
    global ccd0_right_point_last, ccd1_right_point_last, ccd0_mid_point_last, ccd1_mid_point_last
    global ccd0_track_width_last, ccd1_track_width_last, ccd0_good_mid, ccd1_good_mid
    global ccd0_threshold_max, ccd0_threshold_min, ccd1_threshold_max, ccd1_threshold_min

    update_ccd_last()
    
    # 获取CCD数据
    ccd0_real_data = ccd.get(2)
    ccd1_real_data = ccd.get(1)
    
    # 检查数据有效性
    if not ccd0_real_data or len(ccd0_real_data) != 128:
        ccd0_real_data = [0] * 128
    if not ccd1_real_data or len(ccd1_real_data) != 128:
        ccd1_real_data = [0] * 128
    
    # 计算平均亮度
    ccd0_avg_brightness = sum(ccd0_real_data) // 128
    ccd1_avg_brightness = sum(ccd1_real_data) // 128
    
    # 优化最大最小值计算 - 避免sorted()
    # 使用简单遍历方法
    ccd0_max_value = max(ccd0_real_data)
    ccd0_min_value = min(ccd0_real_data)
    ccd1_max_value = max(ccd1_real_data)
    ccd1_min_value = min(ccd1_real_data)
    
    # 如果需要更精确的最大最小值，可以用这种方法：
    # 找到倒数第5大和第5小的值，但不创建新列表
    # ccd0_values = []
    # ccd1_values = []
    # for val in ccd0_real_data:
    #     if len(ccd0_values) < 10:
    #         ccd0_values.append(val)
    #     else:
    #         if val > min(ccd0_values):
    #             ccd0_values.remove(min(ccd0_values))
    #             ccd0_values.append(val)
    
    # 计算二值化阈值
    if (ccd0_max_value + 2 * ccd0_min_value) > 0:
        raw_threshold = (ccd0_max_value + 2 * ccd0_min_value) // 3
        ccd0_threshold = max(ccd0_threshold_min, min(raw_threshold, ccd0_threshold_max))
    else:
        ccd0_threshold = ccd0_threshold_min
        
    if (ccd1_max_value + 2 * ccd1_min_value) > 0:
        raw_threshold = (ccd1_max_value + 2 * ccd1_min_value) // 3
        ccd1_threshold = max(ccd1_threshold_min, min(raw_threshold, ccd1_threshold_max))
    else:
        ccd1_threshold = ccd1_threshold_min

    # 生成二值化数据 - 直接在现有数组上操作
    for i in range(128):
        ccd0_binary_data[i] = 1 if ccd0_real_data[i] > ccd0_threshold else 0
        ccd1_binary_data[i] = 1 if ccd1_real_data[i] > ccd1_threshold else 0
    
    # 重置丢失标志
    ccd0_left_lost = True
    ccd0_right_lost = True
    ccd1_left_lost = True
    ccd1_right_lost = True
 
    # CCD0边界检测
    for i in range(ccd0_mid_point_last, 3, -1):
        if i > 0 and ccd0_binary_data[i] == 1 and ccd0_binary_data[i-1] == 0:
            ccd0_left_point = i
            ccd0_left_lost = False
            break
    
    for i in range(ccd0_mid_point_last, 123):
        if i < 127 and ccd0_binary_data[i] == 1 and ccd0_binary_data[i+1] == 0:
            ccd0_right_point = i
            ccd0_right_lost = False
            break

    if ccd0_left_lost !=ccd0_right_lost:
        if ccd0_left_lost:
            for i in range(ccd0_right_point, 5, -1):
                if i > 0 and ccd0_binary_data[i] == 1 and ccd0_binary_data[i-1] == 0 and ccd0_binary_data[i-2] == 0:
                    ccd0_left_point = i
                    ccd0_left_lost = False
                    break
        else:
            for i in range(ccd0_left_point, 123):
                if i < 127 and ccd0_binary_data[i] == 1 and ccd0_binary_data[i+1] == 0 and ccd0_binary_data[i+2] == 0:
                    ccd0_right_point = i
                    ccd0_right_lost = False
                    break
        


    # 边界检查
    if ccd0_left_point <= ccd_inv_range:
        ccd0_left_lost = True
    if ccd0_right_point >= 127 - ccd_inv_range:
        ccd0_right_lost = True

    ccd0_left_point = 0 if ccd0_left_lost else ccd0_left_point
    ccd0_right_point = 127 if ccd0_right_lost else ccd0_right_point
    
    ccd0_mid_point = (ccd0_left_point + ccd0_right_point) // 2
    ccd0_track_width = ccd0_right_point - ccd0_left_point
    ccd0_err = ccd0_mid_point - ccd_expect_mid_point

    # CCD1边界检测
    for i in range(ccd1_mid_point_last, 5, -1):
        if i > 0 and ccd1_binary_data[i] == 1 and ccd1_binary_data[i-1] == 0:
            ccd1_left_point = i
            ccd1_left_lost = False
            break
    for i in range(ccd1_mid_point_last, 123):
        if i < 127 and ccd1_binary_data[i] == 1 and ccd1_binary_data[i+1] == 0:
            ccd1_right_point = i
            ccd1_right_lost = False
            break
    # 边界检查
    if ccd1_left_point <= ccd_inv_range:
        ccd1_left_lost = True
    if ccd1_right_point >= 127 - ccd_inv_range:
        ccd1_right_lost = True
    ccd1_left_point = 0 if ccd1_left_lost else ccd1_left_point
    ccd1_right_point = 127 if ccd1_right_lost else ccd1_right_point


    ccd1_mid_point = (ccd1_left_point + ccd1_right_point) // 2
    ccd1_track_width = ccd1_right_point - ccd1_left_point
    ccd1_err = ccd1_mid_point - ccd_expect_mid_point

def protect_ccd():
    """冲出赛道进行保护"""
    global ccd0_max_value, CCD_Protect
    if ccd0_max_value < CCD_Protect:
        return True
    else:
        return False

def check_zebra():
    """检测斑马线状态 - 适配现有代码结构"""
    global zebra_pass_time, zebra_detected_time, zebra_is_standby, zebra_stop_time
    global ccd0_binary_data

    # 计算中间区域0-1变化次数 (黑白交替)
    transitions = 0
    last_value = ccd0_binary_data[30]
    for i in range(30, 100):
        if ccd0_binary_data[i] != last_value:
            transitions += 1
            last_value = ccd0_binary_data[i]
    
    # 斑马线检测判定
    zebra_detected = transitions > 7
    
    # 斑马线状态机处理
    if zebra_is_standby:       # 处于待机状态
        if time.ticks_diff(time.ticks_ms(), zebra_detected_time) > 1000:
            zebra_is_standby = False
            beep.value(0)  # 关闭蜂鸣器
    else:
        if zebra_detected:      # 检测到斑马线
            zebra_pass_time += 1
            zebra_detected_time = time.ticks_ms()
            zebra_is_standby = True
            beep.value(1)  # 开启蜂鸣器
    
    # 判断是否需要停车
    return zebra_pass_time >= zebra_stop_time

# # element_start_ticks = 0  # 记录element=1时的ticks
# # circle_min_duration_ticks = 100  # 100个ticks = 1秒
# # temp_time = 0
# def get_circle_gyro_i():
#     """获取圆环内的陀螺仪积分值"""
#     global circle_gyro_i, circle_gyro_i_enable, imu_processor, imu_data, circle_flag,pit_5ms_flag
#     lcd.clear(0x0000)
#     beep_OK()
#     circle_gyro_i = 0
#     reset_car_state()
#     while (key_data[3] == 0):
#             global key_data
#             key_data = key.get()
#             if (pit_5ms_flag):
#                 imu_processor.imu_data = imu.get()                                         # 从IMU读取数据
#                 imu_processor.calibrate_data()
#                 kal_data_imu_4 = kal_imu_4.update(imu_processor.imu_data[4])                # 卡尔曼滤波
#                 circle_gyro_i += kal_data_imu_4 * 0.005  # 5ms间隔
#                 pit_5ms_flag = False
#             lcd.str24(0, 0, "Circle Gyro I:", 0xFFFF)
#             lcd.str24(0, 30, f"{circle_gyro_i:9.3f}", 0x07E0)  # 9位总宽度（含小数点），3位小数
#     beep_OK()

def update_circle():
    global r_circle_left_point,openart,circle_gyro_i_out,circle_gyro_i,circle_gyro_i_enable,ccd0_left_point,ccd0_right_point,ccd1_left_point,ccd1_right_point,follow_which,circle_flag,element,element_start_ticks,circle_min_duration_ticks
    if element == 0 and openart == 1:                      # 检测到红色元素
        element = 1
        circle_flag = 0
    if element == 1:
        if circle_flag == 0:
            element_start_ticks = pit2.ticks()                  # 使用10ms定时器的ticks计数
            circle_gyro_i = 0
            circle_gyro_i_enable = 0
        if circle_flag == 0 and ccd1_left_lost == True :        # 远CCD左边界丢失 表示左圆环
            circle_flag = 1             
            element = 10
            follow_which = 2                                    # 然后近ccd跟随右边界
            # beep.value(0)                                     # 关闭蜂鸣器
        elif circle_flag == 0 and ccd1_right_lost == True:      # 远CCD右边界丢失 表示右圆环
            circle_flag = 1             
            element = 11
            follow_which = 1                                    # 然后近ccd跟随左边界
            # beep.value(1)                                     # 开启蜂鸣器

    if element == 10:
        if circle_flag == 1 and ccd0_left_lost == True and ccd0_right_lost == False :
            circle_flag = 2
            follow_which = 2                                    # 近CCD跟随右边界
            beep.value(1)                                       # 关闭蜂鸣器
            circle_gyro_i_enable = 1
        # if circle_flag == 2 and ((ccd0_track_width < 70 and ccd0_left_point - ccd0_left_point_last < 0) or openart == 0):
        if circle_flag == 2 and (ccd0_track_width < 80 and ccd0_left_point - ccd0_left_point_last < 0) :
            circle_flag = 3
            follow_which = 6                                    # 近CCD中线往左固定拉
            beep.value(0)                                     
            # element_start_ticks = pit2.ticks()                # 记录时间
        if circle_flag == 3 and circle_gyro_i > 500:
            circle_flag = 4
        if circle_flag == 4 and ccd0_right_lost == True:
            circle_flag = 5
        if circle_flag == 5 and ccd0_right_point < l_circle_right_point:
            circle_flag = 6
            follow_which = 2                                    # 近CCD跟随右边界
            beep.value(1)                                       # 开启蜂鸣器
        if circle_flag == 6 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
            circle_flag = 0
            element = 0
            follow_which = 0                                    # 跟随中间线
            beep.value(0)                                       # 关闭蜂鸣器
            element_start_ticks = 0                             # 重置元素开始时间
            circle_gyro_i = 0
            circle_gyro_i_enable = 0


    if element == 11:                                           # 右圆环
        if circle_flag == 1 and ccd0_left_lost == False and ccd0_right_lost == True :
            circle_flag = 2
            follow_which = 1                                    # 近CCD跟随左边界
            beep.value(1)                                       # 关闭蜂鸣器
            circle_gyro_i_enable = 1
        if circle_flag == 2 and (ccd0_track_width < 80 and ccd0_right_point - ccd0_right_point_last > 0) and openart == 0:
            circle_flag = 3
            follow_which = 7                                    # 近CCD往右边界
            beep.value(0)                               
        if circle_flag == 3 and circle_gyro_i < -500:
            circle_flag = 4
        if circle_flag == 4 and ccd0_left_lost == True:
            circle_flag = 5
        if circle_flag == 5 and ccd0_left_point > r_circle_left_point:
            circle_flag = 6
            follow_which = 1                                    # 近CCD跟随左边界
            beep.value(1)                                       # 开启蜂鸣器
        if circle_flag == 6 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
            circle_flag = 0
            element = 0
            follow_which = 0                                    # 跟随中间线
            beep.value(0)                                       # 关闭蜂鸣器
            element_start_ticks = 0                             # 重置元素开始时间
            circle_gyro_i = 0
            circle_gyro_i_enable = 0



    # 一直用的出环 打算要改 他妈的鲁棒性太低了!!!
    # if element == 10:
    #     if circle_flag == 1 and ccd0_left_lost == True and ccd0_right_lost == False :
    #         circle_flag = 2
    #         follow_which = 2                                    # 近CCD跟随右边界
    #         beep.value(1)                                       # 关闭蜂鸣器
    #     # if circle_flag == 2 and ((ccd0_track_width < 70 and ccd0_left_point - ccd0_left_point_last < 0) or openart == 0):
    #     if circle_flag == 2 and (ccd0_track_width < 80 and ccd0_left_point - ccd0_left_point_last < 0) and openart == 0:
    #         circle_flag = 3
    #         follow_which = 6                                    # 近CCD往左边界
    #         beep.value(0)                                     
    #         # element_start_ticks = pit2.ticks()                # 记录时间
    #     if circle_flag == 3 and openart == 1:
    #         circle_flag = 4
    #     if circle_flag == 4 and openart == 0:
    #         circle_flag = 5
    #         follow_which = 2                                    # 近CCD跟随右边界
    #         beep.value(1)                                       # 开启蜂鸣器
    #     if circle_flag == 5 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
    #         circle_flag = 0
    #         element = 0
    #         follow_which = 0                                    # 跟随中间线
    #         beep.value(0)                                       # 关闭蜂鸣器
    #         element_start_ticks = 0                             # 重置元素开始时间


    # if element == 11:                                           # 右圆环
    #     if circle_flag == 1 and ccd0_left_lost == False and ccd0_right_lost == True :
    #         circle_flag = 2
    #         follow_which = 1                                    # 近CCD跟随左边界
    #         beep.value(1)                                       # 关闭蜂鸣器
    #     if circle_flag == 2 and (ccd0_track_width < 80 and ccd0_right_point - ccd0_right_point_last > 0) and openart == 0:
    #         circle_flag = 3
    #         follow_which = 7                                    # 近CCD往右边界
    #         beep.value(0)                               
    #     if circle_flag == 3 and openart == 1:
    #         circle_flag = 4
    #     if circle_flag == 4 and openart == 0:
    #         circle_flag = 5
    #         follow_which = 1                                    # 近CCD跟随左边界
    #         beep.value(1)                                       # 开启蜂鸣器
    #     if circle_flag == 5 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
    #         circle_flag = 0
    #         element = 0
    #         follow_which = 0                                    # 跟随中间线
    #         beep.value(0)                                       # 关闭蜂鸣器
    #         element_start_ticks = 0                             # 重置元素开始时间


        # if circle_flag == 3 and ccd0_left_lost == True and ccd0_right_lost == False and pit2.ticks() - element_start_ticks > 75 and follow_which == 6    and temp_time == 0 :
        #     beep.value(0)
        #     follow_which = 0                                  # 近CCD跟随中间线
        #     temp_time = 1
        # if circle_flag == 3 and openart == 1:
        #     follow_which = 1
        #     circle_flag = 4
        # if circle_flag == 4 and openart == 0:
        #     circle_flag = 5
        #     follow_which = 2                                    # 近CCD跟随右边界
        # if circle_flag == 5 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
        #     circle_flag = 0
        #     element = 0
        #     follow_which = 0                                    # 跟随中间线
        #     beep.value(0)                                       # 关闭蜂鸣器
        #     element_start_ticks = 0                              # 重置元素开始时间
        



    # if element == 10:                                           # 左圆环
    #     if circle_flag == 1 and ccd0_left_lost == True and ccd0_right_lost == False :
    #         circle_flag = 2                     
    #         follow_which = 2                                    # 近CCD跟随右边界
    #         beep.value(0)
    #     if circle_flag == 2 and ccd0_track_width < 70:
    #         circle_flag = 3
    #         follow_which = 1                                    # 近CCD跟随左边界
    #         beep.value(1)
    #         circle_gyro_i_enable = True
    #         circle_gyro_i = 0
    #         temp_time = pit2.ticks()
    #         # beep.value(1)
    #     # if circle_flag == 3 and ccd0_right_lost == False and follow_which == 1 and pit2.ticks() - temp_time > 50:
    #     if circle_flag == 3 and ccd0_right_lost == False and follow_which == 1:
    #         temp_time = pit2.ticks()
    #         follow_which = 0                                    # 近CCD跟随中间线
    #         circle_flag = 4
    #     if circle_flag == 4 and ccd0_right_lost == True and pit2.ticks() - temp_time > 10 :
    #         temp_time = pit2.ticks()
    #         circle_flag = 5
    #         # circle_gyro_i_enable = False
    #         follow_which = 1                                    
    #         beep.value(0)
    #     if circle_flag == 5 and ccd0_right_point < 95 and circle_gyro_i > circle_gyro_i_out:
    #         circle_flag = 6
    #         follow_which = 2                                    # 近CCD跟随右边界
    #         beep.value(1)
    #     if circle_flag == 6 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
    #         circle_flag = 0
    #         element = 0
    #         follow_which = 0                                    # 跟随中间线
    #         beep.value(0)                                       # 关闭蜂鸣器
    #         element_start_ticks = 0
    


    # if element == 10:                                           # 左圆环
    #     if circle_flag == 1 and ccd0_left_lost == True and ccd0_right_lost == False :
    #         circle_flag = 2                     
    #         follow_which = 2                                    # 近CCD跟随右边界
    #         beep.value(0)
    #     if circle_flag == 2 and ccd0_track_width < 75 and ccd0_left_point - ccd0_left_point_last < 0:
    #         circle_flag = 3
    #         follow_which = 1                                    # 近CCD跟随左边界
    #         beep.value(1)
    #         temp_time = pit2.ticks()
    #     if circle_flag == 3 and pit2.ticks() - temp_time > 50:
    #         follow_which = 0                                    
    #     if circle_flag == 3 and ccd0_left_lost == True and ccd0_right_lost == True and pit2.ticks() - temp_time > 75:
    #         circle_flag = 4
    #         follow_which = 5                                    # 直行
    #     if circle_flag == 4 and ccd0_right_point < 90:
    #         circle_flag = 5
    #         follow_which = 2                                    # 近CCD跟随右边界
    #         beep.value(1)
    #     if circle_flag == 5 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
    #         circle_flag = 0
    #         element = 0
    #         follow_which = 0                                    # 跟随中间线
    #         beep.value(0)                                       # 关闭蜂鸣器
    #         element_start_ticks = 0

            # circle_gyro_i_enable = True
            # circle_gyro_i = 0
            # temp_time = pit2.ticks()
            # beep.value(1)
        # if circle_flag == 3 and ccd0_right_lost == False and follow_which == 1 and pit2.ticks() - temp_time > 50:
        # if circle_flag == 3 and ccd0_right_lost == False:
        #     temp_time = pit2.ticks()
        #     follow_which = 0                                    # 近CCD跟随中间线
        #     circle_flag = 4
        # if circle_flag == 4 and ccd0_right_lost == True and pit2.ticks() - temp_time > 10 :
        #     temp_time = pit2.ticks()
        #     circle_flag = 5
        #     # circle_gyro_i_enable = False
        #     follow_which = 1                                    
        #     beep.value(0)
        # if circle_flag == 5 and ccd0_right_point < 95 and circle_gyro_i > circle_gyro_i_out:
        #     circle_flag = 6
        #     follow_which = 2                                    # 近CCD跟随右边界
        #     beep.value(1)
        # if circle_flag == 6 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
        #     circle_flag = 0
        #     element = 0
        #     follow_which = 0                                    # 跟随中间线
        #     beep.value(0)                                       # 关闭蜂鸣器
        #     element_start_ticks = 0
    


    # elif element == 11:                                         # 右圆环
    #     element = 0
    #     circle_flag = 0
    #     follow_which = 0
    # elif element == 11:                                         # 右圆环
    #     if  circle_flag == 1 and ccd0_left_lost == False and ccd0_right_lost == True :
    #         circle_flag = 2
    #         follow_which = 1                                    # 近CCD跟随左边界
    #     if circle_flag == 2 and ccd0_track_width < 70:
    #         circle_flag = 3
    #         follow_which = 2                                    # 近CCD跟随右边界
    #         # circle_gyro_i_enable = True
    #         circle_gyro_i = 0
    #         beep.value(0)
    #     if circle_flag == 3 and ccd0_left_lost == False and follow_which == 2:
    #         follow_which = 0                                    # 近CCD跟随中间线
    #         circle_flag = 4
    #     if circle_flag == 4 and ccd0_left_lost == True:
    #         circle_flag = 5
    #         # circle_gyro_i_enable = False
    #         follow_which = 5                                    # 直行
    #         beep.value(1)
    #     if circle_flag == 5 and ccd0_left_point > 30:
    #         circle_flag = 6
    #         follow_which = 1                                    # 近CCD跟随左边界
    #     if circle_flag == 6 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
    #         circle_flag = 0
    #         element = 0
    #         follow_which = 0                                    # 跟随中间线
    #         beep.value(0)                                       # 关闭蜂鸣器
    #         element_start_ticks = 0

    

    # if (circle_flag == 3 and
    #     ccd1_left_lost == True and 
    #     ccd1_right_lost == True and 
    #     ccd1_avg_brightness > 40 and
    #     element == 10 and
    #     (pit2.ticks() - element_start_ticks) >= circle_min_duration_ticks):  # 增加1秒时间判断
    #     circle_flag = 4
    #     follow_which = 1                                       # 近CCD跟随左边界
    #     beep.value(1)                                         # 开启蜂鸣器

    # elif (circle_flag == 3 and
    #     ccd1_left_lost == True and
    #     ccd1_right_lost == True and
    #     ccd1_avg_brightness > 40 and
    #     element == 11 and
    #     (pit2.ticks() - element_start_ticks) >= circle_min_duration_ticks):  # 增加1秒时间判断
    #     circle_flag = 4
    #     follow_which = 2                                       # 近CCD跟随右边界
    #     beep.value(1)                                         # 开启蜂鸣器


    # if element == 10:
    #     if circle_flag == 4 and abs(ccd0_right_point-ccd1_right_point) < 25 and ccd0_right_point < 105:          # 圆环结束
    #         circle_flag = 5
    #         follow_which = 2                                       #  近CCD跟随右边界
    #         beep.value(0)
    #         element_start_ticks = 0  # 重置时间记录
    #     elif circle_flag == 5 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
    #         circle_flag = 0
    #         element = 0
    #         follow_which = 0                                        # 跟随中间线
    #         beep.value(0)
    # elif element == 11:
    #     if circle_flag == 4 and abs(ccd0_right_point-ccd1_right_point) < 25 and ccd0_left_point > 15:          # 圆环结束
    #         circle_flag = 5
    #         follow_which = 1                                        #  近CCD跟随左边界
    #         beep.value(0)
    #         element_start_ticks = 0  # 重置时间记录
    #     elif circle_flag == 5 and ccd0_track_width < 80 and ccd1_track_width < 70 and ccd0_left_lost == False and ccd0_right_lost == False and ccd1_left_lost == False and ccd1_right_lost == False:
    #         circle_flag = 0
    #         element = 0
    #         follow_which = 0                                        # 跟随中间线
    #         beep.value(0)
    

    # if circle_flag == 3:                                      # 圆环结束

# 测试时间用
# def check_time():
#     global temp_time, element_start_ticks
#     if temp_time == 0:
#         element_start_ticks = pit2.ticks()  # 使用10ms定时器的ticks计数
#         temp_time = 1
#         print("start")
#     if pit2.ticks() - element_start_ticks > circle_min_duration_ticks:
#         element_start_ticks = pit2.ticks()
#         print("time:", temp_time)
#         temp_time+=1

def update_roadblock():
    global follow_which,element,openart
    if element == 0 and (openart == 3 or openart == 4):
        if openart == 3:    # 黄色在左边
            follow_which = 8  # 中线左偏roadblock_err
        elif openart == 4:  # 黄色在右边
            follow_which = 9  # 中线右偏roadblock_err
    elif element == 0 and openart == 0:
        follow_which = 0  # 跟随中间线

def update_ramp():
    global follow_which, element, openart,ramp_start_ticks,angle_state
    if element == 0 and openart == 2:  # 检测到绿色颜色
        # follow_which = 5  # 不跟随
        follow_which = 0 
        element = 2  # 设置元素为坡道
        # 记录当前时间
        ramp_start_ticks = pit2.ticks()
        beep.value(1)  # 开启蜂鸣器
        angle_state = 3  # 设置坡道状态

    elif element == 2 and (pit2.ticks() - ramp_start_ticks) > 150:  # 如果坡道元素已经存在且超过1秒
        follow_which = 0  # 跟随中间线
        element = 0       # 重置元素状态
        beep.value(0)     # 关闭蜂鸣器
        angle_state = 2   # 设置正常状态

def update_err():
    global  err_ave, ccd_err,ccd0_err,ccd1_err,follow_which,ccd0_left_point,ccd0_right_point,ccd_err_last,circle_err,roadblock_err
    ccd_err_last = ccd_err
    if follow_which == 0 :          # 跟随中间线
        ccd_err = ccd0_err
    elif follow_which == 1:         # 近CCD跟随左边界
        ccd_err = (int)(ccd0_left_point + 0.5*ccd0_nomal_width - ccd_expect_mid_point)
        # beep.value(1)
    elif follow_which == 2:         # 近CCD跟随右边界
        ccd_err = (int)(ccd0_right_point - 0.5*ccd0_nomal_width - ccd_expect_mid_point)
        # beep.value(1)
    elif follow_which == 3:         # 误差平均巡线 
        ccd_err = err_ave
    elif follow_which == 4:         # 跟随远CCD左边界
        ccd_err = ccd1_err          
    elif follow_which == 5:         # 不跟随
        ccd_err = 0
    elif follow_which == 6:         # 中线左偏circle_err
        ccd_err = ccd0_err - circle_err
    elif follow_which == 7:         # 中线右偏circle_err
        ccd_err = ccd0_err + circle_err
    elif follow_which == 8:         # 中线左偏roadblock_err
        beep.value(1)  # 开启蜂鸣器  
        ccd_err = ccd0_err - roadblock_err
    elif follow_which == 9:         # 中线右偏roadblock_err
        beep.value(1)  # 开启蜂鸣器
        ccd_err = ccd0_err + roadblock_err
    # if abs(ccd_err_last - ccd_err) > 25:
    #     ccd_err = (int)((1*ccd_err_last + 9*ccd_err)//10)
    # elif abs(ccd_err_last - ccd_err) > 20:
    #     ccd_err = (int)((1*ccd_err_last + 4*ccd_err)//5)
    # elif abs(ccd_err_last - ccd_err) > 15:
    #     ccd_err = (int)((2*ccd_err_last + 3*ccd_err)//5)
    # elif abs(ccd_err_last - ccd_err) > 10:
    #     ccd_err = (int)((3*ccd_err + 2*ccd_err_last)//5)
    # else:
    # ccd_err = (int)((4*ccd_err + ccd_err_last)//5)
    ccd_err = max(min(ccd_err, 30), -30)

def start_cargo():
    global angle_state,turn_pwm_max,openart,circle_gyro_i,circle_gyro_i_enable,circle_speed_k,element,circle_flag,ccd_err,ccd0_err,zebra_is_standby,motor_ALL_speed,kal_data_encoder_left,kal_data_encoder_right,menu_need_redraw,basic_pwm,turn_pwm,kal_data_imu_4,pit_5ms_flag, pit_10ms_flag, pit_20ms_flag,target_gyro,target_angle,target_speed,encoder_sum,kal_encoder_right,kal_encoder_left
    outseason = 0    
    reset_car_state()
    save_to_file("BackUp.txt")
    lcd.clear(0x0000)
    ccd_err = 0
    angle_state = 0 # 0:刚开始只是直立 1:一开始没有达到目标速度前 固定角度! 2:正常的速度环 3:坡道
    target_angle = mid_angle
    start_time = pit2.ticks() # 使用10ms定时器的ticks计数
    element_parser.update()  # 更新元素识别数据
    element_parser.clear_element()  # 清除元素识别数据
    openart = 0
    element = 0
    target_speed_run = target_speed
    while outseason == 0:
        if (pit_5ms_flag):
            imu_processor.imu_data = imu.get()                                         # 从IMU读取数据
            imu_processor.calibrate_data()
            basic_pwm = -balance_gyro_controller.calculate(target_gyro, imu_processor.imu_data[3])  # 陀螺仪环
            # turn_pwm = direction_controller.calculate(ccd0_err, imu_processor.imu_data[4])
            kal_data_imu_4 = kal_imu_4.update(imu_processor.imu_data[4])                # 卡尔曼滤波
            if circle_gyro_i_enable:
                circle_gyro_i += kal_data_imu_4 * 0.005  # 5ms间隔

            # 如果检测出来是斑马线 ccd_err = 0
            if zebra_is_standby or angle_state == 0:
                ccd0_err = 0
            # turn_pwm_last = turn_pwm
            turn_pwm = direction_controller.calculate(-ccd_err, -kal_data_imu_4)
            if angle_state == 0:
                turn_pwm = 0  # 刚开始只需要直立
            # else:
            #     turn_pwm = (int)(0.8 * turn_pwm + 0.2 * turn_pwm_last)
            turn_pwm = max(min(turn_pwm, turn_pwm_max), -turn_pwm_max)                                 # 限制转向PWM
            if element == 2:  # 坡道状态
                turn_pwm = (int)(turn_pwm * 0)  # 坡道状态下减小转向PWM
            pit_5ms_flag = False
        if (pit_10ms_flag):
            imu_processor.update()                                                     # 更新IMU数据
            element_parser.update()
            openart = element_parser.get_element()                                                # 获取元素识别数据
            # print(f"{openart}")
            # wireless.send_str(f"{element},{circle_flag},{circle_gyro_i},{circle_gyro_i_enable},{ccd0_left_point},{ccd0_right_point},{openart}\n")
            update_ccd()
            update_circle()
            update_roadblock()
            update_ramp()                                                       # 更新坡道状态
            # check_time()
            update_err()

            if element!=2 and protect_ccd():                                                          # CCD保护
                outseason = 5
                break
            if angle_state != 0 and element != 2 and openart != 0:                     # 如果不是坡道元素并且不是刚开始发车的时候              if check_zebra():                                                   # 检测斑马线
                if check_zebra():                                                   # 检测斑马线
                    outseason = 3
                    break
            target_gyro = -balance_angle_controller.calculate(target_angle, -imu_processor.filter_angle)
            pit_10ms_flag = False

        if (pit_20ms_flag):
            get_speed()                                                                # 获取电机速度
            elapsed_time = pit2.ticks() - start_time
            if angle_state == 0:                                                       # 刚开始只需要直立
                angle_out = speed_controller.calculate(0, motor_ALL_speed)
                if elapsed_time > 50:
                    angle_state = 1
            elif angle_state == 1:        
                if abs(motor_ALL_speed) >= abs(target_speed) * 0.5:
                    angle_state = 2
                    # 清除速度环的积分项，避免切换时的冲击
                    speed_controller.clear()
                angle_out = first_angle_out
            elif angle_state == 2:                                                     # 正常速度环
                angle_out = speed_controller.calculate(target_speed_run, motor_ALL_speed)
                # if abs(motor_ALL_speed) > abs(target_speed)*0.95:
                #     target_speed = (int)(target_speed * 1.05)
            elif angle_state == 3:                                                     # 坡道状态
                angle_out = -500
            target_angle = mid_angle - angle_out
            if element == 10 or element == 11:  # 圆环元素
                target_speed_run = (int)(target_speed * circle_speed_k)
            else:
                target_speed_run = target_speed  # 恢复正常速度
                # beep.value(1)  # 开启蜂鸣器
            if target_angle < min_angle:
                target_angle = min_angle
            pit_20ms_flag = False
        if abs(kal_data_encoder_left) > 2300 or abs(kal_data_encoder_right) > 2300:
            outseason = 4
            break
        SetMotor(basic_pwm-turn_pwm,basic_pwm+turn_pwm)
        gc.collect()
    SetMotor(0, 0)  # 停止电机
    time.sleep_ms(1000)
    for i in range(outseason):
        beep.value(1)
        time.sleep_ms(75)
        beep.value(0)
        time.sleep_ms(125)
    # while Key_Click() == False:
    #     if outseason==1:
    #         lcd.str24(40, 150, "Zebra  Line", 0x07E0)
    #     elif outseason==2:
    #         lcd.str24(40, 150, "Encoder    ", 0x07E0)
    #     elif outseason==3:
    #         lcd.str24(40, 150, "CCD Protect", 0x07E0)
    #     elif outseason==4:
    #         lcd.str24(40, 150, "Key Pressed", 0x07E0)
    #     gc.collect()
    #     lcd.str24(40, 175, f"element{element:d}",0x07E0)
    #     lcd.str24(40, 200, f"circle_flag{circle_flag:d}",0x07E0)
    #     lcd.str24(40, 225, f"follow_which{follow_which:d}",0x07E0)        
    #     time.sleep_ms(100)
    lcd.clear(0x0000)
    menu_need_redraw = True

imu_processor = IMUProcessor(imu)
element_parser = ElementParser(uart)
balance_gyro_controller = SimplePID()
balance_angle_controller = SimplePID()
speed_controller = SimplePID()
direction_controller = DirectionController()
kal_encoder_left = KalmanFilter(Q=0.008, R=0.1)
kal_encoder_right = KalmanFilter(Q=0.008, R=0.1)
kal_imu_4 = KalmanFilter(Q=0.01, R=0.8)
init_params()

while True:
    process_menu()
    gc.collect()

    if switch2.value() != state2:
        pit1.stop()
        pit2.stop()
        pit3.stop()
        pit4.stop()
        motor_L.duty(0)
        motor_R.duty(0)
        break