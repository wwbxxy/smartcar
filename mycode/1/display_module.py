from machine import Pin
from display import LCD_Drv, LCD
from seekfree import KEY_HANDLER
import time


class DISPLAY:
    def __init__(self):
        # ===== LCD 初始化 =====
        cs = Pin('B29', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        cs.high()
        cs.low()
        rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        dc = Pin('B5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
        self.lcd = LCD(drv)
        self.lcd.color(0xFFFF, 0x0000)
        self.lcd.mode(2)
        self.lcd.clear(0x0000)

        # ===== 按键初始化 (扫描周期5ms，与ticker一致) =====
        self.key = KEY_HANDLER(5)

        # ===== 蜂鸣器 =====
        self.beep = Pin('D24', Pin.OUT, value=False)

        # ===== 颜色常量 (RGB565) =====
        self.WHITE = 0xFFFF
        self.BLACK = 0x0000
        self.RED = 0xF800
        self.GREEN = 0x07E0
        self.BLUE = 0x001F
        self.YELLOW = 0xFFE0
        self.CYAN = 0x07FF

    # ===== 蜂鸣器辅助 =====
    def beep_ok(self):
        self.beep.value(1)
        time.sleep_ms(100)
        self.beep.value(0)

    def beep_error(self):
        for _ in range(3):
            self.beep.value(1)
            time.sleep_ms(100)
            self.beep.value(0)
            time.sleep_ms(100)

    def clear(self):
        self.lcd.clear(self.BLACK)

    # ===== 菜单绘制 =====
    def show_menu(self, page, index, items, params=None, edit=False):
        """绘制菜单
        page:   "main" 或 "params"
        index:  当前选中项索引
        items:  菜单项列表
        params: 参数字典 (params页面用)
        edit:   是否编辑模式
        """
        self.lcd.clear(self.BLACK)

        # 标题
        if page == "main":
            self.lcd.str16(5, 5, "== BALANCE CAR ==", self.WHITE)
        else:
            title_color = self.RED if edit else self.WHITE
            self.lcd.str16(5, 5, "== PARAMETERS ==", title_color)

        y_start = 25
        y_step = 16
        max_visible = 14

        start_idx = 0
        if index >= max_visible:
            start_idx = index - max_visible + 1
        end_idx = min(start_idx + max_visible, len(items))

        for i in range(start_idx, end_idx):
            y = y_start + (i - start_idx) * y_step
            text_color = self.YELLOW if i == index else self.WHITE

            if page == "params" and i < len(items) - 1:
                param_key = items[i]
                value = params.get(param_key, 0) if params else 0
                prefix = "*" if edit and i == index else (">" if i == index else " ")
                self.lcd.str16(5, y, "{}{}".format(prefix, param_key), text_color)
                value_color = self.RED if edit and i == index else self.GREEN
                self.lcd.str16(130, y, "{:.3f}".format(value), value_color)
            else:
                prefix = ">" if i == index else " "
                self.lcd.str16(5, y, "{}{}".format(prefix, items[i]), text_color)

    # ===== 状态/运行界面 =====
    def show_status(self, angle, gyro, output, running, target_angle=0.0,
                    raw_acc=None, raw_gyro=None, calibrated=False):
        """实时状态显示
        angle:       互补滤波角度
        gyro:        角速度 (dps)
        output:      电机输出PWM
        running:     是否正在运行平衡控制
        target_angle:目标角度
        raw_acc:     (ax, ay, az) 原始加速度
        raw_gyro:    (gx, gy, gz) 原始陀螺仪
        calibrated:  IMU是否已校准
        """
        self.lcd.clear(self.BLACK)

        title = ">> RUNNING <<" if running else "-- STATUS --"
        title_color = self.GREEN if running else self.CYAN
        self.lcd.str16(5, 0, title, title_color)

        self.lcd.str16(5, 20, "Angle:{:>8.2f}".format(angle), self.YELLOW)
        self.lcd.str16(5, 40, "Gyro :{:>8.2f}".format(gyro), self.CYAN)
        self.lcd.str16(5, 60, "Out  :{:>8.1f}".format(output), self.RED)
        self.lcd.str16(5, 80, "Tgt  :{:>8.2f}".format(target_angle), self.WHITE)

        if raw_acc is not None:
            self.lcd.str16(5, 110, "Acc:{:>5d},{:>5d},{:>5d}".format(raw_acc[0], raw_acc[1], raw_acc[2]), self.WHITE)
        if raw_gyro is not None:
            self.lcd.str16(5, 130, "Gyr:{:>5d},{:>5d},{:>5d}".format(raw_gyro[0], raw_gyro[1], raw_gyro[2]), self.WHITE)

        cal_str = "Yes" if calibrated else "No"
        cal_color = self.GREEN if calibrated else self.RED
        self.lcd.str16(5, 160, "Cal: {}".format(cal_str), cal_color)

        hint = "[Back] Stop" if running else "[Back] Menu"
        self.lcd.str16(5, 300, hint, self.WHITE)

    # ===== 消息提示 =====
    def show_message(self, msg, color=None):
        if color is None:
            color = self.GREEN
        self.lcd.clear(self.BLACK)
        self.lcd.str16(10, 100, msg, color)
