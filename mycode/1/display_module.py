from machine import Pin
from display import LCD_Drv, LCD
from seekfree import KEY_HANDLER
import time


# ===== 菜单节点类 =====
class MenuItem:
    """菜单节点
    name:      显示名称
    children:  子节点列表 (有children=文件夹, 无=叶子)
    on_enter:  进入时的回调 (动作节点用)
    param_key: 绑定的参数键名 (参数节点用, 可编辑)
    step:      参数调节步长
    """
    def __init__(self, name, children=None, on_enter=None, param_key=None, step=1.0):
        self.name = name
        self.children = children if children else []
        self.parent = None
        self.on_enter = on_enter
        self.param_key = param_key
        self.step = step
        for c in self.children:
            c.parent = self


# ===== 菜单控制器 (导航+绘制全封装) =====
class Menu:
    def __init__(self, display, root, params):
        self.disp = display
        self.root = root
        self.current = root        # 当前所在文件夹节点
        self.index = 0             # 当前选中项索引
        self.editing = False       # 是否编辑参数中
        self.need_redraw = True
        self.params = params       # 参数字典引用 (主程序维护)
        self.params_dirty = False  # 参数已改标记 (主循环检测后同步到PID)
    def reset(self):
        """回到根菜单"""
        self.current = self.root
        self.index = 0
        self.editing = False
        self.need_redraw = True

    def _items(self):
        return self.current.children

    def handle_keys(self, kd):
        """处理按键 [up, down, ok, back]"""
        items = self._items()
        if not items:
            return

        # --- 返回键 (优先处理) ---
        if kd[3] > 0:
            self.disp.key.clear(4)
            if self.editing:
                self.editing = False
            elif self.current.parent is not None:
                # 回到父节点, 定位到刚才进入的子项
                parent = self.current.parent
                try:
                    self.index = parent.children.index(self.current)
                except:
                    self.index = 0
                self.current = parent
            self.need_redraw = True
            return

        # --- 编辑模式 ---
        if self.editing:
            item = items[self.index]
            if item.param_key is not None:
                val = self.params.get(item.param_key, 0)
                if kd[0] > 0:       # 上 → 增大
                    self.params[item.param_key] = val + item.step
                    self.disp.key.clear(1)
                    self.need_redraw = True
                elif kd[1] > 0:     # 下 → 减小
                    self.params[item.param_key] = val - item.step
                    self.params_dirty = True
                    self.disp.key.clear(2)
                    self.need_redraw = True
            if kd[2] > 0:           # 确认 → 退出编辑
                self.editing = False
                self.params_dirty = True
                self.disp.key.clear(3)
                self.need_redraw = True

        # --- 浏览模式 ---
        else:
            if kd[0] > 0:
                self.index = (self.index - 1) % len(items)
                self.disp.key.clear(1)
                self.need_redraw = True
            elif kd[1] > 0:
                self.index = (self.index + 1) % len(items)
                self.disp.key.clear(2)
                self.need_redraw = True
            elif kd[2] > 0:
                self.disp.key.clear(3)
                item = items[self.index]
                if item.children:              # 文件夹 → 进入
                    self.current = item
                    self.index = 0
                    self.need_redraw = True
                elif item.param_key is not None:  # 参数 → 编辑
                    self.editing = True
                    self.need_redraw = True
                elif item.on_enter:               # 动作 → 执行回调
                    item.on_enter()

    def draw(self):
        lcd = self.disp.lcd
        lcd.clear(self.disp.BLACK)

        # 标题
        title = self.current.name
        tcolor = self.disp.RED if self.editing else self.disp.WHITE
        lcd.str16(5, 5, "== {} ==".format(title), tcolor)

        items = self._items()
        y_start = 25
        y_step = 16
        max_vis = 14

        start = 0
        if self.index >= max_vis:
            start = self.index - max_vis + 1
        end = min(start + max_vis, len(items))

        for i in range(start, end):
            y = y_start + (i - start) * y_step
            sel = (i == self.index)
            color = self.disp.YELLOW if sel else self.disp.WHITE
            item = items[i]

            if item.param_key is not None:
                # 参数项: 名字 + 数值
                prefix = "*" if (self.editing and sel) else (">" if sel else " ")
                lcd.str16(5, y, "{}{}".format(prefix, item.name), color)
                val = self.params.get(item.param_key, 0)
                vc = self.disp.RED if (self.editing and sel) else self.disp.GREEN
                lcd.str16(130, y, "{:.3f}".format(val), vc)
            else:
                # 普通项/文件夹
                prefix = ">" if sel else " "
                suffix = " >>" if item.children else ""
                lcd.str16(5, y, "{}{}{}".format(prefix, item.name, suffix), color)

    def process(self):
        """主循环调用: 处理→按需重绘 (capture由ticker自动完成)"""
        kd = self.disp.key.get()
        if kd[0] or kd[1] or kd[2] or kd[3]:
            self.handle_keys(kd)
        if self.need_redraw:
            self.draw()
            self.need_redraw = False
            time.sleep_ms(50)


# ===== 显示类 (LCD + 按键 + 蜂鸣器) =====
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
