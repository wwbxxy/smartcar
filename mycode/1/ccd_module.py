# ccd_module.py
# TSL1401 线阵CCD巡线模块
# 功能: 数据采集 → 动态二值化 → 边界检测 → 中点/误差提取 → 屏幕可视化


class CCD:
    """单个CCD的处理类
    ccd_obj:  TSL1401 实例
    index:    ccd.get(index) 的参数 (1=中CCD, 2=近CCD, 按接线而定)
    expect_mid: 期望中线位置 (默认64, 即128像素正中)
    """

    def __init__(self, ccd_obj, index, expect_mid=64):
        self.ccd = ccd_obj
        self.index = index
        self.expect_mid = expect_mid

        # 原始与二值化数据 (128像素)
        self.raw_data = [0] * 128
        self.binary_data = [0] * 128

        # 统计量
        self.max_value = 0
        self.min_value = 0
        self.threshold = 0
        self.avg_brightness = 0

        # 边界与中点
        self.left_point = 64
        self.right_point = 64
        self.mid_point = 64
        self.track_width = 0
        self.left_lost = True
        self.right_lost = True

        # 上次中点 (用于连续性搜索, 防止跳变)
        self.mid_point_last = 64

        # 误差 = mid_point - expect_mid
        self.err = 0

        # ===== 可调参数 (通过菜单修改) =====
        # 二值化阈值上下限 (动态阈值会被限制到此范围)
        self.threshold_max = 100
        self.threshold_min = 10
        # 非法边界范围 (边界落在 [0, inv_range] 或 [127-inv_range, 127] 视为丢失)
        self.inv_range = 10
        # CCD保护阈值 (max_value 低于此值视为冲出赛道/全黑)
        self.protect_value = 12

    def update(self):
        """采集一帧CCD并完成全部处理 (在10ms周期调用)"""
        # 保存上次中点
        self.mid_point_last = self.mid_point

        # 1. 采集
        data = self.ccd.get(self.index)
        if not data or len(data) != 128:
            self.raw_data = [0] * 128
            return
        self.raw_data = data

        # 2. 统计
        self.max_value = max(self.raw_data)
        self.min_value = min(self.raw_data)
        self.avg_brightness = sum(self.raw_data) // 128

        # 3. 动态二值化阈值: (max + min) / 2 取中点
        #    白赛道(亮)与蓝背景(暗)对比度高, 中点阈值最稳定
        #    若光照不均可改回 (max + 2*min) // 3 偏向暗端
        if (self.max_value + self.min_value) > 0:
            raw_th = (self.max_value + self.min_value) // 2
            self.threshold = max(self.threshold_min, min(raw_th, self.threshold_max))
        else:
            self.threshold = self.threshold_min

        # 4. 二值化: 1=白赛道(亮), 0=蓝背景(暗)
        for i in range(128):
            self.binary_data[i] = 1 if self.raw_data[i] > self.threshold else 0

        # 5. 边界检测 (从上次中点向两侧搜索 1→0 跳变)
        self._detect_edges()

        # 6. 中点与误差
        self.mid_point = (self.left_point + self.right_point) // 2
        self.track_width = self.right_point - self.left_point
        self.err = self.mid_point - self.expect_mid

    def _detect_edges(self):
        """边界检测: 从上次中点向两侧找 1→0 跳变"""
        self.left_lost = True
        self.right_lost = True

        # 向左搜索左边界
        for i in range(self.mid_point_last, 3, -1):
            if self.binary_data[i] == 1 and self.binary_data[i - 1] == 0:
                self.left_point = i
                self.left_lost = False
                break

        # 向右搜索右边界
        for i in range(self.mid_point_last, 124):
            if self.binary_data[i] == 1 and self.binary_data[i + 1] == 0:
                self.right_point = i
                self.right_lost = False
                break

        # 单边丢失时, 用另一边 + 标准赛道宽度推算
        if self.left_lost != self.right_lost:
            if self.left_lost:
                # 左丢失, 用右边往左推一个正常宽度
                for i in range(self.right_point, 5, -1):
                    if i > 0 and self.binary_data[i] == 1 and self.binary_data[i - 1] == 0:
                        self.left_point = i
                        self.left_lost = False
                        break
            else:
                # 右丢失, 用左边往右推
                for i in range(self.left_point, 123):
                    if i < 127 and self.binary_data[i] == 1 and self.binary_data[i + 1] == 0:
                        self.right_point = i
                        self.right_lost = False
                        break

        # 边界合法性检查 (落在边缘视为丢失)
        if self.left_point <= self.inv_range:
            self.left_lost = True
        if self.right_point >= 127 - self.inv_range:
            self.right_lost = True

        # 丢失时边界取极值 (中点会偏向另一侧, 实现丢线跟随)
        self.left_point = 0 if self.left_lost else self.left_point
        self.right_point = 127 if self.right_lost else self.right_point

    def is_valid(self):
        """CCD数据是否有效 (max_value 高于保护阈值, 未冲出赛道)"""
        return self.max_value > self.protect_value

    def draw(self, lcd, y_pos, name, colors):
        """在LCD指定位置绘制CCD波形与边界标记
        lcd:    LCD对象
        y_pos:  绘制起始Y坐标
        name:   显示名称 (如 "CCD0-NEAR")
        colors: dict, 含 YELLOW/RED/GREEN/BLUE/WHITE 颜色值
        """
        YELLOW = colors['YELLOW']
        RED = colors['RED']
        GREEN = colors['GREEN']
        BLUE = colors['BLUE']
        WHITE = colors['WHITE']

        # 标题
        lcd.str16(0, y_pos, name, YELLOW)
        # 波形 (128宽, 40高)
        lcd.wave(0, y_pos + 16, 128, 40, self.raw_data, max=255)

        # 边界线
        lcd.line(self.left_point, y_pos + 16, self.left_point, y_pos + 56, color=RED, thick=2)
        lcd.line(self.right_point, y_pos + 16, self.right_point, y_pos + 56, color=GREEN, thick=2)
        lcd.line(self.mid_point, y_pos + 16, self.mid_point, y_pos + 56, color=BLUE, thick=1)
        lcd.line(self.expect_mid, y_pos + 16, self.expect_mid, y_pos + 56, color=YELLOW, thick=1)

        # 参数显示 (右侧)
        lcd.str16(130, y_pos, "Err{:3d}".format(self.err), WHITE)
        lcd.str16(130, y_pos + 15, "W:{:3d}".format(self.track_width), WHITE)
        lcd.str16(130, y_pos + 30, "Th:{:3d}".format(self.threshold), WHITE)
        lcd.str16(130, y_pos + 45, "Av:{:3d}".format(self.avg_brightness), WHITE)
        lcd.str16(180, y_pos, "Max{:3d}".format(self.max_value), WHITE)
        lcd.str16(180, y_pos + 15, "Min{:3d}".format(self.min_value), WHITE)
        lcd.str16(180, y_pos + 30, "L{:3d}".format(self.left_point), WHITE)
        lcd.str16(180, y_pos + 45, "R{:3d}".format(self.right_point), WHITE)

        # 边界状态
        s0 = "L:" + ("LOST" if self.left_lost else "OK  ")
        s1 = "R:" + ("LOST" if self.right_lost else "OK  ")
        lcd.str16(0, y_pos + 58, s0, RED if self.left_lost else GREEN)
        lcd.str16(64, y_pos + 58, s1, RED if self.right_lost else GREEN)
