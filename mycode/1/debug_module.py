# debug_module.py
# 串口命令行调试系统 (替代屏幕菜单, 屏幕坏了也能调试)
# 功能: 命令行调参 + ASCII波形显示 + CCD可视化 + 状态监控
# 使用: 在Thonny控制台输入命令


class DEBUG:
    """串口调试系统

    使用方法:
        debug = DEBUG(params, update_cb)
        debug.attach_imu(imu)
        debug.attach_motor(motor)
        debug.attach_pid(angle_pid, gyro_pid, dir_pid)
        debug.attach_ccd(ccd_near, ccd_mid)

        # 主循环中调用
        debug.process()          # 处理命令行输入
        debug.print_status()     # 周期打印状态 (10Hz)
        debug.print_ccd()        # 周期打印CCD (5Hz)
    """

    def __init__(self, params, update_cb=None):
        self.params = params           # 参数字典引用
        self.update_cb = update_cb     # 参数修改后的同步回调
        self.imu = None
        self.motor = None
        self.angle_pid = None
        self.gyro_pid = None
        self.dir_pid = None
        self.ccd_near = None
        self.ccd_mid = None
        self.running = False           # 运行状态 (主程序同步)

        # 节流计数器 (避免print太频繁卡死)
        self._status_cnt = 0           # 每10次主循环打印一次状态 (10Hz)
        self._ccd_cnt = 0              # 每20次主循环打印一次CCD (5Hz)
        self._ccd_ascii_enable = False # CCD ASCII图默认关闭(占行多)
        self._status_enable = True     # 状态打印默认开启

        # 命令缓冲 (MicroPython input() 可能阻塞, 用非阻塞方式)
        self._cmd_buf = ""

    # ===== 附加各模块引用 =====
    def attach_imu(self, imu):
        self.imu = imu

    def attach_motor(self, motor):
        self.motor = motor

    def attach_pid(self, angle_pid, gyro_pid, dir_pid=None):
        self.angle_pid = angle_pid
        self.gyro_pid = gyro_pid
        self.dir_pid = dir_pid

    def attach_ccd(self, ccd_near, ccd_mid=None):
        self.ccd_near = ccd_near
        self.ccd_mid = ccd_mid

    def set_running(self, running):
        self.running = running

    # ===== 主处理 (主循环调用) =====
    def process(self):
        """处理串口命令 (非阻塞)"""
        try:
            # 非阻塞读取串口 (Thonny REPL)
            # MicroPython的input()会阻塞, 用sys.stdin.read(1)替代
            import sys
            import uselect
            sp = uselect.poll()
            sp.register(sys.stdin, uselect.POLLIN)
            polls = sp.poll(0)   # 0=不等待
            if polls:
                ch = sys.stdin.read(1)
                if ch is not None:
                    if ch == '\r' or ch == '\n':
                        if self._cmd_buf:
                            self._execute(self._cmd_buf.strip())
                            self._cmd_buf = ""
                    elif ch == '\b' or ch == '\x7f':
                        # 退格
                        self._cmd_buf = self._cmd_buf[:-1]
                    else:
                        self._cmd_buf += ch
        except Exception:
            # 出错不阻塞主循环
            pass

    # ===== 命令执行 =====
    def _execute(self, cmd):
        """执行一条命令"""
        cmd = cmd.strip()
        if not cmd:
            return

        parts = cmd.split()
        op = parts[0].lower()

        try:
            if op == 'help' or op == '?':
                self._cmd_help()
            elif op == 'list' or op == 'ls':
                self._cmd_list()
            elif op == 'set':
                self._cmd_set(parts)
            elif op == 'save':
                self._cmd_save()
            elif op == 'load':
                self._cmd_load()
            elif op == 'run':
                self._cmd_run()
            elif op == 'stop':
                self._cmd_stop()
            elif op == 'cal':
                self._cmd_cal()
            elif op == 'status' or op == 'st':
                self._status_enable = not self._status_enable
                print("[{}] 状态打印已{}".format(
                    "ON" if self._status_enable else "OFF",
                    "开启" if self._status_enable else "关闭"))
            elif op == 'ccd':
                self._ccd_ascii_enable = not self._ccd_ascii_enable
                print("[{}] CCD ASCII图已{}".format(
                    "ON" if self._ccd_ascii_enable else "OFF",
                    "开启" if self._ccd_ascii_enable else "关闭"))
            elif op == 'info':
                self._cmd_info()
            else:
                print("未知命令: {} (输入 help 查看帮助)".format(op))
        except Exception as e:
            print("命令执行错误: {}".format(e))

    def _cmd_help(self):
        print("=" * 50)
        print("可用命令:")
        print("  help / ?       - 显示帮助")
        print("  list / ls      - 列出所有参数")
        print("  set <key>=<v>  - 修改参数 (如: set A_Kp=-4.0)")
        print("  save           - 保存参数到flash")
        print("  load           - 从flash加载参数")
        print("  run            - 启动平衡控制")
        print("  stop           - 停止平衡控制")
        print("  cal            - IMU校准 (5秒静止)")
        print("  status / st    - 开/关状态打印")
        print("  ccd            - 开/关CCD ASCII图")
        print("  info           - 显示当前状态")
        print("=" * 50)

    def _cmd_list(self):
        print("----- 参数列表 -----")
        for k, v in self.params.items():
            if isinstance(v, float):
                print("  {:12s} = {:.3f}".format(k, v))
            else:
                print("  {:12s} = {}".format(k, v))
        print("--------------------")

    def _cmd_set(self, parts):
        if len(parts) < 2:
            print("用法: set <参数名>=<值>  (如: set A_Kp=-4.0)")
            return
        arg = parts[1]
        if '=' not in arg:
            print("格式错误, 需要: set <参数名>=<值>")
            return
        key, val_str = arg.split('=', 1)
        key = key.strip()
        if key not in self.params:
            print("未知参数: {} (输入list查看可用参数)".format(key))
            return
        try:
            old = self.params[key]
            # 自动类型转换
            if '.' in val_str or 'e' in val_str.lower():
                new_val = float(val_str)
            else:
                new_val = int(val_str)
            self.params[key] = new_val
            print("[OK] {} = {} -> {}".format(key, old, new_val))
            # 触发同步回调
            if self.update_cb:
                self.update_cb()
        except ValueError:
            print("数值格式错误: {}".format(val_str))

    def _cmd_save(self):
        if self.update_cb:
            self.update_cb()
        try:
            import os, io
            os.chdir("/flash")
            with io.open("params.txt", "w") as f:
                for k, v in self.params.items():
                    f.write("{}={}\n".format(k, v))
            print("[OK] 参数已保存到 /flash/params.txt")
        except Exception as e:
            print("[ERR] 保存失败: {}".format(e))

    def _cmd_load(self):
        try:
            import os, io
            os.chdir("/flash")
            with io.open("params.txt", "r") as f:
                for line in f:
                    line = line.strip()
                    if line and "=" in line and not line.startswith('#'):
                        k, v = line.split("=", 1)
                        try:
                            self.params[k.strip()] = float(v.strip())
                        except:
                            pass
            print("[OK] 参数已加载")
            if self.update_cb:
                self.update_cb()
            self._cmd_list()
        except Exception as e:
            print("[ERR] 加载失败: {}".format(e))

    def _cmd_run(self):
        if self.imu is None:
            print("[ERR] IMU未连接")
            return
        if not self.imu.calibrated:
            print("[WARN] IMU未校准, 请先静止后输入: cal")
        if self.angle_pid:
            self.angle_pid.reset()
        if self.gyro_pid:
            self.gyro_pid.reset()
        if self.dir_pid:
            self.dir_pid.reset()
        self.running = True
        print("[RUN] 平衡控制已启动 (输入stop停止)")

    def _cmd_stop(self):
        self.running = False
        if self.motor:
            self.motor.set_duty(0, 0)
        print("[STOP] 平衡控制已停止")

    def _cmd_cal(self):
        if self.imu is None:
            print("[ERR] IMU未连接")
            return
        self.running = False
        if self.motor:
            self.motor.set_duty(0, 0)
        print("[CAL] IMU校准中, 请保持小车静止 5秒...")
        self.imu.calibrate(5000)
        print("[OK] IMU校准完成")

    def _cmd_info(self):
        print("===== 当前状态 =====")
        print("  运行: {}".format("是" if self.running else "否"))
        if self.imu:
            gyro_dps = getattr(self.imu, 'gyro_y_filt', self.imu.gyro_y) * self.imu.gyro_scale
            print("  角度: {:8.2f} deg".format(self.imu.angle))
            print("  角速: {:8.2f} dps".format(gyro_dps))
            print("  校准: {}".format("是" if self.imu.calibrated else "否"))
        if self.ccd_near:
            print("  CCD误差: {:4d}".format(self.ccd_near.err))
            print("  CCD中点: {:4d}".format(self.ccd_near.mid_point))
            print("  赛道宽: {:4d}".format(self.ccd_near.track_width))
            print("  CCD有效: {}".format("是" if self.ccd_near.is_valid() else "否"))
        if self.angle_pid:
            print("  A.Kp={:.2f} A.Kd={:.2f}".format(
                self.angle_pid.kp, self.angle_pid.kd))
        if self.gyro_pid:
            print("  G.Kp={:.2f} G.Ki={:.2f}".format(
                self.gyro_pid.kp, self.gyro_pid.ki))
        print("====================")

    # ===== 周期打印 (主循环调用) =====
    def print_status(self):
        """周期性打印状态数据 (建议10Hz)"""
        if not self._status_enable:
            return
        self._status_cnt += 1
        if self._status_cnt < 10:
            return
        self._status_cnt = 0

        if self.imu is None:
            return
        gyro_dps = getattr(self.imu, 'gyro_y_filt', self.imu.gyro_y) * self.imu.gyro_scale
        out = 0.0
        if self.gyro_pid:
            out = self.gyro_pid.output
        tgt = self.params.get("T_Angle", 0.0)
        run_flag = "R" if self.running else "S"
        # 紧凑单行格式, 方便观察趋势
        print("[{}]A:{:+7.2f} G:{:+7.1f} O:{:+7.0f} T:{:+5.1f}".format(
            run_flag, self.imu.angle, gyro_dps, out, tgt))

    def print_ccd(self):
        """周期性打印CCD ASCII波形 (建议5Hz)"""
        if not self._ccd_ascii_enable:
            return
        if self.ccd_near is None:
            return
        self._ccd_cnt += 1
        if self._ccd_cnt < 20:
            return
        self._ccd_cnt = 0

        ccd = self.ccd_near
        # ASCII波形: 128像素映射到64字符宽度 (每2像素合并)
        # 高度8级: 0123456789 -> 不同的字符
        # 用 # 表示亮(白赛道), . 表示暗(蓝背景)
        print("----- CCD NEAR -----")
        print("Max:{} Min:{} Th:{} Err:{} Mid:{} L:{} R:{}".format(
            ccd.max_value, ccd.min_value, ccd.threshold,
            ccd.err, ccd.mid_point, ccd.left_point, ccd.right_point))

        # 二值化行 (压缩到64字符)
        line = ""
        for i in range(0, 128, 2):
            line += "#" if ccd.binary_data[i] else "."
        print("BIN:|" + line + "|")

        # 边界标记行
        marks = [' '] * 64
        l64 = ccd.left_point // 2
        r64 = ccd.right_point // 2
        m64 = ccd.mid_point // 2
        if 0 <= l64 < 64:
            marks[l64] = 'L'
        if 0 <= r64 < 64:
            marks[r64] = 'R'
        if 0 <= m64 < 64:
            marks[m64] = 'M'
        print("MRK:|" + ''.join(marks) + "|")
        print("(# = 白赛道, . = 蓝背景, L=左边界 R=右边界 M=中点)")
