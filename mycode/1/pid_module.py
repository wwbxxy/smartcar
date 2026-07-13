# pid_module.py

class PID_CLASS:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, ei_max=0.0, output_max=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e0 = 0.0          # 当前误差
        self.e1 = 0.0          # 上一次误差
        self.e2 = 0.0          # 上上次误差
        self.ei = 0.0          # 积分累计
        self.ei_max = ei_max   # 积分限幅
        self.output_max = output_max  # 输出限幅
        self.output = 0.0

    def pid_standard_integral(self, error):
        """位置式 PID：output = Kp*e + Ki*∑e + Kd*(e - e_last)"""
        self.e2 = self.e1
        self.e1 = self.e0
        self.e0 = error

        # 积分累加 + 抗饱和限幅
        self.ei += self.e0
        if self.ei_max > 0:
            if self.ei > self.ei_max:
                self.ei = self.ei_max
            elif self.ei < -self.ei_max:
                self.ei = -self.ei_max

        self.output = self.kp * self.e0 + self.ki * self.ei + self.kd * (self.e0 - self.e1)

        # 输出限幅
        if self.output_max > 0:
            if self.output > self.output_max:
                self.output = self.output_max
            elif self.output < -self.output_max:
                self.output = -self.output_max

        return self.output

    def pid_standard_incremental(self, error):
        """增量式 PID：output += Kp*Δe + Ki*e + Kd*Δ²e"""
        self.e2 = self.e1
        self.e1 = self.e0
        self.e0 = error

        delta = (self.kp * (self.e0 - self.e1)
               + self.ki * self.e0
               + self.kd * (self.e0 - 2 * self.e1 + self.e2))
        self.output += delta

        if self.output_max > 0:
            if self.output > self.output_max:
                self.output = self.output_max
            elif self.output < -self.output_max:
                self.output = -self.output_max

        return self.output

    def reset(self):
        """清零积分和误差历史"""
        self.e0 = 0.0
        self.e1 = 0.0
        self.e2 = 0.0
        self.ei = 0.0
        self.output = 0.0
