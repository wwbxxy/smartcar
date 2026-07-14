from seekfree import MOTOR_CONTROLLER


class MOTOR:
    def __init__(self):
        # 左右电机，使用PWM+DIR驱动方式（DRV8701）
        # 具体引脚组合需要根据实际车模接线确认
        self.motor_left = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty=0, invert=False)
        self.motor_right = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty=0, invert=True)

    def set_duty(self, left_duty, right_duty):
        """设置左右电机占空比，范围±10000"""
        self.motor_left.duty(left_duty)
        self.motor_right.duty(right_duty)

