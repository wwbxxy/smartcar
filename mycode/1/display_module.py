from machine import Pin
from display import LCD_Drv, LCD


class DISPLAY:
    def __init__(self):
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

    def show(self, angle, gyro, output):
        self.lcd.str16(0, 0, "Angle:{:>8.2f}".format(angle), 0x07E0)
        self.lcd.str16(0, 20, "Gyro :{:>8.2f}".format(gyro), 0x001F)
        self.lcd.str16(0, 40, "Out  :{:>8.1f}".format(output), 0xF800)
