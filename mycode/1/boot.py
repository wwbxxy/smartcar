# boot.py
from machine import *
import time

time.sleep_ms(50)  # 上电启动延时

boot_select = Pin('D8', Pin.IN, pull=Pin.PULL_UP_47K)

# 拨码开关打开（引脚拉低）→ 运行用户文件
# 拨码开关关闭（引脚拉高）→ 跳过用户文件，直接进 REPL
if boot_select.value() == 0:
    try:
        os.chdir("/flash")
        execfile("user_main.py")
    except:
        print("File not found.")
