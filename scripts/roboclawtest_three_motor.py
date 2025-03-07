from time import sleep
import sys
from os import path

# need to add the roboclaw.py file in the path
sys.path.append(
    path.join(
        path.expanduser("~"),
        "/home/raspi/ros/osr-rover-code/src/osr-rover-code/ROS/osr_control/osr_control",
    )
)
from roboclaw_1 import Roboclaw

if __name__ == "__main__":

    # 假设M1、M2、M3的速度值可以从命令行参数传递或者手动设置

    
    M1_Speed_1 = 0
    M2_Speed_1 = 0
    M1_Speed_2 = 0
    M2_Speed_2 = 0
    M3_Speed_1 = 0
    M3_Speed_2 = 0
    
    M1_Speed_1 = M2_Speed_1 = 0
    
    M1_Speed_2 = M2_Speed_2
    M3_Speed_1 = M3_Speed_2
    

    # 控制串口是否开启的标志位（1表示开启，0表示关闭）
    flag0 = 1  # /dev/ttyACM3  
    flag1 = 0  # /dev/ttyACM0
    flag2 = 0  # /dev/ttyACM1

    roboclaw0 = Roboclaw("/dev/ttyACM3", 9600)
    roboclaw1 = Roboclaw("/dev/ttyACM0", 9600)
    roboclaw2 = Roboclaw("/dev/ttyACM1", 9600)

    if flag0:
        connected0 = roboclaw0.Open() == 1
        if connected0:
            print("Connected to /dev/ttyACM3")
            speed_msg_0 = roboclaw0.AGV_ReadSpeed()  # 读取速度信息
            print(f"AGV_Read_Speed_Meta (ACM3): msg = {speed_msg_0}")
            msg_0 = "{}{:0>3d} {}{:0>3d}".format(1 if M1_Speed_1 < 0 else 0, abs(M1_Speed_1), 1 if M2_Speed_1 < 0 else 0, abs(M2_Speed_1))
            print(f"AGV_Set_Speed_Meta (ACM3): msg = {msg_0}")
            roboclaw0.AGV_WriteSpeed(msg_0)
        else:
            print("Could not connect to /dev/ttyACM3")
    else:
        print("/dev/ttyACM3 communication is disabled (flag0=0)")

    if flag1:
        connected1 = roboclaw1.Open() == 1
        if connected1:
            print("Connected to /dev/ttyACM0")
            speed_msg_1 = roboclaw1.AGV_ReadSpeed()  # 读取速度信息
            print(f"AGV_Read_Speed_Meta (ACM0): msg = {speed_msg_1}")
            msg_1 = "{}{:0>3d} {}{:0>3d}".format(1 if M1_Speed_2 < 0 else 0, abs(M1_Speed_2), 1 if M2_Speed_2 < 0 else 0, abs(M2_Speed_2))
            print(f"AGV_Set_Speed_Meta (ACM0): msg = {msg_1}")
            roboclaw1.AGV_WriteSpeed(msg_1)
        else:
            print("Could not connect to /dev/ttyACM0")
    else:
        print("/dev/ttyACM0 communication is disabled (flag1=0)")

    if flag2:
        connected2 = roboclaw2.Open() == 1
        if connected2:
            print("Connected to /dev/ttyACM1")
            speed_msg_2 = roboclaw2.AGV_ReadSpeed()  # 读取速度信息
            print(f"AGV_Read_Speed_Meta (ACM1): msg = {speed_msg_2}")
            msg_2 = "{}{:0>3d} {}{:0>3d}".format(1 if M3_Speed_1 < 0 else 0, abs(M3_Speed_1), 1 if M3_Speed_2 < 0 else 0, abs(M3_Speed_2))
            print(f"AGV_Set_Speed_Meta (ACM1): msg = {msg_2}")
            roboclaw2.AGV_WriteSpeed(msg_2)
        else:
            print("Could not connect to /dev/ttyACM1")
    else:
        print("/dev/ttyACM1 communication is disabled (flag2=0)")

    # 如果没有任何串口连接
    if not (flag0 and connected0) and not (flag1 and connected1) and not (flag2 and connected2):
        print(
            "Could not open any of the comport /dev/ttyACM3, /dev/ttyACM0, or /dev/ttyACM1. "
            "Make sure they have the correct permissions and are available."
        )
