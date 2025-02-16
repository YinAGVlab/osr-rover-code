# A short and sweet script to test communication with a single roboclaw motor controller.
# usage
#   $ python roboclawtest.py 128
# Things are working if you don't get an error and you see something like:
# (1, 'USB Roboclaw 2x7a v4.1.34\n')
# (1, 4314, 128)

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

    roboclaw0 = Roboclaw("/dev/ttyCH341USB1", 9600)
    roboclaw1 = Roboclaw("/dev/serial1", 115200)
    connected0 = roboclaw0.Open() == 1
    connected1 = False
    if connected0:
        print("Connected to 0")

        speed_msg = roboclaw0.AGV_ReadSpeed() # 读速度信息串
        print(f"AGV_Read_Speed_Meta: msg = {speed_msg}")

        M1_Speed = 50
        M2_Speed = 50
        msg = "{}{:0>3d} {}{:0>3d}".format(1 if M1_Speed < 0 else 0,abs(M1_Speed), 1 if M2_Speed < 0 else 0, abs(M2_Speed))
        print(f"AGV_Set_Speed_Meta: msg = {msg}")
        roboclaw0.AGV_WriteSpeed(msg)

        

    elif connected1:
        print("Connected to /dev/serial1.")
        print(roboclaw1.ReadVersion())
        print(roboclaw1.ReadEncM1())
    else:
        print(
            "Could not open comport /dev/serial0 or /dev/serial1, make sure it has the correct permissions and is available"
        )
