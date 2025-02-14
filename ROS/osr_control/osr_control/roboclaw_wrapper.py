import math
from collections import defaultdict

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from osr_control.roboclaw import Roboclaw

from sensor_msgs.msg import JointState
from osr_interfaces.msg import CommandDrive, Status

############################################################################################
serial1 = ""
serial2 = ""
serial3 = ""

STOP_MSG = "@0000 0000en"
############################################################################################


class RoboclawWrapper(Node):
    """Interface between the roboclaw motor drivers and the higher level rover code"""

    def __init__(self):
        super().__init__("roboclaw_wrapper")
        self.log = self.get_logger()
        self.log.info("Initializing motor controllers")

        # initialize attributes
        self.rc = []
        self.err = [None] * 3
        self.address = []
        self.current_enc_vals = None
        self.drive_cmd_buffer = None

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("baud_rate", Parameter.Type.INTEGER),
                ("device", Parameter.Type.STRING),
                ("addresses", Parameter.Type.INTEGER_ARRAY),
                # ('roboclaw_mapping', Parameter.Type.INTEGER_ARRAY),
                ("drive_acceleration_factor", Parameter.Type.DOUBLE),
                ("corner_acceleration_factor", Parameter.Type.DOUBLE),
                ("velocity_timeout", Parameter.Type.DOUBLE),
                ("duty_mode", Parameter.Type.BOOL),
                ("velocity_qpps_to_duty_factor", Parameter.Type.INTEGER),
                ("roboclaw_mapping.drive_left_front.address", Parameter.Type.INTEGER),
                ("roboclaw_mapping.drive_left_front.channel", Parameter.Type.STRING),
                (
                    "roboclaw_mapping.drive_left_front.ticks_per_rev",
                    Parameter.Type.INTEGER,
                ),
                ("roboclaw_mapping.drive_left_front.gear_ratio", Parameter.Type.DOUBLE),
                ("roboclaw_mapping.drive_left_middle.address", Parameter.Type.INTEGER),
                ("roboclaw_mapping.drive_left_middle.channel", Parameter.Type.STRING),
                (
                    "roboclaw_mapping.drive_left_middle.ticks_per_rev",
                    Parameter.Type.INTEGER,
                ),
                (
                    "roboclaw_mapping.drive_left_middle.gear_ratio",
                    Parameter.Type.DOUBLE,
                ),
                ("roboclaw_mapping.drive_left_back.address", Parameter.Type.INTEGER),
                ("roboclaw_mapping.drive_left_back.channel", Parameter.Type.STRING),
                (
                    "roboclaw_mapping.drive_left_back.ticks_per_rev",
                    Parameter.Type.INTEGER,
                ),
                ("roboclaw_mapping.drive_left_back.gear_ratio", Parameter.Type.DOUBLE),
                ("roboclaw_mapping.drive_right_front.address", Parameter.Type.INTEGER),
                ("roboclaw_mapping.drive_right_front.channel", Parameter.Type.STRING),
                (
                    "roboclaw_mapping.drive_right_front.ticks_per_rev",
                    Parameter.Type.INTEGER,
                ),
                (
                    "roboclaw_mapping.drive_right_front.gear_ratio",
                    Parameter.Type.DOUBLE,
                ),
                ("roboclaw_mapping.drive_right_middle.address", Parameter.Type.INTEGER),
                ("roboclaw_mapping.drive_right_middle.channel", Parameter.Type.STRING),
                (
                    "roboclaw_mapping.drive_right_middle.ticks_per_rev",
                    Parameter.Type.INTEGER,
                ),
                (
                    "roboclaw_mapping.drive_right_middle.gear_ratio",
                    Parameter.Type.DOUBLE,
                ),
                ("roboclaw_mapping.drive_right_back.address", Parameter.Type.INTEGER),
                ("roboclaw_mapping.drive_right_back.channel", Parameter.Type.STRING),
                (
                    "roboclaw_mapping.drive_right_back.ticks_per_rev",
                    Parameter.Type.INTEGER,
                ),
                ("roboclaw_mapping.drive_right_back.gear_ratio", Parameter.Type.DOUBLE),
            ],
        )

        """self.roboclaw_mapping = defaultdict(dict)
        self.roboclaw_mapping["drive_left_front"]["address"] = (
            self.get_parameter("roboclaw_mapping.drive_left_front.address")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_left_middle"]["address"] = (
            self.get_parameter("roboclaw_mapping.drive_left_middle.address")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_left_back"]["address"] = (
            self.get_parameter("roboclaw_mapping.drive_left_back.address")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_right_front"]["address"] = (
            self.get_parameter("roboclaw_mapping.drive_right_front.address")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_right_middle"]["address"] = (
            self.get_parameter("roboclaw_mapping.drive_right_middle.address")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_right_back"]["address"] = (
            self.get_parameter("roboclaw_mapping.drive_right_back.address")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_left_front"]["channel"] = (
            self.get_parameter("roboclaw_mapping.drive_left_front.channel")
            .get_parameter_value()
            .string_value
        )
        self.roboclaw_mapping["drive_left_middle"]["channel"] = (
            self.get_parameter("roboclaw_mapping.drive_left_middle.channel")
            .get_parameter_value()
            .string_value
        )
        self.roboclaw_mapping["drive_left_back"]["channel"] = (
            self.get_parameter("roboclaw_mapping.drive_left_back.channel")
            .get_parameter_value()
            .string_value
        )
        self.roboclaw_mapping["drive_right_front"]["channel"] = (
            self.get_parameter("roboclaw_mapping.drive_right_front.channel")
            .get_parameter_value()
            .string_value
        )
        self.roboclaw_mapping["drive_right_middle"]["channel"] = (
            self.get_parameter("roboclaw_mapping.drive_right_middle.channel")
            .get_parameter_value()
            .string_value
        )
        self.roboclaw_mapping["drive_right_back"]["channel"] = (
            self.get_parameter("roboclaw_mapping.drive_right_back.channel")
            .get_parameter_value()
            .string_value
        )
        self.roboclaw_mapping["drive_left_front"]["ticks_per_rev"] = (
            self.get_parameter("roboclaw_mapping.drive_left_front.ticks_per_rev")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_left_middle"]["ticks_per_rev"] = (
            self.get_parameter("roboclaw_mapping.drive_left_middle.ticks_per_rev")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_left_back"]["ticks_per_rev"] = (
            self.get_parameter("roboclaw_mapping.drive_left_back.ticks_per_rev")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_right_front"]["ticks_per_rev"] = (
            self.get_parameter("roboclaw_mapping.drive_right_front.ticks_per_rev")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_right_middle"]["ticks_per_rev"] = (
            self.get_parameter("roboclaw_mapping.drive_right_middle.ticks_per_rev")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_right_back"]["ticks_per_rev"] = (
            self.get_parameter("roboclaw_mapping.drive_right_back.ticks_per_rev")
            .get_parameter_value()
            .integer_value
        )
        self.roboclaw_mapping["drive_left_front"]["gear_ratio"] = (
            self.get_parameter("roboclaw_mapping.drive_left_front.gear_ratio")
            .get_parameter_value()
            .double_value
        )
        self.roboclaw_mapping["drive_left_middle"]["gear_ratio"] = (
            self.get_parameter("roboclaw_mapping.drive_left_middle.gear_ratio")
            .get_parameter_value()
            .double_value
        )
        self.roboclaw_mapping["drive_left_back"]["gear_ratio"] = (
            self.get_parameter("roboclaw_mapping.drive_left_back.gear_ratio")
            .get_parameter_value()
            .double_value
        )
        self.roboclaw_mapping["drive_right_front"]["gear_ratio"] = (
            self.get_parameter("roboclaw_mapping.drive_right_front.gear_ratio")
            .get_parameter_value()
            .double_value
        )
        self.roboclaw_mapping["drive_right_middle"]["gear_ratio"] = (
            self.get_parameter("roboclaw_mapping.drive_right_middle.gear_ratio")
            .get_parameter_value()
            .double_value
        )
        self.roboclaw_mapping["drive_right_back"]["gear_ratio"] = (
            self.get_parameter("roboclaw_mapping.drive_right_back.gear_ratio")
            .get_parameter_value()
            .double_value
        )"""

        self.serial_port_list = ["/dev/ttyCH341USB0"]
        self.encoder_limits = {}
        self.establish_roboclaw_connections()
        self.log.info("try to stop motors")
        self.stop_motors()  # don't move at start
        # self.setup_encoders()

        # self.log.info("Reading parameters")

        # save settings to non-volatile (permanent) memory
        """for address in self.address:
            self.rc.WriteNVM(address)

        for address in self.address:
            self.rc.ReadNVM(address)"""

        # Even though the actual method takes longs (2*32-1), roboclaw blog says 2**15 is 100%
        """accel_max = 2**15 - 1
        self.roboclaw_overflow = 2**15 - 1"""
        # 电机加速度，待定
        accel_max = 2**15 - 1
        accel_rate = (
            self.get_parameter("drive_acceleration_factor")
            .get_parameter_value()
            .double_value
        )
        self.drive_accel = int(accel_max * accel_rate)
        self.velocity_timeout = rclpy.duration.Duration(
            seconds=self.get_parameter("velocity_timeout")
            .get_parameter_value()
            .double_value,
            nanoseconds=0,
        )
        self.velocity_qpps_to_duty_factor = (
            self.get_parameter("velocity_qpps_to_duty_factor")
            .get_parameter_value()
            .integer_value
        )
        self.time_last_cmd = self.get_clock().now()

        self.stop_motors()

        # set up publishers and subscribers
        self.drive_cmd_sub = self.create_subscription(
            CommandDrive, "/cmd_drive", self.drive_cmd_cb, 1
        )  # 订阅电机控制命令
        self.enc_pub = self.create_publisher(
            JointState, "/drive_state", 1
        )  # 发布电机状态，fastupdate
        self.status_pub = self.create_publisher(
            Status, "/status", 1
        )  # 发布状态，slowupdate

        self.status = Status()
        fast_loop_rate = 0.125  # seconds
        slow_loop_rate = 3  # seconds
        # true if we're idling and started ramping down velocity to bring the motors to full stop
        self.idle_ramp = False
        # if we're idled
        self.idle = False
        self.fast_timer = self.create_timer(fast_loop_rate, self.fast_update)
        self.slow_timer = self.create_timer(slow_loop_rate, self.slow_update)
        self.log.info("Initialized Roboclaw wrapper node")

    # 回调函数，待定
    def parameters_callback(self, params):
        """Called when a parameter is created or updated."""
        for param in params:
            if param.value is None:
                continue
            if param.name == "duty_mode":
                self.duty_mode = param.value
                self.get_logger().info(
                    f"Duty mode is {'enabled' if param.value else 'disabled'}"
                )
        return SetParametersResult(successful=True, reason="OK")

    def fast_update(self):
        """Read from and write to roboclaws"""
        # self.log.info("Start fast update")
        # 检查缓冲区中是否有命令，有则发送
        now = self.get_clock().now()
        if self.drive_cmd_buffer:
            drive_fcn = self.send_drive_buffer_velocity
            drive_fcn(self.drive_cmd_buffer)
            self.drive_cmd_buffer = None
            self.idle_ramp = False
            self.idle = False
            self.time_last_cmd = now

        # read from roboclaws and publish
        try:
            self.read_encoder_values()
            self.enc_pub.publish(self.current_enc_vals)
        except AssertionError as read_exc:
            self.get_logger().warn("Failed to read encoder values")
            self.get_logger().warn(read_exc.args)

        # stop the motors if we haven't received a command in a while
        if not self.idle and (now - self.time_last_cmd > self.velocity_timeout):
            # rather than a hard stop, send a ramped velocity command to 0
            if not self.idle_ramp:
                self.get_logger().info("Idling: ramping down velocity to zero")
                self.idle_ramp = True
                drive_cmd_buffer = CommandDrive()
                self.send_drive_buffer_velocity(drive_cmd_buffer)
            # if we've already ramped down, send a full stop to minimize
            # idle power consumption
            else:
                self.get_logger().info("Idling: full stopping motors")
                self.stop_motors()
                self.idle = True

            # so that's there's a delay between ramping and full stop
            self.time_last_cmd = now
            # self.log.info("special fast update finished.")
        
        

    def slow_update(self):
        # self.log.info("slow update start")
        # Slower roboclaw read/write cycle
        """self.status.battery = self.read_battery()
        self.status.temp = self.read_temperatures()
        self.status.current = self.read_currents()
        self.status.error_status = self.read_errors()"""
        # 此项目中不需要使用慢更新获取的参数，因此直接按对应类型置零
        self.status.battery = 0.0  # 浮点数
        self.status.error_status = ["", "", ""]  # 长度为 3 的字符串数组
        self.status.temp = [0.0, 0.0, 0.0]  # 长度为 3 的浮点数数组
        self.status.current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 长度为 6 的浮点数数组
        self.status_pub.publish(self.status)
        # self.log.info("slow update finished")

    def establish_roboclaw_connections(self):
        """
        Attempt connecting to the roboclaws

        :raises Exception: when connection to one or more of the roboclaws is unsuccessful
        """
        # 初始化三个串口上的驱动板
        """front_port = (
                self.get_parameter("device.front").get_parameter_value().string_value
            )
        middle_port = (
                self.get_parameter("device.middle").get_parameter_value().string_value
            )
        back_port = (
                self.get_parameter("device.back").get_parameter_value().string_value
            )"""
        # serial_port_list = [front_port, middle_port, back_port]
        # self.serial_port_list = ["/dev/ttyCH341USB0"]

        for i in range(0, len(self.serial_port_list)):
            # print(f"len(self.serial_port_list) = {len(self.serial_port_list)}    i = {i}")
            serial_port = self.serial_port_list[i]
            baud_rate = (
                self.get_parameter("baud_rate").get_parameter_value().integer_value
            )
            self.rc.append(Roboclaw(serial_port, baud_rate))
            serial_connected = self.rc[i].Open() == 1
            if not serial_connected:
                msg = f"Unable to connect to serial port {serial_port}."
                self.log.fatal(msg)
                raise Exception(msg)

            self.address = (
                self.get_parameter("addresses")
                .get_parameter_value()
                .integer_array_value
            )

        # to do: say hello
        # initialize connection status to successful
        all_connected = True
        """for address in self.address:
            self.get_logger().debug(
                f"Attempting to talk to motor controller {address} through serial port {serial_port} at a {baud_rate} baud_rate."
            )
            version_response = self.rc.ReadVersion(address)
            self.log.debug(f"response for RC at {address}: {version_response}")
            connected = bool(version_response[0])
            if not connected:
                self.get_logger().error(
                    "Unable to connect to roboclaw at '{}' through serial port {} at a {}".format(
                        address, serial_port, baud_rate
                    )
                )
                all_connected = False
            else:
                self.get_logger().debug(
                    "Roboclaw version for address '{}': '{}'".format(
                        address, version_response[1]
                    )
                )"""
        if all_connected:
            self.get_logger().info(
                "Sucessfully connected to RoboClaw motor controllers"
            )
        else:
            raise Exception(
                "Unable to establish connection to one or more of the Roboclaw motor controllers"
            )

    """
    def setup_encoders(self):
    # Set up the encoders
    for motor_name, properties in self.roboclaw_mapping.items():
        self.encoder_limits[motor_name] = (None, None)
        self.rc.ResetEncoders(properties["address"])
    """
    
    # 更新电机参数
    def read_encoder_values(self):
        """Query roboclaws and update current motors status in encoder ticks"""
        motor_name_list = [["drive_left_front","drive_right_front"],["drive_left_middle","drive_right_middle"],["drive_left_back","drive_right_back"]]


        enc_msg = JointState()
        enc_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(0, len(self.serial_port_list)):
            speed = self.AGV_Read_Speed(i)
            enc_msg.name.append(motor_name_list[i][0])
            # position，似乎没用上，先直接取0
            enc_msg.position.append(0)
            # velocity，即为速度
            enc_msg.velocity.append(speed[0])
            # current，电流，在此项目中不使用
            enc_msg.effort.append(0)

            enc_msg.name.append(motor_name_list[i][0])
            enc_msg.position.append(0)
            enc_msg.velocity.append(speed[1])
            enc_msg.effort.append(0)

        self.current_enc_vals = enc_msg

    def drive_cmd_cb(self, cmd):
        """
        Takes the drive command and stores it in the buffer to be sent
        on the next iteration of the run() loop.
        """

        self.get_logger().debug("Drive command callback received: {}".format(cmd))
        self.drive_cmd_buffer = cmd

    # 将velocity信号转换为项目采用的百分制简单占空比
    # velocity信号的值随teleop_twist_joy的配置文件而改变，为float类型
    def velocity2percent(self, velocity):
        # 假设velocity为-10~10
        speed = round(velocity * 10)

        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100

        return speed

    # 发送速度
    def send_drive_buffer_velocity(self, cmd):
        """
        Sends the drive command to the motor controller, closed loop velocity commands
        """
        self.log.info("try to send drive_buffer_velocity")

        left_speed = self.velocity2percent(cmd.left_front_vel)
        right_speed = self.velocity2percent(cmd.right_front_vel * -1) # 右边的轮子控制速度与左轮相反，为负值，很神秘
        self.AGV_Set_Speed_Meta(0, left_speed, right_speed)

        left_speed = self.velocity2percent(cmd.left_middle_vel)
        right_speed = self.velocity2percent(cmd.right_middle_vel * -1)
        self.AGV_Set_Speed_Meta(1, left_speed, right_speed)

        left_speed = self.velocity2percent(cmd.left_back_vel)
        right_speed = self.velocity2percent(cmd.right_back_vel * -1)
        self.AGV_Set_Speed_Meta(2, left_speed, right_speed)

        self.log.info("send drive_buffer_velocity finished")

    #####################################################################################################################
    # 绝对位置刻度与中间位置弧度互换
    def tick2position(self, tick, enc_min, enc_max, ticks_per_rev, gear_ratio):
        """
        Convert the absolute position from ticks to radian relative to the middle position

        :param tick:
        :param enc_min:
        :param enc_max:
        :param ticks_per_rev:
        :return:
        """
        ticks_per_rad = ticks_per_rev / (2 * math.pi)
        if enc_min is None or enc_max is None:
            return tick / ticks_per_rad
        mid = enc_min + (enc_max - enc_min) / 2

        return (tick - mid) / ticks_per_rad * gear_ratio

    def position2tick(self, position, enc_min, enc_max, ticks_per_rev, gear_ratio):
        """
        Convert the absolute position from radian relative to the middle position to ticks

                Clip values that are outside the range [enc_min, enc_max]

        :param position:
        :param enc_min:
        :param enc_max:
        :param ticks_per_rev:
        :return:
        """
        ticks_per_rad = ticks_per_rev / (2 * math.pi)
        if enc_min is None or enc_max is None:
            return position * ticks_per_rad
        mid = enc_min + (enc_max - enc_min) / 2
        tick = int(mid + position * ticks_per_rad / gear_ratio)

        return max(enc_min, min(enc_max, tick))

    #####################################################################################################################
    # 速度与每秒脉冲数互换
    def qpps2velocity(self, qpps, ticks_per_rev, gear_ratio):
        """
        Convert the given quadrature pulses per second to radian/s

        :param qpps: int
        :param ticks_per_rev:
        :param gear_ratio:
        :return:
        """
        return qpps * 2 * math.pi / (gear_ratio * ticks_per_rev)

    def velocity2qpps(self, velocity, ticks_per_rev, gear_ratio):
        """
        Convert the given velocity to quadrature pulses per second

        :param velocity: rad/s
        :param ticks_per_rev:
        :param gear_ratio:
        :return: int
        """
        return int(velocity * gear_ratio * ticks_per_rev / (2 * math.pi))

    #####################################################################################################################

    def stop_motors(self):
        """Stops all motors on Rover"""
        for i in range(3):
            self.AGV_Set_Speed_Meta(i, 0, 0)

    #####################################################################################################################
    # 项目新增部分函数
    def AGV_Set_Speed_Meta(self, index, M1_Speed, M2_Speed):
        # 测试用
        if index >= len(self.serial_port_list):
            self.get_logger().error(f"AGV_Set_Speed_Meta: index = {index} is out of range, max = {len(self.serial_port_list)}")
            return

        # 第一位1表示反向，后三位为速度
        # 例"0050 1050"
        msg = "{}{:0>3d} {}{:0>3d}".format(1 if M1_Speed < 0 else 0,abs(M1_Speed), 1 if M2_Speed < 0 else 0, abs(M2_Speed))
        self.get_logger().info(f"AGV_Set_Speed_Meta: index = {index}, msg = {msg}")
        self.rc[index].AGV_WriteSpeed(msg)
    
    def AGV_Read_Speed(self, index) -> list:
        speed_msg = self.rc[index].AGV_ReadSpeed()
        left_foward = int(speed_msg[1:2])
        left_speed = int(speed_msg[2:5])
        if left_foward == 1:
            left_speed *= -1

        right_foward = int(speed_msg[6:7])
        right_speed = int(speed_msg[7:10])
        if right_foward == 1:
            right_speed *= -1

        return [left_speed, right_speed]


def main(args=None):
    rclpy.init(args=args)

    wrapper = RoboclawWrapper()

    rclpy.spin(wrapper)
    wrapper.stop_motors()
    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
