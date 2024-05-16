import signal

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 1  # message type  # topic name
        )  # history depth

        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,  # function to run upon message arrival
            qos_profile_sensor_data,
        )  # allows packet loss

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.ref_range = None
        self.des_range = None
        self.des_rotation =90
        self.lds_int_count = 0
        self.init_error = 0

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """Publishes linear and angular velocities in percent"""
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        """Is run whenever a LaserScan msg is received"""
        print()
        # print("Distances:")
        n = len(msg.ranges)
        # print("⬆️ :", msg.ranges[0])
        # print("⬇️ :", msg.ranges[n // 2])
        # print("⬅️ :", msg.ranges[n // 4])
        # print("➡️ :", msg.ranges[-n // 4])

        def control(error: float) -> float:
            """Returns the value used to drive the motors in percent based on the
            error. Error is the difference between actual and desired value. Error may
            be also negative. """
            p = 400
            if (error > 50):
                    p = 10 + np.abs(400 - (error - 50) * 10 )        
            control_percentage = error / n * p
            return control_percentage

        def rotation_error(r_actual: list, r_desired: list) -> float:
            """Returns the rotation error in radian"""
            r_actual = np.array(r_actual)
            r_desired = np.array(r_desired)

            mean_vals = [2 for _ in range(n)]

            for i in range(n):
                positions = i % n  # Ensure positions is within range [0, n)
                r_actual_rotated = np.concatenate((r_actual[-positions:], r_actual[:-positions]))
                mean_vals[i] = np.nanmean(np.abs(r_actual_rotated - np.resize(r_desired, n)))

            min_mean = np.min(mean_vals)
            index = np.where(mean_vals == min_mean)[0][0]

            if (index > n/2):
                error = n - index 
            else:
                error = -index

            print(error)
            return error

        def lds_to_robot(point: tuple[float, float]) -> tuple[float, float]:
            """Transforms the point in lds frame to robot frame."""
            return (point[0] - 0.07, point[1])

        if (self.ref_range  is None):
            self.ref_range = [None for _ in range(n)]
            self.des_range = [None for _ in range(n)]

            for i in range(n):
                point_tuple_lds = (msg.ranges[i] * np.cos(np.radians(i* 360/n)), msg.ranges[i] * np.sin(np.radians(i* 360/n)))
                point_tuple_robot = lds_to_robot(point_tuple_lds)
                self.ref_range[i] = np.sqrt(point_tuple_robot[0]**2 + point_tuple_robot[1]**2) 
                self.des_range[(i - (self.des_rotation* n) // 360) % n] = self.ref_range[i]

        ranges_actual = [None for _ in range(n)]
        for i in range(n):
            point_tuple_lds = (msg.ranges[i] * np.cos(np.radians(i * 360/n)), msg.ranges[i] * np.sin(np.radians(i * 360/n)))
            point_tuple_robot = lds_to_robot(point_tuple_lds)
            ranges_actual[i] = np.sqrt(point_tuple_robot[0]**2 + point_tuple_robot[1]**2) 

        error = rotation_error(ranges_actual, self.des_range)
        ctr = control(error)
        self.vel(0, ctr)

def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()
    print("Waiting for messages...")
    print("If you cannot kill the process using CTRL+C, use CTRL+\\")

    def stop_robot(sig, frame):
        robot_controller.vel(0, 0)
        robot_controller.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT
    rclpy.spin(robot_controller)

if __name__ == "__main__":
    main()
