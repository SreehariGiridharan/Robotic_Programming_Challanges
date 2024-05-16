import signal
import rclpy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np

class StateMachine:
    def __init__(self):
        self.current_state = 'moving_forward'
        self.turnbool=True
        self.current_index=-1
        self.state_commands=['rotation_movement','moving_forward','stop']


    def transition_to(self, new_state):
        self.current_state = new_state

    def increment_index(self):
        self.current_index = (self.current_index + 1) % len(self.state_commands)
        self.transition_to(self.state_commands[self.current_index])


    def run(self):
        
        if self.current_state == 'moving_forward':
            self.moving_forward()
        elif self.current_state == 'rotation_movement':
            self.rotation_movement()
        elif self.current_state == 'stop':
            self.vehicle_stop()
        else:
            print(f"Error: Invalid state '{self.current_state}'")
            
    def vehicle_stop(self):
        self.robot_controller.vel(0)

        
    def moving_forward(self):
        
        if self.robot_controller.distance < 0.15:
            print("distance",self.robot_controller.distance)
            self.robot_controller.velocity_value = 20
        else:
            self.robot_controller.velocity_value = 0
            self.increment_index()
            # if self.turnbool:
            #     self.robot_controller.i+=1
            #     print("robot controller i",self.robot_controller.i)
            #     self.turnbool=False
            # self.turnbool=True
        print("Robot velocity",self.robot_controller.velocity_value)
        self.robot_controller.vel(self.robot_controller.velocity_value)

    def rotation_movement(self):
        if self.robot_controller.yaw_turn < 90:
            self.robot_controller.vel(0, -20)
        else:
            self.robot_controller.vel(0, 0)
            self.increment_index()
            self.robot_controller.startbool = True


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 1  # message type  # topic name
        )  # history depth

        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.robot_position = Point()
        self.robot_orientation = Quaternion()
        self.startbool = True
        self.velocity_value = 0
        # self.current_state = 0

        self.i=0

        self.state_machine = StateMachine()
        self.state_machine.robot_controller = self

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

    def odom_callback(self, msg):
        """Callback function for odometry"""
        self.robot_position = msg.pose.pose.position
        self.robot_orientation = msg.pose.pose.orientation
        euler_angles = self.euler_from_quaternion(self.robot_orientation.x, self.robot_orientation.y,
                                                  self.robot_orientation.z, self.robot_orientation.w)
        roll, pitch, yaw = euler_angles
        yaw_degree = yaw * (180 / np.pi)
        print("Yaw-rotating around Z", yaw_degree)
        if self.startbool:
            self.odom_to_robot_start(self.robot_position, yaw_degree)
            self.startbool = False

        self.robot_to_robot_start(self.robot_position, yaw_degree)
        self.state_machine.run()

    def odom_to_robot_start(self, robo_pos, robo_yaw):
        self.robo_start_pos = robo_pos
        self.robo_start_yaw = robo_yaw

    def robot_to_robot_start(self, current_pos, current_yaw):
        self.distance = math.sqrt((current_pos.x - self.robo_start_pos.x) ** 2 +
                                  (current_pos.y - self.robo_start_pos.y) ** 2)
        print("Distance", self.distance)
        self.yaw_turn = abs(current_yaw - self.robo_start_yaw)
        print("Yaw turn", self.yaw_turn)
        return

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


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
