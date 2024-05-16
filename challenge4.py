import signal
import rclpy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np




class StateMachine:
    def __init__(self):
        self.current_state = 'moving_forward'
        self.turnbool=True
        self.finished=True
        self.rotatebool=True
        self.startrightleft=True
        self.forwardbool=True
        self.j=0

    def set_initial_state(self, initial_state):
        self.current_state = initial_state

    def transition_to(self, new_state):
        self.current_state = new_state

    def main_transition(self, new_state):
        self.main_state = new_state

    def run(self):
     
        if self.main_state == 'forward_right':
          
            self.forward_right()
        elif self.main_state == 'right_left':
            
            
            print("reached 2")
            if self.startrightleft:
                self.transition_to('rotation_movement_right')
                self.startrightleft=False
            self.right_left()
        

########################################################################################################
    def forward_right(self): 
        print("forward right")  
        if self.current_state == 'moving_forward':
            
            self.moving_forward()
        elif self.current_state == 'rotation_movement_right':
            self.rotation_movement_right()
        else:
            print(f"Error: Invalid state '{self.current_state}'")

    def right_left(self):
        print("right_left")
        self.rotatebool=False
        if self.current_state == 'moving_forward':
            print("moving_forward")
            self.moving_forward()
        elif self.current_state == 'rotation_movement_left':
            print("rotation_movement_left")
            self.rotation_movement_left()
        elif self.current_state == 'rotation_movement_right':
            print("rotation_movement_right")
            self.rotation_movement_right()
        else:
            print(f"Error: Invalid state '{self.current_state}'")



###################################################################################################
    def moving_forward(self):
        if self.j<2:
            if self.robot_controller.distance < 1.1:
                print("distance",self.robot_controller.distance)
                self.robot_controller.velocity_value = 50
            else:
                self.robot_controller.velocity_value = 0
                print("reached -1")
                if self.forwardbool:
                    self.j+=1
                    print("j is",self.j)
                    self.forwardbool=False
                self.forwardbool=True
                # self.transition_to('rotation_movement_left')
                if self.rotatebool:
                    self.transition_to('rotation_movement_right')
                    print("reached 0")
                else:
                    self.transition_to('rotation_movement_left')
                    print("reached 1")
                if self.turnbool:
                    self.robot_controller.i+=1
                    # print("robot controller i",self.robot_controller.i)
                    self.turnbool=False
                self.turnbool=True
            # print("Robot velocity",self.robot_controller.velocity_value)
            self.robot_controller.vel(self.robot_controller.velocity_value)


    def rotation_movement_right(self):
        if self.j<2:
            if self.robot_controller.yaw_turn < 90:
                self.robot_controller.vel(0, -20)
            else:
                self.robot_controller.vel(0, 0)
                self.transition_to('moving_forward')
                self.robot_controller.startbool = True

    def rotation_movement_left(self):
        if self.j<2:
            if self.robot_controller.yaw_turn < 90:
                self.robot_controller.vel(0, 20)
            else:
                self.robot_controller.vel(0, 0)
                self.transition_to('moving_forward')
                self.robot_controller.startbool = True
########################################################################################################


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

        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,  # function to run upon message arrival
            qos_profile_sensor_data,
        )  # allows packet loss

        self.startscanbool=True

        self.robot_position = Point()
        self.robot_orientation = Quaternion()
        self.startbool = True
        self.velocity_value = 0
        # self.current_state = 0

        self.i=0
        self.wall_limit=1.3

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
        # print("Yaw-rotating around Z", yaw_degree)
        if self.startbool:
            self.odom_to_robot_start(self.robot_position, yaw_degree)
            self.startbool = False
        self.robot_to_robot_start(self.robot_position, yaw_degree)


        # if self.i<3:
        #     print("i",self.i)
        #     self.state_machine.run()

    def odom_to_robot_start(self, robo_pos, robo_yaw):
        self.robo_start_pos = robo_pos
        self.robo_start_yaw = robo_yaw

    def robot_to_robot_start(self, current_pos, current_yaw):
        self.distance = math.sqrt((current_pos.x - self.robo_start_pos.x) ** 2 +
                                  (current_pos.y - self.robo_start_pos.y) ** 2)
        # print("Distance", self.distance)
        self.yaw_turn = abs(current_yaw - self.robo_start_yaw)
        # print("Yaw turn", self.yaw_turn)
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

    def scan_callback(self, msg):
        """Is run whenever a LaserScan msg is received"""
        print()
        # print("Distances:")
        
        # print("⬆️ :", msg.ranges[0])
        # print("⬇️ :", msg.ranges[n // 2])
        # print("⬅️ :", msg.ranges[n // 4])
        # print("➡️ :", msg.ranges[-n // 4])
        if self.startscanbool:
            init_scan=msg.ranges
            self.splitting_direction(init_scan)
            self.startscanbool=False
        self.wall_check()

    
    def wall_check(self):
        if(self.forward_dist>self.wall_limit and self.angled_dist>self.wall_limit):
            self.state_machine.main_transition('forward_right')
            self.wall_limit=0
        else:
            self.state_machine.main_transition('right_left')
            self.wall_limit=10
        self.state_machine.run()


    def splitting_direction(self,input_array):

        n = len(input_array)
        self.forward_dist=input_array[0]
        self.angled_dist=input_array[int(n/1.0909)] 
        print("forward dist",self.forward_dist)
        print("angled dist",self.angled_dist)

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
