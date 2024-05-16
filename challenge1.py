import rclpy  # ROS client library
import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LENGTH= 0.281
WIDTH=  0.306
PADDING_DIST=0.03
MAX_VEL=20
ACELERATION=2
DECELERATION=2

plt.figure()
plt.figure()


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type -The Twist message typically contains linear and angular velocities, often used in robotic motion control.
                'cmd_vel',  # topic name
                1)          # history depth- The history depth determines how many messages will be stored in the topic's history. 

        self.scan_sub = self.create_subscription(
                LaserScan,                   # message type
                'scan',                      # topic name
                self.scan_callback,          # function to run upon message arrival
                qos_profile_sensor_data)     # allows packet loss

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.Prev_Dist=0

        self.Init_Vel=0

        self.Counter1=0
        self.Counter2=0
        
        

        self.x=[]
        self.y=[]
        self.z=[]

        self.New_Vel=0

        self.max_vel=(MAX_VEL*0.26)/100
        self.deceleration=(DECELERATION*0.26)/100
        self.aceleration=(ACELERATION*0.26)/100
        self.Dec_Dist=((self.max_vel*self.max_vel)/(2*self.deceleration))+PADDING_DIST #o start deceleration for stoping the vehicle without hitting the wall 
        
        
     

      

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
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
        """ is run whenever a LaserScan msg is received
        """
        print()
        print('Distances:')
        print('⬆️ :', msg.ranges[0])
        print('⬇️ :', msg.ranges[180])
        print('⬅️ :', msg.ranges[90])
        print('➡️ :', msg.ranges[-90])


        Forward_Dist=msg.ranges[0]
        
        
        self.Avg_Dist=(Forward_Dist+self.Prev_Dist)/2
        self.Prev_Dist=self.Avg_Dist
        print("Forward distance", Forward_Dist)
        print("Average Distance", self.Avg_Dist)
        print(self.Avg_Dist-Forward_Dist)
        print("Minimum Distance for Deceleration :", self.Dec_Dist)
        self.x.append(self.New_Vel) 
        self.z.append(Forward_Dist)
        self.y.append(self.Counter1)
        print("forward dist",self.Avg_Dist)
        print("Dec dist",self.Dec_Dist)
        
        
        if self.Avg_Dist<=self.Dec_Dist:
            if self.New_Vel<=0:
                self.New_Vel=0
                print("Vehicle stopped")
                
            else:
                print("Deceleration Started")
                self.New_Vel=self.Init_Vel-(DECELERATION) # FInding decreasing velocity instantaniously
                self.Init_Vel=self.New_Vel
                
                print("Deceleration:", self.New_Vel)
                  
        else:
            if self.New_Vel!=MAX_VEL:
                self.New_Vel=self.Init_Vel+(ACELERATION) # Finding acelerating velocity instantaniously
                self.Init_Vel=self.New_Vel
                print("Initial vel", self.Init_Vel)
                print("Aceleration started velocity:", self.New_Vel)
                print("Counter1",self.Counter1)
            else:
                print("Constand Speed",self.New_Vel)   


               
        
               
        self.Counter1 += 1                  # Increasing the Counter1
        plt.figure(1)
        self.vel(self.New_Vel) 
        plt.plot(self.y,self.x,color='green')
        plt.xlabel('Time')
        plt.ylabel('Velocity')

        plt.figure(2)
        plt.plot(self.y,self.z,color='red')
        plt.xlabel('Time')
        plt.ylabel('Distance')

        
       






def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')
    
    try:
        rclpy.spin(tb3)     # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        plt.show()

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
