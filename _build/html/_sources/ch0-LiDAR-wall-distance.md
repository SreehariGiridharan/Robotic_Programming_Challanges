# Challenge 0

Find out the minimal distance to the walls.

## Key concepts

**Nodes** 
- Nodes are basic building blocks  in Ros2 , help to make everything modular.

- Each node can perform specific task

- Nodes can communicates by sending and receiving messages over Topics, services and parameters. 

**Topics**

- A Mechanism for a asynchronous communication between nodes

- Nodes can publish and subscribe to a topic

- Multiple nodes can publish and subscribe to same topic simultaneously.

- In this we are using one node `Tb3`, one topic subscribed  `scan` ( for LiDAR ) and one topic published `cmd_vel` ( for giving velocity to actuators). Subscribed message type is `LaserScan` and published message type is `Twist`.

**LiDAR** 
- Light detection and ranging, a method for gauging distances, involves directing a laser towards an object or surface and calculating the duration it takes for the reflected light to return to the sensor.

## Approach


I got the minimal distance to the wall using two ways.

**First approach :**  Run the python program `tb3.py`. It will generate the minimum distance to the each direction of the wall.

![Image](Wall_distance.png)

**Second approach :** From the Topic visualization selecting the scan topic in the gazebo 

![Image](Gazebo_wall_distance.png)

## Result

The minimal distance to each walls are 0.5474, 0.3583, 0.4932, 0.459. 

## Challenges faced 
- Initially, I was using Foxy Fitzroy (Distribution version of ROS2). To update it to Humble Hawksbill (New distribution version), I had to update my Ubuntu version to 22.04 ( Jammy Jellyfish).

