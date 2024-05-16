# Challenge 1

Drive forward the robot near to the wall in front of robot without colliding. The movement should be smooth (both acceleration and the deceleration) 

## Key concepts

**The third equation** of motion is given by
![Image](velcityequation.png)

Where:

- v is the final velocity of the object,
- u is the initial velocity of the object,
- a is the constant acceleration experienced by the object, and
- s is the displacement of the object.

## Approach

Primarily, we require two pieces of information:

1. The distance necessary for the robot to decelerate to a complete stop.
2. The distance between the robot and the wall ahead, obtained from Lidar data.


The formula used to calculate the distance required for deceleration is derived from:

![Image](distance.png)

Where:
- $v$ is the final velocity, which is 0 in this case.
- $u$ is the initial linear velocity, representing the current linear velocity of the robot.
- $a$ is the deceleration, the rate at which the robot slows down.
- $s$ is the distance needed to decelerate from the initial velocity to the final velocity.

- A padding distance will be added to the calculated distance; otherwise, the vehicle will stop at the same position as the wall.

![Image](velocity.jpg)

The distance to the wall is obtained from Lidar data, which is collected in an array comprising nearly 360 values. To determine the distance ahead of the robot, we extract the value at the zeroth index of the array.

Due to potential fluctuations in Lidar data, it's essential to maintain consistency by calculating an average with the previous Lidar value.

## Algorithm
An if-else loop is utilized to manage the switching control between different states, such as acceleration, deceleration, and stopping velocity.

![Image](Flowchart_challenge1.jpg)

## Result
- The vehicle can be halted at the desired distance from the wall by adjusting the padding distance values.
 - Changes in velocity and deceleration will consequently change the distances required for acceleration and deceleration.

 ![Image](Velocity_to_TIme.png)
 ![Image](Distance_to_Time.png)

 - From the distance vs. time graph, it is evident that the fluctuations observed are attributed to the noise originating from the LiDAR data.

## Challenges faced

- The Lidar values exhibit fluctuations, resulting in the vehicle stopping at varying distances each time.

- The deceleration value wasn't proportional to the velocity provided because only a percentage of the velocity (2.6m/s) was being utilized. Therefore, when calculating distances, it's essential to map all values accordingly.