# Robtic-Dog
A robotic dog capable of walking using 12 robotic servos, an Arduino and a lot of 3D printing. The servos had position feedback and torque control. The onboard arduino mega board talked to the servos through a custom communication curcuit. All the servos used a communication bus which made it possible to daisy chain the communication. The power was drawn from an onboard battery through a dcdc converter to regulate the voltage. It was planned for the robot to use 2 more servos as the spine but there was not enough time to add control code for these 2 servos, the spine was made stiff. The robot did managed to walk but due to the high latency communication, the low resolution position feedback and the time limit it was too difficult to make a dynamic gait work well.

# Documentation
Read the rapport in the docs dir if you want to know more.

Robot CAD                  |  Target gait
:---------------------------:|:---------------------------------------------:
![](https://github.com/pinkponk/Robotic-Dog/blob/master/Images/main%20st%C3%B6dben%20close%20up.jpg)  |  ![](https://github.com/pinkponk/Robotic-Dog/blob/master/Images/CADGhostGait.jpeg)

Complete robot                  |  Matlab simulation of gait and inverse kinematics
:---------------------------:|:---------------------------------------------:
![](https://github.com/pinkponk/Robotic-Dog/blob/master/Images/WP_20150520_002.jpg) | ![](https://github.com/pinkponk/Robotic-Dog/blob/master/Images/FourstepsMatlab.jpeg)

Calculating Inverse kinematics                  |  Calculating Inverse kinematics
:---------------------------:|:---------------------------------------------:
![](https://github.com/pinkponk/Robotic-Dog/blob/master/Images/InverseKinematics0.png)  |  ![](https://github.com/pinkponk/Robotic-Dog/blob/master/Images/Leg3DOF_FrontViewV1V2V3.png)
