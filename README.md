# autonomous-vehicle

*This is my graduation final project*

A robot which perform two main features: Trajectory Tracking and Obstacle Avoidance.
1. Trajectory tracking: Choose some points through google map interface integrated in app, then generate a likely-curve line (trajectory) for robot. It's mission is to follow the line from A to B with minimum average error.
2. While tracking the trajectory, robot might face some obstacles (static or movable) which it has to deflect (maybe get out of trajectory) then comeback to the line and continue to track the path.
Technologies: C/C++, C#, .NET, Python, Robot operating system, Hardware & Sensors (self-designed electronic circuits, gps, imu, RF, camera, microcontroller, embedded computer)

**1. Hardware**
1) 1 stm32f411 + 1 stm32f407
2) 2 H-Bridge HI216
3) 2 DC motor with high resolution encoder
4) 4 modules RF LoRa 433MHz SX1278
5) 4 SMA antenna for module Lora
6) 2 modules GPS RTK NEO–M8P
7) 1 IMU ADIS16488
8) 2 acquy 12V-3AH GLOBE for Motor Power Stage
9) 1 acquy 12V-20AH GLOBE for Embedded PC104 and Controller 
10) Some workpieces 

**About Algorithm**
* Map planning
* Stanley Controller
* Fuzzy Controller
* PID Controller

**About C# App**
