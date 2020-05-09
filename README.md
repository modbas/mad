# Mini-Auto-Drive MAD
MAD is a lab experiment in the 1:24 scale for self-driving cars.

Please visit [https://asert.hs-heilbronn.de](https://asert.hs-heilbronn.de)
for videos and further information.

MAD is applied in teaching and research at Hochschule Heilbronn, Germany.

MAD was presented on the Bundesgartenschau BUGA 2019 in Heilbronn.

This repository contains template files for model-based engineering of
software functions for self-driving cars in C++14/ROS or MATLAB(R)/Simulink(R).

MAD runs on ROS or ROS2 on Linux with or without Preempt-RT or
virtually in MATLAB(R)/Simulink(R).

Contact frank.traenkle(at)hs-heilbronn.de for course material /
tutorial<br/>
in order to learn how to develop simulation models and software
functions for self-driving in ROS / C++ / MATLAB(R)/Simulink(R).

## MAD Features
* Autonomous driving in 1:24 scale
* Up to 10 cars running in parallel on 2.7 x 1.8m track
* Model-in-the-Loop simulation
* Software-in-the-Loop simulation
* Topview infrared camera
* Bluetooth LE car remote control
* Vehicle dynamics simulation
* Perception
* Localization
* Mission Control
* Behavior Planning
* Path Planning
* Motion Control
* Functional Safety for Self-Driving
* GPLv3 License

## Tutorial Contents
* Model-Based Software Engineering for Self-Driving Cars
* Robot Operating System ROS
* System Dynamics Modeling and Simulation
* Vehicle Dynamics Simulation
* Speed Control
* Parking Position Control
* Path Definition and Interpolation
* Path Following Control
* Exercises in ROS / C++ / MATLAB(R)/Simulink(R)

## ROS Installation
1. Install `ros-kinetik-desktop-full` or `ros-melodic-desktop-full`<br/>
See [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Install LTTng<br/>
`sudo apt install lttng-tools liblttng-ctl-dev liblttng-ust-dev`
3. Clone this GitHub repository<br/>
`git clone https://github.com/modbas/mad`
4. Change to Catkin workspace directory<br/>
`cd mad/catkin_ws`
5. Source ROS environment<br/>
`source /opt/ros/melodic/setup.bash`
6. Build Catkin workspace<br/>
`catkin_make`
7. Source workspace environment<br/>
`source devel/setup.bash`
8. Launch Software-in-the-Loop testing environment<br/>
`roslaunch madcar simmanual.launch`

## MATLAB(R)/Simulink(R) Installation
1. Install MATLAB(R) R2019a or later including the toolboxes<br/>
Simulink(R), Control System Toolbox, Curve Fitting Toolbox
2. Clone this GitHub repository<br/>
`git clone https://github.com/modbas/mad`
3. Start MATLAB(R)
4. Add `mad/matlab/realtime_pacer` to `matlabpath`<br/>
`addpath mad/matlab/realtime_pacer`<br/>
`savepath`
5. Change to MAD directory<br/>
`cd mad/matlab/madctrl`
6. Open and simulate `s6_template.slx`
