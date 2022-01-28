# Mini-Auto-Drive MAD
MAD is a lab experiment in the 1:24 scale for self-driving cars.

Please visit [https://asert.hs-heilbronn.de](https://asert.hs-heilbronn.de)
for videos and further information.

MAD is applied in teaching and research at Hochschule Heilbronn, Germany.

MAD was presented on the Bundesgartenschau BUGA 2019 in Heilbronn.

This repository contains template files for model-based engineering of
software functions for self-driving cars in C++14/ROS or
MATLAB(R)/Simulink(R)/Stateflow(R).

MAD runs on ROS or ROS2 on Linux with or without Preempt-RT or
virtually in MATLAB/Simulink/Stateflow.

Please read the textbook
[Frank Tränkle (2021). Modellbasierte Entwicklung mechatronischer Systeme:
mit Software- und Simulationsbeispielen für autonomes
Fahren. DeGruyter Studium](https://www.degruyter.com/document/doi/10.1515/9783110723526/html) containing the theory and examples of
software functions for motion control in autonomous driving.

## MAD Features
* Autonomous driving in 1:24 scale
* Model-based engineering of software functions for self-driving cars
  either in C++14 or MATLAB/Simulink/Stateflow
* Model-in-the-Loop simulation in MATLAB/Simulink/Stateflow
* Auto-code-generation with Simulink Embedded Coder(R)
* Software-in-the-Loop simulation in ROS or ROS2
* Real test driving of 1:24-scale cars controlled by ROS or ROS2
* Up to 10 cars running in parallel on 2.7 x 1.8m track
* Topview infrared camera
* Bluetooth LE car remote control
* Vehicle dynamics simulation
* Perception
* Localization
* Mission control by smartphones or web browsers
* Navigation
* Behavior planning
* Path planning
* Motion control
* Functional safety for self-driving
* GPLv3 license

## Tutorial Contents
* Model-Based Software Engineering for Self-Driving Cars
* Robot Operating System ROS
* System Dynamics Modeling and Simulation
* Vehicle Dynamics Simulation
* Speed Control
* Parking Position Control
* Path Definition and Interpolation
* Path Following Control
* Exercises in ROS / C++ / MATLAB/Simulink

## C++/ROS Installation
1. Install `ros-kinetik-desktop-full` or `ros-melodic-desktop-full` or `ros-noetic-desktop-full`<br/>
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

## MATLAB/Simulink Installation
1. Install MATLAB R2019a or later including the toolboxes<br/>
Simulink, Control System Toolbox, Curve Fitting Toolbox
2. Clone this GitHub repository<br/>
`git clone https://github.com/modbas/mad`
3. Start MATLAB
4. Change to MAD directory<br/>
`cd mad/matlab/madctrl`
5. Open and simulate `s6_template.slx`
