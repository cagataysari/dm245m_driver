# DM245M Stepper Motor Driver
This package is a libmodbus and ROS based driver of [DM245M](http://www.robosan.com.tr/step/DM245M.htm) stepper motor driver
It contains a simple demo which subscribes to cmd_vel topic and sends pulse counts to stepper motor.
This package depends on libmodbus library.

# Compile

Go to your catkin workspace source directory

    cd ~/your_catkin_ws/src

Clone this repo:

    git clone https://www.github.com/cagataysari/dm245m_driver


Run catkin make

    cd ~/your_catkin_ws && catkin_make

# Parameters

Parameters are not integrated with ROS, you can modify them inside DM245MDriver.cpp's constructor.
