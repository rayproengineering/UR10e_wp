# UR10e_wp
Thesis Project in automated welding utilizing a ur10e robot, ROS1 framework, developed in Python.
The pdf, or thesis report can be found at: https://yoderaroman.wixsite.com/ray-engineering/project-1

This project makes use and is relient on the preliminary sources:
1) Ubuntu 20.04:
   https://releases.ubuntu.com/focal/
   I recommend installing this as a dual boot system with a bootable USB
   see: https://itsfoss.com/create-live-usb-of-ubuntu-in-windows/
2) ROS1 Noetic Distribution:
   https://wiki.ros.org/noetic/Installation/Ubuntu
   This is the most up-to-date distrubition of ROS1
3) Install the ROS1 Drivers for Universal Robots:
   https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
4) To solve the Optimization of Weld Order and Tack Order, the program is relient on both:
   a) LKH, which is called from a python script found in http://webhotel4.ruc.dk/~keld/research/LKH-3/
