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
   a) LKH, which is called from a python script found at scripts/LKH_solver.py which calls the downloaded program: http://webhotel4.ruc.dk/~keld/research/LKH-3/
   b) Python MIP, which is a custom written Mixed Integer Program to optimize the weld order found at https://www.python-mip.com/
5) Additionaly are requirements in scipy, numpy, pyvista, and Networkx.

Once these dependencies are fully installed the included code can be cloned into the SRC of the UR src, downloaded in (3), from here it can be built.

Photo of the UR10e with attached tool piece:

![0toeecordtransform](https://github.com/rayproengineering/UR10e_wp/assets/155496909/7658bfc8-33b2-4779-a54b-7aaaf0b7cb94)

The program is relient on a provided CAD or stl file that Pyvista can interpret found as 
PART_FILE_NAME='exosent_grid_v3.stl' from the v2 test bench.
Running the program is relient on this test bench which calls and executes the part matching, weld selection, and tack selection- then solving the weld problem based on the user selected data.
If a new part is to be uploaded it is recommended all desired welds are explicit points in the CAD or STL model, as the selection tool can not interpolate.
Example part solution from Thesis:
![image](https://github.com/rayproengineering/UR10e_wp/assets/155496909/b34e969b-1d6a-41dc-86b9-1fce3d99b4d6)

Additionally this project made use of a Tregasskiss Weld Gun, and a Miller based MIG welder (reach out for detials)
Both of these systems were controlled with a Arduino based as a ROS Node.

