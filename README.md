## IJS-Kali
### School project - object position calibration

School project to capture the pose of an object in robot workspace using kinesthetic guidance executed with ROS.

Basic step goals

### 1. Installation of ROS (and dualboot Ubuntu 18.04)
For this project we will be using ROS Melodic together with Ubuntu 18.04 installed in dual boot on a windows machine.

### 2. Simulation Franka Emika
Second goal consists of visualizing (imoprt) of used robot - Franka Emika.

### 3. Rviz visualization/simulation
Importing Emika dynamic characteristics? Inverse kinematics... Exporting the position of the tip of the robot.

### 4. Data capture
Capturing the coordinates and guiding the user to predetermined points on the object to capture.

### 5. Calculating the pose of the object
Using (developing/adapting) an algoritm to position the object in accordance to the captured data.

### 6. Test on robot
Adapting the project to work on a real robot. Changing the topics...


### To run:
in terminal run the Panda tutorial (visualization in RVIZ) with:
`roslaunch panda_moveit_config demo.launch  rviz_tutorial:=true`

then launch the python script:
`GUI.py`

Multiple scripts with dedicated tasks:
- GUI.py written user interface, and connecting all the code together
- marker_diplay.py used for displaying the guide markers (e.g. spheres) as well as the captured points
- object_positioner used for displaying mesh file of object on which the guide markers are attached so the user knows the goal to guide the robot. Second use is after calibration it publishes the updated position (where it really is in respect to the robot base).
- allign_3D_points.py used as a script for point manipulation/allignment functions
