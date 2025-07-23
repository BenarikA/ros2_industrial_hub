# TP programs

This folder contains two TP programs that must be installed on the robot controller to allow Ethernet/IP communication.

### ROS2.TP
This program is necessary to command the robot with the ros2_fanuc_interface.
It reads the command position written via Ethernet/IP from the remote PC, and execute a motion to the position written in Position Register 1 (PR1).

### ROS2_DMP.TP (experimental)
This program allows to switch from direct position control to DMP for small trajectory modification. To set up the robot, some additional configuration must be fixed on the robot side, to let the DMP module modify the trajectory reciving commands from the remote pc.
