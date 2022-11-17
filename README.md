# PX4_MPC
Originally, PX4 uses PID controller for attitude control. In this modified scheme, NMPC is used to generate body angular velocity and thrust command, which will be sent to rate controller module to perform low-level controller to generate torque. Also, NMPC is solved using ACADO. 

The main reference of this controller is according to paper "A New Bundle Picture for the Drone", where dynamics of drone is separated into base and fiber, in which yaw dynamics is developed on fiber and the rest on base, resulting in separated control of yaw angle from the rest.

The reference trajectory is currently recieved from "Offboard Control" via ROS node. To use the controller, please refer to offboard controller tutorial from PX4 Document.
