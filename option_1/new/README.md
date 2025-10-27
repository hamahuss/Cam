# Cam

clone the 1.16 PX4 reposittory and replace the following files 

msg/CMakeLists.txt
src/modules/logger/logged_topics.cpp
src/modules/mc_att_control/mc_att_control.hpp
src/modules/mc_att_control/mc_att_control_main.cpp
src/modules/mc_att_control/mc_att_control_params.c
src/modules/uxrce_dds_client/dds_topics.yaml
msg/VehicleCamAttitude.msg
src/modules/commander/HealthAndArmingChecks/checks/manualControlCheck.cpp
src/modules/commander/ModeUtil/mode_requirements.cpp
src/modules/flight_mode_manager/tasks/Utility/StickTiltXY.cpp
src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp



-------------------------------------------

the new message to add on the jetson and to publish is called  VehicleCamAttitude
this is changed inside src/modules/uxrce_dds_client/dds_topics.yaml to be a new subscriber,
you need to find teeh equivalent px4_msg firectory on the jetson and add the exact uorb topic in the pushlishing 

 Publish Your uORB Message from the Jetson after adding the new msg to the px4_msg on the jetson
Include Header: In your Jetson application, include the C++ header file corresponding to the uORB message you want to publish (e.g., #include <uORB/topics/vehicle_cam_attitude.h>). 
Instantiate and Publish: Get a handle to the topic using its ORB_ID, allocate a message struct, fill it with data, and then publish it using orb_publish


-------------------------------------------

the msg adds a new quaternion to follow which is a float[4] following the ZYX orientation that should be puslided from the jetson to the pixhawk 
to follow this quaternion you need to switch to Manual mode either using joystick or using commander mode manual in the consol
a new parameter is added called MC_TH_CAM by default is 0.75 or 75% which sets the applied throttle while following the quaternion.

-------------------------------------------

to run a sim after replacing the corresponding files, use the following command:

make px4_sitl gazebo-classic 

or add your sdf model to use

make px4_sitl gazebo-classic_mymodel

to verify the signal is received, inside Mavlink consol type:

listener vehicle_cam_attitude 

and you should see the reveiced values
