//ps6 conveyor belt controller node
//Josh Roney(jpr87)
//
//Controls the conveyor belt for ARIAC as per the requirements of PS6.
//Begins by moving the conveyor belt until the object "shipping_box" arrives under logical-camera 2.
//When the box arrives under logical-camera 2, the conveyor belt halts for 5 seconds.
//After 5 seconds, the conveyor belt restarts until the box slides down to the loading dock.
//The drone is then called to fetch the box.

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <osrf_gear/ConveyorBeltState.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <xform_utils/xform_utils.h>
#include <string.h>
using namespace std;
