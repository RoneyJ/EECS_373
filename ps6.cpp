//ps6 conveyor belt controller node
//Josh Roney(jpr87)
//
//Controls the conveyor belt for ARIAC as per the requirements of PS6.
//Begins by moving the conveyor belt until the object "shipping_box_1" arrives under logical_camera_2.
//When the box arrives under the camera, the conveyor belt halts for 5 seconds.
//After 5 seconds, the conveyor belt restarts until the box slides down to the loading dock.
//The drone is then called to fetch the box.

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>
using namespace std;

string object_name("shipping_box");  //hard-coded object name for box

bool g_take_new_snapshot = false;
double z_position = 0.0;
osrf_gear::LogicalCameraImage g_cam1_data;

void cam2CB(const osrf_gear::LogicalCameraImage& message_holder) {
    if (g_take_new_snapshot) {
        ROS_INFO_STREAM("image from cam1: " << message_holder << endl);
        g_cam1_data = message_holder;
	if(g_cam1_data.models.size() > 0)
	    z_position = g_cam1_data.models[0].pose.position.z;
        g_take_new_snapshot = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;

    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    std_srvs::Trigger startup_srv;

    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;

    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2", 1, cam2CB);

    startup_srv.response.success = false;

    while (!startup_srv.response.success) {
        ROS_WARN("not successful starting up yet, waiting on service...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("got successful response from startup service");
    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;

    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got successful response from conveyor service");

    while (g_cam1_data.models.size() < 1) {
	g_take_new_snapshot = true;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("I see a box");

    bool box_aligned = false;
    while(!box_aligned){
	g_take_new_snapshot = true;
	if(z_position > -0.01 && z_position < 0.01)
	    box_aligned = true;
	ros::spinOnce();
	ros::Duration(0.1).sleep();
    }

    ROS_INFO("halting conveyor for 5 seconds");
    conveyor_srv.request.power = 0.0;
    conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful halting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got successful response from conveyor service");    

    //stop conveyor for 5 seconds
    ros::Duration(5.0).sleep();
    
    ROS_INFO("restarting conveyor");
    ROS_INFO("got successful response from startup service");
    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got successful response from conveyor service");

    drone_srv.request.shipment_type = "dummy";

    drone_srv.response.success = false;

    while (!drone_srv.response.success) {
        ROS_WARN("not successful starting drone yet...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("got success response from drone service");

    //logical_camera_2 will need to subscribe to 
    //see if a shipping box shows up and then look at the position
    //when you see the shipping box, wait until it is under the camera (z coord is zero) then stop the conveyor belt
    //let it dwell there for a few seconds (5 secs) then start the conveyor again

}
	
	

