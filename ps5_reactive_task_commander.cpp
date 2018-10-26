//ps5_reactive_task_commander.cpp
//Josh Roney (jpr87)
//
//this version is a variation of irb120_reactive_task_commander, but commands the
//robot to move the object to a set location instead of just touching the top surface
//
//Goal location is (0.2 , 0.2) on the XY plane
//
//Always moves along the X-axis first, will not work if x-axis movement is unable to be done first

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

#include <irb120_fk_ik/irb120_kinematics.h>
#include <fk_ik_virtual/fk_ik_virtual.h>
#include "robot_specific_fk_ik_mappings.h"
#include "robot_specific_names.h"

#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<sensor_msgs/JointState.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>

std::vector<double> g_planner_joint_weights{3, 3, 2, 1, 1, 0.5};

string g_object_name("gear_part");  //hard-coded object name
int g_found_object_code;
geometry_msgs::PoseStamped g_perceived_object_pose;

ros::Publisher *g_pose_publisher;

CartTrajPlanner *pCartTrajPlanner;

//declare XY-coordinate goal locations
double x_goal = 0.2;
double y_goal = 0.2;

Eigen::VectorXd g_q_vec_arm_Xd;

void print_affines(std::vector<Eigen::Affine3d> affine_path) {
    int npts = affine_path.size();
    ROS_INFO("affine path, origins only: ");
    for (int i = 0; i < npts; i++) {
        cout << affine_path[i].translation().transpose() << endl;
    }
}

void print_traj(trajectory_msgs::JointTrajectory des_trajectory) {
    int npts = des_trajectory.points.size();
    int njnts = des_trajectory.points[0].positions.size();
    //Eigen::VectorXd jspace_pt;
    //jspace_pt.resize(njnts);
    ROS_INFO("traj points: ");
    for (int i = 0; i < npts; i++) {
        //jspace_pt=optimal_path[i];
        for (int j = 0; j < njnts; j++) {
            cout << (des_trajectory.points[i]).positions[j] << ", ";
        }
        cout << "; t = " << des_trajectory.points[i].time_from_start.toSec() << endl;

    }
}



//this callback function receives a result from the magic object finder action server
//it sets g_found_object_code to true or false, depending on whether the  object was found
//if the object was found, then components of g_perceived_object_pose are filled in
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const magic_object_finder::magicObjectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND) {
        ROS_WARN("object-finder responded: object not found");
    }
    else if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_commander"); // node name 
    ros::NodeHandle nh; //standard ros node handle   
    Eigen::Affine3d start_flange_affine, goal_flange_affine; //specify start and goal in Cartesian coords
    std::vector<Eigen::VectorXd> optimal_path; //a path in joint space is a sequence of 6-DOF joint-angle specifications
    trajectory_msgs::JointTrajectory new_trajectory; //will package trajectory messages here


    //set up an action client to query object poses using the magic object finder
    actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> object_finder_ac("object_finder_action_service", true);
    bool finished_before_timeout=false; 
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server");
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    magic_object_finder::magicObjectFinderGoal object_finder_goal;
     
    Eigen::Matrix3d R_down;
    Eigen::Vector3d x_axis, y_axis, z_axis, flange_origin;
    z_axis << 0, 0, -1; //points flange down
    x_axis << -1, 0, 0; //arbitrary
    y_axis = z_axis.cross(x_axis); //construct y-axis consistent with right-hand coordinate frame
    R_down.col(0) = x_axis;
    R_down.col(1) = y_axis;
    R_down.col(2) = z_axis;
    flange_origin << 0.2, 0, 0.01;
    int nsteps = 5;
    double arrival_time = 5.0;

    CartesianInterpolator cartesianInterpolator;

    g_q_vec_arm_Xd.resize(NJNTS); //generic vector resized to actual robot number of joints
    g_q_vec_arm_Xd << 0, 0, 0, 0, 0, 0; //assumes arm starts in this pose

    //our irb120 control  interface uses this topic to receive trajectories
    ros::Publisher traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    pCartTrajPlanner = new CartTrajPlanner(pIKSolver, pFwdSolver, njnts);
    pCartTrajPlanner->set_jspace_planner_weights(g_planner_joint_weights);
    pCartTrajPlanner->set_joint_names(g_jnt_names);


    optimal_path.clear(); //reset this std::vector before each use
    optimal_path.push_back(g_q_vec_arm_Xd); //start from current pose
    optimal_path.push_back(g_q_vec_arm_Xd); // go from current pose to current pose
    //publish/subscribe interface
    arrival_time = 3; //move should require zero time, but provide something small

    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);

    traj_publisher.publish(new_trajectory);
    ros::Duration(arrival_time).sleep();

    start_flange_affine = pFwdSolver->fwd_kin_solve(g_q_vec_arm_Xd);
    //display the flange affine corresponding to the specfied arm angles
    ROS_INFO_STREAM("fwd soln: origin = " << start_flange_affine.translation().transpose() << endl);
    ROS_INFO_STREAM("fwd soln: orientation: " << endl << start_flange_affine.linear() << endl);
    
    
    //xxxxxxxxxxxxxx  the following makes an inquiry for the pose of the part of interest
    //specify the part name, send it in the goal message, wait for and interpret the result
    object_finder_goal.object_name = g_object_name.c_str(); //convert string object to old C-style string data
    object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server
        
    finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
    //check the result code to see if object was found or not
    if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND)   {
        ROS_INFO("found object!");
    }    
    else {
        ROS_WARN("object not found!  Quitting");
        return 1;
    }
    //xxxxxxxxxx   done with inquiry.  If here, then part pose is in g_perceived_object_pose.  Use it to compute robot motion




    //
    //**work for PS5 begins here**    
    //
    //The aim of this code is to move the object to position (0.2, 0.2) on the XY-plane
    //by first repositioning it in the x-direction followed by repositioning it in the y-direction

    //declare variables for XY-coordinates of tool
    double x_coor1 = 0.0; //next to object
    double x_coor2 = 0.0; //end x-position of tool frame after pushing object
    double y_coor1 = 0.0; //next to object
    double y_coor2 = 0.0; //end y-position of tool frame after pushing object

    goal_flange_affine.linear() = R_down; //set the  goal orientation for flange to point down

    //determine if gear needs to be moved in the positive or negative x-direction
    //use to determine whether to place tool in front of or behind gear (in x-direction)
    if(g_perceived_object_pose.pose.position.x < x_goal){
	x_coor1 = g_perceived_object_pose.pose.position.x - 0.1;
	x_coor2 = x_goal - 0.06;
    }
    else{
	x_coor1 = g_perceived_object_pose.pose.position.x + 0.1;
	x_coor2 = x_goal + 0.06;
    }

    //xxxx  use the y and z coordinates of the object, but specify an x-value next to object
    //moves the tool next to the object to push it in x-direction
    flange_origin << x_coor1, g_perceived_object_pose.pose.position.y, g_perceived_object_pose.pose.position.z;
    goal_flange_affine.translation() = flange_origin;
    ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);
    ROS_INFO_STREAM("with orientation: " << endl << goal_flange_affine.linear() << endl);

    nsteps = 100; //many points to allow smoother motion

    //compute an optimal joint-space path:
    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }
    //if here, have a viable joint-space path; convert it to a trajectory
    arrival_time = 4.0;
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete
    ROS_INFO("done with first trajectory");
    //xxxxxxxxxxxxxxxxxx

    //go to push object to x-coordiante; same y and z, but modified x posistion to push object
    //moves the tool against the object to push it along x-axis
    flange_origin <<x_coor2, g_perceived_object_pose.pose.position.y, g_perceived_object_pose.pose.position.z; 
    goal_flange_affine.translation() = flange_origin;
    ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);

    g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the plan
    optimal_path.clear();
    nsteps = 100; //many points to allow smoother motion
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }
    //convert the path to a trajectory
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory
    ros::Duration(arrival_time).sleep(); //wait for the motion
    ROS_INFO("done with second trajectory");
    ros::Duration(2.0).sleep(); //dwell here to observe motion performed
    //xxxxxxxxxxxxxxxxxx

    //depart vertically to repostion tool
    flange_origin << x_coor2, g_perceived_object_pose.pose.position.y, 0.1;
    goal_flange_affine.translation() = flange_origin;
    ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);
    g_q_vec_arm_Xd = optimal_path.back(); //start from the joint-space pose that ended the prior plan
    optimal_path.clear();
    nsteps = 100; //many points to allow smoother motion

    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory
    ros::Duration(arrival_time).sleep(); //wait for the motion
 
    ROS_INFO("done with third trajectory");

    //determine if gear needs to be moved in the positive or negative y-direction
    //use to determine whether to place tool in front of or behind gear (in y-direction)
    if(g_perceived_object_pose.pose.position.y < y_goal){
	y_coor1 = g_perceived_object_pose.pose.position.y - 0.1;
	y_coor2 = y_goal - 0.06;
    }
    else{
	y_coor1 = g_perceived_object_pose.pose.position.y + 0.1;
	y_coor2 = y_goal + 0.06;
    }
    
    //xxxx  use the x and z coordinates of the gear part, but specify a y-value next to gear
    //moves the tool next to the object to push it in y-direction
    flange_origin << x_goal, y_coor1, g_perceived_object_pose.pose.position.z;
    goal_flange_affine.translation() = flange_origin;
    ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);
    ROS_INFO_STREAM("with orientation: " << endl << goal_flange_affine.linear() << endl);
    g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the plan

    nsteps = 100; //many points to allow smoother motion

    optimal_path.clear();
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }
    //if here, have a viable joint-space path; convert it to a trajectory:
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory
    ros::Duration(arrival_time).sleep(); //wait for the motion to complete
    ROS_INFO("done with fourth trajectory");
    //xxxxxxxxxxxxxxxxxx

    //go to push gear part to y-coordiante; same x and z, but modified y posistion to push gear part
    //moves the tool against the object to push it along y-axis
    flange_origin <<x_goal, y_coor2, g_perceived_object_pose.pose.position.z; 
    goal_flange_affine.translation() = flange_origin;
    ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);

    g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the plan
    optimal_path.clear();
    nsteps = 100; //many points to allow smoother motion
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }
    //convert the path to a trajectory (adds joint-space names,  arrival times, etc)
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory
    ros::Duration(arrival_time).sleep(); //wait for the motion
    ROS_INFO("done with fifth trajectory");
    ros::Duration(2.0).sleep(); //dwell here to observe motion performed
    //xxxxxxxxxxxxxxxxxx

    //depart vertically, movement complete
    flange_origin << x_goal, y_goal, 0.5;
    goal_flange_affine.translation() = flange_origin;
    ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);
    g_q_vec_arm_Xd = optimal_path.back(); //start from the joint-space pose that ended the prior plan
    optimal_path.clear();
    nsteps = 100; //many points to allow smoother motion

    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
            nsteps, optimal_path)) {
        ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
        return 1;
    }
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);
    traj_publisher.publish(new_trajectory); //publish the trajectory
    ros::Duration(arrival_time).sleep(); //wait for the motion
 
    ROS_INFO("done with final trajectory");
}




