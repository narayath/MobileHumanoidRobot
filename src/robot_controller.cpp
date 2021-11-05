#include <ros/ros.h> //ALWAYS need to include this
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>
#include <vector>
#include <unistd.h>

#include "pluto/arm_angles_msg.h"


using namespace std;
//some "magic number" global params:
const double Kp = 10.0; //controller gains
const double Kv = 3;
const double dt = 0.01;
const double pi=3.14;

//a simple saturation function; provide saturation threshold, sat_val, and arg to be saturated, val

double sat(double val, double sat_val) {
    if (val > sat_val)
        return (sat_val);
    if (val< -sat_val)
        return (-sat_val);
    return val;

}

double min_periodicity(double theta_val) {
    double periodic_val = theta_val;
    while (periodic_val > M_PI) {
        periodic_val -= 2 * M_PI;
    }
    while (periodic_val< -M_PI) {
        periodic_val += 2 * M_PI;
    }
    return periodic_val;
}


double torso_pos = 0.0; //position command input-- global var
double right_shoulder_pos = -pi/2;
double right_upper_arm_bottom_pos = -pi/2;
double right_upper_arm_top_pos = 0.0;
double right_forearm_bottom_pos = -pi/2;
double right_forearm_top_pos = 0.0;


//0.998017493175748, -1.646447451906756, -2.691219719248358, -0.959145876381672, -2.524650105955758, -2.185107573424105
/*
double torso_pos = 0.998017493175748; //position command input-- global var
double right_shoulder_pos = -1.646447451906756;
double right_upper_arm_bottom_pos = -2.691219719248358;
double right_upper_arm_top_pos = -0.959145876381672;
double right_forearm_bottom_pos = -2.524650105955758;
double right_forearm_top_pos = -2.185107573424105;
double right_palm_pos = 0.0;
double right_thumb_piece_pos = 0.0;
*/



double left_shoulder_pos = 0.0;
double left_upper_arm_bottom_pos = 0.0;
double left_upper_arm_top_pos = 0.0;
double left_forearm_bottom_pos = -pi/2;
double left_forearm_top_pos = 0.0;


ros::ServiceClient set_trq_client; //client to send Effort commands to Gazebo for controlling the joints
ros::ServiceClient get_jnt_state_client; //client to reciece position and velociy information of each joint from Gazebo

ros::Publisher joint_state_publisher; //Publisher for Joint State Sensor messages


bool posCmdCB(pluto::arm_angles_msgRequest& req, pluto::arm_angles_msgResponse& res) 
{
    if(req.arm == 0)
    {
	    torso_pos = req.torso_ang;
	    right_shoulder_pos = req.shoulder_ang;
	    right_upper_arm_bottom_pos = req.upper_arm_bottom_ang;
	    right_upper_arm_top_pos = req.upper_arm_bottom_ang;
	    right_forearm_bottom_pos = req.forearm_bottom_ang;
	    right_forearm_top_pos = req.forearm_top_ang;
    }
    
    else if(req.arm == 1)
    {
	    torso_pos = req.torso_ang;
	    left_shoulder_pos = req.shoulder_ang;
	    left_upper_arm_bottom_pos = req.upper_arm_bottom_ang;
	    left_upper_arm_top_pos = req.upper_arm_bottom_ang;
	    left_forearm_bottom_pos = req.forearm_bottom_ang;
	    left_forearm_top_pos = req.forearm_top_ang;
    }  
    
    //bool result = false;

    res.pos_ok = true;
    return true;
}

bool test_services() {
    bool service_ready = false;
    if (!ros::service::exists("/gazebo/apply_joint_effort", true)) {
        ROS_WARN("waiting for apply_joint_effort service");
        return false;
    }
    if (!ros::service::exists("/gazebo/get_joint_properties", true)) {
        ROS_WARN("waiting for /gazebo/get_joint_properties service");
        return false;
    }
    ROS_INFO("services are ready");
    return true;
}

void moveJointToPosition(char* jointName, gazebo_msgs::ApplyJointEffort effortMsg, gazebo_msgs::GetJointProperties jointPropertyMsg, sensor_msgs::JointState jointStateMsg, float targetPos, float scale)
{
        bool result = false;
        double q1, q1dot, q1_err, trq_cmd;
        
        //Call the client to get the currrent velocity and position of the elbow joint
	get_jnt_state_client.call(jointPropertyMsg);
        q1 = jointPropertyMsg.response.position[0];
        //q1_msg.data = q1;
        //pos_publisher.publish(q1_msg); //republish his val on topic jnt_pos
                
        q1dot = jointPropertyMsg.response.rate[0];
        //q1dot_msg.data = q1dot;
        //vel_publisher.publish(q1dot_msg);
	 
	 //ROS_INFO("%s q= %f , Q1dot= %f", jointName, q1, q1dot);
	 //Publish the current position and velocity of the elbow joint
        jointStateMsg.header.stamp = ros::Time::now();
        jointStateMsg.position[0] = q1;
        jointStateMsg.velocity[0] = q1dot;
        joint_state_publisher.publish(jointStateMsg);
        
        jointStateMsg.header.stamp = ros::Time::now();
	//Calculate the difference in current position and desired position
        q1_err = min_periodicity(targetPos - q1); //jnt angle err; watch for periodicity
	
	//Calculate the torque requrired to move the joint to the desired position
        trq_cmd = (Kp * (q1_err) - Kv*q1dot) * scale;
        //ROS_INFO("Torque=  %f ", trq_cmd);
        //trq_msg.data = trq_cmd;
        //trq_publisher.publish(trq_msg);
        
	//Set the torque to the command message to be sent to Gazebo
        effortMsg.request.effort = trq_cmd; // send torque command to Gazebo
        //Call the torque client to send the command to Gazebo that will move the joint
        set_trq_client.call(effortMsg);
        
        //make sure service call was successful
        result = effortMsg.response.success;
        if (!result)
        {
            ROS_WARN("service call to set Effort Command failed!");
        }	
}

int main(int argc, char **argv) {
    //initializations:
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);

    // make sure services are available before attempting to proceed, else node will crash
    while (!test_services()) {
        ROS_INFO("starting");
        ros::spinOnce();
        ROS_INFO("spun");
        half_sec.sleep();
    }
    
    ros::ServiceServer service = nh.advertiseService("robot_controller", posCmdCB);
    
    //Set the client to send Effort commands to Gazebo for controlling the joints
    set_trq_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    
    //Set the client to reciece position and velociy information of each joint from Gazebo
    get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
   
    
    
    //publishers for publishing torque, velocity, position for joints
    ros::Publisher trq_publisher = nh.advertise<std_msgs::Float64>("jnt_trq", 1);
    ros::Publisher vel_publisher = nh.advertise<std_msgs::Float64>("jnt_vel", 1);
    ros::Publisher pos_publisher = nh.advertise<std_msgs::Float64>("jnt_pos", 1);
    //publisher for state of the joint (velocity and position)
    joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    //message defintions for torque, position and velocity publishers
    std_msgs::Float64 trq_msg, q1_msg, q1dot_msg;
    char* jointName;
    
    //Message to send effort command and receive position and velocity info to/from Torso revolute joint
    gazebo_msgs::ApplyJointEffort torso_effort_cmd;
    gazebo_msgs::GetJointProperties torso_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from shoulder revolute joint
    //This joint allow the arm to move in the horizontal (parallel to  torso) plane
    gazebo_msgs::ApplyJointEffort right_shoulder_effort_cmd;
    gazebo_msgs::GetJointProperties right_shoulder_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from upper arm revolute joint
    //This joint allows the whole arm to move in vertical (Vertical to  torso) plane
    gazebo_msgs::ApplyJointEffort right_upper_arm_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties right_upper_arm_bottom_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from upper arm revolute joint
    //This joint allows the whole arm to move in vertical (Vertical to  torso) plane
    gazebo_msgs::ApplyJointEffort right_upper_arm_top_effort_cmd;
    gazebo_msgs::GetJointProperties right_upper_arm_top_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from forearm bottom revolute joint
    //This is the elbow joint
    gazebo_msgs::ApplyJointEffort right_forearm_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties right_forearm_bottom_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from forearm top revolute joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_forearm_top_effort_cmd;
    gazebo_msgs::GetJointProperties right_forearm_top_joint_property;
 
    //messages that will be received from joint state publisher for each joint
    sensor_msgs::JointState torso_joint_state_msg;
    sensor_msgs::JointState right_shoulder_joint_state_msg;
    sensor_msgs::JointState right_upper_arm_bottom_joint_state_msg;
    sensor_msgs::JointState right_upper_arm_top_joint_state_msg;
    sensor_msgs::JointState right_forearm_bottom_joint_state_msg;
    sensor_msgs::JointState right_forearm_top_joint_state_msg;
    
    
    //Messages for left hand joints
    //Message to send effort command and receive position and velocity info to/from shoulder revolute joint
    //This joint allow the arm to move in the horizontal (parallel to  torso) plane
    gazebo_msgs::ApplyJointEffort left_shoulder_effort_cmd;
    gazebo_msgs::GetJointProperties left_shoulder_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from upper arm revolute joint
    //This joint allows the whole arm to move in vertical (Vertical to  torso) plane
    gazebo_msgs::ApplyJointEffort left_upper_arm_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties left_upper_arm_bottom_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from upper arm revolute joint
    //This joint allows the whole arm to move in vertical (Vertical to  torso) plane
    gazebo_msgs::ApplyJointEffort left_upper_arm_top_effort_cmd;
    gazebo_msgs::GetJointProperties left_upper_arm_top_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from forearm bottom revolute joint
    //This is the elbow joint
    gazebo_msgs::ApplyJointEffort left_forearm_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties left_forearm_bottom_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from forearm top revolute joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_forearm_top_effort_cmd;
    gazebo_msgs::GetJointProperties left_forearm_top_joint_property;
 
    
    //messages that will be received from joint state publisher for each joint
    sensor_msgs::JointState left_shoulder_joint_state_msg;
    sensor_msgs::JointState left_upper_arm_bottom_joint_state_msg;
    sensor_msgs::JointState left_upper_arm_top_joint_state_msg;
    sensor_msgs::JointState left_forearm_bottom_joint_state_msg;
    sensor_msgs::JointState left_forearm_top_joint_state_msg;

    //Initialize Messages 
    ros::Duration duration(dt);
    ros::Rate rate_timer(1 / dt);

    torso_effort_cmd.request.joint_name = "torso_joint";
    torso_effort_cmd.request.effort = 0.0;
    torso_effort_cmd.request.duration = duration;
    torso_joint_property.request.joint_name = "torso_joint";
    
    right_shoulder_effort_cmd.request.joint_name = "right_shoulder_joint";
    right_shoulder_effort_cmd.request.effort = 0.0;
    right_shoulder_effort_cmd.request.duration = duration;
    right_shoulder_joint_property.request.joint_name = "right_shoulder_joint";
    
    right_upper_arm_bottom_effort_cmd.request.joint_name = "right_upper_arm_bottom_joint";
    right_upper_arm_bottom_effort_cmd.request.effort = 0.0;
    right_upper_arm_bottom_effort_cmd.request.duration = duration;
    right_upper_arm_bottom_joint_property.request.joint_name = "right_upper_arm_bottom_joint";
    
    right_upper_arm_top_effort_cmd.request.joint_name = "right_upper_arm_top_joint";
    right_upper_arm_top_effort_cmd.request.effort = 0.0;
    right_upper_arm_top_effort_cmd.request.duration = duration;
    right_upper_arm_top_joint_property.request.joint_name = "right_upper_arm_top_joint";
 
    right_forearm_bottom_effort_cmd.request.joint_name = "right_forearm_bottom_joint";
    right_forearm_bottom_effort_cmd.request.effort = 0.0;
    right_forearm_bottom_effort_cmd.request.duration = duration;
    right_forearm_bottom_joint_property.request.joint_name = "right_forearm_bottom_joint";
    
    right_forearm_top_effort_cmd.request.joint_name = "right_forearm_top_joint";
    right_forearm_top_effort_cmd.request.effort = 0.0;
    right_forearm_top_effort_cmd.request.duration = duration;
    right_forearm_top_joint_property.request.joint_name = "right_forearm_top_joint";
    

    //Initialize the joint command messages for the left arm
        left_shoulder_effort_cmd.request.joint_name = "left_shoulder_joint";
    left_shoulder_effort_cmd.request.effort = 0.0;
    left_shoulder_effort_cmd.request.duration = duration;
    left_shoulder_joint_property.request.joint_name = "left_shoulder_joint";
    
    left_upper_arm_bottom_effort_cmd.request.joint_name = "left_upper_arm_bottom_joint";
    left_upper_arm_bottom_effort_cmd.request.effort = 0.0;
    left_upper_arm_bottom_effort_cmd.request.duration = duration;
    left_upper_arm_bottom_joint_property.request.joint_name = "left_upper_arm_bottom_joint";
    
    left_upper_arm_top_effort_cmd.request.joint_name = "left_upper_arm_top_joint";
    left_upper_arm_top_effort_cmd.request.effort = 0.0;
    left_upper_arm_top_effort_cmd.request.duration = duration;
    left_upper_arm_top_joint_property.request.joint_name = "left_upper_arm_top_joint";
 
    left_forearm_bottom_effort_cmd.request.joint_name = "left_forearm_bottom_joint";
    left_forearm_bottom_effort_cmd.request.effort = 0.0;
    left_forearm_bottom_effort_cmd.request.duration = duration;
    left_forearm_bottom_joint_property.request.joint_name = "left_forearm_bottom_joint";
    
    left_forearm_top_effort_cmd.request.joint_name = "left_forearm_top_joint";
    left_forearm_top_effort_cmd.request.effort = 0.0;
    left_forearm_top_effort_cmd.request.duration = duration;
    left_forearm_top_joint_property.request.joint_name = "left_forearm_top_joint";
    

    // Setup the initial position and velocity values of each joint
    torso_joint_state_msg.header.stamp = ros::Time::now();
    torso_joint_state_msg.name.push_back("torso");
    torso_joint_state_msg.position.push_back(0.0);
    torso_joint_state_msg.velocity.push_back(0.0);
    
    right_shoulder_joint_state_msg.header.stamp = ros::Time::now();
    right_shoulder_joint_state_msg.name.push_back("right_shoulder");
    right_shoulder_joint_state_msg.position.push_back(0.0);
    right_shoulder_joint_state_msg.velocity.push_back(0.0);
    
    right_upper_arm_bottom_joint_state_msg.header.stamp = ros::Time::now();
    right_upper_arm_bottom_joint_state_msg.name.push_back("right_upper_arm_bottom");
    right_upper_arm_bottom_joint_state_msg.position.push_back(0.0);
    right_upper_arm_bottom_joint_state_msg.velocity.push_back(0.0);
    
    right_upper_arm_top_joint_state_msg.header.stamp = ros::Time::now();
    right_upper_arm_top_joint_state_msg.name.push_back("right_upper_arm_top");
    right_upper_arm_top_joint_state_msg.position.push_back(0.0);
    right_upper_arm_top_joint_state_msg.velocity.push_back(0.0);
    
    right_forearm_bottom_joint_state_msg.header.stamp = ros::Time::now();
    right_forearm_bottom_joint_state_msg.name.push_back("right_forearm_bottom_joint");
    right_forearm_bottom_joint_state_msg.position.push_back(0.0);
    right_forearm_bottom_joint_state_msg.velocity.push_back(0.0);
    
    right_forearm_top_joint_state_msg.header.stamp = ros::Time::now();
    right_forearm_top_joint_state_msg.name.push_back("right_forearm_top");
    right_forearm_top_joint_state_msg.position.push_back(0.0);
    right_forearm_top_joint_state_msg.velocity.push_back(0.0);
    

    //Set the iniail values for the left hand joint messages
    left_shoulder_joint_state_msg.header.stamp = ros::Time::now();
    left_shoulder_joint_state_msg.name.push_back("left_shoulder");
    left_shoulder_joint_state_msg.position.push_back(0.0);
    left_shoulder_joint_state_msg.velocity.push_back(0.0);
    
    left_upper_arm_bottom_joint_state_msg.header.stamp = ros::Time::now();
    left_upper_arm_bottom_joint_state_msg.name.push_back("left_upper_arm_bottom");
    left_upper_arm_bottom_joint_state_msg.position.push_back(0.0);
    left_upper_arm_bottom_joint_state_msg.velocity.push_back(0.0);
    
    left_upper_arm_top_joint_state_msg.header.stamp = ros::Time::now();
    left_upper_arm_top_joint_state_msg.name.push_back("left_upper_arm_top");
    left_upper_arm_top_joint_state_msg.position.push_back(0.0);
    left_upper_arm_top_joint_state_msg.velocity.push_back(0.0);
    
    left_forearm_bottom_joint_state_msg.header.stamp = ros::Time::now();
    left_forearm_bottom_joint_state_msg.name.push_back("left_forearm_bottom_joint");
    left_forearm_bottom_joint_state_msg.position.push_back(0.0);
    left_forearm_bottom_joint_state_msg.velocity.push_back(0.0);
    
    left_forearm_top_joint_state_msg.header.stamp = ros::Time::now();
    left_forearm_top_joint_state_msg.name.push_back("left_forearm_top");
    left_forearm_top_joint_state_msg.position.push_back(0.0);
    left_forearm_top_joint_state_msg.velocity.push_back(0.0);
  
    //Finished Initializing messages
    int i = 0;
    //here is the main controller loop:
    while (ros::ok()) { 


	i++;

       
    	//Move Torso to postion   
        moveJointToPosition("torso_joint", torso_effort_cmd, torso_joint_property, torso_joint_state_msg, torso_pos, 1.0);

        //Move the right shoulder to postion   
        moveJointToPosition("right_shoulder_joint", right_shoulder_effort_cmd, right_shoulder_joint_property, right_shoulder_joint_state_msg, right_shoulder_pos, 1.0);
        
        //Move the right upper arm (Vertical shoulder joint) to postion   
        moveJointToPosition("right_upper_arm_bottom_joint", right_upper_arm_bottom_effort_cmd, right_upper_arm_bottom_joint_property, right_upper_arm_bottom_joint_state_msg, right_upper_arm_bottom_pos, 2.0);
        
         //Move the right upper arm (Vertical shoulder joint) to postion   
        moveJointToPosition("right_upper_arm_top_joint", right_upper_arm_top_effort_cmd, right_upper_arm_top_joint_property, right_upper_arm_top_joint_state_msg, right_upper_arm_top_pos, 1.0);
        
        //Move the forarm bottom (Elbow joint) to position
      moveJointToPosition("right_forearm_bottom_joint", right_forearm_bottom_effort_cmd, right_forearm_bottom_joint_property, right_forearm_bottom_joint_state_msg, right_forearm_bottom_pos, 2.0);
       
        //Controlling the forearm top link. This link allows the rotation of the wrist along x-y plane 
        moveJointToPosition("right_forearm_top_joint", right_forearm_top_effort_cmd, right_forearm_top_joint_property, right_forearm_top_joint_state_msg, right_forearm_top_pos, 1.0);
        
        
        //Setting the left hand joint positions
   
        //Move the left shoulder to postion   
        moveJointToPosition("left_shoulder_joint", left_shoulder_effort_cmd, left_shoulder_joint_property, left_shoulder_joint_state_msg, left_shoulder_pos, 1.0);
        
        //Move the left upper arm (Vertical shoulder joint) to postion   
        moveJointToPosition("left_upper_arm_bottom_joint", left_upper_arm_bottom_effort_cmd, left_upper_arm_bottom_joint_property, left_upper_arm_bottom_joint_state_msg, left_upper_arm_bottom_pos, 2.0);
        
        //Move the left upper arm (Vertical shoulder joint) to postion   
        moveJointToPosition("left_upper_arm_top_joint", left_upper_arm_top_effort_cmd, left_upper_arm_top_joint_property, left_upper_arm_top_joint_state_msg, left_upper_arm_top_pos, 1.0);
        
        //Move the forarm bottom (Elbow joint) to position
        moveJointToPosition("left_forearm_bottom_joint", left_forearm_bottom_effort_cmd, left_forearm_bottom_joint_property, left_forearm_bottom_joint_state_msg, left_forearm_bottom_pos, 2.0);
       
        //Controlling the forearm top link. This link allows the rotation of the wrist along x-y plane 
        moveJointToPosition("left_forearm_top_joint", left_forearm_top_effort_cmd, left_forearm_top_joint_property, left_forearm_top_joint_state_msg, left_forearm_top_pos, 1.0);
        
        //ROS_INFO("Count=  %d ", i);
        //ROS_INFO(" ");

        ros::spinOnce();
        rate_timer.sleep();
    }
} 
