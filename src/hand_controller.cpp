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
#include "pluto/hand_motion_msg.h"


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


double right_palm_pos = 0.0;

double right_index_bottom_pos = 0.0;
double right_middle_bottom_pos = 0.0;
double right_ring_bottom_pos = 0.0;
double right_pinky_bottom_pos = 0.0;

double right_thumb_bottom_pos = 0.0;
double right_thumb_top_pos = 0.0;
double right_thumb_piece_pos = 0.0;

double right_index_top_pos = 0.0;
double right_middle_top_pos = 0.0;
double right_ring_top_pos = 0.0;
double right_pinky_top_pos = 0.0;



double left_palm_pos = 0.0;

double left_index_bottom_pos = 0.0;
double left_middle_bottom_pos = 0.0;
double left_ring_bottom_pos = 0.0;
double left_pinky_bottom_pos = 0.0;

double left_thumb_bottom_pos = 0.0;
double left_thumb_top_pos = 0.0;
double left_thumb_piece_pos = 0.0;

double left_index_top_pos = 0.0;
double left_middle_top_pos = 0.0;
double left_ring_top_pos = 0.0;
double left_pinky_top_pos = 0.0;


ros::ServiceClient set_trq_client; //client to send Effort commands to Gazebo for controlling the joints
ros::ServiceClient get_jnt_state_client; //client to reciece position and velociy information of each joint from Gazebo

ros::Publisher joint_state_publisher; //Publisher for Joint State Sensor messages


bool posCmdCB(pluto::hand_motion_msgRequest& req, pluto::hand_motion_msgResponse& res) 
{
    if(req.arm == 0)
    {
    	if(req.handMotion == 0)
    	{
    		ROS_INFO("closing right hand");
	    	right_palm_pos = 0.0;

		right_index_bottom_pos = 1.57;
		right_middle_bottom_pos = 1.57;
		right_ring_bottom_pos = 1.57;
		right_pinky_bottom_pos = 1.57;

		right_thumb_bottom_pos = 1.57;
		right_thumb_top_pos = 1.57;
		right_thumb_piece_pos = 0.0;

		right_index_top_pos = 1.57;
		right_middle_top_pos = 1.57;
		right_ring_top_pos = 1.57;
		right_pinky_top_pos = 1.57;    
		
	}
	else
	{
		ROS_INFO("opening right hand");
		right_palm_pos = 0.0;

		right_index_bottom_pos = 0.0;
		right_middle_bottom_pos = 0.0;
		right_ring_bottom_pos = 0.0;
		right_pinky_bottom_pos = 0.0;

		right_thumb_bottom_pos = 0.0;
		right_thumb_top_pos = 0.0;
		right_thumb_piece_pos = 0.0;

		right_index_top_pos = 0.0;
		right_middle_top_pos = 0.0;
		right_ring_top_pos = 0.0;
		right_pinky_top_pos = 0.0;   
	}
    }
    
    else if(req.arm == 1)
    {
    	if(req.handMotion == 0)
    	{
		left_palm_pos = 0.0;

		left_index_bottom_pos = 1.57;
		left_middle_bottom_pos = 1.57;
		left_ring_bottom_pos = 1.57;
		left_pinky_bottom_pos = 1.57;

		left_thumb_bottom_pos = 1.57;
		left_thumb_top_pos = 1.57;
		left_thumb_piece_pos = 1.57;

		left_index_top_pos = 1.57;
		left_middle_top_pos = 1.57;
		left_ring_top_pos = 1.57;
		left_pinky_top_pos = 1.57;
	}
	else
	{
		left_palm_pos = 0.0;

		left_index_bottom_pos = 0.0;
		left_middle_bottom_pos = 0.0;
		left_ring_bottom_pos = 0.0;
		left_pinky_bottom_pos = 0.0;

		left_thumb_bottom_pos = 0.0;
		left_thumb_top_pos = 0.0;
		left_thumb_piece_pos = 0.0;

		left_index_top_pos = 0.0;
		left_middle_top_pos = 0.0;
		left_ring_top_pos = 0.0;
		left_pinky_top_pos = 0.0;
	}
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
    ros::init(argc, argv, "hand_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);

    // make sure services are available before attempting to proceed, else node will crash
    while (!test_services()) {
        ros::spinOnce();
        half_sec.sleep();
    }

    ros::ServiceServer service = nh.advertiseService("hand_controller", posCmdCB);
    
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

 
    //Message to send effort command and receive position and velocity info to/from forearm top revolute joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_palm_effort_cmd;
    gazebo_msgs::GetJointProperties right_palm_joint_property;  
    
    //Message to send effort command and receive position and velocity info to/from right index finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_index_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties right_index_bottom_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from right middle finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_middle_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties right_middle_bottom_joint_property; 
    
     //Message to send effort command and receive position and velocity info to/from right ring finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_ring_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties right_ring_bottom_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from right pinky finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_pinky_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties right_pinky_bottom_joint_property; 
    
    
    //Message to send effort command and receive position and velocity info to/from right index finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_index_top_effort_cmd;
    gazebo_msgs::GetJointProperties right_index_top_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from right middle finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_middle_top_effort_cmd;
    gazebo_msgs::GetJointProperties right_middle_top_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from right ring finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_ring_top_effort_cmd;
    gazebo_msgs::GetJointProperties right_ring_top_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from right pinky finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_pinky_top_effort_cmd;
    gazebo_msgs::GetJointProperties right_pinky_top_joint_property;
    
        
    //Message to send effort command and receive position and velocity info to/from right thumb piece joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_thumb_piece_effort_cmd;
    gazebo_msgs::GetJointProperties right_thumb_piece_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from right thumb bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_thumb_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties right_thumb_bottom_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from right thumb bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort right_thumb_top_effort_cmd;
    gazebo_msgs::GetJointProperties right_thumb_top_joint_property;  
 
    
    //messages that will be received from joint state publisher for each joint
    sensor_msgs::JointState right_palm_joint_state_msg;
    sensor_msgs::JointState right_index_bottom_joint_state_msg;
    sensor_msgs::JointState right_middle_bottom_joint_state_msg;
    sensor_msgs::JointState right_ring_bottom_joint_state_msg;
    sensor_msgs::JointState right_pinky_bottom_joint_state_msg;
    
    sensor_msgs::JointState right_index_top_joint_state_msg;
    sensor_msgs::JointState right_middle_top_joint_state_msg;
    sensor_msgs::JointState right_ring_top_joint_state_msg;
    sensor_msgs::JointState right_pinky_top_joint_state_msg;
    
    sensor_msgs::JointState right_thumb_piece_joint_state_msg;
    sensor_msgs::JointState right_thumb_bottom_joint_state_msg;
    sensor_msgs::JointState right_thumb_top_joint_state_msg;
    
    
    //Messages for left hand joints
    //Message to send effort command and receive position and velocity info to/from forearm top revolute joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_palm_effort_cmd;
    gazebo_msgs::GetJointProperties left_palm_joint_property;  
    
    //Message to send effort command and receive position and velocity info to/from left index finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_index_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties left_index_bottom_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from left middle finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_middle_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties left_middle_bottom_joint_property; 
    
     //Message to send effort command and receive position and velocity info to/from left ring finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_ring_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties left_ring_bottom_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from left pinky finger bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_pinky_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties left_pinky_bottom_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from left index finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_index_top_effort_cmd;
    gazebo_msgs::GetJointProperties left_index_top_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from left middle finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_middle_top_effort_cmd;
    gazebo_msgs::GetJointProperties left_middle_top_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from left ring finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_ring_top_effort_cmd;
    gazebo_msgs::GetJointProperties left_ring_top_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from left pinky finger top joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_pinky_top_effort_cmd;
    gazebo_msgs::GetJointProperties left_pinky_top_joint_property;
        
    //Message to send effort command and receive position and velocity info to/from left thumb piece joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_thumb_piece_effort_cmd;
    gazebo_msgs::GetJointProperties left_thumb_piece_joint_property;
    
    //Message to send effort command and receive position and velocity info to/from left thumb bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_thumb_bottom_effort_cmd;
    gazebo_msgs::GetJointProperties left_thumb_bottom_joint_property; 
    
    //Message to send effort command and receive position and velocity info to/from left thumb bottom joint 
    //This joint controls the movement of the write in xy plane
    gazebo_msgs::ApplyJointEffort left_thumb_top_effort_cmd;
    gazebo_msgs::GetJointProperties left_thumb_top_joint_property;  
 
    
    //messages that will be received from joint state publisher for each joint
    sensor_msgs::JointState left_palm_joint_state_msg;
    sensor_msgs::JointState left_index_bottom_joint_state_msg;
    sensor_msgs::JointState left_middle_bottom_joint_state_msg;
    sensor_msgs::JointState left_ring_bottom_joint_state_msg;
    sensor_msgs::JointState left_pinky_bottom_joint_state_msg;
    
    sensor_msgs::JointState left_index_top_joint_state_msg;
    sensor_msgs::JointState left_middle_top_joint_state_msg;
    sensor_msgs::JointState left_ring_top_joint_state_msg;
    sensor_msgs::JointState left_pinky_top_joint_state_msg;
    
    sensor_msgs::JointState left_thumb_piece_joint_state_msg;
    sensor_msgs::JointState left_thumb_bottom_joint_state_msg;
    sensor_msgs::JointState left_thumb_top_joint_state_msg;


    //Initialize Messages 
    ros::Duration duration(dt);
    ros::Rate rate_timer(1 / dt);
    
    right_palm_effort_cmd.request.joint_name = "right_palm_joint";
    right_palm_effort_cmd.request.effort = 0.0;
    right_palm_effort_cmd.request.duration = duration;
    right_palm_joint_property.request.joint_name = "right_palm_joint";
    
    right_index_bottom_effort_cmd.request.joint_name = "right_index_bottom_joint";
    right_index_bottom_effort_cmd.request.effort = 0.0;
    right_index_bottom_effort_cmd.request.duration = duration;
    right_index_bottom_joint_property.request.joint_name = "right_index_bottom_joint";
    
    right_middle_bottom_effort_cmd.request.joint_name = "right_middle_bottom_joint";
    right_middle_bottom_effort_cmd.request.effort = 0.0;
    right_middle_bottom_effort_cmd.request.duration = duration;
    right_middle_bottom_joint_property.request.joint_name = "right_middle_bottom_joint";
    
    right_ring_bottom_effort_cmd.request.joint_name = "right_ring_bottom_joint";
    right_ring_bottom_effort_cmd.request.effort = 0.0;
    right_ring_bottom_effort_cmd.request.duration = duration;
    right_ring_bottom_joint_property.request.joint_name = "right_ring_bottom_joint";
    
    right_pinky_bottom_effort_cmd.request.joint_name = "right_pinky_bottom_joint";
    right_pinky_bottom_effort_cmd.request.effort = 0.0;
    right_pinky_bottom_effort_cmd.request.duration = duration;
    right_pinky_bottom_joint_property.request.joint_name = "right_pinky_bottom_joint";
    
    
    
    right_index_top_effort_cmd.request.joint_name = "right_index_top_joint";
    right_index_top_effort_cmd.request.effort = 0.0;
    right_index_top_effort_cmd.request.duration = duration;
    right_index_top_joint_property.request.joint_name = "right_index_top_joint";
    
    right_middle_top_effort_cmd.request.joint_name = "right_middle_top_joint";
    right_middle_top_effort_cmd.request.effort = 0.0;
    right_middle_top_effort_cmd.request.duration = duration;
    right_middle_top_joint_property.request.joint_name = "right_middle_top_joint";
    
    right_ring_top_effort_cmd.request.joint_name = "right_ring_top_joint";
    right_ring_top_effort_cmd.request.effort = 0.0;
    right_ring_top_effort_cmd.request.duration = duration;
    right_ring_top_joint_property.request.joint_name = "right_ring_top_joint";
    
    right_pinky_top_effort_cmd.request.joint_name = "right_pinky_top_joint";
    right_pinky_top_effort_cmd.request.effort = 0.0;
    right_pinky_top_effort_cmd.request.duration = duration;
    right_pinky_top_joint_property.request.joint_name = "right_pinky_top_joint";
    
    right_thumb_piece_effort_cmd.request.joint_name = "right_thumb_piece_joint";
    right_thumb_piece_effort_cmd.request.effort = 0.0;
    right_thumb_piece_effort_cmd.request.duration = duration;
    right_thumb_piece_joint_property.request.joint_name = "right_thumb_piece_joint";
    
    right_thumb_bottom_effort_cmd.request.joint_name = "right_thumb_bottom_joint";
    right_thumb_bottom_effort_cmd.request.effort = 0.0;
    right_thumb_bottom_effort_cmd.request.duration = duration;
    right_thumb_bottom_joint_property.request.joint_name = "right_thumb_bottom_joint";
    
    right_thumb_top_effort_cmd.request.joint_name = "right_thumb_top_joint";
    right_thumb_top_effort_cmd.request.effort = 0.0;
    right_thumb_top_effort_cmd.request.duration = duration;
    right_thumb_top_joint_property.request.joint_name = "right_thumb_top_joint";
    


    //Initialize the joint command messages for the left arm
    left_palm_effort_cmd.request.joint_name = "left_palm_joint";
    left_palm_effort_cmd.request.effort = 0.0;
    left_palm_effort_cmd.request.duration = duration;
    left_palm_joint_property.request.joint_name = "left_palm_joint";
    
    left_index_bottom_effort_cmd.request.joint_name = "left_index_bottom_joint";
    left_index_bottom_effort_cmd.request.effort = 0.0;
    left_index_bottom_effort_cmd.request.duration = duration;
    left_index_bottom_joint_property.request.joint_name = "left_index_bottom_joint";
    
    left_middle_bottom_effort_cmd.request.joint_name = "left_middle_bottom_joint";
    left_middle_bottom_effort_cmd.request.effort = 0.0;
    left_middle_bottom_effort_cmd.request.duration = duration;
    left_middle_bottom_joint_property.request.joint_name = "left_middle_bottom_joint";
    
    left_ring_bottom_effort_cmd.request.joint_name = "left_ring_bottom_joint";
    left_ring_bottom_effort_cmd.request.effort = 0.0;
    left_ring_bottom_effort_cmd.request.duration = duration;
    left_ring_bottom_joint_property.request.joint_name = "left_ring_bottom_joint";
    
    left_pinky_bottom_effort_cmd.request.joint_name = "left_pinky_bottom_joint";
    left_pinky_bottom_effort_cmd.request.effort = 0.0;
    left_pinky_bottom_effort_cmd.request.duration = duration;
    left_pinky_bottom_joint_property.request.joint_name = "left_pinky_bottom_joint";
    
    
    
    left_index_top_effort_cmd.request.joint_name = "left_index_top_joint";
    left_index_top_effort_cmd.request.effort = 0.0;
    left_index_top_effort_cmd.request.duration = duration;
    left_index_top_joint_property.request.joint_name = "left_index_top_joint";
    
    left_middle_top_effort_cmd.request.joint_name = "left_middle_top_joint";
    left_middle_top_effort_cmd.request.effort = 0.0;
    left_middle_top_effort_cmd.request.duration = duration;
    left_middle_top_joint_property.request.joint_name = "left_middle_top_joint";
    
    left_ring_top_effort_cmd.request.joint_name = "left_ring_top_joint";
    left_ring_top_effort_cmd.request.effort = 0.0;
    left_ring_top_effort_cmd.request.duration = duration;
    left_ring_top_joint_property.request.joint_name = "left_ring_top_joint";
    
    left_pinky_top_effort_cmd.request.joint_name = "left_pinky_top_joint";
    left_pinky_top_effort_cmd.request.effort = 0.0;
    left_pinky_top_effort_cmd.request.duration = duration;
    left_pinky_top_joint_property.request.joint_name = "left_pinky_top_joint";
    
    left_thumb_piece_effort_cmd.request.joint_name = "left_thumb_piece_joint";
    left_thumb_piece_effort_cmd.request.effort = 0.0;
    left_thumb_piece_effort_cmd.request.duration = duration;
    left_thumb_piece_joint_property.request.joint_name = "left_thumb_piece_joint";
    
    left_thumb_bottom_effort_cmd.request.joint_name = "left_thumb_bottom_joint";
    left_thumb_bottom_effort_cmd.request.effort = 0.0;
    left_thumb_bottom_effort_cmd.request.duration = duration;
    left_thumb_bottom_joint_property.request.joint_name = "left_thumb_bottom_joint";
    
    left_thumb_top_effort_cmd.request.joint_name = "left_thumb_top_joint";
    left_thumb_top_effort_cmd.request.effort = 0.0;
    left_thumb_top_effort_cmd.request.duration = duration;
    left_thumb_top_joint_property.request.joint_name = "left_thumb_top_joint";

    // Setup the initial position and velocity values of each joint
    right_palm_joint_state_msg.header.stamp = ros::Time::now();
    right_palm_joint_state_msg.name.push_back("right_palm");
    right_palm_joint_state_msg.position.push_back(0.0);
    right_palm_joint_state_msg.velocity.push_back(0.0);
    
    right_index_bottom_joint_state_msg.header.stamp = ros::Time::now();
    right_index_bottom_joint_state_msg.name.push_back("right_index_bottom");
    right_index_bottom_joint_state_msg.position.push_back(0.0);
    right_index_bottom_joint_state_msg.velocity.push_back(0.0);
    
    right_middle_bottom_joint_state_msg.header.stamp = ros::Time::now();
    right_middle_bottom_joint_state_msg.name.push_back("right_middle_bottom");
    right_middle_bottom_joint_state_msg.position.push_back(0.0);
    right_middle_bottom_joint_state_msg.velocity.push_back(0.0);
    
    right_ring_bottom_joint_state_msg.header.stamp = ros::Time::now();
    right_ring_bottom_joint_state_msg.name.push_back("right_ring_bottom");
    right_ring_bottom_joint_state_msg.position.push_back(0.0);
    right_ring_bottom_joint_state_msg.velocity.push_back(0.0);
    
    right_pinky_bottom_joint_state_msg.header.stamp = ros::Time::now();
    right_pinky_bottom_joint_state_msg.name.push_back("right_pinky_bottom");
    right_pinky_bottom_joint_state_msg.position.push_back(0.0);
    right_pinky_bottom_joint_state_msg.velocity.push_back(0.0);
    
    
    
    right_index_top_joint_state_msg.header.stamp = ros::Time::now();
    right_index_top_joint_state_msg.name.push_back("right_index_top");
    right_index_top_joint_state_msg.position.push_back(0.0);
    right_index_top_joint_state_msg.velocity.push_back(0.0);
    
    right_middle_top_joint_state_msg.header.stamp = ros::Time::now();
    right_middle_top_joint_state_msg.name.push_back("right_middle_top");
    right_middle_top_joint_state_msg.position.push_back(0.0);
    right_middle_top_joint_state_msg.velocity.push_back(0.0);
    
    right_ring_top_joint_state_msg.header.stamp = ros::Time::now();
    right_ring_top_joint_state_msg.name.push_back("right_ring_top");
    right_ring_top_joint_state_msg.position.push_back(0.0);
    right_ring_top_joint_state_msg.velocity.push_back(0.0);
    
    right_pinky_top_joint_state_msg.header.stamp = ros::Time::now();
    right_pinky_top_joint_state_msg.name.push_back("right_pinky_top");
    right_pinky_top_joint_state_msg.position.push_back(0.0);
    right_pinky_top_joint_state_msg.velocity.push_back(0.0);
    
    
    right_thumb_piece_joint_state_msg.header.stamp = ros::Time::now();
    right_thumb_piece_joint_state_msg.name.push_back("right_thumb_piece");
    right_thumb_piece_joint_state_msg.position.push_back(0.0);
    right_thumb_piece_joint_state_msg.velocity.push_back(0.0);
    
    right_thumb_bottom_joint_state_msg.header.stamp = ros::Time::now();
    right_thumb_bottom_joint_state_msg.name.push_back("right_thumb_bottom");
    right_thumb_bottom_joint_state_msg.position.push_back(0.0);
    right_thumb_bottom_joint_state_msg.velocity.push_back(0.0);
    
    right_thumb_top_joint_state_msg.header.stamp = ros::Time::now();
    right_thumb_top_joint_state_msg.name.push_back("right_top_bottom");
    right_thumb_top_joint_state_msg.position.push_back(0.0);
    right_thumb_top_joint_state_msg.velocity.push_back(0.0);
    
    //Set the iniail values for the left hand joint messages
    left_palm_joint_state_msg.header.stamp = ros::Time::now();
    left_palm_joint_state_msg.name.push_back("left_palm");
    left_palm_joint_state_msg.position.push_back(0.0);
    left_palm_joint_state_msg.velocity.push_back(0.0);
    
    left_index_bottom_joint_state_msg.header.stamp = ros::Time::now();
    left_index_bottom_joint_state_msg.name.push_back("left_index_bottom");
    left_index_bottom_joint_state_msg.position.push_back(0.0);
    left_index_bottom_joint_state_msg.velocity.push_back(0.0);
    
    left_middle_bottom_joint_state_msg.header.stamp = ros::Time::now();
    left_middle_bottom_joint_state_msg.name.push_back("left_middle_bottom");
    left_middle_bottom_joint_state_msg.position.push_back(0.0);
    left_middle_bottom_joint_state_msg.velocity.push_back(0.0);
    
    left_ring_bottom_joint_state_msg.header.stamp = ros::Time::now();
    left_ring_bottom_joint_state_msg.name.push_back("left_ring_bottom");
    left_ring_bottom_joint_state_msg.position.push_back(0.0);
    left_ring_bottom_joint_state_msg.velocity.push_back(0.0);
    
    left_pinky_bottom_joint_state_msg.header.stamp = ros::Time::now();
    left_pinky_bottom_joint_state_msg.name.push_back("left_pinky_bottom");
    left_pinky_bottom_joint_state_msg.position.push_back(0.0);
    left_pinky_bottom_joint_state_msg.velocity.push_back(0.0);
    
    
    
    left_index_top_joint_state_msg.header.stamp = ros::Time::now();
    left_index_top_joint_state_msg.name.push_back("left_index_top");
    left_index_top_joint_state_msg.position.push_back(0.0);
    left_index_top_joint_state_msg.velocity.push_back(0.0);
    
    left_middle_top_joint_state_msg.header.stamp = ros::Time::now();
    left_middle_top_joint_state_msg.name.push_back("left_middle_top");
    left_middle_top_joint_state_msg.position.push_back(0.0);
    left_middle_top_joint_state_msg.velocity.push_back(0.0);
    
    left_ring_top_joint_state_msg.header.stamp = ros::Time::now();
    left_ring_top_joint_state_msg.name.push_back("left_ring_top");
    left_ring_top_joint_state_msg.position.push_back(0.0);
    left_ring_top_joint_state_msg.velocity.push_back(0.0);
    
    left_pinky_top_joint_state_msg.header.stamp = ros::Time::now();
    left_pinky_top_joint_state_msg.name.push_back("left_pinky_top");
    left_pinky_top_joint_state_msg.position.push_back(0.0);
    left_pinky_top_joint_state_msg.velocity.push_back(0.0);
    
    
    left_thumb_piece_joint_state_msg.header.stamp = ros::Time::now();
    left_thumb_piece_joint_state_msg.name.push_back("left_thumb_piece");
    left_thumb_piece_joint_state_msg.position.push_back(0.0);
    left_thumb_piece_joint_state_msg.velocity.push_back(0.0);
    
    left_thumb_bottom_joint_state_msg.header.stamp = ros::Time::now();
    left_thumb_bottom_joint_state_msg.name.push_back("left_thumb_bottom");
    left_thumb_bottom_joint_state_msg.position.push_back(0.0);
    left_thumb_bottom_joint_state_msg.velocity.push_back(0.0);
    
    left_thumb_top_joint_state_msg.header.stamp = ros::Time::now();
    left_thumb_top_joint_state_msg.name.push_back("left_top_bottom");
    left_thumb_top_joint_state_msg.position.push_back(0.0);
    left_thumb_top_joint_state_msg.velocity.push_back(0.0);
    
    //Finished Initializing messages
    int i = 0;
    //here is the main controller loop:
    while (ros::ok()) { 
    

	i++;

        
        //Controlling the wrist link. This link allows the rotation of the wrist
        moveJointToPosition("right_palm_joint", right_palm_effort_cmd, right_palm_joint_property, right_palm_joint_state_msg, right_palm_pos, 1.0);
        
        //Controlling the right index bottom link. This link allows the rotation of the index finger top against base of the finger 
        moveJointToPosition("right_index_top_joint", right_index_top_effort_cmd, right_index_top_joint_property, right_index_top_joint_state_msg, right_index_top_pos, 0.0001);
        
         //Controlling the right middle bottom link. This link allows the rotation of the middle finger top against base of the finger 
        moveJointToPosition("right_middle_top_joint", right_middle_top_effort_cmd, right_middle_top_joint_property, right_middle_top_joint_state_msg, right_middle_top_pos, 0.0001);
        
         //Controlling the right ring bottom link. This link allows the rotation of the ring finger top against base of the finger 
        moveJointToPosition("right_ring_top_joint", right_ring_top_effort_cmd, right_ring_top_joint_property, right_ring_top_joint_state_msg, right_ring_top_pos, 0.0001);
        
         //Controlling the right pinky bottom link. This link allows the rotation of the pinky finger top against base of the finger 
        moveJointToPosition("right_pinky_top_joint", right_pinky_top_effort_cmd, right_pinky_top_joint_property, right_pinky_top_joint_state_msg, right_pinky_top_pos, 0.0001);
        
        //Controlling the right thump piece link. This link allows the rotation of the thump piece top against the palm 
        moveJointToPosition("right_thumb_piece_joint", right_thumb_piece_effort_cmd, right_thumb_piece_joint_property, right_thumb_piece_joint_state_msg, right_thumb_piece_pos, 0.002); //2
        
	//Controlling the right index bottom link. This link allows the rotation of the index finger against the palm 
        moveJointToPosition("right_thumb_bottom_joint", right_thumb_bottom_effort_cmd, right_thumb_bottom_joint_property, right_thumb_bottom_joint_state_msg, right_thumb_bottom_pos, 0.002);
        
        //Controlling the right index bottom link. This link allows the rotation of the index finger against the palm 
        moveJointToPosition("right_thumb_top_joint", right_thumb_top_effort_cmd, right_thumb_top_joint_property, right_thumb_top_joint_state_msg, right_thumb_top_pos, 0.0001);
        
         //Controlling the right index bottom link. This link allows the rotation of the index finger against the palm 
        moveJointToPosition("right_index_bottom_joint", right_index_bottom_effort_cmd, right_index_bottom_joint_property, right_index_bottom_joint_state_msg, right_index_bottom_pos, 0.002);
        
        //Controlling the right middle bottom link. This link allows the rotation of the middle finger against the palm 
        moveJointToPosition("right_middle_bottom_joint", right_middle_bottom_effort_cmd, right_middle_bottom_joint_property, right_middle_bottom_joint_state_msg, right_middle_bottom_pos, 0.002);
        
        //Controlling the right middle bottom link. This link allows the rotation of the middle finger against the palm 
        moveJointToPosition("right_ring_bottom_joint", right_ring_bottom_effort_cmd, right_ring_bottom_joint_property, right_ring_bottom_joint_state_msg, right_ring_bottom_pos, 0.002);
        
        //Controlling the right middle bottom link. This link allows the rotation of the middle finger against the palm 
        moveJointToPosition("right_pinky_bottom_joint", right_pinky_bottom_effort_cmd, right_pinky_bottom_joint_property, right_pinky_bottom_joint_state_msg, right_pinky_bottom_pos, 0.002);
        
       
        
        //Setting the left hand joint positions
        //Controlling the wrist link. This link allows the rotation of the wrist
        moveJointToPosition("left_palm_joint", left_palm_effort_cmd, left_palm_joint_property, left_palm_joint_state_msg, left_palm_pos, 1.0);
        
        //Controlling the left index bottom link. This link allows the rotation of the index finger against the palm 
        moveJointToPosition("left_index_bottom_joint", left_index_bottom_effort_cmd, left_index_bottom_joint_property, left_index_bottom_joint_state_msg, left_index_bottom_pos, 0.0001);
        
        //Controlling the left middle bottom link. This link allows the rotation of the middle finger against the palm 
        moveJointToPosition("left_middle_bottom_joint", left_middle_bottom_effort_cmd, left_middle_bottom_joint_property, left_middle_bottom_joint_state_msg, left_middle_bottom_pos, 0.0001);
        
        //Controlling the left middle bottom link. This link allows the rotation of the middle finger against the palm 
        moveJointToPosition("left_ring_bottom_joint", left_ring_bottom_effort_cmd, left_ring_bottom_joint_property, left_ring_bottom_joint_state_msg, left_ring_bottom_pos, 0.0001);
        
        //Controlling the left middle bottom link. This link allows the rotation of the middle finger against the palm 
        moveJointToPosition("left_pinky_bottom_joint", left_pinky_bottom_effort_cmd, left_pinky_bottom_joint_property, left_pinky_bottom_joint_state_msg, left_pinky_bottom_pos, 0.0001);
        
        //Controlling the left index bottom link. This link allows the rotation of the index finger top against base of the finger 
        moveJointToPosition("left_index_top_joint", left_index_top_effort_cmd, left_index_top_joint_property, left_index_top_joint_state_msg, left_index_top_pos, 0.0001);
        
         //Controlling the left middle bottom link. This link allows the rotation of the middle finger top against base of the finger 
        moveJointToPosition("left_middle_top_joint", left_middle_top_effort_cmd, left_middle_top_joint_property, left_middle_top_joint_state_msg, left_middle_top_pos, 0.0001);
        
         //Controlling the left ring bottom link. This link allows the rotation of the ring finger top against base of the finger 
        moveJointToPosition("left_ring_top_joint", left_ring_top_effort_cmd, left_ring_top_joint_property, left_ring_top_joint_state_msg, left_ring_top_pos, 0.0001);
        
         //Controlling the left pinky bottom link. This link allows the rotation of the pinky finger top against base of the finger 
        moveJointToPosition("left_pinky_top_joint", left_pinky_top_effort_cmd, left_pinky_top_joint_property, left_pinky_top_joint_state_msg, left_pinky_top_pos, 0.0001);
        
        //Controlling the left thump piece link. This link allows the rotation of the thump piece top against the palm 
        moveJointToPosition("left_thumb_piece_joint", left_thumb_piece_effort_cmd, left_thumb_piece_joint_property, left_thumb_piece_joint_state_msg, left_thumb_piece_pos, 0.002);
        
	//Controlling the left index bottom link. This link allows the rotation of the index finger against the palm 
        moveJointToPosition("left_thumb_bottom_joint", left_thumb_bottom_effort_cmd, left_thumb_bottom_joint_property, left_thumb_bottom_joint_state_msg, left_thumb_bottom_pos, 0.001);
        
        //Controlling the left index bottom link. This link allows the rotation of the index finger against the palm 
        moveJointToPosition("left_thumb_top_joint", left_thumb_top_effort_cmd, left_thumb_top_joint_property, left_thumb_top_joint_state_msg, left_thumb_top_pos, 0.001);
        
        //ROS_INFO("Count=  %d ", i);
        //ROS_INFO(" ");

        ros::spinOnce();
        rate_timer.sleep();
    }
} 
