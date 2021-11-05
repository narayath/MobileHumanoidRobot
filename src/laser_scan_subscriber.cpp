#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

float forwardsDist;
float leftDist;
float backwardsDist;
float rightDist;


void laserScanCallback(const sensor_msgs::LaserScan message)
{
    forwardsDist = message.ranges[0];
    ROS_INFO("Distance at -180 or 180, forward %f", forwardsDist);
    leftDist = message.ranges[179];
    ROS_INFO("Distance at -90, left %f", leftDist);
    backwardsDist = message.ranges[359];
    ROS_INFO("Distance at 0, backward %f", backwardsDist);
    rightDist = message.ranges[539];
    ROS_INFO("Distance at 90, right %f", rightDist); 

}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "laser_scan_subscriber");
    ros::NodeHandle n;
    ros::Subscriber scan_subscriber = n.subscribe("/navigation_robot/laser/scan", 1, laserScanCallback);
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);    
    ros::Rate loop_rate(1.0);
    
    geometry_msgs::Twist moveMsg;
    moveMsg.linear.x = 0.0;
    moveMsg.linear.y = 0.0;
    moveMsg.linear.z = 0.0;
    moveMsg.angular.x = 0.0;
    moveMsg.angular.y = 0.0;
    moveMsg.angular.z = 0.0;

    while(ros::ok())
    {
        ros::spinOnce();
        
        if(forwardsDist > 1.5)
        {
            moveMsg.linear.x = -0.5;
            moveMsg.angular.z = 0.0;
        }
        else
        {
            moveMsg.linear.x = 0.0;
            if(rightDist > 2)
            {
                moveMsg.angular.z = -2.0;
            }
            else if(leftDist > 2)
            {
                moveMsg.angular.z = 2.0;
            }
        }
        
        velocity_publisher.publish(moveMsg);
        loop_rate.sleep();
    }
    
    return 0;

}
