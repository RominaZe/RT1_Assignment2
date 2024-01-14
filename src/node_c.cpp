#include "ros/ros.h"
#include "assignment_2_2023/AverageSpeed.h"
#include "geometry_msgs/Twist.h"

assignment_2_2023::AverageSpeed average_speed;
int number_of_messages = 0;
double sum_x = 0, sum_z = 0;


void SpeedCallback(const assignment_2_2023::AverageSpeed::ConstPtr& msg) {
    sum_x += msg->linear_x;
    sum_z += msg->angular_z;
    number_of_messages++;
}

bool AverageSpeedCallback(assignment_2_2023::AverageSpeed::Request& req,
    assignment_2_2023::AverageSpeed::Response& res) {
    if (number_of_messages == 0) {
        ROS_INFO("No messages received");
        return false;
    }
    else{

    res.average_speed.linear_x = sum_x / number_of_messages;
    res.average_speed.angular_z = sum_z / number_of_messages;}
    return true;}
    

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_c_robot_pv");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/position_velocity", 1, SpeedCallback);
    ros::ServiceServer service = nh.advertiseService("/average_speed", AverageSpeedCallback);

    ros::spin();

    return 0;
}

