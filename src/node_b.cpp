#include "ros/ros.h"
#include "assignment_2_2023/PlanningAction.h"
#include "geometry_msgs/Point.h"
#include "assignment_2_2023/LastPosition.h"


double x = 0, y = 0;

void GetPosition(const assignment_2_2023::PlanningActionGoal::ConstPtr& msg){
	x = msg ->goal.target_pose.pose.position.x;
	y = msg ->goal.target_pose.pose.position.y;
}


bool targetCallback(assignment_2_2023::LastPosition::Request& req,
	assignment_2_2023::LastPosition::Response& res){ 
    
    res.last_position_x = x;
    res.last_position_y = y;
    ROS_INFO("The last goal position was x: %f, y: %f", x, y);
    return true;
    
}


int main(int argc, char** argv){
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/reaching_goal/goal",1, GetPosition);
    ros::ServiceServer service = nh.advertiseService("/last_goal", targetCallback);
    
    while (ros::ok()) {
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();}
    return 0;
}
