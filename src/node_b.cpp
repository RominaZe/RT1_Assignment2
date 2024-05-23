/**
 * \file node_b.cpp
 * \brief Report the last goal define in  node A
 * \author Romina Contreras
 * \version 0.1
 * \date 01/03/2024
 *
 * \param [out] x Use to allocate the x axis goal.
 * \param [out] y Use to allocate the y axis goal. 
 *
 * Subscriber: <BR>
 *  °/reaching_goal/goal
 *
 * Publisher: <BR>
 *  °None
 *
 * Service: <BR>
 * °/last_goal
 *
 * Description:
 *
 * This node operates as a service node that, upon invocation, returns the coordinates of the last target set by the user in the node A.
 * 
 */


#include "ros/ros.h"
#include "assignment_2_2023/PlanningAction.h"
#include "geometry_msgs/Point.h"
#include "assignment_2_2023/LastPosition.h"


double x = 0, y = 0;

/**
 * \brief Use to put in the parameter x and y the actual last goal position.
 *
 * \param x
 * \param y
 *
 * \return void
 *
 * This function is the callbak of the subscriber and is use to put the message of planning on the variables x and y, infact this change the value. Then the service will give back the number that are allocate here.
 */
void GetPosition(const assignment_2_2023::PlanningActionGoal::ConstPtr& msg){
	x = msg ->goal.target_pose.pose.position.x;
	y = msg ->goal.target_pose.pose.position.y;
}

/**
 * \brief This function implement the service, when is call they give the user the last value of the goal.
 *
 * \param req Define the request that is ask by the service.
 * \param res Define the response of the service.
 *
 * \return  true
 *
 */
bool targetCallback(assignment_2_2023::LastPosition::Request& req,
	assignment_2_2023::LastPosition::Response& res){ 
    
    res.last_position_x = x;
    res.last_position_y = y;
    ROS_INFO("The last goal position was x: %f, y: %f", x, y);
    return true;
    
}


/**
 * \brief Main function
 * This in the main function where the the node is initialized as node_b. This ndoe is subscribe to the topic /reaching_goal/goal and generate a callback where there is allocated the actual goal of the robot. Then there is implemented a service call /last_call. When call it gives to the user the target position of the last goal
 *
 */

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
