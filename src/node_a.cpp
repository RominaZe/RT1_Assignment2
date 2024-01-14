#include "ros/ros.h"
#include "nav_msgs/Odometry.h" //this is use for the odom message

#include "geometry_msgs/Point.h" 
#include "geometry_msgs/Pose.h" //This provide us the Pose message
#include "geometry_msgs/Twist.h" //this is the message for the velocity

#include "assignment_2_2023/RobotPV.h" // This is the custom message that we create

#include "actionlib/client/simple_action_client.h"
#include "assignment_2_2023/PlanningAction.h"
#include "assignment_2_2023/PlanningGoal.h"
#include "actionlib_msgs/GoalStatus.h"
#include "thread"


ros::Publisher pub;
ros::Subscriber sub;

void CallbackPositionVelocity(const nav_msgs::Odometry::ConstPtr& msg){

	geometry_msgs::Point pos; //type point is composed by x, y and z
	geometry_msgs::Twist vel; //type twist is composed by linear and angular velocity

	pos = msg->pose.pose.position;
	vel= msg->twist.twist;

	//Declare type of message (All this float are the name of the topic)
	assignment_2_2023::RobotPV position_velocity;
	position_velocity.x = pos.x; //data is the value of the topic
	position_velocity.y = pos.y;
	position_velocity.linear_x = vel.linear.x;
	position_velocity.angular_z = vel.angular.z;
 	
 	//Publish the position and velocity of the robot
	pub.publish(position_velocity);
}

void ClientGoal(){
    actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> client("/reaching_goal", true);
    client.waitForServer();

    while (ros::ok()) {
        // Get the current goal position
        double x, y;
        ros::param::get("/des_pos_x", x);
        ros::param::get("/des_pos_y", y);
        
        // Create a new PlanningGoal
        assignment_2_2023::PlanningGoal target;
        target.target_pose.pose.position.x = x;
        target.target_pose.pose.position.y = y;

        ROS_INFO("Current goal: target_x = %f, target_y = %f", x, y);
        
        // Ask for new goal coordinates
        ROS_INFO("Set the coordinates of the goal");
        double input_x, input_y;
        std::cout << "Enter the new x: ";
        std::cin >> input_x;
        std::cout << "Enter the new y: ";
        std::cin >> input_y;

        // Set the new goal coordinates
        ros::param::set("/des_pos_x", input_x);
        ros::param::set("/des_pos_y", input_y);
        
        target.target_pose.pose.position.x = input_x;
        target.target_pose.pose.position.y = input_y;
        
        // Send the new goal
        client.sendGoal(target);

   	while(client.getState() == actionlib::SimpleClientGoalState::PENDING){    
        std::string canc;
        std::cout << "Do you want to cancel the goal? (y/n): ";
        std::cin >> canc;
        if (canc == "y") {    
        	ROS_WARN("Current goal has been cancelled");
                client.cancelGoal();
                break;} 
        else if (canc == "n") {
           bool finished_before_timeout = client.waitForResult(ros::Duration(120.0));
           
           if (finished_before_timeout) {
            	ROS_INFO("We reach the goal in time!");}
           else {
            	ROS_WARN("We didn't reach the goal before 120s.");
                break;}}
       else {
           ROS_WARN("Invalid command");
           continue;}}
    ros::spinOnce();     
    }}



int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;
    
    sub = nh.subscribe("/odom", 1, CallbackPositionVelocity);
    pub = nh.advertise<assignment_2_2023::RobotPV>("/position_velocity", 1);
    
    std::thread clientThread(ClientGoal);

    // Start the ROS event loop
    ros::spin();

    // Wait for the ClientGoal thread to finish before exiting
    clientThread.join();
    
    return 0;
    
}
