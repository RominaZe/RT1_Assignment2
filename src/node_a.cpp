/**
 * \file node_a.cpp
 * \brief Comunication and Navigation Node A
 * \author Romina Contreras
 * \version 0.1
 * \date 27/02/2024
 *
 * \param [in] des_pos_x Define the desired x axis target.
 * \param [in] des_pos_y Define the desired y axis target.
 *
 * Subscribes to: <BR>
 *  °/odom
 *
 * Publishes to: <BR>
 *  °/position_velocity
 *
 * Description:
 *
 * This node implement an interface and an action client where the user is allowed to set a new target or to cancel it. If the user choose a new target then have to put the new target. For this aim are use the parameter des_pos_x and de_pos_y
 * Also in this node publish the robot position and velocity in a topic call /position_velocity.
 * 
 */


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


ros::Publisher pub; ///< Publisher.
ros::Subscriber sub; ///< Subscriber.


/**
 * CallbackPositionVelocity"("const nav_msgs::Odometry::ConstPtr& msg")"
 * \brief Publishes the position and velocity values on the topic /position_velocity.
 *
 * \param msg Define the velocity and position of the robot in the actual state.
 * \param [out] position_velocity Actual position, linear velocity and angular velocity publish in /position_velocity.
 * \param msg The message containing the velocity and position of the robot in the current state.
 *
 * This function creates a RobotPV message filled with values taken from the /odom topic, which reppresent the robot's position and velocity. 
 * 
 * Then this message is publish in the topic /position_velocity using the parameter position_velocity
 */
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

/**
 * \brief Manages goal setting and cancellation.
 *
 * \param [in] des_pos_x Define the actual desired position x.
 * \param [in] des_pos_y Define the actual desired position y.
 * \param [in] input_x The desired x postion choose by the user.
 * \param [in] input_y The dedired y position choos by the user.
 * \param [in] canc Define the dedsition take by the user cancel the goal or put another goal
 *
 * This function set-up an action client to send goals for reaching a target position. It continuously prompts the user to set new coordinates for the target position. 
 * The user can also cancel the current goal if needed. 
 * 
 * The function retrieves the current goal position from the parameters `/des_pos_x` and `/des_pos_y`, creates a new PlanningGoal message with these coordinates, and sends it to the action server.
 * If the user chooses to set a new goal, they are prompted to input new x and y coordinates. These coordinates are then updated in the parameters and used to set the new goal.
 * 
 * If the user chooses to cancel the current goal, the function cancels the action goal and exits the loop. 
 * 
 * The function also waits for the action server to return the goal result and prints appropriate messages based on the outcome.
 * 
 * Note: This function is executed in a separate thread to allow continuous interaction while the main event loop is active.
 */
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


/**
 * \brief Main function where we coordinate a
 * This is the main function where is initialized the node_a. THe node is subscribe at the topic /odom and have the callback that publish on the topic position_velocity. Another functionality of the main function is advertizing the topic /position_velocity where is publish the actual position and the actual velocity of the robot.
 *
 */
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
