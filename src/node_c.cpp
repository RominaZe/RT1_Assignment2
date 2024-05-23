/**
 * \file node_c.cpp
 * \brief Metrics Calculation 
 * \author Romina Contreras
 * \version 0.1
 * \date 29/02/2024
 *
 * \param /des_pos_x Define the goal x position.
 * \param /des_pos_y Define the goal y position.
 * \param /window_x Define the windows of the enviroment used in the linear speed.
 * \param /window_y Define the window of the enviroment used to make the average angular speed.
 * \param distance Define the distance.
 * \param avg_speed_z Define the average linear speed.
 * \param avg_speed_x Define the average angular speed.
 *
 * Subscriber: <BR>
 *  °/reaching_goal/goal
 *
 * Publisher: <BR>
 *  °None
 *
 * Service: <BR>
 * °/distance_velocity
 *
 * Description:
 *
 * This node operates as a service node that subscribes to the topic /position_velocity. 
 * Compute the distance between the robot between the robot and the target, and the average speed.
 * 
 */
 
#include "ros/ros.h"
#include "assignment_2_2023/RobotPV.h"
#include "assignment_2_2023/Average.h"
#include "assignment_2_2023/Distance.h"
#include "nav_msgs/Odometry.h"
#include "numeric"


double distance = 0, avg_speed_x = 0, avg_speed_z;
std::vector<double> window_x, window_z;

/**
 * \brief Callback function to compute average speed and distance to the goal position.
 *
 * This function is a callback for the /position_velocity topic messages. It computes the distance between the current robot position and the goal position specified by the parameters `/des_pos_x` and `/des_pos_y`.
 * It also computes the average linear and angular speed of the robot over a specified window size.
 * 
 * \param msg A constant pointer to the received message containing the robot's position and velocity.
 * 
 * This function computes the distance between the current robot position and the goal position using the Euclidean distance formula.
 * It retrieves the goal position coordinates from the ROS parameters `/des_pos_x` and `/des_pos_y`.
 * 
 * The average speed calculation requires a window size, which is obtained from the parameter `/window_size`. The function maintains sliding windows for linear and angular velocities and updates them with new values from the received message.
 * Once the window size is reached, it calculates the average speed by summing up the velocities in the window and dividing by the window size.
 * 
 * \return void
 * \note This function modifies the values of global variables to store the computed distance and average speeds.
 *
 */
void AverageCallback(const assignment_2_2023::RobotPV::ConstPtr& msg)
{
    double position_x, position_y;   
    position_x = msg->x;
    position_y = msg->y;
    
    double x_goal,y_goal;
    ros::param::get("/des_pos_x",x_goal);
    ros::param::get("/des_pos_y",y_goal);
    
    distance = std::sqrt(std::pow(position_x-x_goal,2) + std::pow(position_y-y_goal,2));
    
    
    //Compute the average speed
    int window_size;
    ros::param::get("/window_size", window_size);
    
    if (window_x.size() > static_cast<size_t>(window_size)){
        window_x.erase(window_x.begin()); }

    if (window_z.size() > static_cast<size_t>(window_size)){
        window_z.erase(window_z.begin());}

    window_x.push_back(msg->linear_x);
    window_z.push_back(msg->angular_z); 

    avg_speed_x = std::accumulate(window_x.begin(), window_x.end(), 0.0) / window_size;
    avg_speed_z = std::accumulate(window_z.begin(), window_z.end(), 0.0) / window_size;
}

/**
 * \brief Callback function to handle distance and speed service requests.
 * 
 * \param req The service request containing the requested information (not used in this function).
 * \param res The service response containing the computed average linear and angular speeds, along with the distance to the goal position.
 *
 * This function is a callback for the service requests to compute and return the average linear and angular speeds, along with the distance to the goal position.
 * 
 * This function populates the response message with the current values of the average linear and angular speeds and the distance to the goal position.
 * 
 * The service request is not used in this function as the response is computed based on the global variables.
 * 
 * \return true 
 *
 * \note This function accesses and uses global variables to populate the response message.
 */
bool DistSpeedCallback(assignment_2_2023::Average::Request& req,
                 assignment_2_2023::Average::Response& res)
{
    res.linear_avg = avg_speed_x;
    res.angular_avg = avg_speed_z;
    res.distance = distance;
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_c");
    ros::NodeHandle nh;
    
    //ros::Subscriber sub2 = nh.subscribe("/odom",1, DistAverageCallback);

    ros::ServiceServer service = nh.advertiseService("/distance_averageVelocity", DistSpeedCallback);
    ros::Subscriber sub = nh.subscribe("/position_velocity", 1, AverageCallback);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

