#include "ros/ros.h"
#include "assignment_2_2023/RobotPV.h"
#include "assignment_2_2023/Average.h"
#include "nav_msgs/Odometry.h"
#include "numeric"


double distance = 0, avg_speed_x = 0, avg_speed_z;
std::vector<double> window_x, window_z;

void AverageCallback(const assignment_2_2023::RobotPV::ConstPtr& msg)
{
    double position_x, position_y;   
    position_x = msg->x;
    position_y = msg->y;
    
    double x_goal,y_goal;
    ros::param::get("/des_pos_x",x_goal);
    ros::param::get("/des_pos_y",y_goal);
    
    //Compute distance
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
