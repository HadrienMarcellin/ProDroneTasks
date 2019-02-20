#include "ros/ros.h"
#include "task4/Delay.h"
#include <string>



bool wait_for_delay(task4::Delay::Request &req, 
                    task4::Delay::Response &res)
{
    double now = ros::Time::now().toSec()*1e6;
    
    if(req.Delay_s < 5 & req.Delay_s > 0)
    {
        ros::Duration(req.Delay_s).sleep();
        res.Time = std::to_string(now); 
        ROS_INFO("time delay %d[s] is passed. Time is now %.0f[us].", (int)req.Delay_s, now);
    }
    else
    {
        res.Time = std::to_string(now);
        ROS_INFO("Time delay %d out of range, Time is : %.0f[us].", (int)req.Delay_s, now);
    }
    
    return 1;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "thread_server");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::ServiceServer service = n.advertiseService("unix_time_now", wait_for_delay);
    ROS_INFO("Ready to wait for delay.");
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    /**
    while(ros::ok())
    {
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    **/

}
