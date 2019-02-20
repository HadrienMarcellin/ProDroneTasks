#include "ros/ros.h"
#include "task4/Delay.h"
#include <boost/thread/thread.hpp>

void print_current_time(int* publish_rate)
{

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Rate loop_rate(*publish_rate);

    while (ros::ok())
    {
        ros::Time t = ros::Time::now();
        double now_ns = t.toSec() * 1e9;
        ROS_INFO("Current Time: *%.0f*", now_ns);
        loop_rate.sleep();
    }
}


void print_difference_time(ros::ServiceClient* client, task4::Delay* srv)
{
    task4::Delay tempSrv = *srv;
    ros::Time beginning = ros::Time::now();    
    if((*client).call(tempSrv))
        {
            ros::Duration diff = ros::Time::now() - beginning;
            double diff_ms = diff.toSec() * 1e3;
            ROS_INFO("Time Difference : %f[ms]", diff_ms);
        }
        else
        {
            ROS_INFO("Failed to call service.");
            //return 1;
        }

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "threads_client");
    if (argc != 1)
    {
        ROS_INFO("usage: threads_client");
        return 1;
    }
    int print_rate = 2;
    boost::thread thread_print(print_current_time, &print_rate);
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<task4::Delay>("unix_time_now");
    
    task4::Delay srv;

    ros::Rate loop_rate(1);    
    //ros::AsyncSpinner spinner(10); //Choose how many threads
    //spinner.start();

    
    while(ros::ok())
    {
        
        
        int delay_s = rand() % 5;
        srv.request.Delay_s = delay_s;
        //ROS_INFO("Client asked for %d[s]", delay_s);
        
        boost::thread thread_difference(print_difference_time, &client, &srv);
        thread_difference.detach();
        //client.call(srv);
        /**
        if(client.call(srv))
        {
            ROS_INFO("Server waited for %d[s]", (int)srv.request.Delay_s);
        }
        else
        {
            ROS_INFO("Failed to call service.");
            return 1;
        }
**/
        ros::spinOnce();
        loop_rate.sleep();
    
    
    }
    
    //ros::waitForShutdown();
    thread_print.join();
    
    
    return 0;
    


}


