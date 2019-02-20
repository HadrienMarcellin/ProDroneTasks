#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "listen_to_keyboard");
    ros::NodeHandle n;
    
    ros::Publisher keyboard_pub = n.advertise<std_msgs::String>("keyboard", 1);
    
    
    ros::Rate loop_rate(1);
    
    while(ros::ok())
    {
        std::string str;
        std::cout << "Coordinates ?" << std::endl;
        std::getline (std::cin, str);
    
        std_msgs::String msg;
        msg.data = str;
        keyboard_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    return 0;


}
