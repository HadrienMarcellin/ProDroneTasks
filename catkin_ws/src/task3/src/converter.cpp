#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "tf_receiver_translator");

  ros::NodeHandle n;

  ros::Subscriber frame_pub = n.subscribe<geometry_msgs::TransformStamped>("tf_pub", 1000); 
  ros::Rate loop_rate(10);
  
  ros::Time current_time;

  while (ros::ok())
  {
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }


  return 0;
}
