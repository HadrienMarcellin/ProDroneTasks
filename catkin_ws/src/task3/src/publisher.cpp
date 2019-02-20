#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>


class PublishCoordinates
{
    public: 
        PublishCoordinates(ros::Publisher FramePub);
        void Receive(const std_msgs::StringConstPtr& msg);
        void Publish();
        std::vector<double> Coordinates() {return coordinates; };
        int Count() {return count; };
    
    private:
        std::vector<double> coordinates;
        geometry_msgs::TransformStamped my_transf;
        ros::Publisher frame_pub;
        void VectToMsg();
        int count;
};

PublishCoordinates::PublishCoordinates(ros::Publisher FramePub)
{
    frame_pub = FramePub;
    coordinates = {0, 0, 0, 0, 0, 0, 0};
    count = 0;
}


void PublishCoordinates::VectToMsg()
{
    
    my_transf.transform.translation.x = coordinates.at(0);
    my_transf.transform.translation.y = coordinates.at(1);
    my_transf.transform.translation.z = coordinates.at(2);
    my_transf.transform.rotation.x = coordinates.at(3);
    my_transf.transform.rotation.y = coordinates.at(4);
    my_transf.transform.rotation.z = coordinates.at(5);
    my_transf.transform.rotation.w = coordinates.at(6);

}

void PublishCoordinates::Publish()
{
    ros::Time current_time = ros::Time::now();
    my_transf.header.stamp = current_time;
    my_transf.header.frame_id = "world";
    my_transf.child_frame_id = "base_tf_enu";
    
    frame_pub.publish(my_transf);
    ++count;
}

void PublishCoordinates::Receive(const std_msgs::StringConstPtr& msg)
{
    std::string str = msg->data;
    std::istringstream temp{str};
    std::vector<double> vect_temp{std::istream_iterator<double>{temp},         std::istream_iterator<double>{}};  
    coordinates = vect_temp;
    VectToMsg();
    count = 0;  

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "tf_publisher_base");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
   
  
  
  

    ros::Publisher frame_pub = n.advertise<geometry_msgs::TransformStamped>("base_tf_enu", 1000); 
    PublishCoordinates my_publish_coordinates(frame_pub);
    ros::Subscriber keyboard_sub = n.subscribe("keyboard", 1, &PublishCoordinates::Receive, &my_publish_coordinates);
    ros::Rate loop_rate(1);


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  
  while (ros::ok())
  {
    
    my_publish_coordinates.Publish();
    

    std_msgs::String msg;

    std::stringstream ss;
    ss << "Publishing coordinates with new values " << my_publish_coordinates.Count();
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
