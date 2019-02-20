#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "image_transport_tutorial/TakeSnapshot.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#include <string>
#include <iostream>



class ImageReceived
{
    private:
        int i = 0;
        bool save = false;
        std::string filename = "/home/hadrien/default.jpg";

    protected:
        
    public:
 
    ImageReceived()
    {

    }
 
    bool takeSnapshot(image_transport_tutorial::TakeSnapshot::Request& req,
                       image_transport_tutorial::TakeSnapshot::Response& res)
    {
        
        std::ostringstream oss;
        oss << "/home/hadrien/" << i << ".jpg";
        i++;
        filename = oss.str();

        if(req.take_snap == true)
        {
            save = true;
        }
        res.image_location = filename;
        return 1;

    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
            cv::waitKey(10);
            if(save)
            {
                ROS_INFO("Saving image to location %s ...", filename.c_str());
                cv::imwrite(filename, cv_bridge::toCvShare(msg, "bgr8")->image);
                save = false;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
        
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ImageReceived myImage;  
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, &ImageReceived::imageCallback, &myImage);
  
  // Add service to take snapshot on command
  ros::ServiceServer snapshot_service = nh.advertiseService("take_snapshot", &ImageReceived::takeSnapshot, &myImage);

  ros::spin();
  cv::destroyWindow("view");
}
