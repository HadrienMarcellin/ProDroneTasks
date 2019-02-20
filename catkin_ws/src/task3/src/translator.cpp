#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"

class Translator
{
    public:
        Translator(ros::Publisher TfPubNed, ros::Publisher TfPubNwu);
        void Receive(const geometry_msgs::TransformStampedConstPtr& transformed_stamped);
    private:
        ros::Publisher tf_pub_ned;
        ros::Publisher tf_pub_nwu;
        geometry_msgs::TransformStamped tf_enu;
        geometry_msgs::TransformStamped tf_ned;
        geometry_msgs::TransformStamped tf_nwu;
        void EnuToNed();
        void EnuToNwu();
};


Translator::Translator(ros::Publisher TfPubNed, ros::Publisher TfPubNwu)
{
    tf_pub_ned = TfPubNed;
    tf_pub_nwu = TfPubNwu;
}

void Translator::EnuToNed()
{
    tf_ned = tf_enu;
    tf_ned.child_frame_id = "tf_base_ned";    
}

void Translator::EnuToNwu()
{
    tf_nwu = tf_enu;
    tf_nwu.child_frame_id = "tf_base_nwu";
}


void Translator::Receive(const geometry_msgs::TransformStampedConstPtr& transformed_stamped)
{
    tf_enu = *transformed_stamped;
    EnuToNed();
    EnuToNwu();
    tf_pub_ned.publish(tf_ned);
    tf_pub_nwu.publish(tf_nwu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_receiver_translator");
    ros::NodeHandle n;
    
    ros::Publisher tf_pub_ned = n.advertise<geometry_msgs::TransformStamped>("base_tf_ned", 1);
    ros::Publisher tf_pub_nwu = n.advertise<geometry_msgs::TransformStamped>("base_tf_nwu", 1);
    
    Translator my_translator(tf_pub_ned, tf_pub_nwu);
    
    ros::Subscriber tf_sub = n.subscribe("base_tf_enu", 1, &Translator::Receive, &my_translator);
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

} 
