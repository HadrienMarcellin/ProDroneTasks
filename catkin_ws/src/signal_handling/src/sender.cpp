#include "signal_handling/sender.hpp"


SignalSender::SignalSender()
{
    ROS_INFO("Initialize SignalSender object.");
    killProgram = false;
    pidToKill = 0;
}

void SignalSender::savePid(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Received PID from msg.");
    pidToKill = msg->data;
}

int SignalSender::getPID()
{
    this->killProgram = true;
    return this->pidToKill;  
}

bool SignalSender::getKillProgram()
{   
    return this->killProgram;
}


void SignalSender::sendSignals()
{
    int i = 0;
    ros::Rate myRate(1);
    
    while(ros::ok() && i < 5)
    {
        ROS_INFO("Sending signal (%d) to PID: %d", this->signals[i], this->pidToKill);
        kill(this->pidToKill, signals[i]);
        
        if(kill(this->pidToKill, signals[i]) == -1)
        {
            ROS_INFO("Program was killed using signal (%d), shutting down... ", signals[i]);
            exit(0);
        }
        
        ros::spinOnce();
        myRate.sleep();
        
        i++;
    }
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "signalSender");
    ros::NodeHandle n;
    
    SignalSender mySender;
    ros::Subscriber sub = n.subscribe("handlerPID", 1, &SignalSender::savePid, &mySender);
    
    ros::Rate loopRate(1);
    while(ros::ok() && (mySender.getPID() == 0))
    {
        ros::spinOnce();
        loopRate.sleep();
    }
    
    sub.shutdown();
    
    if(mySender.getKillProgram())
        mySender.sendSignals();
    



}


