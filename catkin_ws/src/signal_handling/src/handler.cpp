#include <signal_handling/handler.hpp>

SignalHandle::SignalHandle()
{
    this->defineKillSignal();
}

void SignalHandle::defineKillSignal()
{
    srand(time(NULL));
    killSignal = signals[rand() % 4];
    ROS_INFO("Kill signal is %d.", killSignal);
}

void SignalHandle::signalHandler(int signal_num)
{
    ROS_INFO("I received interrupt signal n (%d)", signal_num);
    if(signal_num == killSignal)
        exit(15);        
}

void signalhandle(int s) 
{
    myHandler.signalHandler(s); 
}

void assignSignalCallback()
{
    ROS_INFO("Assigning signals to callback.");
    
    for(int i = 0; i < 5; i++)
    {
        signal(myHandler.signals[i], signalhandle);
    }
   
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "signalHandler");
    ros::NodeHandle n;
    
    ros::Publisher handlerPid = n.advertise<std_msgs::Int32>("handlerPID", 1000);
    
    std_msgs::Int32 msg;
    msg.data = getpid();

    ros::Rate loopRate(1);
    
    assignSignalCallback();
    
    while(ros::ok())
    {
        handlerPid.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }



}


