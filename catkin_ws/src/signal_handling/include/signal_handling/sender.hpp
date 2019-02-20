#include "ros/ros.h"
#include <iostream>
#include <csignal>
#include "std_msgs/Int32.h"
#include <sys/types.h>
#include <signal.h>



class SignalSender
{
    private :
        int signals[5] = {SIGINT, SIGABRT, SIGSEGV, SIGFPE, SIGTERM};
        bool killProgram;
        int pidToKill;
    protected:
    
    public:
    
        SignalSender();        
        void savePid(const std_msgs::Int32::ConstPtr& msg);
        int getPID();
        void sendSignals();
        bool getKillProgram();
};
