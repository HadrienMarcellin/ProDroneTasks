#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <csignal>
//#include <rand>
#include <time.h>
#include <unistd.h>



class SignalHandle
{
    private :        
        int killSignal;
    protected:
    
    public:
        int signals[5] = {SIGINT, SIGTERM, SIGABRT, SIGSEGV, SIGFPE};
        SignalHandle();
        int getKillSignal() { return killSignal; }
        void defineKillSignal(); 
        void signalHandler(int signal_num); 

};

SignalHandle myHandler;
