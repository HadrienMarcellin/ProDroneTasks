#include <iostream>
#include "logger.hpp"
#include <chrono>

using namespace std;


// On utilise typename et non pas classe pour eviter les confusions

template<typename Type> Type calculerSomme(Type operande1, Type operande2)

{

    Type resultat = operande1 + operande2;

    return resultat;

}


int main(int argc, char *argv[])
{
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    
    const char *log_file;
    
    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <log filename>" << std::endl;
        std::cout << "Default file is log.txt" << std::endl;
        log_file = "log.txt";
        
    }
    else
    {
        log_file = argv[1];
    }
    
    Logger mylogger(log_file, start_time);
    mylogger.open_file();
    
    //Do things
    
    bool val1 = true;
    bool val2 = false;
    double val3 = 1.0;
    double val4 = 4.0;
    
    double res1 = mylogger.write_log_to_file(val1);
    double res2 = mylogger.write_log_to_file(val2);
    double res3 = mylogger.write_log_to_file(val3);
    double res4 = mylogger.write_log_to_file(val4);
    
    mylogger.close_file();
    
    return 1;
    
}
