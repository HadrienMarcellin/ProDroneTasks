#include "logger.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <utility>


Logger::Logger(const char *file,std::chrono::high_resolution_clock::time_point time)
{
    event_number = 0;
    start_time = time;
    fileName = file;
}

void Logger::open_file()
{
    std::cout << "Opening file : " << this->fileName << std::endl;   
    log_file.open(fileName);
    log_file << "Event Number; Value; Time;\n";    
}

void Logger::close_file()
{
    std::cout << "Closing file : " << this->fileName << std::endl;  
    if(log_file.is_open())
    {
        log_file << "Closing this file.\n";  
        log_file.close();
    }
    else {std::cout << "Cannot close file since it is not open." << std::endl; }    
}

bool Logger::does_file_exist(const char *fileName)
{
    std::ifstream infile("example.txt");
    return infile.good();
}

