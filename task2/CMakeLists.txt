cmake_minimum_required(VERSION 3.5.1)
project(task2)

#Set g++ to c++11
#macro(use_cxx11)
#  if (CMAKE_VERSION VERSION_LESS "3.1")
#    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
#    endif ()
#  else ()
#    set (CMAKE_CXX_STANDARD 11)
#  endif ()
#endmacro(use_cxx11)

# Locate GTest
find_package(GTest REQUIRED)

#Bring the headers, such as Student.h into the project
include_directories(include/logger)
include_directories(${GTEST_INCLUDE_DIRS})

#Can manally add the sources using a set of commands as follow:
#set(SOURCES src/mainapp.cpp src/Students.cpp)

#However, the file(GLOB... )  allows for wildcard addition:
file(GLOB SOURCES "src/*.cpp")

# Link runTests with what we want to test and the GTest and pthread library
add_executable(runTests tests/gtest.cpp)
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)
add_executable(loggertask ${SOURCES})

set_property(TARGET loggertask PROPERTY CXX_STANDARD 11)
set_property(TARGET runTests PROPERTY CXX_STANDARD 11)


 

