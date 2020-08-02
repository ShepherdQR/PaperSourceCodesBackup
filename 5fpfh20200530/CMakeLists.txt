#************
#  * @Author: Shepherd Qirong
#  * @Date: 2020-02-22 01:09:25
#  * @Github: https://github.com/ShepherdQR
#  * @LastEditors: Shepherd Qirong
#  * @LastEditTime: 2020-05-30 00:43:48
#  * @Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
#************
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(main)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



file(GLOB cpp_executable *.cpp)

#file(GLOB cpp_class ../ClassCollections/*.cpp)
#include_directories(../ClassCollections) #指定头文件目录

message("Building the main cpp: " ${cpp_executable})
add_executable(main ${cpp_executable} )#${cpp_class}


target_link_libraries (main ${PCL_LIBRARIES})
