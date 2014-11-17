# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
PROJECT(yarpWholeBodyInterfaceExample)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(YCM REQUIRED)
find_package(iDynTree REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

#use yarp and ycm find package scripts
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YCM_MODULE_PATH})

FILE(GLOB folder_source ./*.cpp)
FILE(GLOB folder_header ./*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${wholeBodyInterface_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS})

ADD_EXECUTABLE(yarpWholeBodyInterfaceExample ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(yarpWholeBodyInterfaceExample ${wholeBodyInterface_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES})

if(WIN32)
	INSTALL_TARGETS(/bin/Release yarpWholeBodyInterfaceExample)
else(WIN32)
	INSTALL_TARGETS(/bin yarpWholeBodyInterfaceExample)
endif(WIN32)
