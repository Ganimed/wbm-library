# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
PROJECT(yarpWholeBodyRootWorldTest)

#find_package(YARP REQUIRED)
#find_package(ICUB REQUIRED)
#find_package(iDynTree REQUIRED)

find_package(YARP 2.3.63.2 REQUIRED)
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)
find_package(wholeBodyInterface 0.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(wholeBodyInterface REQUIRED)
#find_package(yarpWholeBodyInterface REQUIRED)

set (SOURCE ./main.cpp)

source_group("Source Files" FILES ${SOURCE})

include_directories(${wholeBodyInterface_INCLUDE_DIRS}
                    ./../include/yarpWholeBodyInterface
                    ${iDynTree_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})
                    
add_executable(${PROJECT_NAME} ${SOURCE})

target_link_libraries(${PROJECT_NAME} ${wholeBodyInterface_LIBRARIES}
                                     yarpwholebodyinterface)

#if(WIN32)
	#INSTALL_TARGETS(/bin/Release ${PROJECT_NAME})
#else(WIN32)
	#INSTALL_TARGETS(/bin ${PROJECT_NAME})
#endif(WIN32)

add_test(NAME "test_$PROJECT_NAME" COMMAND "$PROJECT_NAME")