# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
PROJECT(yarpWholeBodyRootWorldTest)


set (SOURCE ./main.cpp)

source_group("Source Files" FILES ${SOURCE})

include_directories(${wholeBodyInterface_INCLUDE_DIRS}
                    ./../include/yarpWholeBodyInterface
                    ${iDynTree_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${SOURCE})

target_link_libraries(${PROJECT_NAME} ${wholeBodyInterface_LIBRARIES}
                                     yarpwholebodyinterface)
