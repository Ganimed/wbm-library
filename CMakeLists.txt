# Copyright (C) 2016 CoDyCo
# Authors: Gabriele Nava (gabriele.nava@iit.it); 2016
#          Martin Neururer (martin.neururer@gmail.com); 2019
# Date: October 2019
#
# This CMake file is part of the Whole-Body Model Library for Matlab (WBML).
#
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or later.
cmake_minimum_required(VERSION 2.8.12.2)

PROJECT(yarpWBM)       # Whole-Body Model for YARP-based robots
SET(LIBRARY_NAME WBML) # WBM-Library

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Look for dependencies
find_package(Matlab REQUIRED
                    MX_LIBRARY
                    MAIN_PROGRAM)

if(NOT Matlab_FOUND)
    message(FATAL_ERROR "Matlab not found")
endif()

find_package(ICUB           REQUIRED)
find_package(YARP           REQUIRED)
find_package(YCM            REQUIRED)
find_package(TinyXML        REQUIRED)
find_package(iDynTree 0.4.0 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

include(YCMDefaultDirs)
include(YarpInstallationHelpers)
include(AddInstallRPATHSupport)

#yarp_configure_external_installation(codyco)

## Settings for rpath
if(NOT MSVC)
    # add the option to enable RPATH
    option(WBML_ENABLE_RPATH "Enable RPATH installation" TRUE)
    mark_as_advanced(WBML_ENABLE_RPATH)
endif(NOT MSVC)

add_install_rpath_support(BIN_DIRS ${CMAKE_INSTALL_PREFIX}/bin
                          LIB_DIRS ${CMAKE_INSTALL_PREFIX}/mex ${CMAKE_INSTALL_PREFIX}/lib
                          DEPENDS WBML_ENABLE_RPATH
                          USE_LINK_PATH)

# Enable testing and generate test target
include(CTest)
enable_testing()

# Subdirectories
add_subdirectory(libraries/wholebodyinterface)
add_subdirectory(libraries/yarp-wholebodyinterface)
add_subdirectory(mex-wholebodymodel)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# STARTUP mexWholeBodyModel
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/.startup_mexWholeBodyModel.m.in ${CMAKE_BINARY_DIR}/startup_mexWholeBodyModel.m)
install(FILES ${CMAKE_BINARY_DIR}/startup_mexWholeBodyModel.m DESTINATION share/wbm-library)
