# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

cmake_minimum_required(VERSION 2.8.11)
project(yarpWholeBodyInterface CXX)

set(LIBRARY_NAME yarpwholebodyinterface)
set(VARS_PREFIX ${PROJECT_NAME})
set(INCLUDE_DIRS wbiIcub)

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 0)
set(${VARS_PREFIX}_PATCH_VERSION 1)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

option(CODYCO_USES_URDFDOM "Compile yarpWholeBodyInterface with URDF support" TRUE)
option(ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL "Compile yarpwholebodyinterface to use JTC external torque control" FALSE)
option(COMPILE_AS_SHARED_LIBRARY "Compile ${PROJECT_NAME} as a shared library" TRUE)

add_definitions(-DIDYNTREE_USES_ICUB)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(paramHelp REQUIRED)

#take in account the new pwm interface
if(YARP_VERSION VERSION_LESS 2.3.62.3)
    message(STATUS "found YARP (${YARP_VERSION}) has the old motor interface")
else()
    message(STATUS "found YARP (${YARP_VERSION}) has the new interaction mode motor interface interface")
    add_definitions(-DYARP_INTERACTION_MODE_MOTOR_INTERFACE)
endif()


set(folder_source src/wbiIcubUtil.cpp
                  src/icubWholeBodyInterface.cpp
                  src/icubWholeBodyModel.cpp
                  src/icubWholeBodyStates.cpp
                  src/icubWholeBodyStatesLocal.cpp
                  src/icubWholeBodyActuators.cpp
                  src/icubWholeBodySensors.cpp)
set(folder_header include/wbiIcub/wholeBodyInterfaceIcub.h
                  include/wbiIcub/icubWholeBodyModel.h
                  include/wbiIcub/icubWholeBodyStates.h
                  include/wbiIcub/icubWholeBodyStatesLocal.h
                  include/wbiIcub/icubWholeBodyActuators.h
                  include/wbiIcub/icubWholeBodySensors.h
                  include/wbiIcub/wbiIcubUtil.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

option(ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL "Set if iCub WBI implementation uses external torque control. False to use YARP Torque Control." FALSE)
option(YARPWBI_DISABLE_RPATH "Disable RPATH for the library" FALSE)

if(CODYCO_USES_URDFDOM)
    set(folder_source ${folder_source}
                      src/yarpWholeBodyInterface.cpp)
    set(folder_header ${folder_header}
                      include/wbiIcub/yarpWholeBodyInterface.h)
    add_definitions(-DCODYCO_USES_URDFDOM)
endif()

if (ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL)
    add_definitions(-DWBI_ICUB_COMPILE_PARAM_HELP)
endif()

if(MSVC)
    message(STATUS "Building ${PROJECT_NAME} as a shared library is not currently supported on Windows")
    set(COMPILE_AS_SHARED_LIBRARY FALSE)
endif()

if(COMPILE_AS_SHARED_LIBRARY)
    set(LIB_TYPE SHARED)
else()
    set(LIB_TYPE STATIC)
endif()

#Configure RPATH
set(CMAKE_MACOSX_RPATH 1) #enable RPATH on OSX. This also suppress warnings on CMake >= 3.0
# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#This is relative RPATH for libraries built in the same project
#I assume that the directory is 
# - install_dir/something for binaries
# - install_dir/lib for libraries
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_INSTALL_RPATH "@loader_path/../lib")
else(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) #very important!


if(YARPWBI_DISABLE_RPATH)
    set(CMAKE_SKIP_BUILD_RPATH TRUE)
else(YARPWBI_DISABLE_RPATH)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
endif(YARPWBI_DISABLE_RPATH)


add_library(${LIBRARY_NAME}  ${LIB_TYPE} ${folder_source} ${folder_header})

set_target_properties(${LIBRARY_NAME}  PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                       SOVERSION ${${VARS_PREFIX}_VERSION}
                                       PUBLIC_HEADER "${folder_header}")

target_include_directories(${LIBRARY_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                           ${YARP_INCLUDE_DIRS}
                                           ${iDynTree_INCLUDE_DIRS}
                                           ${wholeBodyInterface_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_NAME}  ${YARP_LIBRARIES}
                                       ${iDynTree_LIBRARIES}
                                       ${wholeBodyInterface_LIBRARIES}
                                       ${paramHelp_LIBRARIES})

install(TARGETS ${LIBRARY_NAME}
        EXPORT  ${PROJECT_NAME}
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/${INCLUDE_DIRS}" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${LIBRARY_NAME})

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VARS_PREFIX ${VARS_PREFIX}
                                            VERSION ${${VARS_PREFIX}_VERSION}
                                            COMPATIBILITY SameMajorVersion
                                            TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)
