# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

cmake_minimum_required(VERSION 2.8.11)
project(yarpWholeBodyInterface CXX)

set(LIBRARY_NAME yarpwholebodyinterface)
set(VARS_PREFIX ${PROJECT_NAME})
set(INCLUDE_DIRS yarpWholeBodyInterface)

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 2)
set(${VARS_PREFIX}_PATCH_VERSION 0)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

option(CODYCO_USES_URDFDOM "Compile yarpWholeBodyInterface with URDF support" TRUE)
option(ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL "Compile yarpwholebodyinterface to use JTC external torque control" FALSE)
option(COMPILE_AS_SHARED_LIBRARY "Compile ${PROJECT_NAME} as a shared library" TRUE)
option(ENABLE_TESTS "Enable unit testing using Gazebo simulator" TRUE)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)
find_package(wholeBodyInterface 0.2.0 REQUIRED)

SET(folder_source src/yarpWbiUtil.cpp
                  src/yarpWholeBodyInterface.cpp
                  src/yarpWholeBodyModel.cpp
                  src/yarpWholeBodyStates.cpp
                  src/yarpWholeBodyStatesLocal.cpp
                  src/yarpWholeBodyActuators.cpp
                  src/yarpWholeBodySensors.cpp)
SET(folder_header include/yarpWholeBodyInterface/yarpWholeBodyInterface.h
                  include/yarpWholeBodyInterface/yarpWholeBodyModel.h
                  include/yarpWholeBodyInterface/yarpWholeBodyStates.h
                  include/yarpWholeBodyInterface/yarpWholeBodyStatesLocal.h
                  include/yarpWholeBodyInterface/yarpWholeBodyActuators.h
                  include/yarpWholeBodyInterface/yarpWholeBodySensors.h
                  include/yarpWholeBodyInterface/yarpWbiUtil.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

add_definitions(-Wall)

OPTION(ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL "Set if iCub WBI implementation uses external torque control. False to use YARP Torque Control." FALSE)

if(CODYCO_USES_URDFDOM)
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
                                       ${wholeBodyInterface_LIBRARIES})

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

if(ENABLE_TESTS)
#enable testing
include(CTest)
enable_testing()
add_subdirectory(tests)
endif()
