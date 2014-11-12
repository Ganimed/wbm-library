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
option(ENABLE_TESTS "Enable unit testing using Gazebo simulator" FALSE)

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

#### Settings for rpath
if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    if(NOT MSVC)
        #add the option to disable RPATH
        set(YARPWBI_DISABLE_RPATH FALSE CACHE BOOL "Disable RPATH installation")
        mark_as_advanced(YARPWBI_DISABLE_RPATH)
    endif(NOT MSVC)

    #Configure RPATH
    set(CMAKE_MACOSX_RPATH 1) #enable RPATH on OSX. This also suppress warnings on CMake >= 3.0
    # when building, don't use the install RPATH already
    # (but later on when installing)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    #build directory by default is built with RPATH
    set(CMAKE_SKIP_BUILD_RPATH  FALSE)

    #This is relative RPATH for libraries built in the same project
    #I assume that the directory is 
    # - install_dir/something for binaries
    # - install_dir/lib for libraries
    file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_PREFIX}/bin" "${CMAKE_INSTALL_PREFIX}/lib")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
    else()
        set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
    endif()

    # the RPATH to be used when installing, but only if it's not a system directory (copied form CMake File. To be tested)
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
       set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    endif("${isSystemDir}" STREQUAL "-1")

    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) #very important!

    if(YARPWBI_DISABLE_RPATH)
        #what to do? disable RPATH altogether or just revert to the default CMake configuration?
        #I revert to default
        unset(CMAKE_INSTALL_RPATH) #remove install rpath
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
    endif(YARPWBI_DISABLE_RPATH)
endif()
##### end RPATH

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

add_subdirectory(app)