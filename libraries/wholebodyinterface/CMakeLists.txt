# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Andrea Del Prete; 2013
#          Francesco Romano; 2014
#          Silvio Traversaro; 2014
#          Martin Neururer (martin.neururer@gmail.com); 2019
# Date: October 2019
#
# This CMake file is part of the Whole-Body Model Library for Matlab (WBML).
#
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or later.
cmake_minimum_required(VERSION 2.8.12.2)

SET(PROJECT_NAME wholeBodyInterface)
SET(LIBRARY_NAME whole-body-interface)
SET(HEADERS_DIR wbi)

PROJECT(${PROJECT_NAME})

set (${PROJECT_NAME}_VERSION_MAJOR 0)
set (${PROJECT_NAME}_VERSION_MINOR 2)
set (${PROJECT_NAME}_VERSION_PATCH 6)

set (${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})

message (STATUS "${PROJECT_NAME} version ${${PROJECT_NAME}_VERSION}")

option(WBI_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(WBI_ENABLE_RPATH)

SET( PROJ_SOURCE_DIR ${${PROJECT_NAME}_SOURCE_DIR} )
SET( PROJ_BINARY_DIR ${${PROJECT_NAME}_BINARY_DIR} )

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

SET(folder_source src/wbiUtil.cpp
                  src/wbiID.cpp
                  src/wbiConstants.cpp
                  src/iWholeBodyModel.cpp
                  src/iWholeBodyStates.cpp
                  src/iWholeBodySensors.cpp
                  src/iWholeBodyActuators.cpp
                  src/wholeBodyInterface.cpp
                  src/Error.cpp)

SET(folder_header include/wbi/wbi.h
                  include/wbi/wbiConstants.h
                  include/wbi/wbiUtil.h
                  include/wbi/wbiID.h
                  include/wbi/wbiUtil.inl
                  include/wbi/iWholeBodyModel.h
                  include/wbi/iWholeBodyStates.h
                  include/wbi/iWholeBodySensors.h
                  include/wbi/iWholeBodyActuators.h
                  include/wbi/wholeBodyInterface.h
                  include/wbi/Error.h)

include_directories(include/wbi)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

if(MSVC)
    set(LIB_TYPE STATIC)
else()
    set(LIB_TYPE SHARED)
endif()

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS WBI_ENABLE_RPATH
                          USE_LINK_PATH)


add_library(${LIBRARY_NAME} ${LIB_TYPE} ${folder_source} ${folder_header})

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES
  SOVERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}"
  VERSION "${${PROJECT_NAME}_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}"
  PUBLIC_HEADER "${folder_header}"
)


if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12.2")
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(CMAKE_VERSION VERSION_LESS 2.8.12.2)
    if(NOT WBI_DISABLE_RPATH)
        set_target_properties(${LIBRARY_NAME} PROPERTIES
            INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
        )
    endif(NOT WBI_DISABLE_RPATH)
endif(CMAKE_VERSION VERSION_LESS 2.8.12.2)

target_include_directories(${LIBRARY_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")
INSTALL(TARGETS ${LIBRARY_NAME}
  EXPORT ${PROJECT_NAME}-target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/${HEADERS_DIR}
)

INSTALL(DIRECTORY include/${HEADERS_DIR} DESTINATION include/${HEADERS_DIR})

EXPORT(TARGETS ${LIBRARY_NAME} FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-target.cmake")

#EXPORT(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(cmake/${PROJECT_NAME}Config.cmake.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(cmake/${PROJECT_NAME}ConfigVersion.cmake.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME})
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME})
INSTALL(EXPORT ${PROJECT_NAME}-target  DESTINATION share/${PROJECT_NAME})

include(AddUninstallTarget)
