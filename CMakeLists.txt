# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

cmake_minimum_required(VERSION 2.8.11)
project(yarpWholeBodyInterface CXX)

set(LIBRARY_NAME yarpwholebodyinterface)
set(VARS_PREFIX ${PROJECT_NAME})
set(INCLUDE_DIRS yarpWholeBodyInterface)

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 3)
set(${VARS_PREFIX}_PATCH_VERSION 3)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

option(YARPWBI_USES_KDL "Compile the parts of yarp-wholebodyinterface that depend on KDL" TRUE)
option(COMPILE_AS_SHARED_LIBRARY "Compile ${PROJECT_NAME} as a shared library" TRUE)
option(YARPWHOLEBODYINTERFACE_ENABLE_TESTS "Enable unit testing" FALSE)

find_package(YARP 2.3.63.7 REQUIRED)

if(YARPWBI_USES_KDL)
    find_package(iDynTree 0.3.11 REQUIRED)
    find_package(ICUB REQUIRED)
    find_package(wholeBodyInterface 0.2.6 REQUIRED)
    find_package(Eigen3 REQUIRED)

    SET(folder_source src/yarpWbiUtil.cpp
                    src/yarpWholeBodyInterface.cpp
                    src/yarpWholeBodyModel.cpp
                    src/yarpWholeBodyStates.cpp
                    src/floatingBaseEstimators.cpp
                    src/yarpWholeBodyActuators.cpp
                    src/yarpWholeBodySensors.cpp
                    src/PIDList.cpp)
    SET(folder_header include/yarpWholeBodyInterface/yarpWholeBodyInterface.h
                    include/yarpWholeBodyInterface/yarpWholeBodyModel.h
                    include/yarpWholeBodyInterface/yarpWholeBodyStates.h
                    include/yarpWholeBodyInterface/yarpWholeBodyActuators.h
                    include/yarpWholeBodyInterface/yarpWholeBodySensors.h
                    include/yarpWholeBodyInterface/floatingBaseEstimators.h
                    include/yarpWholeBodyInterface/yarpWbiUtil.h
                    include/yarpWholeBodyInterface/PIDList.h)

    source_group("Source Files" FILES ${folder_source})
    source_group("Header Files" FILES ${folder_header})

    add_definitions(-Wall)

    # import math symbols from standard cmath
    add_definitions(-D_USE_MATH_DEFINES)

    option(YARPWBI_ENABLE_RPATH "Enable RPATH for the library" TRUE)
    mark_as_advanced(YARPWBI_ENABLE_RPATH)

    if(MSVC)
        message(STATUS "Building ${PROJECT_NAME} as a shared library is not currently supported on Windows")
        set(COMPILE_AS_SHARED_LIBRARY FALSE)
    endif()

    if(COMPILE_AS_SHARED_LIBRARY)
        set(LIB_TYPE SHARED)
    else()
        set(LIB_TYPE STATIC)
    endif()

    include(AddInstallRPATHSupport)
    add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                            LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                            DEPENDS YARPWBI_ENABLE_RPATH
                            USE_LINK_PATH)

    add_library(${LIBRARY_NAME}  ${LIB_TYPE} ${folder_source} ${folder_header})

    set_target_properties(${LIBRARY_NAME}  PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                        SOVERSION ${${VARS_PREFIX}_VERSION}
                                        PUBLIC_HEADER "${folder_header}")

    target_include_directories(${LIBRARY_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>")
    target_include_directories(${LIBRARY_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/yarpWholeBodyInterface>"
                                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

    target_include_directories(${LIBRARY_NAME} PUBLIC
                                        ${YARP_INCLUDE_DIRS}
                                        ${iDynTree_INCLUDE_DIRS}
                                        ${wholeBodyInterface_INCLUDE_DIRS}
                                        ${EIGEN3_INCLUDE_DIR}
                                        ${skinDynLib_INCLUDE_DIRS}
                                        ${ctrlLib_INCLUDE_DIRS})


    target_link_libraries(${LIBRARY_NAME} LINK_PUBLIC ${YARP_LIBRARIES}
                                                    ${wholeBodyInterface_LIBRARIES}
                                        LINK_PRIVATE ${iDynTree_LIBRARIES}
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

    if(YARPWHOLEBODYINTERFACE_ENABLE_TESTS)
        #enable testing
        #include(CTest)
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()

include(AddUninstallTarget)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

add_subdirectory(app)

