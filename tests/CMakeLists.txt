find_package(YARP 2.3.62 REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")



# Finding dependencies
find_package(Gazebo REQUIRED)
find_package(Boost REQUIRED serialization system thread)
find_package(kdl_codyco REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDFORMAT sdformat)
  pkg_check_modules(PROTOBUF protobuf)
endif()

# Get gtest
# from http://mifrosu.blogspot.it/2013/02/cmake-and-google-test-framework.html
if(DEFINED ENV{GTEST_ROOT})  # Note we omit the $ here!
    message(" ... using gtest found in $ENV{GTEST_ROOT}")
ELSE()
    message("GTEST_ROOT is not defined. You must tell CMake where
                 to find the gtest source")
    # exit early
    return()
endif()

# note: if adding out-of-tree directories, a target binary directory must
# be specified
add_subdirectory($ENV{GTEST_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/gtest)

# Build static GazeboYarpServerFixture library
set(GazeboYarpServerFixtureSources
  GazeboYarpServerFixture.cc
)

configure_file (gazebo_yarp_test_config.h.in ${PROJECT_BINARY_DIR}/gazebo_yarp_test_config.h)

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${PROJECT_BINARY_DIR}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${SDFORMAT_INCLUDE_DIRS}
                    ${PROTOBUF_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ../include)
add_library(gazebo_yarp_server_fixture STATIC ${GazeboYarpServerFixtureSources})
#add_dependencies(gazebo_yarp_server_fixture gazebo_msgs)
target_link_libraries(gazebo_yarp_server_fixture gtest gtest_main ${Gazebo_LIBRARIES}  ${YARP_LIBRARIES}  ${SDFORMAT_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES})

#adding tests

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${PROJECT_BINARY_DIR}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${SDFORMAT_INCLUDE_DIRS}
                    ${PROTOBUF_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ../include)

add_executable(yarpWbiActuactorsUnitTest yarpWbiActuactorsUnitTest.cpp)
target_link_libraries(yarpWbiActuactorsUnitTest ${YARP_LIBRARIES}
                                                ${Gazebo_LIBRARIES}
                                                ${SDFORMAT_LIBRARIES}
                                                ${GAZEBO_LIBRARIES}
                                                ${PROTOBUF_LIBRARIES}
                                                ${Boost_LIBRARIES}
                                                gazebo_yarp_server_fixture
                                                ${wholeBodyInterface_LIBRARIES}
                                                ${LIBRARY_NAME})

add_test(NAME test_yarpWbiActuactorsUnitTest COMMAND yarpWbiActuactorsUnitTest)

add_executable(yarpWbiModelUnitTest yarpWbiModelUnitTest.cpp)
target_link_libraries(yarpWbiModelUnitTest ${YARP_LIBRARIES}
                                                ${Gazebo_LIBRARIES}
                                                ${SDFORMAT_LIBRARIES}
                                                ${GAZEBO_LIBRARIES}
                                                ${PROTOBUF_LIBRARIES}
                                                ${Boost_LIBRARIES}
                                                gazebo_yarp_server_fixture
                                                ${wholeBodyInterface_LIBRARIES}
                                                ${LIBRARY_NAME})

add_test(NAME test_yarpWbiModelUnitTest COMMAND yarpWbiModelUnitTest)
