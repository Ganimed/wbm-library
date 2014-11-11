option(CODYCO_INSTALL_ALL_ROBOTS "Installation of all available robots" ON)

set(ROBOT_NAME "$ENV{YARP_ROBOT_NAME}" CACHE PATH
    "Name of your robot")

set(ROBOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT_NAME}")

# If no env variable YARP_ROBOT_NAME is defined
if(CODYCO_INSTALL_ALL_ROBOTS)
  add_subdirectory(iCubGenova03)
  add_subdirectory(icubGazeboSim)
else()
  if(ROBOT_NAME)
    add_subdirectory(${ROBOT_NAME})
  endif()
endif() 
  

#setup symbolic link (on UNIX...)

if ("${ROBOT_NAME}" STREQUAL "")
  message(STATUS "No environmental variable YARP_ROBOT_NAME set up. Installing all robots by default")
else ("${ROBOT_NAME}" STREQUAL "")
  if (EXISTS ${ROBOT_DIR})
    
    if(UNIX)
      message(STATUS "Setting up symbolic link for ${ROBOT_NAME}")
      install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/${ICUB_ROBOTS_INSTALL_DIR}/${ROBOT_NAME})\n
      execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_INSTALL_PREFIX}/${ICUB_ROBOTS_INSTALL_DIR}/${ROBOT_NAME} ${CMAKE_INSTALL_PREFIX}/${ICUB_ROBOTS_INSTALL_DIR}/default)")
      add_custom_target(robotSymLink ALL COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/${ICUB_ROBOTS_INSTALL_DIR}/${ROBOT_NAME} ${CMAKE_BINARY_DIR}/${ICUB_ROBOTS_INSTALL_DIR}/default DEPENDS ${CMAKE_BINARY_DIR}/${ICUB_ROBOTS_INSTALL_DIR}/${ROBOT_NAME})      
    else()
      message(STATUS "Cannot set up symbolic link for ${ROBOT_NAME} on this OS, please set the YARP_ROBOT_NAME environment variable to ${ROBOT_NAME}")
    endif()

  else(EXISTS ${ROBOT_DIR})
    message(WARNING "ROBOT_NAME is set to ${ROBOT_NAME} but the app/robots/${ROBOT_NAME} directory does not exist")
  endif(EXISTS ${ROBOT_DIR})
endif ("${ROBOT_NAME}" STREQUAL "")
