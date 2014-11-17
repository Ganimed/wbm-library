/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Example for wholeBodyInterface
 */

// Yarp Headers
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <yarp/os/ResourceFinder.h>

// wholeBodyInterface headers
#include <wbi/wholeBodyInterface.h>

// yarpWholeBodyInterface headers
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.configure(argc,argv);


    if( rf.check("help") )
    {
        std::cout << "yarpWholeBodyInterface example, available options:" << std::endl;
        std::cout << "  --wbi_conf_file filename : specify the configuration file used by the wbi," << std::endl
                  << "                             that is searched using the standard ResourceFinder" << std::endl
                  << "                             policies (see http://wiki.icub.org/yarpdoc/yarp_resource_finder_tutorials.html)" << std::endl
                  << "                             (default value : yarpWholeBodyInterface.ini)" << std::endl;
        std::cout << "  --robot robotName       : yarp port prefix used by the robot." << std::endl
                  << "                            (default value: the one contained in wbi_conf_file file ) " << std::endl;
        std::cout << "  --urdf  urdf_filename   : specify the URDF file used by the wbi. Also this file is found using" << std::endl
                  << "                            the standard ResouceFinder policy." << std::endl
                  << "                            (default value: the one contained in wbi_conf_file file ) " << std::endl;
        std::cout << "  --local localName        : yarp port prefix used by this example to open yarp ports. " << std::endl;
        std::cout << "                           : (default value: wbiTest)" << std::endl;

        return 0;
    }

    //Parse options from command line
    // first find the configuration file use to configure the wbi
    // if a file is specified with the wbi_conf_file option, use that one
    // otherwise use the standard yarpWholeBodyInterface.ini name.
    //
    // In both cases, the specified file name is searched using the
    // standard ResourceFinder policy.
    // For more informations see http://wiki.icub.org/yarpdoc/yarp_data_dirs.html
    yarp::os::Property yarpWbiConfiguration;
    std::string yarpWbiConfigurationFile;

    if(rf.check("wbi_conf_file") && rf.find("wbi_conf_file").isString())
    {
        yarpWbiConfigurationFile = rf.findFile("wbi_conf_file");
    }
    else
    {
        yarpWbiConfigurationFile = rf.findFileByName("yarpWholeBodyInterface.ini");
    }

    //It may be convenient to overload some option of the configuration file,
    // so we load in the yarpWbiConfiguration also the option passed in the command line
    yarpWbiConfiguration.fromConfigFile(yarpWbiConfigurationFile);

    yarpWbiConfiguration.fromString(rf.toString().c_str(),false);

    // Create yarpWholeBodyInterface
    std::string localName = "wbiTest";
    if( rf.check("local") )
    {
        localName = rf.find("local").asString();
    }

    wbi::wholeBodyInterface *yarpRobot = new yarpWbi::yarpWholeBodyInterface (localName.c_str(), yarpWbiConfiguration);

    std::cout << "yarpWholeBodyInterface created, adding joints" << std::endl;
    wbi::IDList RobotMainJoints;
    std::string RobotMainJointsListName = "ROBOT_TORQUE_CONTROL_JOINTS";
    if( !yarpWbi::loadIdListFromConfig(RobotMainJointsListName,yarpWbiConfiguration,RobotMainJoints) )
    {
        fprintf(stderr, "[ERR] locomotionControl: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
    }
    yarpRobot->addJoints(RobotMainJoints);

    std::cout << "Joints added, calling init method" <<  std::endl;

    if(!yarpRobot->init())
    {
        std::cout << "Error: init() method failed" << std::endl;
        return -1;
    }

    Time::delay(0.5);

    //Get the number of controlled degrees of freedom of the robot
    int dof = yarpRobot->getDoFs();
    printf("Number of (internal) controlled DoFs: %d\n", dof);

	//Allocate yarp vector of the right dimensions
    Vector q(dof), dq(dof), d2q(dof), qInit(dof), qd(dof), trq(dof);

    //Read position and torque estimates
    yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data());
    yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_TORQUE, trq.data());

    //Set a bunch of desired positions, by just adding 15 degrees to the current position of the joints
    qd = q;
    qInit = q;
    qd = 45.0*CTRL_DEG2RAD+q;
    printf("Current and desired position:\n");
    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
    printf("Qd:  %s\n", (CTRL_RAD2DEG*qd).toString(1).c_str());

    //Set the robot in position control mode
    yarpRobot->setControlMode(wbi::CTRL_MODE_POS);

    //Set trajectory velocity parameters
    Vector refSpeed(dof, CTRL_DEG2RAD*20.0);
    yarpRobot->setControlParam(wbi::CTRL_PARAM_REF_VEL, refSpeed.data());

   printf("Set control reference to qd:\n");
    yarpRobot->setControlReference(qd.data());

    //The wbi is indipendent uses just double * in its interfaces
    //so is compatible with yarp vector data type, eigen data types
    //and with all data types that expose a pointer to their data
    //storage
    Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> jacob;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> fb_mass_matrix;


    //For more information on the dimension of the jacobian
    //check iWholeBodyModel doxygen documentation
    jacob.resize(6,dof+6);
    fb_mass_matrix.resize(dof+6,dof+6);

    Vector com(7,0.0);

    //Just looping for 15 seconds
    for(int i=0; i<15; i++)
    {

        wbi::Frame world2base;
        world2base.identity();

        Time::delay(1);
        yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data());
        yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_VEL, dq.data());
        yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_ACC,d2q.data());

        yarpRobot->computeJacobian(q.data(),world2base,wbi::iWholeBodyModel::COM_LINK_ID,jacob.data());

        yarpRobot->computeMassMatrix(q.data(),world2base,fb_mass_matrix.data());

        yarpRobot->forwardKinematics(q.data(),world2base,wbi::iWholeBodyModel::COM_LINK_ID,com.data());

    }

    //Go back to original position
    std::cout << "Setting position " << std::endl;
    qd -= CTRL_DEG2RAD*15.0;
    yarpRobot->setControlMode(wbi::CTRL_MODE_POS);
    yarpRobot->setControlReference(qInit.data());

    Time::delay(20.0);


    yarpRobot->close();

    delete yarpRobot;

    printf("Main returning...\n");
    return 0;
}

