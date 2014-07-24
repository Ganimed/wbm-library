/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Example for wholeBodyInterface
 */
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/os/Property.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>

#include <wbiIcub/wholeBodyInterfaceIcub.h>

#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace Eigen;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    Network yarp; 
    Property options;
    options.fromCommand(argc,argv);
    
	//Parse options from command line
    std::string robotName;
    if(options.check("robot")) {
      robotName = options.find("robot").asString();
    } else {
      robotName = "icubGazeboSim";
    }
    
    bool use_urdf = false;
    
    std::string urdf_file;
    if(options.check("urdf")) { 
        use_urdf = true;
        urdf_file = options.find("urdf").asString();
    } else {
        use_urdf = false;
    }
    
    
    // Create yarpWholeBodyInterface
    std::string localName = "wbiTest";
    
    wholeBodyInterface *icub;
    iCub::iDynTree::iCubTree_version_tag icub_version;

	int neck_version = 2;
    int leg_version = 2;
    int has_feet_ft_sensors = true;
    if( use_urdf ) {
    	icub_version = iCub::iDynTree::iCubTree_version_tag (neck_version, leg_version, has_feet_ft_sensors, use_urdf, urdf_file);
	} else {
		icub_version = iCub::iDynTree::iCubTree_version_tag (neck_version, leg_version, has_feet_ft_sensors);
	}

    icub = new icubWholeBodyInterface (localName.c_str(), robotName.c_str(), icub_version);
    
    std::cout << "icubWholeBodyInterface created, adding joints" << std::endl;
    
    //controlling just the torso and the arms
    icub->addJoints(LocalIdList(RIGHT_ARM,0,1,2,3,4));
    icub->addJoints(LocalIdList(LEFT_ARM,0,1,2,3,4));
    icub->addJoints(LocalIdList(TORSO,0,1,2));
    
    //but you can also control the 25 joints of iCub that is possible to control in torque mode
    //icub->addJoints(ICUB_MAIN_JOINTS);
    
    std::cout << "Joints added, calling init method" <<  std::endl;

    if(!icub->init())
    {

        return 0;
    }
    
    Time::delay(0.5);
    
    //Get the number of internal degrees of freedom of the robot
    int dof = icub->getDoFs();
    printf("Joint list: %s\n", icub->getJointList().toString().c_str());
    printf("Number of (internal) DoFs: %d\n", dof);
    
	//Allocate yarp vector of the right dimensions
    Vector q(dof), dq(dof), d2q(dof), qInit(dof), qd(dof), trq(dof);

    //Read position and torque estimates
    icub->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data());
    icub->getEstimates(wbi::ESTIMATE_JOINT_TORQUE, trq.data());

    //Set a bunch of desired positions, by just adding 15 degrees to the current position of the joints 
    qd = q;
    qInit = q;
    qd = 15.0*CTRL_DEG2RAD;
    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
    printf("Qd:  %s\n", (CTRL_RAD2DEG*qd).toString(1).c_str());
    
    //Set trajectory velocity parameters
    Vector refSpeed(dof, CTRL_DEG2RAD*10.0);
    icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
    icub->setControlReference(qd.data());
   
    //The wbi is indipendent uses just double * in its interfaces
    //so is compatible with yarp vector data type, eigen data types
    //and with all data types that expose a pointer to their data
    //storage 
    Eigen::Matrix<double,6,Dynamic,RowMajor> jacob; 

    //For more information on the dimension of the jacobian, check 
    jacob.resize(6,dof+6);
    Vector com(7,0.0);

    //Just looping for 15 seconds
    for(int i=0; i<15; i++)
    {

        wbi::Frame world2base;
        world2base.identity();
        
        Time::delay(1);
        icub->getEstimates(ESTIMATE_JOINT_POS, q.data());
        icub->getEstimates(ESTIMATE_JOINT_VEL, dq.data());
        icub->getEstimates(ESTIMATE_JOINT_ACC,d2q.data());
        
        icub->computeJacobian(q.data(),world2base,wbi::iWholeBodyModel::COM_LINK_ID,jacob.data());
        //cout<<"COM Jacobian: "<<jacob<<endl;
        
        icub->forwardKinematics(q.data(),world2base,wbi::iWholeBodyModel::COM_LINK_ID,com.data());
        printf("Center of Mass:  %.10f \t %.10f \t %.10f\n",com[0],com[1],com[2]);
                
    }
 
    //Go back to original position
    std::cout << "Setting position " << std::endl;
    qd -= CTRL_DEG2RAD*15.0;
    icub->setControlMode(CTRL_MODE_POS);
    icub->setControlReference(qInit.data());

    Time::delay(20.0);
   
 
    icub->close();
    
    delete icub;
    
    printf("Main returning...\n");
    return 0;
}

