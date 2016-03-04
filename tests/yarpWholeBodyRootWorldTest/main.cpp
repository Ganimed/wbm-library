/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Tests for wholeBodyInterfaceYarp.
 */
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>
#include <fstream>

#include "yarpWholeBodyInterface.h"
#include "yarpWbiUtil.h"

#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

#include <Eigen/Core>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace yarpWbi;//wbiIcub;
using namespace Eigen;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    Network yarpNet;

    Property options;
    options.fromCommand(argc,argv);

    std::string robotName;
    if(options.check("robot")) {
      robotName = options.find("robot").asString();
    } else {
      robotName = "icubGazeboSim";
    }
    /*
    bool use_urdf = false;

    std::string urdf_file;
    if(options.check("urdf")) {
        use_urdf = true;
        urdf_file = options.find("urdf").asString();
    } else {
        use_urdf = false;
    }

    */


  printf("Robot name is  \n\n");
  yarp::os::ResourceFinder rf;
  yarp::os::Property yarpWbiOptions;
  //Get wbi options from the canonical file
  if( !rf.check("yarp") )
  {
      fprintf(stderr,"[ERR] locomotionControl: impossible to open wholeBodyInterface: wbi_conf_file option missing");
  }

  rf.setVerbose (true);
  rf.setDefaultConfigFile ("yarpWholeBodyInterface.ini");

  rf.configure(0,0);

  std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");
  yarpWbiOptions.fromConfigFile(wbiConfFile);
  //Overwrite the robot parameter that could be present in wbi_conf_file
  yarpWbiOptions.put("robot",robotName);

    // TEST WHOLE BODY INTERFACE
    std::string localName = "wbiTest";

    wholeBodyInterface *icub;


    icub =  new yarpWbi::yarpWholeBodyInterface(localName.c_str(), yarpWbiOptions);
    std::cout << "icubWholeBodyInterface created, adding joints" << std::endl;
  wbi::IDList RobotMainJoints;
  std::string RobotMainJointsListName = "ROBOT_TORQUE_CONTROL_JOINTS";
  if( !yarpWbi::loadIdListFromConfig(RobotMainJointsListName,yarpWbiOptions,RobotMainJoints) )
  {
      fprintf(stderr, "[ERR] locomotionControl: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
  }
  icub->addJoints(RobotMainJoints);

    //icub->addFTsens(LocalId(RIGHT_LEG,1));
    std::cout << "Joints added, calling init method" <<  std::endl;

    if(!icub->init())
        return 0;

    int dof = icub->getDoFs();
    printf("Joint list: %s\n", icub->getJointList().toString().c_str());
    printf("Number of DoFs: %d\n", dof);

    Vector q(dof), dq(dof), d2q(dof), qInit(dof), qd(dof),basePos(12), baseVel(6);

    double timeIni = Time::now();
     icub->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data());
    double timeEnd = Time::now();


    double elapsedTime = timeEnd - timeIni;
    qInit = q;
    qd = q;


    int moveMode; // 1 - torso twist, 2 - knee crunch

    if(options.check("testType")) {

      moveMode = options.find("testType").asInt();
    }

    if(moveMode != 1 && moveMode !=2)
    {
      moveMode = 1;
    }
    /*else {
      robotName = "icubGazeboSim";
    }*/


    std::ofstream logFile;
    std::string tempName = "/tmp/wholeBodyEstimateRootWorldTestDataset_";
    std::stringstream ssm;
    ssm<<tempName<<moveMode<<".dat";
    tempName = ssm.str();
    //sprintf(tempName,
    logFile.open(tempName.c_str());

    for (int temp = 0;temp<8;temp++)
    {
      Vector refSpeed(dof, yarpWbi::Deg2Rad*10.0);//, qd = q;

      float posd;
      if(temp%2==0)
      {
	posd = +25.0;
      }
      else
      {
	posd = -25.0;
      }

      if(moveMode == 1)
      {
	for (int ctr = 0; ctr <13;ctr++)
	{
	  qd(ctr) += posd*yarpWbi::Deg2Rad;
	}
      }
      else
      {
	unsigned int tempJVectPlus[]  = {13,17,19,24};
	unsigned int tempJVectMinus[]  = {16,22};

	for (int ctr = 0; ctr<4;ctr++)
	{
	  qd(tempJVectPlus[ctr]) += posd*yarpWbi::Deg2Rad;
	}
	for (int ctr = 0; ctr<2;ctr++)
	{
	  qd(tempJVectMinus[ctr]) -= posd*yarpWbi::Deg2Rad;
	}

      }
      printf("Q:   %s\n", (yarpWbi::Rad2Deg*q).toString(1).c_str());
      printf("Qd:  %s\n", (yarpWbi::Rad2Deg*qd).toString(1).c_str());
      icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
      icub->setControlReference(qd.data());
      Eigen::Matrix<double,6,Dynamic,RowMajor> jacob;
      jacob.resize(6,dof+6); //13 because in this test we only have right and left arm plus torso

      for(int i=0; i<100; i++)
      {
	  Vector com(7,0.0);
	  wbi::Frame world2base;
	  world2base.identity();

	  Time::delay(0.05);
// 	  icub->getEstimates(ESTIMATE_JOINT_POS, q.data());
// 	  icub->getEstimates(ESTIMATE_JOINT_VEL, dq.data());
// 	  icub->getEstimates(ESTIMATE_JOINT_ACC,d2q.data());
// 	    printf("(Q, dq, d2q):   %.2f \t %.2f \t %.2f\n", yarpWbi::Rad2Deg*q(j), yarpWbi::Rad2Deg*dq(j), yarpWbi::Rad2Deg*d2q(j));

	    icub->getEstimates(ESTIMATE_BASE_POS,basePos.data());
	    printf("BasePos: (%2.6f %2.6f %2.6f)\n\n",basePos(0),basePos(1),basePos(2));

	    printf("BaseRot:\n %2.6f %2.6f %2.6f\n %2.6f %2.6f %2.6f\n %2.6f %2.6f %2.6f\n",basePos(3),basePos(4),basePos(5),basePos(6),basePos(7),basePos(8),basePos(9),basePos(10),basePos(11));

	    icub->getEstimates(ESTIMATE_BASE_VEL,baseVel.data());
	    printf("BaseVel: (%2.6f %2.6f %2.6f), (%2.6f %2.6f %2.6f)\n\n\n",baseVel(0),baseVel(1),baseVel(2),baseVel(3),baseVel(4),baseVel(5));
	    logFile<<basePos[0]<<","<<basePos[1]<<","<<basePos[2]<<",";
	    logFile<<baseVel[0]<<","<<baseVel[1]<<","<<baseVel[2]<<","<<baseVel[3]<<","<<baseVel[4]<<","<<baseVel[5]<<"\n";

      }
   }
//    printf("Test finished. Press return to exit.");
   getchar();

//    printf("Q:   %s\n", (yarpWbi::Rad2Deg*q).toString(1).c_str());

//    qd -= yarpWbi::Rad2Deg*15.0;
//    icub->setControlMode(CTRL_MODE_POS);
//    icub->setControlReference(qd.data());

   Time::delay(1.0);
   printf("Test finished. Press return to exit.");
   getchar();

   Vector refSpeedFinal(dof, yarpWbi::Deg2Rad*25.0);//, qd = q;
//   qd += 15.0*yarpWbi::Rad2Deg;
//   printf("Q:   %s\n", (yarpWbi::Rad2Deg*q).toString(1).c_str());
//   printf("Qd:  %s\n", (yarpWbi::Rad2Deg*qd).toString(1).c_str());
   icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeedFinal.data());

   icub->setControlReference(qInit.data());

    printf("Test finished. Press return to exit.");
    getchar();

    icub->close();

    delete icub;
    logFile.close();

    printf("Main returning...\n");
    return 0;
}

