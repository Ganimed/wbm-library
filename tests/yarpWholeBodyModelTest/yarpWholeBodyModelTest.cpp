/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Tests for yarpWholeBodyModel.
 */
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/os/Property.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <yarp/os/ResourceFinder.h>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include <wbi/wbiUtil.h>

#include <stdio.h>
#include <math.h>
#include <string>
#include <cstdlib>

#include <iostream>
#include <set>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace wbi;
using namespace yarpWbi;

const double TOL = 1e-8;

/**
 * Do a consistency test on a wholeBodyModel interface. Nothing of implementation specific,
 * so eventually we could move this function to wbi to provided a consistency check for an arbitrary implementation
 *
 */
bool checkInverseDynamicsAndMassMatrixConsistency(iWholeBodyModel * model_interface,
                                                  const wbi::IDList & possible_joints,
                                                 double tol, bool verbose)
{
    std::set<int> added_joints;

    int nr_of_possible_joints = possible_joints.size();

    //Select the random number of considered joints
    int nr_of_considered_joints = rand() % nr_of_possible_joints;
    int nr_of_activated_joints = 0;

    double threshold = ((double)nr_of_considered_joints)/((double)nr_of_possible_joints);

    //Select the random considered joints
    while(  nr_of_activated_joints < nr_of_considered_joints ) {
        for(int j=0; j < (int)possible_joints.size(); j++) {
            if( nr_of_activated_joints < nr_of_considered_joints && yarp::math::Rand::scalar() < threshold ) {
                ID id;
                possible_joints.indexToID(j,id);
                if( added_joints.find(j) == added_joints.end()  ) {
                    model_interface->addJoint(id);
                    nr_of_activated_joints++;
                    added_joints.insert(j);
                }
            }
        }
    }

    bool ret = model_interface->init();

    if( !ret )
    {
       std::cerr << "Model interface init failed, test failed\n" << std::endl;
    }

    for( int local_check= 0; local_check < 10; local_check++ )
    {

    assert(nr_of_activated_joints == nr_of_considered_joints);
    assert(nr_of_considered_joints == (int)model_interface->getJointList().size());

    //std::cout << "checkInverseDynamicsAndMassMatrixConsistency: nrOfPossibleJoints : " << nr_of_possible_joints << " nrOfConsiderJoints " << nr_of_considered_joints << " " << nr_of_activated_joints << std::endl;

    //Select the random input

    wbi::Frame xB(wbi::Rotation::RPY(Rand::scalar(),Rand::scalar(),Rand::scalar()));

    //wbi::Frame xB;//(wbi::Rotation::rotX(M_PI));


    xB.p[0] = Rand::scalar();
    xB.p[1] = Rand::scalar();
    xB.p[2] = Rand::scalar();


    yarp::sig::Vector dxB = 2*M_PI*yarp::math::Rand::vector(6);
    yarp::sig::Vector ddxB = 2*M_PI*yarp::math::Rand::vector(6);
    yarp::sig::Vector g = yarp::math::Rand::vector(3);
    yarp::sig::Vector theta = 2*M_PI*yarp::math::Rand::vector(nr_of_considered_joints);
    yarp::sig::Vector dtheta = 2*M_PI*yarp::math::Rand::vector(nr_of_considered_joints);
    yarp::sig::Vector ddtheta = 2*M_PI*yarp::math::Rand::vector(nr_of_considered_joints);


    //xB = wbi::Frame::identity();
    //dxB.zero();


    yarp::sig::Vector dq =  cat(dxB,dtheta);
    yarp::sig::Vector ddq = cat(ddxB,ddtheta);



    //Create the outputs
    yarp::sig::Vector generalized_torques(6+nr_of_considered_joints);
    yarp::sig::Vector generalized_torques_computed_with_mass_matrix(6+nr_of_considered_joints);
    yarp::sig::Vector generalized_bias_torques(6+nr_of_considered_joints);
    yarp::sig::Matrix mass_matrix(6+nr_of_considered_joints,6+nr_of_considered_joints);

    mass_matrix.zero();

    if( !model_interface->inverseDynamics(theta.data(),xB,dtheta.data(),dxB.data(),ddtheta.data(),ddxB.data(),g.data(),generalized_torques.data()) ) {
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: inverseDynamics failed" << std::endl; }
        return false;
    }
    if( !model_interface->computeMassMatrix(theta.data(),xB,mass_matrix.data()) ) {
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: computeMassMatrix failed" << std::endl; }
        return false;
    }
    if( !model_interface->computeGeneralizedBiasForces(theta.data(),xB,dtheta.data(),dxB.data(),g.data(),generalized_bias_torques.data()) ) {
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: computeGeneralizedBiasForces failed" << std::endl; }
        return false;
    }



    std::cout << "Mass Matrix:             " << std::endl << mass_matrix.toString() << std::endl;
    std::cout << "ddq:                     " << std::endl << ddq.toString() << std::endl;
    std::cout << "M*ddq                    " << std::endl << (mass_matrix*ddq).toString() << std::endl;
    std::cout << "M*ddq with inv dyn       " << std::endl << (generalized_torques-generalized_bias_torques).toString() << std::endl;
    std::cout << "bias:                    " << std::endl << generalized_bias_torques.toString() << std::endl;

    std::cout << "invDyn:                  " << std::endl << generalized_torques.toString() << std::endl;


    generalized_torques_computed_with_mass_matrix = mass_matrix*ddq + generalized_bias_torques;
    //std::cout << "invDyn with mass matrix: " << std::endl << generalized_torques_computed_with_mass_matrix.toString() << std::endl;


    for(int i = 0; i < (int)generalized_torques.size(); i++ ) {
        if( fabs(generalized_torques[i]-generalized_torques_computed_with_mass_matrix[i]) > tol ) {
            if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: generalized torque " << i << " is different, failing" << std::endl; }
            return false;
        }
    }

    //Compute consistency of linear momentum with center of mass velocity
    double robotMass = mass_matrix(0,0);
    yarp::sig::Matrix com_jacobian(6,6+nr_of_considered_joints);
    //yarp::sig::Matrix momentum_jacobian(6,6+nr_of_considered_joints);
    yarp::sig::Vector centroidal_momentum(6);
    yarp::sig::Vector com_twist(6);
    yarp::sig::Vector linear_momentum(3);

   if( !model_interface->computeCentroidalMomentum(theta.data(),xB,dtheta.data(),dxB.data(),centroidal_momentum.data()) ) {
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: computeCentroidalMomentum failed" << std::endl; }
        return false;
    }
    if( !model_interface->computeJacobian(theta.data(),xB,iWholeBodyModel::COM_LINK_ID,com_jacobian.data()) ) {
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: computeJacobian for COM failed" << std::endl; }
        return false;
    }

    com_twist = com_jacobian*dq;

    for(int i=0; i <= 2; i++) {
        linear_momentum(i) = robotMass*com_twist(i);
    }

    /*
    std::cout << "Com twist:               " << std::endl << com_twist.toString() << std::endl;
    std::cout << "centroidal_momentu       " << std::endl << centroidal_momentum.toString() << std::endl;
    std::cout << "robotmass                " << std::endl << robotMass << std::endl;
    std::cout << "linear_momentu           " << std::endl << linear_momentum.toString() << std::endl;
    */

    for(int i=0; i <= 2; i++) {
        if( fabs(linear_momentum[i]-centroidal_momentum[i]) > tol ) {
            if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: component " << i << " of the linear momentum is different, failing" << std::endl; }
            return false;
        }

    }

    }

    return true;
}

int main(int argc, char * argv[])
{
    Property options;
    options.fromCommand(argc,argv);

    int n_checks = 10;
    if( options.check("n_checks") ) {
        n_checks = options.find("n_checks").asInt();
    }

    // TEST WHOLE BODY MODEL
    std::string localName = "wbiTest";
    std::string robotName = "icub";
    std::cout << "Creating yarpWholeBodyModel with robotName " << robotName << " and localName " << localName << std::endl;

    //--------------------------WHOLE BODY INTERFACE--------------------------
    yarp::os::ResourceFinder rf;
    yarp::os::Property yarpWbiOptions;
    std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");
    yarpWbiOptions.fromConfigFile(wbiConfFile);

    IDList RobotDynamicModelJoints;
    std::string RobotDynamicModelJointsName = "ROBOT_DYNAMIC_MODEL_JOINTS";
    if( !loadIdListFromConfig(RobotDynamicModelJointsName,yarpWbiOptions,RobotDynamicModelJoints) )
    {
        fprintf(stderr, "[ERR] locomotionControl: impossible to load wbiId joint list with name %s\n",RobotDynamicModelJointsName.c_str());
        return EXIT_FAILURE;
    }

    Rand::init();
    for(int i = 0; i < n_checks; i++ ) {
        if( i % 100 == 0 ) { std::cout << "wholeBodyModelIcub inverse dynamics : test " << i << std::endl; }
        iWholeBodyModel *icub = new yarpWholeBodyModel(localName.c_str(), yarpWbiOptions);
        if( ! checkInverseDynamicsAndMassMatrixConsistency(icub,RobotDynamicModelJoints,TOL,true) ) {
            return EXIT_FAILURE;
        }
        delete icub;
    }


    return EXIT_SUCCESS;
}
