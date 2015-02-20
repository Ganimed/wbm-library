/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
 * email: andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <iCub/skinDynLib/common.h>
#include <yarp/os/Time.h>
#include <string>
#include <boost/concept_check.hpp>
//#include <eigen3/Eigen/src/Core/arch/SSE/Complex.h>
#include<Eigen/Core>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/api.h>

#include "yarpWholeBodyInterface/yarpWholeBodyStates.h"
#include "yarpWholeBodyInterface/yarpWbiUtil.h"

#include <wbi/iWholeBodyModel.h>

//#include <Eigen/Sparse>
#include <Eigen/LU>

using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;
using namespace yarp::math;

#define ESTIMATOR_PERIOD 10 // <------- FIXME TODO UUUUUUUUUGGGGGGGLYYYYYYY


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyStates::yarpWholeBodyStates(const char* _name, const yarp::os::Property & opt,wbi::iWholeBodyModel *wholeBodyModelRef):
initDone(false), 
name(_name), 
wbi_yarp_properties(opt),
sensors(0),
estimator(0)
{
    estimateIdList.resize(wbi::ESTIMATE_TYPE_SIZE);
    wholeBodyModel = wholeBodyModelRef;
}

bool yarpWholeBodyStates::setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties)
{
    if( initDone ) return false;

    wbi_yarp_properties = yarp_wbi_properties;
    return true;
}

bool yarpWholeBodyStates::getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties)
{
    yarp_wbi_properties = wbi_yarp_properties;
    return true;
}

yarpWholeBodyStates::~yarpWholeBodyStates()
{
    close();
}

bool yarpWholeBodyStates::loadCouplingsFromConfigurationFile()
{
    Bottle couplings_bot = wbi_yarp_properties.findGroup("WBI_YARP_JOINTS_MOTOR_KINEMATIC_COUPLINGS");
    if( couplings_bot.isNull() )
    {
        std::cerr << "[WARN] yarpWholeBodyStates::loadCouplingsFromConfigurationFile : WBI_YARP_JOINTS_MOTOR_KINEMATIC_COUPLINGS group not found in configuration file" << std::endl;
        return false;
    }

    for(int encoder_id = 0; encoder_id < (int)estimateIdList[SENSOR_ENCODER].size(); encoder_id++)
    {
        //search if coupling information is provided for each motor, if not return false
        wbi::ID joint_encoder_name;
        estimateIdList[ESTIMATE_JOINT_POS].indexToID(encoder_id,joint_encoder_name);
        if( !couplings_bot.check(joint_encoder_name.toString()) )
        {
            std::cerr << "[ERR] WBI_YARP_JOINTS_MOTOR_KINEMATIC_COUPLINGS group found, but no coupling found for joint "
                      << joint_encoder_name.toString() << ", exiting" << std::endl;
                      return false;
        }

        //Check coupling configuration is well formed
        Bottle * joint_coupling_bot = couplings_bot.find(joint_encoder_name.toString()).asList();
        if( !joint_coupling_bot )
        {
            std::cerr << "[ERR] WBI_YARP_JOINTS_MOTOR_KINEMATIC_COUPLINGS group found, but coupling found for joint "
                      << joint_encoder_name.toString() << " is malformed" << std::endl;
            return false;
        }

        for(int coupled_motor=0; coupled_motor < joint_coupling_bot->size(); coupled_motor++ )
        {
            if( !(joint_coupling_bot->get(coupled_motor).isList()) ||
                !(joint_coupling_bot->get(coupled_motor).asList()->size() == 2) ||
                !(joint_coupling_bot->get(coupled_motor).asList()->get(0).isDouble()) ||
                !(joint_coupling_bot->get(coupled_motor).asList()->get(1).isString()) )
            {
                std::cerr << "[ERR] WBI_YARP_JOINTS_MOTOR_KINEMATIC_COUPLINGS group found, but coupling found for joint "
                      << joint_encoder_name.toString() << " is malformed" << std::endl;
                return false;
            }
        }
    }

    //Reset motor estimate list (the motor list will be induced by the joint list
    estimateIdList[ESTIMATE_MOTOR_POS] = IDList();

    for(int encoder_id = 0; encoder_id < (int)estimateIdList[SENSOR_ENCODER].size(); encoder_id++)
    {
        wbi::ID joint_encoder_name;
        estimateIdList[ESTIMATE_JOINT_POS].indexToID(encoder_id,joint_encoder_name);

        //Check coupling configuration is well formed
        Bottle * joint_coupling_bot = couplings_bot.find(joint_encoder_name.toString()).asList();

        //Load motors names
        for(int coupled_motor=0; coupled_motor < joint_coupling_bot->size(); coupled_motor++ )
        {
            std::string motor_name = joint_coupling_bot->get(coupled_motor).asList()->get(1).asString();
            //Add the motor name to all relevant estimates lists
            estimateIdList[ESTIMATE_MOTOR_POS].addID(motor_name);
        }

    }

    if( estimateIdList[ESTIMATE_MOTOR_POS].size() !=  estimateIdList[ESTIMATE_JOINT_POS].size() )
    {
        std::cerr << "[ERR] WBI_YARP_JOINTS_MOTOR_KINEMATIC_COUPLINGS group found, but coupling where"
                  << "the number of joints is different from the number of motors is not currently supported" << std::endl;
                return false;
    }

    //Build coupling matrix
    int encoders = estimateIdList[ESTIMATE_MOTOR_POS].size();

    Eigen::MatrixXd motor_to_joint_kinematic_coupling(encoders,encoders);
    motor_to_joint_kinematic_coupling.setZero();


    for(int encoder_id = 0; encoder_id < (int)estimateIdList[SENSOR_ENCODER].size(); encoder_id++)
    {
        wbi::ID joint_encoder_name;
        estimateIdList[ESTIMATE_JOINT_POS].indexToID(encoder_id,joint_encoder_name);

        //Check coupling configuration is well formed
        Bottle * joint_coupling_bot = couplings_bot.find(joint_encoder_name.toString()).asList();

        //Load coupling
        for(int coupled_motor=0; coupled_motor < joint_coupling_bot->size(); coupled_motor++ )
        {
            double coupling_coefficient = joint_coupling_bot->get(coupled_motor).asList()->get(0).asDouble();
            std::string motor_name = joint_coupling_bot->get(coupled_motor).asList()->get(1).asString();
            int motor_id;
            estimateIdList[ESTIMATE_MOTOR_POS].idToIndex(motor_name,motor_id);

            motor_to_joint_kinematic_coupling(encoder_id,motor_id) = coupling_coefficient;
        }

    }

    //Transform loaded coupling to the one actually needed
    Eigen::MatrixXd I(encoders,encoders);
    I.setIdentity();
    double sparse_eps = 1e-3;
    Eigen::MatrixXd joint_to_motor_kinematic_coupling_dense = motor_to_joint_kinematic_coupling.inverse();
    Eigen::MatrixXd joint_to_motor_torque_coupling_dense = motor_to_joint_kinematic_coupling.transpose();
    estimator->joint_to_motor_kinematic_coupling = joint_to_motor_kinematic_coupling_dense;//.sparseView(sparse_eps);
    estimator->joint_to_motor_torque_coupling = joint_to_motor_torque_coupling_dense;//.sparseView(sparse_eps);

    estimator->motor_quantites_estimation_enabled = true;

    return true;
}

bool yarpWholeBodyStates::init()
{
    if( initDone ) return false;

    std::string robot;

    //Loading configuration
    if( wbi_yarp_properties.check("robot") )
    {
        robot = wbi_yarp_properties.find("robot").asString().c_str();
    }
    else if (wbi_yarp_properties.check("robotName") )
    {
        std::cerr << "[WARN] yarpWholeBodyStates: robot option not found, using robotName" << std::endl;
        robot = wbi_yarp_properties.find("robotName").asString().c_str();
    }
    else
    {
        std::cerr << "[ERR] yarpWholeBodyStates: robot option not found" << std::endl;
        return false;
    }

    sensors = new yarpWholeBodySensors(name.c_str(), wbi_yarp_properties);              // sensor interface
    estimator = new yarpWholeBodyEstimator(ESTIMATOR_PERIOD, sensors,wholeBodyModel);  // estimation thread

    //wb
    if(wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").check("WORLD_REFERENCE_FRAME"))
    {
     //yInf
      yInfo()<<"\n\n\n\nFound world reference frame mention in yarpConfig. Setting as "<<wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").find("WORLD_REFERENCE_FRAME").asString().c_str()<<"\n\n\n";
      estimator->setWorldBaseLinkName(wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").find("WORLD_REFERENCE_FRAME").asString().c_str());      
    }
    else
    {
      yInfo()<<"\n\n\n\nDid not find WORLD_REFERENCE_FRAME option in config file\n\n\n";
    }
    
    //Add required sensors given the estimate list
    // TODO FIXME ugly, we should probably have a way to iterate on estimate type
    // indipendent from enum values
    for(int et_i=0; et_i < wbi::ESTIMATE_TYPE_SIZE; et_i++)
    {
        EstimateType et = static_cast<EstimateType>(et_i);
        //bool SensorAddOk = true;
        //int addedSensors = 0;
        // this is the perfect example of switch that should be avoided
        switch(et)
        {
            case ESTIMATE_JOINT_POS:
            case ESTIMATE_JOINT_VEL:
            case ESTIMATE_JOINT_ACC:
                sensors->addSensors(SENSOR_ENCODER, estimateIdList[et]);
                break;

            case ESTIMATE_JOINT_TORQUE:
            case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
            case ESTIMATE_MOTOR_TORQUE:
            case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:
                sensors->addSensors(SENSOR_TORQUE, estimateIdList[et]);
                break;
            case ESTIMATE_MOTOR_PWM:
                sensors->addSensors(SENSOR_PWM,  estimateIdList[et]);
                break;
            //case ESTIMATE_IMU:
            //  SensorAddOk = lockAndAddSensors(SENSOR_IMU, sids);
            //  break;
            case ESTIMATE_FORCE_TORQUE_SENSOR:
                sensors->addSensors(SENSOR_FORCE_TORQUE, estimateIdList[et]);
                break;
            case ESTIMATE_EXTERNAL_FORCE_TORQUE:
                ///< \todo TODO properly account for external forque torque
                break;
            default:
                break;
        }
    }

    // Load joint coupling information
    this->loadCouplingsFromConfigurationFile();

    // Initialized sensor interface
    bool ok = sensors->init();              // initialize sensor interface
    if( !ok )
    {
        std::cerr << "[ERR] yarpWholeBodyStates::init : sensor interface initialization failed" << std::endl;
        std::cerr << "[ERR] yarpWholeBodyStates::init : halting initialization routine" << std::endl;
        delete sensors;
        sensors = 0;
        return false;
    }


    ok = estimator->start();

    if(ok)
    {
        std::cout << "[DEBUG] yarpWholeBodyStates correctly initialized " << std::endl;
        this->initDone = true;
        return true;
    }
    else
    {
        std::cerr << "[ERR] yarpWholeBodyStates::init : estimator thread initialization failed" << std::endl;

        sensors->close();
        delete sensors;
        sensors = 0;
        return false; // start estimation thread
    }
}

bool yarpWholeBodyStates::close()
{
    if(estimator) estimator->stop();  // stop estimator BEFORE closing sensor interface
    bool ok = (sensors ? sensors->close() : true);
    if(sensors) { delete sensors; sensors = 0; }
    if(estimator) { delete estimator; estimator = 0; }
    return ok;
}

bool yarpWholeBodyStates::setWorldBasePosition(const wbi::Frame & xB)
{
    return false;
    //return estimator->setWorldBasePosition(xB);
}

bool yarpWholeBodyStates::addEstimate(const EstimateType et, const ID &sid)
{
    if( initDone ) return false;
    /*
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensor(SENSOR_PWM, sid);
    //case ESTIMATE_IMU:                      return lockAndAddSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndAddSensor(SENSOR_FORCE_TORQUE, sid);
    ///< \todo TODO properly account for external forque torque
    case ESTIMATE_EXTERNAL_FORCE_TORQUE:    return false; //estimator->openEEWrenchPorts();
    default: break;
    }
    return false;
    */
    estimateIdList[et].addID(sid);
    return true;
}

int yarpWholeBodyStates::addEstimates(const EstimateType et, const IDList &sids)
{
    if( initDone ) return false;
    /*
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensors(SENSOR_PWM, sids);
    //case ESTIMATE_IMU:                    return lockAndAddSensors(SENSOR_IMU, sids);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndAddSensors(SENSOR_FORCE_TORQUE, sids);
    ///< \todo TODO properly account for external forque torque
    case ESTIMATE_EXTERNAL_FORCE_TORQUE:    return true;
    default: break;
    }
    return false;
    */
    estimateIdList[et].addIDList(sids);
    return true;
}

bool yarpWholeBodyStates::removeEstimate(const EstimateType et, const ID &sid)
{
    if( initDone ) return false;

    /*
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndRemoveSensor(SENSOR_PWM, sid);
    //case ESTIMATE_IMU:                      return lockAndRemoveSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:             return lockAndRemoveSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
    */
   estimateIdList[et].removeID(sid);
   return true;
}

const IDList& yarpWholeBodyStates::getEstimateList(const EstimateType et)
{
    if( initDone )
    {
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorList(SENSOR_TORQUE);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_POS:                return estimateIdList[ESTIMATE_MOTOR_POS];
    case ESTIMATE_MOTOR_VEL:                return estimateIdList[ESTIMATE_MOTOR_POS];
    case ESTIMATE_MOTOR_ACC:                return estimateIdList[ESTIMATE_MOTOR_POS];
    case ESTIMATE_MOTOR_TORQUE:             return estimateIdList[ESTIMATE_MOTOR_POS];
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return estimateIdList[ESTIMATE_MOTOR_POS];
    case ESTIMATE_MOTOR_PWM:                return estimateIdList[ESTIMATE_MOTOR_POS];
    //case ESTIMATE_IMU:                    return sensors->getSensorList(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorList(SENSOR_FORCE_TORQUE);
    // ESTIMATE_EXTERNAL_FORCE_TORQUE: return list of links where is possible to get contact
    default: break;
    }
    return emptyList;
    }
    return estimateIdList[et];
}

int yarpWholeBodyStates::getEstimateNumber(const EstimateType et)
{
    if( initDone ) return false;
    /*
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorNumber(SENSOR_PWM);
    //case ESTIMATE_IMU:                      return sensors->getSensorNumber(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorNumber(SENSOR_FORCE_TORQUE);
    ESTIMATE_EXTERNAL_FORCE_TORQUE: return number of links where is possible to get contact
    default: break;
    }
    return 0;
    */
    return estimateIdList[et].size();
}

bool yarpWholeBodyStates::getEstimate(const EstimateType et, const int numeric_id, double *data, double time, bool blocking)
{
    if( !initDone ) return false;
    switch(et)
    {
    case ESTIMATE_JOINT_POS:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:
        return false;
    case ESTIMATE_MOTOR_VEL:
        return getMotorVel(numeric_id, data, time, blocking);
    case ESTIMATE_MOTOR_ACC:
        return false;
    case ESTIMATE_MOTOR_TORQUE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:
        return lockAndReadSensor(SENSOR_PWM, numeric_id, data, time, blocking);
    //case ESTIMATE_IMU:
    //    return lockAndReadSensor(SENSOR_IMU, sid, data, time, blocking);
    case ESTIMATE_FORCE_TORQUE_SENSOR:
        return lockAndReadSensor(SENSOR_FORCE_TORQUE, numeric_id, data, time, blocking);
    case ESTIMATE_EXTERNAL_FORCE_TORQUE:
        return false; //lockAndGetExternalWrench(sid,data);
   case ESTIMATE_BASE_POS:
	return estimator->lockAndCopyVectorElement(numeric_id,estimator->estimates.lastBasePos,data);
   case ESTIMATE_BASE_VEL:
	return estimator->lockAndCopyVectorElement(numeric_id,estimator->estimates.lastBaseVel,data);
     //  return estimator->lockAndCopyVectorElement(numeric_id,estimator->lastBasePos ,data);
    default: break;
    }
    return false;
}

bool yarpWholeBodyStates::getEstimates(const EstimateType et, double *data, double time, bool blocking)
{
    if( !initDone )
    {
        printf("[ERR] yarpWholeBodyStates::getEstimates error, called before init\n");
        return false;
    }

    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return estimator->lockAndCopyVector(estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:                return estimator->lockAndCopyVector(estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:                return estimator->lockAndCopyVector(estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:             return estimator->lockAndCopyVector(estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return estimator->lockAndCopyVector(estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:                return false;
    case ESTIMATE_MOTOR_VEL:                return getMotorVel(data, time, blocking);
    case ESTIMATE_MOTOR_ACC:                return false;
    case ESTIMATE_MOTOR_TORQUE:             return estimator->lockAndCopyVector(estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return estimator->lockAndCopyVector(estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:                return lockAndReadSensors(SENSOR_PWM, data, time, blocking);
    //case ESTIMATE_IMU:                    return lockAndReadSensors(SENSOR_IMU, data, time, blocking);
    case ESTIMATE_BASE_POS:		    return estimator->lockAndCopyVector(estimator->estimates.lastBasePos,data);
    case ESTIMATE_BASE_VEL:		    return estimator->lockAndCopyVector(estimator->estimates.lastBaseVel,data);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndReadSensors(SENSOR_FORCE_TORQUE, data, time, blocking);
    default: break;
    }
    return false;
}

bool yarpWholeBodyStates::setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    return estimator->lockAndSetEstimationParameter(et, ep, value);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          PRIVATE METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************

bool yarpWholeBodyStates::getMotorVel(double *data, double time, bool /*blocking*/)
{
    bool res = estimator->lockAndCopyVector(estimator->estimates.lastDq, data);    ///< read joint vel
    if(!res) return false;
    /*
    IDList idList = lockAndGetSensorList(SENSOR_ENCODER);
    int i=0;
    FOR_ALL_OF(itBp, itJ, idList)   ///< manage coupled joints
    {
        if(itBp->first == LEFT_ARM && *itJ==0)          // left arm shoulder
            data[i] = data[i];
        else if(itBp->first == LEFT_ARM && *itJ==1)     // left arm shoulder
            data[i] = data[i];
        else if(itBp->first == LEFT_ARM && *itJ==2)     // left arm shoulder
            data[i] = data[i];
        else if(itBp->first == RIGHT_ARM && *itJ==0)    // right arm shoulder
            data[i] = data[i];
        else  if(itBp->first == RIGHT_ARM && *itJ==1)   // right arm shoulder
            data[i] = data[i];
        else if(itBp->first == RIGHT_ARM && *itJ==2)    // right arm shoulder
            data[i] = data[i];
        else if(itBp->first == TORSO && *itJ==1)        // torso
            data[i] = data[i];
        else if(itBp->first == TORSO && *itJ==2)        // torso
            data[i] = data[i];
        i++;
    }
    */
    return false;
}

bool yarpWholeBodyStates::getMotorVel(const int numeric_id, double *data, double /*time*/, bool /*blocking*/)
{
    ///< read joint vel
    //return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(lid), estimator->estimates.lastDq, data);
    return false;
}

bool yarpWholeBodyStates::lockAndReadSensors(const SensorType st, double *data, double /*time*/, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensors(st, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool yarpWholeBodyStates::lockAndReadSensor(const SensorType st, const int numeric_id, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensor(st, numeric_id, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

/*
bool yarpWholeBodyStates::lockAndGetExternalWrench(const int numeric_id, double * data)
{
    return false;

    estimator->mutex.wait();
    if( !estimator->ee_wrenches_enabled )
    {
        estimator->mutex.post();
        return false;
    }

    if( sid == estimator->right_gripper_local_id )
    {
        memcpy(data,estimator->RAExtWrench.data(),6*sizeof(double));
    }
    else if( sid == estimator->left_gripper_local_id )
    {
        memcpy(data,estimator->LAExtWrench.data(),6*sizeof(double));
    }
    else if( sid == estimator->right_sole_local_id )
    {
        memcpy(data,estimator->RLExtWrench.data(),6*sizeof(double));
    }
    else if( sid == estimator->left_sole_local_id )
    {
        memcpy(data,estimator->LLExtWrench.data(),6*sizeof(double));
    }
    estimator->mutex.post();
    return true;
}
*/

bool yarpWholeBodyStates::lockAndAddSensor(const SensorType st, const ID &sid)
{
    estimator->mutex.wait();
    bool res = sensors->addSensor(st, sid);
    estimator->mutex.post();
    return res;
}

int yarpWholeBodyStates::lockAndAddSensors(const SensorType st, const IDList &sids)
{
    estimator->mutex.wait();
    int res = sensors->addSensors(st, sids);
    estimator->mutex.post();
    return res;
}

bool yarpWholeBodyStates::lockAndRemoveSensor(const SensorType st, const ID &sid)
{
    estimator->mutex.wait();
    bool res = sensors->removeSensor(st, sid);
    estimator->mutex.post();
    return res;
}

IDList yarpWholeBodyStates::lockAndGetSensorList(const SensorType st)
{
    estimator->mutex.wait();
    IDList res = sensors->getSensorList(st);
    estimator->mutex.post();
    return res;
}

int yarpWholeBodyStates::lockAndGetSensorNumber(const SensorType st)
{
    estimator->mutex.wait();
    int res = sensors->getSensorNumber(st);
    estimator->mutex.post();
    return res;
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY ESTIMATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyEstimator::yarpWholeBodyEstimator(int _period, yarpWholeBodySensors *_sensors, wbi::iWholeBodyModel *wholeBodyModelRef)
: RateThread(_period),
  sensors(_sensors),
  dqFilt(0),
  d2qFilt(0),
  dTauJFilt(0),
  dTauMFilt(0),
  tauJFilt(0),
  tauMFilt(0),
  motor_quantites_estimation_enabled(false)
 //,
  //ee_wrenches_enabled(false)
{
  
    wholeBodyModel = wholeBodyModelRef;
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));

    ///< Window lengths of adaptive window filters
    dqFiltWL            = 16;
    d2qFiltWL           = 25;
    dTauJFiltWL         = 30;
    dTauMFiltWL         = 30;

    ///< Threshold of adaptive window filters
    dqFiltTh            = 1.0;
    d2qFiltTh           = 1.0;
    dTauJFiltTh         = 0.2;
    dTauMFiltTh         = 0.2;

    ///< Cut frequencies
    tauJCutFrequency    =   3.0;
    tauMCutFrequency    =   3.0;
    pwmCutFrequency     =   3.0;
    
    //default setting for refence frame as l_sole for icub to maintain backward compatibility
    robot_reference_frame_link = 9;
   
      
}

bool yarpWholeBodyEstimator::threadInit()
{
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    ///< create derivative filters
    dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
    d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
    dTauJFilt = new AWLinEstimator(dTauJFiltWL, dTauJFiltTh);
    dTauMFilt = new AWLinEstimator(dTauMFiltWL, dTauMFiltTh);
    ///< read sensors
    assert((int)estimates.lastQ.size() == sensors->getSensorNumber(SENSOR_ENCODER));
    bool ok = sensors->readSensors(SENSOR_ENCODER, estimates.lastQ.data(), qStamps.data(), true);
    ok = ok && sensors->readSensors(SENSOR_TORQUE, estimates.lastTauJ.data(), tauJStamps.data(), true);
    ok = ok && sensors->readSensors(SENSOR_PWM, estimates.lastPwm.data(), 0, true);
    ///< create low pass filters
    tauJFilt    = new FirstOrderLowPassFilter(tauJCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    tauMFilt    = new FirstOrderLowPassFilter(tauMCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    pwmFilt     = new FirstOrderLowPassFilter(pwmCutFrequency, getRate()*1e-3, estimates.lastPwm);

    //H_world_base.resize(4,4);
    //H_world_base.eye();
   
 //   robot_reference_frame_link = 9; // Default value corresponds to l_sole to maintain backward compatibility.
 /*
    if(wholeBodyModel!=NULL)
    {
      wholeBodyModel->getFrameList().idToIndex("l_sole",robot_reference_frame_link);
    }
 */   
    /*
    right_gripper_local_id = wbi::ID(RIGHT_ARM,8);
    left_gripper_local_id = wbi::ID(LEFT_ARM,8);
    left_sole_local_id = wbi::ID(LEFT_LEG,8);
    right_sole_local_id = wbi::ID(RIGHT_LEG,8);
    */
    
    /*    Eigen::Map<Eigen::VectorXd> dqjVect(dqj,dof);
    Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> complete_jacobian(6,dof+6), joint_jacobian(6,dof),floatingBase_jacobian(6,6);
    Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> tempMatForComputation(6,dof);//, minusTemp(6,dof);
    Eigen::VectorXd dvbVect(6);
    Eigen::Map<Eigen::VectorXd> yarpWrapper(estimates.lastBaseVel.data(), estimates.lastBaseVel.size());
  */  
/*
    int dof = estimates.lastQ.length();
    
    complete_jacobian.resize(6,dof+6);
    joint_jacobian.resize(6,dof);
    floatingBase_jacobian(6,6);
    tempMatForComputation.resize(6,dof);
    dvbVect.resize(6);
    new(&rotationalVelocityWrapper) Eigen::Map<Eigen::VectorXd>(estimates.lastBaseVel.data(), estimates.lastBaseVel.size());
    //Eigen::Map<Eigen::VectorXd> yarpWrapper;//(estimates.lastBaseVel.data(), estimates.lastBaseVel.size());
    //new (&v) Map<RowVectorXi>(data+4,5);
    /*
    std::cout<<" CompleteJacobian size : "<<complete_jacobian.rows()<<"X"<<complete_jacobian.cols()<<"\n";
    std::cout<<" JointJacobian size : "<<joint_jacobian.rows()<<"X"<<joint_jacobian.cols()<<"\n";
    std::cout<<" FloatingBaseJacobian size : "<<floatingBase_jacobian.rows()<<"X"<<floatingBase_jacobian.cols()<<"\n";
    std::cout<<" TempMat size : "<<tempMatForComputation.rows()<<"X"<<tempMatForComputation.cols()<<"\n";
    std::cout<<" rotationalVelocityWrapper size : "<<rotationalVelocityWrapper.rows()<<"\n\n";*/
    return ok;
}

/*
bool yarpWholeBodyEstimator::openEEWrenchPorts()
{
    return false;

    bool okEE = true;
    if( ee_wrenches_enabled ) return true;
    mutex.wait();
    okEE = okEE && openEEWrenchPorts(right_gripper_local_id);
    okEE = okEE && openEEWrenchPorts(left_gripper_local_id);
    okEE = okEE && openEEWrenchPorts(right_sole_local_id);
    okEE = okEE && openEEWrenchPorts(left_sole_local_id);
    ee_wrenches_enabled = okEE;
    mutex.post();

    return okEE;

}*/

bool yarpWholeBodyEstimator::setWorldBasePosition(const wbi::Frame & xB)
{
    return false;
    /*
    mutex.wait();
    H_world_base.resize(4,4);
    xB.get4x4Matrix(H_world_base.data());
    mutex.post();
    return true;
    */
}

Eigen::Map<Eigen::VectorXd> toEigen(yarp::sig::Vector & vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(),vec.size());
}

void yarpWholeBodyEstimator::run()
{
    mutex.wait();
    {
        resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));

        ///< Read encoders
        if(sensors->readSensors(SENSOR_ENCODER, q.data(), qStamps.data(), false))
        {
            estimates.lastQ = q;
            AWPolyElement el;
            el.data = q;
            el.time = yarp::os::Time::now();
            estimates.lastDq = dqFilt->estimate(el);
            estimates.lastD2q = d2qFilt->estimate(el);

            //if motor quantites are enabled, estimate also motor motor_quantities
            if( this->motor_quantites_estimation_enabled )
            {
                toEigen(estimates.lastQM)
                    = this->joint_to_motor_kinematic_coupling*toEigen(estimates.lastQ);
                toEigen(estimates.lastDqM)
                    = this->joint_to_motor_kinematic_coupling*toEigen(estimates.lastDq);
                toEigen(estimates.lastD2qM)
                    = this->joint_to_motor_kinematic_coupling*toEigen(estimates.lastD2q);
            }
        }

        ///< Read joint torque sensors
        if(sensors->readSensors(SENSOR_TORQUE, tauJ.data(), tauJStamps.data(), false))
        {
            // @todo Convert joint torques into motor torques
            AWPolyElement el;
            el.time = yarp::os::Time::now();

            estimates.lastTauJ = tauJFilt->filt(tauJ);  ///< low pass filter

            if( this->motor_quantites_estimation_enabled )
            {
                toEigen(estimates.lastTauM)
                    = this->joint_to_motor_torque_coupling*toEigen(estimates.lastTauJ);
            }

            //Here there are some inefficencies... \todo TODO FIXME
            el.data = tauJ;
            estimates.lastDtauJ = dTauJFilt->estimate(el);  ///< derivative filter

            if( this->motor_quantites_estimation_enabled )
            {
                el.data = estimates.lastTauM;
                estimates.lastDtauM = dTauMFilt->estimate(el);  ///< derivative filter
            }
        }

        ///< Read motor pwm
        sensors->readSensors(SENSOR_PWM, pwm.data(), 0, false);
        estimates.lastPwm = pwmFilt->filt(pwm);     ///< low pass filter

        //This pwms are actually obtained through getOutputs() yarp calls, so they are
        //"joint" PWMs that need to be decoupled
        if( this->motor_quantites_estimation_enabled )
        {
            toEigen(estimates.lastPwmBuffer)
                = this->joint_to_motor_torque_coupling*toEigen(estimates.lastPwm);
            estimates.lastPwm = estimates.lastPwmBuffer;
        }

        // Compute world to base 
        computeBasePosition(estimates.lastQ.data());
	computeBaseVelocity(estimates.lastQ.data(),estimates.lastDq.data());
    }
    mutex.post();

    return;
}

/*
bool yarpWholeBodyEstimator::openEEWrenchPorts(const wbi::ID & local_id)
{
    std::string wbd_module_name = "wholeBodyDynamicsTree";
    std::string part, remotePort;
    part = getPartName(local_id);
    if( part == "")
    {
        std::cerr << "yarpWholeBodyEstimator::openEEWrenchPorts: local_id not found " << std::endl;
    }
    remotePort = "/" + wbd_module_name + "/" + part + "/" + "cartesianEndEffectorWrench:o";
    stringstream localPort;
    localPort << "/" << "yarpWholeBodyEstimator" << "/eeWrench" << local_id.bodyPart << "_" << local_id.index << ":i";
    portsEEWrenches[local_id] = new BufferedPort<Vector>();
    if(!portsEEWrenches[local_id]->open(localPort.str().c_str())) {
        // open local input port
        std::cerr << " yarpWholeBodyEstimator::openEEWrenchPorts: Open of localPort " << localPort << " failed " << std::endl;
        //YARP_ASSERT(false);
        ee_wrenches_enabled = false;
        return false;
    }
    if(!Network::exists(remotePort.c_str())) {            // check remote output port exists
        std::cerr << "yarpWholeBodyEstimator::openEEWrenchPorts:  " << remotePort << " does not exist " << std::endl;
        //YARP_ASSERT(false);
        ee_wrenches_enabled = false;
        return false;
    }
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp")) {  // connect remote to local port
        std::cerr << "yarpWholeBodyEstimator::openEEWrenchPorts:  could not connect " << remotePort << " to " << localPort << std::endl;
        //YARP_ASSERT(false);
        ee_wrenches_enabled = false;
        return false;
    }

    //allocate lastRead variables
    lastEEWrenches[local_id].resize(6,0.0);
    ee_wrenches_enabled = true;

    return true;
}

void yarpWholeBodyEstimator::readEEWrenches(const wbi::ID & local_id, yarp::sig::Vector & vec)
{
    vec.resize(6);
    vec.zero();
    if( ee_wrenches_enabled )
    {
        yarp::sig::Vector*res = portsEEWrenches[local_id]->read();
        if( res )
        {
            lastEEWrenches[local_id].setSubvector(0,H_world_base.submatrix(0,2,0,2)*(*res).subVector(0,2));
            lastEEWrenches[local_id].setSubvector(3,H_world_base.submatrix(0,2,0,2)*(*res).subVector(3,5));
        }
        vec = lastEEWrenches[local_id];
    }
}

void yarpWholeBodyEstimator::closeEEWrenchPorts(const wbi::ID & local_id)
{
    if( ee_wrenches_enabled )
    {
        portsEEWrenches[local_id]->close();
        delete portsEEWrenches[local_id];
    }
}*/

void yarpWholeBodyEstimator::threadRelease()
{
    //this causes a memory access violation (to investigate)
    if(dqFilt!=0)    { delete dqFilt;  dqFilt=0; }
    if(d2qFilt!=0)   { delete d2qFilt; d2qFilt=0; }
    if(dTauJFilt!=0) { delete dTauJFilt; dTauJFilt=0; }
    if(dTauMFilt!=0) { delete dTauMFilt; dTauMFilt=0; }     // motor torque derivative filter
    if(tauJFilt!=0)  { delete tauJFilt; tauJFilt=0; }  ///< low pass filter for joint torque
    if(tauMFilt!=0)  { delete tauMFilt; tauMFilt=0; }  ///< low pass filter for motor torque
    if(pwmFilt!=0)   { delete pwmFilt; pwmFilt=0;   }

    /*
    if( ee_wrenches_enabled )
    {
        closeExternalWrenchPorts(right_gripper_local_id);
    }*/

    return;
}

void yarpWholeBodyEstimator::lockAndResizeAll(int n)
{
    mutex.wait();
    resizeAll(n);
    mutex.post();
}

void yarpWholeBodyEstimator::resizeAll(int n)
{
    q.resize(n);
    qStamps.resize(n);
    tauJ.resize(n);
    tauJStamps.resize(n);
    pwm.resize(n);
    pwmStamps.resize(n);
    estimates.lastQ.resize(n);
    estimates.lastDq.resize(n);
    estimates.lastD2q.resize(n);
    estimates.lastTauJ.resize(n);
    estimates.lastTauM.resize(n);
    estimates.lastDtauJ.resize(n);
    estimates.lastDtauM.resize(n);
    estimates.lastPwm.resize(n);
    estimates.lastPwmBuffer.resize(n);
    estimates.lastBasePos.resize(12);
    estimates.lastBaseVel.resize(6);
    estimates.lastBaseAccl.resize(6);
}

bool yarpWholeBodyEstimator::lockAndCopyVector(const Vector &src, double *dest)
{
    if(dest==0)
    {
        printf("[ERR] yarpWholeBodyEstimator::lockAndCopyVector called with NULL dest");
        return false;
    }
    mutex.wait();
    memcpy(dest, src.data(), sizeof(double)*src.size());
    mutex.post();
    return true;
}

bool yarpWholeBodyEstimator::lockAndCopyVectorElement(int index, const Vector &src, double *dest)
{
    mutex.wait();
    dest[0] = src[index];
    mutex.post();
    return true;
}

bool yarpWholeBodyEstimator::lockAndSetEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    bool res = false;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_JOINT_VEL:
    case ESTIMATE_MOTOR_VEL:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setVelFiltParams(((int*)value)[0], dqFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setVelFiltParams(dqFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_JOINT_ACC:
    case ESTIMATE_MOTOR_ACC:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setAccFiltParams(((int*)value)[0], d2qFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setAccFiltParams(d2qFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_JOINT_TORQUE:
        if(ep==ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ)
            res = setTauJCutFrequency(((double*)value)[0]);
        break;

    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setDtauJFiltParams(((int*)value)[0], dTauJFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setDtauJFiltParams(dTauJFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_MOTOR_TORQUE:
        if(ep==ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ)
            res = setTauMCutFrequency(((double*)value)[0]);
        break;

    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setDtauMFiltParams(((int*)value)[0], dTauMFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setDtauMFiltParams(dTauMFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_MOTOR_PWM:
        if(ep==ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ)
            res = setPwmCutFrequency(((double*)value)[0]);
        break;

    //case ESTIMATE_IMU:
    case ESTIMATE_FORCE_TORQUE_SENSOR:
    case ESTIMATE_JOINT_POS:
    case ESTIMATE_MOTOR_POS:
    default: break;
    }
    mutex.post();
    return res;
}

bool yarpWholeBodyEstimator::setVelFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    dqFiltWL = windowLength;
    dqFiltTh = threshold;
    if(dqFilt!=NULL)
    {
        AWPolyList list = dqFilt->getList();
        dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            dqFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyEstimator::setAccFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    d2qFiltWL = windowLength;
    d2qFiltTh = threshold;
    if(d2qFilt!=NULL)
    {
        AWPolyList list = d2qFilt->getList();
        d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            d2qFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyEstimator::setDtauJFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    dTauJFiltWL = windowLength;
    dTauJFiltTh = threshold;
    if(dTauJFilt!=NULL)
    {
        AWPolyList list = dTauJFilt->getList();
        dTauJFilt = new AWLinEstimator(windowLength, threshold);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            dTauJFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyEstimator::setDtauMFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    dTauMFiltWL = windowLength;
    dTauMFiltTh = threshold;
    if(dTauMFilt!=NULL)
    {
        AWPolyList list = dTauMFilt->getList();
        dTauMFilt = new AWLinEstimator(windowLength, threshold);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            dTauMFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyEstimator::setTauJCutFrequency(double fc)
{
    return tauJFilt->setCutFrequency(fc);
}

bool yarpWholeBodyEstimator::setTauMCutFrequency(double fc)
{
    return tauMFilt->setCutFrequency(fc);
}

bool yarpWholeBodyEstimator::setPwmCutFrequency(double fc)
{
    return pwmFilt->setCutFrequency(fc);
}
bool yarpWholeBodyEstimator::setWorldBaseLinkName(std::string linkName)
{
  if(wholeBodyModel!=NULL)
  {
    yInfo()<<"Reference link set as world was "<<robot_reference_frame_link;
    wholeBodyModel->getFrameList().idToIndex(linkName.c_str(),robot_reference_frame_link);
    yInfo()<<", now it is set to "<<robot_reference_frame_link<<"\n\n\n";
    return(true);
  }
  else
    return(false);

}



bool yarpWholeBodyEstimator::computeBasePosition(double *q_temp)
{
  if(wholeBodyModel!=NULL)
  {
      wholeBodyModel->computeH(q_temp,wbi::Frame::identity(),robot_reference_frame_link, rootLink_H_ReferenceLink);
      rootLink_H_ReferenceLink.setToInverse().get4x4Matrix (H_w2b.data());
      referenceLink_H_rootLink.set4x4Matrix (H_w2b.data());
      world_H_rootLink = world_H_reference*referenceLink_H_rootLink ;
      
      int ctr;
      
      for (ctr=0;ctr<3;ctr++)
      {
	estimates.lastBasePos(ctr) = world_H_rootLink.p[ctr];	
      }
      for (ctr=0;ctr<9;ctr++)
      {
	estimates.lastBasePos(3+ctr) = world_H_rootLink.R.data[ctr];
      }
      return(true);
  }
  else    
    return(false);
}
bool yarpWholeBodyEstimator::computeBaseVelocity(double* qj,double* dqj)
{
  
  if(wholeBodyModel!=NULL)
  {
    int dof = estimates.lastQ.size();
   // new (&dqjVect) Eigen::Map<Eigen::VectorXd>(dqj,dof);
    
    Eigen::Map<Eigen::VectorXd> dqjVect(dqj,dof);
    Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> complete_jacobian(6,dof+6), joint_jacobian(6,dof),floatingBase_jacobian(6,6);
    Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> tempMatForComputation(6,dof);//, minusTemp(6,dof);
    Eigen::VectorXd dvbVect(6);
    Eigen::Map<Eigen::VectorXd> rotationalVelocityWrapper(estimates.lastBaseVel.data(), estimates.lastBaseVel.size());
    
    dvbVect.setZero();
    complete_jacobian.setZero();
    joint_jacobian.setZero();
    floatingBase_jacobian.setZero();
    tempMatForComputation.setZero();
    //std::cout<<" 1. temp size : "<<temp.rows()<<"X"<<temp.cols()<<"\n\n";

    wholeBodyModel->computeJacobian(qj,world_H_rootLink,robot_reference_frame_link,complete_jacobian.data());  
    floatingBase_jacobian = complete_jacobian.leftCols(6);
    joint_jacobian = complete_jacobian.rightCols(dof);
    
    
    tempMatForComputation = (floatingBase_jacobian.inverse()*joint_jacobian);
    
    // std::cout<<" 3. temp size : "<<temp.rows()<<"X"<<temp.cols()<<"\n\n";
    tempMatForComputation*=-1.0;
    //minusTemp = -1.0 * temp;
    
    //std::cout<<"4.dvb size : "<<dqjVect.rows()<<"X"<<dqjVect.cols()<<" temp size : "<<temp.rows()<<"X"<<temp.cols()<<"\n\n";
//     std::cout<<"\ndqj: "<<dqjVect.transpose()<<"\n \n";
    
    dvbVect =tempMatForComputation*dqjVect;
 
    rotationalVelocityWrapper = dvbVect;
    
    return(true);
  }
  else
    return(false);
}


