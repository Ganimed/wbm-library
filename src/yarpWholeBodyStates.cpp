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

    for(int encoder_id = 0; encoder_id < (int)estimateIdList[SENSOR_ENCODER_POS].size(); encoder_id++)
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

    for(int encoder_id = 0; encoder_id < (int)estimateIdList[SENSOR_ENCODER_POS].size(); encoder_id++)
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


    for(int encoder_id = 0; encoder_id < (int)estimateIdList[SENSOR_ENCODER_POS].size(); encoder_id++)
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

    int estimatorPeriod_in_ms = 10;
    if( wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").check("estimatorPeriod") &&
        wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").find("estimatorPeriod").isDouble() )
    {
        double estimatorPeriod_in_ms_dbl = wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").find("estimatorPeriod").asDouble();
        if( estimatorPeriod_in_ms_dbl >= 1.0 )
        {
            estimatorPeriod_in_ms = (int)estimatorPeriod_in_ms_dbl;
            yInfo() << "yarpWholeBodyStates : valid estimatorPeriod option found"
                    << ", setting estimator thread period to " << estimatorPeriod_in_ms << "milliseconds";
        }
        else
        {
            yWarning() << "yarpWholeBodyStates : estimatorPeriod option found but invalid (< 1ms)"
                       << ", setting estimator thread period to default value of " << estimatorPeriod_in_ms << " milliseconds";
        }
    }
    else
    {
        yInfo() << "yarpWholeBodyStates : estimatorPeriod option not found"
                << ", setting estimator thread period to default value of " << estimatorPeriod_in_ms << " milliseconds";
    }


    sensors = new yarpWholeBodySensors(name.c_str(), wbi_yarp_properties);              // sensor interface
    estimator = new yarpWholeBodyEstimator(estimatorPeriod_in_ms, sensors,wholeBodyModel);  // estimation thread


    if( wbi_yarp_properties.check("readSpeedAccFromControlBoard") )
    {
        yInfo() << "yarpWholeBodyStates : readSpeedAccFromControlBoard option found, reading velocities and accelerations from controlboard";
        estimator->readSpeedAccFromControlBoard = true;
    }
    else
    {
        yInfo() << "yarpWholeBodyStates : readSpeedAccFromControlBoard option not found, reading velocities and accelerations from high level numerical derivatives";
        estimator->readSpeedAccFromControlBoard = false;
    }

    if( wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").check("estimateBasePosAndVel") )
    {
        yInfo() << "yarpWholeBodyStates : estimateBasePosAndVel option found, we are estimating base position and velocity";
        estimator->estimateBasePosAndVel = true;
    }
    else
    {
        yInfo() << "yarpWholeBodyStates : estimateBasePosAndVel option not found, we are estimating base position and velocity";
        estimator->estimateBasePosAndVel = false;
    }

    if(wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").check("WORLD_REFERENCE_FRAME"))
    {
        yInfo() << "Found world reference frame mention in wbiConfig. Setting as "<<wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").find("WORLD_REFERENCE_FRAME").asString().c_str();
        estimator->setWorldBaseLinkName(wbi_yarp_properties.findGroup("WBI_STATE_OPTIONS").find("WORLD_REFERENCE_FRAME").asString().c_str());
    }
    else
    {
        yWarning()<<"Did not find WORLD_REFERENCE_FRAME option in config file, disabling world to base position and velocity estimation";
        estimator->estimateBasePosAndVel = false;
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
                sensors->addSensors(SENSOR_ENCODER_POS, estimateIdList[et]);
                break;
            case ESTIMATE_JOINT_VEL:
                sensors->addSensors(SENSOR_ENCODER_SPEED, estimateIdList[et]);
                break;
            case ESTIMATE_JOINT_ACC:
                sensors->addSensors(SENSOR_ENCODER_ACCELERATION, estimateIdList[et]);
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

bool yarpWholeBodyStates::addEstimate(const EstimateType et, const ID &sid)
{
    if( initDone ) return false;

    estimateIdList[et].addID(sid);
    return true;
}

int yarpWholeBodyStates::addEstimates(const EstimateType et, const IDList &sids)
{
    if( initDone ) return false;
    estimateIdList[et].addIDList(sids);
    return true;
}

bool yarpWholeBodyStates::removeEstimate(const EstimateType et, const ID &sid)
{
    if( initDone ) return false;

   estimateIdList[et].removeID(sid);
   return true;
}

const IDList& yarpWholeBodyStates::getEstimateList(const EstimateType et)
{
    if( initDone )
    {
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorList(SENSOR_ENCODER_POS);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorList(SENSOR_ENCODER_SPEED);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorList(SENSOR_ENCODER_ACCELERATION);
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
    case ESTIMATE_BASE_POS:
    {
        if( estimator->estimateBasePosAndVel )
        {
            return estimator->lockAndCopyVector(estimator->estimates.lastBasePos,data);
        }
        else
        {
            return false;
        }
    }
    case ESTIMATE_BASE_VEL:
    {
        if( estimator->estimateBasePosAndVel )
        {
            return estimator->lockAndCopyVector(estimator->estimates.lastBaseVel,data);
        }
        else
        {
            return false;
        }
    }
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

    return false;
}

bool yarpWholeBodyStates::getMotorVel(const int numeric_id, double *data, double /*time*/, bool /*blocking*/)
{
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
//                                         YARP WHOLE BODY ESTIMATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyEstimator::yarpWholeBodyEstimator(int _period_in_milliseconds, yarpWholeBodySensors *_sensors, wbi::iWholeBodyModel *wholeBodyModelRef)
: RateThread(_period_in_milliseconds),
  sensors(_sensors),
  dqFilt(0),
  d2qFilt(0),
  dTauJFilt(0),
  dTauMFilt(0),
  tauJFilt(0),
  tauMFilt(0),
  motor_quantites_estimation_enabled(false),
  dvbVect(6),
  dqjVect(NULL,NULL),
  floatingBase_jacobian(6,6),
  rotationalVelocityWrapper(NULL,NULL)
{

    wholeBodyModel = wholeBodyModelRef;
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER_POS));

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

    //default setting: invalid
    robot_reference_frame_link = -1;

}

bool yarpWholeBodyEstimator::threadInit()
{
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER_POS));
    ///< create derivative filters
    dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
    d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
    dTauJFilt = new AWLinEstimator(dTauJFiltWL, dTauJFiltTh);
    dTauMFilt = new AWLinEstimator(dTauMFiltWL, dTauMFiltTh);
    ///< read sensors
    assert((int)estimates.lastQ.size() == sensors->getSensorNumber(SENSOR_ENCODER_POS));
    bool ok = sensors->readSensors(SENSOR_ENCODER_POS, estimates.lastQ.data(), qStamps.data(), true);
    ok = ok && sensors->readSensors(SENSOR_TORQUE, estimates.lastTauJ.data(), tauJStamps.data(), true);
    ok = ok && sensors->readSensors(SENSOR_PWM, estimates.lastPwm.data(), 0, true);
    ///< create low pass filters
    tauJFilt    = new FirstOrderLowPassFilter(tauJCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    tauMFilt    = new FirstOrderLowPassFilter(tauMCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    pwmFilt     = new FirstOrderLowPassFilter(pwmCutFrequency, getRate()*1e-3, estimates.lastPwm);

     int dof = estimates.lastQ.length();

    complete_jacobian.resize(6,dof+6);
    joint_jacobian.resize(6,dof);
    tempMatForComputation.resize(6,dof);
    new (&rotationalVelocityWrapper) Eigen::Map<Eigen::VectorXd>(estimates.lastBaseVel.data(), estimates.lastBaseVel.size());
    new (&dqjVect) Eigen::Map<Eigen::VectorXd>(estimates.lastDq.data(),dof);


    return ok;
}


Eigen::Map<Eigen::VectorXd> toEigen(yarp::sig::Vector & vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(),vec.size());
}

void yarpWholeBodyEstimator::run()
{
    mutex.wait();
    {
        resizeAll(sensors->getSensorNumber(SENSOR_ENCODER_POS));

        ///< Read encoders
        if(sensors->readSensors(SENSOR_ENCODER_POS, q.data(), qStamps.data(), false))
        {
            estimates.lastQ = q;

            // in case we estimate the speeds and accelerations instead of reading them
            AWPolyElement el;
            el.data = q;
            el.time = yarp::os::Time::now();

            /* If the encoders speeds/accelerations estimation by the firmware are enabled
            read these values from the controlboard. */
            if(this->readSpeedAccFromControlBoard )
            {
                sensors->readSensors(SENSOR_ENCODER_SPEED, dq.data(), 0, false);
                sensors->readSensors(SENSOR_ENCODER_ACCELERATION, d2q.data(), 0, false);

                estimates.lastDq = dq;
                estimates.lastD2q = d2q;
            }
            else
            {
                estimates.lastDq = dqFilt->estimate(el);
                estimates.lastD2q = d2qFilt->estimate(el);
            }

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

        // Compute world to base position, if the estimate was added
        if( this->estimateBasePosAndVel )
        {
            computeBasePosition(estimates.lastQ.data());
            computeBaseVelocity(estimates.lastQ.data(),estimates.lastDq.data());
        }

    }
    mutex.post();

    return;
}

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
    dq.resize(n);
    d2q.resize(n);
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
        yInfo()<<", now it is set to "<<robot_reference_frame_link;
        return true;
    }
    else
    {
        return false;
    }

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

        dvbVect.setZero();
        complete_jacobian.setZero();
        joint_jacobian.setZero();
        floatingBase_jacobian.setZero();
        tempMatForComputation.setZero();


        wholeBodyModel->computeJacobian(qj,world_H_rootLink,robot_reference_frame_link,complete_jacobian.data());
        floatingBase_jacobian = complete_jacobian.leftCols(6);
        joint_jacobian = complete_jacobian.rightCols(dof);


        tempMatForComputation = (floatingBase_jacobian.inverse()*joint_jacobian);
        tempMatForComputation*=-1.0;

        dvbVect =tempMatForComputation*dqjVect;
        rotationalVelocityWrapper = dvbVect;

        return true;
    }
    else
    {
        return false;
    }
}


