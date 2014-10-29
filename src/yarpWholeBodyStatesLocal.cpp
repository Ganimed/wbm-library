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

#include "yarpWholeBodyInterface/yarpWholeBodyStatesLocal.h"
#include "yarpWholeBodyInterface/yarpWholeBodySensors.h"
#include <iCub/skinDynLib/common.h>

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/ResourceFinder.h>

#include <string>
#include <iostream>
#include <yarp/os/Log.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <kdl/frames_io.hpp>

#define INITIAL_TIMESTAMP -1000.0


using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

/// < \todo TODO make it a proper parameter
#define ESTIMATOR_PERIOD 10


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyStatesLocal::yarpWholeBodyStatesLocal(const char* _name,
                                                   yarp::os::Property & _wbi_yarp_conf)
{
    sensors = new yarpWholeBodySensors(_name, _wbi_yarp_conf);              // sensor interface
    skin_contacts_port = new yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;
    skin_contacts_port->open(string("/"+string(_name)+"/skin_contacts:i").c_str());
    estimator = new yarpWholeBodyDynamicsEstimator(ESTIMATOR_PERIOD, sensors, skin_contacts_port, _wbi_yarp_conf);  // estimation thread
}

bool yarpWholeBodyStatesLocal::init()
{
    bool ok = sensors->init();              // initialize sensor interface
    if( !ok ) { std::cerr << "yarpWholeBodyStatesLocal::init() failed: error in sensor initialization." << std::endl; return false; }
    ok = estimator->start();
    if( !ok ) { std::cerr << "yarpWholeBodyStatesLocal::init() failed: error in estimator initialization." << std::endl; return false; }
    return ok; // start estimation thread
}

bool yarpWholeBodyStatesLocal::close()
{
    std::cout << "yarpWholeBodyStatesLocal::close() : closing estimator thread" << std::endl;
    if(estimator) estimator->stop();  // stop estimator BEFORE closing sensor interface
    std::cout << "yarpWholeBodyStatesLocal::close() : closing sensor interface" << std::endl;
    bool ok = (sensors ? sensors->close() : true);
    std::cout << "yarpWholeBodyStatesLocal::close() : deleting sensor interface" << std::endl;
    if(sensors) { delete sensors; sensors = 0; }
    std::cout << "yarpWholeBodyStatesLocal::close() : deleting estimator thread" << std::endl;
    if(estimator) { delete estimator; estimator = 0; }
    return ok;
}

bool yarpWholeBodyStatesLocal::addEstimate(const EstimateType et, const wbiId &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndAddSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:             return lockAndAddSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

int yarpWholeBodyStatesLocal::addEstimates(const EstimateType et, const wbiIdList &sids)
{
    //\todo TODO properly handle dependencies
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensors(SENSOR_PWM, sids);
    case ESTIMATE_IMU:                      return lockAndAddSensors(SENSOR_IMU, sids);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndAddSensors(SENSOR_FORCE_TORQUE, sids);
    default: break;
    }
    return false;
}

bool yarpWholeBodyStatesLocal::removeEstimate(const EstimateType et, const wbiId &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndRemoveSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndRemoveSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndRemoveSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

const wbiIdList& yarpWholeBodyStatesLocal::getEstimateList(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorList(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorList(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorList(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return emptyList;
}

int yarpWholeBodyStatesLocal::getEstimateNumber(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorNumber(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorNumber(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorNumber(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return 0;
}

bool yarpWholeBodyStatesLocal::getEstimate(const EstimateType et, const int numeric_id, double *data, double time, bool blocking)
{
    wbi::wbiId sid;
    switch(et)
    {
    case ESTIMATE_JOINT_POS:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:
        return estimator->lockAndCopyVectorElement(numeric_id,estimator->estimates.lastTauJ, data);
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
    case ESTIMATE_IMU:
        return estimator->lockAndCopyElementVectorFromVector(numeric_id, estimator->estimates.lastIMUs, data);
    case ESTIMATE_FORCE_TORQUE_SENSOR:
        return estimator->lockAndCopyElementVectorFromVector(numeric_id, estimator->estimates.lastForceTorques, data);
    case ESTIMATE_EXTERNAL_FORCE_TORQUE:
        this->getEstimateList(wbi::ESTIMATE_EXTERNAL_FORCE_TORQUE).numericIdToWbiId(numeric_id,sid);
        return estimator->lockAndCopyExternalForceTorque(sid,data);
    default: break;
    }
    return false;
}

bool yarpWholeBodyStatesLocal::getEstimates(const EstimateType et, double *data, double time, bool blocking)
{
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
    case ESTIMATE_IMU:                      return estimator->lockAndCopyVectorOfVectors(estimator->estimates.lastIMUs, data);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return estimator->lockAndCopyVectorOfVectors(estimator->estimates.lastForceTorques, data);
    default: break;
    }
    return false;
}

bool yarpWholeBodyStatesLocal::setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    return estimator->lockAndSetEstimationParameter(et, ep, value);
}

bool yarpWholeBodyStatesLocal::setEstimationOffset(const EstimateType et, const wbiId & sid, const double *value)
{
    return estimator->lockAndSetEstimationOffset(et,sid,value);
}

bool yarpWholeBodyStatesLocal::getEstimationOffset(const EstimateType et, const wbiId & sid, double *value)
{
    return estimator->lockAndGetEstimationOffset(et,sid,value);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                    IMPLEMENTATION SPECIFIC METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************


bool yarpWholeBodyStatesLocal::getEstimatedExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    return lockAndReadExternalForces(external_forces_list);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          PRIVATE METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************

bool yarpWholeBodyStatesLocal::getMotorVel(double *data, double time, bool blocking)
{
    bool res = estimator->lockAndCopyVector(estimator->estimates.lastDq, data);    ///< read joint vel
    if(!res) return false;
    wbiIdList idList = lockAndGetSensorList(SENSOR_ENCODER);
    //int i=0;
    /*
     \todo TODO FIXME
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
    }*/
    return true;
}

bool yarpWholeBodyStatesLocal::getMotorVel(const int numeric_id, double *data, double time, bool blocking)
{
    ///< read joint vel
    return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDq, data);
}

bool yarpWholeBodyStatesLocal::lockAndReadSensors(const SensorType st, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensors(st, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool yarpWholeBodyStatesLocal::lockAndReadSensor(const SensorType st, const int numeric_id, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensor(st, numeric_id, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool yarpWholeBodyStatesLocal::lockAndReadExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    estimator->mutex.wait();
    external_forces_list = estimator->estimatedLastSkinDynContacts;
    estimator->mutex.post();
    return true;
}

bool yarpWholeBodyStatesLocal::lockAndAddSensor(const SensorType st, const wbiId &sid)
{
    estimator->mutex.wait();
    bool res = sensors->addSensor(st, sid);
    estimator->mutex.post();
    return res;
}

int yarpWholeBodyStatesLocal::lockAndAddSensors(const SensorType st, const wbiIdList &sids)
{
    estimator->mutex.wait();
    int res = sensors->addSensors(st, sids);
    estimator->mutex.post();
    return res;
}

bool yarpWholeBodyStatesLocal::lockAndRemoveSensor(const SensorType st, const wbiId &sid)
{
    estimator->mutex.wait();
    bool res = sensors->removeSensor(st, sid);
    estimator->mutex.post();
    return res;
}

wbiIdList yarpWholeBodyStatesLocal::lockAndGetSensorList(const SensorType st)
{
    estimator->mutex.wait();
    wbiIdList res = sensors->getSensorList(st);
    estimator->mutex.post();
    return res;
}

int yarpWholeBodyStatesLocal::lockAndGetSensorNumber(const SensorType st)
{
    estimator->mutex.wait();
    int res = sensors->getSensorNumber(st);
    estimator->mutex.post();
    return res;
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY DYNAMICS ESTIMATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyDynamicsEstimator::yarpWholeBodyDynamicsEstimator(int _period,
                                                               yarpWholeBodySensors *_sensors,
                                                               yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> * _port_skin_contacts,
                                                               yarp::os::Property & _wbi_yarp_conf
                                                              )
: RateThread(_period),
   sensors(_sensors),
   port_skin_contacts(_port_skin_contacts),
   dqFilt(0), d2qFilt(0),
   enable_omega_domega_IMU(false),
   min_taxel(0),
   wbi_yarp_conf(_wbi_yarp_conf)
{

    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
    resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

    ///< Window lengths of adaptive window filters
    dqFiltWL            = 16;
    d2qFiltWL           = 25;
    dTauJFiltWL         = 30;
    dTauMFiltWL         = 30;
    imuAngularAccelerationFiltWL = 25;

    ///< Threshold of adaptive window filters
    dqFiltTh            = 1.0;
    d2qFiltTh           = 1.0;
    dTauJFiltTh         = 0.2;
    dTauMFiltTh         = 0.2;
    imuAngularAccelerationFiltTh = 1.0;

    ///< Cut frequencies
    tauJCutFrequency    =   3.0;
    tauMCutFrequency    =   3.0;
    pwmCutFrequency     =   3.0;
    imuLinearAccelerationCutFrequency = 3.0;
    imuAngularVelocityCutFrequency    = 3.0;
    imuMagnetometerCutFrequency       = 3.0;
    forcetorqueCutFrequency           = 3.0;

    ///< Skin timestamp
    last_reading_skin_contact_list_Stamp = -1000.0;

    if( _wbi_yarp_conf.check("fixed_base") )
    {
        assume_fixed_base = true;
        std::string _fixed_link;
        _fixed_link = _wbi_yarp_conf.find("fixed_base").asString();
        if( _fixed_link == "root_link" )
        {
            fixed_link = FIXED_ROOT_LINK;
        }
        else if( _fixed_link == "l_sole" )
        {
            fixed_link = FIXED_L_SOLE;
        }
        else if( _fixed_link == "r_sole" )
        {
            fixed_link = FIXED_R_SOLE;
        }
        else
        {
            YARP_ASSERT(false);
        }
    }
    else
    {
        assume_fixed_base = false;
    }

    if( assume_fixed_base )
    {

    }
}

bool yarpWholeBodyDynamicsEstimator::threadInit()
{
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
    resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

    ///< create derivative filters
    dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
    d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
    dTauJFilt = new AWLinEstimator(dTauJFiltWL, dTauJFiltTh);
    dTauMFilt = new AWLinEstimator(dTauMFiltWL, dTauMFiltTh);
    ///< read sensors
    bool ok = sensors->readSensors(SENSOR_ENCODER, estimates.lastQ.data(), qStamps.data(), true);
    ok = ok && sensors->readSensors(SENSOR_PWM, estimates.lastPwm.data(), 0, true);

    ///< create low pass filters
    tauJFilt    = new FirstOrderLowPassFilter(tauJCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    tauMFilt    = new FirstOrderLowPassFilter(tauMCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    pwmFilt     = new FirstOrderLowPassFilter(pwmCutFrequency, getRate()*1e-3, estimates.lastPwm);

    wbiIdList available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
    for(int ft_numeric = 0; ft_numeric < (int)available_ft_sensors.size(); ft_numeric++ )
    {
        wbiId wbi_id;
        available_ft_sensors.numericIdToWbiId(ft_numeric,wbi_id);
        int ft_index = ft_numeric;
        sensors->readSensor(SENSOR_FORCE_TORQUE, ft_index, forcetorques[ft_index].data(), &(forcetorquesStamps[ft_index]),true );
        forcetorqueFilters[ft_index] = new FirstOrderLowPassFilter(forcetorqueCutFrequency,getRate()*1e-3,forcetorques[ft_index]); ///< low pass filter
    }

     wbiIdList available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
     for(int numeric_imu_id = 0; numeric_imu_id < (int)available_imu_sensors.size(); numeric_imu_id++)
     {
         int imu_index = numeric_imu_id;
         //std::cout << "readSensor for IMU " << imu_index << std::endl;
         assert((int)IMUs.size() > imu_index && (int)IMUs[imu_index].size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
         if( sensors->readSensor(SENSOR_IMU, numeric_imu_id, IMUs[imu_index].data(), &(IMUStamps[imu_index]),true ) ) {
             imuLinearAccelerationFilters[imu_index] = new FirstOrderLowPassFilter(imuLinearAccelerationCutFrequency,getRate()*1e-3,IMUs[imu_index].subVector(4,6));  ///< linear acceleration is filtered with a low pass filter
             imuAngularVelocityFilters[imu_index] = new FirstOrderLowPassFilter(imuAngularVelocityCutFrequency,getRate()*1e-3,IMUs[imu_index].subVector(7,9));  ///< angular velocity is filtered with a low pass filter
             imuMagnetometerFilters[imu_index] = new FirstOrderLowPassFilter(imuMagnetometerCutFrequency,getRate()*1e-3,IMUs[imu_index].subVector(10,12));  ///< magnetometer readings are filtered with a low pass filter
         } else {
             std::cout << "icubWholeBodyStates: Error in reading IMU, exiting" << std::endl;
             YARP_ASSERT(false);
         }
         //std::cout << "IMU measured " << std::endl;
         //std::cout << IMUs[imu_index].toString() << std::endl;
         //std::cout << "timestamp: " << IMUStamps[imu_index] << std::endl;
     }

     //Allocating a filter for angular acceleration estimation only for IMU used in iDynTree
    imuAngularAccelerationFilt = new AWLinEstimator(imuAngularAccelerationFiltWL, imuAngularAccelerationFiltTh);

    //Allocation model
    std::string fixed_link_name;
    if( assume_fixed_base )
    {
        switch( fixed_link )
        {
            case FIXED_ROOT_LINK:
                fixed_link_name = "root_link";
            break;
            case FIXED_R_SOLE:
                fixed_link_name = "r_sole";
            break;
            case FIXED_L_SOLE:
                fixed_link_name = "l_sole";
            break;
        }
    }

    if( !this->wbi_yarp_conf.check("urdf_file") )
    {
        std::cerr << "[ERR] yarpWholeBodyStatesLocal error: urdf_file not found in configuration files" << std::endl;
        return false;
    }

    std::string urdf_file = this->wbi_yarp_conf.find("urdf_file").asString().c_str();
    yarp::os::ResourceFinder rf;
    std::string urdf_file_path = rf.findFile(urdf_file.c_str());


    model_mutex.wait();
    {
        if( !assume_fixed_base )
        {
            robot_estimation_model = new iCub::iDynTree::iCubTree(urdf_file_path);
        } else {
            robot_estimation_model = new iCub::iDynTree::iCubTree(urdf_file_path,fixed_link_name);
        }
    }
    model_mutex.post();

    left_hand_link_id = "l_hand";
    right_hand_link_id = "r_hand";
    left_foot_link_id = "l_foot";
    right_foot_link_id = "r_foot";

    left_gripper_frame_id = "l_gripper";
    right_gripper_frame_id = "r_gripper";
    left_sole_frame_id = "l_sole";
    right_sole_frame_id = "r_sole";

    //Find end effector ids
    left_hand_link_idyntree_id = robot_estimation_model->getLinkIndex(left_hand_link_id.toString());
    YARP_ASSERT(left_hand_link_idyntree_id >= 0);
    right_hand_link_idyntree_id = robot_estimation_model->getLinkIndex(right_hand_link_id.toString());
    YARP_ASSERT(right_hand_link_idyntree_id >= 0);
    left_foot_link_idyntree_id = robot_estimation_model->getLinkIndex(left_foot_link_id.toString());
    YARP_ASSERT(left_foot_link_idyntree_id >= 0);
    right_foot_link_idyntree_id = robot_estimation_model->getLinkIndex(right_foot_link_id.toString());
    YARP_ASSERT(right_foot_link_idyntree_id >= 0);

    left_gripper_frame_idyntree_id = robot_estimation_model->getLinkIndex(left_gripper_frame_id.toString());
    YARP_ASSERT(left_gripper_frame_idyntree_id >= 0);
    right_gripper_frame_idyntree_id = robot_estimation_model->getLinkIndex(right_gripper_frame_id.toString());
    YARP_ASSERT(right_hand_link_idyntree_id >= 0);
    left_sole_frame_idyntree_id = robot_estimation_model->getLinkIndex(left_sole_frame_id.toString());
    YARP_ASSERT(left_sole_frame_idyntree_id >= 0);
    right_sole_frame_idyntree_id = robot_estimation_model->getLinkIndex(right_sole_frame_id.toString());
    YARP_ASSERT(right_sole_frame_idyntree_id >= 0);

    //Compatibility layer for hardcoded skin ids
    KDL::CoDyCo::TreePartition icub_partition = robot_estimation_model->getKDLUndirectedTree().getPartition();
    left_hand_link_old_id = icub_partition.getLocalLinkIndex(left_hand_link_idyntree_id);
    YARP_ASSERT(left_hand_link_old_id >= 0);
    right_hand_link_old_id = icub_partition.getLocalLinkIndex(right_hand_link_idyntree_id);
    YARP_ASSERT(right_hand_link_old_id >= 0);
    left_foot_link_old_id = icub_partition.getLocalLinkIndex(left_foot_link_idyntree_id);
    right_foot_link_old_id = icub_partition.getLocalLinkIndex(right_foot_link_idyntree_id);
    left_gripper_frame_old_id = icub_partition.getLocalLinkIndex(left_gripper_frame_idyntree_id);
    right_gripper_frame_old_id = icub_partition.getLocalLinkIndex(right_gripper_frame_idyntree_id);
    left_sole_frame_old_id = icub_partition.getLocalLinkIndex(left_sole_frame_idyntree_id);
    right_sole_frame_old_id = icub_partition.getLocalLinkIndex(right_sole_frame_idyntree_id);


    return ok;
}

void yarpWholeBodyDynamicsEstimator::run()
{
    run_mutex.wait();
    //Temporary workaround: yarpWholeBodyStatesLocal needs all the DOF present in the dynamical model
    if( sensors->getSensorNumber(wbi::SENSOR_ENCODER) != robot_estimation_model->getNrOfDOFs() )
    {
        wbiIdList list = sensors->getSensorList(wbi::SENSOR_ENCODER);

        std::cerr << "Available sensors: " << list.toString() << std::endl;

           std::cerr << "yarpWholeBodyDynamicsEstimator::run() error: " <<
                  sensors->getSensorNumber(wbi::SENSOR_ENCODER) << " joint sensors are available, while  " <<
                  robot_estimation_model->getNrOfDOFs() << " joints are present in the model " << std::endl;
        assert(false);
        return;
    }

    ///< \todo improve robustness: what if a sensor dies or stop working? interface should warn the user
    mutex.wait();
    {
        resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
        resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
        resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

        ///< Read encoders
        if(sensors->readSensors(SENSOR_ENCODER, q.data(), qStamps.data(), false))
        {
            estimates.lastQ = q;
            AWPolyElement el;
            el.data = q;
            el.time = qStamps[0];
            estimates.lastDq = dqFilt->estimate(el);
            estimates.lastD2q = d2qFilt->estimate(el);

        }

        ///< Read force/torque sensors
        ///< \todo TODO buffer value of available_ft_sensors to avoid memory allocation (?)
        wbiIdList available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
        for(int ft_numeric = 0; ft_numeric < (int)available_ft_sensors.size(); ft_numeric++ )
        {
            int ft_index = ft_numeric;
            if( sensors->readSensor(SENSOR_FORCE_TORQUE, ft_numeric, forcetorques[ft_index].data(), &(forcetorquesStamps[ft_index]),false ) ) {
                estimates.lastForceTorques[ft_index] = forcetorqueFilters[ft_index]->filt(forcetorques[ft_index]); ///< low pass filter
                estimates.lastForceTorques[ft_index] = estimates.lastForceTorques[ft_index] - forcetorques_offset[ft_index]; /// remove offset
            } else {
                std::cout << "yarpWholeBodyStatesLocal: Error in reading F/T sensors, exiting" << std::endl;
                YARP_ASSERT(false);
            }
        }

        ///< Read IMU
        ///< \todo TODO buffer value of available_imu_sensors to avoid memory allocation (?)
        ///< \todo TODO add filters for imu values ->
        wbiIdList available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
        for(int imu_numeric = 0; imu_numeric < (int) available_ft_sensors.size(); imu_numeric++ )
        {
            int imu_index = imu_numeric;
            //std::cout << "readSensor for IMU " << imu_index << std::endl;
            assert((int)IMUs.size() > imu_index );
            assert((int)IMUs[imu_index].size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
            if( sensors->readSensor(SENSOR_IMU, imu_numeric, IMUs[imu_index].data(), &(IMUStamps[imu_index]),false ) ) {
                estimates.lastIMUs[imu_index].setSubvector(0,IMUs[imu_index].subVector(0,3)); ///< orientation is simply copied as already result of an estimation
                estimates.lastIMUs[imu_index].setSubvector(4,imuLinearAccelerationFilters[imu_index]->filt(IMUs[imu_index].subVector(4,6)));  ///< linear acceleration is filtered with a low pass filter
                estimates.lastIMUs[imu_index].setSubvector(7,imuAngularVelocityFilters[imu_index]->filt(IMUs[imu_index].subVector(7,9)));  ///< angular velocity is filtered with a low pass filter
                estimates.lastIMUs[imu_index].setSubvector(10,imuMagnetometerFilters[imu_index]->filt(IMUs[imu_index].subVector(10,12))); ///< magnetometer readings are filtered with a low pass filter
            } else {
                std::cout << "yarpWholeBodyStatesLocal: Error in reading IMU, exiting" << std::endl;
                YARP_ASSERT(false);
            }
            //std::cout << "IMU measured " << std::endl;
            //std::cout << IMUs[imu_index].toString() << std::endl;
            //std::cout << estimates.lastIMUs[imu_index].toString() << std::endl;
            //std::cout << "timestamp: " << IMUStamps[imu_index] << std::endl;
        }

        //Estimate angular acceleration only for the IMU used in iDynTree
        //std::cout << "Angular velocity used for acceleration estimation " <<  estimates.lastIMUs[0].subVector(7,9).toString() << std::endl;
        AWPolyElement el;
        el.data = omega_used_IMU = estimates.lastIMUs[0].subVector(7,9);
        el.time = IMUStamps[0];

        domega_used_IMU = imuAngularAccelerationFilt->estimate(el);

        ddp_used_IMU = estimates.lastIMUs[0].subVector(4,6);

        /*
        std::cout << "domega " <<  domega_used_IMU.toString() << std::endl;
        std::cout << "omega  " << omega_used_IMU.toString() << std::endl;
        std::cout << "ddp " << ddp_used_IMU.toString() << std::endl;
        */

        ///< Read skin contacts
        readSkinContacts();

        ///< Estimate joint torque sensors from force/torque measurements
        estimateExternalForcesAndJointTorques();

        ///< Filter obtained joint torque measures
        {
            // @todo Convert joint torques into motor torques
            AWPolyElement el;
            el.time = yarp::os::Time::now();

            estimates.lastTauJ = tauJ;// tauJFilt->filt(tauJ);  ///< low pass filter
            estimates.lastTauM = tauMFilt->filt(tauJ);  ///< low pass filter

            el.data = tauJ;
            estimates.lastDtauJ = dTauJFilt->estimate(el);  ///< derivative filter

            //el.data = tauM;
            estimates.lastDtauM = dTauMFilt->estimate(el);  ///< derivative filter
        }

        ///< Read motor pwm
        sensors->readSensors(SENSOR_PWM, pwm.data(),0, false);
        estimates.lastPwm = pwmFilt->filt(pwm);     ///< low pass filter
    }
    mutex.post();

    run_mutex.post();

    return;
}

void yarpWholeBodyDynamicsEstimator::readSkinContacts()
{
    skinContactList *scl = port_skin_contacts->read(false);
    if(scl)
    {
        //< \todo TODO check for envelope
        last_reading_skin_contact_list_Stamp = Time::now();
        if(scl->empty())   // if no skin contacts => leave the old contacts but reset pressure and contact list
        {
            //< \todo TODO this (using the last contacts if no contacts are detected) should be at subtree level, not at global level
            for(skinContactList::iterator it=skinContacts.begin(); it!=skinContacts.end(); it++)
            {
                it->setPressure(0.0);
                it->setActiveTaxels(0);
            }
            return;
        }

        //Probably source of crazy inefficiencies, here just to reach a working state as soon as possible \todo TODO
        map<BodyPart, skinContactList> contactsPerBp = scl->splitPerBodyPart();

        skinContacts.clear();

        // if there are more than 1 contact and less than 10 taxels are active then suppose zero moment
        for(map<BodyPart,skinContactList>::iterator it=contactsPerBp.begin(); it!=contactsPerBp.end(); it++)
        {
            if(it->second.size()>1)
            {
                for(skinContactList::iterator c=it->second.begin(); c!=it->second.end(); c++)
                {
                    if( c->getActiveTaxels()<10 )
                    {
                        c->fixMoment();
                    }

                    //Insert a contact in skinContacts only if the number of taxel is greater than ActiveTaxels
                    if( (int)c->getActiveTaxels() > min_taxel )
                    {
                        skinContacts.insert(skinContacts.end(),*c);
                    }
                }
            }
        }

        //TODO \todo add other parts
        /*
        skinContacts = contactsPerBp[LEFT_ARM];
        skinContacts.insert(skinContacts.end(), contactsPerBp[RIGHT_ARM].begin(), contactsPerBp[RIGHT_ARM].end());
        skinContacts.insert(skinContacts.end(), contactsPerBp[TORSO].begin(), contactsPerBp[TORSO].end());
        */
        //skinContacts.insert(skinContacts.end(), contactsPerBp[LEFT_LEG].begin(), contactsPerBp[LEFT_LEG].end());
        //skinContacts.insert(skinContacts.end(), contactsPerBp[RIGHT_LEG].begin(), contactsPerBp[RIGHT_LEG].end());
    }
    else if(Time::now()-last_reading_skin_contact_list_Stamp>SKIN_EVENTS_TIMEOUT && last_reading_skin_contact_list_Stamp!=0.0)
    {
        // if time is up, use default contact points \todo TODO
        skinContacts.clear();
    }

    //At this point, in a way or the other skinContacts must have at least a valid contact for each subtree
    //If this is not true, we add a default contact for each subgraph
    map<BodyPart, skinContactList> contactsPerBp = skinContacts.splitPerBodyPart();

    dynContacts = skinContacts.toDynContactList();

    //Ugly, but if we depend on skinContact data structure we have to do in this way
    //default contact torso
    if( contactsPerBp[TORSO].size() == 0 ) {
        dynContacts.push_back(getDefaultContact(TORSO_SUBTREE));
    }

    if( contactsPerBp[RIGHT_ARM].size() == 0 ) {
        dynContacts.push_back(getDefaultContact(RIGHT_ARM_SUBTREE));
    }

    if( contactsPerBp[LEFT_ARM].size() == 0 ) {
        dynContacts.push_back(getDefaultContact(LEFT_ARM_SUBTREE));
    }

    if( contactsPerBp[RIGHT_LEG].size() == 0 ) {
        /// \todo TODO handle v1 and v2 legs
        dynContacts.push_back(getDefaultContact(RIGHT_LEG_SUBTREE));
        dynContacts.push_back(getDefaultContact(RIGHT_FOOT_SUBTREE));
    }

    if( contactsPerBp[LEFT_LEG].size() == 0 ) {
        /// \todo TODO handle v1 and v2 legs
        dynContacts.push_back(getDefaultContact(LEFT_LEG_SUBTREE));
        dynContacts.push_back(getDefaultContact(LEFT_FOOT_SUBTREE));
    }

}



dynContact yarpWholeBodyDynamicsEstimator::getDefaultContact(const iCubSubtree icub_subtree)
{
    dynContact return_value;
    Vector r_knee_pos(3,0.0);
    r_knee_pos[0] = 0.18;
    r_knee_pos[1] = 0.04;
    r_knee_pos[2] = 0.0;
    Vector l_knee_pos(3,0.0);
    l_knee_pos[0] = 0.18;
    l_knee_pos[1] = 0.04;
    l_knee_pos[2] = 0.0;
    switch( icub_subtree ) {
        case RIGHT_ARM_SUBTREE:
            //Copied by iDynContactSolver::computeExternalContacts
            return_value = dynContact(RIGHT_ARM,6,Vector(3,0.0));
            break;
        case LEFT_ARM_SUBTREE:
            //Copied by iDynContactSolver::computeExternalContacts
            return_value = dynContact(LEFT_ARM,6,Vector(3,0.0));
            break;
        case TORSO_SUBTREE:
            // \todo TODO if floating base, use dynContact(TORSO,0,Vector(3,0.0))
            return_value = dynContact(TORSO,0,Vector(3,0.0));
            break;
        case RIGHT_LEG_SUBTREE:
            return_value = dynContact(RIGHT_LEG,3,r_knee_pos); //Approximate position of the knee
            break;
        case LEFT_LEG_SUBTREE:
            return_value = dynContact(LEFT_LEG,3,l_knee_pos);  //Approximate position of the knee
            break;
        case RIGHT_FOOT_SUBTREE:
            //Copied by wholeBodyDynamics run() method
            return_value = dynContact(RIGHT_LEG,5,Vector(3,0.0));
            break;
        case LEFT_FOOT_SUBTREE:
            //Copied by wholeBodyDynamics run() method
            return_value = dynContact(LEFT_LEG,5,Vector(3,0.0));
            break;
        default:
            break;
    }
    return return_value;
}

void getEEWrench(const iCub::iDynTree::iCubTree & icub_model,
                 const iCub::skinDynLib::dynContact & dyn_contact,
                 bool & contact_found,
                 yarp::sig::Vector & link_wrench,
                 yarp::sig::Vector & gripper_wrench,
                 int ee_frame_idyntree_id,
                 int link_idyntree_id)
{
    //std::cout << "getEEWRench " << std::endl;
    contact_found = true;
    KDL::Wrench f_gripper, f_link, f_contact;
    KDL::Vector COP;
    YarptoKDL(dyn_contact.getCoP(),COP);
    YarptoKDL(dyn_contact.getForceMoment(),f_contact);
    //std::cout << "f_contact " << f_contact << std::endl;
    KDL::Frame H_link_contact(COP);
    f_link = H_link_contact*f_contact;
    KDLtoYarp(f_link,link_wrench);
    yarp::sig::Matrix H_gripper_link_yarp = icub_model.getPosition(ee_frame_idyntree_id,link_idyntree_id);
    YARP_ASSERT(H_gripper_link_yarp.cols() == 4 &&
                H_gripper_link_yarp.rows() == 4);
    KDL::Frame H_gripper_link;
    YarptoKDL(H_gripper_link_yarp,H_gripper_link);
    f_gripper = H_gripper_link*f_link;
    KDLtoYarp(f_gripper,gripper_wrench);
}

wbi::wbiId yarpWholeBodyDynamicsEstimator::linkOldIdToNewId(const int bodyPart, const int link_index)
{
    // \todo TODO FIXME Remove hardcoded values
    if( bodyPart == LEFT_ARM && link_index == left_hand_link_old_id )
    {
        return left_hand_link_id;
    }
    if( bodyPart == RIGHT_ARM && link_index == right_hand_link_old_id )
    {
        return right_hand_link_id;
    }
    if( bodyPart == LEFT_LEG && link_index == left_foot_link_old_id )
    {
        return left_foot_link_id;
    }
    if( bodyPart == RIGHT_LEG && link_index == right_foot_link_old_id )
    {
        return right_foot_link_id;
    }
    return wbiId();
}

void yarpWholeBodyDynamicsEstimator::estimateExternalForcesAndJointTorques()
{
    //Assume that only a IMU is available

    /** \todo TODO check that serialization between wbi and iDynTree are the same */
    assert(omega_used_IMU.size() == 3);
    assert(domega_used_IMU.size() == 3);
    assert(ddp_used_IMU.size() == 3);

    if( !enable_omega_domega_IMU )
    {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
    }

    double gravity = 9.8;

    if( assume_fixed_base )
    {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
        ddp_used_IMU.zero();
        switch( fixed_link )
        {
            case FIXED_ROOT_LINK:
                ddp_used_IMU[2] = gravity;
            break;
            case FIXED_L_SOLE:
            case FIXED_R_SOLE:
                ddp_used_IMU[0] = gravity;
            break;
        }
    }

    model_mutex.wait();
    assert((int)estimates.lastQ.size() == robot_estimation_model->getNrOfDOFs());
    assert((int)estimates.lastDq.size() == robot_estimation_model->getNrOfDOFs());
    assert((int)estimates.lastD2q.size() == robot_estimation_model->getNrOfDOFs());

    YARP_ASSERT(robot_estimation_model->setInertialMeasure(omega_used_IMU,domega_used_IMU,ddp_used_IMU));
    (robot_estimation_model->setAng(estimates.lastQ));
    (robot_estimation_model->setDAng(estimates.lastDq));
    (robot_estimation_model->setD2Ang(estimates.lastD2q));
    for(int i=0; i < robot_estimation_model->getNrOfFTSensors(); i++ ) {
        //std::cout << "Number of F/T sensors available " << estimates.lastForceTorques.size() << std::endl;
        //std::cout << "Number of F/T sensors required by the model " << icub_model->getNrOfFTSensors() << std::endl;
        YARP_ASSERT((int)estimates.lastForceTorques.size() == robot_estimation_model->getNrOfFTSensors());
        assert(estimates.lastForceTorques[i].size() == 6);
        YARP_ASSERT(robot_estimation_model->setSensorMeasurement(i,estimates.lastForceTorques[i]));
    }
    robot_estimation_model->setContacts(dynContacts);

    /** \todo TODO avoid unlocking/locking a mutex locked in the calling function in the called function */
    /** \todo TODO use a different mutex to ensure that the dimensions of the sensors/states does not change? */
    //mutex.post();

    YARP_ASSERT(robot_estimation_model->kinematicRNEA());
    YARP_ASSERT(robot_estimation_model->estimateContactForcesFromSkin());
    YARP_ASSERT(robot_estimation_model->dynamicRNEA());
    YARP_ASSERT(robot_estimation_model->computePositions());

    estimatedLastDynContacts = robot_estimation_model->getContacts();

    //Create estimatedLastSkinDynContacts using original skinContacts list read from skinManager
    // for each dynContact find the related skinContact (if any) and set the wrench in it
    unsigned long cId;
    bool contactFound=false;

    bool raContactFound, laContactFound, rlContactFound, llContactFound;
    raContactFound = laContactFound = rlContactFound = llContactFound = false;

    for(unsigned int i=0; i < estimatedLastDynContacts.size(); i++)
    {
        //Workaround for bug in iCubGui
        if( estimatedLastDynContacts[i].getBodyPart() == TORSO &&
            estimatedLastDynContacts[i].getLinkNumber() == 2 )
        {
            //Invert second component
            yarp::sig::Vector wrench = estimatedLastDynContacts[i].getForceMoment();
            wrench[1] = -wrench[1];
            wrench[3+1] = -wrench[3+1];
            estimatedLastDynContacts[i].setForceMoment(wrench);
        }


        /*
        std::cout << "Found contact at " << estimatedLastDynContacts[i].getBodyPart() <<
                     " " << estimatedLastDynContacts[i].getLinkNumber() << std::endl;
        std::cout << "Left hand link id " << left_hand_link_id << std::endl;
        */
        cId = estimatedLastDynContacts[i].getId();
        for(unsigned int j=0; j<skinContacts.size(); j++)
        {
            if(cId == skinContacts[j].getId())
            {
                skinContacts[j].setForceMoment( estimatedLastDynContacts[i].getForceMoment() );
                contactFound = true;
                j = skinContacts.size();    // break from the inside for loop
            }
        }
        // if there is no associated skin contact, create one
        if(!contactFound)
            skinContacts.push_back(skinContact(estimatedLastDynContacts[i]));
        contactFound = false;

        wbiId contactLink = linkOldIdToNewId(estimatedLastDynContacts[i].getBodyPart(),estimatedLastDynContacts[i].getLinkNumber());

        //If a dyn contact is found on the end effector, store its value
        if( contactLink == left_hand_link_id )
        {
            getEEWrench(*robot_estimation_model,estimatedLastDynContacts[i],left_arm_ee_contact_found,
                        left_hand_ee_wrench,left_gripper_ee_wrench,left_gripper_frame_idyntree_id,left_hand_link_idyntree_id);
            laContactFound = true;
        }
        if( contactLink ==  right_hand_link_id )
        {
            getEEWrench(*robot_estimation_model,estimatedLastDynContacts[i],right_arm_ee_contact_found,
                        right_hand_ee_wrench,right_gripper_ee_wrench,right_gripper_frame_idyntree_id,right_hand_link_idyntree_id);
            raContactFound = true;
        }
        if( contactLink ==  left_foot_link_id )
        {
            getEEWrench(*robot_estimation_model,estimatedLastDynContacts[i],left_leg_ee_contact_found,
                        left_foot_ee_wrench,left_sole_ee_wrench,left_sole_frame_idyntree_id,left_foot_link_idyntree_id);
            llContactFound = true;
        }
        if( contactLink ==  right_foot_link_id )
        {
            getEEWrench(*robot_estimation_model,estimatedLastDynContacts[i],right_leg_ee_contact_found,
                        right_foot_ee_wrench,right_sole_ee_wrench,right_sole_frame_idyntree_id,right_foot_link_idyntree_id);
            rlContactFound = true;
        }
    }

    if( !laContactFound )
    {
        left_gripper_ee_wrench.zero();
        left_hand_ee_wrench.zero();
    }

    if( !raContactFound )
    {
        right_gripper_ee_wrench.zero();
        right_hand_ee_wrench.zero();
    }

    if( !llContactFound )
    {
        left_sole_ee_wrench.zero();
        left_foot_ee_wrench.zero();
    }

    if( !raContactFound )
    {
        right_sole_ee_wrench.zero();
        right_foot_ee_wrench.zero();
    }


    //mutex.wait();

    estimatedLastSkinDynContacts = skinContacts;

    assert((int)tauJ.size() == robot_estimation_model->getNrOfDOFs());
    tauJ = robot_estimation_model->getTorques();
    model_mutex.post();

}


void deleteFirstOrderFilterVector(std::vector<iCub::ctrl::FirstOrderLowPassFilter *> & vec)
{
    for(int i=0; i < (int)vec.size(); i++ ) {
        if( vec[i]!= 0 ) { delete vec[i]; vec[i]=0; }
    }
    vec.resize(0);
}

void yarpWholeBodyDynamicsEstimator::threadRelease()
{
    if(dqFilt!=0)    { delete dqFilt;  dqFilt=0; }
    if(d2qFilt!=0)   { delete d2qFilt; d2qFilt=0; }
    if(dTauJFilt!=0) { delete dTauJFilt; dTauJFilt=0; }
    if(dTauMFilt!=0) { delete dTauMFilt; dTauMFilt=0; }     // motor torque derivative filter
    if(tauJFilt!=0)  { delete tauJFilt; tauJFilt=0; }  ///< low pass filter for joint torque
    if(tauMFilt!=0)  { delete tauMFilt; tauMFilt=0; }  ///< low pass filter for motor torque
    if(pwmFilt!=0)   { delete pwmFilt; pwmFilt=0;   }
    deleteFirstOrderFilterVector(imuLinearAccelerationFilters);
    deleteFirstOrderFilterVector(imuAngularVelocityFilters);
    deleteFirstOrderFilterVector(imuMagnetometerFilters);
    deleteFirstOrderFilterVector(forcetorqueFilters);
    if(imuAngularAccelerationFilt!=0) { delete imuAngularAccelerationFilt; imuAngularAccelerationFilt=0; }
    return;
}

void yarpWholeBodyDynamicsEstimator::lockAndResizeAll(int n)
{
    mutex.wait();
    resizeAll(n);
    mutex.post();
}

void yarpWholeBodyDynamicsEstimator::resizeAll(int n)
{
    q.resize(n,0.0);
    qStamps.resize(n,INITIAL_TIMESTAMP);
    tauJ.resize(n,0.0);
    tauJStamps.resize(n,INITIAL_TIMESTAMP);
    pwm.resize(n,0);
    pwmStamps.resize(n,INITIAL_TIMESTAMP);
    estimates.lastQ.resize(n,0.0);
    estimates.lastDq.resize(n,0.0);
    estimates.lastD2q.resize(n,0.0);
    estimates.lastTauJ.resize(n,0.0);
    estimates.lastTauM.resize(n,0.0);
    estimates.lastDtauJ.resize(n,0.0);
    estimates.lastDtauM.resize(n,0.0);
    estimates.lastPwm.resize(n,0.0);
}

void yarpWholeBodyDynamicsEstimator::lockAndResizeFTs(int n)
{
    mutex.wait();
    resizeFTs(n);
    mutex.post();
}

void yarpWholeBodyDynamicsEstimator::resizeFTs(int n)
{
    forcetorques.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
    forcetorques_offset.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
    forcetorquesStamps.resize(n,INITIAL_TIMESTAMP);
    forcetorqueFilters.resize(n);
    estimates.lastForceTorques.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
}

void yarpWholeBodyDynamicsEstimator::lockAndResizeIMUs(int n)
{
    mutex.wait();
    resizeIMUs(n);
    mutex.post();
}

void yarpWholeBodyDynamicsEstimator::resizeIMUs(int n)
{
    IMUs.resize(n,Vector(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0));
    IMUStamps.resize(n,INITIAL_TIMESTAMP);
    imuAngularVelocityFilters.resize(n);
    imuLinearAccelerationFilters.resize(n);
    imuMagnetometerFilters.resize(n);
    estimates.lastIMUs.resize(n,Vector(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0));
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyVector(const Vector &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    memcpy(dest, src.data(), sizeof(double)*src.size());
    mutex.post();
    return true;
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyVectorElement(int index, const Vector &src, double *dest)
{
    mutex.wait();
    dest[0] = src[index];
    mutex.post();
    return true;
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyElementVectorFromVector(int index, const std::vector<Vector> &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    memcpy(dest,src[index].data(),sizeof(double)*src[index].size());
    mutex.post();
    return true;
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyVectorOfVectors(const std::vector<Vector> &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    for(int i=0, offset = 0; i < (int)src.size(); i++) {
        memcpy(dest+offset,src[i].data(),sizeof(double)*src[i].size());
        offset += src[i].size();
    }
    mutex.post();
    return true;
}

void copyVector(const yarp::sig::Vector & src, double * dest)
{
    memcpy(dest,src.data(),src.size()*sizeof(double));
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyExternalForceTorque(const wbiId & sid, double * dest)
{
    bool external_ft_available = false;
    if(dest==0)
    {
        return false;
    }
    if( sid == left_hand_link_id )
    {
        copyVector(left_hand_ee_wrench,dest);
        external_ft_available = true;
    }
    else if ( sid == left_gripper_frame_id )
    {
        copyVector(left_gripper_ee_wrench,dest);
        external_ft_available = true;
    }
    else if( sid == right_hand_link_id )
    {
        copyVector(right_hand_ee_wrench,dest);
        external_ft_available = true;
    }
    else if ( sid == right_gripper_frame_id )
    {
        copyVector(right_gripper_ee_wrench,dest);
        external_ft_available = true;
    }
    else if( sid == left_foot_link_id )
    {
        copyVector(left_foot_ee_wrench,dest);
        external_ft_available = true;
    }
    else if ( sid == left_sole_frame_id )
    {
        copyVector(left_sole_ee_wrench,dest);
        external_ft_available = true;
    }
    else if( sid == right_foot_link_id )
    {
        copyVector(right_foot_ee_wrench,dest);
        external_ft_available = true;
    }
    else if ( sid == right_sole_frame_id )
    {
        copyVector(right_sole_ee_wrench,dest);
        external_ft_available = true;
    }
    return external_ft_available;
}

bool yarpWholeBodyDynamicsEstimator::lockAndSetEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
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
        else if(ep==wbi::ESTIMATION_PARAM_ENABLE_OMEGA_IMU_DOMEGA_IMU)
            res = setEnableOmegaDomegaIMU(*((bool*)value));
        else if(ep==wbi::ESTIMATION_PARAM_MIN_TAXEL)
            res = setMinTaxel(*((int*)value));
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

    case ESTIMATE_IMU: ///< \todo TODO
    case ESTIMATE_FORCE_TORQUE_SENSOR: ///< \todo TODO
    case ESTIMATE_JOINT_POS:
    case ESTIMATE_MOTOR_POS:
    default: break;
    }
    mutex.post();
    return res;
}

bool yarpWholeBodyDynamicsEstimator::lockAndSetEstimationOffset(const EstimateType et, const wbiId & sid, const double *value)
{
    bool res = true;
    int ft_index;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_FORCE_TORQUE_SENSOR: ///< \todo TODO
        sensors->getSensorList(SENSOR_FORCE_TORQUE).wbiIdToNumericId(sid,ft_index);
        memcpy(forcetorques_offset[ft_index].data(), (double*)value, sizeof(double)*sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize);
        break;
    default:
        break;
    }
    mutex.post();
    return res;
}

bool yarpWholeBodyDynamicsEstimator::lockAndGetEstimationOffset(const EstimateType et, const wbiId & sid, double *value)
{
    bool res = true;
    int ft_index;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_FORCE_TORQUE_SENSOR: ///< \todo TODO
        sensors->getSensorList(SENSOR_FORCE_TORQUE).wbiIdToNumericId(sid,ft_index);
        memcpy(value, forcetorques_offset[ft_index].data(), sizeof(double)*sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize);
        break;
    default:
        break;
    }
    mutex.post();
    return res;
}



bool yarpWholeBodyDynamicsEstimator::setVelFiltParams(int windowLength, double threshold)
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

bool yarpWholeBodyDynamicsEstimator::setAccFiltParams(int windowLength, double threshold)
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

bool yarpWholeBodyDynamicsEstimator::setDtauJFiltParams(int windowLength, double threshold)
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

bool yarpWholeBodyDynamicsEstimator::setDtauMFiltParams(int windowLength, double threshold)
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

bool yarpWholeBodyDynamicsEstimator::setTauJCutFrequency(double fc)
{
    return tauJFilt->setCutFrequency(fc);
}

bool yarpWholeBodyDynamicsEstimator::setTauMCutFrequency(double fc)
{
    return tauMFilt->setCutFrequency(fc);
}

bool yarpWholeBodyDynamicsEstimator::setPwmCutFrequency(double fc)
{
    return pwmFilt->setCutFrequency(fc);
}

bool yarpWholeBodyDynamicsEstimator::setEnableOmegaDomegaIMU(bool _enabled_omega_domega_IMU)
{
    enable_omega_domega_IMU = _enabled_omega_domega_IMU;
    return true;
}

bool yarpWholeBodyDynamicsEstimator::setMinTaxel(const int _min_taxel)
{
    if( _min_taxel < 0 )
    {
        return false;
    }
    min_taxel = _min_taxel;
    return true;
}
