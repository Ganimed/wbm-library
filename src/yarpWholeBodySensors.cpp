/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email: marco.randazzo@iit.it
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

#include "yarpWholeBodyInterface/yarpWholeBodySensors.h"
#include "yarpWholeBodyInterface/yarpWbiUtil.h"

#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <string>
#include <sstream>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

#define MAX_NJ 20 ///< Maximum number of joints for body part (used for buffers to avoid dynamic memory allocation)
#define WAIT_TIME 0.001
#define INITIAL_TIMESTAMP -1000.0

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY SENSORS
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodySensors::yarpWholeBodySensors(const char* _name, const yarp::os::Property & opt):
initDone(false), name(_name), wbi_yarp_properties(opt)
{
    /*
    loadBodyPartsFromConfig(opt,controlBoardNames);
    loadReverseTorsoJointsFromConfig(opt,reverse_torso_joints);
    loadFTSensorPortsFromConfig(opt,controlBoardNames,ftSens_2_port);
    loadIMUSensorPortsFromConfig(opt,controlBoardNames,imu_2_port);
    */
}

bool yarpWholeBodySensors::setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties)
{
    wbi_yarp_properties = yarp_wbi_properties;
    return true;
}

bool yarpWholeBodySensors::getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties)
{
    yarp_wbi_properties = wbi_yarp_properties;
    return true;
}

bool yarpWholeBodySensors::init()
{
    if( initDone ) return true;

    //Get encoders
    #ifndef NDEBUG
    std::cout << "yarpWholeBodySensors: initializing with " << encoderIdList.size() << " encoders " <<
                                                               pwmSensIdList.size() << " pwm sensors " <<
                                                               ftSensIdList.size() << " F/T sensors " <<
                                                               imuIdList.size() << " IMUs " << std::endl;
    #endif

    if( !wbi_yarp_properties.check("robotName") )
    {
        std::cerr << "yarpWholeBodySensors error: robotName not found in configuration files" << std::endl;
        return false;
    }

    robot = wbi_yarp_properties.find("robotName").asString().c_str();

    yarp::os::Bottle & joints_config = getWBIYarpJointsOptions(wbi_yarp_properties);
    controlBoardNames.clear();
    initDone = appendNewControlBoardsToVector(joints_config,encoderIdList,controlBoardNames);
    initDone = initDone && appendNewControlBoardsToVector(joints_config,pwmSensIdList,controlBoardNames);
    initDone = initDone && appendNewControlBoardsToVector(joints_config,torqueSensorIdList,controlBoardNames);
    if( !initDone )
    {
        return false;
    }

    //Resize all the data structure that depend on the number of controlboards
    int nrOfControlBoards = controlBoardNames.size();
    ienc.resize(nrOfControlBoards);
    iopl.resize(nrOfControlBoards);
    dd.resize(nrOfControlBoards);
    itrq.resize(nrOfControlBoards);

    controlBoardAxes.resize(nrOfControlBoards);
    qLastRead.resize(nrOfControlBoards);
    qStampLastRead.resize(nrOfControlBoards);
    pwmLastRead.resize(nrOfControlBoards);
    torqueSensorsLastRead.resize(nrOfControlBoards);

    getControlBoardAxisList(joints_config,encoderIdList,controlBoardNames,encoderControlBoardAxisList);
    getControlBoardAxisList(joints_config,pwmSensIdList,controlBoardNames,pwmControlBoardAxisList);
    getControlBoardAxisList(joints_config,torqueSensorIdList,controlBoardNames,torqueControlBoardAxisList);

    encoderControlBoardList = getControlBoardList(encoderControlBoardAxisList);
    pwmControlBoardList     = getControlBoardList(pwmControlBoardAxisList);
    torqueControlBoardList  = getControlBoardList(torqueControlBoardAxisList);

    for(int i=0; i < (int)encoderControlBoardList.size(); i++ )
    {
        int ctrlBoard = encoderControlBoardList[i];
        initDone = initDone && openEncoder(ctrlBoard);
    }
    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening encoders." << std::endl;
        return false;
    }

    for(int i=0; i < (int)pwmControlBoardList.size(); i++ )
    {
        int ctrlBoard = pwmControlBoardList[i];
        initDone = initDone && openPwm(ctrlBoard);
    }

    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening motor pwm inputs." << std::endl;
        return false;
    }

    for(int i=0; i < (int)torqueControlBoardList.size(); i++ )
    {
        int ctrlBoard = torqueControlBoardList[i];
        initDone = initDone && openTorqueSensor(ctrlBoard);
    }

    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening torques sensors." << std::endl;
        return false;
    }


    //Load imu and ft sensors information
    std::vector<string> imu_ports, ft_ports;
    bool ret = loadFTSensorPortsFromConfig(wbi_yarp_properties,ftSensIdList,imu_ports);
    ret = ret && loadIMUSensorPortsFromConfig(wbi_yarp_properties,imuIdList,ft_ports);
    if( ! ret )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in loading configuration of IMU and FT sensors." << std::endl;
        return false;
    }

    //Resize all the data structure that depend on the number of fts
    int nrOfFtSensors = ftSensIdList.size();
    ftSensLastRead.resize(nrOfFtSensors);
    ftStampSensLastRead.resize(nrOfFtSensors);
    int nrOfImuSensors = imuIdList.size();
    imuLastRead.resize(nrOfImuSensors);
    imuStampLastRead.resize(nrOfImuSensors);

    for(int ft_numeric_id = 0; ft_numeric_id < (int)ftSensIdList.size(); ft_numeric_id++)
    {
            initDone = initDone && openFTsens(ft_numeric_id,ft_ports[ft_numeric_id]);
    }

    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening force/torque sensors." << std::endl;
        return false;
    }

    for(int imu_numeric_id = 0; imu_numeric_id < (int)imuIdList.size(); imu_numeric_id++)
    {
            initDone = initDone && openImu(imu_numeric_id,imu_ports[imu_numeric_id]);
    }

    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening imu sensors." << std::endl;
        return false;
    }

    return initDone;
}

bool yarpWholeBodySensors::close()
{
    bool ok = true;
    for(int i=0; i < (int)encoderControlBoardList.size(); i++ )
    {
        int ctrlBoard = encoderControlBoardList[i];
        assert(dd[ctrlBoard]!=NULL);
        ok = ok && dd[ctrlBoard]->close();
        delete dd[ctrlBoard];
        dd[ctrlBoard] = NULL;
    }

    for(int i=0; i < (int)pwmControlBoardList.size(); i++ )
    {
        int ctrlBoard = pwmControlBoardList[i];
        if(dd[ctrlBoard]!=NULL)
        {
            ok = ok && dd[ctrlBoard]->close();
            delete dd[ctrlBoard];
            dd[ctrlBoard] = NULL;
        }
    }

    for(int i=0; i < (int)torqueControlBoardList.size(); i++ )
    {
        int ctrlBoard = torqueControlBoardList[i];
        if(dd[ctrlBoard])
        {
            ok = ok && dd[ctrlBoard]->close();
            delete dd[ctrlBoard];
            dd[ctrlBoard] = NULL;
        }
    }

    for(std::vector<BufferedPort<Vector>*>::iterator it=portsIMU.begin(); it!=portsIMU.end(); it++)
    {
        if( *it != 0 ) {
            (*it)->close();
            delete *it;
            *it=0;
        }
    }

    for(std::vector<BufferedPort<Vector>*>::iterator it=portsFTsens.begin(); it!=portsFTsens.end(); it++)
    {
        if( *it != 0 ) {
            (*it)->close();
            delete *it;
            *it=0;
        }
    }

    return ok;
}

bool yarpWholeBodySensors::addSensor(const SensorType st, const wbiId &sid)
{
    if( initDone )
    {
        return false;
    }

    switch(st)
    {
    case SENSOR_ENCODER:        return addEncoder(sid);
    case SENSOR_PWM:            return addPwm(sid);
    case SENSOR_IMU:            return addIMU(sid);
    case SENSOR_FORCE_TORQUE:   return addFTsensor(sid);
    case SENSOR_TORQUE:         return addTorqueSensor(sid);
    default: break;
    }
    return false;
}

int yarpWholeBodySensors::addSensors(const SensorType st, const wbiIdList &sids)
{
    if( initDone )
    {
        return 0;
    }

    switch(st)
    {
    case SENSOR_ENCODER:        return addEncoders(sids);
    case SENSOR_PWM:            return addPwms(sids);
    case SENSOR_IMU:            return addIMUs(sids);
    case SENSOR_FORCE_TORQUE:   return addFTsensors(sids);
    case SENSOR_TORQUE:         return addTorqueSensors(sids);
    default: break;
    }
    return false;
}

bool yarpWholeBodySensors::removeSensor(const SensorType st, const wbiId &sid)
{
    return false;
}

const wbiIdList& yarpWholeBodySensors::getSensorList(const SensorType st)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return encoderIdList;
    case SENSOR_PWM:            return pwmSensIdList;
    case SENSOR_IMU:            return imuIdList;
    case SENSOR_FORCE_TORQUE:   return ftSensIdList;
    case SENSOR_TORQUE:         return torqueSensorIdList;
    default:break;
    }
    return emptyList;
}

int yarpWholeBodySensors::getSensorNumber(const SensorType st)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return encoderIdList.size();
    case SENSOR_PWM:            return pwmSensIdList.size();
    case SENSOR_IMU:            return imuIdList.size();
    case SENSOR_FORCE_TORQUE:   return ftSensIdList.size();
    case SENSOR_TORQUE:         return torqueSensorIdList.size();
    default: break;
    }
    return 0;
}

bool yarpWholeBodySensors::readSensor(const SensorType st, const int sid, double *data, double *stamps, bool blocking)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return readEncoder(sid, data, stamps, blocking);
    case SENSOR_PWM:            return readPwm(sid, data, stamps, blocking);
    case SENSOR_IMU:            return readIMU(sid, data, stamps, blocking);
    case SENSOR_FORCE_TORQUE:   return readFTsensor(sid, data, stamps, blocking);
    case SENSOR_TORQUE:         return readTorqueSensor(sid, data, stamps, blocking);
    default: break;
    }
    return false;
}

bool yarpWholeBodySensors::readSensors(const SensorType st, double *data, double *stamps, bool blocking)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return readEncoders(data, stamps, blocking);
    case SENSOR_PWM:            return readPwms(data, stamps, blocking);
    case SENSOR_IMU:            return readIMUs(data, stamps, blocking);
    case SENSOR_FORCE_TORQUE:   return readFTsensors(data, stamps, blocking);
    case SENSOR_TORQUE:         return readTorqueSensors(data, stamps, blocking);
    default: break;
    }
    return false;
}

/********************************************************************************************************************************************/
/**************************************************** PRIVATE METHODS ***********************************************************************/
/********************************************************************************************************************************************/

bool yarpWholeBodySensors::openEncoder(const int bp)
{
    // check whether the encoder interface is already open
    if(ienc[bp]!=0) return true;
    // check whether the poly driver is already open (here I assume the elements of dd are initialized to 0)
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], controlBoardNames[bp])) return false;
    // open the encoder interface
    if(!dd[bp]->view(ienc[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", controlBoardNames[bp].c_str());
        return false;
    }
    ///< store the number of joints in this body part
    int nj=0;
    ienc[bp]->getAxes(&nj);
    controlBoardAxes[bp] = nj;

    //allocate lastRead variables
    qLastRead[bp].resize(nj);
    qStampLastRead[bp].resize(nj);
    pwmLastRead[bp].resize(nj);
    torqueSensorsLastRead[bp].resize(nj);

    return true;
}

bool yarpWholeBodySensors::openPwm(const int bp)
{
    ///< check whether the motor PWM interface is already open
    if(iopl[bp]!=0)             return true;

    ///< if necessary open the poly driver
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], controlBoardNames[bp]))
    {
        return false;
    }

    if(!dd[bp]->view(iopl[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", controlBoardNames[bp].c_str());
        return false;
    }

    //allocate lastRead variables
    qLastRead[bp].resize(controlBoardAxes[bp]);
    qStampLastRead[bp].resize(controlBoardAxes[bp]);
    pwmLastRead[bp].resize(controlBoardAxes[bp]);
    torqueSensorsLastRead[bp].resize(controlBoardAxes[bp]);

    return true;
}

bool yarpWholeBodySensors::openImu(const int numeric_id, const std::string & port_name)
{
    if( numeric_id < 0 || numeric_id >= (int)imuIdList.size() )
    {
        return false;
    }

    string remotePort = "/" + robot + port_name;
    stringstream localPort;
    wbi::wbiId wbi_id;
    imuIdList.numericIdToWbiId(numeric_id,wbi_id);
    localPort << "/" << name << "/imu/" <<  wbi_id.toString() << ":i";
    portsIMU[numeric_id] = new BufferedPort<Vector>();
    if(!portsIMU[numeric_id]->open(localPort.str().c_str())) { // open local input port
        std::cerr << "yarpWholeBodySensors::openImu(): Open of localPort " << localPort << " failed " << std::endl;
        return false;
    }
    if(!Network::exists(remotePort.c_str())) {       // check remote output port exists
        std::cerr << "yarpWholeBodySensors::openImu():  " << remotePort << " does not exist " << std::endl;
        return false;
    }
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp", true)) {  // connect remote to local port
        std::cerr << "yarpWholeBodySensors::openImu():  could not connect " << remotePort << " to " << localPort << std::endl;
        return false;
    }

    //allocate lastRead variables
    imuLastRead[numeric_id].resize(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0);
    imuStampLastRead[numeric_id] = INITIAL_TIMESTAMP;

    return true;
}

bool yarpWholeBodySensors::openFTsens(const int ft_sens_numeric_id, const std::string & port_name)
{
    string remotePort = "/" + robot + port_name;
    stringstream localPort;
    wbi::wbiId wbi_id;
    ftSensIdList.numericIdToWbiId(ft_sens_numeric_id,wbi_id);
    localPort << "/" << name << "/ftSens/" << wbi_id.toString() << ":i";
    portsFTsens[ft_sens_numeric_id] = new BufferedPort<Vector>();
    if(!portsFTsens[ft_sens_numeric_id]->open(localPort.str().c_str())) {
        // open local input port
        std::cerr << "yarpWholeBodySensors::openFTsens(): Open of localPort " << localPort << " failed " << std::endl;
        return false;
    }
    if(!Network::exists(remotePort.c_str())) {            // check remote output port exists
        std::cerr << "yarpWholeBodySensors::openFTsens():  " << remotePort << " does not exist " << std::endl;
        return false;
    }
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp")) {  // connect remote to local port
        std::cerr << "yarpWholeBodySensors::openFTsens():  could not connect " << remotePort << " to " << localPort << std::endl;
        return false;
    }

    //allocate lastRead variables
    ftSensLastRead[ft_sens_numeric_id].resize(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0);
    ftStampSensLastRead[ft_sens_numeric_id] = INITIAL_TIMESTAMP;

    return true;
}

bool yarpWholeBodySensors::openTorqueSensor(const int bp)
{
    ///< check that we are not in simulation, because iCub simulator does not implement torque sensors
    if(isICubSimulator(robot))
        return true;

    ///< check whether the joint control interface is already open
    if(itrq[bp])
        return true;

    ///< if necessary open the poly driver
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], controlBoardNames[bp]))
        return false;

    if(!dd[bp]->view(itrq[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", controlBoardNames[bp].c_str());
        return false;
    }

    //allocate lastRead variables
    qLastRead[bp].resize(controlBoardAxes[bp]);
    qStampLastRead[bp].resize(controlBoardAxes[bp]);
    pwmLastRead[bp].resize(controlBoardAxes[bp]);
    torqueSensorsLastRead[bp].resize(controlBoardAxes[bp]);

    return true;
}

/********************************************** ADD *******************************************************/

bool yarpWholeBodySensors::addEncoder(const wbiId &j)
{
    // if initialization was done, no sensors can be added
    if(initDone)
    {
        return false;
    }

    // if initialization was not done, drivers will be opened during initialization
    return encoderIdList.addId(j);
}

int yarpWholeBodySensors::addEncoders(const wbiIdList &jList)
{
    if(initDone)
    {
        return false;
    }

    // if initialization was not done, drivers will be opened during initialization
    return encoderIdList.addIdList(jList);
}

bool yarpWholeBodySensors::addPwm(const wbiId &j)
{
    if(initDone)
    {
        return false;
    }

    return pwmSensIdList.addId(j);
}

int yarpWholeBodySensors::addPwms(const wbiIdList &jList)
{
    if( initDone )
    {
        return false;
    }

    return pwmSensIdList.addIdList(jList);
}

bool yarpWholeBodySensors::addIMU(const wbi::wbiId &i)
{
    // if initialization was not done, ports will be opened during initialization
    if(initDone)
    {
            return false;
    }

    return imuIdList.addId(i);
}

int yarpWholeBodySensors::addIMUs(const wbi::wbiIdList &jList)
{
    // if initialization was done, then open port of specified IMU
    // if initialization was not done, ports will be opened during initialization
    if(initDone)
    {
        return 0;
    }

    return imuIdList.addIdList(jList);
}

bool yarpWholeBodySensors::addFTsensor(const wbi::wbiId &i)
{
    // if initialization was done, then open port of specified F/T sensor
    // if initialization was not done, ports will be opened during initialization
    if(initDone)
    {
        return false;
    }

    return ftSensIdList.addId(i);
}

int yarpWholeBodySensors::addFTsensors(const wbi::wbiIdList &jList)
{
    if(initDone)
    {
        return 0;
    }

    return ftSensIdList.addIdList(jList);
}

bool yarpWholeBodySensors::addTorqueSensor(const wbi::wbiId &i)
{
    if(initDone)
    {
        return false;
    }

    return torqueSensorIdList.addId(i);
}

int yarpWholeBodySensors::addTorqueSensors(const wbi::wbiIdList &jList)
{
    if(initDone)
    {
        return 0;
    }

    return torqueSensorIdList.addIdList(jList);
}

/**************************** READ ************************/

bool yarpWholeBodySensors::readEncoders(double *q, double *stamps, bool wait)
{
    //std::cout << "|||||||||| Read encoders " << std::endl;
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool res = true, update=false;

     //Read data from all controlboards
    for(std::vector<int>::const_iterator ctrlBoard = encoderControlBoardList.begin();
        ctrlBoard != encoderControlBoardList.end(); ctrlBoard++ )
    {
        // read data
        // std::cout << "|||||||||| getEncodersTimed " << std::endl;
        qTemp[0] = -10.0;
        qTemp[1] = -10.0;
        while( !(update=ienc[*ctrlBoard]->getEncodersTimed(qTemp, tTemp)) && wait)
        {
            //std::cout << "waitign " << qTemp[0] << " " << qTemp[1] << std::endl;
            Time::delay(WAIT_TIME);
        }

        // if reading has succeeded, update last read data
        if(update)
        {
            for(int axis=0; axis < (int)qLastRead[*ctrlBoard].size(); axis++ )
            {
                //std::cout << "read qTemp : " << qTemp[axis] << std::endl;
                qLastRead[*ctrlBoard][axis] = CTRL_DEG2RAD*qTemp[axis];
                qStampLastRead[*ctrlBoard][axis] = tTemp[axis];
            }
        }

        res = res && update;
    }

     //Copy readed data in the output vector
    for(int encNumericId = 0; encNumericId < (int)encoderIdList.size(); encNumericId++)
    {
        int encControlBoard = encoderControlBoardAxisList[encNumericId].first;
        int encAxis = encoderControlBoardAxisList[encNumericId].second;
        q[encNumericId] = qLastRead[encControlBoard][encAxis];
        if(stamps!=0)
                stamps[encNumericId] = qStampLastRead[encControlBoard][encAxis];
    }


    return res || wait;
}

bool yarpWholeBodySensors::readPwms(double *pwm, double *stamps, bool wait)
{
    //Do not support stamps on pwm
    assert(stamps == 0);

    ///< check that we are not in simulation, because iCub simulator does not implement pwm control
    if(isICubSimulator(robot))
    {
        memset(pwm, 0, sizeof(double) * pwmSensIdList.size());
        return true;
    }

    double pwmTemp[MAX_NJ];
    bool res = true, update=false;

    //Read data from all controlboards
    for(std::vector<int>::iterator ctrlBoard=pwmControlBoardList.begin();
        ctrlBoard != pwmControlBoardList.end(); ctrlBoard++ )
    {
        // read data
        while( !(update=iopl[*ctrlBoard]->getOutputs(pwmTemp)) && wait)
            Time::delay(WAIT_TIME);

        // if reading has succeeded, update last read data
        if(update)
        {
            for(int axis=0; axis < (int) pwmLastRead[*ctrlBoard].size(); axis++ )
            {
                pwmLastRead[*ctrlBoard][axis] = pwmTemp[axis];
            }
        }

        res = res && update;
    }

    //Copy readed data in the output vector
    for(int pwmNumericId = 0; pwmNumericId < (int)pwmSensIdList.size(); pwmNumericId++)
    {
        int pwmControlBoard = pwmControlBoardAxisList[pwmNumericId].first;
        int pwmAxes = pwmControlBoardAxisList[pwmNumericId].second;
        pwm[pwmNumericId] = pwmLastRead[pwmControlBoard][pwmAxes];
    }

    return res || wait;
}

bool yarpWholeBodySensors::readIMUs(double *inertial, double *stamps, bool wait)
{
    assert(false);
    return false;
    Vector *v;
    for(int i=0; i < (int)imuIdList.size(); i++)
    {
        v = portsIMU[i]->read(wait);
        if(v!=0)
        {
            yarp::os::Stamp info;
            imuLastRead[i] = *v;
            portsIMU[i]->getEnvelope(info);
            imuStampLastRead[i] = info.getTime();
        }
        convertIMU(&inertial[sensorTypeDescriptions[SENSOR_IMU].dataSize*i],imuLastRead[i].data());
        if( stamps != 0 ) {
            stamps[i] = imuStampLastRead[i];
        }
    }
    return true;
}

bool yarpWholeBodySensors::readFTsensors(double *ftSens, double *stamps, bool wait)
{
    ///< iCub simulator does not implement the force/torque sensors
    if(isICubSimulator(robot))
    {
        memset(ftSens, 0, sizeof(double) * portsFTsens.size());
        return true;
    }

    Vector *v;
    for(int i=0; i < (int)ftSensIdList.size(); i++)
    {
        v = portsFTsens[i]->read(wait);
        if(v!=0)
        {
            ftSensLastRead[i] = *v;
            yarp::os::Stamp info;
            portsFTsens[i]->getEnvelope(info);
            ftStampSensLastRead[i] = info.getTime();
        }
        memcpy(&ftSens[i*6], ftSensLastRead[i].data(), 6*sizeof(double));
        if( stamps != 0 ) {
                stamps[i] = ftStampSensLastRead[i];
        }
        i++;
    }
    return true;
}

bool yarpWholeBodySensors::readTorqueSensors(double *jointSens, double *stamps, bool wait)
{
    if(isICubSimulator(robot))
    {
        memset(jointSens, 0, sizeof(double) * torqueSensorIdList.size());
        return true;
    }

    bool res = true, update=false;
   //Do not support stamps on torque sensors

    //Read data from all controlboards
    for(std::vector<int>::iterator ctrlBoard=torqueControlBoardList.begin();
        ctrlBoard != torqueControlBoardList.end(); ctrlBoard++ )
    {
        // read data
        while( !(update=itrq[*ctrlBoard]->getTorques(torqueSensorsLastRead[*ctrlBoard].data())) && wait)
            Time::delay(WAIT_TIME);

        res = res && update;
    }

    //Copy readed data in the output vector
    for(int torqueNumericId = 0; torqueNumericId < (int)torqueSensorIdList.size(); torqueNumericId++)
    {
        int torqueControlBoard = torqueControlBoardAxisList[torqueNumericId].first;
        int torqueAxis = torqueControlBoardAxisList[torqueNumericId].second;
        jointSens[torqueNumericId] = torqueSensorsLastRead[torqueControlBoard][torqueAxis];
    }

    if(stamps != 0)
    {
        double now = yarp::os::Time::now();
        for(int torqueNumericId = 0; torqueNumericId < (int)torqueSensorIdList.size(); torqueNumericId++)
        {
            stamps[torqueNumericId] = now;
        }
    }

    return res || wait;
}

bool yarpWholeBodySensors::readEncoder(const int encoder_numeric_id, double *q, double *stamps, bool wait)
{
    bool update=false;
    int encoderCtrlBoard = encoderControlBoardAxisList[encoder_numeric_id].first;
    int encoderCtrlBoardAxis = encoderControlBoardAxisList[encoder_numeric_id].second;

    double qTemp[MAX_NJ], tTemp[MAX_NJ];

    // read encoders
    while( !(update=ienc[encoderCtrlBoard]->getEncodersTimed(qTemp,tTemp)) && wait)
        Time::delay(WAIT_TIME);

    if( update )
    {
          for(int axis=0; axis < (int)qLastRead[encoderCtrlBoard].size(); axis++ )
          {
                //std::cout << "read qTemp : " << qTemp[axis] << std::endl;
                qLastRead[encoderCtrlBoard][axis] = CTRL_DEG2RAD*qTemp[axis];
                qStampLastRead[encoderCtrlBoard][axis] = tTemp[axis];
          }
    }

    // copy most recent data into output variables
    q[0] = qLastRead[encoderCtrlBoard][encoderCtrlBoardAxis];
    if(stamps!=0)
        stamps[0] = qStampLastRead[encoderCtrlBoard][encoderCtrlBoardAxis];

    return update || wait;  // if read failed => return false
}

bool yarpWholeBodySensors::readPwm(const int pwm_numeric_id, double *pwm, double *stamps, bool wait)
{
    assert(stamps == 0);
    if(isICubSimulator(robot))
    {
        pwm[0] = 0.0;   // iCub simulator does not have pwm sensors
        return true;    // does not return false, so programs can be tested in simulation
    }

    bool update=false;
    int pwmCtrlBoard = pwmControlBoardAxisList[pwm_numeric_id].first;
    int pwmCtrlBoardAxis = pwmControlBoardAxisList[pwm_numeric_id].second;

    // read pwm sensors
    while( !(update=iopl[pwmCtrlBoard]->getOutputs(pwmLastRead[pwmCtrlBoard].data())) && wait)
        Time::delay(WAIT_TIME);

    // copy most recent data into output variables
    pwm[0] = pwmLastRead[pwmCtrlBoard][pwmCtrlBoardAxis];

    return update || wait;  // if read failed => return false
}

bool yarpWholeBodySensors::convertIMU(double * wbi_imu_readings, const double * yarp_imu_readings)
{
    //wbi orientation is expressed in axis-angle, yarp orientation in euler angles (roll pitch yaw)
    //wbi  : orientation(4) - linear acceleration (3) - angular velocity    (3) - magnetometer (3)
    //yarp : orientation(3) - linear acceleration (3) - angular velocity    (3) - magnetometer (3)
    /// \todo TODO check right semantics of yarp IMU

    //Note: this function stop working if the definition of the IMU message in the wbi changes
    assert(sensorTypeDescriptions[SENSOR_IMU].dataSize == 13);

    Rotation imu_orientation = Rotation::eulerZYX(yarp_imu_readings[0],yarp_imu_readings[1],yarp_imu_readings[2]);
    imu_orientation.getAxisAngle(*wbi_imu_readings,*(wbi_imu_readings+1),*(wbi_imu_readings+2),*(wbi_imu_readings+3));
    memcpy(wbi_imu_readings+4,yarp_imu_readings+3,9*sizeof(double));
    return true;
}

bool yarpWholeBodySensors::readIMU(const int imu_sensor_numeric_id, double *inertial, double *stamps, bool wait)
{
    #ifndef NDEBUG
    if( imu_sensor_numeric_id < 0 || imu_sensor_numeric_id >= (int)imuIdList.size() ) {
        std::cerr << "yarpWholeBodySensors::readIMU(..) error: no port found for imu with numeric id " << imu_sensor_numeric_id<< std::endl;
        return false;
    }
    #endif

    Vector *v = portsIMU[imu_sensor_numeric_id]->read(wait);
    if(v!=0) {
        yarp::os::Stamp info;
        imuLastRead[imu_sensor_numeric_id] = *v;
        portsIMU[imu_sensor_numeric_id]->getEnvelope(info);
        imuStampLastRead[imu_sensor_numeric_id] = info.getTime();
    }
    if( stamps != 0 ) {
        *stamps = imuStampLastRead[imu_sensor_numeric_id];
    }
    convertIMU(inertial,imuLastRead[imu_sensor_numeric_id].data());

    return true;
}

bool yarpWholeBodySensors::readFTsensor(const int ft_sensor_numeric_id, double *ftSens, double *stamps, bool wait)
{
    #ifndef NDEBUG
    if( ft_sensor_numeric_id < 0 && ft_sensor_numeric_id >= (int)ftSensIdList.size() ) {
        std::cerr << "yarpWholeBodySensors::readFTsensor(..) error: no port found for ft sensor " << ft_sensor_numeric_id  << std::endl;
        return false;
    }
    #endif

    if(isICubSimulator(robot))    // icub simulator doesn't have force/torque sensors
    {
        ftSens[0] = 0.0;
        return true;
    }

    Vector *v = portsFTsens[ft_sensor_numeric_id]->read(wait);
    if(v!=NULL) {
        ftSensLastRead[ft_sensor_numeric_id] = *v;
        yarp::os::Stamp info;
        portsFTsens[ft_sensor_numeric_id]->getEnvelope(info);
        ftStampSensLastRead[ft_sensor_numeric_id] = info.getTime();
    }
    memcpy(&ftSens[0], ftSensLastRead[ft_sensor_numeric_id].data(), 6*sizeof(double));
    if( stamps != 0 ) {
        *stamps = ftStampSensLastRead[ft_sensor_numeric_id];
    }

    return true;
}

bool yarpWholeBodySensors::readTorqueSensor(const int numeric_torque_id, double *jointTorque, double *stamps, bool wait)
{
    if(isICubSimulator(robot))
    {
        jointTorque[0] = 0.0;   // iCub simulator does not have joint torque sensors
        return true;            // does not return false, so programs can be tested in simulation
    }
    double torqueTemp;
    bool update=false;

    int torqueCtrlBoard = torqueControlBoardAxisList[numeric_torque_id].first;
    int torqueCtrlBoardAxis = torqueControlBoardAxisList[numeric_torque_id].second;

    assert(itrq[torqueCtrlBoard]!=0);

    // read joint torque
    while(!(update = itrq[torqueCtrlBoard]->getTorque(torqueCtrlBoardAxis, &torqueTemp)) && wait)
        Time::delay(WAIT_TIME);

    // if read succeeded => update data
    if(update)
        torqueSensorsLastRead[torqueCtrlBoard][torqueCtrlBoardAxis] = torqueTemp;

    // copy most recent data into output variables
    jointTorque[0] = torqueSensorsLastRead[torqueCtrlBoard][torqueCtrlBoardAxis];

    return update || wait;  // if read failed => return false
}
