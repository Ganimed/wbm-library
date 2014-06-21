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
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <string>
#include <sstream>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

#define MAX_NJ 20 ///< Maximum number of joints for body part (used for buffers to avoid dynamic memory allocation)
#define WAIT_TIME 0.001

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY SENSORS
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodySensors::yarpWholeBodySensors(const char* _name, const char* _robotName, yarp::os::Property & opt):
initDone(false), name(_name), robot(_robotName)
{
    /*
    loadBodyPartsFromConfig(opt,controlBoardNames);
    loadReverseTorsoJointsFromConfig(opt,reverse_torso_joints);
    loadFTSensorPortsFromConfig(opt,controlBoardNames,ftSens_2_port);
    loadIMUSensorPortsFromConfig(opt,controlBoardNames,imu_2_port);
    */
}


bool yarpWholeBodySensors::init()
{
    //Get encoders
    #ifndef NDEBUG
    std::cout << "yarpWholeBodySensors: initializing with " << encoderIdList.size() << " encoders " <<
                                                               pwmSensIdList.size() << " pwm sensors " <<
                                                               ftSensIdList.size() << " F/T sensors " <<
                                                               imuIdList.size() << " IMUs " << std::endl;
    #endif

    bool initDone = true;

    yarp::os::Bottle & joints_config = getWBIYarpJointsOptions(wbi_yarp_properties);
    controlBoardNames.clear();
    initDone = appendNewControlBoardsToVector(joints_config,encoderIdList,controlBoardNames);
    initDone = initDone && appendNewControlBoardsToVector(joints_config,pwmSensIdList,controlBoardNames);
    initDone = initDone && appendNewControlBoardsToVector(joints_config,torqueSensorIdList,controlBoardNames);
    if( !initDone )
    {
        return false;
    }

    encoderControlBoardAxisList = getControlBoardAxisList(joints_config,encoderIdList,controlBoardNames);
    pwmControlBoardAxisList = getControlBoardAxisList(joints_config,pwmSensIdList,controlBoardNames);
    torqueControlBoardAxisList = getControlBoardAxisList(joints_config,torqueSensorIdList,controlBoardNames);

    encoderControlBoardList = getControlBoardList(encoderControlBoardAxisList);
    pwmControlBoardList     = getControlBoardList(pwmControlBoardAxisList);
    torqueControlBoardList  = getControlBoardList(torqueControlBoardAxisList);

    for(int i=0; i < encoderControlBoardList.size(); i++ )
    {
        int ctrlBoard = encoderControlBoardList[i];
        initDone = initDone && openEncoder(ctrlBoard);
    }
    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening encoders." << std::endl;
        return false;
    }

    for(int i=0; i < pwmControlBoardList.size(); i++ )
    {
        int ctrlBoard = pwmControlBoardList[i];
        initDone = initDone && openPwm(ctrlBoard);
    }

    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening motor pwm inputs." << std::endl;
        return false;
    }

    for(int i=0; i < torqueControlBoardList.size(); i++ )
    {
        int ctrlBoard = torqueControlBoardList[i];
        initDone = initDone && openTorqueSensor(ctrlBoard);
    }

    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening torques sensors." << std::endl;
        return false;
    }

    for(wbiIdList::iterator itFt=ftSensIdList.begin(); itFt!=ftSensIdList.end(); itFt++)
    {
            initDone = initDone && openFTsens(*itFt);
    }

    if( !initDone )
    {
        std::cerr << "yarpWholeBodySensors::init() error: failing in opening force/torque sensors." << std::endl;
        return false;
    }

    for(wbiIdList::iterator itImu=imuIdList.begin(); itImu!=imuIdList.end(); itImu++)
    {
            initDone = initDone && openImu(*itImu);
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
    for(int i=0; i < encoderControlBoardList.size(); i++ )
    {
        int ctrlBoard = encoderControlBoardList[i];
        assert(dd[ctrlBoard]!=NULL);
        ok = ok && dd[ctrlBoard]->close();
        delete dd[ctrlBoard];
        dd[ctrlBoard] = NULL;
    }

    for(int i=0; i < pwmControlBoardList.size(); i++ )
    {
        int ctrlBoard = pwmControlBoardList[i];
        if(dd[ctrlBoard]!=NULL)
        {
            ok = ok && dd[ctrlBoard]->close();
            delete dd[ctrlBoard];
            dd[ctrlBoard] = NULL;
        }
    }

    for(int i=0; i < torqueControlBoardList.size(); i++ )
    {
        int ctrlBoard = torqueControlBoardList[i];
        if(dd[ctrlBoard])
        {
            ok = ok && dd[ctrlBoard]->close();
            delete dd[ctrlBoard];
            dd[ctrlBoard] = NULL;
        }
    }

    for(map<wbiId,BufferedPort<Vector>*>::iterator it=portsIMU.begin(); it!=portsIMU.end(); it++)
    {
        if( it->second != 0 ) {
            it->second->close();
            delete it->second;
            it->second=0;
        }
    }

    for(map<wbiId,BufferedPort<Vector>*>::iterator it=portsFTsens.begin(); it!=portsFTsens.end(); it++)
    {
        if( it->second != 0 ) {
            it->second->close();
            delete it->second;
            it->second=0;
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
    switch(st)
    {
    case SENSOR_ENCODER:        return encoderIdList.removeId(sid);;
    case SENSOR_PWM:            return pwmSensIdList.removeId(sid);
    case SENSOR_IMU:            return imuIdList.removeId(sid);
    case SENSOR_FORCE_TORQUE:   return ftSensIdList.removeId(sid);
    case SENSOR_TORQUE:         return torqueSensorIdList.removeId(sid);
    default:break;
    }
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

bool yarpWholeBodySensors::readSensor(const SensorType st, const wbiId &sid, double *data, double *stamps, bool blocking)
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
    return true;
}

bool yarpWholeBodySensors::openImu(const wbiId &i, const std::string & port_name, const int numeric_id)
{
    string remotePort = "/" + robot + port_name;
    stringstream localPort;
    localPort << "/" << name << "/imu" << i.bodyPart << "_" << i.index << ":i";
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
    imuLastRead[i].resize(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0);
    imuStampLastRead[i] = INITIAL_TIMESTAMP;

    return true;
}

bool yarpWholeBodySensors::openFTsens(const wbiId &i, std::string port_name)
{
    /*
     * Disable because Gazebo has FT sensor (the difference in sensor availability between simulator and real robot should be addressed in interface initialization
    if(isRobotSimulator(robot)) // icub simulator doesn't have force/torque sensors
        return true;
    */
    if (isICubSimulator(robot))
        return true;

    string remotePort = "/" + robot + port_name;
    stringstream localPort;
    localPort << "/" << name << "/ftSens/" << i.toString() << ":i";
    portsFTsens[i] = new BufferedPort<Vector>();
    if(!portsFTsens[i]->open(localPort.str().c_str())) {
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
    ftSensLastRead[i].resize(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0);
    ftStampSensLastRead[i] = INITIAL_TIMESTAMP;


    return true;
}

bool yarpWholeBodySensors::openTorqueSensor(const int bp)
{
//    torqueSensorsLastRead[bp].resize(6,0.0);
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
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, encoderIdList)
    {
        assert(ienc[itBp->first]!=NULL);
        // read encoders
        while( !(update=ienc[itBp->first]->getEncodersTimed(qTemp, tTemp)) && wait)
            Time::delay(WAIT_TIME);

        // if read succeeded => update data
        if(update)
            for(unsigned int i=0; i<controlBoardAxes[itBp->first]; i++)
            {
                assert( i < qLastRead[itBp->first].size() );
                qLastRead[itBp->first][bodyPartJointMapping(itBp->first,i)] = CTRL_DEG2RAD*qTemp[i];;
                qStampLastRead[itBp->first][bodyPartJointMapping(itBp->first,i)]   = tTemp[i];
            }

        // copy most recent data into output variables
        FOR_ALL_JOINTS(itBp, itJ)
        {
            q[i] = qLastRead[itBp->first][*itJ];
            if(stamps!=0)
                stamps[i] = qStampLastRead[itBp->first][*itJ];
            i++;
        }
        res = res && update;    // if read failed => return false
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
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, pwmSensIdList)
    {
        // read data
        while( !(update=iopl[itBp->first]->getOutputs(pwmTemp)) && wait)
            Time::delay(WAIT_TIME);

        // if reading has succeeded, update last read data
        if(update)
            for(unsigned int i=0; i<controlBoardAxes[itBp->first]; i++)
                pwmLastRead[itBp->first][bodyPartJointMapping(itBp->first,i)] = pwmTemp[i];    // joints of the torso are in reverse order

        // copy data in output vector
        FOR_ALL_JOINTS(itBp, itJ)
        {
            pwm[i]      = pwmLastRead[itBp->first][*itJ];
            // stamps[i]   = ?;
            i++;
        }
        res = res && update;
    }
    return res || wait;
}

bool yarpWholeBodySensors::readIMUs(double *inertial, double *stamps, bool wait)
{
    assert(false);
    return false;
    Vector *v;
    int i=0;    // sensor index
    for(map<wbiId,BufferedPort<Vector>*>::iterator it=portsIMU.begin(); it!=portsIMU.end(); it++)
    {
        v = it->second->read(wait);
        if(v!=0)
        {
            yarp::os::Stamp info;
            imuLastRead[it->first] = *v;
            it->second->getEnvelope(info);
            imuStampLastRead[it->first] = info.getTime();
        }
        convertIMU(&inertial[sensorTypeDescriptions[SENSOR_IMU].dataSize*i],imuLastRead[it->first].data());
        if( stamps != 0 ) {
            stamps[i] = imuStampLastRead[it->first];
        }
        i++;
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
    int i=0;    // sensor index
    for(map<wbiId,BufferedPort<Vector>*>::iterator it=portsFTsens.begin(); it!=portsFTsens.end(); it++)
    {
        v = it->second->read(wait);
        if(v!=0)
        {
            ftSensLastRead[it->first] = *v;
            yarp::os::Stamp info;
            it->second->getEnvelope(info);
            ftStampSensLastRead[it->first] = info.getTime();
        }
        memcpy(&ftSens[i*6], ftSensLastRead[it->first].data(), 6*sizeof(double));
        if( stamps != 0 ) {
                stamps[i] = ftStampSensLastRead[it->first];
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

    double torqueTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, torqueSensorIdList)
    {
        // read data
        while( !(update = itrq[itBp->first]->getTorques(torqueTemp)) && wait)
            Time::delay(WAIT_TIME);

        // if reading has succeeded, update last read data
        if(update)
            for(unsigned int i = 0; i < bodyPartAxes[itBp->first]; i++)
                torqueSensorsLastRead[itBp->first][bodyPartJointMapping(itBp->first,i)] = torqueTemp[i];    // joints of the torso are in reverse order

        // copy data in output vector
        FOR_ALL_JOINTS(itBp, itJ)
        {
            jointSens[i] = torqueSensorsLastRead[itBp->first][*itJ];
            i++;
        }
        res = res && update;
    }
    return res || wait;
}

bool yarpWholeBodySensors::readEncoder(const wbiId &sid, double *q, double *stamps, bool wait)
{
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool update=false;
    assert(ienc[sid.bodyPart]!=0);
    // read encoders
    while( !(update=ienc[sid.bodyPart]->getEncodersTimed(qTemp, tTemp)) && wait)
        Time::delay(WAIT_TIME);

    // if read succeeded => update data
    if(update)
        for(unsigned int i=0; i<controlBoardAxes[sid.bodyPart]; i++)
        {
            // joints 0 and 2 of the torso are swapped
            qLastRead[sid.bodyPart][bodyPartJointMapping(sid.bodyPart,i)]        = CTRL_DEG2RAD*qTemp[i];
            qStampLastRead[sid.bodyPart][bodyPartJointMapping(sid.bodyPart,i)]   = tTemp[i];
        }

    // copy most recent data into output variables
    q[0] = qLastRead[sid.bodyPart][sid.index];
    if(stamps!=0)
        stamps[0] = qStampLastRead[sid.bodyPart][sid.index];

    return update || wait;  // if read failed => return false
}

bool yarpWholeBodySensors::readPwm(const wbiId &sid, double *pwm, double *stamps, bool wait)
{
    assert(stamps == 0);
    if(isICubSimulator(robot))
    {
        pwm[0] = 0.0;   // iCub simulator does not have pwm sensors
        return true;    // does not return false, so programs can be tested in simulation
    }
    double pwmTemp[MAX_NJ];
    bool update=false;
    assert(iopl[sid.bodyPart]!=0);
    // read motor PWM
    while( !(update=iopl[sid.bodyPart]->getOutputs(pwmTemp)) && wait)
        Time::delay(WAIT_TIME);

    // if read succeeded => update data
    if(update) // joints 0 and 2 of the torso are swapped
        pwmLastRead[sid.bodyPart][bodyPartJointMapping(sid.bodyPart,sid.index)] = pwmTemp[sid.index];

    // copy most recent data into output variables
    pwm[0] = pwmLastRead[sid.bodyPart][sid.index];

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

bool yarpWholeBodySensors::readIMU(const wbiId &sid, double *inertial, double *stamps, bool wait)
{
    #ifndef NDEBUG
    if( portsIMU.find(sid) == portsIMU.end() ) {
        std::cerr << "yarpWholeBodySensors::readIMU(..) error: no port found for localId " << sid.bodyPart << " " << sid.index << " " << sid.description << std::endl;
        std::cerr << "Available sensors: " << std::endl;
        for(map<wbiId,BufferedPort<Vector>*>::iterator it=portsIMU.begin(); it!=portsIMU.end(); it++) {
            std::cout << "Bp: " << it->first.bodyPart << " index " << it->first.index << std::endl;
        }

        return false;
    }
    #endif

    Vector *v = portsIMU[sid]->read(wait);
    if(v!=0) {
        yarp::os::Stamp info;
        imuLastRead[sid] = *v;
        portsIMU[sid]->getEnvelope(info);
        imuStampLastRead[sid] = info.getTime();
    }
    if( stamps != 0 ) {
        *stamps = imuStampLastRead[sid];
    }
    convertIMU(inertial,imuLastRead[sid].data());

    return true;
}

bool yarpWholeBodySensors::readFTsensor(const wbiId &sid, double *ftSens, double *stamps, bool wait)
{
    #ifndef NDEBUG
    if( portsFTsens.find(sid) == portsFTsens.end() ) {
        std::cerr << "yarpWholeBodySensors::readFTsensor(..) error: no port found for localId " << sid.bodyPart << " " << sid.index << " " << sid.description << std::endl;
        return false;
    }
    #endif

    if(isICubSimulator(robot))    // icub simulator doesn't have force/torque sensors
    {
        ftSens[0] = 0.0;
        return true;
    }

    Vector *v = portsFTsens[sid]->read(wait);
    if(v!=NULL) {
        ftSensLastRead[sid] = *v;
        yarp::os::Stamp info;
        portsFTsens[sid]->getEnvelope(info);
        ftStampSensLastRead[sid] = info.getTime();
    }
    memcpy(&ftSens[0], ftSensLastRead[sid].data(), 6*sizeof(double));
    if( stamps != 0 ) {
        *stamps = ftStampSensLastRead[sid];
    }

    return true;
}

bool yarpWholeBodySensors::readTorqueSensor(const wbiId &sid, double *jointTorque, double *stamps, bool wait)
{
    if(isICubSimulator(robot))
    {
        jointTorque[0] = 0.0;   // iCub simulator does not have joint torque sensors
        return true;            // does not return false, so programs can be tested in simulation
    }
    double torqueTemp;
    bool update=false;
    assert(itrq[sid.bodyPart]!=0);

    // read joint torque
    int jointIndex = bodyPartJointMapping(sid.bodyPart,sid.index);
    while(!(update = itrq[sid.bodyPart]->getTorque(jointIndex, &torqueTemp)) && wait)
        Time::delay(WAIT_TIME);

    // if read succeeded => update data
    if(update) // joints 0 and 2 of the torso are swapped
        torqueSensorsLastRead[sid.bodyPart][jointIndex] = torqueTemp;

    // copy most recent data into output variables
    jointTorque[0] = torqueSensorsLastRead[sid.bodyPart][jointIndex];

    return update || wait;  // if read failed => return false
}

int yarpWholeBodySensors::bodyPartJointMapping(int bodypart_id, int local_id)
{
    if( reverse_torso_joints ) {
        return bodypart_id==TORSO ? 2-local_id : local_id;
    } else {
        return local_id;
    }
}
