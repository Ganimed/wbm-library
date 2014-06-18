/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete, Marco Randazzo
 * email: andrea.delprete@iit.it marco.randazzo@iit.it
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

#include "yarpWholeBodyInterface/yarpWholeBodyActuators.h"
#include <wbi/wbiConstants.h>
#include <yarp/os/Property.h>
#include <string>
#include <cassert>

//*********TEMP**************//
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
#include <paramHelp/paramHelperClient.h>
#include <motorFrictionIdentificationLib/jointTorqueControlParams.h>
#endif
//*********END TEMP**********//

using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;


#define WAIT_TIME 0.001         ///< waiting time in seconds before retrying to perform an operation that has failed
#define DEFAULT_REF_SPEED 10.0  ///< default reference joint speed for the joint position control

//constants
const std::string yarpWholeBodyActuactors::icubWholeBodyActuactorsUseExternalTorqueModule = "icubWholeBodyActuactorsUseExternalTorqueModuleKey";
const std::string yarpWholeBodyActuactors::icubWholeBodyActuactorsExternalTorqueModuleName = "icubWholeBodyActuactorsExternalTorqueModuleNameKey";
const std::string yarpWholeBodyActuactors::icubWholeBodyActuactorsExternalTorqueModuleAutoconnect = "icubWholeBodyActuactorsExternalTorqueModuleAutoconnect";

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY ACTUATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
//yarpWholeBodyActuactors::yarpWholeBodyActuactors(const char* _name,
//                                               const char* _robotName,
//                                               const std::vector<std::string> &_bodyPartNames)
//: m_commandedParts(0), initDone(false), dof(0), name(_name), robot(_robotName), bodyPartNames(_bodyPartNames), reverse_torso_joints(true)
//#ifdef WBI_ICUB_COMPILE_PARAM_HELP
//,_torqueModuleConnection(0)
//#endif
//{}



yarpWholeBodyActuators::yarpWholeBodyActuators(const char* _name,
                                               const char* _robotName,
                                               const yarp::os::Property & yarp_wbi_properties)
: initDone(false), dof(0), name(_name), robot(_robotName)
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
,_torqueModuleConnection(0)
#endif
{
    configurationParameters = yarp_wbi_properties;
}



yarpWholeBodyActuators::~yarpWholeBodyActuators()
{
    close();
}

bool yarpWholeBodyActuators::openControlBoardDrivers(int bp)
{
    if( bp >= (int)controlBoardNames.size() || bp < 0 )
    {
        std::cerr << "yarpWholeBodyActuators::openDrivers error: called with bodypart " << bp <<
                     " but the total number of bodyparts considered in the interface is " << controlBoardNames.size() << std::endl;
        return false;
    }
    itrq[bp]=0; iimp[bp]=0; icmd[bp]=0; ivel[bp]=0; ipos[bp]=0; iopl[bp]=0;  dd[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], controlBoardNames[bp].c_str()))
    {
        std::cerr << "yarpWholeBodyActuators::openDrivers error: enable to open controlboard " << controlBoardNames[bp]
                  << "of robot " << robot  << std::endl;
        return false;
    }

    //Open all necessary interfaces
    bool ok = dd[bp]->view(itrq[bp]) && dd[bp]->view(iimp[bp]) && dd[bp]->view(icmd[bp])
              && dd[bp]->view(ivel[bp]) && dd[bp]->view(ipos[bp]) && dd[bp]->view(iopl[bp])
              && dd[bp]->view(positionDirectInterface[bp]);


    if(!ok)
    {
        std::cerr << "yarpWholeBodyActuators::openDrivers error: enable to open all necessary interfaces of " <<
                     controlBoardNames[bp]  << "of robot " << robot  << std::endl;
        return false;
    }

    return true;
}

bool yarpWholeBodyActuators::init()
{
    //Function return value
    bool ok = false;

    //Loading configuration
    loadControlledJointsFromConfig(configurationParameters,
                                   jointIdList,
                                   controlBoardNames,
                                   controlBoardAxisList);

    //Update internal structure reference the controlled joints for each controlboard
    updateControlledJointsForEachControlBoard();

    //Open necessary yarp controlboard drivers
    //iterate all used body parts
    for(int bp=0; bp < controlBoardNames.size(); bp++ )
    {
        ok = openControlBoardDrivers(bp);
        if(!ok)
        {
            //If there is an error, close all the opened driver and return
            int lastDriverOpenedCorrectly = bp-1;
            for(int driverToClose = lastDriverOpenedCorrectly; driverToClose >=0; driverToClose--)
            {
                if( dd[driverToClose] != 0 )
                {
                    dd[driverToClose]->close();
                    delete dd[driverToClose];
                    dd[driverToClose] = 0;
                }
            }
            return false;
        }
    }

    //All drivers opened without errors, save the dimension of all used controlboards
    totalAxesInControlBoard.resize(controlBoardNames.size());
    for(int ctrlBrd = 0; ctrlBrd < controlBoardNames.size(); ctrlBrd++ )
    {
        ok = ipos[ctrlBrd]->getAxes(totalAxesInControlBoard[ctrlBrd]);
        if( !ok )
        {
            return false;
        }
    }

#ifdef WBI_ICUB_COMPILE_PARAM_HELP
    ///TEMP
    if (_torqueModuleConnection) {
        _torqueModuleConnection->close();
        delete _torqueModuleConnection; _torqueModuleConnection = NULL;
    }
    if (ok) {
        //read options
        yarp::os::Value found;
        _rpcAutoConnect = false;
        found = configurationParameters.find(yarpWholeBodyActuactorsExternalTorqueModuleAutoconnect.c_str());

        if (!found.isNull() && found.isBool()) {
            _rpcAutoConnect = found.asBool();
        }
        found = configurationParameters.find(yarpWholeBodyActuactorsUseExternalTorqueModule.c_str());
        if (!found.isNull() && found.isBool() && found.asBool()) {
            found = configurationParameters.find(yarpWholeBodyActuactorsExternalTorqueModuleName.c_str());
            if (found.isNull()) {
                ok = false;
            }
            else {

                _torqueModuleConnection = new paramHelp::ParamHelperClient(jointTorqueControl::jointTorqueControlParamDescr, jointTorqueControl::PARAM_ID_SIZE,
                                                                           jointTorqueControl::jointTorqueControlCommandDescr, jointTorqueControl::COMMAND_ID_SIZE);

                Bottle initMsg;
                if (!_torqueModuleConnection || !_torqueModuleConnection->init(name, found.asString().c_str(), initMsg)) {
                    ok = false;
                }
                else {
                    _torqueRefs.resize(jointTorqueControl::N_DOF);
                    ok = _torqueModuleConnection->linkParam(jointTorqueControl::PARAM_ID_TAU_OFFSET, _torqueRefs.data());
                    if (_rpcAutoConnect) {
                        _rpcLocalName = "/" + name + "/rpc:o";
                        _rpcRemoteName = "/" + found.asString() + "/rpc";
                        ok = ok && _torqueModuleRPCClientPort.open(_rpcLocalName);
                        ok = ok && Network::connect(_rpcLocalName, _rpcRemoteName);
                    }
                }
            }
        }
    }
    ///END TEMP
#endif


    initDone = true;
    return ok;
}

bool yarpWholeBodyActuatorsControlledJoints::reset(const int nrOfControlBoards)
{
    if( nrOfControlBoards < 0 )
    {
        return false;
    }
    positionControlledJoints.clear();
    positionControlledJoints.resize(nrOfControlBoards);
    positionDirectedControlledJoints.clear();
    positionDirectedControlledJoints.resize(nrOfControlBoards);
    velocityControlledJoints.clear();
    velocityControlledJoints.resize(nrOfControlBoards);
    torqueControlledJoints.clear();
    torqueControlledJoints.resize(nrOfControlBoards);
    pwmControlledJoints.clear();
    pwmControlledJoints.resize(nrOfControlBoards);
}


bool yarpWholeBodyActuators::updateControlledJointsForEachControlBoard()
{
    controlledJointsForControlBoard.reset(controlBoardNames.size());

    for(int wbi_jnt = 0; wbi_jnt++; wbi_jnt < dof )
    {
        wbi::ControlMode wbi_jnt_ctrl_mode = currentCtrlModes[wbi_jnt];

        int wbi_jnt_controlboard_id = bodyPartAxisList[wbi_jnt].first;
        int wbi_jnt_yarp_axis = bodyPartAxisList[wbi_jnt].second;

        yarpWBAControlledJoint wbi_yarp_jnt;
        wbi_yarp_jnt.wbi_controlboard_id = wbi_jnt_controlboard_id;
        wbi_yarp_jnt.wbi_id = wbi_jnt;
        wbi_yarp_jnt.yarp_controlboard_axis = wbi_jnt_yarp_axis;

        switch(wbi_jnt_ctrl_mode)
        {
            case wbi::CTRL_MODE_POS:
                controlledJointsForControlBoard.positionControlledJoints[wbi_jnt_controlboard_id].push_back(wbi_yarp_jnt);
            break;
            case wbi::CTRL_MODE_DIRECT_POSITION:
                controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_jnt_controlboard_id].push_back(wbi_yarp_jnt);
            break;
            case wbi::CTRL_MODE_VEL:
                controlledJointsForControlBoard.velocityControlledJoints[wbi_jnt_controlboard_id].push_back(wbi_yarp_jnt);
            break;
            case wbi::CTRL_MODE_TORQUE:
                controlledJointsForControlBoard.torqueControlledJoints[wbi_jnt_controlboard_id].push_back(wbi_yarp_jnt);
            break;
            case wbi::CTRL_MODE_MOTOR_PWM:
                controlledJointsForControlBoard.pwmControlledJoints[wbi_jnt_controlboard_id].push_back(wbi_yarp_jnt);
            break;
            default:
                return false;
            break;
        }
    }
    return true;
}


bool yarpWholeBodyActuators::close()
{
    bool ok = true;
    for(int ctrlBrd=0; ctrlBrd < controlBoardNames.size(); ctrlBrd++ )
    {
        if( dd[ctrlBrd]!= 0 ) {
            ok = dd[ctrlBrd]->close();
            delete dd[ctrlBrd];
            dd[ctrlBrd] = 0;
        }
    }

#ifdef WBI_ICUB_COMPILE_PARAM_HELP
    ///TEMP
    if (_torqueModuleConnection) {
        _torqueModuleConnection->close();
        delete _torqueModuleConnection; _torqueModuleConnection = NULL;
    }
    if (_rpcAutoConnect) {
        Network::disconnect(_rpcLocalName, _rpcRemoteName);
        _torqueModuleRPCClientPort.close();
    }
#endif

    return ok;
}

bool yarpWholeBodyActuactors::setConfigurationParameter(const std::string &parameterName, const yarp::os::Value &parameterValue)
{
    /*Note to developers: we can move the functionalities offered by the configuration map to an external class in order to make it more generic */
    if (initDone) return false;
    //check allowed parameters
    if (parameterName.compare(yarpWholeBodyActuactorsUseExternalTorqueModule) == 0) {
        if (parameterValue.isBool()) {
            configurationParameters.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    } else if (parameterName.compare(yarpWholeBodyActuactorsExternalTorqueModuleAutoconnect) == 0) {
        if (parameterValue.isBool()) {
            configurationParameters.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    } else if (parameterName.compare(yarpWholeBodyActuactorsExternalTorqueModuleName) == 0) {
        //simply check value has some length
        if (parameterValue.isString() && parameterValue.asString().length() > 0) {
            configurationParameters.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    }

    return false;
}

bool yarpWholeBodyActuactors::removeActuator(const LocalId &j)
{
    if (initDone) return false;

    if(!jointIdList.removeId(j))
        return false;
    dof--;
    return true;
}

bool yarpWholeBodyActuactors::addActuator(const LocalId &j)
{
    if (initDone) return false;

    if(!jointIdList.addId(j))
        return false;

    dof++;
    return true;
}

int yarpWholeBodyActuactors::addActuators(const LocalIdList &jList)
{
    if (initDone) return false;

    int count = jointIdList.addIdList(jList);
    dof += count;

    return count;
}

bool yarpWholeBodyActuators::setControlMode(ControlMode controlMode, double *ref, int joint)
{
    if(joint>=dof)
    {
        return false;
    }

    bool ok = true;
    if(joint<0)     ///< set all joints to the specified control mode
    {
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
        bool controlModeChanged = false;
#endif
        switch(controlMode)
        {
            case CTRL_MODE_POS:
            case CTRL_MODE_DIRECT_POSITION:
                for(int j=0; j < dof; j++ )
                {
                    //Set only the joints that are not in the desired control mode
                    if(currentCtrlModes[j] != controlMode) {
                        ok = ok && icmd[bodyPartAxisList[j].first]->setPositionMode(bodyPartAxisList[j].second);
                    }
                }
                break;

            case CTRL_MODE_VEL:
                for(int j=0; j < dof; j++ )
                {
                    if(currentCtrlModes[j] != controlMode)
                    {
                        ok = ok && icmd[bodyPartAxisList[j].first]->setVelocityMode(bodyPartAxisList[j].second);
                    }
                }
                break;

            case CTRL_MODE_TORQUE:
                for(int j=0; j < dof; j++ )
                    if(currentCtrlModes[j]!=controlMode) {
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
                        controlModeChanged = true;
                        if (_torqueModuleConnection) {
                            //if torque control connection is true I do not set the torqueMode
                            ok = ok && true;
                        }
                        else
#endif
                        {
                            ok = ok && icmd[bodyPartAxisList[j].first]->setTorqueMode(bodyPartAxisList[j].second);
                        }
                    }
                }
                break;

            case CTRL_MODE_MOTOR_PWM:
                if(!isICubSimulator(robot)) ///< iCub simulator does not implement PWM motor control
                {
                    for(int j=0; j < dof; j++ )
                    {
                        if(currentCtrlModes[j] != controlMode)
                        {
                            ok = ok && icmd[bodyPartAxisList[j].first]->setVelocityMode(bodyPartAxisList[j].second);
                        }
                    }
                }
                break;

            default:
                return false;
        }

        //Update internal structure
        this->updateControlledJointsForEachControlBoard();

        if(ok)
        {
            for(int j=0; j < dof; j++ )
            {
                currentCtrlModes[j] = controlMode;
                if(ref!=0)
                    ok = ok && setControlReference(ref);
            }
        }
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
        //send start or stop via RPC to torque module
        if (_rpcAutoConnect) {
            Bottle startCmd;
            if (controlMode == CTRL_MODE_TORQUE) {
                if (controlModeChanged) {
                    startCmd.addString("start");
                    ok = ok && _torqueModuleRPCClientPort.write(startCmd);
                }
            } else if (controlMode == CTRL_MODE_POS) {
                startCmd.addString("stop");
                ok = ok && _torqueModuleRPCClientPort.write(startCmd);
            }

        }
#endif
        return ok;
    }

    if(currentCtrlModes[joint]!=controlMode)   ///< check that joint is not already in the specified control mode
    {
        int bodyPart = bodyPartAxisList[joint].first;
        int controlBoardJointAxis = bodyPartAxisList[joint].second;
        switch(controlMode)
        {
            case CTRL_MODE_POS:
            case CTRL_MODE_DIRECT_POSITION:
                ok = icmd[bodyPart]->setPositionMode(controlBoardJointAxis); break;
            case CTRL_MODE_VEL:         ok = icmd[bodyPart]->setVelocityMode(controlBoardJointAxis); break;
            case CTRL_MODE_TORQUE:      ok = icmd[bodyPart]->setTorqueMode(controlBoardJointAxis);   break;
            ///< iCub simulator does not implement PWM motor control
            case CTRL_MODE_MOTOR_PWM:   ok = isICubSimulator(robot) ? true : icmd[bodyPart]->setOpenLoopMode(controlBoardJointAxis); break;
            default: break;
        }
        if(ok)
        {
            currentCtrlModes[joint] = controlMode;
        }
    }

    //Update internal structure
    this->updateControlledJointsForEachControlBoard();

    if(ok &&ref!=0)
        ok = setControlReference(ref, joint);   ///< set specified control reference (if any)
    return ok;
}

bool yarpWholeBodyActuators::setControlReference(double *ref, int joint)
{
    if(joint> (int)dof)
        return false;

    bool ok = true;
    if(joint>=0)    // set control reference for the specified joint
    {
        int bodyPart = bodyPartAxisList[joint].first;
        int controlBoardAxis = bodyPartAxisList[joint].second;

        switch(currentCtrlModes[joint])
        {
            case CTRL_MODE_POS:
                return ipos[bodyPart]->positionMove(controlBoardAxis, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_DIRECT_POSITION:
                return positionDirectInterface[bodyPart]->setPosition(controlBoardAxis, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_VEL:
                return ivel[bodyPart]->velocityMove(controlBoardAxis, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_TORQUE:
            {
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
                //TEMP
                if (_torqueModuleConnection) {
                    _torqueRefs.zero();
                    //assume that the wbi serialization of this interface and the one of jointTorqueControl coincide
                    int gid = joint;
                    if (gid < 0 || gid >= jointTorqueControl::N_DOF) {
                        return false;
                    }
                    else {
                        _torqueRefs[gid] = *ref;
                        return _torqueModuleConnection->sendStreamParams();
                    }
                }
                else
                //END TEMP
#endif
                    return itrq[bodyPart]->setRefTorque(controlBoardAxis, *ref);
            }
            ///< iCub simulator does not implement PWM motor control
            case CTRL_MODE_MOTOR_PWM:
                return isICubSimulator(robot) ? true : iopl[bodyPart]->setOutput(controlBoardAxis, *ref);
            default: break;
        }
        return false;
    }

    //Buffer variables
    double buf_references[MAX_NJ];
    int buf_controlledJoints[MAX_NJ];

    // set control references for all joints
    for(int wbi_controlboard_id=0; wbi_controlboard_id < controlBoardNames.size(); wbi_controlboard_id++ )
    {
        ///////////////////////////////////////////////////
        //Sending references for position controlled joints
        ///////////////////////////////////////////////////
        int nrOfPosControlledJointsInControlBoard = controlledJointsForControlBoard.positionControlledJoints[wbi_controlboard_id].size();
        if( nrOfPosControlledJointsInControlBoard > 0 )
        {
            if( nrOfPosControlledJointsInControlBoard == totalJointsInControlBoard[wbi_controlboard_id] )
            {
                //If the wbi controls all the joint in the control board, use the usual interface setPositions
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPosControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                     int wbi_id = controlledJointsForControlBoard.positionControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                     int yarp_controlboard_axis =  controlledJointsForControlBoard.positionControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                     buf_references[yarp_controlboard_axis] = CTRL_RAD2DEG*ref[wbi_id];
                }
                ok = ipos[wbi_controlboard_id]->positionMove(buf_references);
                if(!ok)
                {
                    std::cerr << "yarpWholeBodyActuators::setControlReference error:"
                              << " unable to setPositions for controlboard " << controlBoardNames[wbi_controlboard_id] << std::endl;
                    return false;
                }
            }
            else
            {
                //Otherwise send all the commands individually
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPosControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.positionControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.positionControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    ok = ipos[wbi_controlboard_id]->positionMove(yarp_controlboard_axis,CTRL_RAD2DEG*ref[wbi_id]);
                }
            }
        }

        //////////////////////////////////////////////////////////
        //Sending references for direct position controlled joints
        //////////////////////////////////////////////////////////
        int nrOfPosDirectControlledJointsInControlBoard = controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_controlboard_id].size();
        if( nrOfPosDirectControlledJointsInControlBoard > 0 )
        {
            if( nrOfPosDirectControlledJointsInControlBoard == totalJointsInControlBoard[wbi_controlboard_id] )
            {
                //If the wbi controls all the joint in the control board, use the usual interface setPositions
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPosDirectControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    buf_references[yarp_controlboard_axis] = CTRL_RAD2DEG*ref[wbi_id];
                }
                ok = positionDirectInterface[wbi_controlboard_id]->setPositions(buf_references);
                if(!ok)
                {
                    std::cerr << "yarpWholeBodyActuators::setControlReference error:"
                              << " unable to command position for controlboard " << controlBoardNames[wbi_controlboard_id] << std::endl;
                    return false;
                }
            }
            else
            {
                //Otherwise send all the commands individually
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPosDirectControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    ok = positionDirectInterface[wbi_controlboard_id]->setPosition(yarp_controlboard_axis,CTRL_RAD2DEG*ref[wbi_id]);
                }
            }
        }

        //////////////////////////////////////////////////////////
        //Sending references for velocity controlled joints
        //////////////////////////////////////////////////////////
        int nrOfVelocityControlledJointsInControlBoard = controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id].size();
        if( nrOfVelocityControlledJointsInControlBoard > 0 )
        {
            if( nrOfVelocityControlledJointsInControlBoard == totalJointsInControlBoard[wbi_controlboard_id] )
            {
                //If the wbi controls all the joint in the control board, use the usual interface setPositions
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfVelocityControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    buf_references[yarp_controlboard_axis] = CTRL_RAD2DEG*ref[wbi_id];
                }
                ok = ivel[wbi_controlboard_id]->velocityMove(buf_references);
                if(!ok)
                {
                    std::cerr << "yarpWholeBodyActuators::setControlReference error:"
                              << " unable to command velocities for controlboard " << controlBoardNames[wbi_controlboard_id] << std::endl;
                    return false;
                }
            }
            else
            {
                if( !isICubSimulator(robot) )
                {
                    //Otherwise send all the commands together, but packing them for iVelocityControl interface
                    for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfVelocityControlledJointsInControlBoard; controlBoard_jnt++ )
                    {
                        int wbi_id = controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                        int yarp_controlboard_axis =  controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                        buf_references[controlBoard_jnt] = CTRL_RAD2DEG*ref[wbi_id];
                        buf_controlledJoints[controlBoard_jnt] = yarp_controlboard_axis;
                    }
                    ok = ivel[wbi_controlboard_id]->velocityMove(nrOfVelocityControlledJointsInControlBoard,buf_controlledJoints,buf_references);
                    if(!ok)
                    {
                        std::cerr << "yarpWholeBodyActuators::setControlReference error:"
                              << " unable to command velocities for controlboard " << controlBoardNames[wbi_controlboard_id] << std::endl;
                        return false;
                    }
                }
                else
                {
                    //iCub simulator does not support the new velocityMove, so fall back to the old single joint control mode
                    for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfVelocityControlledJointsInControlBoard; controlBoard_jnt++ )
                    {
                        int wbi_id = controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                        int yarp_controlboard_axis =  controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                        ok = ivel[wbi_controlboard_id]->velocityMove(yarp_controlboard_axis,CTRL_RAD2DEG*ref[wbi_id]);
                    }
                }

            }
        }

        //////////////////////////////////////////////////////////
        //Sending references for torque controlled joints
        //////////////////////////////////////////////////////////
        int nrOfTorqueControlledJointsInControlBoard = controlledJointsForControlBoard.torqueControlledJoints[wbi_controlboard_id].size();
        if( nrOfTorqueControlledJointsInControlBoard > 0 )
        {
            if( nrOfTorqueControlledJointsInControlBoard == totalJointsInControlBoard[wbi_controlboard_id] )
            {
                //If the wbi controls all the joint in the control board, use the usual interface setPositions
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfTorqueControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.torqueControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.torqueControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    buf_references[yarp_controlboard_axis] = ref[wbi_id];
                }
                #ifdef WBI_ICUB_COMPILE_PARAM_HELP
                if (_torqueModuleConnection)
                {
                    _torqueRefs[wbi_id] = ref[wbi_id];
                }
                else
                #endif
                    ok = itrq[wbi_controlboard_id]->setRefTorques(buf_references);
                if(!ok)
                {
                    std::cerr << "yarpWholeBodyActuators::setControlReference error:"
                              << " unable to command position for controlboard " << controlBoardNames[wbi_controlboard_id] << std::endl;
                    return false;
                }
            }
            else
            {
                //Otherwise send all the commands individually
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfTorqueControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.torqueControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.torqueControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    ok = itrq[wbi_controlboard_id]->setRefTorque(yarp_controlboard_axis,ref[wbi_id]);
                }
            }
        }

        //////////////////////////////////////////////////////////
        //Sending references for pwm controlled joints
        //////////////////////////////////////////////////////////
        int nrOfPWMControlledJointsInControlBoard = controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id].size();
        if( nrOfPWMControlledJointsInControlBoard > 0 )
        {
            if( nrOfPWMControlledJointsInControlBoard == totalJointsInControlBoard[wbi_controlboard_id] )
            {
                //If the wbi controls all the joint in the control board, use the usual interface setPositions
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPWMControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    buf_references[yarp_controlboard_axis] = ref[wbi_id];
                }
                ok = iopl[wbi_controlboard_id]->setOutputs(buf_references);
                if(!ok)
                {
                    std::cerr << "yarpWholeBodyActuators::setControlReference error:"
                              << " unable to command position for controlboard " << controlBoardNames[wbi_controlboard_id] << std::endl;
                    return false;
                }
            }
            else
            {
                //Otherwise send all the commands individually
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPWMControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    ok = iopl[wbi_controlboard_id]->setOutput(yarp_controlboard_axis,ref[wbi_id]);
                }
            }
        }

    }


#ifdef WBI_ICUB_COMPILE_PARAM_HELP
    //TEMP
    if (_torqueModuleConnection)
    {
        ok = ok && _torqueModuleConnection->sendStreamParams();
    }
    //END TEMP
#endif

    return ok;
}

bool yarpWholeBodyActuactors::setControlParam(ControlParam paramId, const void *value, int joint)
{
    switch(paramId)
    {
        case CTRL_PARAM_REF_VEL: return setReferenceSpeed((double*)value, joint);
        case CTRL_PARAM_KP: return false;
        case CTRL_PARAM_KD: return false;
        case CTRL_PARAM_KI: return false;
        case CTRL_PARAM_OFFSET: return setControlOffset((double*)value, joint);
        default: break;
    }
    return false;
}

bool yarpWholeBodyActuactors::setReferenceSpeed(double *rspd, int joint)
{
    if(joint>=dof)
    {
        return false;
    }

    if(joint>=0)
    {
        int bodyPart = bodyPartAxisList[jnt].first;
        int controlBoardJointAxis = bodyPartAxisList[jnt].second;
        return ipos[bodyPart]->setRefSpeed(controlBoardJointAxis, CTRL_RAD2DEG*(*rspd));
    }

    bool ok = true;
    for(int jnt=0; jnt < dof; jnt++ )
    {
        int bodyPart = bodyPartAxisList[jnt].first;
        int controlBoardJointAxis = bodyPartAxisList[jnt].second;
        ok = ok && ipos[bodyPart]->setRefSpeed(controlBoardJointAxis, CTRL_RAD2DEG*rspd[jnt]);
    }

    return ok;
}

ControlMode yarpWholeBodyActuactors::yarpToWbiCtrlMode(int yarpCtrlMode)
{
    switch(yarpCtrlMode)
    {
    case VOCAB_CM_TORQUE:   return CTRL_MODE_TORQUE;
    case VOCAB_CM_POSITION: return CTRL_MODE_POS;
    case VOCAB_CM_VELOCITY: return CTRL_MODE_VEL;
    case VOCAB_CM_OPENLOOP: return CTRL_MODE_MOTOR_PWM;
    }
    return CTRL_MODE_UNKNOWN;
}


bool yarpWholeBodyActuactors::setPIDGains(const double *pValue, const double *dValue, const double *iValue, int joint)
{
    //The FOR_ALL atomicity is debated in github.. currently do the same as the rest of the library
    bool result = true;
    if (joint < 0) {
        return false;
//        FOR_ALL(itBp, itJ) {
//            switch (currentCtrlModes[LocalId(itBp->first,*itJ)]) {
//                case wbi::CTRL_MODE_TORQUE:
//                {
//                    //this is wrong...
////                    Pid currentPid;
////                    result = itrq[itBp->first]->getTorquePid(joint, &currentPid);
////                    if (!result) break;
////                    if (pValue != NULL)
////                        currentPid.kp = *pValue;
////                    if (dValue != NULL)
////                        currentPid.kd = *dValue;
////                    if (iValue != NULL)
////                        currentPid.ki = *iValue;
////                    result = itrq[itBp->first]->setTorquePid(joint, currentPid);
////                    break;
//                }
//                default:
//                    break;
//            }
//        }
    }
    else {
        int bodyPart = bodyPartAxisList[joint].first;
        int controlBoardJointAxis = bodyPartAxisList[joint].second;
        switch (currentCtrlModes[li]) {
            case wbi::CTRL_MODE_TORQUE:
            {
                Pid currentPid;
                result = itrq[bodyPart]->getTorquePid(controlBoardJointAxis, &currentPid);
                if (!result) break;
                if (pValue != NULL)
                    currentPid.kp = *pValue;
                if (dValue != NULL)
                    currentPid.kd = *dValue;
                if (iValue != NULL)
                    currentPid.ki = *iValue;
                result = itrq[bodyPart]->setTorquePid(controlBoardJointAxis, currentPid);
                break;
            }
            default:
                break;
        }
    }
    return result;
}


bool yarpWholeBodyActuactors::setControlOffset(const double *value, int joint)
{
    //The FOR_ALL atomicity is debated in github.. currently do the same as the rest of the library
    if (!value) return false;

    bool result = true;
    if (joint < 0) {
        //todo
        result = false;
    }
    else {
        switch (currentCtrlModes[joint]) {
            case wbi::CTRL_MODE_TORQUE:
            {
                int bodyPart = bodyPartAxisList[joint].first;
                int controlBoardJointAxis = bodyPartAxisList[joint].second;
                Pid currentPid;
                result = itrq[bodyPart]->getTorquePid(controlBoardJointAxis, &currentPid);
                if (!result) break;
                currentPid.offset = *value;
                result = itrq[bodyPart]->setTorquePid(controlBoardJointAxis, currentPid);
                break;
            }
            default:
                break;
        }
    }
    return result;
}
