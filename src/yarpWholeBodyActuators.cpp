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

#define MAX_NJ 20
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
const std::string yarpWholeBodyActuators::icubWholeBodyActuatorsUseExternalTorqueModule = "icubWholeBodyActuactorsUseExternalTorqueModuleKey";
const std::string yarpWholeBodyActuators::icubWholeBodyActuatorsExternalTorqueModuleAutoconnect = "icubWholeBodyActuactorsExternalTorqueModuleNameKey";
const std::string yarpWholeBodyActuators::icubWholeBodyActuatorsExternalTorqueModuleName = "icubWholeBodyActuactorsExternalTorqueModuleAutoconnect";

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY ACTUATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
//yarpWholeBodyActuators::yarpWholeBodyActuators(const char* _name,
//                                               const char* _robotName,
//                                               const std::vector<std::string> &_bodyPartNames)
//: m_commandedParts(0), initDone(false), jointIdList.size()(0), name(_name), robot(_robotName), bodyPartNames(_bodyPartNames), reverse_torso_joints(true)
//#ifdef WBI_ICUB_COMPILE_PARAM_HELP
//,_torqueModuleConnection(0)
//#endif
//{}



yarpWholeBodyActuators::yarpWholeBodyActuators(const char* _name,
                                               const yarp::os::Property & yarp_wbi_properties)
: initDone(false), name(_name), wbi_yarp_properties(yarp_wbi_properties)
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
,_torqueModuleConnection(0)
#endif
{
}



yarpWholeBodyActuators::~yarpWholeBodyActuators()
{
    close();
}


bool yarpWholeBodyActuators::setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties)
{
    wbi_yarp_properties = yarp_wbi_properties;
    return true;
}

bool yarpWholeBodyActuators::getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties)
{
    yarp_wbi_properties = wbi_yarp_properties;
    return true;
}

bool yarpWholeBodyActuators::openControlBoardDrivers(int bp)
{
    if( bp >= (int)controlBoardNames.size() || bp < 0 )
    {
        std::cerr << "yarpWholeBodyActuators::openDrivers error: called with bodypart " << bp <<
                     " but the total number of bodyparts considered in the interface is " << controlBoardNames.size() << std::endl;
        return false;
    }
    itrq[bp]=0; iimp[bp]=0; icmd[bp]=0; ivel[bp]=0; ipos[bp]=0; iopl[bp]=0;  dd[bp]=0; ipositionDirect[bp]=0; iinteraction[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], controlBoardNames[bp].c_str()))
    {
        std::cerr << "yarpWholeBodyActuators::openDrivers error: enable to open controlboard " << controlBoardNames[bp]
                  << "of robot " << robot  << std::endl;
        return false;
    }

    //Open all necessary interfaces
    bool ok = dd[bp]->view(itrq[bp]) && dd[bp]->view(iimp[bp]) && dd[bp]->view(icmd[bp])
              && dd[bp]->view(ivel[bp]) && dd[bp]->view(ipos[bp]) && dd[bp]->view(iopl[bp])
              && dd[bp]->view(ipositionDirect[bp]) && dd[bp]->view(iinteraction[bp]);


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
    if( this->initDone ) return true;

    //Function return value
    bool ok = false;

    //Loading configuration
    if( wbi_yarp_properties.check("robot") )
    {
        robot = wbi_yarp_properties.find("robot").asString().c_str();
    }
    else if (wbi_yarp_properties.check("robotName") )
    {
        std::cerr << "[WARN] yarpWholeBodyActuators: robot option not found, using robotName" << std::endl;
        robot = wbi_yarp_properties.find("robotName").asString().c_str();
    }
    else
    {
        std::cerr << "[ERR] yarpWholeBodyActuators: robot option not found" << std::endl;
        return false;
    }


    ok = loadJointsControlBoardFromConfig(wbi_yarp_properties,
                                               jointIdList,
                                               controlBoardNames,
                                               controlBoardAxisList);

    if(!ok) return false;

    //Update internal structure reference the controlled joints for each controlboard
    currentCtrlModes.resize(jointIdList.size(),wbi::CTRL_MODE_POS);

    totalAxesInControlBoard.resize(controlBoardNames.size());
    for(int i=0; i < (int)totalAxesInControlBoard.size(); i++ )
    {
        totalAxesInControlBoard[i] = 0;
    }


    totalControlledAxesInControlBoard.resize(controlBoardNames.size());
    for(int i=0; i < (int)totalControlledAxesInControlBoard.size(); i++ )
    {
        totalControlledAxesInControlBoard[i] = 0;
    }

    for(int wbi_jnt=0; wbi_jnt < (int)jointIdList.size(); wbi_jnt++ )
    {
        //std::cout << "-------------wbi_jnt " << wbi_jnt << " " << controlBoardAxisList[wbi_jnt].first  << " " << controlBoardAxisList[wbi_jnt].second << std::endl;
        totalControlledAxesInControlBoard[ controlBoardAxisList[wbi_jnt].first ]++;
    }

    updateControlledJointsForEachControlBoard();

    //Resize everything that depends on the number of controlboards
    itrq.resize(controlBoardNames.size());
    iimp.resize(controlBoardNames.size());
    icmd.resize(controlBoardNames.size());
    ivel.resize(controlBoardNames.size());
    ipos.resize(controlBoardNames.size());
    iopl.resize(controlBoardNames.size());
    ipositionDirect.resize(controlBoardNames.size());
    iinteraction.resize(controlBoardNames.size());
    dd.resize(controlBoardNames.size());


    //Open necessary yarp controlboard drivers
    //iterate all used body parts
    for(int bp=0; bp < (int)controlBoardNames.size(); bp++ )
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
    for(int ctrlBrd = 0; ctrlBrd < (int)controlBoardNames.size(); ctrlBrd++ )
    {
        ok = ipos[ctrlBrd]->getAxes(&(totalAxesInControlBoard[ctrlBrd]));
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
        found = wbi_yarp_properties.find(yarpWholeBodyActuatorsExternalTorqueModuleAutoconnect.c_str());

        if (!found.isNull() && found.isBool()) {
            _rpcAutoConnect = found.asBool();
        }
        found = wbi_yarp_properties.find(yarpWholeBodyActuatorsUseExternalTorqueModule.c_str());
        if (!found.isNull() && found.isBool() && found.asBool()) {
            found = wbi_yarp_properties.find(yarpWholeBodyActuatorsExternalTorqueModuleName.c_str());
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
    positionControlledJoints.resize(nrOfControlBoards,std::vector<yarpWBAControlledJoint>(0));
    positionDirectedControlledJoints.clear();
    positionDirectedControlledJoints.resize(nrOfControlBoards,std::vector<yarpWBAControlledJoint>(0));
    velocityControlledJoints.clear();
    velocityControlledJoints.resize(nrOfControlBoards,std::vector<yarpWBAControlledJoint>(0));
    torqueControlledJoints.clear();
    torqueControlledJoints.resize(nrOfControlBoards,std::vector<yarpWBAControlledJoint>(0));
    pwmControlledJoints.clear();
    pwmControlledJoints.resize(nrOfControlBoards,std::vector<yarpWBAControlledJoint>(0));
    return true;
}


bool yarpWholeBodyActuators::updateControlledJointsForEachControlBoard()
{
    controlledJointsForControlBoard.reset(controlBoardNames.size());

    #ifndef NDEBUG
    for(int wbi_jnt = 0; wbi_jnt < (int)jointIdList.size(); wbi_jnt++ )
    {
        int wbi_jnt_controlboard_id = controlBoardAxisList[wbi_jnt].first;

        assert(controlledJointsForControlBoard.positionControlledJoints[wbi_jnt_controlboard_id].size() == 0);
        assert(controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_jnt_controlboard_id].size() == 0);
        assert(controlledJointsForControlBoard.velocityControlledJoints[wbi_jnt_controlboard_id].size() == 0);
        assert(controlledJointsForControlBoard.torqueControlledJoints[wbi_jnt_controlboard_id].size() == 0);
        assert(controlledJointsForControlBoard.pwmControlledJoints[wbi_jnt_controlboard_id].size() == 0);

    }
    #endif

    for(int wbi_jnt = 0; wbi_jnt < (int)jointIdList.size(); wbi_jnt++ )
    {
        //std::cout << "wbi_jnt : " << wbi_jnt << std::endl;
        //std::cout << "jointIdList " << jointIdList.toString() << std::endl;

        wbi::ControlMode wbi_jnt_ctrl_mode = currentCtrlModes[wbi_jnt];

        int wbi_jnt_controlboard_id = controlBoardAxisList[wbi_jnt].first;
        int wbi_jnt_yarp_axis = controlBoardAxisList[wbi_jnt].second;

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
                assert(false);
                return false;
            break;
        }
    }

    #ifndef NDEBUG
    int total_controlled_joints = 0;
    assert(totalAxesInControlBoard.size() == controlBoardNames.size());
    for(int wbi_ctrlBoard = 0; wbi_ctrlBoard < (int)controlBoardNames.size(); wbi_ctrlBoard++ )
    {
        int controlled_joints = 0;
        controlled_joints +=  controlledJointsForControlBoard.positionControlledJoints[wbi_ctrlBoard].size();
        controlled_joints +=  controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_ctrlBoard].size();
        controlled_joints +=  controlledJointsForControlBoard.velocityControlledJoints[wbi_ctrlBoard].size();
        controlled_joints +=  controlledJointsForControlBoard.torqueControlledJoints[wbi_ctrlBoard].size();
        controlled_joints +=  controlledJointsForControlBoard.pwmControlledJoints[wbi_ctrlBoard].size();
//        int tot = totalAxesInControlBoard[wbi_ctrlBoard];
        assert(controlled_joints == totalControlledAxesInControlBoard[wbi_ctrlBoard]);
        total_controlled_joints += controlled_joints;
    }
    assert(total_controlled_joints == (int)this->getActuatorList().size());
    #endif

    return true;
}


bool yarpWholeBodyActuators::close()
{
    bool ok = true;
    for(int ctrlBrd=0; ctrlBrd < (int)controlBoardNames.size(); ctrlBrd++ )
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

bool yarpWholeBodyActuators::setConfigurationParameter(const std::string &parameterName, const yarp::os::Value &parameterValue)
{
    /*Note to developers: we can move the functionalities offered by the configuration map to an external class in order to make it more generic */
    if (initDone) return false;
    //check allowed parameters
    if (parameterName.compare(icubWholeBodyActuatorsUseExternalTorqueModule) == 0) {
        if (parameterValue.isBool()) {
            wbi_yarp_properties.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    } else if (parameterName.compare(icubWholeBodyActuatorsExternalTorqueModuleAutoconnect) == 0) {
        if (parameterValue.isBool()) {
            wbi_yarp_properties.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    } else if (parameterName.compare(icubWholeBodyActuatorsExternalTorqueModuleName) == 0) {
        //simply check value has some length
        if (parameterValue.isString() && parameterValue.asString().length() > 0) {
            wbi_yarp_properties.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    }

    return false;
}

bool yarpWholeBodyActuators::removeActuator(const ID &j)
{
    if (initDone) return false;

    if(!jointIdList.removeID(j))
        return false;
    return true;
}

bool yarpWholeBodyActuators::addActuator(const ID &j)
{
    if (initDone) return false;

    if(!jointIdList.addID(j))
        return false;

    return true;
}

int yarpWholeBodyActuators::addActuators(const IDList &jList)
{
    if (initDone) return false;

    int count = jointIdList.addIDList(jList);
    return count;
}

bool yarpWholeBodyActuators::setControlMode(ControlMode controlMode, double *ref, int joint)
{
    if(joint>=(int)jointIdList.size())
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
                for(int j=0; j < (int)jointIdList.size(); j++ )
                {
                    //Set only the joints that are not in the desired control mode
                    if(currentCtrlModes[j] != controlMode) {

                        ok = ok && icmd[controlBoardAxisList[j].first]->setPositionMode(controlBoardAxisList[j].second);
                        if( !ok )
                        {
                            std::cerr << "yarpWholeBodyActuators::setControlMode error: setPositionMode on axis " <<
                                         controlBoardAxisList[j].second << " of controlBoard " << controlBoardNames[controlBoardAxisList[j].first] << std::endl;
                        }
                    }
                }
                break;

            case CTRL_MODE_DIRECT_POSITION:
                for(int j=0; j < (int)jointIdList.size(); j++ )
                {
                    //Set only the joints that are not in the desired control mode
                    if(currentCtrlModes[j] != controlMode) {
                        ok = ok && icmd[controlBoardAxisList[j].first]->setControlMode(controlBoardAxisList[j].second,VOCAB_CM_POSITION_DIRECT);
                        if( !ok )
                        {
                            std::cerr << "yarpWholeBodyActuators::setControlMode error: setDirectionPositionMode on axis " <<
                                         controlBoardAxisList[j].second << " of controlBoard " << controlBoardNames[controlBoardAxisList[j].first] << std::endl;
                        }
                    }
                }
                break;

            case CTRL_MODE_VEL:
                for(int j=0; j < (int)jointIdList.size(); j++ )
                {
                    if(currentCtrlModes[j] != controlMode)
                    {
                        ok = ok && icmd[controlBoardAxisList[j].first]->setVelocityMode(controlBoardAxisList[j].second);
                    }
                }
                break;

            case CTRL_MODE_TORQUE:
                for(int j=0; j < (int)jointIdList.size(); j++ )
                {
                    if(currentCtrlModes[j]!=controlMode)
                    {
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
                        controlModeChanged = true;
                        if (_torqueModuleConnection) {
                            //if torque control connection is true I do not set the torqueMode
                            ok = ok && true;
                        }
                        else
#endif
                        {
                            ok = ok && icmd[controlBoardAxisList[j].first]->setTorqueMode(controlBoardAxisList[j].second);
                        }
                    }
                }
                break;

            case CTRL_MODE_MOTOR_PWM:
                    for(int j=0; j < (int)jointIdList.size(); j++ )
                    {
                        if(currentCtrlModes[j] != controlMode)
                        {
                            ok = ok && icmd[controlBoardAxisList[j].first]->setOpenLoopMode(controlBoardAxisList[j].second);
                        }
                    }
                break;

            default:
                return false;
        }


        //Update internal structure
        ok = ok && this->updateControlledJointsForEachControlBoard();



        if(ok)
        {
            for(int j=0; j < (int)jointIdList.size(); j++ )
            {
                currentCtrlModes[j] = controlMode;
                if(ref!=0)
                    ok = ok && setControlReference(ref);
                    if( !ok )
                    {
                        std::cerr << "yarpWholeBodyActuators::setControlMode error: setControlReference on jnt " << j << " failed " << std::endl;
                    }
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
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;
        switch(controlMode)
        {
            case CTRL_MODE_POS:
                ok = icmd[bodyPart]->setControlMode(controlBoardJointAxis,VOCAB_CM_POSITION);
                ok = ok && iinteraction[bodyPart]->setInteractionMode(controlBoardJointAxis,VOCAB_IM_STIFF);
                break;
            case CTRL_MODE_DIRECT_POSITION:
                ok = icmd[bodyPart]->setControlMode(controlBoardJointAxis,VOCAB_CM_POSITION_DIRECT);
                ok = ok && iinteraction[bodyPart]->setInteractionMode(controlBoardJointAxis,VOCAB_IM_STIFF);
                break;
            case CTRL_MODE_VEL:
                ok = icmd[bodyPart]->setControlMode(controlBoardJointAxis,VOCAB_CM_VELOCITY);
                ok = ok && iinteraction[bodyPart]->setInteractionMode(controlBoardJointAxis,VOCAB_IM_STIFF);
                break;
            case CTRL_MODE_TORQUE:
                ok = icmd[bodyPart]->setControlMode(controlBoardJointAxis,VOCAB_CM_TORQUE);
                break;
            case CTRL_MODE_MOTOR_PWM:
                ok = icmd[bodyPart]->setControlMode(controlBoardJointAxis,VOCAB_CM_OPENLOOP);
                break;
            default:
                break;
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
    //std::cout << "~~~~~~~~~~~~ setControlReference called " << std::endl;
    if(joint> (int)jointIdList.size())
        return false;

    bool ok = true;
    if(joint>=0)    // set control reference for the specified joint
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardAxis = controlBoardAxisList[joint].second;

        switch(currentCtrlModes[joint])
        {
            case CTRL_MODE_POS:
                return ipos[bodyPart]->positionMove(controlBoardAxis, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_DIRECT_POSITION:
                return ipositionDirect[bodyPart]->setPosition(controlBoardAxis, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_VEL:
                return ivel[bodyPart]->velocityMove(controlBoardAxis, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_TORQUE:
            {
                return itrq[bodyPart]->setRefTorque(controlBoardAxis, *ref);
            }
            case CTRL_MODE_MOTOR_PWM:
                return iopl[bodyPart]->setRefOutput(controlBoardAxis, *ref);
            default: break;
        }
        return false;
    }

    //Buffer variables
    double buf_references[MAX_NJ];
    int buf_controlledJoints[MAX_NJ];

    // set control references for all joints
    for(int wbi_controlboard_id=0; wbi_controlboard_id < (int)controlBoardNames.size(); wbi_controlboard_id++ )
    {
        ///////////////////////////////////////////////////
        //Sending references for position controlled joints
        ///////////////////////////////////////////////////
        int nrOfPosControlledJointsInControlBoard = controlledJointsForControlBoard.positionControlledJoints[wbi_controlboard_id].size();
        if( nrOfPosControlledJointsInControlBoard > 0 )
        {
             if( nrOfPosControlledJointsInControlBoard == totalAxesInControlBoard[wbi_controlboard_id] )
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
                    std::cerr << "[ERR] yarpWholeBodyActuators::setControlReference error:"
                              << "[ERR]  unable to setPositions for controlboard " << controlBoardNames[wbi_controlboard_id] << std::endl;
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
            if( nrOfPosDirectControlledJointsInControlBoard == totalAxesInControlBoard[wbi_controlboard_id] )
            {
                //If the wbi controls all the joint in the control board, use the usual interface setPositions
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPosDirectControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.positionDirectedControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    buf_references[yarp_controlboard_axis] = CTRL_RAD2DEG*ref[wbi_id];
                }
                ok = ipositionDirect[wbi_controlboard_id]->setPositions(buf_references);
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
                    ok = ipositionDirect[wbi_controlboard_id]->setPosition(yarp_controlboard_axis,CTRL_RAD2DEG*ref[wbi_id]);
                }
            }
        }

        //////////////////////////////////////////////////////////
        //Sending references for velocity controlled joints
        //////////////////////////////////////////////////////////
        int nrOfVelocityControlledJointsInControlBoard = controlledJointsForControlBoard.velocityControlledJoints[wbi_controlboard_id].size();
        if( nrOfVelocityControlledJointsInControlBoard > 0 )
        {
            if( nrOfVelocityControlledJointsInControlBoard == totalAxesInControlBoard[wbi_controlboard_id] )
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

            }
        }

        //////////////////////////////////////////////////////////
        //Sending references for torque controlled joints
        //////////////////////////////////////////////////////////
        int nrOfTorqueControlledJointsInControlBoard = controlledJointsForControlBoard.torqueControlledJoints[wbi_controlboard_id].size();
        if( nrOfTorqueControlledJointsInControlBoard > 0 )
        {
            if( nrOfTorqueControlledJointsInControlBoard == totalAxesInControlBoard[wbi_controlboard_id] )
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
            if( nrOfPWMControlledJointsInControlBoard == totalAxesInControlBoard[wbi_controlboard_id] )
            {
                //If the wbi controls all the joint in the control board, use the usual interface setPositions
                for( int controlBoard_jnt = 0; controlBoard_jnt < nrOfPWMControlledJointsInControlBoard; controlBoard_jnt++ )
                {
                    int wbi_id = controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id][controlBoard_jnt].wbi_id;
                    int yarp_controlboard_axis =  controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    buf_references[yarp_controlboard_axis] = ref[wbi_id];
                }
                ok = iopl[wbi_controlboard_id]->setRefOutputs(buf_references);
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
                    ok = iopl[wbi_controlboard_id]->setRefOutput(yarp_controlboard_axis,ref[wbi_id]);
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

bool yarpWholeBodyActuators::setControlParam(ControlParam paramId, const void *value, int joint)
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

bool yarpWholeBodyActuators::setReferenceSpeed(double *rspd, int joint)
{
    if(joint>=(int)jointIdList.size())
    {
        return false;
    }

    if(joint>=0)
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;
        return ipos[bodyPart]->setRefSpeed(controlBoardJointAxis, CTRL_RAD2DEG*(*rspd));
    }

    bool ok = true;
    for(int jnt=0; jnt < (int)jointIdList.size(); jnt++ )
    {
        int bodyPart = controlBoardAxisList[jnt].first;
        int controlBoardJointAxis = controlBoardAxisList[jnt].second;
        ok = ok && ipos[bodyPart]->setRefSpeed(controlBoardJointAxis, CTRL_RAD2DEG*rspd[jnt]);
    }

    return ok;
}

ControlMode yarpWholeBodyActuators::yarpToWbiCtrlMode(int yarpCtrlMode)
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


bool yarpWholeBodyActuators::setPIDGains(const double *pValue, const double *dValue, const double *iValue, int joint)
{
    //The FOR_ALL atomicity is debated in github.. currently do the same as the rest of the library
    bool result = true;
    if (joint < 0) {
        return false;
//        FOR_ALL(itBp, itJ) {
//            switch (currentCtrlModes[ID(itBp->first,*itJ)]) {
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
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;
        switch (currentCtrlModes[joint]) {
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


bool yarpWholeBodyActuators::setControlOffset(const double *value, int joint)
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
                int bodyPart = controlBoardAxisList[joint].first;
                int controlBoardJointAxis = controlBoardAxisList[joint].second;
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
