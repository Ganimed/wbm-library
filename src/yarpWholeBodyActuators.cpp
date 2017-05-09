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
#include "yarpWholeBodyActuators.h"
#include <wbi/wbiConstants.h>
#include <wbi/Error.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <string>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;

#define WAIT_TIME 0.001         ///< waiting time in seconds before retrying to perform an operation that has failed
#define DEFAULT_REF_SPEED 10.0  ///< default reference joint speed for the joint position control

const std::string yarpWbi::YarpWholeBodyActuatorsPropertyInteractionModeKey = "yarp.dev.interaction";
const std::string yarpWbi::YarpWholeBodyActuatorsPropertyInteractionModeStiff = "yarp.dev.interaction.stiff";
const std::string yarpWbi::YarpWholeBodyActuatorsPropertyInteractionModeCompliant = "yarp.dev.interaction.compliant";
const std::string yarpWbi::YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey = "yarp.dev.impedance.stiffness";
const std::string yarpWbi::YarpWholeBodyActuatorsPropertyImpedanceDampingKey = "yarp.dev.impedance.damping";

#ifdef YARPWBI_YARP_HAS_LEGACY_IOPENLOOP
#define VOCAB_CM_PWM VOCAB_CM_OPENLOOP
#endif


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY ACTUATOR
// *********************************************************************************************************************
// *********************************************************************************************************************


yarpWholeBodyActuators::yarpWholeBodyActuators(const char* _name,
                                               const yarp::os::Property & yarp_wbi_properties)
: initDone(false), name(_name), wbi_yarp_properties(yarp_wbi_properties)
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
        std::cerr << "[ERR] yarpWholeBodyActuators::openDrivers error: called with bodypart " << bp <<
                     " but the total number of bodyparts considered in the interface is " << controlBoardNames.size() << std::endl;
        return false;
    }
    itrq[bp]=0; iimp[bp]=0; icmd[bp]=0; ivel[bp]=0; ipos[bp]=0; iopl[bp]=0;  dd[bp]=0; ipositionDirect[bp]=0; iinteraction[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], controlBoardNames[bp].c_str()))
    {
        std::cerr << "[ERR] yarpWholeBodyActuators::openDrivers error: unable to open controlboard " << controlBoardNames[bp]
                  << "of robot " << robot  << std::endl;
        return false;
    }

    //Open all necessary interfaces
    //
    // Conditional definition of open loop for compatibility with both YARP master and devel:
    // The iopl pointer is passed to the device driver that implements the interface. The view
    // function is using a dynamic cast which doesn't work properly on a (void*), so we use a
    // proper type here (IPWMControl*) or (IOpenLoopControl*).
#ifndef YARPWBI_YARP_HAS_LEGACY_IOPENLOOP
    IPWMControl * typed_iopl = 0;
#else
    IOpenLoopControl * typed_iopl = 0;
#endif

    bool ok = dd[bp]->view(itrq[bp]) && dd[bp]->view(iimp[bp]) && dd[bp]->view(icmd[bp])
              && dd[bp]->view(ivel[bp]) && dd[bp]->view(ipos[bp]) && dd[bp]->view(typed_iopl)
              && dd[bp]->view(ipositionDirect[bp]) && dd[bp]->view(iinteraction[bp]);
    iopl[bp] = typed_iopl; // copy to iopl which is a (void*)

    if(!ok)
    {
        std::cerr << "[ERR] yarpWholeBodyActuators::openDrivers error: unable to open all necessary interfaces of " <<
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

    if (ok)
    {
        //Update internal structure reference the controlled joints for each controlboard
        currentCtrlModes.resize(jointIdList.size(), wbi::CTRL_MODE_POS);

        totalAxesInControlBoard.resize(controlBoardNames.size());
        for (int i = 0; i < (int)totalAxesInControlBoard.size(); i++)
        {
            totalAxesInControlBoard[i] = 0;
        }

        totalControlledAxesInControlBoard.resize(controlBoardNames.size());
        for (int i = 0; i < (int)totalControlledAxesInControlBoard.size(); i++)
        {
            totalControlledAxesInControlBoard[i] = 0;
        }

        for (int wbi_jnt = 0; wbi_jnt < (int)jointIdList.size(); wbi_jnt++)
        {
            //std::cout << "-------------wbi_jnt " << wbi_jnt << " " << controlBoardAxisList[wbi_jnt].first  << " " << controlBoardAxisList[wbi_jnt].second << std::endl;
            totalControlledAxesInControlBoard[controlBoardAxisList[wbi_jnt].first]++;
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
        for (int bp = 0; bp < (int)controlBoardNames.size(); bp++)
        {
            ok = openControlBoardDrivers(bp);
            if (!ok)
            {
                //If there is an error, close all the opened driver and return
                int lastDriverOpenedCorrectly = bp - 1;
                for (int driverToClose = lastDriverOpenedCorrectly; driverToClose >= 0; driverToClose--)
                {
                    if (dd[driverToClose] != 0)
                    {
                        dd[driverToClose]->close();
                        delete dd[driverToClose];
                        dd[driverToClose] = 0;
                    }
                }
                break;
            }
        }

        if (ok)
        {
            //All drivers opened without errors, save the dimension of all used controlboards
            for (int ctrlBrd = 0; ctrlBrd < (int)controlBoardNames.size(); ctrlBrd++)
            {
                ok = ipos[ctrlBrd]->getAxes(&(totalAxesInControlBoard[ctrlBrd]));
                if (!ok)
                {
                    break;
                }
            }
        }
    }

    if (!ok)
    {
        //roll back the changes: all vectors must be sized 0
        itrq.resize(0);
        iimp.resize(0);
        icmd.resize(0);
        ivel.resize(0);
        ipos.resize(0);
        iopl.resize(0);
        ipositionDirect.resize(0);
        iinteraction.resize(0);
        dd.resize(0);
        controlBoardAxisList.resize(0);

        return false;
    }

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

    return ok;
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

bool yarpWholeBodyActuators::setControlModeSingleJoint(ControlMode controlMode, double *ref, int joint)
{
    if (!initDone) return false;

    bool ok = false;
    ///< check that joint is not already in the specified control mode
    // commented out for now
    if(currentCtrlModes[joint]!=controlMode)
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
                if( ok && ref )
                {
                    itrq[bodyPart]->setRefTorque(controlBoardJointAxis, *ref);
                }
                break;
            case CTRL_MODE_MOTOR_PWM:
                ok = icmd[bodyPart]->setControlMode(controlBoardJointAxis,VOCAB_CM_PWM);
                break;
            default:
                break;
        }

        if(ok)
        {
            currentCtrlModes[joint] = controlMode;
            this->updateControlledJointsForEachControlBoard();

            if(ref != 0)
            {
                setControlReference(ref,joint);
            }
        } else {
            fprintf(stderr, "yarpWholeBodyActuators: Cannot set control mode %d on joint %d \n", controlMode, joint);
        }
    }

    return ok;
}

bool yarpWholeBodyActuators::setControlMode(ControlMode controlMode, double *ref, int joint)
{
    if (!initDone) return false;

    if(joint>=(int)jointIdList.size())
    {
        return false;
    }

    bool ok = true;
    ///< set all joints to the specified control mode
    if(joint<0)
    {
        for(int j=0; j < (int)jointIdList.size(); j++ )
        {
            if(ref != 0)
            {
                setControlModeSingleJoint(controlMode,ref+j,j);
            }
            else
            {
                setControlModeSingleJoint(controlMode,0,j);
            }
        }
    }
    else //set a single joint
    {
        assert(joint >=0 && joint < (int)jointIdList.size());
        setControlModeSingleJoint(controlMode,ref,joint);
    }

    return ok;
}

bool yarpWholeBodyActuators::setControlReference(double *ref, int joint)
{
    if (!initDone) return false;

    //std::cout << "~~~~~~~~~~~~ setControlReference called " << std::endl;
    if(joint> (int)jointIdList.size())
        return false;

    bool ok = true;
    if(joint>=0)    // set control reference for the specified joint
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardAxis = controlBoardAxisList[joint].second;

        bool ret_value = false;
        switch(currentCtrlModes[joint])
        {
            case CTRL_MODE_POS:
                ret_value = ipos[bodyPart]->positionMove(controlBoardAxis, yarpWbi::Rad2Deg * (*ref));
                break;
            case CTRL_MODE_DIRECT_POSITION:
                ret_value = ipositionDirect[bodyPart]->setPosition(controlBoardAxis, yarpWbi::Rad2Deg * (*ref));
                break;
            case CTRL_MODE_VEL:
                ret_value = ivel[bodyPart]->velocityMove(controlBoardAxis, yarpWbi::Rad2Deg * (*ref));
                break;
            case CTRL_MODE_TORQUE:
            {
                ret_value = itrq[bodyPart]->setRefTorque(controlBoardAxis, *ref);
            }
                break;
            case CTRL_MODE_MOTOR_PWM:
#ifndef YARPWBI_YARP_HAS_LEGACY_IOPENLOOP
                ret_value = ((IPWMControl*)iopl[bodyPart])->setRefDutyCycle(controlBoardAxis, *ref);
#else
                ret_value = ((IOpenLoopControl*)iopl[bodyPart])->setRefOutput(controlBoardAxis, *ref);
#endif
                break;
            default:
                ret_value = false;
        }
        return ret_value;
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
                     buf_references[yarp_controlboard_axis] = yarpWbi::Rad2Deg * ref[wbi_id];
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
                    ok = ipos[wbi_controlboard_id]->positionMove(yarp_controlboard_axis,yarpWbi::Rad2Deg*ref[wbi_id]);
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
                    buf_references[yarp_controlboard_axis] = yarpWbi::Rad2Deg*ref[wbi_id];
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
                    ok = ipositionDirect[wbi_controlboard_id]->setPosition(yarp_controlboard_axis,yarpWbi::Rad2Deg*ref[wbi_id]);
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
                    buf_references[yarp_controlboard_axis] = yarpWbi::Rad2Deg*ref[wbi_id];
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
                        buf_references[controlBoard_jnt] = yarpWbi::Rad2Deg*ref[wbi_id];
                        buf_controlledJoints[controlBoard_jnt] = yarp_controlboard_axis;
                    }
                    ok = ivel[wbi_controlboard_id]->velocityMove(nrOfVelocityControlledJointsInControlBoard, buf_controlledJoints, buf_references);
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
                    ok = itrq[wbi_controlboard_id]->setRefTorque(yarp_controlboard_axis, ref[wbi_id]);
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
                    int yarp_controlboard_axis = controlledJointsForControlBoard.pwmControlledJoints[wbi_controlboard_id][controlBoard_jnt].yarp_controlboard_axis;
                    buf_references[yarp_controlboard_axis] = ref[wbi_id];
                }
#ifndef YARPWBI_YARP_HAS_LEGACY_IOPENLOOP
                ok = ((IPWMControl*)iopl[wbi_controlboard_id])->setRefDutyCycles(buf_references);
#else
                ok = ((IOpenLoopControl*)iopl[wbi_controlboard_id])->setRefOutputs(buf_references);
#endif
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
#ifndef YARPWBI_YARP_HAS_LEGACY_IOPENLOOP
                    ok = ((IPWMControl*)iopl[wbi_controlboard_id])->setRefDutyCycle(yarp_controlboard_axis,ref[wbi_id]);
#else
                    ok = ((IOpenLoopControl*)iopl[wbi_controlboard_id])->setRefOutput(yarp_controlboard_axis,ref[wbi_id]);
#endif
                }
            }
        }

    }

    return ok;
}

bool yarpWholeBodyActuators::setControlParam(ControlParam paramId, const void *value, int joint)
{
    if (!initDone || !value) return false;
    switch(paramId)
    {
        case CTRL_PARAM_REF_VEL: { return setReferenceSpeed((double*)value, joint);}
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
    if (!initDone || !rspd) return false;
    if(joint>=(int)jointIdList.size())
    {
        return false;
    }

    if(joint>=0)
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;
        return ipos[bodyPart]->setRefSpeed(controlBoardJointAxis, yarpWbi::Rad2Deg*(*rspd));
    }

    bool ok = true;
    for(int jnt=0; jnt < (int)jointIdList.size(); jnt++ )
    {
        int bodyPart = controlBoardAxisList[jnt].first;
        int controlBoardJointAxis = controlBoardAxisList[jnt].second;

        ok = ok && ipos[bodyPart]->setRefSpeed(controlBoardJointAxis, yarpWbi::Rad2Deg*rspd[jnt]);
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
    case VOCAB_CM_PWM: return CTRL_MODE_MOTOR_PWM;
    }
    return CTRL_MODE_UNKNOWN;
}


bool yarpWholeBodyActuators::setPIDGains(const double *pValue, const double *dValue, const double *iValue, int joint)
{
    if (!initDone) return false;
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

bool yarpWholeBodyActuators::setPIDGains(yarp::dev::Pid *pids, wbi::ControlMode controlMode, int joint)
{
    if (!initDone) return false;
    bool result = true;
    if (joint < 0) {
        int i = 0;
        switch (controlMode) {
            case wbi::CTRL_MODE_TORQUE:
                for (std::vector< std::pair<int,int> >::const_iterator jointPair = controlBoardAxisList.begin();
                     jointPair != controlBoardAxisList.end(); ++jointPair) {
                    result = result && itrq[jointPair->first]->setTorquePid(jointPair->second, pids[i++]);
                }
                break;
            default:
                break;
        }
    } else {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;
        switch (controlMode) {
            case wbi::CTRL_MODE_TORQUE:
            {
                result = itrq[bodyPart]->setTorquePid(controlBoardJointAxis, *pids);
                break;
            }
            default:
                break;
        }
    }
    return result;
}

bool yarpWholeBodyActuators::getPIDGains(yarp::dev::Pid *pids, wbi::ControlMode controlMode, int joint)
{
    if (!initDone) return false;
    bool result = true;
    if (joint < 0) {
        int i = 0;
        switch (controlMode) {
            case wbi::CTRL_MODE_TORQUE:
                for (std::vector< std::pair<int,int> >::const_iterator jointPair = controlBoardAxisList.begin();
                     jointPair != controlBoardAxisList.end(); ++jointPair) {
                    result = result && itrq[jointPair->first]->getTorquePid(jointPair->second, &pids[i++]);
                }
                break;
            default:
                break;

        }
    } else {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;
        switch (controlMode) {
            case wbi::CTRL_MODE_TORQUE:
            {
                result = itrq[bodyPart]->getTorquePid(controlBoardJointAxis, pids);
                break;
            }
            default:
                break;
        }
    }
    return result;
}

bool yarpWholeBodyActuators::setMotorTorqueParameters(const yarp::dev::MotorTorqueParameters *motorParameters, int joint)
{
    if (!initDone) return false;
    bool result = true;
    if (joint < 0) {
        int i = 0;
        for (std::vector< std::pair<int,int> >::const_iterator jointPair = controlBoardAxisList.begin();
             jointPair != controlBoardAxisList.end(); ++jointPair) {
            result = result && itrq[jointPair->first]->setMotorTorqueParams(jointPair->second, motorParameters[i++]);
        }
    }
    else {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;
        result = itrq[bodyPart]->setMotorTorqueParams(controlBoardJointAxis, *motorParameters);
    }
    return result;
}

bool yarpWholeBodyActuators::getMotorTorqueParameters(yarp::dev::MotorTorqueParameters *motorParameters, int joint)
{
    if (!initDone) return false;
    bool result = true;
    if (joint < 0) {
        int i = 0;
        for (std::vector< std::pair<int,int> >::const_iterator jointPair = controlBoardAxisList.begin();
             jointPair != controlBoardAxisList.end(); ++jointPair) {
            result = result && itrq[jointPair->first]->getMotorTorqueParams(jointPair->second, &motorParameters[i++]);
        }
    }
    else {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardJointAxis = controlBoardAxisList[joint].second;

        result = itrq[bodyPart]->getMotorTorqueParams(controlBoardJointAxis, motorParameters);

    }
    return result;
}

bool yarpWholeBodyActuators::setControlOffset(const double *value, int joint)
{
    if (!initDone) return false;
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

bool yarpWholeBodyActuators::setControlProperty(const std::string key, const std::string value, int joint, ::wbi::Error *error)
{
    //supported keys:
    //- interaction => {values: stiff, complaint}
    //- Stiffness => double
    //- Damping => double

    if (key == YarpWholeBodyActuatorsPropertyInteractionModeKey) {
        yarp::dev::InteractionModeEnum interactioMode = yarp::dev::VOCAB_IM_UNKNOWN;
        if (value == YarpWholeBodyActuatorsPropertyInteractionModeCompliant)
        {
            interactioMode = yarp::dev::VOCAB_IM_COMPLIANT;
        }
        else if (value == YarpWholeBodyActuatorsPropertyInteractionModeStiff)
        {
            interactioMode = yarp::dev::VOCAB_IM_STIFF;
        }
        if (interactioMode == yarp::dev::VOCAB_IM_UNKNOWN)
        {
            if (error)
            {
                error->setError(ErrorDomain, ErrorCodeConfigurationNotValid, "Value not supported for Interaction property");
            }
            yError("Value not supported for Interaction property");
            return false; //interaction mode not supported
        }
        return setInteractionModeSingleJoint(interactioMode, joint, error);
    }
    else {
        if (key == YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey) {
            yarp::os::Value doubleValue;
            doubleValue.fromString(value.c_str());
            return setImpedanceStiffness(doubleValue.asDouble(), joint, error);

        } else if(key == YarpWholeBodyActuatorsPropertyImpedanceDampingKey) {
            yarp::os::Value doubleValue;
            doubleValue.fromString(value.c_str());
            return setImpedanceDamping(doubleValue.asDouble(), joint, error);
        } else {
            if (error) {
                error->setError(ErrorDomain, ErrorCodePropertyNotSupported, "Property key not supported");
            }
            yError("Property key not supported");
            return false; //not supported yet
        }
    }
}

bool yarpWholeBodyActuators::getControlProperty(std::string key, std::string &value, int joint, ::wbi::Error *error) const
{
    if (key == YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey
        || key == YarpWholeBodyActuatorsPropertyImpedanceDampingKey) {
        if (joint > (int)jointIdList.size())
        {
            if (error)
            {
                error->setError(ErrorDomain, ErrorCodeIndexOutOfRange, "Index out of range");
            }
            yError("Control board index out of range");
            return false;
        }

        if (joint < 0)
        {
            if (error)
            {
                error->setError(ErrorDomain, ErrorCodeNotImplementedYet, "Interaction mode for all the robot not supported yet");
            }
            yError("Interaction mode for all the robot not supported yet");
            return false; //not supported yet
        }
        else
        {
            int bodyPart = controlBoardAxisList[joint].first;
            int controlBoardAxis = controlBoardAxisList[joint].second;
            double stiffness = 0;
            double damping = 0;
            bool result = iimp[bodyPart]->getImpedance(controlBoardAxis, &stiffness, &damping);
            if (key == YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey) {
                value = Value(stiffness).toString();
            } else {
                value = Value(damping).toString();
            }
            return result;
        }

    } else {
        if (error) {
            error->setError(ErrorDomain, ErrorCodePropertyNotSupported, "Property key not supported");
        }
        yError("Property key not supported");
        return false; //not supported yet
    }
}

bool yarpWholeBodyActuators::setInteractionModeSingleJoint(yarp::dev::InteractionModeEnum mode, int joint, ::wbi::Error *error)
{
    if (joint > (int)jointIdList.size())
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeIndexOutOfRange, "Index out of range");
        }
        yError("Control board index out of range");
        return false;
    }

    if (joint < 0)
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeNotImplementedYet, "Interaction mode for all the robot not supported yet");
        }
        yError("Interaction mode for all the robot not supported yet");
        return false; //not supported yet
    }
    else
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardAxis = controlBoardAxisList[joint].second;
        return iinteraction[bodyPart]->setInteractionMode(controlBoardAxis, mode);
    }

    return false;
}

bool yarpWholeBodyActuators::setImpedanceStiffness(double stiffness, int joint, wbi::Error *error)
{
    if (joint > (int)jointIdList.size())
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeIndexOutOfRange, "Index out of range");
        }
        yError("Control board index out of range");
        return false;
    }

    if (joint < 0)
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeNotImplementedYet, "Interaction mode for all the robot not supported yet");
        }
        yError("Interaction mode for all the robot not supported yet");
        return false; //not supported yet
    }
    else
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardAxis = controlBoardAxisList[joint].second;
        double oldStiffness = 0; double damping = 0;
        iimp[bodyPart]->getImpedance(controlBoardAxis, &oldStiffness, &damping);
        yDebug("Setting stiffness to control board %d, axis %d, value %lf, damp %lf", bodyPart, controlBoardAxis, stiffness, damping);
        return iimp[bodyPart]->setImpedance(controlBoardAxis, stiffness, damping);
    }
}

bool yarpWholeBodyActuators::setImpedanceDamping(double damping, int joint, wbi::Error *error)
{
    if (joint > (int)jointIdList.size())
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeIndexOutOfRange, "Index out of range");
        }
        yError("Control board index out of range");
        return false;
    }

    if (joint < 0)
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeNotImplementedYet, "Interaction mode for all the robot not supported yet");
        }
        yError("Interaction mode for all the robot not supported yet");
        return false; //not supported yet
    }
    else
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardAxis = controlBoardAxisList[joint].second;
        double stiffness = 0; double oldDamping = 0;
        iimp[bodyPart]->getImpedance(controlBoardAxis, &stiffness, &oldDamping);
        yDebug("Setting damping to control board %d, axis %d, value %lf, damp %lf", bodyPart, controlBoardAxis, stiffness, damping);
        return iimp[bodyPart]->setImpedance(controlBoardAxis, stiffness, damping);
    }
}

bool yarpWholeBodyActuators::setFullImpedance(double stiffness, double damping, int joint, wbi::Error *error)
{
    if (joint > (int)jointIdList.size())
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeIndexOutOfRange, "Index out of range");
        }
        yError("Control board index out of range");
        return false;
    }

    if (joint < 0)
    {
        if (error)
        {
            error->setError(ErrorDomain, ErrorCodeNotImplementedYet, "Interaction mode for all the robot not supported yet");
        }
        yError("Interaction mode for all the robot not supported yet");
        return false; //not supported yet
    }
    else
    {
        int bodyPart = controlBoardAxisList[joint].first;
        int controlBoardAxis = controlBoardAxisList[joint].second;
        return iimp[bodyPart]->setImpedance(controlBoardAxis, stiffness, damping);
    }
}
