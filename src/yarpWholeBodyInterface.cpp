/**
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences -
 *Istituto Italiano di Tecnologia
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

#include "yarpWholeBodyInterface.h"
#include <iCub/skinDynLib/common.h>
#include <yarp/os/Os.h>
#include <string>
#include <cassert>

#include "yarpWholeBodyActuators.h"
#include "yarpWholeBodyModel.h"
#include "yarpWholeBodyStates.h"

using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#define MAX_NJ 20
#define WAIT_TIME 0.001

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY INTERFACE
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyInterface::yarpWholeBodyInterface(
    const char* _name, const yarp::os::Property& _yarp_wbi_properties)
{
    actuatorInt = new yarpWholeBodyActuators((_name + string("actuator")).c_str(),
        _yarp_wbi_properties);
    modelInt = new yarpWholeBodyModel((_name + string("model")).c_str(),
        _yarp_wbi_properties);
    modelForStateInt = new yarpWholeBodyModel((_name + string("model")).c_str(),
        _yarp_wbi_properties);
    stateInt = new yarpWholeBodyStates((_name + string("state")).c_str(),
        _yarp_wbi_properties, modelForStateInt);
}

yarpWholeBodyInterface::~yarpWholeBodyInterface() { close(); }

yarpWholeBodyActuators* yarpWholeBodyInterface::wholeBodyActuator()
{
    return actuatorInt;
}
yarpWholeBodyModel* yarpWholeBodyInterface::wholeBodyModel()
{
    return modelInt;
}
yarpWholeBodyStates* yarpWholeBodyInterface::wholeBodyState()
{
    return stateInt;
}

bool yarpWholeBodyInterface::setYarpWbiProperties(
    const yarp::os::Property& yarp_wbi_properties)
{
    actuatorInt->setYarpWbiProperties(yarp_wbi_properties);
    stateInt->setYarpWbiProperties(yarp_wbi_properties);
    modelInt->setYarpWbiProperties(yarp_wbi_properties);
    modelForStateInt->setYarpWbiProperties(yarp_wbi_properties);
    return true;
}

bool yarpWholeBodyInterface::getYarpWbiProperties(
    yarp::os::Property& yarp_wbi_properties)
{
    yarp::os::Property buffer;
    actuatorInt->getYarpWbiProperties(buffer);
    yarp_wbi_properties.fromString(buffer.toString(), false);
    stateInt->getYarpWbiProperties(buffer);
    yarp_wbi_properties.fromString(buffer.toString(), false);
    modelInt->getYarpWbiProperties(buffer);
    yarp_wbi_properties.fromString(buffer.toString(), false);
    return true;
}

bool yarpWholeBodyInterface::init()
{
    bool ok = actuatorInt->init();
    if (!ok)
        printf(
            "[ERR] Error while initializing yarpWholeBodyActuators interface.\n");
    if (ok)
        ok = modelInt->init();
    if (!ok)
        printf("[ERR] Error while initializing yarpWholeBodyModel interface.\n");
    if (ok)
        ok = modelForStateInt->init();
    if (!ok)
        printf("[ERR] Error while initializing yarpWholeBodyModel interface.\n");
    if (ok)
        ok = stateInt->init();
    if (!ok)
        printf("[ERR] Error while initializing yarpWholeBodyStates interface.\n");
    return ok;
}

bool yarpWholeBodyInterface::close()
{
    bool ok = true;
    if (actuatorInt) {
        if (actuatorInt->close()) {
            delete actuatorInt;
            actuatorInt = 0;
        }
        else {
            ok = false;
        }
    }

    if (stateInt) {
        if (stateInt->close()) {
            delete stateInt;
            stateInt = 0;
        }
        else {
            ok = false;
        }
    }

    if (modelForStateInt) {
        if (modelForStateInt->close()) {
            delete modelForStateInt;
            modelForStateInt = 0;
        }
        else {
            ok = false;
        }
    }

    if (modelInt) {
        if (modelInt->close()) {
            delete modelInt;
            modelInt = 0;
        }
        else {
            ok = false;
        }
    }

    return ok;
}

bool yarpWholeBodyInterface::removeJoint(const ID& j)
{
    bool ok = actuatorInt->removeActuator(j);
    for (int i = 0; ok && i < JOINT_ESTIMATE_TYPES_SIZE; i++)
        ok = stateInt->removeEstimate(jointEstimateTypes[i], j);
    // removing pos removes also vel and acc estimation
    ok = ok && modelForStateInt->removeJoint(j);

    return ok ? modelInt->removeJoint(j) : false;
}

bool yarpWholeBodyInterface::addJoint(const ID& j)
{
    bool ok = actuatorInt->addActuator(j);
    for (int i = 0; ok && i < JOINT_ESTIMATE_TYPES_SIZE; i++)
        ok = stateInt->addEstimate(jointEstimateTypes[i], j);
    ok = ok && modelForStateInt->addJoint(j);
    return ok ? modelInt->addJoint(j) : false;
}

int yarpWholeBodyInterface::addJoints(const IDList& jList)
{
    int res1 = actuatorInt->addActuators(jList);
    for (int i = 0; i < JOINT_ESTIMATE_TYPES_SIZE; i++)
        stateInt->addEstimates(jointEstimateTypes[i], jList);
    // adding pos adds also vel and acc estimation
    int res2 = modelForStateInt->addJoints(jList);
    int res4 = modelInt->addJoints(jList);
    assert(res1 == res4);
    return res1 && res2 && res4;
}

yarp::os::Mutex& yarpWholeBodyInterface::getInterfaceMutex()
{
    return wbiMutex;
}

bool yarpWholeBodyInterface::removeActuator(const wbi::ID& j)
{
    return actuatorInt->removeActuator(j);
}

bool yarpWholeBodyInterface::addActuator(const wbi::ID& j)
{
    return actuatorInt->addActuator(j);
}

int yarpWholeBodyInterface::addActuators(const wbi::IDList& j)
{
    return actuatorInt->addActuators(j);
}

const wbi::IDList& yarpWholeBodyInterface::getActuatorList()
{
    return actuatorInt->getActuatorList();
}
bool yarpWholeBodyInterface::setControlMode(wbi::ControlMode cm, double* ref,
    int jnt)
{
    return actuatorInt->setControlMode(cm, ref, jnt);
}
bool yarpWholeBodyInterface::setControlReference(double* ref, int jnt)
{
    return actuatorInt->setControlReference(ref, jnt);
}
bool yarpWholeBodyInterface::setControlParam(wbi::ControlParam parId, const void* val, int jnt)
{
    return actuatorInt->setControlParam(parId, val, jnt);
}

// STATES
bool yarpWholeBodyInterface::addEstimate(const wbi::EstimateType st,
    const wbi::ID& sid)
{
    return false;
}
int yarpWholeBodyInterface::addEstimates(const wbi::EstimateType st,
    const wbi::IDList& sids)
{
    return 0;
}
bool yarpWholeBodyInterface::removeEstimate(const wbi::EstimateType st,
    const wbi::ID& sid)
{
    return false;
}
const wbi::IDList&
yarpWholeBodyInterface::getEstimateList(const wbi::EstimateType st)
{
    return empty_id_list;
}
int yarpWholeBodyInterface::getEstimateNumber(const wbi::EstimateType st)
{
    return 0;
}
bool yarpWholeBodyInterface::getEstimate(const wbi::EstimateType et,
    const int estimate_numeric_id,
    double* data, double time,
    bool blocking)
{
    return stateInt->getEstimate(et, estimate_numeric_id, data, time, blocking);
}
bool yarpWholeBodyInterface::getEstimates(const wbi::EstimateType et,
    double* data, double time,
    bool blocking)
{
    return stateInt->getEstimates(et, data, time, blocking);
}
bool yarpWholeBodyInterface::setEstimationParameter(
    const wbi::EstimateType et, const wbi::EstimationParameter ep,
    const void* value)
{
    return stateInt->setEstimationParameter(et, ep, value);
}

// MODEL
int yarpWholeBodyInterface::getDoFs() { return modelInt->getDoFs(); }
const wbi::IDList& yarpWholeBodyInterface::getJointList()
{
    return modelInt->getJointList();
}
const wbi::IDList& yarpWholeBodyInterface::getFrameList()
{
    return modelInt->getFrameList();
}
bool yarpWholeBodyInterface::getJointLimits(double* qMin, double* qMax,
    int joint)
{
    return modelInt->getJointLimits(qMin, qMax, joint);
}
bool yarpWholeBodyInterface::computeH(double* q, const wbi::Frame& xB,
    int linkId, wbi::Frame& H, double *pos)
{
    return modelInt->computeH(q, xB, linkId, H, pos);
}
bool yarpWholeBodyInterface::computeJacobian(double* q, const wbi::Frame& xB,
    int linkId, double* J,
    double* pos)
{
    return modelInt->computeJacobian(q, xB, linkId, J, pos);
}
bool yarpWholeBodyInterface::computeDJdq(double* q, const wbi::Frame& xB,
    double* dq, double* dxB, int linkId,
    double* dJdq, double* pos)
{
    return modelInt->computeDJdq(q, xB, dq, dxB, linkId, dJdq, pos);
}
bool yarpWholeBodyInterface::forwardKinematics(double* q, const wbi::Frame& xB,
    int linkId, double* x, double *pos)
{
    return modelInt->forwardKinematics(q, xB, linkId, x, pos);
}
bool yarpWholeBodyInterface::inverseDynamics(double* q, const wbi::Frame& xB,
    double* dq, double* dxB,
    double* ddq, double* ddxB,
    double* g, double* tau)
{
    return modelInt->inverseDynamics(q, xB, dq, dxB, ddq, ddxB, g, tau);
}
bool yarpWholeBodyInterface::computeMassMatrix(double* q, const wbi::Frame& xB,
    double* M)
{
    return modelInt->computeMassMatrix(q, xB, M);
}
bool yarpWholeBodyInterface::computeGeneralizedBiasForces(
    double* q, const wbi::Frame& xB, double* dq, double* dxB, double* g,
    double* h)
{
    return modelInt->computeGeneralizedBiasForces(q, xB, dq, dxB, g, h);
}
bool yarpWholeBodyInterface::computeCentroidalMomentum(double* q,
    const wbi::Frame& xB,
    double* dq, double* dxB,
    double* h)
{
    return modelInt->computeCentroidalMomentum(q, xB, dq, dxB, h);
}

bool yarpWholeBodyInterface::computeGravityBiasForces(double *q, const wbi::Frame &xBase, double* g, double *h)
{
    return modelInt->computeGravityBiasForces(q, xBase, g, h);
}
