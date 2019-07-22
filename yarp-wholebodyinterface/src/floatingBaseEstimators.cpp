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

#include "floatingBaseEstimators.h"

#include <wbi/wbiUtil.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>

namespace yarpWbi
{

//////////////////////////////////////////////////////////////////////////////
/// localFloatingBaseStateEstimator methods
//////////////////////////////////////////////////////////////////////////////

localFloatingBaseStateEstimator::localFloatingBaseStateEstimator(wbi::iWholeBodyModel * _wholeBodyModel, int _dof):
    wholeBodyModel(NULL),
    robot_reference_frame_link(-1),
    luDecompositionOfBaseJacobian(6)
{
    init(_wholeBodyModel,_dof);
}

bool localFloatingBaseStateEstimator::init(wbi::iWholeBodyModel * _wholeBodyModel, int _dof)
{
    wholeBodyModel = _wholeBodyModel;
    return changeDoF(_dof);
}

bool localFloatingBaseStateEstimator::changeDoF(int _dof)
{
    dof = _dof;
    complete_jacobian.resize(6,dof+6);
    return true;
}



bool localFloatingBaseStateEstimator::setWorldBaseLinkName(std::string linkName)
{
    if (!wholeBodyModel) {
        yError("setWorldBaseLinkName: Model not yet created");
        return false;
    }

    bool found = wholeBodyModel->getFrameList().idToIndex(linkName.c_str(), robot_reference_frame_link);
    if (!found || robot_reference_frame_link < 0)
    {
        return false;
    }
    return true;
}

bool localFloatingBaseStateEstimator::computeBasePosition(double *q_temp, double * base_pos_estimate)
{
    if (!wholeBodyModel) return false;

    wholeBodyModel->computeH(q_temp,wbi::Frame::identity(),robot_reference_frame_link, rootLink_H_ReferenceLink);

    referenceLink_H_rootLink = rootLink_H_ReferenceLink.getInverse();
    world_H_rootLink = world_H_reference*referenceLink_H_rootLink ;

    wbi::serializationFromFrame(world_H_rootLink, base_pos_estimate);

    return true;
}
bool localFloatingBaseStateEstimator::computeBaseVelocity(double* qj, double* dqj, double* base_vel_estimate)
{
    if (!wholeBodyModel) return false;

    complete_jacobian.setZero();
    Eigen::Map<Eigen::VectorXd> dqjVect(dqj, dof);
    Eigen::Map<Eigen::VectorXd> baseVelocityWrapper(base_vel_estimate, 6);
    baseVelocityWrapper.setZero();

    wholeBodyModel->computeJacobian(qj, world_H_rootLink, robot_reference_frame_link, complete_jacobian.data());
    luDecompositionOfBaseJacobian.compute(complete_jacobian.leftCols<6>());

    baseVelocityWrapper =-luDecompositionOfBaseJacobian.solve(complete_jacobian.rightCols(dof) * dqjVect);

    return true;
}


//////////////////////////////////////////////////////////////////////////////
/// localFloatingBaseStateEstimator methods
//////////////////////////////////////////////////////////////////////////////

remoteFloatingBaseStateEstimator::remoteFloatingBaseStateEstimator():
    callbackHandler(this->buff_mutex),
    measureAvailable(false)
{
    callbackHandler.setBuffers(this->base_pos_estimate,
                               this->base_vel_estimate,
                               this->base_acc_estimate,
                               &(this->lastTimestamp));
    callbackHandler.setFlags(&(this->measureAvailable));
}


bool remoteFloatingBaseStateEstimator::open(yarp::os::Searchable &config)
{
    yarp::os::LockGuard guard(buff_mutex);

    yarp::os::ConstString carrier = config.check("carrier", yarp::os::Value("udp"), "default carrier for streaming floating base state").asString().c_str();

    timeoutLimit_in_seconds = config.check("timeoutLimit", yarp::os::Value(1.0), "time limit after which a timeout is raised").asDouble();


    local.clear();
    remote.clear();

    local  = config.find("local").asString().c_str();
    remote = config.find("remote").asString().c_str();

    if (local=="")
    {
        yError("remoteFloatingBaseEstimator::open() error you have to provide valid local name");
        return false;
    }
    if (remote=="")
    {
        yError("remoteFloatingBaseEstimator::open() error you have to provide valid remote name");
        return false;
    }

    if (!inputPort.open(local.c_str()))
    {
        yError("remoteFloatingBaseEstimator::open() error could not open port %s, check network",local.c_str());
        return false;
    }
    inputPort.useCallback(callbackHandler);

    bool ok=yarp::os::Network::connect(remote.c_str(), local.c_str(), carrier.c_str());
    if (!ok)
    {
        yError("remoteFloatingBaseEstimator::open() error could not connect to %s", remote.c_str());
        return false;
    }

    return true;
}

bool remoteFloatingBaseStateEstimator::close()
{
    yarp::os::LockGuard guard(buff_mutex);
    yarp::os::Network::disconnect(remote.c_str(), local.c_str());
    inputPort.close();
    return true;
}

bool remoteFloatingBaseStateEstimator::getBasePosition(double* _base_pos_estimate)
{
    yarp::os::LockGuard guard(buff_mutex);
    updateTimeout();
    if( measureAvailable )
    {
        memcpy(_base_pos_estimate,this->base_pos_estimate,BASE_POS_ESTIMATE_SIZE*sizeof(double));
        return true;
    }
    else
    {
        yWarning("remoteFloatingBaseStateEstimator::getBasePosition called, but no floating base pos estimate is available");
        return false;
    }
}

bool remoteFloatingBaseStateEstimator::getBaseVelocity(double* _base_vel_estimate)
{
    yarp::os::LockGuard guard(buff_mutex);
    updateTimeout();
    if( measureAvailable )
    {
        memcpy(_base_vel_estimate,this->base_vel_estimate,BASE_VEL_ESTIMATE_SIZE*sizeof(double));
        return true;
    }
    else
    {
        yWarning("remoteFloatingBaseStateEstimator::getBaseVelocity called, but no floating base vel estimate is available");
        return false;
    }
}

bool remoteFloatingBaseStateEstimator::getBaseAcceleration(double* _base_acc_estimate)
{
    yarp::os::LockGuard guard(buff_mutex);
    updateTimeout();
    if( measureAvailable )
    {
        memcpy(_base_acc_estimate,this->base_acc_estimate,BASE_ACC_ESTIMATE_SIZE*sizeof(double));
        return true;
    }
    else
    {
        yWarning("remoteFloatingBaseStateEstimator::getBaseAcceleration called, but no floating base vel estimate is available");
        return false;
    }
}

bool remoteFloatingBaseStateEstimator::getBaseState(double* _base_pos_estimate,
                                                    double* _base_vel_estimate,
                                                    double* _base_acc_estimate)
{
    yarp::os::LockGuard guard(buff_mutex);
    updateTimeout();
    if( measureAvailable )
    {
        memcpy(_base_pos_estimate,this->base_pos_estimate,BASE_POS_ESTIMATE_SIZE*sizeof(double));
        memcpy(_base_vel_estimate,this->base_vel_estimate,BASE_VEL_ESTIMATE_SIZE*sizeof(double));
        memcpy(_base_acc_estimate,this->base_acc_estimate,BASE_ACC_ESTIMATE_SIZE*sizeof(double));
        return true;
    }
    else
    {
        yWarning("remoteFloatingBaseStateEstimator::getBaseState called, but no floating base state estimate is available");
        return false;
    }
}

void remoteFloatingBaseStateEstimator::updateTimeout()
{
    if( !measureAvailable ) return;

    double now = yarp::os::Time::now();
    if( (now - lastTimestamp) > timeoutLimit_in_seconds )
    {
        yError("remoteFloatingBaseStateEstimator::updateTimeout() : timeout detected");
        measureAvailable = false;
    }

    return;
}


remoteFloatingBaseStatePortProcessor::remoteFloatingBaseStatePortProcessor(yarp::os::Mutex & _mutex):
    base_pos_mat(4,4),
    base_vel_vec(6),
    base_acc_vec(6),
    mutex(_mutex),
    base_pos_estimate_buf(0),
    base_vel_estimate_buf(0),
    base_acc_estimate_buf(0),
    measureAvailableFlag(0)
{
}



void remoteFloatingBaseStatePortProcessor::setBuffers(double* _base_pos_estimate_buf,
                                                      double* _base_vel_estimate_buf,
                                                      double* _base_acc_estimate_buf,
                                                      double* _lastTimestamp
                                                     )
{
    base_pos_estimate_buf = _base_pos_estimate_buf;
    base_vel_estimate_buf = _base_vel_estimate_buf;
    base_acc_estimate_buf = _base_acc_estimate_buf;
    lastTimestamp         = _lastTimestamp;
    return;
}

void remoteFloatingBaseStatePortProcessor::setFlags(bool* _measureAvailableFlag)
{
    this->measureAvailableFlag = _measureAvailableFlag;
}

bool bot2Matrix(yarp::os::Bottle * p_bot, yarp::sig::Matrix & matrix)
{
    int rows = p_bot->get(0).asDouble();
    int cols = p_bot->get(1).asDouble();
    if( rows != 4 || cols != 4 )
    {
        return false;
    }

    matrix.resize(rows,cols);
    yarp::os::Bottle * data_bot = p_bot->get(2).asList();

    if( data_bot->size() != rows*cols )
    {
        return false;
    }

    for(int row =0; row < rows; row++ )
    {
        for(int col = 0; col < cols; col++ )
        {
            matrix(row,col) = data_bot->get(cols*row+col).asDouble();
        }
    }

    return true;
}

bool bot2Vector(yarp::os::Bottle * p_bot, yarp::sig::Vector & vec )
{
    if( p_bot->size() != 6 )
    {
        return false;
    }

    for(int i=0; i < 6; i++ )
    {
        vec(i) = p_bot->get(i).asDouble();
    }

    return true;
}

void remoteFloatingBaseStatePortProcessor::onRead(yarp::os::Bottle& b)
{
    yarp::os::LockGuard guard(mutex);

    // process the bottle
    if( b.size() != 3 ||
        !(b.get(0).isList()) ||
        !(b.get(1).isList()) ||
        !(b.get(2).isList()) )
    {
        yError("remoteFloatingBaseStatePortProcessor::onRead : bottle is not made up of 3 list");
        return;
    }


    // Try to read the base matrix
    bool ok = true;
    ok = bot2Matrix(b.get(0).asList(),base_pos_mat);
    ok = ok && bot2Vector(b.get(1).asList(),base_vel_vec);
    ok = ok && bot2Vector(b.get(2).asList(),base_acc_vec);

    if( !ok )
    {
        yError("remoteFloatingBaseStatePortProcessor::onRead : deserialization failed");
        return;
    }

    // copy the data
    memcpy(base_pos_estimate_buf,base_pos_mat.data(),BASE_POS_ESTIMATE_SIZE*sizeof(double));
    memcpy(base_vel_estimate_buf,base_vel_vec.data(),BASE_VEL_ESTIMATE_SIZE*sizeof(double));
    memcpy(base_acc_estimate_buf,base_acc_vec.data(),BASE_ACC_ESTIMATE_SIZE*sizeof(double));

    *lastTimestamp = yarp::os::Time::now();
    *measureAvailableFlag = true;

    return;
}


}
