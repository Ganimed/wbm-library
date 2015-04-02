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

#include "yarpWholeBodyInterface/floatingBaseEstimators.h"


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
    if(wholeBodyModel!=NULL)
    {
        wholeBodyModel->getFrameList().idToIndex(linkName.c_str(),robot_reference_frame_link);
        if( robot_reference_frame_link < 0 )
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }

}

bool localFloatingBaseStateEstimator::computeBasePosition(double *q_temp, double * base_pos_estimate)
{
  if(wholeBodyModel!=NULL)
  {
      wholeBodyModel->computeH(q_temp,wbi::Frame::identity(),robot_reference_frame_link, rootLink_H_ReferenceLink);

      referenceLink_H_rootLink = rootLink_H_ReferenceLink.getInverse();
      world_H_rootLink = world_H_reference*referenceLink_H_rootLink ;

      int ctr;
      for (ctr=0;ctr<3;ctr++)
      {
        base_pos_estimate[ctr] = world_H_rootLink.p[ctr];
      }
      for (ctr=0;ctr<9;ctr++)
      {
        base_pos_estimate[3+ctr] = world_H_rootLink.R.data[ctr];
      }
      return(true);
  }
  else
        return(false);
}
bool localFloatingBaseStateEstimator::computeBaseVelocity(double* qj, double* dqj, double* base_vel_estimate)
{
    if(wholeBodyModel!=NULL)
    {
        complete_jacobian.setZero();
        Eigen::Map<Eigen::VectorXd> dqjVect(dqj, dof);
        Eigen::Map<Eigen::VectorXd> baseVelocityWrapper(base_vel_estimate, 6);
        baseVelocityWrapper.setZero();

        wholeBodyModel->computeJacobian(qj, world_H_rootLink, robot_reference_frame_link, complete_jacobian.data());
        luDecompositionOfBaseJacobian.compute(complete_jacobian.leftCols<6>());

        baseVelocityWrapper =-luDecompositionOfBaseJacobian.solve(complete_jacobian.rightCols(dof) * dqjVect);

        return true;
    }
    else
    {
        return false;
    }
}


//////////////////////////////////////////////////////////////////////////////
/// localFloatingBaseStateEstimator methods
//////////////////////////////////////////////////////////////////////////////

remoteFloatingBaseStateEstimator::remoteFloatingBaseStateEstimator()
{

}


bool remoteFloatingBaseStateEstimator::open(yarp::os::Searchable &config)
{
    ConstString carrier = config.check("carrier", Value("udp"), "default carrier for streaming floating base state").asString().c_str();

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
    inputPort.useCallback();

    bool ok=Network::connect(remote.c_str(), local.c_str(), carrier.c_str());
    if (!ok)
    {
        yError("remoteFloatingBaseEstimator::open() error could not connect to %s", remote.c_str());
        return false;
    }

    return true;
}

bool remoteFloatingBaseEstimator::close()
{
    inputPort.close();
    return true;
}

}
