/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
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

#include "yarpWholeBodyModelV2.h"
#include "yarpWbiUtil.h"

#include <string>
#include <cfloat>
#include <cmath>

#include <Eigen/Core>

#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/math.h>

#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/ResourceFinder.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRowMajor;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY MODEL
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyModelV2::yarpWholeBodyModelV2(const char* _name,
                                       const yarp::os::Property & _wbi_yarp_conf)
        : initDone(false),
          dof(0),
          wbi_yarp_properties(_wbi_yarp_conf),
          getLimitsFromControlBoard(false)
{
}

yarpWholeBodyModelV2::~yarpWholeBodyModelV2()
{
}

bool yarpWholeBodyModelV2::init()
{
    if( this->initDone ) return true;

    //Loading configuration
    if( wbi_yarp_properties.check("robot") )
    {
        robot = wbi_yarp_properties.find("robot").asString().c_str();
    }
    else if (wbi_yarp_properties.check("robotName") )
    {
        yWarning() << "yarpWholeBodyModel: robot option not found, using robotName";
        robot = wbi_yarp_properties.find("robotName").asString().c_str();
    }
    else
    {
        yError() << "yarpWholeBodyModel: robot option not found";
        return false;
    }

    if(  !wbi_yarp_properties.check("urdf") && !wbi_yarp_properties.check("urdf_file") )
    {
        yError() << "yarpWholeBodyModel error: urdf not found in configuration files";
        return false;
    }

    std::string urdf_file;
    if( wbi_yarp_properties.check("urdf") )
    {
        urdf_file = wbi_yarp_properties.find("urdf").asString().c_str();
    }
    else
    {
        urdf_file = wbi_yarp_properties.find("urdf_file").asString().c_str();
    }

    yarp::os::ResourceFinder rf;
    if(  wbi_yarp_properties.check("verbose") )
    {
        rf.setVerbose();
    }

    std::string urdf_file_path = rf.findFile(urdf_file.c_str());

    // Create the list of joints added to the WBI to load a reduced
    // model with just that joints
    std::vector<std::string> consideredJoints;
    consideredJoints.resize(jointIdList.size());
    for(int wbi_numeric_id =0;  wbi_numeric_id < (int)jointIdList.size(); wbi_numeric_id++ )
    {
        wbi::ID joint_id;
        jointIdList.indexToID(wbi_numeric_id,joint_id);
        consideredJoints[wbi_numeric_id] = joint_id.toString();
    }

    // Load the model
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadReducedModelFromFile(urdf_file_path,consideredJoints);

    if( !ok )
    {
        yError() << "yarpWholeBodyModel: impossible to load model from " << urdf_file_path << " init failed.";
        return false;
    }

    // Initialize the KinDynComputations object
    ok = m_kinDynComp.loadRobotModel(mdlLoader.model());

    if( !ok )
    {
        yError() << "yarpWholeBodyModel: impossible to create iDynTree::KinDynComputations, init failed.";
        return false;
    }

    //Loading information on the limits
    // if getLimitsFromControlBoard is set, get the limits from the real robot
    // otherwise use the limits in the urdf
    if( wbi_yarp_properties.check("getLimitsFromControlBoard") )
    {
        this->getLimitsFromControlBoard = true;
    }

    if( this->getLimitsFromControlBoard )
    {
        loadJointsControlBoardFromConfig(wbi_yarp_properties,
                                         jointIdList,
                                         controlBoardNames,
                                         controlBoardAxisList);

        dd.resize(controlBoardNames.size());
        ilim.resize(controlBoardNames.size());
    }

    // Populate the frame id list
    for(iDynTree::FrameIndex frameIdx=0;
        frameIdx < static_cast<iDynTree::FrameIndex>(m_kinDynComp.model().getNrOfFrames());
        frameIdx++ )
    {
        std::string frame_name = m_kinDynComp.model().getFrameName(frameIdx);
        frameIdList.addID(frame_name);
    }

    // Resize buffers
    m_modelPos.resize(m_kinDynComp.model());
    m_baseVel.zero();
    m_jntVel.resize(m_kinDynComp.model());
    m_baseAcc.zero();
    m_jntAcc.resize(m_kinDynComp.model());
    m_worldGravity.zero();
    m_zeroExtWrenches.resize(m_kinDynComp.model());
    m_zeroExtWrenches.zero();

    m_genTrqs.resize(m_kinDynComp.model());
    m_massMatrix.resize(m_kinDynComp.model());
    m_frameJacobian.resize(m_kinDynComp.model());
    m_momentumJacobian.resize(m_kinDynComp.model());
    m_comJacobian.resize(3,m_kinDynComp.getNrOfDegreesOfFreedom()+6);

    this->initDone = true;
    return this->initDone;
}

bool yarpWholeBodyModelV2::openDrivers(int bp)
{
    ilim[bp]=0; dd[bp]=0;
    if(!openPolyDriver(name+"model", robot, dd[bp], controlBoardNames[bp]))
        return false;
    bool ok = dd[bp]->view(ilim[bp]);   //if(!isRobotSimulator(robot))
    if(ok)
        return true;
    yError("Problem initializing drivers of %s", controlBoardNames[bp].c_str());
    return false;
}

bool yarpWholeBodyModelV2::closeDrivers()
{
    bool ok = true;
    for(int bp=0; bp < (int)controlBoardNames.size(); bp++ )
    {
        if( dd[bp] != 0 ) {
            ok = ok && dd[bp]->close();
            delete dd[bp];
            dd[bp] = 0;
        }
    }
    return ok;
}

bool yarpWholeBodyModelV2::close()
{
    return closeDrivers();
}

bool yarpWholeBodyModelV2::removeJoint(const wbi::ID &j)
{
    return false;
}

bool yarpWholeBodyModelV2::setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties)
{
    wbi_yarp_properties = yarp_wbi_properties;
    return true;
}

bool yarpWholeBodyModelV2::getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties)
{
    yarp_wbi_properties = wbi_yarp_properties;
    return true;
}

int yarpWholeBodyModelV2::getDoFs()
{
    return dof;
}

bool yarpWholeBodyModelV2::addJoint(const wbi::ID &j)
{
    if( initDone )
    {
        return false;
    }

    if(!jointIdList.addID(j))
    {
        return false;
    }

    return true;
}

int yarpWholeBodyModelV2::addJoints(const wbi::IDList &j)
{
    int count = jointIdList.addIDList(j);
    return count;
}

void yarpWholeBodyModelV2::convertBasePose(const wbi::Frame &xBase, iDynTree::Transform & world_H_base)
{
    iDynTree::Position pos(xBase.p[0],xBase.p[1],xBase.p[2]);
    iDynTree::Rotation rot;

    // Both iDynTree::Rotation and wbi::Rotation are row-major
    memcpy(rot.data(),xBase.R.data,9*sizeof(double));

    world_H_base = iDynTree::Transform(rot,pos);
}

void yarpWholeBodyModelV2::convertBaseVelocity(const double *dxB, iDynTree::Twist & baseVel)
{
    for(int i=0; i < 6; i++)
    {
        baseVel(i) = dxB[i];
    }
}

void yarpWholeBodyModelV2::convertBaseAcceleration(const double *ddxB, iDynTree::Vector6 & baseAcc)
{
    for(int i=0; i < 6; i++)
    {
        baseAcc(i) = ddxB[i];
    }
}

void yarpWholeBodyModelV2::convertQ(const double *q_input, iDynTree::JointPosDoubleArray & q_idyntree)
{
    toEigen(q_idyntree) = Eigen::Map< const Eigen::VectorXd >(q_input,q_idyntree.size());
}

void yarpWholeBodyModelV2::convertDQ(const double *dq_input, iDynTree::JointDOFsDoubleArray & dq_idyntree)
{
    toEigen(dq_idyntree) = Eigen::Map< const Eigen::VectorXd >(dq_input,dq_idyntree.size());
}

void yarpWholeBodyModelV2::convertDDQ(const double *ddq_input, iDynTree::JointDOFsDoubleArray & ddq_idyntree)
{
    toEigen(ddq_idyntree) = Eigen::Map< const Eigen::VectorXd >(ddq_input,ddq_idyntree.size());
}

void yarpWholeBodyModelV2::convertGeneralizedTorques(const iDynTree::Wrench & baseWrench_idyntree,
                                                   const iDynTree::JointDOFsDoubleArray & jntTrqs_idyntree,
                                                   double * tau)
{
    Eigen::Map< Eigen::VectorXd > outGenTrqs(tau,jntTrqs_idyntree.size()+6);
    outGenTrqs.segment(0,6) = toEigen(baseWrench_idyntree);
    outGenTrqs.segment(6,jntTrqs_idyntree.size()) = toEigen(jntTrqs_idyntree);
}


void yarpWholeBodyModelV2::convertOutputMatrix(const iDynTree::MatrixDynSize & mat,
                                                     double * outMatBuffer)
{
    // Both iDynTree and WBI store matrices in row-major format, so it is quite trivial to copy them
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > outMat(outMatBuffer,mat.rows(),mat.cols());

    outMat = toEigen(mat);
}

void yarpWholeBodyModelV2::convertWorldGravity(const double *grav,
                         iDynTree::Vector3 & worldGravity)
{
    worldGravity(0) = grav[0];
    worldGravity(1) = grav[1];
    worldGravity(2) = grav[2];
}

void yarpWholeBodyModelV2::updateState()
{
    m_kinDynComp.setRobotState(m_modelPos.worldBasePos(),
                               m_modelPos.jointPos(),
                               m_baseVel,
                               m_jntVel,
                               m_worldGravity);
}

void yarpWholeBodyModelV2::posToHomMatrix(double * pos, iDynTree::Transform & frameWithoutOffset_H_frameWithOffset)
{
    iDynTree::Position pos_idyn(pos[0],pos[1],pos[2]);
    frameWithoutOffset_H_frameWithOffset = iDynTree::Transform(iDynTree::Rotation::Identity(),pos_idyn);
}

void yarpWholeBodyModelV2::posToAdjMatrix(double * pos,
                                        const iDynTree::Rotation & world_R_frame,
                                        iDynTree::Transform & frameWithoutOffset_world_H_frameWithOffset_world)
{
    iDynTree::Position pos_idyn_frame(pos[0],pos[1],pos[2]);
    iDynTree::Position pos_world = world_R_frame*pos_idyn_frame;

    frameWithoutOffset_world_H_frameWithOffset_world = iDynTree::Transform(iDynTree::Rotation::Identity(),pos_world);
}



bool yarpWholeBodyModelV2::getJointLimitFromControlBoard(double *qMin, double *qMax, int joint)
{
    int controlBoardId = controlBoardAxisList[joint].first;
    int index = controlBoardAxisList[joint].second;
    assert(ilim[controlBoardId]!=NULL);
    bool res = ilim[controlBoardId]->getLimits(index, qMin, qMax);
    if(res)
    {
        *qMin = (*qMin) * yarpWbi::Deg2Rad;   // convert from deg to rad
        *qMax = (*qMax) * yarpWbi::Deg2Rad;   // convert from deg to rad
    }
    return res;
}

bool yarpWholeBodyModelV2::getJointLimitFromModel(double *qMin, double *qMax, int joint)
{
    iDynTree::IJointConstPtr p_joint = m_kinDynComp.model().getJoint(joint);

    if( !p_joint )
    {
        yError() << "yarpWholeBodyModel: Error in getting joint limits for joint " << joint;
        return false;
    }

    if( !(p_joint->hasPosLimits()) )
    {
        yWarning() << "yarpWholeBodyModel: Joint " << m_kinDynComp.model().getJointName(joint) << " has no limits enabled, returning min double and max double as limits.";
        *qMin = -DBL_MAX;
        *qMax = DBL_MAX;
    }
    else
    {
        p_joint->getPosLimits(0,*qMin,*qMax);
    }

    return true;
}


bool yarpWholeBodyModelV2::getJointLimits(double *qMin, double *qMax, int joint)
{
    if ( (joint < 0 || joint >= (int)jointIdList.size()) && joint != -1 )
    {
        return false;
    }

    // TODO(traversaro) : I don't know how often this is called, but it should be cached
    if (this->getLimitsFromControlBoard)
    {
        bool ret = true;
        for(int bp=0; bp < (int)controlBoardNames.size(); bp++ )
        {
            ret = ret && openDrivers(bp);
        }

        if( ret )
        {
            if(joint>=0)
            {
                ret = ret &&  getJointLimitFromControlBoard(qMin,qMax,joint);
            }
            else
            {
                int n = jointIdList.size();
                for(int i=0; i<n; i++)
                    ret = ret && getJointLimitFromControlBoard(qMin+i, qMax+i, i);
            }
        }

        ret = ret && closeDrivers();

        return ret;
    }
    else
    {
        bool ret = true;

        if ( joint >= 0 )
        {
            ret = ret &&  getJointLimitFromModel(qMin,qMax,joint);
        }
        else
        {
            int n = jointIdList.size();
            for(int i=0; i<n; i++)
            {
                ret = ret && getJointLimitFromModel(qMin + i, qMax + i, i);
            }
        }

        return ret;
    }
}


bool yarpWholeBodyModelV2::computeH(double *q, const Frame &xBase, int frameId, Frame &H, double *pos)
{
    if (!this->initDone)
    {
        return false;
    }

    // Cast the frameId  to a iDynTree::FrameIndex
    iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    if ( !(m_kinDynComp.model().isValidFrameIndex(frameIndex)) && frameId != COM_LINK_ID )
    {
        return false;
    }

    convertBasePose(xBase,m_modelPos.worldBasePos());
    convertQ(q,m_modelPos.jointPos());
    this->updateState();

    iDynTree::Transform world_H_frame = iDynTree::Transform::Identity();

    if (frameId != COM_LINK_ID)
    {
        world_H_frame = m_kinDynComp.getWorldTransform(frameIndex);

        // Handle pos argument
        if (pos)
        {
            iDynTree::Transform frameWithoutOffset_H_frameWithOffset;
            posToHomMatrix(pos,frameWithoutOffset_H_frameWithOffset);
            iDynTree::Transform world_H_frameWithoutOffset = world_H_frame;
            world_H_frame = world_H_frameWithoutOffset*frameWithoutOffset_H_frameWithOffset;
        }
    }
    else
    {
        world_H_frame.setPosition(m_kinDynComp.getCenterOfMassPosition());
    }

    iDynTree::Matrix4x4 homMat = world_H_frame.asHomogeneousTransform();
    H.set4x4Matrix(homMat.data());
    return true;
}


bool yarpWholeBodyModelV2::computeJacobian(double *q, const Frame &xBase, int frameId, double *J, double *pos)
{
    if( !this->initDone )
    {
        return false;
    }

    // Cast the frameId  to a iDynTree::FrameIndex
    iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    if ( !(m_kinDynComp.model().isValidFrameIndex(frameIndex)) && frameId != COM_LINK_ID )
    {
        return false;
    }

    convertBasePose(xBase,m_modelPos.worldBasePos());
    convertQ(q,m_modelPos.jointPos());
    this->updateState();

    //Get Jacobian, the one of the link or the one of the COM
    // Note: historically the angular part of the COM Jacobian has been filled with no-sense data
    // To preserve the interface, we still return a 6 \times 6 + n matrix, but we fill the angular
    // part (the last three columns) with zeros, consistenty with computeDJdq .
    bool ok = true;
    if (frameId != COM_LINK_ID)
    {
        ok = m_kinDynComp.getFrameFreeFloatingJacobian(frameIndex,m_frameJacobian);

        if( pos )
        {
            yError() << "yarpWholeBodyModel: pos option of computeJacobian currently not supported";
        }

    } else {
        ok = m_kinDynComp.getCenterOfMassJacobian(m_comJacobian);
        int cols = m_frameJacobian.cols();
        toEigen(m_frameJacobian).block(0,0,3,cols) = toEigen(m_comJacobian);
        toEigen(m_frameJacobian).block(3,0,3,cols).setZero();
    }

    convertOutputMatrix(m_frameJacobian,J);

    return ok;
}

bool yarpWholeBodyModelV2::computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int frameId, double *dJdq, double *pos)
{
    if( !this->initDone )
    {
        return false;
    }

    // Cast the frameId  to a iDynTree::FrameIndex
    iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    if ( !(m_kinDynComp.model().isValidFrameIndex(frameIndex)) && frameId != COM_LINK_ID )
    {
        return false;
    }

    convertBasePose(xBase,m_modelPos.worldBasePos());
    convertQ(q,m_modelPos.jointPos());
    this->updateState();

    iDynTree::Vector6 biasAcc;
    if (frameId != COM_LINK_ID)
    {
        biasAcc = m_kinDynComp.getFrameBiasAcc(frameIndex);

        if( pos )
        {
            yError() << "yarpWholeBodyModel: pos option of computeDJdq currently not supported";
        }
    } else {
        iDynTree::Vector3 comBiasAcc = m_kinDynComp.getCenterOfMassBiasAcc();

        iDynTree::toEigen(biasAcc).segment<3>(0) = iDynTree::toEigen(comBiasAcc);
        iDynTree::toEigen(biasAcc).segment<3>(3).setZero();
    }

    biasAcc.fillBuffer(dJdq);

    return true;
}

bool yarpWholeBodyModelV2::forwardKinematics(double *q, const Frame &xB, int linkId, double *x, double * pos)
{
    wbi::Frame output;
    bool ok = computeH(q,xB,linkId,output,pos);

    x[0] = output.p[0];
    x[1] = output.p[1];
    x[2] = output.p[2];

    // Remember: both YARP and WBI store matrices in row-major order
    Matrix dcm(3,3);
    memcpy(dcm.data(),output.R.data,9*sizeof(double));

    Vector axisangle = dcm2axis(dcm);

    x[3] = axisangle(0);
    x[4] = axisangle(1);
    x[5] = axisangle(2);
    x[6] = axisangle(3);

    return ok;
}

bool yarpWholeBodyModelV2::inverseDynamics(double *q, const Frame &xB,
                                         double *dq, double *dxB,
                                         double *ddq, double *ddxB,
                                         double *g, double *tau)
{
    if( !this->initDone )
    {
        return false;
    }

    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xB, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());
    convertBaseVelocity(dxB, m_baseVel);
    convertDQ(dq, m_jntVel);
    convertBaseAcceleration(ddxB, m_baseAcc);
    convertDDQ(ddq, m_jntAcc);
    convertWorldGravity(g,m_worldGravity);

    // Update iDynTree state
    this->updateState();

    //Computing inverse dynamics
    m_kinDynComp.inverseDynamics(m_baseAcc,m_jntAcc,m_zeroExtWrenches,m_genTrqs);

    // Copy result in output buffer
    convertGeneralizedTorques(m_genTrqs.baseWrench(),m_genTrqs.jointTorques(),tau);

    return true;
}

bool yarpWholeBodyModelV2::computeMassMatrix(double *q, const Frame &xBase, double *M)
{
    if( !this->initDone )
    {
        return false;
    }

    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());

    // Update iDynTree state
    this->updateState();

    //Computing inverse dynamics
    m_kinDynComp.getFreeFloatingMassMatrix(m_massMatrix);

    // Copy result in output buffer
    convertOutputMatrix(m_massMatrix,M);

    return true;
}

bool yarpWholeBodyModelV2::computeGeneralizedBiasForces(double *q, const Frame &xBase, double *dq, double *dxB, double *g, double *h)
{
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());
    convertBaseVelocity(dxB, m_baseVel);
    convertDQ(dq, m_jntVel);
    convertWorldGravity(g,m_worldGravity);

    // Update iDynTree state
    this->updateState();

    //Computing specialized inverse dynamics
    m_kinDynComp.generalizedBiasForces(m_genTrqs);

    // Copy result in output buffer
    convertGeneralizedTorques(m_genTrqs.baseWrench(),m_genTrqs.jointTorques(),h);

    return true;
}

bool yarpWholeBodyModelV2::computeGravityBiasForces(double *q, const wbi::Frame &xBase, double* g, double *h)
{
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());
    convertWorldGravity(g,m_worldGravity);

    // Update iDynTree state
    this->updateState();

    //Computing specialized inverse dynamics
    m_kinDynComp.generalizedGravityForces(m_genTrqs);

    // Copy result in output buffer
    convertGeneralizedTorques(m_genTrqs.baseWrench(),m_genTrqs.jointTorques(),h);

    return true;
}


bool yarpWholeBodyModelV2::computeCentroidalMomentum(double *q, const Frame &xBase, double *dq, double *dxB, double *h)
{
    convertBasePose(xBase, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());
    convertBaseVelocity(dxB, m_baseVel);
    convertDQ(dq, m_jntVel);
    this->updateState();

    iDynTree::SpatialMomentum centroidalMomentum = this->m_kinDynComp.getCentroidalTotalMomentum();

    Eigen::Map< Vector6d > hEigen(h);

    hEigen = iDynTree::toEigen(centroidalMomentum);

    return true;
}

const wbi::IDList & yarpWholeBodyModelV2::getJointList()
{
    return jointIdList;
}

const wbi::IDList & yarpWholeBodyModelV2::getFrameList()
{
    return frameIdList;
}



