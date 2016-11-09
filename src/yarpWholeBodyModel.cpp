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

#include "yarpWholeBodyModel.h"
#include "yarpWbiUtil.h"

#include <string>
#include <cmath>

#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/math.h>

#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/ResourceFinder.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>

#include <iCub/iDynTree/DynTree.h>

#include <iDynTree/yarp/YARPConversions.h>

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
yarpWholeBodyModel::yarpWholeBodyModel(const char* _name,
                                       const yarp::os::Property & _wbi_yarp_conf)
    : initDone(false),
      dof(0),
      wbi_yarp_properties(_wbi_yarp_conf),
      p_model(0),
      six_elem_buffer(6,0.0),
      three_elem_buffer(3,0.0),
      homMatrixBuffer(4,4),
      adjMatrixBuffer(6,6),
      getLimitsFromControlBoard(false)
{
}

yarpWholeBodyModel::~yarpWholeBodyModel()
{
    if( p_model )
    {
        delete p_model;
        p_model = 0;
    }
}

bool yarpWholeBodyModel::init()
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

    // Use the default kinematic base link
    std::string kinematic_base_link_name = "";

    std::vector<std::string> joint_names;
    joint_names.resize(0,"");
    dof = jointIdList.size();
    p_model = new iCub::iDynTree::DynTree(std::string(urdf_file_path),joint_names,kinematic_base_link_name);
    all_q.resize(p_model->getNrOfDOFs(),0.0);
    all_q_min = all_q_max = all_ddq = all_dq = all_q;
    floating_base_mass_matrix.resize(p_model->getNrOfDOFs(),p_model->getNrOfDOFs());
    floating_base_mass_matrix.zero();

    world_base_transformation.resize(4,4);
    world_base_transformation.eye();

    v_base.resize(3,0.0);

    a_base = omega_base = domega_base = v_base;

    v_six_elems_base.resize(6,0.0);
    a_six_elems_base.resize(6,0.0);

    //For now set the not actuate position to zero
    if( true ) {
        all_q.zero();
        //memcpy(all_q.data(),initial_q,all_q.size()*sizeof(double));
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



    //Build the map between wbi id and iDynTree id
    wbiToiDynTreeJointId.resize(jointIdList.size());
    for(int wbi_numeric_id =0;  wbi_numeric_id < (int)jointIdList.size(); wbi_numeric_id++ )
    {
        wbi::ID joint_id;
        jointIdList.indexToID(wbi_numeric_id,joint_id);
        int idyntree_id = p_model->getDOFIndex(joint_id.toString());
        if( idyntree_id == - 1 )
        {
            yError() << "yarpWholeBodyModel error: joint " << joint_id.toString() << " not found in URDF file";
            initDone = false;
            return false;
        }

        wbiToiDynTreeJointId[wbi_numeric_id] = idyntree_id;
    }

    //Populate the frame id add
    // \todo TODO FIXME properly implement frames in iDynTree
    for(int frame_numeric_id=0; frame_numeric_id < p_model->getNrOfFrames(); frame_numeric_id++ )
    {
        std::string frame_name;
        p_model->getFrameName(frame_numeric_id,frame_name);
        frameIdList.addID(frame_name);
    }

    this->initDone = true;
    return this->initDone;
}

bool yarpWholeBodyModel::openDrivers(int bp)
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

bool yarpWholeBodyModel::closeDrivers()
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

bool yarpWholeBodyModel::close()
{
    return closeDrivers();
}

bool yarpWholeBodyModel::removeJoint(const wbi::ID &j)
{
    return false;
}

bool yarpWholeBodyModel::setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties)
{
    wbi_yarp_properties = yarp_wbi_properties;
    return true;
}

bool yarpWholeBodyModel::getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties)
{
    yarp_wbi_properties = wbi_yarp_properties;
    return true;
}

int yarpWholeBodyModel::getDoFs()
{
    return dof;
}

bool yarpWholeBodyModel::addJoint(const wbi::ID &j)
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

int yarpWholeBodyModel::addJoints(const wbi::IDList &j)
{
    int count = jointIdList.addIDList(j);
    return count;
}

bool yarpWholeBodyModel::convertBasePose(const Frame &xBase, yarp::sig::Matrix & H_world_base)
{
    if( H_world_base.cols() != 4 || H_world_base.rows() != 4 )
        H_world_base.resize(4,4);
    xBase.get4x4Matrix(H_world_base.data());
    return true;
}

bool yarpWholeBodyModel::convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_b, yarp::sig::Vector & omega_b)
{
    if (!dxB) {
        //shortcut to zero the vectors
        v_b.zero();
        omega_b.zero();
        return true;
    }
    v_b[0] = dxB[0];
    v_b[1] = dxB[1];
    v_b[2] = dxB[2];
    omega_b[0] = dxB[3];
    omega_b[1] = dxB[4];
    omega_b[2] = dxB[5];
    return true;
}

bool yarpWholeBodyModel::convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_b)
{
    v_b[0] = dxB[0];
    v_b[1] = dxB[1];
    v_b[2] = dxB[2];
    v_b[3] = dxB[3];
    v_b[4] = dxB[4];
    v_b[5] = dxB[5];
    return true;
}

bool yarpWholeBodyModel::convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_b, yarp::sig::Vector & domega_b)
{
    a_b[0] = ddxB[0];
    a_b[1] = ddxB[1];
    a_b[2] = ddxB[2];
    domega_b[0] = ddxB[3];
    domega_b[1] = ddxB[4];
    domega_b[2] = ddxB[5];
    return true;
}

bool yarpWholeBodyModel::convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_b)
{
    a_b[0] = ddxB[0];
    a_b[1] = ddxB[1];
    a_b[2] = ddxB[2];
    a_b[3] = ddxB[3];
    a_b[4] = ddxB[4];
    a_b[5] = ddxB[5];
    return true;
}

bool yarpWholeBodyModel::convertQ(const double *_q_input, yarp::sig::Vector & q_complete_output)
{
    if (!_q_input) {
        //shortcut to zero vector
        q_complete_output.zero();
        return true;
    }

    for(int wbi_joint_numeric_id=0; wbi_joint_numeric_id < this->dof; wbi_joint_numeric_id++ )
    {
            double tmp;
            tmp = _q_input[wbi_joint_numeric_id];
            assert(wbiToiDynTreeJointId[wbi_joint_numeric_id] >= 0);
            assert(wbiToiDynTreeJointId[wbi_joint_numeric_id] < (int)q_complete_output.size());
            q_complete_output[wbiToiDynTreeJointId[wbi_joint_numeric_id]] = tmp;
    }
    return true;
}

bool yarpWholeBodyModel::convertQ(const yarp::sig::Vector & q_complete_input, double *_q_output )
{
    for(int wbi_joint_numeric_id=0; wbi_joint_numeric_id < this->dof; wbi_joint_numeric_id++ )
    {
        assert(wbiToiDynTreeJointId[wbi_joint_numeric_id] >= 0);
        assert(wbiToiDynTreeJointId[wbi_joint_numeric_id] < (int)q_complete_input.size());
        _q_output[wbi_joint_numeric_id] = q_complete_input[wbiToiDynTreeJointId[wbi_joint_numeric_id]];

    }
    return true;
}

bool yarpWholeBodyModel::convertDQ(const double *_dq_input, yarp::sig::Vector & dq_complete_output)
{
    convertQ(_dq_input,dq_complete_output);
    return true;
}

bool yarpWholeBodyModel::convertDDQ(const double *_ddq_input, yarp::sig::Vector & ddq_complete_output)
{
    convertQ(_ddq_input,ddq_complete_output);
    return true;
}

bool yarpWholeBodyModel::convertGeneralizedTorques(yarp::sig::Vector idyntree_base_force, yarp::sig::Vector idyntree_torques, double * tau)
{
    if( idyntree_base_force.size() != 6 && (int)idyntree_torques.size() != p_model->getNrOfDOFs() ) { return false; }
    for(int j = 0; j < 6; j++ ) {
        tau[j] = idyntree_base_force[j];
    }
    for(int wbi_joint_numeric_id=0; wbi_joint_numeric_id < this->dof; wbi_joint_numeric_id++ )
    {
        tau[wbi_joint_numeric_id+6] = idyntree_torques[wbiToiDynTreeJointId[wbi_joint_numeric_id]];
    }

    return true;
}

void yarpWholeBodyModel::posToHomMatrix(double * pos, yarp::sig::Matrix & homMatrix)
{
    iDynTree::Position pos_idyn(pos[0],pos[1],pos[2]);
    iDynTree::Transform frameWithoutOffset_H_frameWithOffset(iDynTree::Rotation::Identity(),pos_idyn);

    toYarp(frameWithoutOffset_H_frameWithOffset.asHomogeneousTransform(),homMatrix);
    return;
}

void yarpWholeBodyModel::posToAdjMatrix(double * pos, const yarp::sig::Matrix & world_R_frame, yarp::sig::Matrix & adjMatrix)
{
    iDynTree::Position pos_idyn_frame(pos[0],pos[1],pos[2]);
    iDynTree::Rotation world_R_frame_idyn;
    toiDynTree(world_R_frame,world_R_frame_idyn);
    iDynTree::Position pos_world = world_R_frame_idyn*pos_idyn_frame;

    iDynTree::Transform frameWithoutOffset_world_H_frameWithOffset_world(iDynTree::Rotation::Identity(),pos_world);

    toYarp(frameWithoutOffset_world_H_frameWithOffset_world.inverse().asAdjointTransform(),adjMatrix);
}



bool yarpWholeBodyModel::getJointLimitFromControlBoard(double *qMin, double *qMax, int joint)
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

bool yarpWholeBodyModel::getJointLimits(double *qMin, double *qMax, int joint)
{
    if( (joint < 0 || joint >= (int)jointIdList.size()) && joint != -1 )
    {
        return false;
    }

    if( this->getLimitsFromControlBoard ) {

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


    } else {
      // OLD IMPLEMENTATION
      all_q_min = p_model->getJointBoundMin();
      all_q_max = p_model->getJointBoundMax();

      if( joint == -1 ) {
         //Get all joint limits
          convertQ(all_q_min,qMin);
          convertQ(all_q_max,qMax);
      } else {
          *qMin = all_q_min[wbiToiDynTreeJointId[joint]];
          *qMax = all_q_max[wbiToiDynTreeJointId[joint]];
      }
      return true;
    }
}


bool yarpWholeBodyModel::computeH(double *q, const Frame &xBase, int linkId, Frame &H, double *pos)
{
    if( (linkId < 0 || linkId >= p_model->getNrOfLinks()) && linkId != COM_LINK_ID ) return false;

    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);

    p_model->setWorldBasePose(world_base_transformation);
    p_model->setAng(all_q);

    Matrix H_result, H_result_buf(4,4);
    H_result.zero();
    if( linkId != COM_LINK_ID ) {
        H_result = p_model->getPosition(linkId);
        // Handle pos argument
        if(pos)
        {
            posToHomMatrix(pos,homMatrixBuffer);
            H_result_buf = H_result;
            H_result = H_result_buf*homMatrixBuffer;
        }

        if( H_result.cols() != 4 || H_result.rows() != 4 ) { return false; }
    } else {
       H_result = Matrix(4,4);
       H_result.eye();
       Vector com = p_model->getCOM();
       if( com.size() == 0 ) { return false; }
       H_result.setSubcol(com,0,3);
    }

    H.set4x4Matrix(H_result.data());
    return true;
}


bool yarpWholeBodyModel::computeJacobian(double *q, const Frame &xBase, int linkId, double *J, double *pos)
{
    if( (linkId < 0 || linkId >= p_model->getNrOfLinks()) && linkId != COM_LINK_ID ) return false;

    bool ret_val;

    int dof_jacobian = dof+6;
    Matrix complete_jacobian(6,all_q.size()+6), reduced_jacobian(6,dof_jacobian);
    complete_jacobian.zero();
    reduced_jacobian.zero();

    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);

    p_model->setWorldBasePose(world_base_transformation);
    p_model->setAng(all_q);

    //Get Jacobian, the one of the link or the one of the COM
    if( linkId != COM_LINK_ID ) {
         ret_val = p_model->getJacobian(linkId,complete_jacobian);
         if( !ret_val ) return false;
    } else {
         ret_val = p_model->getCOMJacobian(complete_jacobian);
         if( !ret_val ) return false;
    }


    int i=0;
    for(int wbi_joint_numeric_id=0; wbi_joint_numeric_id < this->dof; wbi_joint_numeric_id++ )
    {
        reduced_jacobian.setCol(wbi_joint_numeric_id+6,complete_jacobian.getCol(6+wbiToiDynTreeJointId[wbi_joint_numeric_id]));
        i++;
    }
    reduced_jacobian.setSubmatrix(complete_jacobian.submatrix(0,5,0,5),0,0);

    if( pos )
    {
        Matrix world_H_frameWithoutOffset = p_model->getPosition(linkId);
        Matrix world_R_frameWithoutOffset = world_H_frameWithoutOffset.submatrix(0,2,0,2);

        reducedJacobianBuffer = reduced_jacobian;

        posToAdjMatrix(pos,world_R_frameWithoutOffset,adjMatrixBuffer);

        reduced_jacobian = adjMatrixBuffer*reducedJacobianBuffer;
    }


    memcpy(J,reduced_jacobian.data(),sizeof(double)*6*dof_jacobian);

    return true;
}

bool yarpWholeBodyModel::computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int linkID, double *dJdq, double *pos)
{
    if ((linkID < 0 || linkID >= p_model->getNrOfLinks()) && linkID != COM_LINK_ID) return false;

    //joints
    convertQ(q, all_q);
    convertDQ(dq, all_dq);
    all_ddq.zero();

    //base
    convertBasePose(xBase, world_base_transformation);
    convertBaseVelocity(dxB, v_six_elems_base);
    a_six_elems_base.zero();

    p_model->setAng(all_q);
    p_model->setDAng(all_dq);
    p_model->setD2Ang(all_ddq);

    p_model->setWorldBasePose(world_base_transformation);

    //The setKinematicBaseVelAcc accepts the velocity and accelerations of the kinematic base in world orientation
    p_model->setKinematicBaseVelAcc(v_six_elems_base,a_six_elems_base);

    p_model->kinematicRNEA();

    bool ret;

    if( linkID != COM_LINK_ID ) {
        ret = p_model->getAcc(linkID,six_elem_buffer);

        if( pos )
        {
            Matrix world_H_frameWithoutOffset = p_model->getPosition(linkID);
            Matrix world_R_frameWithoutOffset = world_H_frameWithoutOffset.submatrix(0,2,0,2);

            Vector retBuffer = six_elem_buffer;

            posToAdjMatrix(pos,world_R_frameWithoutOffset,adjMatrixBuffer);

            six_elem_buffer = adjMatrixBuffer*retBuffer;
        }

    } else {
        // Only the linear part of DJdq is supported for the COM
        ret = p_model->getAccCOM(three_elem_buffer);
        for(int i=0; i < 3; i++ ) {
            six_elem_buffer[i] = three_elem_buffer[i];
            six_elem_buffer[i+3] = 0.0;
        }
    }

    if( !ret ) {
        if( six_elem_buffer.size() != 6 ) {
            six_elem_buffer.resize(6,0.0);
        }
        return false;
    }

    //should I copy directly?
    YARP_ASSERT(six_elem_buffer.size() == 6);
    memcpy(dJdq, six_elem_buffer.data(), sizeof(double) * six_elem_buffer.size());
    return true;

}

bool yarpWholeBodyModel::forwardKinematics(double *q, const Frame &xB, int linkId, double *x, double * pos)
{
    if( (linkId < 0 || linkId >= p_model->getNrOfLinks()) && linkId != COM_LINK_ID ) return false;

    convertBasePose(xB,world_base_transformation);
    convertQ(q,all_q);

    p_model->setWorldBasePose(world_base_transformation);
    p_model->setAng(all_q);

    Matrix H_result, H_result_buf;

    if( linkId != COM_LINK_ID ) {
        H_result = p_model->getPosition(linkId);

        // Handle pos argument
        if(pos)
        {
            posToHomMatrix(pos,homMatrixBuffer);
            H_result_buf = H_result;
            H_result = H_result_buf*homMatrixBuffer;
        }

        if( H_result.cols() != 4 || H_result.rows() != 4 ) { return false; }
    } else {
       H_result = Matrix(4,4);
       H_result.eye();
       Vector com = p_model->getCOM();
       if( com.size() == 0 ) { return false; }
       H_result.setSubcol(com,0,3);
    }



    Vector axisangle(4);

    x[0] = H_result(0,3);
    x[1] = H_result(1,3);
    x[2] = H_result(2,3);

    axisangle =  dcm2axis(H_result.submatrix(0,2,0,2));

    x[3] = axisangle(0);
    x[4] = axisangle(1);
    x[5] = axisangle(2);
    x[6] = axisangle(3);

    return true;
}

bool yarpWholeBodyModel::inverseDynamics(double *q, const Frame &xB, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau)
{
    //We can take into account the gravity efficiently by adding a fictional acceleration to the base
    double baseAcceleration[6] = {0, 0, 0, 0, 0, 0};
    baseAcceleration[0] = ddxB[0] - g[0];
    baseAcceleration[1] = ddxB[1] - g[1];
    baseAcceleration[2] = ddxB[2] - g[2];
    baseAcceleration[3] = ddxB[3];
    baseAcceleration[4] = ddxB[4];
    baseAcceleration[5] = ddxB[5];

    /** \todo move all conversion (also the one relative to frames) in convert* functions */
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xB, world_base_transformation);
    convertQ(q, all_q);
    convertBaseVelocity(dxB, v_base,omega_base);
    convertDQ(dq, all_dq);
    convertBaseAcceleration(baseAcceleration, a_base,domega_base);
    convertDDQ(ddq, all_ddq);

    //Setting iDynTree variables
    p_model->setWorldBasePose(world_base_transformation);
    p_model->setAng(all_q);
    //The kinematic initial values are expressed in the imu link (in this case, the base) for iDynTree
    yarp::sig::Matrix base_world_rotation = world_base_transformation.submatrix(0,2,0,2).transposed();

    p_model->setInertialMeasure(base_world_rotation * omega_base,
                                base_world_rotation * domega_base,
                                base_world_rotation * a_base);
    p_model->setDAng(all_dq);
    p_model->setD2Ang(all_ddq);

    //Computing inverse dynamics
    p_model->kinematicRNEA();
    p_model->dynamicRNEA();

    //Get the output floating base torques and convert them to wbi generalized torques
    yarp::sig::Vector base_force = p_model->getBaseForceTorque(iCub::iDynTree::WORLD_FRAME);

    return convertGeneralizedTorques(base_force,p_model->getTorques(),tau);
}

bool yarpWholeBodyModel::computeMassMatrix(double *q, const Frame &xBase, double *M)
{
    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);

    //Setting iDynTree variables
    p_model->setWorldBasePose(world_base_transformation);
    p_model->setAng(all_q);



    //iDynTree floating base mass matrix is already world orientation friendly
    //(i.e. expects the base velocity to be expressed in world reference frame)
    floating_base_mass_matrix.zero();
    p_model->getFloatingBaseMassMatrix(floating_base_mass_matrix);

    if( reduced_floating_base_mass_matrix.cols() != 6+dof ||
        reduced_floating_base_mass_matrix.rows() != 6+dof ) {
        reduced_floating_base_mass_matrix.resize(6+dof,6+dof);
    }

    reduced_floating_base_mass_matrix.zero();

    //Converting the iDynTree complete floating_base_mass_matrix to the reduced one
    //           that includes only the joint added in the wholeBodyModel interfacec

    //Given the structure of the wholeBodyModel, this manual quadratic loop is necessary
    //To speed-up the case in which all the joint are considered, it could be possible to add a check to directly copy the mass matrix

    //Using mapped eigen matrices to avoid the overhead of using setSubmatrix / submatrix methods in yarp
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_reduced_mm(reduced_floating_base_mass_matrix.data(),
                                                                                                           reduced_floating_base_mass_matrix.rows(),
                                                                                                           reduced_floating_base_mass_matrix.cols());

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_complete_mm(floating_base_mass_matrix.data(),
                                                                                                            floating_base_mass_matrix.rows(),
                                                                                                            floating_base_mass_matrix.cols());


    //Left top submatrix (spatial inertia matrix)
    mapped_reduced_mm.block<6,6>(0,0) = mapped_complete_mm.block<6,6>(0,0);

    //Rest of the matrix
    for(int reduced_dof_row=0; reduced_dof_row < this->dof; reduced_dof_row++ )
    {
            int complete_dof_row = wbiToiDynTreeJointId[reduced_dof_row];

            //Left bottom submatrix
            mapped_reduced_mm.block<1,6>(6+reduced_dof_row,0) = mapped_complete_mm.block<1,6>(6+complete_dof_row,0);

            ///Right top submatrix (using the row loop to avoid doing another loop)
            mapped_reduced_mm.block<6,1>(0,6+reduced_dof_row) =  mapped_reduced_mm.block<1,6>(6+reduced_dof_row,0).transpose();

            for(int reduced_dof_column=0; reduced_dof_column < this->dof; reduced_dof_column++ )
            {
                    int complete_dof_column = wbiToiDynTreeJointId[reduced_dof_column];
                    mapped_reduced_mm(6+reduced_dof_row,6+reduced_dof_column) =
                        mapped_complete_mm(6+complete_dof_row,6+complete_dof_column);
            }

    }

    memcpy(M,reduced_floating_base_mass_matrix.data(),sizeof(double)*(6+dof)*(6+dof));

    return true;
}

bool yarpWholeBodyModel::computeGeneralizedBiasForces(double *q, const Frame &xBase, double *dq, double *dxB, double *g, double *h)
{
    /** \todo move all conversion (also the one relative to frames) in convert* functions */
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);
    convertBaseVelocity(dxB,v_base,omega_base);
    convertDQ(dq,all_dq);
    yarp::sig::Vector ddxB(6, 0.0);
    yarp::sig::Vector ddq(dof, 0.0);

    //We can take into account the gravity efficiently by adding a fictional acceleration to the base
    ddxB[0] = - g[0];
    ddxB[1] = - g[1];
    ddxB[2] = - g[2];

    convertBaseAcceleration(ddxB.data(),a_base,domega_base);


    convertDDQ(ddq.data(),all_ddq);

    //Setting iDynTree variables
    p_model->setWorldBasePose(world_base_transformation);
    p_model->setAng(all_q);
    //The kinematic initial values are expressed in the imu link (in this case, the base) for iDynTree
    yarp::sig::Matrix base_world_rotation = world_base_transformation.submatrix(0,2,0,2).transposed();

    p_model->setInertialMeasure(base_world_rotation * omega_base,
                                     base_world_rotation * domega_base,
                                     base_world_rotation * a_base);
    p_model->setDAng(all_dq);
    p_model->setD2Ang(all_ddq);

    //Computing inverse dynamics
    p_model->kinematicRNEA();
    p_model->dynamicRNEA();

    //Get the output floating base torques and convert them to wbi generalized torques
    yarp::sig::Vector base_force = p_model->getBaseForceTorque(iCub::iDynTree::WORLD_FRAME);


    convertGeneralizedTorques(base_force,p_model->getTorques(),h);

    return true;
}

bool yarpWholeBodyModel::computeGravityBiasForces(double *q, const wbi::Frame &xBase, double* g, double *h)
{
    return this->computeGeneralizedBiasForces(q, xBase, 0, 0, g, h);
}

bool yarpWholeBodyModel::computeCentroidalMomentum(double *q, const Frame &xBase, double *dq, double *dxB, double *h)
{
    /** \todo move all conversion (also the one relative to frames) in convert* functions */
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);

    convertBaseVelocity(dxB, v_six_elems_base);
    convertDQ(dq,all_dq);

    a_six_elems_base.zero();

    //Setting iDynTree variables
    p_model->setWorldBasePose(world_base_transformation);
    p_model->setKinematicBaseVelAcc(v_six_elems_base,a_six_elems_base);
    p_model->setAng(all_q);
    p_model->setDAng(all_dq);
    p_model->setD2Ang(all_ddq);

    //Computing centroidal momentum
    if( six_elem_buffer.size() != 6 ) {
        six_elem_buffer.resize(6,0.0);
    }

    six_elem_buffer = p_model->getCentroidalMomentum();

    memcpy(h,six_elem_buffer.data(),6*sizeof(double));

    return true;
}

const wbi::IDList & yarpWholeBodyModel::getJointList()
{
    return jointIdList;
}

const wbi::IDList & yarpWholeBodyModel::getFrameList()
{
    return frameIdList;
}

iCub::iDynTree::DynTree * yarpWholeBodyModel::getRobotModel()
{
    return this->p_model;
}



