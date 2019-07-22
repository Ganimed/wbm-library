/*
 * Copyright (C) 2013-2016 RBCS Department & iCub Facility - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete, Francesco Romano, Silvio Traversaro
 * email: andrea.delprete@iit.it
 *
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

#ifndef YARP_WBMODEL_V2_H
#define YARP_WBMODEL_V2_H

#include <wbi/wbiUtil.h>
#include <wbi/wbiID.h>
#include <wbi/iWholeBodyModel.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>

#include <map>

namespace wbi {
    class ID;
    class IDList;
}

namespace yarpWbi
{
    /**
     * Interface to the kinematic/dynamic model of yarp robot
     *
     * You can configure this object with a yarp::os::Property object, that you can
     * pass to the constructor. Alternativly you can set the Property through the setYarpWbiProperties method,
     * but in that case you have to set the property before calling the init method.
     *
     * The option that this Property object should contain are:
     *
     * | Parameter name | Type | Units | Default Value | Required | Description | Notes |
     * |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
     * | urdf | - | - | - | Yes | File name of the urdf file to load for getting the model of the robot. | The file name will be opened by the ResourceFinder::findFile call, using the search rules of the ResourceFinder, that you can find in http://wiki.icub.org/yarpdoc/yarp_resource_finder_tutorials.html |
     * | getLimitsFromControlBoard | string | - | - | No | Get limits from the real robot instead of the URDF model. |  |
     *
     *  Given that the limits in the URDF file could be outdated with respect to the real robot,
     * limits can be loaded by the real robot, by passing to the yarpWholeBodyModel the getLimitsFromControlBoard
     * option. In that case the ControlBoard of the robot will be opened, using the same parameters used by the
     * yarpWholeBodyActuators interface.
     */
    class yarpWholeBodyModelV2 : public wbi::iWholeBodyModel
    {
    protected:
        wbi::IDList jointIdList;
        wbi::IDList frameIdList;
        bool initDone;
        yarp::os::Property wbi_yarp_properties;

        // iDynTree class actually used for all computations
        iDynTree::KinDynComputations m_kinDynComp;

        ////////////////////////////////////////////////////////
        //// Input buffers
        ////////////////////////////////////////////////////////
        iDynTree::FreeFloatingPos      m_modelPos;
        iDynTree::Twist                m_baseVel;
        iDynTree::JointDOFsDoubleArray m_jntVel;
        iDynTree::Vector6              m_baseAcc;
        iDynTree::JointDOFsDoubleArray m_jntAcc;
        iDynTree::Vector3              m_worldGravity;

        /** External wrenches: used for inverse dynamics, but actually
         *  always set to zero.
         */
        iDynTree::LinkNetExternalWrenches m_zeroExtWrenches;

        ////////////////////////////////////////////////////////
        //// Output buffers
        ////////////////////////////////////////////////////////
        iDynTree::FreeFloatingGeneralizedTorques m_genTrqs;
        iDynTree::FreeFloatingMassMatrix         m_massMatrix;
        iDynTree::FrameFreeFloatingJacobian      m_frameJacobian;
        iDynTree::MomentumFreeFloatingJacobian   m_momentumJacobian;
        iDynTree::MatrixDynSize                  m_comJacobian;

        ////////////////////////////////////////////////////////////
        /// Variables needed for opening IControlLimits interfaces
        ////////////////////////////////////////////////////////////
        bool getLimitsFromControlBoard;
        std::string                                 name;           // name used as root for the local ports
        std::string                                 robot;          // name of the robot
        std::vector<int>                            bodyParts;      // list of the body parts
        std::vector<std::string>                    controlBoardNames;  // names of the body parts
        // list of controlBoard/Axis pair for each joint
        std::vector< std::pair<int,int> >  controlBoardAxisList;
        std::vector<yarp::dev::PolyDriver*>       dd;
        std::vector<yarp::dev::IControlLimits*>   ilim;
        bool openDrivers(int bp);
        bool closeDrivers();
        bool getJointLimitFromControlBoard(double *qMin, double *qMax, int joint);
        bool getJointLimitFromModel(double *qMin, double *qMax, int joint);


        //////////////////////////////////////////////////////////////////
        /// Conversion functions between input data and internal buffers
        //////////////////////////////////////////////////////////////////
        void convertBasePose(const wbi::Frame &xBase, iDynTree::Transform & world_H_base);
        void convertBaseVelocity(const double *dxB, iDynTree::Twist & baseVel);
        void convertBaseAcceleration(const double *ddxB, iDynTree::Vector6 & baseAcc);
        void convertQ(const double *q_input, iDynTree::JointPosDoubleArray & q_idyntree);
        void convertDQ(const double *dq_input, iDynTree::JointDOFsDoubleArray & dq_idyntree);
        void convertDDQ(const double *ddq_input, iDynTree::JointDOFsDoubleArray & ddq_idyntree);
        void convertWorldGravity(const double *grav, iDynTree::Vector3 & ddq_idyntree);
        void convertGeneralizedTorques(const iDynTree::Wrench & baseWrench_idyntree,
                                       const iDynTree::JointDOFsDoubleArray & jntTrqs_idyntree,
                                       double * tau);
        void convertOutputMatrix(const iDynTree::MatrixDynSize & idyn_mat,
                                double * raw_mat);

        // Update the state (pos and vel) of the KinDynComputations class
        void updateState();

        /**
         * Helper function: convert a pos offset 3D vector in a iDynTree::Transform
         * matrix that transforms from the offset frame to the one specified)
         * ( specifiedFrame_H_frameWithOffset ).
         */
        void posToHomMatrix(double * posIn,
                            iDynTree::Transform & homMatrix);

        /**
         * Helper function: convert a pos offset 3D vector in a iDynTree::Transform
         * that transforms from the (frameWithoutOffset,world) frame to the
         * (frameWithOffset,world) . [We mean (position,orientation)].
         */
        void posToAdjMatrix(double * posIn,
                            const iDynTree::Rotation & world_R_frame,
                            iDynTree::Transform & adjMatrix);

    public:
        /**
         * @param _name Local name of the interface (used as stem of port names)
        * @param wbi_yarp_conf the yarp::os::Property containg the options for wbi
         */
        yarpWholeBodyModelV2(const char* _name,
                           const yarp::os::Property & wbi_yarp_conf=yarp::os::Property());


        virtual ~yarpWholeBodyModelV2();

        virtual bool init();
        virtual bool close();

        /**
         * Set the properties of the yarpWbiActuactors interface
         * Note: this function must be called before init, otherwise it takes no effect
         * @param yarp_wbi_properties the properties of the yarpWholeBodyModel object
         */
        virtual bool setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties);

        /**
         * Get the properties of the yarpWbiActuactors interface
         * @param yarp_wbi_properties the properties of the yarpWholeBodyModel object
         */
        virtual bool getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties);


        /** @return The number of degrees of freedom of the robot model. */
        virtual int getDoFs();

        /** Remove the specified joint from the robot model. The joint is considered blocked
         * at its last known value (values of the joint angles are stored whenever a method of
         * iWholeBodyModel is called). If no previous value of the joint angle is known, zero is assumed.
         * @param j Id of the joint to remove
         * @return True if the joint was found and removed, false otherwise. */
        virtual bool removeJoint(const wbi::ID &j);
        virtual bool addJoint(const wbi::ID &j);
        virtual int addJoints(const wbi::IDList &j);

        virtual const wbi::IDList& getJointList();

        /** Get the upper and lower limits of the joint position(s).
         * @param qMin Output lower limit(s) (rad).
         * @param qMax Output upper limit(s) (rad).
         * @param joint Id of the joint, -1 for getting the limits of all the joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1);

        /** Compute homogenous transformation matrix that appliend on a vector expressed in the specified frame it transform it in the world.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param frameId Id of the link that is the target of the homogeneous transformation .
         * @param H Output 4x4 homogeneous transformation  matrix (stored by rows).
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         * @return True if the operation succeeded, false otherwise (invalid input parameters).
         */
        virtual bool computeH(double *q, const wbi::Frame &xBase, int frameId, wbi::Frame &H, double *pos=0);

        /**
         * Compute the Jacobian of a specified frame of the robot.
         * The first three rows are the jacobian of the velocity of the frame origin
         * (unless a offset is specified with the pos parameter).
         * The bottom three rows are the jacobian of the angular velocity of the frame, expressed in the world frame.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param frameId Id of the frame.
         * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints.
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         *        If pos is not specified (0, default) then the origin of frame is assumed.
         * @return True if the operation succeeded, false otherwise (invalid input parameters).
         * @note If frameId==COM_LINK_ID then the angular part of J is related to the angular velocity of the
         *       whole multi-body system. This Jacobian premultiplied by the whole robot's 6D inertia
         *       matrix is equal to the Jacobian of the angular momentum of the whole robot.
         *       If frameId is COM_LINK_ID, the position offset (pos argument) is ignored.
         */
        virtual bool computeJacobian(double *q, const wbi::Frame &xBase, int frameId, double *J, double *pos=0);

        /**
         * Given a frame on the robot , compute the product between the time derivative of its
         * Jacobian and the joint velocity vector. The origin of the specified frame can be modified with the pos parameter.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param dq Joint velocities (rad/s).
         * @param frameId Id of the link.
         * @param dJdq Output 6-dim vector containing the product \f$ \dot{J}\dot{q} \f$ .
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         *        If pos is not specified (0, default) then the origin of frame is assumed.
         *        If frameId is COM_LINK_ID, the position offset (pos argument) is ignored.
         *        WARNING : in the computation of the DJdq with a pos, a term is missing, so
         *                  the result is not 100% correct.
         * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeDJdq(double *q, const wbi::Frame &xBase, double *dq, double *dxB, int frameId, double *dJdq, double *pos=0);

        /**
         * Compute the forward kinematics of the specified frame.
         * The frame is specified with the id of a frame in the robot, plus an linear offset
         * of the frame origin.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param frameId Id of the frame.
         * @param x Output 7-dim pose vector (3 for pos, 4 for orientation expressed in axis/angle).
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool forwardKinematics(double *q, const wbi::Frame &xBase, int frameId, double *x, double *pos=0);

        /**
         * Compute the inverse dynamics.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param ddq Joint accelerations (rad/s^2).
         * @param ddxB Acceleration of the robot base in world reference frame, 3 values for linear and 3 for angular acceleration.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param tau Output generalized forces at the joints and base (N+6 dimensional, with N=number of joints).
         * @return True if the operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau);

        /**
         * Compute the floating base Mass Matrix.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeMassMatrix(double *q, const wbi::Frame &xBase, double *M);

        /** Compute the generalized bias forces (gravity+Coriolis+centrifugal) terms.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal), with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */

        virtual bool computeGeneralizedBiasForces(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double* g, double *h);
        virtual bool computeGravityBiasForces(double *q, const wbi::Frame &xBase, double* g, double *h);

        /** Compute the 6 element centroidal momentum, as defined in:
         * Centroidal dynamics of a humanoid robot - DE Orin, A Goswami, SH Lee - Autonomous Robots 35 (2-3), 161-176
         * @param q Joint angles (in radians)
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity
         * @param h output 6-element vector containg the centroidal momentum (3 value for linear momentum and 3 for angular momentum)
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeCentroidalMomentum(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *h);

        /**
         * Get the list of available frames.
         * This is useful to get information on the frames
         * for which the different methods of iWholeBodyModel can
         * be called.
         * @return the list of available frames.
         */
        virtual const wbi::IDList& getFrameList();
    };
}

#endif
