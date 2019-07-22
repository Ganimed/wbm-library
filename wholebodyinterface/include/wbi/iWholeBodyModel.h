/*
 * Copyright (C) 2013  CoDyCo Consortium
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
 *
 * Authors: Andrea Del Prete, Silvio Traversaro, Marco Randazzo
 * email: andrea.delprete@iit.it - silvio.traversaro@iit.it - marco.randazzo@iit.it
 */

#ifndef IWHOLEBODYMODEL_H
#define IWHOLEBODYMODEL_H

namespace wbi {
    class ID;
    class IDList;
    class Frame;

    /**
     * Interface to the kinematic/dynamic model of the robot.
     *
     * All the quantities follow the convention defined in http://wiki.icub.org/codyco/dox/html/dynamics_notation.html .
     *
     * For all the methods for which a frame needs to be specified, the frame can be specified using a
     * frameId, i.e. an signed integer . A list of all the frames available in the model can be obtained using
     * the getFrameList() method. An additional offset for the frame origin can be specified with the pos argument.
     * The pos argument is always defined to be a 3D vector expressed in the specified frame orientation.
     */
    class iWholeBodyModel
    {
    public:
        /** Id of the virtual link associated to the Center of Mass of the robot.
         * This id can be used to compute any kinematic quantity (position, velocity, Jacobian)
         * associated to the center of mass. */
        static const int COM_LINK_ID = -1;

        virtual ~iWholeBodyModel();
        virtual bool init() = 0;
        virtual bool close() = 0;

        /** @return The number of degrees of freedom of the robot model. */
        virtual int getDoFs() = 0;

        /** Remove the specified joint from the robot model. The joint is considered blocked
         * at its last known value (values of the joint angles are stored whenever a method of
         * iWholeBodyModel is called). If no previous value of the joint angle is known, zero is assumed.
         * @param j Id of the joint to remove
         * @return True if the joint was found and removed, false otherwise. */
        virtual bool removeJoint(const ID &j) = 0;
        virtual bool addJoint(const ID &j) = 0;
        virtual int addJoints(const IDList &j) = 0;

        virtual const IDList& getJointList() = 0;

        /** Get the upper and lower limits of the joint position(s).
         * @param qMin Output lower limit(s) (rad).
         * @param qMax Output upper limit(s) (rad).
         * @param joint Id of the joint, -1 for getting the limits of all the joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1) = 0;

        /** Compute homogeneous transformation matrix from root reference frame to reference frame associated to the specified link.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).         * @param frameId Id of the link that is the target of the homogeneous transformation.
         * @param H Output 4x4 homogeneous transformation matrix (stored by rows).
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         * @return True if the operation succeeded, false otherwise (invalid input parameters).
         */
        virtual bool computeH(double *q, const Frame &xBase, int frameId, Frame &H, double *pos=0) = 0;

        /**
         * Compute the Jacobian of a specified frame of the robot.
         * The first three rows are the jacobian of the velocity of the frame origin
         * (unless a offset is specified with the pos parameter).
         * The bottom three rows are the jacobian of the angular velocity of the frame, expressed in the world frame.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).         * @param frameId Id of the frame.
         * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints.
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         *        If pos is not specified (0, default) then the origin of frame is assumed.
         * @return True if the operation succeeded, false otherwise (invalid input parameters).
         * @note If frameId==COM_LINK_ID then the angular part of J is related to the angular velocity of the
         *       whole multi-body system. This Jacobian premultiplied by the whole robot's 6D inertia
         *       matrix is equal to the Jacobian of the angular momentum of the whole robot.
         *       If frameId is COM_LINK_ID, the position offset (pos argument) is ignored.
         */
        virtual bool computeJacobian(double *q, const Frame &xBase, int frameId, double *J, double *pos=0) = 0;

        /**
         * Given a frame on the robot , compute the product between the time derivative of its
         * Jacobian and the joint velocity vector. The origin of the specified frame can be modified with the pos parameter.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).         * @param dq Joint velocities (rad/s).
         * @param frameId Id of the link.
         * @param dJdq Output 6-dim vector containing the product \f$\dot{J}\dot{q}\f$.
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         *        If pos is not specified (0, default) then the origin of frame is assumed.
         *        If frameId is COM_LINK_ID, the position offset (pos argument) is ignored.
         * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int frameId, double *dJdq, double *pos=0) = 0;

        /**
         * Compute the forward kinematics of the specified frame.
         * The frame is specified with the id of a frame in the robot, plus an linear offset
         * of the frame origin.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).         * @param frameId Id of the frame.
         * @param x Output 7-dim pose vector (3 for pos, 4 for orientation expressed in axis/angle).
         * @param pos 3d position of the point expressed w.r.t the specified frame.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool forwardKinematics(double *q, const Frame &xBase, int frameId, double *x, double *pos=0) = 0;

        /**
         * Compute the inverse dynamics.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param ddq Joint accelerations (rad/s^2).
         * @param ddxB Acceleration of the robot base in world reference frame, 3 values for linear and 3 for angular acceleration.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param tau Output generalized forces at the joints and base (N+6 dimensional, with N=number of joints).
         * @return True if the operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau) = 0;

        /**
         * Compute the floating base Mass Matrix.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on 4d homegenous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeMassMatrix(double *q, const Frame &xBase, double *M) = 0;

        /** Compute the generalized bias forces (gravity+Coriolis+centrifugal) terms.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).         
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal), with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */

        virtual bool computeGeneralizedBiasForces(double *q, const Frame &xBase, double *dq, double *dxB, double* g, double *h) = 0;

        /** Compute the gravity bias forces terms.
         * @param q Joint angles (rad).
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param h Output N+6-dim vector containing the gravity bias forces, with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */

        virtual bool computeGravityBiasForces(double *q, const Frame &xBase, double* g, double *h) = 0;

        /** Compute the 6 element centroidal momentum, as defined in:
         * Centroidal dynamics of a humanoid robot - DE Orin, A Goswami, SH Lee - Autonomous Robots 35 (2-3), 161-176
         * @param q Joint angles (in radians)
         * @param xBase homogeneous transformation that applied on a 4d homogeneous position vector expressed in the base frame transforms it in the world frame (world_H_base).
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity
         * @param h output 6-element vector containg the centroidal momentum (3 value for linear momentum and 3 for angular momentum)
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeCentroidalMomentum(double *q, const Frame &xBase, double *dq, double *dxB, double *h) = 0;

        /**
         * Get the list of available frames.
         * This is useful to get information on the frames
         * for which the different methods of iWholeBodyModel can
         * be called.
         * @return the list of available frames.
         */
        virtual const IDList& getFrameList() = 0;

    };

}

#endif //IWHOLEBODYMODEL_H
