/*
 * Copyright (C) 2013 RBCS Department & iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef WBMODEL_ICUB_H
#define WBMODEL_ICUB_H

#include <wbi/wbiUtil.h>
#include <wbi/wbiID.h>
#include <wbi/iWholeBodyModel.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/iDynTree/TorqueEstimationTree.h>
#include <iCub/skinDynLib/skinContactList.h>
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
     * pass to the constructor or to the setYarpWbiProperties method.
     * The option that this Property object should contain are:
     *
     * * | Parameter name | Type | Units | Default Value | Required | Description | Notes |
     * |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
     * | urdf | - | - | - | Yes | File name of the urdf file to load for getting the model of the robot. | The file name will be opened by the ResourceFinder::findFile call, using the search rules of the ResourceFinder, that you can find in http://wiki.icub.org/yarpdoc/yarp_resource_finder_tutorials.html |
     * | getLimitsFromControlBoard | string | - | - | No | Get limits from the real robot instead of the URDF model. |  |
     *
     *  Given that the limits in the URDF file could be outdated with respect to the real robot,
     * limits can be loaded by the real robot, by passing to the yarpWholeBodyModel the getLimitsFromControlBoard
     * option. In that case the ControlBoard of the robot will be opened, using the same parameters used by the
     * yarpWholeBodyActuactors interface.
     */
    class yarpWholeBodyModel: public wbi::iWholeBodyModel
    {
    protected:
        wbi::IDList jointIdList;
        wbi::IDList frameIdList;

        bool initDone;
        int dof;

        //iCub::iDynTree::DynTree * p_model;
        yarp::os::Property wbi_yarp_properties;
        iCub::iDynTree::DynTree * p_model;

        yarp::sig::Matrix world_base_transformation;

        yarp::sig::Vector v_base, a_base;
        yarp::sig::Vector omega_base, domega_base;

        yarp::sig::Vector v_six_elems_base;
        yarp::sig::Vector a_six_elems_base;


        yarp::sig::Vector all_q;
        yarp::sig::Vector all_dq;
        yarp::sig::Vector all_ddq;

        yarp::sig::Vector all_q_min, all_q_max;

        //Output buffers
        yarp::sig::Vector generalized_floating_base_torques; //n+6 outputs for inverse dynamics
        yarp::sig::Matrix floating_base_mass_matrix;
        yarp::sig::Matrix reduced_floating_base_mass_matrix;
        yarp::sig::Vector six_elem_buffer;
        yarp::sig::Vector three_elem_buffer;

        // *** Variables needed for opening IControlLimits interfaces
        bool getLimitsFromControlBoard;
        std::string                                 name;           // name used as root for the local ports
        std::string                                 robot;          // name of the robot
        std::vector<int>                            bodyParts;      // list of the body parts
        std::vector<std::string>                    controlBoardNames;  // names of the body parts
        // list of controlBoard/Axis pair for each joint
        std::vector< std::pair<int,int> >  controlBoardAxisList;
        std::vector<yarp::dev::PolyDriver*>       dd;
        std::vector<yarp::dev::IControlLimits*>   ilim;


        std::vector<int> wbiToiDynTreeJointId;

        bool openDrivers(int bp);

        bool convertBasePose(const wbi::Frame &xBase, yarp::sig::Matrix & H_world_base);
        bool convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_b, yarp::sig::Vector & omega_b);
        bool convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_b, yarp::sig::Vector & domega_b);
        bool convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_six_elems_b);
        bool convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_six_elems_b);


        bool convertQ(const double *q_input, yarp::sig::Vector & q_complete_output);
        bool convertQ(const yarp::sig::Vector & q_complete_input, double *q_output);
        bool convertDQ(const double *dq_input, yarp::sig::Vector & dq_complete_output);
        bool convertDDQ(const double *ddq_input, yarp::sig::Vector & ddq_complete_output);

        bool convertGeneralizedTorques(yarp::sig::Vector idyntree_base_force, yarp::sig::Vector idyntree_torques, double * tau);

        bool closeDrivers();

        bool getJointLimitFromControlBoard(double *qMin, double *qMax, int joint);


    public:
         /**
          * @param _name Local name of the interface (used as stem of port names)
         * @param wbi_yarp_conf the yarp::os::Property containg the options for wbi
          */
        yarpWholeBodyModel(const char* _name,
                           const yarp::os::Property & wbi_yarp_conf=yarp::os::Property());


        virtual ~yarpWholeBodyModel();

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


        inline virtual int getDoFs(){ return dof; }

        /** Remove the specified joint form the robot model. The joint is considered blocked
          * at its last known value (values of the joint angles are stored whenever a method of
          * iWholeBodyModel is called). If no previous value of the joint angle is known, zero is assumed.
          * @param j Id of the joint to remove
          * @return True if the joint was found and removed, false otherwise. */
        virtual bool removeJoint(const wbi::ID &j);
        virtual bool addJoint(const wbi::ID &j);
        virtual int addJoints(const wbi::IDList &j);
        virtual const wbi::IDList& getJointList();

        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1);

        /** Compute rototranslation matrix from root reference frame to reference frame associated to the specified link.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link that is the target of the rototranslation
          * @param H Output 4x4 rototranslation matrix (stored by rows)
          * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeH(double *q, const wbi::Frame &xBase, int linkId, wbi::Frame &H);

        /** Compute the Jacobian of the specified point of the robot.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link
          * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters)
          * @note If linkId==COM_LINK_ID then the angular part of J is related to the angular velocity of the
          *       whole multi-body system. This Jacobian premultiplied by the whole robot's 6D inertia
          *       matrix is equal to the Jacobian of the angular momentum of the whole robot. */
        virtual bool computeJacobian(double *q, const wbi::Frame &xBase, int linkId, double *J, double *pos=0);

        /** Given a point on the robot, compute the product between the time derivative of its
          * Jacobian and the joint velocity vector.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities
          * @param linkID ID of the link
          * @param dJdq Output 6-dim vector containing the product dJ*dq
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters)
          * \note If linkId==COM_LINK_ID only the first three elements of dJdq (the linear part) are computed,
          *          the angular part is zero
          */
        virtual bool computeDJdq(double *q, const wbi::Frame &xBase, double *dq, double *dxB, int linkID, double *dJdq, double *pos=0);

        /** Compute the forward kinematics of the specified joint.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link.
          * @param x Output 7-dim pose vector (3 for pos, 4 for orientation expressed in axis/angle).
          * @return True if the operation succeeded, false otherwise. */
        virtual bool forwardKinematics(double *q, const wbi::Frame &xBase, int linkId, double *x);

        /** Compute the inverse dynamics.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities.
          * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity.
          * @param ddq Joint accelerations.
          * @param ddxB Acceleration of the robot base, 3 values for linear acceleration and 3 values for angular acceleration.
          * @param g gravity acceleration expressed in world frame (3 values)
          * @param tau Output joint torques.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau);

        /** Compute the floating base Mass Matrix.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @return True if the operation succeeded, false otherwise.
         */
        virtual bool computeMassMatrix(double *q, const wbi::Frame &xBase, double *M);

        /** Compute the generalized bias forces (gravity+Coriolis+centrifugal) terms.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal), with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeGeneralizedBiasForces(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double*g, double *h);

        /** Compute the 6 element centroidal momentum, as defined in:
         * Centroidal dynamics of a humanoid robot - DE Orin, A Goswami, SH Lee - Autonomous Robots 35 (2-3), 161-176
         * @param q Joint angles (in radians)
         * @param xBase Rototranslation from world frame to robot base frame (\f${}^w H_b \f$)
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity
         * @param h output 6-element vector containg the centroidal momentum (3 value for linear momentum and 3 for angular momentum)
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeCentroidalMomentum(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *h);

        virtual const wbi::IDList & getFrameList();

    };
}

#endif
