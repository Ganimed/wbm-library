/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
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

#ifndef WBI_YARP_H
#define WBI_YARP_H

#include "yarpWholeBodyInterface/yarpWbiUtil.h"
#include "yarpWholeBodyInterface/yarpWholeBodyActuators.h"
#include "yarpWholeBodyInterface/yarpWholeBodyModel.h"
#include "yarpWholeBodyInterface/yarpWholeBodyStates.h"

#define INITIAL_TIMESTAMP -1000.0

/* CODE UNDER DEVELOPMENT */

namespace wbi {
    class ID;
    class IDList;
}

namespace yarpWbi {
    class yarpWholeBodyActuators;
    class yarpWholeBodyModel;
    class yarpWholeBodyStates;
}

namespace yarpWbi
{


    const int JOINT_ESTIMATE_TYPES_SIZE = 3;
    ///< estimate types that are automatically added when calling addJoint(s) and automatically removed when calling removeJoint
    const wbi::EstimateType jointEstimateTypes[JOINT_ESTIMATE_TYPES_SIZE] =
    {
        wbi::ESTIMATE_JOINT_POS,         // joint position
        //wbi::ESTIMATE_JOINT_VEL,         // joint velocity
        //wbi::ESTIMATE_JOINT_ACC,         // joint acceleration
        wbi::ESTIMATE_JOINT_TORQUE,      // joint torque
        //wbi::ESTIMATE_MOTOR_VEL,         // motor velocity
        //wbi::ESTIMATE_MOTOR_TORQUE,      // motor torque
        wbi::ESTIMATE_MOTOR_PWM,         // motor PWM (proportional to motor voltage)
    };

    /**
     * Class to communicate with yarp-powered robot.
     */
    class yarpWholeBodyInterface : public wbi::wholeBodyInterface
    {
    private:
        yarp::os::Mutex wbiMutex;

    protected:
        yarpWholeBodyActuators  *actuatorInt;
        yarpWholeBodyModel      *modelInt;
        yarpWholeBodyStates     *stateInt;
        yarpWholeBodyModel      *modelForStateInt;

        wbi::IDList empty_id_list;

    public:
        // *** CONSTRUCTORS ***
        yarpWholeBodyInterface(const char* _interfaceName,
                               const yarp::os::Property & _yarp_wbi_properties=yarp::os::Property());

        /** gets the actuator implementation
         * @return the wholeBodyActuator implementation
         */
        yarpWholeBodyActuators* wholeBodyActuator();

        /** gets the model implementation
         * @return the wholeBodyModel implementation
         */
        yarpWholeBodyModel* wholeBodyModel();

        /** gets the state implementation
         * @return the wholeBodyStates implementation
         */
        yarpWholeBodyStates* wholeBodyState();

         /**
         * Set the properties of the yarpWholeBodyInterface
         * Note: this function must be called before init, otherwise it has no effect
         * @param yarp_wbi_properties the properties of the yarpWholeBodyInterface object
         */
        virtual bool setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties);

        /**
         * Get the properties of the yarpWholeBodyInterface
         * @param yarp_wbi_properties the properties of the yarpWholeBodyInterface object
         */
        virtual bool getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties);

        virtual ~yarpWholeBodyInterface();
        virtual bool init();
        virtual bool close();
        virtual bool removeJoint(const wbi::ID &j);
        virtual bool addJoint(const wbi::ID &j);
        virtual int addJoints(const wbi::IDList &j);

        yarp::os::Mutex& getInterfaceMutex();

        // ACTUATORS
        virtual bool removeActuator(const wbi::ID &j);
        virtual bool addActuator(const wbi::ID &j);
        virtual int addActuators(const wbi::IDList &j);
        virtual const wbi::IDList& getActuatorList();
        virtual bool setControlMode(wbi::ControlMode cm, double *ref=0, int jnt=-1);
        virtual bool setControlReference(double *ref, int jnt=-1);
        virtual bool setControlParam(wbi::ControlParam parId, const void *val, int jnt=-1);

        // STATES
        virtual bool addEstimate(const wbi::EstimateType st, const wbi::ID &sid);
        virtual int addEstimates(const wbi::EstimateType st, const wbi::IDList &sids);
        virtual bool removeEstimate(const wbi::EstimateType st, const wbi::ID &sid);
        virtual const wbi::IDList& getEstimateList(const wbi::EstimateType st);
        virtual int getEstimateNumber(const wbi::EstimateType st);
        virtual bool getEstimate(const wbi::EstimateType et, const int estimate_numeric_id, double *data, double time=-1.0, bool blocking=true);
        virtual bool getEstimates(const wbi::EstimateType et, double *data, double time=-1.0, bool blocking=true);
        virtual bool setEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value);

        // MODEL
        virtual int getDoFs();
        virtual const wbi::IDList& getJointList();
        virtual const wbi::IDList& getFrameList();
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1);
        virtual bool computeH(double *q, const wbi::Frame &xB, int linkId, wbi::Frame &H, double *pos);
        virtual bool computeJacobian(double *q, const wbi::Frame &xB, int linkId, double *J, double *pos=0);
        virtual bool computeDJdq(double *q, const wbi::Frame &xB, double *dq, double *dxB, int linkId, double *dJdq, double *pos=0);
        virtual bool forwardKinematics(double *q, const wbi::Frame &xB, int linkId, double *x, double *pos);
        virtual bool inverseDynamics(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau);
        virtual bool computeMassMatrix(double *q, const wbi::Frame &xB, double *M);
        virtual bool computeGeneralizedBiasForces(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *g, double *h);
        virtual bool computeCentroidalMomentum(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *h);
        virtual bool computeGravityBiasForces(double *q, const wbi::Frame &xBase, double* g, double *h);

    };

} // end namespace yarpWbi

#endif
