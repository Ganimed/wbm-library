/*
 * Copyright (C) 2015 RBCS/iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef WB_FLOATING_BASE_ESTIMATOR_YARP_H
#define WB_FLOATING_BASE_ESTIMATOR_YARP_H

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>

#include<Eigen/Core>
#include<Eigen/LU>


#include "yarpWholeBodyInterface/yarpWbiUtil.h"
#include "yarpWholeBodyInterface/yarpWholeBodySensors.h"

#include <map>


namespace wbi {
    class ID;
    class IDList;
}

namespace yarpWbi
{
    const int BASE_POS_ESTIMATE_SIZE = 16;
    const int BASE_VEL_ESTIMATE_SIZE  = 6;
    const int BASE_ACC_ESTIMATE_SIZE  = 6;

    /**
     * Class that performs local estimation of the floating base state (position, velocity, acceleration)
     *
     */
    class localFloatingBaseStateEstimator
    {
    protected:
        int baseFrameLinkID;         // ID of the assigned base frame for base to root rototranslation computation
        wbi::iWholeBodyModel *wholeBodyModel;
        int dof;                                            // Number of degrees of freedom in the wbi

        int robot_reference_frame_link;                     //Reference link assigned as world frame
        wbi::Frame rootLink_H_ReferenceLink;                //Rototranslation between Reference frame (assigned as world) and Root Link
        wbi::Frame world_H_rootLink;                        //Rototranslation between Root link and World
        wbi::Frame world_H_reference;                        //Rototranslation between Reference frame and world (future work)
        wbi::Frame referenceLink_H_rootLink;                //Rototranslation between Root link and Reference frame

        /*
         * optimised computation of world-to-base velocity
        */

        Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> complete_jacobian;

        Eigen::PartialPivLU<Eigen::MatrixXd::PlainObject> luDecompositionOfBaseJacobian;

    public:
        localFloatingBaseStateEstimator(wbi::iWholeBodyModel * _wholeBodyModel=0, int _dof=0);

        /** Initialize the class */
        bool init(wbi::iWholeBodyModel * _wholeBodyModel=0, int _dof=0);

        bool changeDoF(int new_dof);

        /** Sets a desired link as the world reference frame **/
        bool setWorldBaseLinkName(std::string);


        /** @brief Computes the Base position for a given joint configuration
         *
         * The resulting estimate is serialized into a 16x1 vector of double. Use wbi#frameFromSerialization function to transform it into a proper Frame object
         * @param q the current joint positions
         * @param base_pos_estimate estimate of the base position w.r.t. world frame.
         * @return true if successful, false otherwise
         */
        bool computeBasePosition(double *q, double * base_pos_estimate);

        /** Computes the Base velocity for a given set of joint velocities
         * Output a 6x1 representing the linear and angular velocity of the base frame w.r.t world frame
         *
         * @param q joint positions
         * @param dq joint velocities
         * @param base_vel_estimate resulting estimate of base velocity
         * @return true if successful, false otherwise
         */
        bool computeBaseVelocity(double *q, double *dq, double * base_vel_estimate);
        /** Computes the Base acceleration for a given joint velocity **/
        //bool computeBaseAcceleration();

    };


    /**
     * Helper class for remoteFloatingBaseStateEstimator, providing a callback to directly
     * fill remoteFloatingBaseStateEstimator.
     *
     */
    class remoteFloatingBaseStatePortProcessor : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
    {
            yarp::sig::Matrix base_pos_mat;
            yarp::sig::Vector base_vel_vec;
            yarp::sig::Vector base_acc_vec;

            yarp::os::Mutex * p_mutex;
            double * base_pos_estimate_buf;
            double * base_vel_estimate_buf;
            double * base_acc_estimate_buf;
            bool   * measureAvailableFlag;

            double * lastTimestamp;

        public:
            remoteFloatingBaseStatePortProcessor();
            void setMutex(yarp::os::Mutex * p_mutex);

            void setBuffers(double * base_pos_estimate_buf,
                            double * base_vel_estimate_buf,
                            double * base_acc_estimate_buf,
                            double * lastTimestamp
                           );

            void setFlags(bool * measureAvailableFlag);

            virtual void onRead(yarp::os::Bottle& b);
    };



    /**
     * Class that performs read the floating base state (position, velocity, acceleration)
     * from an external port. Currently the message is read from the wholeBodyDynamicsTree in
     * a custom form:
     *  * the first element of the bottle should be a 4x4 yarp::sig::Matrix encoding the
     *    homogeneous transformation that transform a point expressed in floating base frame
     *    in a point expressed in the world/inertial frame.
     *  * the second element (if available) of the bottle should be a 6 elements yarp::sig::Vector enconding
     *    the twist (with linear/angular serialization) of the floating base frame with respect to the world/inertial frame, expressed
     *    with the orientation of the world/inertial frame but with respect to the origin of floating base frame.
     *  * the third element (if available) of the bottle should be a 6 elements yarp::sig::Vector enconding
     *    the floating base acceleration. The first three elements should be the classical 3D linear acceleration of
     *    the origin of the floating base frame with respect to the world/inertial frame, expressed in the world/
     *    inertial orientation. The last three elements should be the angular acceleration of the floating base frame with
     *    respect to the world/inertial frame, expressed using the world/inertial frame orientation.
     *
     * This format is bound to eventually change, but for now it is this one.
     *
     *  For more info on this convention, please check: http://wiki.icub.org/codyco/dox/html/dynamics_notation.html
     */
    class remoteFloatingBaseStateEstimator
    {
    private:
        remoteFloatingBaseStatePortProcessor callbackHandler;

    protected:
        yarp::os::BufferedPort<yarp::os::Bottle> inputPort;

        yarp::os::ConstString local;
        yarp::os::ConstString remote;

        yarp::os::Mutex buff_mutex;

        double base_pos_estimate[BASE_POS_ESTIMATE_SIZE];
        double base_vel_estimate[BASE_VEL_ESTIMATE_SIZE];
        double base_acc_estimate[BASE_ACC_ESTIMATE_SIZE];

        bool measureAvailable; // this flag is false if a measure is not available,
                               // for example if it has not have been reader of a
                               // timeout has been detected

        double timeoutLimit_in_seconds;
        double lastTimestamp;

        void updateTimeout();

    public:
        remoteFloatingBaseStateEstimator();

        /** Initialize the class */
        bool open(yarp::os::Searchable &config);

        /** Gets the Base position for the remote floating base published source.
            Check the class info for more informations. **/
        bool getBasePosition(double * base_pos_estimate);

        /** Gets the Base velocity  for the remote floating base published source.
            Check the class info for more informations. **/
        bool getBaseVelocity(double * base_vel_estimate);

        /** Gets the Base velocity  for the remote floating base published source.
            Check the class info for more informations. **/
        bool getBaseAcceleration(double * base_acc_estimate);

        bool getBaseState(double * base_pos_estimate,
                               double * base_vel_estimate,
                               double * base_acc_estimate);


        bool close();
    };


}

#endif
