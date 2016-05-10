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

#ifndef WBSTATES_YARP_H
#define WBSTATES_YARP_H
#include "yarpWholeBodyInterface/floatingBaseEstimators.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/skinDynLib/skinContactList.h>

//#include <Eigen/Sparse>
#include<Eigen/Core>
#include<Eigen/LU>




#include <map>


namespace wbi {
    class ID;
    class IDList;
}

namespace yarpWbi
{
    class yarpWholeBodySensors;

    /**
     * Thread that estimates the state of the iCub robot.
     */
    class yarpWholeBodyEstimator: public yarp::os::RateThread
    {
    protected:
        yarpWbi::yarpWholeBodySensors        *sensors;
        //double                      estWind;        // time window for the estimation

        iCub::ctrl::AWLinEstimator  *dqFilt;        // joint velocity filter
        iCub::ctrl::AWQuadEstimator *d2qFilt;       // joint acceleration filter
        iCub::ctrl::AWLinEstimator  *dTauJFilt;     // joint torque derivative filter
        iCub::ctrl::AWLinEstimator  *dTauMFilt;     // motor torque derivative filter
        iCub::ctrl::FirstOrderLowPassFilter *tauJFilt;  ///< low pass filter for joint torque
        iCub::ctrl::FirstOrderLowPassFilter *tauMFilt;  ///< low pass filter for motor torque
        iCub::ctrl::FirstOrderLowPassFilter *pwmFilt;   ///< low pass filter for motor PWM
        iCub::ctrl::FirstOrderLowPassFilter *velocitiesFilt;   ///< low pass filter for joint velocities

        int dqFiltWL, d2qFiltWL;                    // window lengths of adaptive window filters
        double dqFiltTh, d2qFiltTh;                 // threshold of adaptive window filters
        int dTauMFiltWL, dTauJFiltWL;               // window lengths of adaptive window filters
        double dTauMFiltTh, dTauJFiltTh;            // threshold of adaptive window filters
        double tauJCutFrequency;
        double tauMCutFrequency;
        double pwmCutFrequency;
        double velocitiesCutFrequency;

        yarp::sig::Vector           q, dq, d2q, qStamps;         // last joint position estimation
        yarp::sig::Vector           tauJ, tauJStamps;
        yarp::sig::Vector           pwm, pwmStamps;

        /* Resize all vectors using current number of DoFs. */
        void resizeAll(int n);
        void lockAndResizeAll(int n);

        /** Set the parameters of the adaptive window filter used for velocity estimation. */
        bool setVelFiltParams(int windowLength, double threshold);
        /** Set the parameters of the adaptive window filter used for acceleration estimation. */
        bool setAccFiltParams(int windowLength, double threshold);
        /** Set the parameters of the adaptive window filter used for joint torque derivative estimation. */
        bool setDtauJFiltParams(int windowLength, double threshold);
        /** Set the parameters of the adaptive window filter used for motor torque derivative estimation. */
        bool setDtauMFiltParams(int windowLength, double threshold);
        /** Set the cut frequency of the joint torque low pass filter. */
        bool setTauJCutFrequency(double fc);
        /** Set the cut frequency of the motor torque low pass filter. */
        bool setTauMCutFrequency(double fc);
        /** Set the cut frequency of the motor PWM low pass filter. */
        bool setPwmCutFrequency(double fc);

    public:
        /**
         * Set the cut frequency of the velocities low pass filter.
         *
         * @param fc the new cutoff frequency. If negative the filter is disabled
         *
         * @return true if succeded, false otherwise.
         */
        bool setVelocitiesCutFrequency(double fc);


    public:

        yarp::os::Semaphore         mutex;          // mutex for access to class global variables

        // the elements of this struct are accessed by the state interface
        // the state interface takes the mutex before accessing this struct
        struct
        {
            yarp::sig::Vector lastQ;                    // last joint position estimation
            yarp::sig::Vector lastDq;                   // last joint velocity estimation
            yarp::sig::Vector lastD2q;                  // last joint acceleration estimation
            yarp::sig::Vector lastQM;                   // last motor position estimation
            yarp::sig::Vector lastDqM;                  // last motor velocity estimation
            yarp::sig::Vector lastD2qM;                 // last motor acceleration estimation
            yarp::sig::Vector lastTauJ;                 // last joint torque
            yarp::sig::Vector lastTauM;                 // last motor torque
            yarp::sig::Vector lastDtauJ;                // last joint torque derivative
            yarp::sig::Vector lastDtauM;                // last motor torque derivative
            yarp::sig::Vector lastPwm;                  // last motor PWM
            yarp::sig::Vector lastPwmBuffer;            // buffer for proper decoupling PWM readings
            yarp::sig::Vector lastBasePos;                // last Base Position
            yarp::sig::Vector lastBaseVel;                // last Base Velocity
            yarp::sig::Vector lastBaseAcc;                // last Base Acceleration
        }
        estimates;

        /** Matrix such that m_dot = joint_kinematic_to_motor_kinematic_coupling*q_dot */
        Eigen::MatrixXd joint_to_motor_kinematic_coupling;

        /** Matrix such that tau_m = joint_kinematic_to_motor_kinematic_coupling*tau_joint */
        Eigen::MatrixXd joint_to_motor_torque_coupling;

        /** If true, read speed and accelerations from the controlboard */
        bool readSpeedAccFromControlBoard;

        bool motor_quantites_estimation_enabled;

        /** If true, perform base position and velocity estimation */
        bool estimateBaseState;

        /** helper for base state estimation */
        bool use_localFloatingBaseStateEstimator;
        localFloatingBaseStateEstimator localFltBaseStateEstimator;

        bool use_remoteFloatingBaseStateEstimator;
        remoteFloatingBaseStateEstimator remoteFltBaseStateEstimator;

        /** Constructor.
         */
        yarpWholeBodyEstimator(int period_in_ms, double cutOffFrequencyTorqueInHz, yarpWbi::yarpWholeBodySensors *_sensors);

        bool lockAndSetEstimationParameter(const wbi::EstimateType et,
                                           const wbi::EstimationParameter ep,
                                           const void *value);

        bool threadInit();
        void run();
        void threadRelease();

        /** Take the mutex and copy the content of src into dest. */
        bool lockAndCopyVector(const yarp::sig::Vector &src, double *dest);
        /** Take the mutex and copy the i-th element of src into dest. */
        bool lockAndCopyVectorElement(int i, const yarp::sig::Vector &src, double *dest);

    };


    /**
     * Class to access the estimates of the state of an YARP robot.
     * You can configure this object with a yarp::os::Property object, that you can
     * pass to the constructor or to the setYarpWbiProperties method.
     *
     * The option that this Property object should contain at first level are:
     *
     * | Parameter name | Type | Units | Default Value | Required | Description | Notes |
     * |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
     * | readSpeedAccFromControlBoard | string | - | - | No | If present, read speeds and accelerations from the low level controlboards instead of using an high level numerical derivative.  |  |
     *
     * All other options should be placed in the WBI_STATE_OPTIONS group, to separate
     * them from the other yarpWholeBodyInterface option when placed all together in the .ini configuration file.
     *
     * # WBI_STATE_OPTIONS
     *
     * Notice that for estimating floating base state (position, velocity and acceleration)
     * of the floating base with respect to the inertial/world frame, you have to explictly
     * enable this estimation in the configuration file using the estimateBaseState option. After that, you can either
     * read this estimate externally (reading it from a yarp port) using the `externalFloatingBaseStatePort`
     * or computing it internally by assuming that a frame is always fixed with respect to the inertial/world .
     * You can specify this frame that is always fixed using the `localWorldReferenceFrame` option.
     *
     * | Parameter name | Type | Units | Default Value | Required | Description | Notes |
     * |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
     * | estimatorPeriod       | double | milliseconds | 10 | No | Period (in milliseconds) of the estimator thread | For undeliyng limitations of the yarp::os::RateThread class, this period should not be lower of 1.0 ms . |
     * | estimateBaseState | - | - | - | No | Necessary for estimation of root roto translation and velocity. If not present these estimates will always return 0  |
     * | externalFloatingBaseStatePort     | string | - | - | - | If present, reads the floating base state (position, velocities and acceleration from an external port, using the format described in remoteFloatingBaseStateEstimator class. | Not compatible with localWorldReferenceFrame option  |
     * | localWorldReferenceFrame | string | - | - | No | If present, specifies the default frame for computation of the world-to-root rototranslation.  | Not compatible with the externalFloatingBaseStatePort |
     * | cutOffFrequencyTorqueInHz  | double | Hz | 3.0 | No | Specify the cutoff frequency of the first order filter used to filter joint torque measurements, motor torque measurements and pwm | |
     * | cutOffFrequencyVelocitiesInHz | double | Hz | (If not present, no filter is used) | No | If present, specify the cutoff frequency of the first order filter used to filter joint velocities measurements. If not present, no filter is used. | | 
     * 
     * Furthermore for accessing joint sensors, the property should contain all the information used
     * for configuring a a yarpWholeBodyActuators object.
     *
     * # FILTERS
     *
     * For historical reasons, the yarpWholeBodyStates always filters the readed torques and pwm with a first order filter,
     * while the joint velocities are filtered only if the cutOffFrequencyVelocitiesInHz is present in the config file, 
     * and joint acceleration are the one returned directly by the controlboard.
     *
     */
    class yarpWholeBodyStates : public wbi::iWholeBodyStates
    {
    protected:
        bool initDone;
        std::string name;
        yarp::os::Property wbi_yarp_properties;

        yarpWbi::yarpWholeBodySensors        *sensors;       // interface to access the robot sensors
        yarpWholeBodyEstimator      *estimator;     // estimation thread
        wbi::IDList               emptyList;      ///< empty list of IDs to return in case of error

        //List of IDList for each estimate
        std::vector<wbi::IDList> estimateIdList;

        virtual bool lockAndReadSensor(const wbi::SensorType st, const int numeric_id, double *data, double time, bool blocking);
        virtual bool lockAndReadSensors(const wbi::SensorType st, double *data, double time, bool blocking);
        virtual bool lockAndAddSensor(const wbi::SensorType st, const wbi::ID &sid);
        virtual int lockAndAddSensors(const wbi::SensorType st, const wbi::IDList &sids);
        virtual bool lockAndRemoveSensor(const wbi::SensorType st, const wbi::ID &sid);
        virtual wbi::IDList lockAndGetSensorList(const wbi::SensorType st);
        virtual int lockAndGetSensorNumber(const wbi::SensorType st);
        //virtual bool lockAndGetExternalWrench(const wbi::LocalId sid, double * data);

        /** Get the velocity of the specified motor. */
        bool getMotorVel(const int numeric_id, double *data, double time, bool blocking);
        /** Get the velocities of all the robot motors. */
        bool getMotorVel(double *data, double time, bool blocking);

        // Motor-quantities estimation
        // For now we support motor quantites estimation by assuming a stiff actuation
        // and knowledge of the coupling matrix
        bool loadCouplingsFromConfigurationFile();


        // End motor-quantites estimation

        // Configure (using options provided by a configuration file)
        // the estimate of the floating base state
        bool configureFloatingBaseStateEstimator();

        // Pointer to a wholeBodyModel
        wbi::iWholeBodyModel * wholeBodyModel;


    public:
        // *** CONSTRUCTORS ***
        yarpWholeBodyStates(const char* _name, const yarp::os::Property & _wbi_yarp_conf, wbi::iWholeBodyModel *wholeBodyModelRef=NULL);
        virtual ~yarpWholeBodyStates();

        virtual bool init();
        virtual bool close();

        /**
         * Set the properties of the yarpWbiActuactors interface
         * Note: this function must be called before init, otherwise it takes no effect
         * @param yarp_wbi_properties the properties of the yarpWholeBodyActuators object
         */
        virtual bool setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties);

        /**
         * Get the properties of the yarpWbiActuactors interface
         * @param yarp_wbi_properties the properties of the yarpWholeBodyActuators object
         */
        virtual bool getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties);


        /** Add the specified estimate so that it can be read.
         * @param st Type of estimate.
         * @param sid Id of the estimate.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual bool addEstimate(const wbi::EstimateType st, const wbi::ID &sid);

        /** Add the specified estimates so that they can be read.
         * @param st Type of estimates.
         * @param sids Ids of the estimates.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual int addEstimates(const wbi::EstimateType st, const wbi::IDList &sids);

        /** Remove the specified estimate.
         * @param st Type of the estimate to remove.
         * @param j Id of the estimate to remove.
         * @return True if the estimate has been removed, false otherwise.
         */
        virtual bool removeEstimate(const wbi::EstimateType st, const wbi::ID &sid);

        /** Get a copy of the estimate list of the specified estimate type.
         * @param st Type of estimate.
         * @return A copy of the estimate list. */
        virtual const wbi::IDList& getEstimateList(const wbi::EstimateType st);

        /** Get the number of estimates of the specified type.
         * @return The number of estimates of the specified type. */
        virtual int getEstimateNumber(const wbi::EstimateType st);

        /** Get the estimate of the specified quantity at the specified time.
         * @param et Type of estimate to get.
         * @param estimate_numeric_id Id of the estimate
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimate(const wbi::EstimateType et, const int estimate_numeric_id, double *data, double time=-1.0, bool blocking=true);

        /** Get all the estimates of the specified estimate type at the specified time.
         * @param et Type of estimate to get.
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimates(const wbi::EstimateType et, double *data, double time=-1.0, bool blocking=true);

        /** Set the value of the specified parameter of the estimation algorithm
         * of the specified estimate type.
         * @param et Estimation type (e.g. joint velocity, motor torque).
         * @param ep Parameter to set.
         * @param value Value of the parameter to set.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool setEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value);
    };


}

#endif
