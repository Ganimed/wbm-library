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

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/skinDynLib/skinContactList.h>

//#include <Eigen/Sparse>

#include "yarpWholeBodyInterface/yarpWbiUtil.h"
#include "yarpWholeBodyInterface/yarpWholeBodySensors.h"

#include <map>


namespace wbi {
    class ID;
    class IDList;
}

namespace yarpWbi
{
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
        
        
        int baseFrameLinkID; 	// ID of the assigned base frame for base to root rototranslation computation
        wbi::iWholeBodyModel *wholeBodyModel;
	
        

        int dqFiltWL, d2qFiltWL;                    // window lengths of adaptive window filters
        double dqFiltTh, d2qFiltTh;                 // threshold of adaptive window filters
        int dTauMFiltWL, dTauJFiltWL;               // window lengths of adaptive window filters
        double dTauMFiltTh, dTauJFiltTh;            // threshold of adaptive window filters
        double tauJCutFrequency;
        double tauMCutFrequency;
        double pwmCutFrequency;
	
	int robot_reference_frame_link;			//Reference link assigned as base frame
	wbi::Frame rootLink_H_ReferenceLink;		//Rototranslation between Reference frame (assigned as world) and Root Link
	wbi::Frame world_H_rootLink;			//Rototranslation between Root link and World
	wbi::Frame world_H_reference;			//Rototranslation between Reference frame and world (future work)
	wbi::Frame referenceLink_H_rootLink;		//Rototranslation between Root link and Reference frame
	Eigen::Matrix4d H_w2b;				//Temporary matrix used for inversion
	
	/*
	 * optimised computation of world-to-base velocity..todo fix and then uncomment
	Eigen::Map<Eigen::VectorXd> dqjVect;
	Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> complete_jacobian, joint_jacobian,floatingBase_jacobian;
	Eigen::Matrix<double,6,Eigen::Dynamic,Eigen::RowMajor> tempMatForComputation;//, minusTemp(6,dof);		
	Eigen::VectorXd dvbVect;
	Eigen::Map<Eigen::VectorXd> rotationalVelocityWrapper;//(estimates.lastBaseVel.data(), estimates.lastBaseVel.size());
	//new (&v) Map<RowVectorXi>(data+4,5);
	*/
	
        yarp::sig::Vector           q, qStamps;         // last joint position estimation
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

        /*
        std::map<wbi::ID, yarp::os::BufferedPort<yarp::sig::Vector>*>  portsEEWrenches;
        std::map<wbi::ID, yarp::sig::Vector>  lastEEWrenches;

        bool openEEWrenchPorts(const wbi::LocalId & local_id);
        void readEEWrenches(const wbi::LocalId & local_id,yarp::sig::Vector & vec);
        void closeEEWrenchPorts(const wbi::LocalId & local_id);

        yarp::sig::Matrix H_world_base;
        */

    public:
        // end effector wrenches ports (the key of the maps is the sensor id)
        //bool ee_wrenches_enabled;
        //bool openEEWrenchPorts();

        /*
        yarp::sig::Vector RAExtWrench;
        yarp::sig::Vector LAExtWrench;
        yarp::sig::Vector RLExtWrench;
        yarp::sig::Vector LLExtWrench;


        wbi::ID right_gripper_local_id;
        wbi::ID left_gripper_local_id;
        wbi::ID left_sole_local_id;
        wbi::ID right_sole_local_id;
        */

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
            yarp::sig::Vector lastBasePos;		// last Base Position
            yarp::sig::Vector lastBaseVel;		// last Base Velocity
            yarp::sig::Vector lastBaseAccl;		// last Base Acceleration
        }
        estimates;

        /** Matrix such that m_dot = joint_kinematic_to_motor_kinematic_coupling*q_dot */
        Eigen::MatrixXd joint_to_motor_kinematic_coupling;

        /** Matrix such that tau_m = joint_kinematic_to_motor_kinematic_coupling*tau_joint */
        Eigen::MatrixXd joint_to_motor_torque_coupling;

        bool motor_quantites_estimation_enabled;

        /** Constructor.
         */
        yarpWholeBodyEstimator(int _period, yarpWbi::yarpWholeBodySensors *_sensors, wbi::iWholeBodyModel *wholeBodyModelRef=NULL);

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
	/** To be implemented **/
        bool setWorldBasePosition(const wbi::Frame & xB);
	/** Sets a desired link as the world reference frame **/
	bool setWorldBaseLinkName(std::string);
	/** Computes the Base position for a given joint configuration **/
	bool computeBasePosition(double *q);
	/** Computes the Base velocity for a given set of joint velocities **/
	bool computeBaseVelocity(double *q, double *dq);
	/** Computes the Base acceleration for a given joint velocity **/
	//bool computeBaseAcceleration();

    };


    /**
     * Class to access the estimates of the states of iCub.
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
        //double                      estWind;      // time window for the estimation

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
	wbi::iWholeBodyModel *wholeBodyModel;

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

        virtual bool setWorldBasePosition(const wbi::Frame & xB);
    };


}

#endif
