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

#ifndef WBSENSORS_ICUB_H
#define WBSENSORS_ICUB_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <wbiIcub/wbiIcubUtil.h>
#include <map>


namespace wbiIcub
{

    /**
     * Class for reading the sensors of a yarp robot.
     */
    class yarpWholeBodySensors: public wbi::iWholeBodySensors
    {
    protected:
        bool                        initDone;
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot

        //std::vector<int>            bodyParts;      // list of the body parts
        std::vector<std::string>    controlBoardNames;  // names of the body parts
        std::vector<id_2_PortName>  ftSens_2_port;  // list containing the port name for each force/torque sensor
        std::vector<id_2_PortName>  imu_2_port;     // list containing the port name for each IMU
        std::map<int,unsigned int>  bodyPartAxes;   // number of axes for each body part

        wbi::wbiIdList            emptyList;      ///< empty list of IDs to return in case of error
        wbi::wbiIdList            encoderIdList;  // list of the joint encoder ids
        wbi::wbiIdList            pwmSensIdList;  // list of the motor PWM sensor ids
        wbi::wbiIdList            imuIdList;      // list of the IMU ids
        wbi::wbiIdList            ftSensIdList;   // list of the force/torque sensor ids
        wbi::wbiIdList            torqueSensorIdList; //list of the torque sensor ids

        // LAST READING DATA (map body parts to data)
        std::map<int, yarp::sig::Vector>            qLastRead;
        std::map<int, yarp::sig::Vector>            qStampLastRead;
        std::map<int, yarp::sig::Vector>            pwmLastRead;
        std::map<int, yarp::sig::Vector>            torqueSensorsLastRead;

        // the "key" of these vectors is the wbi numeric id
        std::vector<yarp::sig::Vector>  imuLastRead;
        std::map<wbi::wbiId, yarp::sig::Vector>  ftSensLastRead;
        std::map<wbi::wbiId, double>  imuStampLastRead;
        std::map<wbi::wbiId, double>  ftStampSensLastRead;

        // yarp interfaces (the "key" of these vector is wbi numeric controlboard id
        std::vector<yarp::dev::IEncodersTimed*>       ienc;   // interface to read encoders
        std::vector<yarp::dev::IOpenLoopControl*>     iopl;   // interface to read motor PWM
        std::vector<yarp::dev::PolyDriver*>           dd; //device drivers
        std::vector<yarp::dev::ITorqueControl*>       itrq;  // interface to read joint torques

        // input ports (the key of the maps is the wbi numeric sensor id)
        std::vector< yarp::os::BufferedPort<yarp::sig::Vector>*>   portsFTsens;
        std::vector< yarp::os::BufferedPort<yarp::sig::Vector>*>   portsIMU;
        std::vector< yarp::os::BufferedPort<yarp::sig::Vector>*>   portsTorqueSensor;

        bool openPwm(const int controlBoard);
        bool openEncoder(const int controlBoard);
        bool openImu(const wbi::wbiId &i);
        bool openFTsens(const wbi::wbiId &i);
        bool openTorqueSensor(const int controlBoard);

        bool convertIMU(double * wbi_inertial_readings, const double * yarp_inertial_readings);

        // *** ENCODERS
        virtual bool addEncoder(const wbi::wbiId &j);
        virtual int addEncoders(const wbi::wbiIdList &j);
        // *** PWMs
        virtual bool addPwm(const wbi::wbiId &j);
        virtual int addPwms(const wbi::wbiIdList &j);
        // *** IMUs
        virtual bool addIMU(const wbi::wbiId &i);
        virtual int addIMUs(const wbi::wbiIdList &i);
        // *** FORCE/TORQUE SENSORS
        virtual bool addFTsensor(const wbi::wbiId &i);
        virtual int addFTsensors(const wbi::wbiIdList &i);
        // *** TORQUE SENSORS *** //
        virtual bool addTorqueSensor(const wbi::wbiId &i);
        virtual int addTorqueSensors(const wbi::wbiIdList &i);

        virtual bool readEncoder(const wbi::wbiId &i, double *q, double *stamps=0, bool wait=true);
        virtual bool readPwm(const wbi::wbiId &i, double *pwm, double *stamps=0, bool wait=true);
        virtual bool readIMU(const wbi::wbiId &i, double *inertial, double *stamps=0, bool wait=true);
        virtual bool readFTsensor(const wbi::wbiId &i, double *ftSens, double *stamps=0, bool wait=true);
        virtual bool readTorqueSensor(const wbi::wbiId &i, double *jointTorque, double *stamps=0, bool wait=true);

        virtual bool readEncoders(double *q, double *stamps=0, bool wait=true);
        virtual bool readPwms(double *pwm, double *stamps=0, bool wait=true);
        virtual bool readIMUs(double *inertial, double *stamps=0, bool wait=true);
        virtual bool readFTsensors(double *ftSens, double *stamps=0, bool wait=true);
        virtual bool readTorqueSensors(double *jointTorques, double *stamps=0, bool wait=true);

    public:
        // *** CONSTRUCTORS ***
        yarpWholeBodySensors(const char* _name, const char* _robotName);

        /**
          * @param _name Local name of the interface (used as stem of port names)
          * @param _robotName Name of the robot
          * @param _yarp_wbi_properties yarp::os::Property object used to configure the interface
          */
        yarpWholeBodySensors(const char* _name, const char* _robotName,
                             yarp::os::Property & _yarp_wbi_properties);

        virtual bool init();
        virtual bool close();

        /**
         * Add the specified sensor so that it can be read.
         * @param st Type of sensor.
         * @param sid Id of the sensor.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual bool addSensor(const wbi::SensorType st, const wbi::wbiId &sid);

        /**
         * Add the specified sensors so that they can be read.
         * @param st Type of sensors.
         * @param sids Ids of the sensors.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual int addSensors(const wbi::SensorType st, const wbi::wbiIdList &sids);

        /**
         * Remove the specified sensor.
         * @param st Type of the sensor to remove.
         * @param j Id of the sensor to remove.
         * @return True if the sensor has been removed, false otherwise.
         */
        virtual bool removeSensor(const wbi::SensorType st, const wbi::wbiId &sid);

        /**
         * Get a copy of the sensor list of the specified sensor type.
         * @param st Type of sensors.
         * @return A copy of the sensor list. */
        virtual const wbi::wbiIdList& getSensorList(const wbi::SensorType st);

        /**
         * Get the number of sensors of the specified type.
         * @return The number of sensors of the specified type. */
        virtual int getSensorNumber(const wbi::SensorType st);

        /**
         * Read the specified sensor.
         * @param st Type of sensor to read.
         * @param sid Id of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readSensor(const wbi::SensorType st, const wbi::wbiId &sid,
                                double *data, double *stamps=0, bool blocking=true);

        /**
         * Read all the sensors of the specified type.
         * @param st Type of the sensor to read.
         * @param sid Id of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if the reading succeeded, false otherwise.
         */
        virtual bool readSensors(const wbi::SensorType st, double *data, double *stamps=0, bool blocking=true);
    };

}

#endif


