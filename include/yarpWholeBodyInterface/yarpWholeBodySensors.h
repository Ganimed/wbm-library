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

#include "yarpWholeBodyInterface/yarpWbiUtil.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
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
    enum AccelerometerType { IMU_STYLE };

    /** List of available encoder types. */
    enum EncoderType
    {
        /* Position Encoder and its speed/acceleration estimation
         by the control board firmware */
        ENCODER_POS,
        ENCODER_SPEED,
        ENCODER_ACCELERATION
    };

    /**
     * Struct for holding information about loaded accelerometers
     */
    struct AccelerometerConfigurationInfo
    {
        AccelerometerType type;
        std::string type_option;
    };

    /**
     * Struct for holding information about loaded accelerometers
     */
    struct AccelerometerRuntimeInfo
    {
        AccelerometerType type;
        int type_reference_index;
    };



    /**
     * Class for reading the sensors of a yarp robot.
     *
     * You can configure this object with a yarp::os::Property object, that you can
     * pass to the constructor. Alternativly you can set the Property through the setYarpWbiProperties method,
     * but in that case you have to set the property before calling the init method.
     * 
     *
     */
    class yarpWholeBodySensors: public wbi::iWholeBodySensors
    {
    protected:
        bool                        initDone;
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot

        yarp::os::Property          wbi_yarp_properties; //configuration options

        //std::vector<int>            bodyParts;      // list of the body parts
        std::vector<std::string>    controlBoardNames;  // names of the body parts
        std::vector<unsigned int>   controlBoardAxes;   // number of axes for each body part


        //List of controlboard related sensors
        ///< empty list of IDs to return in case of error
        wbi::IDList            emptyList;

        ///< list of sensor IDs, indexed by wbi::SensorType enum
        std::vector<wbi::IDList>            sensorIdList;
        ///< list of controlboard numeric IDs (i.e. indeces of controlBoardNames vector) for which encoder are added
        std::vector<int>          encoderControlBoardList;
        ///< map from encoder numeric IDs (i.e. indeces of encoderIdList vector) to a pair of int that are: (controlboard id,axis)
        std::vector< std::pair<int,int> >  encoderControlBoardAxisList;

        //wbi::IDList            pwmSensIdList;  // list of the motor PWM sensor ids
        std::vector<int>          pwmControlBoardList;
        std::vector< std::pair<int,int> > pwmControlBoardAxisList;

        //wbi::IDList            torqueSensorIdList; //list of the torque sensor ids
        std::vector<int>          torqueControlBoardList;
        std::vector< std::pair<int,int> > torqueControlBoardAxisList;

        //List of sensors that have their own device
        //wbi::IDList            imuIdList;      // list of the IMU ids
        //wbi::IDList            ftSensIdList;   // list of the force/torque sensor ids

        // LAST READING DATA (map controlboard numeric IDs (i.e. indeces of controlBoardNames vector) to data)
        std::vector<yarp::sig::Vector>            qLastRead;
        std::vector<yarp::sig::Vector>            qStampLastRead;
        std::vector<yarp::sig::Vector>            pwmLastRead;
        std::vector<yarp::sig::Vector>            torqueSensorsLastRead;

        // the "key" of these vectors is the wbi numeric id
        std::vector<yarp::sig::Vector>  imuLastRead;
        std::vector<double>  imuStampLastRead;
        std::vector<yarp::sig::Vector>  ftSensLastRead;
        std::vector<double>  ftStampSensLastRead;
        std::vector<yarp::sig::Vector> accLastRead;
        std::vector<double>  accStampLastRead;

        // yarp interfaces (the "key" of these vector is wbi numeric controlboard id
        std::vector<yarp::dev::IEncodersTimed*>       ienc;   // interface to read encoders
        std::vector<yarp::dev::IOpenLoopControl*>     iopl;   // interface to read motor PWM
        std::vector<yarp::dev::PolyDriver*>           dd; //device drivers
        std::vector<yarp::dev::ITorqueControl*>       itrq;  // interface to read joint torques

        // input ports (the key of the maps is the wbi numeric sensor id)
        std::vector< yarp::os::BufferedPort<yarp::sig::Vector>*>   portsFTsens;
        std::vector< yarp::os::BufferedPort<yarp::sig::Vector>*>   portsIMU;
        std::vector< yarp::os::BufferedPort<yarp::sig::Vector>*>   portsTorqueSensor;

        // reference to other sensor (for accelerometers we always get their information
        //  from another sensor, such as the IMU)
        std::vector< AccelerometerRuntimeInfo > accelerometersReferenceIndeces;



        //ControlBoard oriented sensors
        bool openPwm(const int controlBoard);
        bool openEncoder(const int controlBoard);
        bool openTorqueSensor(const int controlBoard);

        //
        bool loadAccelerometerInfoFromConfig(const yarp::os::Searchable & opts,
                                        const wbi::IDList & list,
                                        std::vector<AccelerometerConfigurationInfo> & infos);

        //Indipendent sensors
        bool openImu(const int id, const std::string & port_name);
        bool openFTsens(const int id, const std::string & port_name);
        bool openAccelerometer(const int id, const AccelerometerConfigurationInfo & info);

        bool convertIMU(double * wbi_inertial_readings, const double * yarp_inertial_readings);


        virtual bool readEncoder(const EncoderType st, const int id, double *data, double *stamps=0, bool wait=true);
        virtual bool readPwm(const int id, double *pwm, double *stamps=0, bool wait=true);
        virtual bool readIMU(const int id, double *inertial, double *stamps=0, bool wait=true);
        virtual bool readFTsensor(const int id, double *ftSens, double *stamps=0, bool wait=true);
        virtual bool readTorqueSensor(const int id, double *jointTorque, double *stamps=0, bool wait=true);
        virtual bool readAccelerometer(const int id, double *acc, double *stamps=0, bool wait=true);

        virtual bool readEncoders(const EncoderType st, double *data, double *stamps=0, bool wait=true);
        virtual bool readPwms(double *pwm, double *stamps=0, bool wait=true);
        virtual bool readIMUs(double *inertial, double *stamps=0, bool wait=true);
        virtual bool readFTsensors(double *ftSens, double *stamps=0, bool wait=true);
        virtual bool readTorqueSensors(double *jointTorques, double *stamps=0, bool wait=true);
        virtual bool readAccelerometers(double *accs, double *stamps=0, bool wait=true);

        bool getEncodersPosSpeedAccTimed(const EncoderType st, yarp::dev::IEncodersTimed* ienc, double *encs, double *time);

    public:
        /**
         *
         *
         * @param _name Local name of the interface (used as stem of port names)
         * @param _yarp_wbi_properties yarp::os::Property object used to configure the interface
         */
        yarpWholeBodySensors(const char* _name,
                             const yarp::os::Property & _yarp_wbi_properties=yarp::os::Property());

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


        /**
         * Add the specified sensor so that it can be read.
         * @param st Type of sensor.
         * @param sid Id of the sensor.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual bool addSensor(const wbi::SensorType st, const wbi::ID &sid);

        /**
         * Add the specified sensors so that they can be read.
         * @param st Type of sensors.
         * @param sids Ids of the sensors.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual int addSensors(const wbi::SensorType st, const wbi::IDList &sids);

        /**
         * Remove the specified sensor.
         * @param st Type of the sensor to remove.
         * @param j Id of the sensor to remove.
         * @return True if the sensor has been removed, false otherwise.
         */
        virtual bool removeSensor(const wbi::SensorType st, const wbi::ID &sid);

        /**
         * Get a copy of the sensor list of the specified sensor type.
         * @param st Type of sensors.
         * @return A copy of the sensor list. */
        virtual const wbi::IDList& getSensorList(const wbi::SensorType st);

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
        virtual bool readSensor(const wbi::SensorType st, const int sensor,
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


