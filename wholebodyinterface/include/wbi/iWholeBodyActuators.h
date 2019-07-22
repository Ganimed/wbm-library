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

#ifndef IWHOLEBODYACTUATORS_H
#define IWHOLEBODYACTUATORS_H

#include <wbi/wbiConstants.h>

namespace wbi {
    class ID;
    class IDList;

    /**
     * Interface to the actuators of the robot.
     */
    class iWholeBodyActuators
    {
    public:
        virtual ~iWholeBodyActuators();
        virtual bool init() = 0;
        virtual bool close() = 0;

        //virtual int getActuatorNumber() = 0;
        virtual bool removeActuator(const ID &j) = 0;
        virtual bool addActuator(const ID &j) = 0;
        virtual int addActuators(const IDList &j) = 0;
        virtual const IDList& getActuatorList() = 0;

        /** 
         * Set the control mode of the specified joint(s).
         * 
         * Depending on the implementation, this method can be slow. 
         * It should be used before starting the control to switch
         * the control mode to the desired one. Once the control mode
         * is set, you should send reference in the control loop using the
         * setControlReference method. 
         * 
         * For the unit of measurement of the the ref argument,
         *  please check the setControlReference method. 
         * 
         * @param controlMode Id of the control mode.
         * @param ref Reference value(s) for the controller.
         * @param joint Joint number, if negative, all joints are considered.
         * @return True if operation succeeded, false otherwise.
         */
        virtual bool setControlMode(ControlMode controlMode, double *ref=0, int joint=-1) = 0;

        /** 
         * Set the reference value for the controller of the specified joint(s).
         * 
         * For control modes that take in input a  desired position of a revolute joint,
         *  the unit of measurement of the reference value is Radians.
         * 
         * For control modes that take in input a desired velocity of a revolute joint,
         *  the unit of measurement of the reference value is Radians/Seconds.
         * 
         * For control modes that take in input a desired torque of a revolute joint,
         *  the unit of measurement of the reference value is Newton*Meters.
         * 
         * @param ref Reference value(s) for the controller.
         * @param joint Joint number, if negative, all joints are considered.
         * @return True if operation succeeded, false otherwise.
         */
        virtual bool setControlReference(double *ref, int joint=-1) = 0;

        /** Set a parameter (e.g. a gain) of one or more joint controllers.
         * @param paramId Id of the parameter.
         * @param value Value(s) of the parameter.
         * @param joint Joint number, if negative, all joints are considered.
         * @return True if operation succeeded, false otherwise. */
        virtual bool setControlParam(ControlParam paramId, const void *value, int joint=-1) = 0;
    };

}

#endif //IWHOLEBODYACTUATORS_H
