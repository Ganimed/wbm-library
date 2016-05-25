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

#ifndef WBI_ICUB_UTIL_H
#define WBI_ICUB_UTIL_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <wbi/wbi.h>
#include <vector>
#include <cstdio>

/* CODE UNDER DEVELOPMENT */

namespace KDL {
    namespace CoDyCo {
        class TreeSerialization;
    }
}

namespace yarpWbi
{
    extern const double Rad2Deg; /*<! Radians to degrees conversion */
    extern const double Deg2Rad; /*<! Degrees to radians conversion */

    extern const std::string ErrorDomain;

    enum ErrorCode
    {
        ErrorCodeUnknown = 1, /*<! Unknown error */
        ErrorCodeGeneric = 1 << 1, /*<! Generic error */
        ErrorCodeNotImplementedYet = 1 << 2, /*<! Feature not implemented yet error */
        ErrorCodeIndexOutOfRange = 1 << 3, /*<! Index out of range error */
        ErrorCodePropertyNotSupported = 1 << 4, /*<! Property not supported error */
        ErrorCodeConfigurationNotValid = 1 << 5, /*<! Configuration not valid error */
    };


    /** Open a remote control board driver for the specified body part.
     * @param localName Name to use as stem for the names of the YARP ports to open.
     * @param robotName Name of the robot to connect to.
     * @param pd Pointer to the poly driver to instanciate.
     * @param bodyPartName Name of the body part for which to open the poly driver.
     * @return True if the operation succeeded, false otherwise. */
    bool openPolyDriver(const std::string &localName,
                        const std::string &robotName,
                          yarp::dev::PolyDriver *&pd,
                        const std::string &bodyPartName);

    bool closePolyDriver(yarp::dev::PolyDriver *&pd);


    /*
    bool loadControlBoardsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 std::vector<std::string> & body_parts_vector);

    bool loadReverseTorsoJointsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                          bool &reverse_torso_joints);
    */




    bool loadTreeSerializationFromConfig(yarp::os::Property & wbi_yarp_properties,
                                         KDL::CoDyCo::TreeSerialization& serialization);


    bool loadJointsControlBoardFromConfig(yarp::os::Property & wbi_yarp_properties,
                                          const wbi::IDList & jointIdList,
                                          std::vector<std::string> & controlBoardNames,
                                          std::vector< std::pair<int,int> > & controlBoardAxisList);

    std::map<std::string,int> getControlBoardIdsMap(const std::vector<std::string> & controlBoardNames);

    yarp::os::Bottle & getWBIYarpJointsOptions(yarp::os::Property & wbi_yarp_properties);

    bool appendNewControlBoardsToVector(yarp::os::Bottle & joints_config,
                                    const wbi::IDList & jointIdList,
                                    std::vector<std::string> & controlBoardNames);

    bool getControlBoardAxisList(yarp::os::Bottle & joints_config,
                                                          const wbi::IDList &jointIdList,
                                                          const std::vector<std::string>& controlBoardNames,
                                                          std::vector< std::pair<int,int> > & controlBoardAxisList);

    std::vector< int > getControlBoardList(const std::vector< std::pair<int,int> > & controlBoardAxisList);

    bool loadSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                               const wbi::IDList & sensorIdList,
                               std::vector<std::string> & ports,
                               const std::string group_name);

    bool loadFTSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 wbi::IDList & sensorIdList,
                                 std::vector<std::string> & ports);

    bool loadIMUSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                      wbi::IDList & sensorIdList,
                                      std::vector<std::string> & ports);

    /**
     * Get the requested joint id list from a yarp::os::Searchable
     *
     * The Searchable should contain a group
     * under which the list are defined as Yarp lists.
     *
     * It is also possible to define a list as a sequence of lists,
     *  as long as the list are defined in the same property.
     *
     * @param requested_list name of the requested list
     * @param wbi_yarp_properties Searchable that contains the list group
     * @param requestedIdList reference of an IDList object in which to load the list
     * @param list_group name of the group in which to find the lists (default: WBI_ID_LISTS)
     * @return true if the list was loaded correctly, false otherwise
     */
    bool loadIdListFromConfig(std::string requested_list,
                              const yarp::os::Searchable & wbi_yarp_properties,
                              wbi::IDList & requestedIdList,
                              std::string list_group = "WBI_ID_LISTS", bool verbose=false);


    /**
     * Get string representation of the specified integer
     *
     * @param num the number to be converted to string
     * @return a string representation of the specified number
     */
    std::string stringFromInt(int num);


    /**
     * Fill the pid object fields with the content coming from the
     * specified bottle.
     * @note: If some fields are not specified, their values are left untouched
     *
     * The parameters that can be set are (@see http://www.yarp.it/classyarp_1_1dev_1_1Pid.html for their description)
     *
     *     - kp
     *     - kd
     *     - ki
     *     - stic_up
     *     - stic_down
     *
     * @param[in] bottle bottle representation of the PID object
     * @param[out] pid    pid object with the fields filled by its bottle representation
     *
     * @return true if succeded. False otherwise
     */
    bool pidFromBottleDescription(const yarp::os::Bottle &bottle,
                                  yarp::dev::Pid &pid);

    /**
     * Fill the motor torque parameters object fields
     * with the content coming from the specified bottle.
     * @note: If some fields are not specified, their values are left untouched
     *
     * The parameters that can be set are (@see http://www.yarp.it/classyarp_1_1dev_1_1MotorTorqueParameters.html for their description)
     *
     *     - bmef
     *     - ktau
     *
     * @param[in] bottle          bottle representation of the motor parameters object
     * @param[out] motorParameters object with the fields filled by its bottle representation
     *
     * @return true if succeded. False otherwise
     */
    bool motorTorqueParametersFromBottleDescription(const yarp::os::Bottle &bottle,
                                                    yarp::dev::MotorTorqueParameters& motorParameters);


    /**
     * Load the used wbi::IDList from the WBI configuration and the list parameter
     *
     *  Currently the following format for the listSpecification parameter is supported:
     *  - a single element representing the name of a list
     *  - a yarp bottle (aka list) which can contains either joint names and/or list name (mixed together)
     *  @note: a single indirection is allowed, i.e. the following is not allowed
     * (LIST_1, joint_1, (joint_2), joint_3)
     *
     * @param[in] wbiConfigProp         wbi configuration file
     * @param[in] listSpecification     the description of list to be loaded.
     * @param[out] idList               output list of joint loaded
     *
     * @return true if loading the list was successful, false otherwise.
     */
    bool idListFromParsedConfigurationOption(const yarp::os::Property& wbiConfigProp,
                                             const yarp::os::Value& listSpecification,
                                             wbi::IDList& idList);


} // end namespace yarpWbi

#endif
