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
#include <iCub/skinDynLib/common.h>
#include <wbi/wbi.h>
#include <vector>
#include <cstdio>

#include <kdl_codyco/treeserialization.hpp>
#include <kdl_codyco/treepartition.hpp>

/* CODE UNDER DEVELOPMENT */

namespace yarpWbi
{
    /** Return true if the robotName is "icubSim", false otherwise (deprecated function, do not use). */
    inline bool isICubSimulator(const std::string &robotName)
    { return robotName=="icubSim"; }


    /** Return true if the robotName is "icubSim" , false otherwise (deprecated function, do not use). */
    inline bool isRobotSimulator(const std::string &robotName)
    { return isICubSimulator(robotName); }

    /** Open a remote control board driver for the specified body part.
     * @param localName Name to use as stem for the names of the YARP ports to open.
     * @param robotName Name of the robot to connect to.
     * @param pd Pointer to the poly driver to instanciate.
     * @param bodyPartName Name of the body part for which to open the poly driver.
     * @return True if the operation succeeded, false otherwise. */
    inline bool openPolyDriver(const std::string &localName, const std::string &robotName, yarp::dev::PolyDriver *&pd, const std::string &bodyPartName)
    {
        std::string localPort  = "/" + localName + "/" + bodyPartName;
        std::string remotePort = "/" + robotName + "/" + bodyPartName;
        yarp::os::Property options;
        options.put("robot",robotName.c_str());
        options.put("part",bodyPartName.c_str());
        options.put("device","remote_controlboard");
        options.put("local",localPort.c_str());
        options.put("remote",remotePort.c_str());

        pd = new yarp::dev::PolyDriver(options);
        if(!pd || !(pd->isValid()))
        {
            std::fprintf(stderr,"Problems instantiating the device driver %s\n", bodyPartName.c_str());
            return false;
        }
        return true;
    }



    /*
    bool loadControlBoardsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 std::vector<std::string> & body_parts_vector);

    bool loadReverseTorsoJointsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                          bool &reverse_torso_joints);
    */




    bool loadTreeSerializationFromConfig(yarp::os::Property & wbi_yarp_properties,
                                         KDL::CoDyCo::TreeSerialization& serialization);

    bool loadTreePartitionFromConfig(yarp::os::Property & wbi_yarp_properties,
                                     KDL::CoDyCo::TreePartition& serialization);


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
                               wbi::IDList & sensorIdList,
                               std::vector<std::string> & ports,
                               const std::string group_name);

    bool loadFTSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 wbi::IDList & sensorIdList,
                                 std::vector<std::string> & ports);

    bool loadIMUSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                      wbi::IDList & sensorIdList,
                                      std::vector<std::string> & ports);

    /**
     * Get the requested joint id list from a yarp os Property
     *
     * The Property object should contain a WBI_ID_LISTS group
     * under which the list are defined as Yarp lists.
     *
     * It is also possible to define a list as a sequence of lists,
     *  as long as the list are defined in the same property.
     *
     */
    bool loadIdListFromConfig(std::string requested_list,
                              yarp::os::Property & wbi_yarp_properties,
                              wbi::IDList & requestedIdList);



} // end namespace wbiIcub

#endif
