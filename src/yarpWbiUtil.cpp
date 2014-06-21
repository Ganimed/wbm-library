/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete, Marco Randazzo
 * email: andrea.delprete@iit.it marco.randazzo@iit.it
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

#include <iostream>
#include <algorithm>

#include "yarpWholeBodyInterface/yarpWbiUtil.h"

static const std::string WBI_YARP_JOINTS_GROUP = "WBI_YARP_JOINTS";


namespace yarpWbi
{

yarp::os::Bottle & getWBIYarpJointsOptions(yarp::os::Property & wbi_yarp_properties)
{
    return wbi_yarp_properties.findGroup(WBI_YARP_JOINTS_GROUP);
}

bool appendNewControlBoardsToVector(yarp::os::Bottle & joints_config,
                                    const wbi::wbiIdList & jointIdList,
                                    std::vector<std::string> & controlBoardNames)
{
    for(int jnt=0; jnt < jointIdList.size(); jnt++ )
    {
        wbi::wbiId jnt_name;
        jointIdList.numericIdTowbiId(jnt,jnt_name);

        if( !joints_config.check(jnt_name.toString().c_str()) )
        {
            std::cout << "wholeBodyActuactors error: joint " << jnt_name.toString() <<
                         " not found in WBI_YARP_JOINTS section of configuration file " << std::endl;
            return false;
        }

        yarp::os::Bottle * ctrlBoard_mapping = joints_config.find(jnt_name.toString().c_str()).asList();
        if( !ctrlBoard_mapping || ctrlBoard_mapping->size() != 2 )
        {
            std::cout << "wholeBodyActuactors error: joint " << jnt_name.toString() <<
                         " found in WBI_YARP_JOINTS but is not in the canonical form (controlBoardName,axis)" << std::endl;
            return false;
        }

        std::string controlboard_name = ctrlBoard_mapping->get(0).asString().c_str();
        if (std::find(controlBoardNames.begin(), controlBoardNames.end(), controlboard_name) == controlBoardNames.end())
        {
            controlBoardNames.push_back(controlboard_name);
        }
    }
    return true;
}

std::vector< std::pair<int,int> > getControlBoardAxisList(yarp::os::Bottle & joints_config,
                                                          const wbi::wbiIdList &jointIdList,
                                                          const std::vector<std::string>& controlBoardNames)
{
    std::map<std::string,int> controlBoardIds = getControlBoardIdsMap(controlBoardNames);

    std::vector< std::pair<int,int> > controlBoardAxisList;
    controlBoardAxisList.resize(jointIdList.size());
    for(int wbi_jnt=0; wbi_jnt < jointIdList.size(); wbi_jnt++ )
    {
        wbi::wbiId wbi_jnt_name;
        jointIdList.numericIdTowbiId(wbi_jnt,wbi_jnt_name);

        yarp::os::Bottle * ctrlBoard_mapping = joints_config.find(wbi_jnt_name.toString().c_str()).asList();

        std::string controlboard_name = ctrlBoard_mapping->get(0).asString().c_str();
        int controlboard_jnt_axis = ctrlBoard_mapping->get(1).asInt();
        int controlboard_wbi_id   = controlBoardIds[controlboard_name];

        //totalControlledJointsInControlBoard[controlboard_wbi_id]++;
        controlBoardAxisList[wbi_jnt] = std::pair<int,int>(controlboard_wbi_id,controlboard_jnt_axis);
    }
    return controlBoardAxisList;

}

std::vector< int > getControlBoardList(const std::vector< std::pair<int,int> > & controlBoardAxisList)
{
    std::vector< int > controlBoardList;
    for(int jnt=0; jnt < controlBoardAxisList; jnt++ )
    {
        int wbi_ctrlBoard_numericId = controlBoardAxisList[jnt].first;
        if( std::find(controlBoardList.begin(), controlBoardList.end(), wbi_ctrlBoard_numericId) == controlBoardList.end() )
        {
            controlBoardList.push_back(wbi_ctrlBoard_numericId);
        }
    }
    return controlBoardList;
}

std::map<std::string,int> getControlBoardIdsMap(std::vector<std::string> & controlBoardNames)
{
    std::map<std::string,int> controlBoardIds;
    for(int ctrlBoard=0; ctrlBoard < controlBoardNames.size(); ctrlBoard++)
    {
        controlBoardIds[controlBoardNames[ctrlBoard]] = ctrlBoard;
    }
}

bool loadJointsControlBoardFromConfig(yarp::os::Property & wbi_yarp_properties,
                                      const wbi::wbiIdList & jointIdList,
                                      std::vector<std::string> & controlBoardNames,
                                      std::vector< std::pair<int,int> > & controlBoardAxisList)
{
    yarp::os::Bottle & joints_config = getWBIYarpJointsOptions(wbi_yarp_properties);

    //First check that all the joint in the jointIdList an appropriate controlboard mapping is defined
    controlBoardNames.clear();
    if( !appendNewControlBoardsToVector(joints_config,jointIdList,controlBoardNames) )
    {
        return false;
    }

    std::map<std::string,int> controlBoardIds = getControlBoardIdsMap(controlBoardNames);


    //Resizing structures
    controlBoardAxisList.resize(jointIdList.size());
    //totalControlledJointsInControlBoard.resize(controlBoardNames.size(),0);
    controlBoardAxisList = getControlBoardAxisList(joints_config,jointIdList);


    //All elements in the jointIdList have a mapping to an axis of a controlboard
    //we have to save this mapping
    for(int wbi_jnt=0; wbi_jnt < jointIdList.size(); wbi_jnt++ )
    {
        wbi::wbiId wbi_jnt_name;
        jointIdList.numericIdTowbiId(wbi_jnt,wbi_jnt_name);

        yarp::os::Bottle * ctrlBoard_mapping = joints_config.find(wbi_jnt_name.toString().c_str()).asList();

        std::string controlboard_name = ctrlBoard_mapping->get(0).asString().c_str();
        int controlboard_jnt_axis = ctrlBoard_mapping->get(1).asInt();
        int controlboard_wbi_id   = controlBoardIds[controlboard_name];

        //totalControlledJointsInControlBoard[controlboard_wbi_id]++;
        controlBoardAxisList[wbi_jnt] = std::pair<int,int>(controlboard_wbi_id,controlboard_jnt_axis);
    }

    return true;
}

/*
bool loadBodyPartsFromConfig(yarp::os::Property & wbi_yarp_properties, std::vector<std::string> & body_parts_vector)
{
        std::cout << "yarpWbi::loadBodyPartsFromConfig : config passed " << wbi_yarp_properties.toString() << std::endl;
        yarp::os::Bottle parts_config = wbi_yarp_properties.findGroup("WBI_YARP_BODY_PARTS");
        const std::string numBodyPartsOption = "numBodyParts";
        if( !parts_config.check(numBodyPartsOption) ) {
            std::cout << "yarpWbi::loadBodyPartsFromConfig error: " << numBodyPartsOption << " option not found" << std::endl;
            return false;
        }
        int numBodyParts = parts_config.find(numBodyPartsOption).asInt();
        std::cout << "yarpWbi::loadBodyPartsFromConfig : Loading body parts: expecting " << numBodyParts << " parts " << std::endl;
        body_parts_vector.resize(numBodyParts);
        for(int bp=0; bp < numBodyParts; bp++ ) {
            std::ostringstream bodyPart_strm;
            bodyPart_strm<<"bodyPart"<<bp;
            std::string bodyPart = bodyPart_strm.str();
            if( ! parts_config.check(bodyPart) ) {
                std::cout << "yarpWbi::loadBodyPartsFromConfig error: " << bodyPart << " name not found" << std::endl;
                return false;
            }
            body_parts_vector[bp] = parts_config.find(bodyPart).asString().c_str();
        }
        std::cout << "yarpWbi::loadBodyPartsFromConfig: Loaded body parts: ";
        for(int i=0; i < body_parts_vector.size(); i++ ) { std::cout << " " << body_parts_vector[i] << std::endl; }
        std::cout << std::endl;
        return true;
}

bool loadSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                               const std::vector<std::string> & body_parts_vector,
                               std::vector<id_2_PortName> &ports,
                               const std::string group_name)
{
    yarp::os::Bottle ports_list = wbi_yarp_properties.findGroup(group_name);
    if( ports_list.isNull() || ports_list.size() == 0 ) {
        ports.resize(0);
        return true;
    }
    ports.resize(ports_list.size());
    for(int port_id = 0; port_id < ports_list.size(); port_id++ ) {
        yarp::os::Bottle * port = ports_list.get(port_id).asList();
        if( port == NULL || port->size() != 3 ) {
            std::cout << "yarpWbi::loadSensorPortsFromConfig error: " << ports_list.toString() << " has an element malformed element" << std::endl;
            return false;
        }
        std::string bodyPart = port->get(0).asString().c_str();
        int id = port->get(1).asInt();
        std::string port_name = port->get(2).asString().c_str();
        int body_part_index = std::find(body_parts_vector.begin(), body_parts_vector.end(), bodyPart) - body_parts_vector.begin();
        if( body_part_index >= (int)body_parts_vector.size() || body_part_index < 0 ) {
            std::cout << "yarpWbi::loadSensorPortsFromConfig error: bodyPart in " << port->toString() << " not recognized." << std::endl;
            return false;
        }
        id_2_PortName id_port_map;
        id_port_map.id = wbi::LocalId(body_part_index,id);
        id_port_map.portName = port_name;
        ports[port_id] = id_port_map;
    }
    return true;
}

bool loadFTSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 const std::vector<std::string> & body_parts_vector,
                                 std::vector<id_2_PortName> &ft_ports)
{
    return loadSensorPortsFromConfig(wbi_yarp_properties,body_parts_vector,ft_ports,"WBI_YARP_FT_PORTS");
}

bool loadIMUSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                      const std::vector<std::string> & body_parts_vector,
                                      std::vector<id_2_PortName> &imu_ports)
{
    return loadSensorPortsFromConfig(wbi_yarp_properties,
                                     body_parts_vector,
                                     imu_ports,
                                     "WBI_YARP_IMU_PORTS");
}
*/

bool loadTreeSerializationFromConfig(yarp::os::Property & wbi_yarp_properties,
                                     KDL::Tree& tree,
                                     KDL::CoDyCo::TreeSerialization& serialization)
{
    std::string dofSerializationParamName = "idyntree_dof_serialization";
    std::string linkSerializationParamName = "idyntree_link_serialization";

    if( !wbi_yarp_properties.check(dofSerializationParamName) ) { return false; }
    if( !wbi_yarp_properties.check(linkSerializationParamName) ) { return false; }

    yarp::os::Bottle * dofs_bot = wbi_yarp_properties.find(dofSerializationParamName).asList();
    yarp::os::Bottle * links_bot = wbi_yarp_properties.find(linkSerializationParamName).asList();

    if( !dofs_bot || !links_bot ) { return false; }

    int serializationDOFs = dofs_bot->size()-1;
    int serializationLinks = links_bot->size()-1;

    std::vector<std::string> links;
    links.resize(serializationLinks);

    std::vector<std::string> dofs;
    dofs.resize(serializationDOFs);

    for(int link=0; link < serializationLinks; link++) {
        links[link] = links_bot[link+1].toString().c_str();
    }

    for(int dof=0; dof < serializationDOFs; dof++) {
        dofs[dof] = dofs_bot[dof+1].toString().c_str();
    }

    serialization = KDL::CoDyCo::TreeSerialization(tree,links,dofs);

    return serialization.is_consistent(tree);
}

bool loadTreePartitionFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 KDL::CoDyCo::TreePartition& partition)
{
    wbi_yarp_properties.find("idyntree_serialization");
    return false;
}


}
