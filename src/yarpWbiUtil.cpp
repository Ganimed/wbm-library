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
    for(int jnt=0; jnt < (int)jointIdList.size(); jnt++ )
    {
        wbi::wbiId jnt_name;
        jointIdList.numericIdToWbiId(jnt,jnt_name);

        if( !joints_config.check(jnt_name.toString().c_str()) )
        {
            std::cout << "[ERR] wholeBodyInterface error: joint " << jnt_name.toString() <<
                         "[ERR]  not found in WBI_YARP_JOINTS section of configuration file " << std::endl;
            return false;
        }

        yarp::os::Bottle * ctrlBoard_mapping = joints_config.find(jnt_name.toString().c_str()).asList();
        if( !ctrlBoard_mapping || ctrlBoard_mapping->size() != 2 )
        {
            std::cout << "wholeBodyInterface error: joint " << jnt_name.toString() <<
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

bool getControlBoardAxisList(yarp::os::Bottle & joints_config,
                                                          const wbi::wbiIdList &jointIdList,
                                                          const std::vector<std::string>& controlBoardNames,
                                                          std::vector< std::pair<int,int> > & controlBoardAxisList)
{
    std::map<std::string,int> controlBoardIds = getControlBoardIdsMap(controlBoardNames);

    assert(controlBoardIds.size() == controlBoardNames.size());

    //Error check loop
    for(int wbi_jnt=0; wbi_jnt < (int)jointIdList.size(); wbi_jnt++ )
    {
        wbi::wbiId wbi_jnt_name;
        jointIdList.numericIdToWbiId(wbi_jnt,wbi_jnt_name);

        std::cout << joints_config.toString() << std::endl;

        if( joints_config.find(wbi_jnt_name.toString().c_str()).isNull() )
        {
            std::cerr << "[ERR] yarpWbiUtil error: joint " << wbi_jnt_name.toString() << " not found in WBI_YARP_JOINTS section " << std::endl;
            return false;
        }

        yarp::os::Bottle * ctrlBoard_mapping = joints_config.find(wbi_jnt_name.toString().c_str()).asList();

        if( ctrlBoard_mapping->size() != 2 )
        {
            std::cerr << "[ERR] yarpWbiUtil error: joint " << wbi_jnt_name.toString() << " found in WBI_YARP_JOINTS section, but with wrong format " << std::endl;
            return false;
        }

        std::string controlboard_name = ctrlBoard_mapping->get(0).asString().c_str();
        if( controlBoardIds.find(controlboard_name) == controlBoardIds.end() )
        {
            std::cerr << "[ERR] yarpWbiUtil :: getControlBoardAxisList error in configuration files with joint " << wbi_jnt_name.toString() << std::endl;
            return false;
        }
    }

    controlBoardAxisList.resize(jointIdList.size());
    for(int wbi_jnt=0; wbi_jnt < (int)jointIdList.size(); wbi_jnt++ )
    {
        wbi::wbiId wbi_jnt_name;
        jointIdList.numericIdToWbiId(wbi_jnt,wbi_jnt_name);

        yarp::os::Bottle * ctrlBoard_mapping = joints_config.find(wbi_jnt_name.toString().c_str()).asList();

        std::string controlboard_name = ctrlBoard_mapping->get(0).asString().c_str();


        int controlboard_jnt_axis = ctrlBoard_mapping->get(1).asInt();
        int controlboard_wbi_id   = controlBoardIds[controlboard_name];

        //totalControlledJointsInControlBoard[controlboard_wbi_id]++;
        controlBoardAxisList[wbi_jnt] = std::pair<int,int>(controlboard_wbi_id,controlboard_jnt_axis);
    }
    return true;
}

std::vector< int > getControlBoardList(const std::vector< std::pair<int,int> > & controlBoardAxisList)
{
    std::vector< int > controlBoardList;
    for(int jnt=0; jnt < (int)controlBoardAxisList.size(); jnt++ )
    {
        int wbi_ctrlBoard_numericId = controlBoardAxisList[jnt].first;
        if( std::find(controlBoardList.begin(), controlBoardList.end(), wbi_ctrlBoard_numericId) == controlBoardList.end() )
        {
            controlBoardList.push_back(wbi_ctrlBoard_numericId);
        }
    }
    return controlBoardList;
}

std::map<std::string,int> getControlBoardIdsMap(const std::vector<std::string> & controlBoardNames)
{
    std::map<std::string,int> controlBoardIds;
    for(int ctrlBoard=0; ctrlBoard < (int)controlBoardNames.size(); ctrlBoard++)
    {
        controlBoardIds[controlBoardNames[ctrlBoard]] = ctrlBoard;
    }
    return controlBoardIds;
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
    getControlBoardAxisList(joints_config,jointIdList,controlBoardNames,controlBoardAxisList);


    //All elements in the jointIdList have a mapping to an axis of a controlboard
    //we have to save this mapping
    for(int wbi_jnt=0; wbi_jnt < (int)jointIdList.size(); wbi_jnt++ )
    {
        wbi::wbiId wbi_jnt_name;
        jointIdList.numericIdToWbiId(wbi_jnt,wbi_jnt_name);

        yarp::os::Bottle * ctrlBoard_mapping = joints_config.find(wbi_jnt_name.toString().c_str()).asList();

        std::string controlboard_name = ctrlBoard_mapping->get(0).asString().c_str();
        int controlboard_jnt_axis = ctrlBoard_mapping->get(1).asInt();
        int controlboard_wbi_id   = controlBoardIds[controlboard_name];

        //totalControlledJointsInControlBoard[controlboard_wbi_id]++;
        controlBoardAxisList[wbi_jnt] = std::pair<int,int>(controlboard_wbi_id,controlboard_jnt_axis);
    }

    return true;
}

bool loadSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                               wbi::wbiIdList & sensorIdList,
                               std::vector<std::string> & ports,
                               const std::string group_name)
{
    yarp::os::Bottle ports_list = wbi_yarp_properties.findGroup(group_name);
    if( ports_list.isNull() || ports_list.size() == 0 ) {
        ports.resize(0);
        return true;
    }

    sensorIdList = wbi::wbiIdList();
    ports.resize(0);

    ports.resize(ports_list.size()-1);
    for(int port_id = 0; port_id < ports_list.size()-1; port_id++ ) {
        yarp::os::Bottle * port = ports_list.get(port_id+1).asList();
        if( port == NULL || port->size() != 2 ) {
            std::cout << "yarpWbi::loadSensorPortsFromConfig error: " << ports_list.toString() << " has a malformed element" << std::endl;
            return false;
        }
        std::string sensorId = port->get(0).asString().c_str();
        sensorIdList.addId(wbi::wbiId(sensorId));
        std::string port_name = port->get(1).asString().c_str();
        ports[port_id] = port_name;
    }
    return true;
}

bool loadFTSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 wbi::wbiIdList & sensorIdList,
                                 std::vector<std::string> & ports)
{
    return loadSensorPortsFromConfig(wbi_yarp_properties,
                                     sensorIdList,
                                     ports,
                                     "WBI_YARP_FT_PORTS");
}

bool loadIMUSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties,
                                 wbi::wbiIdList & sensorIdList,
                                 std::vector<std::string> & ports)
{
    return loadSensorPortsFromConfig(wbi_yarp_properties,
                                     sensorIdList,
                                     ports,
                                     "WBI_YARP_IMU_PORTS");
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

    for(int link=0; link < serializationLinks; link++)
    {
        links[link] = links_bot[link+1].toString().c_str();
    }

    for(int dof=0; dof < serializationDOFs; dof++)
    {
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

bool loadIdListsFromConfigRecursiveHelper(std::string & requested_list,
                                          std::vector<std::string> & lists_names_stack,
                                          std::vector<std::string> & id_list_elements,
                                          yarp::os::Bottle & list_bots)
{
    yarp::os::Bottle * requested_list_bot = list_bots.find(requested_list).asList();
    //std::cout << "[INFO] Requested list bot: " << requested_list_bot->toString() << std::endl;
    if( requested_list_bot == NULL )
    {
        std::cerr << "[ERR] loadIdListFromConfig error: requested list " << requested_list << " not found in configuration file " << std::endl;
        return false;
    }
    //This is needed to check for a circular inclusion in the lists
    if (std::find(lists_names_stack.begin(), lists_names_stack.end(), requested_list) != lists_names_stack.end())
    {
        // List already include once, error
        std::cerr << "[ERR] loadIdListFromConfig error: requested list " << requested_list << " is duplicated " 
                  << "[ERR] inside of parent list " << lists_names_stack[0] << std::endl;
        return false;
    }

    lists_names_stack.push_back(requested_list);

    for( int el = 0; el < requested_list_bot->size(); el++ )
    {
        std::string id_to_add = requested_list_bot->get(el).asString();
        if( !(list_bots.find(id_to_add).isNull()) )
        {
            //Composite list, expand the element
            bool ok = loadIdListsFromConfigRecursiveHelper(id_to_add,lists_names_stack,id_list_elements,list_bots);

            if( !ok )
            {
                return false;
            }
        }
        else
        {
            id_list_elements.push_back(id_to_add);
        }
    }


    return true;
}

bool loadIdListFromConfig(std::string requested_list,
                          yarp::os::Property & wbi_yarp_properties,
                          wbi::wbiIdList & requestedIdList)
{
    yarp::os::Bottle & list_bot = wbi_yarp_properties.findGroup("WBI_ID_LISTS");
    yarp::os::Value & requested_list_val = list_bot.find(requested_list.c_str());
    yarp::os::Bottle * requested_list_bot = requested_list_val.asList();

    if( requested_list_val.isNull() || (requested_list_bot == NULL) )
    {
        std::cerr << "[ERR] loadIdListFromConfig error: requested list " << requested_list << " not found" << std::endl;
        return false;
    }

    std::vector<std::string> ids;
    std::vector<std::string> lists_names_stack;

    bool ret = loadIdListsFromConfigRecursiveHelper(requested_list,lists_names_stack,ids,list_bot);

    if( !ret )
    {
        std::cerr << "[ERR] loadIdListFromConfig error: requested list " << requested_list << " is malformed" << std::endl;
        return false;
    }

    requestedIdList = wbi::wbiIdList();

    for(int id = 0; id < (int)ids.size(); id++ )
    {
        requestedIdList.addId(wbi::wbiId(ids[id]));
    }

    return ret;
}



}
