## yarpWholeBodyInterface 0.1 to 0.2

The yarpWholeBodyInterface version 0.2 depends on wholeBodyInterface 0.2, that
itself introduced a lot of changes in the wholeBodyInterface. yarpWholeBodyInterface
specific changes are discussed in this file.
In 0.2 we aim to provide a generic interface to yarp-powered robot, removing any
iCub specific software, most of the changes are related to that.

### Namespace change
The namespace for all the project changed from `wbiIcub` to `yarpWbi`
#### Relevant code changes
```
using namespace wbiIcub;
```
becomes
```
using namespace yarpWbi;
```

### Headers names changes
The namespace for all the headers changed from `icub` related names to `yarp` related names
```
#include <wbiIcub/wholeBodyInterfaceIcub.h>
```
becomes
```
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
```

### WBI Configuration files
In version 0.2, no robot information is hardcoded in the `yarp-wholebodyinterface`.

All information is now gather from appropriate configuration files. An example configuration file
for iCubHeidelberg01 is available here: https://github.com/robotology/icub-main/blob/master/app/robots/iCubHeidelberg01/wholeBodyInterface.ini

The information in this configuration file is parsed to a `yarp::os::Property` object, and then passed
to the interface throught the constructor or using the `setYarpWbiProperties` methods. This information
is then used in the `init()` method, and it cannot be altered after the interface initialization.

Fof the modules in codyco-modules, the name of this configuration file is passed via a `wbi_conf_file`
configuration option (with the default set to `wholeBodyInterface.ini`). Similarly in the wbi configuration
file the name of the used urdf file is passed throught a `urdf_file` option.
The actual location of this two files is then obtained throught the Yarp `ResourceFinder`.

You can get more information about the `ResourceFinder` in the official Yarp documentation:
* [ResourceFinder Tutorials and Specification](http://wiki.icub.org/yarpdoc/yarp_resource_finder_tutorials.html).
Using the ResourceFinder we can exploit the `robots` directories, through which the user can use/install robot specific configuration files using the `YARP_ROBOT_NAME` enviromental variable.

#### Relevant code changes
```
 //--------------------------WHOLE BODY INTERFACE--------------------------
iCub::iDynTree::iCubTree_version_tag icub_version;
iCubVersionFromRf(rf,icub_version);
robotInterface = new icubWholeBodyInterface(moduleName.c_str(), robotName.c_str(),icub_version);
```
becomes (just an example, details can vary):
```
//--------------------------WHOLE BODY INTERFACE--------------------------
yarp::os::Property yarpWbiOptions;
//Get wbi options from the canonical file
if( !rf.check("wbi_conf_file") )
{
    fprintf(stderr,"[ERR] locomotionControl: impossible to open wholeBodyInterface: wbi_conf_file option missing");
}
std::string wbiConfFile = rf.findFile("wbi_conf_file");
yarpWbiOptions.fromConfigFile(wbiConfFile);
//Overwrite the robot parameter that could be present in wbi_conf_file
yarpWbiOptions.put("robot",robotName);
robotInterface = new yarpWholeBodyInterface(moduleName.c_str(), yarpWbiOptions);
```

### ID Lists
In version 0.1, the list of IDs where hard-coded at compile time, for example in the `ICUB_MAIN_JOINTS`
constant list. In version 0.2, list of IDs are loaded at configuration time from configuration files.

Again, for an example of configuration file check:
https://github.com/robotology/icub-main/blob/d4b6c70076876bc6ce2947b1a1969101328b1af5/app/robots/iCubHeidelberg01/wholeBodyInterface.ini

We started using "pseudo" standard names for joint ID lists, with some semantical meaning attached.
For now we are using: 
  * `ROBOT_TORQUE_CONTROL_JOINTS` : the set of all joints in the robot for which torque control is enabled.
  * `ROBOT_DYNAMIC_MODEL_JOINTS`  : the set of joints typicaly used for the dynamical model of the robot (i.e. typically things like the fingers are not included in this list). 

#### Relevant code changes
```
robotInterface->addJoints(ICUB_MAIN_JOINTS);
```
becomes (just an example, details can vary):
```
IDList RobotMainJoints;
std::string RobotMainJointsListName = "ROBOT_MAIN_JOINTS";
if( !loadIdListFromConfig(RobotMainJointsListName,yarpWbiOptions,RobotMainJoints) )
{
    fprintf(stderr, "[ERR] locomotionControl: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
}
robotInterface->addJoints(RobotMainJoints);
```

### Drop is*Simulator calls
`isICubSimulator` and `isRobotSimulator` have been dropped, being hardcoded function
that checked just the robotname. They can be substituted by alternative wbi configuration
files for simulator that lack some sensors, or eventually checks in the user code. 

### iCub naming changes 
Even if this is not something directly related to the yarp-wholebodyinterface, it 
is worth mentioning that some names in the iCub models changed, for more information
check https://github.com/robotology/icub-main/issues/57 . 
One of the most important changes is that the two links `l_shank` and `r_shank` are now
named `l_leg` and `r_leg`. 
