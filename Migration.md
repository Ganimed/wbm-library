## yarpWholeBodyInterface 0.0.1 to 0.2.0 

The yarpWholeBodyInterface version 0.2.0 depends on wholeBodyInterface 0.2.0, that
itself introduced a lot of changes in the wholeBodyInterface. yarpWholeBodyInterface
specific changes are discussed in this file. 
In 0.2.0 we aim to provide a generic interface to yarp-powered robot, removing any
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
