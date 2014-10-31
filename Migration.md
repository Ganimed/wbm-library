## wholeBodyInterface 0.1.0 to 0.2.0 

In this version we changed a lot of the internal working of the wholeBodyInterface.

### wbi::LocalId --> wbi::wbiId
Relevant discussion:
The main identifier for all elements (Sensors, Estimates, Joints, ..) is now the 
wbi::wbiId, that is a basic wrapper over the standard std::string . There are a lot
of changes related to this, more notably:
 * wbi::LocalIdList --> wbi::wbiIdList 

### All add* methods should be called before init()
Relevant discussion: https://github.com/robotology/codyco-modules/issues/18
To simplify implementation, now all `add*` / `remove*` methods must be called 
before calling the init() function. Once the interface is initialized, the 
list of joints/sensors/estimates cannot be changed. 
