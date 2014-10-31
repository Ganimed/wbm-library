## wholeBodyInterface 0.1.0 to 0.2.0 

In this version we changed a lot of the internal working of the wholeBodyInterface.

### wbi::LocalId --> wbi::wbiId
The main identifier for all elements (Sensors, Estimates, Joints, ..) is now the 
wbi::wbiId, that is a basic wrapper over the standard std::string . There are a lot
of changes related to this, more notably:
 * wbi::LocalIdList --> wbi::wbiIdList 
