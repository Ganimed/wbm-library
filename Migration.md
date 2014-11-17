## wholeBodyInterface 0.1.0 to 0.2.0 

In this version we changed a lot of the internal working of the wholeBodyInterface.

### wbi::LocalId --> wbi::ID
Relevant discussion:
The main identifier for all elements (Sensors, Estimates, Joints, ..) is now the 
wbi::ID, that is a basic wrapper over the standard std::string . There are a lot
of changes related to this, more notably the list type migrated from wbi::LocalIdList 
to wbi::IDList.

### All add* methods should be called before init()
Relevant discussion: https://github.com/robotology/codyco-modules/issues/18

Now all `add*` / `remove*` methods must be called 
before calling the init() function. Once the interface is initialized, the 
list of joints/sensors/estimates cannot be changed. 

### All the geometric iWholeBodyModel methods now accept frames id
Relevant discussion: https://github.com/robotology/codyco-modules/issues/39

Accordingly, a `virtual const wbiIdList& getFrameList()` method has been added
to the iWholeBodyModel. This is substituting the ugly `getLinkId` method.

#### Relevant code changes
```
int root_link_id;
bool ok = getLinkId("root_link",root_link_id);
```
becomes
```
int root_link_id;
bool ok = getFramesList().wbiIdToNumericId("root_link",root_link_id);
```

