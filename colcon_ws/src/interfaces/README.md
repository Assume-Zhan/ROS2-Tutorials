# Custom interfaces

## Custom Messages

- Create message folder
```bash
# In ws/src/package/
mkdir msg
```

- Custom message
```msg
geometry_msgs/Point center
float64 radius
```

- Specified in ```CMakeList.txt```
```txt
# Message dependencies
find_package(geometry_msgs REQUIRED)

# Message library generator
find_package(rosidl_default_generators REQUIRED)
```
```txt
# Add message file
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Test.msg"
    DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

- Add in ```package.xml```
```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

- Test add include folder
```cpp
#include "<package name>/msg/<message name in Lower case>.hpp"
```


## Custom service

- Create service folder
```bash
# In ws/src/package
mkdir srv
```

- Custom service
```srv
# Request
float64 x
---
bool equal
```

- Specified in ```CMakeList.txt```
```txt
# Message library generator
find_package(rosidl_default_generators REQUIRED)
```
```txt
# Add message file
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Test.srv"
    DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

- Test add include folder
```cpp
#include "<package name>/srv/<service name in Lower case>.hpp"
```


- ```CMakeList.txt```
```cpp
set(msg_files
  "msg/Message1.msg"
  "msg/Message2.msg"
)

set(srv_files
  "srv/Service1.srv"
  "srv/Service2.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
```