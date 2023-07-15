# Topic

## Simple publisher

- Publisher is inherient from ```rclcpp::Node```
```cpp
class MinimalPublisher : public rclcpp::Node {

}
```

- Node setup
```cpp
Node("minimal_publisher")
Node("node name")
```

- Create a publisher with class
```cpp
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
```

- Timer Callback
```cpp
timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
```

- Add dependencies
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

- Add executable
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME})
```

## Log

- ```ROS_INFO```
```cpp
RCLCPP_INFO(this->get_logger(), "something %d", var);
```