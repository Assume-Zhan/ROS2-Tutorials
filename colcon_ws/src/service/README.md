# Service

- Create a package
    - A package with dependencies : ```rclcpp``` and ```example_interfaces```
```cpp
ros2 pkg create --build-type ament_cmake service --dependencies rclcpp example_interfaces
```

- Node initialization
```cpp
rclcpp::init(argc, argv);

std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node name");
```

- Create a service
```cpp
rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr
    service = node->create_service<example_interfaces::srv::AddTwoInts>("node name", &callback_function);
```

- Service callback
```cpp
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                    request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}
```

- Spin the node
```cpp
rclcpp::spin(node);
```

- Make
```cpp
add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

install(TARGETS server DESTINATION lib/${PROJECT_NAME})
```