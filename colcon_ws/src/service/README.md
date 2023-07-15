# Service

- Create a package
    - A package with dependencies : ```rclcpp``` and ```example_interfaces```
```cpp
ros2 pkg create --build-type ament_cmake service --dependencies rclcpp example_interfaces
```

