# Client tutorial

## Colcon

- A more powerful tool than ```catkin_make```
- Directory
    - ```src```
    - ```build```
    - ```install```
    - ```log```
        - Directory for log information when compiling
```
Our workspace, ros2_ws, will be an overlay on top of the existing ROS 2 installation. In general, it is recommended to use an overlay when you plan to iterate on a small number of packages, rather than putting all of your packages into the same workspace.
```
- Build workspace
```bash
colcon build --symlink-install
```

## Create package

