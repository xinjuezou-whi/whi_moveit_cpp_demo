# whi_moveit_cpp_demo
tutorial source: https://ros-planning.github.io/moveit_tutorials/doc/moveit_cpp/moveitcpp_tutorial.html

- add yaml configure file to adapt both Panda arm and my own arm
- with plan under scene with collision object and attached object
- with computation on multiple waypoints of Cartesian path

## runding the demo
### panda arm
Open a shell, run the launch file:
```
roslaunch whi_moveit_cpp_demo moveit_cpp_demo.launch arm:=panda
```

### whi arm
Open a shell, run the launch file:
```
roslaunch whi_moveit_cpp_demo moveit_cpp_demo.launch
```
