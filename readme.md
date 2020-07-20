## WARNING 
**This repository is for informational purposes. To launch the scripts you have to modify the package filess**

Documentation has not been created because the trajectory test results with the T99A10 camera are negative. Therefore, this package will not be used

The following documentation was created to remember the commands to perform the tests. They may not work because the package was modified.



## 1. Using velocity control (sim)

Launch axis_cam simulation with position control using velocity (Gazebo and Rviz):

```
roslaunch axis_cam axis_cam_complete.launch
```

Set setpoint position using command:

```
rostopic pub /pan_joint_position_controller/command std_msgs/Float64 "data: 1.0"
```

View current position in a graph (sent by Gazebo)

Open rviz and select MatPlot plugin. Then select this topic to view pan velocity (velocity[0]). 
Rembember set Y axis plot from 0.5 to -0.5

```
/joint_states/velocity[0]
```

## 2. Using trayectory control: velocity points (sim)

Edit axis_cam_control.yaml and use trajectory control, same in axis_cam_complete
Launch axis_cam simulation with trajectory control using velocity (Gazebo and Rviz):

```
roslaunch axis_cam axis_cam_complete.launch
```

Set trayectory using set_trajectory launch file:

```
roslaunch axis_cam set_trajectory.launch
```

Now axis cam in gazebo turns right and come back.

## 3. Using trajectory generator: position setpoints (real)

If you don't want to use a trajectory control you can send trajectory points in open loop mode.
That means the points are sent at each interval regardless of whether the camera has reached that position.

Launch axis_camera in order to control pant tilt

```
roslaunch axis_camera axis.launch hostname:=192.168.0.90 enable_ptz:=true
```

Launch trajectory generator using position as setpoints

```
roslaunch axis_cam trajectory_generator.launch
```

Two windows will be open. The first is a trajectory graph. The second shows the real time commands and states of the
axis cam.
* axis/cmd: commands sent by trajectory_generator
* axis/state: current state of axis_cam


## 4. Using trjectory generator: velocity setpoints (real)

Edit axis_ptz.py. In line 290 set speed_control to True. Then, launch axis_camera

```
roslaunch axis_camera axis.launch hostname:=192.168.0.90 enable_ptz:=true
```

Launch trajectory generator using velocity as setpoints

```
roslaunch axis_cam trajectory_generator.launch use_velocity:=true
```
