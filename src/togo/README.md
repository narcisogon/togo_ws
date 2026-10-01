# togo

## Table of Contents

- [togo](#togo)
  - [Table of Contents](#table-of-contents)
  - [Sensors](#sensors)
    - [OAK-D cameras](#oak-d-cameras)
    - [Fixposition GNSS](#fixposition-gnss)
    - [Seyond 3D lidar](#seyond-3d-lidar)
    - [Phidgets spatial](#phidgets-spatial)
  - [External packages to pull in source code from](#external-packages-to-pull-in-source-code-from)
  - [Deploy](#deploy)
    - [Deploy Testing](#deploy-testing)
      - [Stopping Clearpath](#stopping-clearpath)
      - [Starting Select Clearpath Services](#starting-select-clearpath-services)
      - [Restart Clearpath](#restart-clearpath)
  - [Gazebo](#gazebo)

## Sensors

### OAK-D cameras

These run from apt packages included from ros-jazzy-depthai-ros.
These end up launching the driver itself for the camera information, as well as a node that converts the RGBD data into point clouds.
Both of these nodes are launched inside a composable node container.
I think the source code should be [here](https://github.com/luxonis/depthai-ros/tree/jazzy).

### Fixposition GNSS

These run on this open source driver package, [fixposition_driver](https://github.com/fixposition/fixposition_driver) package.
The documentation for the driver exists [here](https://docs.fixposition.com/fd/fixposition-ros-driver)

### Seyond 3D lidar

These run on this open source driver package, [seyond_ros_driver](https://github.com/Seyond-Inc/seyond_ros_driver).

### Phidgets spatial

I think this is based on the apt package ros-jazzy-phidgets-spatial.
Clearpath typically launches the generic driver node along with an imu filter node as well.
These also get launched as a part of a composable node container.
I think the source code should be [here](https://github.com/ros-drivers/phidgets_drivers/tree/jazzy).

## External packages to pull in source code from

fixposition_driver

- The driver is located [here](https://github.com/fixposition/fixposition_driver).
  - Make sure you are cloning recursively, as there are submodules.
- There is an extra step where you should run the script `setup_ros_ws.sh` from the fixposition_driver directory in your directory. Instructions are [here](<https://docs.fixposition.com/fd/installation-and-usage#Installationandusage-a)SetupdriverforanexistingROSworkspace>). It seems like that might just add some colcon ignores on things you don't need depending on your ros version.

seyond_ros_driver

- The driver is located [here](https://github.com/Seyond-Inc/seyond_ros_driver).
  - Make sure you are cloning recursively, as there are submodules.
- There are instructions for building the drivers inside the workspace [here](https://github.com/Seyond-Inc/seyond_ros_driver/blob/main/src/seyond_lidar_ros/README.md#compile).

## Deploy

To deploy Togo hardware:

1. Start Husky hardware communications:

    ```bash
    ros2 launch togo_deploy husky_comm.launch.py
    ```

2. To bring up Togo's controllers and teleop control (enabling control through the PS4 controller),
we include a few convenient launch files for Togo's different operation modes.
   1. For Togo hardware:

        ```bash
        ros2 launch togo_deploy control_hardware.launch.py
        ```

        This launch file is equivalent to launching controls and teleop separately:

        ```bash
        # controllers
        ros2 launch togo_deploy control.launch.py
        # teleop
        ros2 launch togo_deploy teleop.launch.py
        ```

3. Start Togo's sensors (and related nodes, including the IMU filter and localization) and view the robot and sensor data in RViz:

    ```bash
    ros2 launch togo_deploy togo_sensors.launch.py
    ```

4. (Optional; ***BE READY ON THE E-STOP!***) To check that the controllers are communicating with the motor driver properly, you can publish a small velocity command from the command line:

    ```bash
    ros2 topic pub /platform_velocity_controller/cmd_vel geometry_msgs/msg/TwistStamped 'header:
    stamp: now
    frame_id: ''
    twist:
    linear:
        x: 0.05
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0
    '
    ```

### Deploy Testing

This info will eventually be wrapped up in the `systemd` processes on the Togo controls computer.
For now, a few helpful notes on manually starting/stopping Clearpath services:

#### Stopping Clearpath

Stop all Clearpath stuff:

- To stop all of the Clearpath processes:

    ```bash
    sudo systemctl stop clearpath-robot.service
    ```

- To disable all of the Clearpath processes and prevent them from automatically restarting when they die:

    ```bash
    sudo systemctl disable clearpath-robot.service
    ```

- Clearpath starts a lot of docker containers by default. We can view all of the running containers:

    ```bash
    docker container ps
    ```

    To stop all running Clearpath dockers:

    ```bash
    docker stop $(docker ps -q)
    ```

- As a sanity check, you can confirm everything has stopped:

    ```bash
    # Clearpath robot services
    systemctl status clearpath-robot.service
    # Clearpath dockers
    docker container ps
    ```

#### Starting Select Clearpath Services

Start the background Clearpath services that we do actually need:

- ROS discovery service: copy the commands from `/etc/clearpath/discovery-server-start`:

    ```bash
    # source ROS
    source /opt/ros/jazzy/setup.bash
    # start ROS discovery service
    fastdds discovery -i 0 -p 11811
    ```

    This server will hang in the terminal.
- VCAN
  - Start the VCAN service:

    ```bash
    sudo systemctl start clearpath-vcan.service
    ```

  - Check the status of this process:

    ```bash
    systemctl status clearpath-vcan.service
    ```

#### Restart Clearpath

It's nice to restart Clearpath for now while we are still bringing up Togo.

```bash
# re-enable Clearpath robot services
sudo systemctl enable clearpath-robot.service
# re-start Clearpath robot services
sudo systemctl start clearpath-robot.service
```

## Gazebo

To bring up Togo in Gazebo, run:

```bash
ros2 launch togo_gz sim_gz.launch.py
```

This launch file will launch the controls appropriately from the `togo_deploy` package using the Gazebo URDF in the `togo_gz` package.
The Gazebo URDF instantiates the Togo macro in `togo_description` and adds the appropriate `ros2_control` plugins for Gazebo.
This launch file will also automatically launch RViz to view the simulated robot and sensor information.

Once Gazebo is running, you can publish velocity commands from the command line:

```bash
ros2 topic pub --once /platform_velocity_controller/cmd_vel geometry_msgs/msg/TwistStamped 'header:
  stamp: now
  frame_id: ''
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5
'
```

Publish a zero command to stop the robot:

```bash
ros2 topic pub --once /platform_velocity_controller/cmd_vel geometry_msgs/msg/TwistStamped 'header:
  stamp: now
  frame_id: ''
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
'
```
