# LAEA: A 2D LiDAR Assisted UAV Exploration Algorithm for Unknown Environment 

[**LAEA**]() is a 2D LiDAR Assisted UAV Exploration Algorithm based on the framework of FAEP. 

# CODE WILL COME SOON

Please kindly star ⭐ this project if it helps you. We take great efforts to develope and maintain it.

[<img src="https://markdown-videos-api.jorgenkh.no/youtube/_a1Vl518Ra8" style="zoom: 200%;" />](https://youtu.be/_a1Vl518Ra8)



## Dependencies

We use **default px4 gazebo simulation environments**, the below **dependencies** is needed. Among them, regarding the configuration of the px4 gazebo simulation environment, you need to refer to the information on the Internet. 

- ros & mavros: Support Ubuntu18.04 **Melodic**, Ubuntu20.04 **Noetic**. Please search for the detailed installation tutorial. The following installation steps are for reference only.

```bash
# you can use below to install ros-full quickly
# During this process, you need to make a selection from the terminal
wget http://fishros.com/install -O fishros && sudo bash fishros

########################## mavros ########################
# for Ubuntu18.04 Melodic
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y
# for Ubuntu20.04 Noetic
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras -y

wget https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh
sudo chmod a+x ./install_geographiclib_datasets.sh
# The next step takes a while to install, you can wait patiently
sudo ./install_geographiclib_datasets.sh
```

- PX4-Autopilot: the **drone model files and world files** required for the px4 simulation are located in the `px4_gazebo/resource`. You need to **copy the corresponding files to your own px4 environment**. 

```bash
reference: https://github.com/PX4/PX4-Autopilot
```

- others

```bash
sudo apt-get install libarmadillo-dev libdw-dev

# nlopt for non-linear optimization
# Source code installation is recommended, as follows
git clone https://github.com/stevengj/nlopt.git
cd nlopt/
mkdir build&&cd build/
cmake ..
make 
sudo make install
```



## Quick Start

Once the relevant environment has been configured (especially PX4), you can run simulation experiments using the provided code. The **drone model files and world files** required for the px4 simulation are located in the `px4_gazebo/resource`. You need to **copy the corresponding files to your own px4 environment**. 

```bash
# cd ~/your_workspace_dir/src
# mkdir -p ~/laea/src&&cd ~/laea/src
git clone https://github.com/Poaos/LAEA --recursive
cd ..
catkin_make

################ 1) Start your simulation environment ###############
# My simulation environment boot example
source ./devel/setup.bash
roslaunch px4_gazebo laea_gazebo_lidar.launch # drone&sensor data&sim-env

################ 2) Activate your drone controller ##################
# We're using the default [mavros_controllers]
# If you want to use the controller for real flight, a carefully adjustment for the parameters is needed, otherwise... 
source ./devel/setup.bash
roslaunch px4_gazebo controller.launch # controller

################ 3) Start the octomap mapping service ###############
source ./devel/setup.bash
roslaunch octomap_server scan_mapping.launch # octomap mapping

################ 4) Activate our exploration algorithm ##############
source ./devel/setup.bash
roslaunch exploration_manager explore_test.launch # exploration algorithm

################ 5) rviz for visualization ##########################
source ./devel/setup.bash
roslaunch exploration_manager rviz_alg.launch # rviz 

#####################################################################
The above startup steps are many, you can integrate them into a launch file, start separately for ease of understanding.
```

Of course, **any simulation environment is fine** as long as it **provides the following data required** by the algorithm. Specifically, you need to modify the following files:

- exploration_manager/launch/poaozz/**explore_test.launch**: Provides drone and camera position and depth image

```lua
<!-- topic of your odometry such as VIO or LIO -->
<arg name="odom_topic" value="/mavros/local_position/odom" />

<!-- sensor pose: transform of camera frame in the world frame -->
<arg name="sensor_pose_topic" value="/mavros/camera/pose"/>

<!-- depth topic: depth image, 640x480 by default -->
<arg name="depth_topic" value="/camera/depth/image_raw"/>
```

- utils/depthimage_to_laserscan/launch/depth2scan.launch: Provides depth camera data to generate lidar scans

```lua
<!-- change here for your camera depth topic name. -->
<remap from="image"       to="/camera/depth/image_raw"/> 
<remap from="camera_info" to="/camera/depth/camera_info"/> 
```

- utils/laserscan_to_pointcloud/launch/cloud.launch: Provides 2d lidar data

```lua
<!-- lidar scan data -->
<arg name="laser_scan_topics" default="/iris_0/scan" />
```



## Guidelines for Parameters

As described in paper, LAEA enables efficient exploration of unknown environments by **timely access to the detected special frontier regions**. Therefore, you may need to refer to the meaning of the parameters and adjust them for the current exploration environment. Parameter file is located at `exploration_manager/launch/poaozz/algorithm_.xml`. The main parameters are as follows: 

```lua
<!-- atsp cost params  -->
<param name="frontier/small_area_thresh" value="2.0" type="double"/>
<param name="frontier/small_thresh_low" value="0.6" type="double"/>
<param name="frontier/isolateArea_gain" value="15" type="double"/>
<!-- max angle gap -->
<param name="exploration/max_yaw_gap" value="100" type="int"/>
<!-- Path time constraints when filtering for middle yaw -->
<param name="exploration/lens_lb_rate" value="1.35" type="double"/>
<param name="exploration/yaw_rotate_rate" value="1.2" type="double"/>
```



## Acknowledgements

Our code is developed based on [**FAEP**](https://github.com/Zyhlibrary/FAEP). We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.



