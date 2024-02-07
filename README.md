# SLAM_by_fusing_multiple_IMU_sonsors_and_a_LIDAR_sensor_using_Extended_Kalman_Filter-ROS-
To fuse noisy IMU sensors (to replicate physical scenarios) for Simultaneous localization and mapping (SLAM)

# Introduction
This repository tries to replicate the physical environment using the Gazebo virtual simulator. It simulates the sensor fusion and slam on the turtlebot3. Althought the gazebo and the robot state publisher publishes the exact location of the robot (because it's present in the gazebo environment) on the /tf topic, but in real life scenaios, an pose esimate would be required in the form of /tf topic for the gmapping node to localize and simultaneously map the environment. Thus this repository mainly focuses on implementing Extended Kalman filter for sensor fusion.

## Details
#### Implementing Extended Kalman Filter to fuse multiple IMUs for pose estimation

The Urdf of the turtlebot3 is updated with two IMU's and both publishes on differenet /imu topic. For the prediction step of the EKF, the constant velocity motion model is used. Thus the input from the /cmd_vel topic is taken for EKF. The data from each of the /imu topic is fed to the extended kalman filter to estimate the pose (the measurement step). THe code implements the EKF according to the sensor update reate and pose is extimated at every received data. 

#### Used pose estimation and lidar data for SLAM
Then the estimated pose from the EKF is used to publish on the /tf topic to provide the predicted pose at each measurement and velocity update. The estimation is then used along with the LIDAR scan data by the gmapping node for SLAM. 

![rosgraph](https://user-images.githubusercontent.com/115849836/207501271-b5bf96fd-eda6-4baa-a696-8230c237142e.png)

rqt_graph


![Screenshot from 2022-12-05 01-48-53](https://user-images.githubusercontent.com/115849836/207501235-52e1fb46-137b-4e5c-bdd7-4a6102b5b1fc.png)

Compared the occupancy grid map with the floor map created in Gazebo.


# Usage
Clone the codes
```ruby
cd catkin_ws/
```
```ruby
cd src/
```
```ruby
git clone https://github.com/ROBOTIS-GIT/turtlebot3


```

Copy files turtlebot3_burger.urdf.xacro and turtlebot3_burger.gazebo.xacro from the repository and repplace with the file in the >>turtlebot3 >> turtlebot3_description>>urdf>>
```ruby
cd ..
```
Then paste the project.launch in any ROS package

```ruby
cd ..
```
Build the ROS packages
```ruby
catkin_make
```
```ruby
source /devel/setup.bash
```
```ruby
roslaunch -YOURROSPACKAGE- project.launch
```
Paste EKF.py to any folder
cd to that folder

```ruby
python3 EKF.py
```



PS: install slam_gmapping package before running the gmapping.launch
```ruby
git clone https://github.com/ros-perception/slam_gmapping.git
```
Then run catkin_make for the new package
