![YDLIDAR](sdk/image/TSA.png  "TSA")

# YDLIDAR ROS PACKAGE(V1.0.0)

## Dataset 
|LIDAR      | Model  |  Baudrate |  SampleRate(K) | Range(m)  		   |  Frequency(HZ) | Intenstiy(bit) | SingleChannel | voltage(V)|
| :-------- |:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| TSA    　 | 130	   |  256000   |   3.121        |  0.05~12      	   | 5~8       | true          | false      	  | 4.8~5.2   |



## How to [install ROS](http://wiki.ros.org/ROS/Installation)

[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

[windows](http://wiki.ros.org/Installation/Windows)

## How to Create a ROS workspace
[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

## How to build YDLiDAR ROS Package

    1) Clone this project to your catkin's workspace src folder
    	
    2) Running catkin_make to build ydlidar_ros_driver_node
    3) Create the name "/dev/ydlidar" for YDLIDAR
    --$ roscd ydlidar_ros/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh

## How to Run YDLIDAR ROS Package
#### 1. Run YDLIDAR node and view in the rviz
------------------------------------------------------------
	roslaunch ydlidar_ros_driver lidar_view.launch

#### 2. Run YDLIDAR node and view using test application
------------------------------------------------------------
	roslaunch ydlidar_ros_driver lidar.launch

## ros-interfaces

<center>

| Topic                | Type                    | Description                                      |
|----------------------|-------------------------|--------------------------------------------------|
| `scan`               | sensor_msgs/LaserScan   | 2D laser scan of the 0-angle ring                |

| Parameter         | Type                    | Description                                         |
|-----------------------|------------------------|-----------------------------------------------------|
| `port`        		| String                 	| port of lidar (ex. /dev/ydlidar)                         		|
| `baudrate`     	| int                      	| baudrate of lidar (ex. 256000)           				|
| `frame_id`      	| String                	| TF frame of sensor, default: `laser_frame`    		|
| `resolution_fixed` | bool                     	| Fixed angluar resolution, default: false                    	|
| `auto_reconnect` | bool                  	| Automatically reconnect the LiDAR, default: true    	|
| `reversion`     	| bool                  	| Reversion LiDAR, default: false  					|
| `angle_min`       	| float                 	| Minimum Valid Angle, defalut: -180.0     			|
| `angle_max`       	| float                  	| Maximum Valid Angle, defalut: 180.0      			|
| `range_min`       	| float                  	| Minimum Valid range, defalut: 0.01m      			|
| `range_max`       	| float                  	| Maximum Valid range, defalut: 12.0m      			|
| `ignore_array`      | String                  	| LiDAR filtering angle area, default: ""      			|
| `frequency`       	| float                  	| scan frequency of lidar,default: 6.0      			|



</center>

## Contact EAI
![Development Path](sdk/image/EAI.png)

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)