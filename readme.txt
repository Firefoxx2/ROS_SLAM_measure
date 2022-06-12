1ST STEP GENERATE BAGFILE
rosrun:
 - run gazebo
    $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
    $ roslaunch utbm_empty_world utbm_empty_world.launch
 - run bringup
    $ roslaunch turtlebot3_bringup turtlebot3_remote.launch
 - run teleop node
    $ rosrun r1_teleop teleop.py
	$ rosrun obstacle-avoidance-turtlebot naive_obs_avoid_tb3.py

Save bagfile
 - launch file :
   $ rosbag record -O myRecord -a



2ND STEP RECORD DATA
 - close all
 - run roscore
 - run measure node 
   $ rosrun r1_slam_measure measure.py _/measure/gazebo_robot_id:=turtlebot3 <- replace by you robot name
 - run SLAM node
   $ rosrun SLAM_PKG SLAM_node.py
 - record position
   $ rostopic echo /measure > myFile.csv
     -> stop when rosbag end
 - run rosbag
   $ rosbag play myRecord.bag


3RD STEP extract DATA
 - open myFile.csv with LibreOffice Calc
 - delete "---", '"', and "data: "
 - do what you want white those datas


1st collumn : estimated location x
2nd collumn : estimated location y
3rd collumn : true location x
4th collumn : true location y

By default, each line correspond to 100ms