# FetchRobotTracker
## Fetch robot following a guider: Assesment 3 Mecatronic

The goal of our project is to control a Fetch robot, called tracker, to follow the path of the guider in front of him. We need to maintain a stable distance between the tracker robot and the guider robot. 

This project uses ROS environment, Ubuntu OS on Linux, as well as basic computer vision.

We also use several packages such as Gazebo, RViz, Fetch & Freight Robot package and TurtleBot3 package.


## How to install

### ROS setup:
The first thing to do is setup the environment ROS in version Melodic follow the tutorial to install ROS
Here is the link for installation on Ubuntu 18.04: <a href="http://wiki.ros.org/melodic/Installation/Ubuntu">ROS install</a>; this OS is the easiest to install ROS and is very stable.

### Create catkin_ws:
To set up the catkin workspace follow the tutorial: <a href="http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment">ROS tutorials</a>
Then follow all the tutorials until ``Navigating the ROS Filesystem`` 

### Install tracker_robot package:
In a terminal go to ``~/catkin_ws/src`` 
> cd ~/catkin_ws/src

Then download the package tracker_robot

> git clone https://github.com/alexandredorc/tracker_robot.git

Then go inside the package

> cd tracker_robot

And install all the requirements by running the ``setup.sh``

>sudo ./setup.sh 

it should install all the requirements say accept all the [Y/n], if this file is not working try doing all the command inside the file line by line.

After launching the nav.launch file if the turtlebot is not orange and the camera is not looking down thats means the 

## Launching mapping

This step is necessary if you change the environment model.

Do the following command

> roslaunch tracker_robot mapping.launch

Then when the mapping is done save the map by doing

> rosrun map_server map_saver -f maps/<map_name>

Then change fetch_map.yaml in all launch files into the name you choose

## Launching tracking 

To launch tracking just do the following command

> roslaunch tracker_robot nav.launch

Then wait until the fetch robot has move to choose the 2D Nav Goal in Rviz for the turtlebot to go to.

