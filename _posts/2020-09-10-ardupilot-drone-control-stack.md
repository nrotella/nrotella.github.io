---
layout: post
title: "The Ardupilot Drone Control Stack"
author: "Nick Rotella"
categories: journal
tags: [drones,control,open-source]
image: ardupilot.jpg
---

A while back, I made a few posts about the [PX4](https://px4.io/) drone flight control stack and how to develop controllers within PX4's Gazebo simulator. However, another widely-used flight control stack is [ArduPilot](http://ardupilot.org/) - which has actually been used in various forms on multirotor UAVs (drones), fixed-wing UAVS, AUVs (autonomous underwater vehicles), AMRs (autonomous mobile robots), and beyond. [ArduCopter](https://ardupilot.org/copter/) or **Copter** for short is one of the most popular variants of Ardupilot, seeing hobbyist, academic, and professional use.

The goal of this post is to dive into ArduCopter as we did for PX4 - specifically, how to set up SITL (simulation in the loop) using Gazebo and ROS. The cool thing is that both PX4 and ArduCopter use [MAVLink](https://mavlink.io/en/) as a shared messaging protocol, meaning that **it's largely possible to write high-level code which is entirely agnostic of your choice of flight controller!** Further, as we saw in the PX4 tutorial, we can use [MAVROS](http://wiki.ros.org/mavros) - the ROS package which wraps MAVLink communication - to abstract away the flight controller even more. Let's take a look at how it all fits together. 

# Background material

There are a number of great guides which I've drawn material from, namely [this guide from the Air Force Research Laboratory](https://github.com/AS4SR/general_info/wiki/ArduPilot:-Instructions-to-set-up-and-run-an-autopilot-using-SITL-and-Gazebo-simulator#Summer_of_Innovation_2017_AFRL) but [this Ardupilot wiki thread](https://github.com/ArduPilot/ardupilot_wiki/issues/1001) and [this open-source guide](https://github.com/swiftgust/ardupilot_gazebo) are also useful resources. Another useful setup guide which also has useful information about Gazebo vs FCU frame definitions is [here](https://magiccvs.byu.edu/gitlab/lab/ardupilot_sim/blob/master/README.md).

Note that some of these instructions may be a bit out of date since this material was originally written in late 2019; this work was done on an Ubuntu 16.04 installation with ROS Kinetic. I haven't gone back through the setup lately, but I expect the general process should be similar for newer versions of these packages.

# Setting up Gazebo and the Ardupilot plugin

We follow the instructions from the [Ardupilot setup guide](http://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html) which explains how to build the plugin and get SITL working; we'll summarize some of the pre-build steps and the build itself below.

## Prerequisites (script install)

Since the prerequisites of Gazebo and ROS packages are the same as those we detailed in the PX4 setup blog post, we can actually use the nice [ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh) install script from the [PX4 install guide](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html) to handle these **assuming we're targeting Ubuntu 18.04/ROS Melodic**.

If you prefer to install Gazebo

## Prerequisites (Manual Install)

### Gazebo

Of course, you'll need to have the [Gazebo simulator](http://gazebosim.org/) installed; this comes with full installations of ROS and was also installed and configured during the PX4 SITL setup guide, so we won't go into much detail here.

Note that as of this writing (originally started in late 2019) You must have **Gazebo version>=8.0** installed; check with ```gazebo --version```.  To upgrade eg to 9.0 following the [official guide](http://gazebosim.org/tutorials?tut=install_ubuntu), first add sources:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

Then uninstall old gazebo versions and install the new ones:

```bash
sudo apt-get remove ros-kinetic-gazebo*
sudo apt-get install ros-kinetic-gazebo9-* libgazebo9-dev # change to anything gazebo8 or higher if desired
```

After installation, check ```gazebo --version``` again to confirm.  Without installing ```libgazebo9-dev```, you will face missing Cmake configfile errors.

### ROS Packages

We won't go into detail on installing ROS here - see the [melodic install guide](http://wiki.ros.org/melodic/Installation/Ubuntu) for details. When this is complete, you'll want to set up a catkin workspace and clone [MAVROS](http://wiki.ros.org/mavros) there, then build.

### Boost

You must also have **boost version > 1.58** installed (again, as of this writing).  Unfortunately, version 1.58 is the default for Ubuntu 16.04, so users of this distro must upgrade boost.  Following [this guide](https://devwiki.ainstein.ai/boost) to upgrade to boost 1.66, first purge the old boost:

```bash
sudo apt-get --purge remove libboost-dev libboost-doc
sudo apt-get --purge remove libboost-dev
sudo apt-get --purge remove libboost-all-dev
```

Then download and install the new version (the install step will take at a few least minutes):

```bash
cd /tmp # or anywhere else
wget https://dl.bintray.com/boostorg/release/1.66.0/source/boost_1_66_0.tar.gz
tar xzvf boost_1_66_0.tar.gz
cd boost_1_66_0
./bootstrap.sh
sudo ./b2 install # takes some time
```

After this finished, run ```cat /usr/local/include/boost/version.hpp``` and check that you see the line 

```
#define BOOST_VERSION 106600
```

which corresponds to version 1.66, as expected.

## Installation

Once the prerequisites above are installed, clone and build the Ardupilot Gazebo plugin as follows:

```bash
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

This last step will install ardupilot-related models in ```/usr/share/gazebo-X/models``` (X is the Gazebo version) and install the plugin itself in ```/usr/lib/x86_64-linux-gnu/gazebo-X/plugins```.  These directories must be added to the appropriate Gazebo paths by adding the following lines to the end of your ```~/.bashrc```:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-9/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugin
```

## Testing

To test the Ardupilot Gazebo plugin, simply launch Gazebo with the sample Ardupilot world specified:

```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```

The world file ```/usr/share/gazebo-9/worlds/iris_arducopter_runway.world``` specifies loading a runway world with an iris model.  If either wasn't loaded properly (check terminal output) then double-check the model and plugin paths after installation.

# Building and Running ArduCopter for SITL

![arducopter.jpg](../assets/img/arducopter.jpg "ArduCopter logo"){: .center-image}

With Gazebo set up and the Ardupilot plugin installed, we're ready to install the actual Ardupilot library and configure it for simulation in-the-loop testing on the type of vehicle we're working with (Copter) as follows:

```bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
./waf configure --board sitl
./waf copter
```

This builds the target ```build/sitl/bin/arducopter``` for *simulation* use rather than to be run on an embedded flight controller (we'll discuss how to run ArduCopter on actual hardware later). The SITL launch script is ```python Tools/autotest/sim_vehicle.py``` which requires installing some prerequisites before running:

```bash
sudo apt-get install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
sudo apt-get install python-scipy python-opencv ccache gawk git python-pip python-pexpect
sudo pip2 install future pymavlink MAVProxy
```

## Running SITL with Gazebo

We now have Ardupilot set up for Copter flight control in simulation, and the Gazebo plugin installed which provides the bridge between the simulated flight controller and the simulation itself.

First, add the path to the python script for SITL by appending the following to the end of your```~/.bashrc```:

```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
```

and re-```source ~/.bashrc```. Now we first run Gazebo with, as before:

```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```

Now, run the flight controller:

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

from anywhere to run the Copter simulator (note that if you were to instead run ```sim_vehicle.py -w``` from the ```ardupilot/ArduCopter``` directories then it would load Copter by default based on your working directory). You should now see some printout in the terminal as the flight controller starts up! This leaves you at the SITL command prompt *MAXProxy* where you can issue commands; this is like a simulated version of MAVLink. Let's try some basic commands to get started.

## Initial Testing

Now that we have the simulation running and connected to the flight controller running in terminal, issue the following commands with some pause between each in MAVProxy (the SITL command prompt):

```bash
mode GUIDED
arm throttle
takeoff 2
```

This will arm the copter and cause it to take off to a height of two meters.  **If the GPS does not have a 3D lock, arming will fail** with the pre-arm check ```FCU: PreArm: Need 3D Fix```.  This will occur each time arming is attempted until the following is printed, confirming the EKF has initialized with GPS:

```
[ INFO] [1557413831.777209537]: FCU: EKF2 IMU0 is using GPS
[ INFO] [1557413831.778028038]: FCU: EKF2 IMU1 is using GPS
```

The simulation takes some time to initialize the GPS and IMU-based Extended Kalman Filter (EKF) positioning system, so be sure to wait until the Flight Control Unit (FCU) reports that the EKF2 is up and running with simulated GPS as above.

# Implementing High-Level Flight Control with ROS

In the [previous tutorial on PX4 + Gazebo + ROS](https://nrotella.github.io/journal/px4-drone-control-stack.html), we used the [MAVROS](http://wiki.ros.org/mavros) packages which is essentially a RPS wwrapper for MAVLink, the standardized protocol for communicating with a FCU (flight control unit). The awesome thing about MAVLInk - and MAVROS by extension - is that **high-level flight controllers we write which use this standardized protocol are agnostic of our choice of low-level flight controller.** In other words, the flight controller we wrote using MAVROS in the PX4 tutorial should work with minimal modifications when running on to op ArduCopter instead of PX4!

Of course, flight control modes themselves and their implementations will differ, and so performance won't be one-to-one. In terms of mechanics, though, we should be able to run what we've already written for PX4 in our new Gazebo + Ardupilot SITL setup.

## Prerequisites

The only package we're missing at this point (unless you already happen to have it installed on your system from another tutorial) is MAVROS, which can be installed via ```apt``` for your system's ROS distribution using

```bash
sudo apt install ros-YOURDISTRO-mavros
```

See the [ROS package usage](http://wiki.ros.org/mavros#Usage) for more details on how to use MAVROS with your FCU.

## Example: Tracking a Circular Trajectory

The high-level control node we wrote in the PX4 tutorial simply tracks a circle in the X-Y plane at a fixed height (Z position). The following code implements this controller using MAVROS to bridge between ROS and ArduCopter via MAVLink; note that the only difference from the PX4 example is that **the flight mode for offboard control is GUIDED in ArduCopter, rather than OFFBOARD for PX4.**

```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node_feedback");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state", 10, state_callback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_position/local", 10);
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
    ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ROS_INFO("Initializing...");
  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Connected.");

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;
  
  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "GUIDED";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  ros::Time time_start = ros::Time::now();  
  while(ros::ok()){
    if( current_state.mode != "GUIDED" &&
	(ros::Time::now() - last_request > ros::Duration(5.0))){
      if( set_mode_client.call(offb_set_mode) &&
	  offb_set_mode.response.mode_sent){
	ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed &&
	  (ros::Time::now() - last_request > ros::Duration(5.0))){
	if( arming_client.call(arm_cmd) &&
	    arm_cmd.response.success){
	  ROS_INFO("Vehicle armed");
	}
	last_request = ros::Time::now();
      }
    }
    
    // Update the desired pose:
    pose.pose.position.x = sin(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
    pose.pose.position.y = cos(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());

    //Update the desired velocity:
    vel.linear.x = 2.0*M_PI*0.1*cos(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
    vel.linear.y = -2.0*M_PI*0.1*sin(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
    
    local_pos_pub.publish(pose);
    local_vel_pub.publish(vel);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
```

As for PX4, we use ROS services to arm the motors (```"mavros/cmd/arming"```) and set the flight mode (```"mavros/set_mode"```), a ROS subscriber to get the current state of the drone from for simulated FCU running an EKF (```"mavros/state"```), and ROS publishers to set the desired *local*-frame pose (```"mavros/setpoint_position/local"```) and twist (```"mavros/setpoint_velocity/cmd_vel_unstamped"```).

We can wrap the above code into a ROS node called for example ```offboard_control_circle_node``` in a ```drone_control``` package, and run it with MAVROS connecting to the simulated flight controller running ArduCopter and Gazebo (as described in previous sections) with:

```xml
<launch>
  <!-- Run the offboard position controller -->
  <node name="pos_controller" pkg="drone_control" type="offboard_control_circle_node" />

  <!-- Run MAVROS -->
  <include file="$(find mavros)/launch/apm.launch" pass_all_args="true">
    <arg name="fcu_url" value="udp://127.0.0.1:14551@14555"/>
  </include>
</launch>
```

The offboard controller should load, receive all parameters and start the mission.

### Troubleshooting

If the simulated drone just sits on the ground and arms/disarms repeatedly forever, with output in the terminal like:

```bash
[ INFO] [1556567506.929185431]: FCU: Arming motors
[ INFO] [1556567517.919116373]: FCU: Disarming motors
[ INFO] [1556567518.976069745]: FCU: Arming motors
[ INFO] [1556567528.985784804]: FCU: Disarming motors
[ INFO] [1556567529.077693447]: FCU: Arming motors
```

then the problem, according to [this issue](https://discuss.ardupilot.org/t/ros-setpoint-command-available/15503/8), is that a [takeoff command (service)](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.Services) must be commanded before sending setpoints in a GUIDED mission.  While the [example PX4 offboard controller](https://dev.px4.io/en/ros/mavros_offboard.html) did not work because of this issue, this [example](https://github.com/Texas-Aerial-Robotics/Controls-ROS/blob/master/src/staple.cpp) worked out of the box for ArduPilot.

# Wrapping Up

This post was mostly taken from some of my old notes on using ArduCopter for SITL flight control - if something doesn't work as of this writing, please feel free to comment below! Next post, I'll share some of my notes about details of the low-level flight controllers. After that, we'll introduce how to set up an airframe with a flight control unit, offboard computer (either Raspberry Pi or Nvidia TX2), and run our example task outdoors on real hardware.