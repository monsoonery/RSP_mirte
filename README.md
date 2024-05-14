# Autonomous driving with Mirte
A ROS project for controlling the mobile robot Mirte on a simulated test track.

Authored by Mees Chammat (y.chammat@student.tudelft.nl)\
October 26th, 2023\
RO47003 Robot Software Practicals - Final Assignment


## About the project
For the final assignment, the task was to use Mirte's camera and sensors to detect pedestrians and obstacles on a virtual test track, in order to generate control commands that enable her to autonomously drive around a test track outlined by cones without hitting any obstacles or pedestrians. To this end, I used ROS to create two packages:

**Package 1: `opencv_person_detector`**
- Contains a node named `opencv_person_detector_node` that subscribes to the `mirte/image_raw` topic and uses the OpenCV HOG detector to detect pedestrians in these images. It then publishes the detections as a Detection2DArray on the `opencv_person_detector_node/detections` topic. In addition, the node also draws bounding boxes (green rectangles) around the pedestrians on the images, and publishes these drawn-on images to the `opencv_person_detector_node/visual` topic. 

**Package 2: `control_barrel_world`**
- Contains a node named `control_barrel_world_node` that subscribes to a) the `pcl_obstacle_detector_node/detections` topic for 3D obstacle detections, and b) the `opencv_person_detector_node/detections` topic for 2D person detections (originating from the `opencv_person_detector_node` described above). Based on the data published onto these topics, a simple control algorithm decides what Mirte should do: drive straight ahead if the road is clear, steer if there are obstacles ahead, or stop if a person is detected close by. This control command is published on the `mirte/mobile_base_controller/cmd_vel` topic as a Twist message. 
- Contains a launch file named `solution.launch`, which launches the two nodes mentioned above as well as the simulation in Gazebo and Rviz. 

The graph below shows how my two packages are related to Mirte, RViz, other packages, and each other.

![ROS graph](https://i.imgur.com/T2hklbz.png)

## Getting started
### Prerequisites
To build this project, you'll have to download and run everything inside the [singularity](https://surfdrive.surf.nl/files/index.php/s/rEkvtYdWei27HFn) provided by the course. It contains ROS Noetic, some required packages (such as `pcl_obstacle_detector`), and the simulator tools _Rviz_ and _Gazebo_. These are all required to build and run this project.

You can run the singularity using the following command (replacing `/path/to/` with the actual path to the file you downloaded):
```
~$ singularity shell /path/to/ro47003_noetic_v5.sif
```

### Build
Inside the Singularity, open a terminal window (e.g. by using `terminator -u`) and take the following steps:

1. Source the ROS `setup.bash` file
```
~$ source /opt/ros/noetic/setup.bash
```

2. Create a catkin workspace (only if you don't have one already or prefer to use a new workspace).
```
~$ mkdir -p catkin_ws/src
```

3. Navigate into your catkin workspace's `src` folder and initialize it (only if you made a new workspace).
```
~/catkin_ws/src$ catkin_init_workspace
```

4. Clone this repository into the `src` folder of your catkin workspace. 
```
~/catkin_ws/src$ git clone git@gitlab.ro47003.3me.tudelft.nl:students-2324/lab4/group131.git
```

5. Clone the Mirte simulator repository into the `src` folder too.
```
~/catkin_ws/src$ git clone git@gitlab.ro47003.3me.tudelft.nl:students-2324/ro47003_mirte_simulator.git
```

6. Return to the workspace's root folder and run `catkin_make` to build and compile all the files. This can take a while.
```
~/catkin_ws$ catkin_make
```

7. Source the catkin workspace's `setup.bash` file.
```
~/catkin_ws$ source devel/setup.bash
```

## Run
To run my solution immediately after following the compile instructions above, simply enter the following command:

```
~$ roslaunch control_barrel_world solution.launch
```

This will launch Gazebo headless (without a GUI) and an RViz window, where you can see Mirte with the 3D obstacle detections in the main window, and the 2D person detections in the panels on the right. Now sit back and relax as Mirte drives around the obstacle track and makes an emergency stop once she detects a person. 

If you wish to open the Gazebo GUI too, run the following command:
```
~$ roslaunch control_barrel_world solution.launch gui:=true
```

This will allow you to easily restart the simulation and edit the test track. For example, you can move the pedestrian in front of Mirte to see the person detection and control algorithm in action. 

**Note:** If you wish to run my solution at a later moment, make sure you are inside the singularity and that you've sourced both the ROS and the catkin workspace `setup.bash` file _before_ running any of the above commands in this section.

Have fun with my project :)


```
  __  __              ____      _____  U _____ u 
U|' \/ '|u   ___   U |  _"\ u  |_ " _| \| ___"|/ 
\| |\/| |/  |_"_|   \| |_) |/    | |    |  _|"   
 | |  | |    | |     |  _ <     /| |\   | |___   
 |_|  |_|  U/| |\u   |_| \_\   u |_|U   |_____|  
<<,-,,-..-,_|___|_,-.//   \\_  _// \\_  <<   >>  
 (./  \.)\_)-' '-(_/(__)  (__)(__) (__)(__) (__) 
```
