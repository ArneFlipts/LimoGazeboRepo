# AgileX Limo Gazebo Simulation
This Repo holds all the necessary information needed to run the simulation, the needed: packages, files, etc. and how to install them.
After the initial setup you will be able to simulate everyting from the real Limo on the same simulation table or your own world.

For this we will use Gazebo and ROS.
More information about that here:
- Gazebo: https://gazebosim.org/home
- ROS: https://www.ros.org

## Initial Setup
### Environment
You can use WSL2 with Ubunutu 20.04 (Setup: https://learn.microsoft.com/en-us/windows/wsl/install) if you are on windows or use native Linux also with Ubuntu 20.04.
For ROS we use Noetic (Setup: https://wiki.ros.org/noetic/Installation/Ubuntu). 

### Extra packages
- ros-control function package, robot control middleware
  ```
  sudo apt-get install ros-noetic-ros-control 
  ```
- ros-controllers function package, kinematics plug-in of common models provided by ROS
  ```
  sudo apt-get install ros-noetic-ros-controllers 
  ```
- gazebo-ros function package, communication interface between gazebo and ROS
  ```
  sudo apt-get install ros-noetic-gazebo-ros
  ```
- gazebo-ros-control function package, communication standard controller between ROS and Gazebo
  ```
  sudo apt-get install ros-noetic-gazebo-ros-control
  ```
- joint-state-publisher-gui package, used to visualize the joint control
  ```
  sudo apt-get install ros-noetic-joint-state-publisher-gui 
  ```
- rqt-robot-steering plug-in, a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar
  ```
  sudo apt-get install ros-noetic-rqt-robot-steering 
  ```
- telop-twist-keyboard is keyboard control function package, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard
  ```
  sudo apt-get install ros-noetic-teleop-twist-keyboard 
  ```
- rosdep, a command-line tool for installing system dependencies
  ```
  sudo apt-get install python3-rosdep
  ```

### Setup workspace
- Create the workspace with name of your choice, for example limo_ws
  ```
  mkdir -p limo_ws/src
  ```
- Initialize the src folder and clone the github
  ```
  cd limo_ws/src
  ```
  ```
  catkin_init_workspace
  ```
  ```
  git clone https://github.com/ArneFlipts/LimoSimFiles.git
  ```
- Go back to the root folder (limo_ws) and check if all the dependencies are installed/compile everything 
  ```
  cd ..
  ```
  ```
  rosdep install --from-paths src --ignore-src -r -y
  ```
  ```
  catkin_make
  ```

### Check if the setup was successful
- In the root folder (limo_ws) source the correct script
  ```
  source devel/setup.bash
  ```
- Run the start file of limo and visualize the model in Rviz
  ```
  roslaunch limo_description display_models.launch
  ```

## The actual Simulation
### Limo drive modes
Here we made a distinction between two drive modes: Ackerman mode and Four-wheel differential steering movement mode
For both we need to again source in the root folder
```
source devel/setup.bash
```

#### Ackerman mode
- Start the simulation environment
  ```
  roslaunch limo_gazebo limo_ackerman.launch
  ```
- Start rqt_robot_steering movement control plug-in, the sliding bar can control the robot motion
  ```
  rosrun rqt_robot_steering rqt_robot_steering
  ```

#### Four-wheel differential mode
- Start the simulation environment
  ```
  roslaunch limo_gazebo limo_ackerman.launch
  ```
- Control by keyboard, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard
  ```
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

### Creating a map and navigating on that map
We will simulate two ways of mapping: GMapping and RTABMap
One thing to note, unlike the real life counterpart we wont need to initialize the sensors first. As this gets done in the xacro and gazebo files, 
in these files you can also play with some sensor values.

#### GMapping
##### Create a map
- Start GMapping
  ```
  roslaunch limo_navigation limo_gmapping.launch
  ```
- Visualize the mapping process
  ```
  roslaunch limo_viz view_navigation_gmapping.launch
  ```
- Once the map is complete, save it (It will get saved in the directory the command was run)
  ```
  rosrun map_server map_saver
  ```
##### Autonomous Navigation
- Start Navigation with Ackerman mode
  ```
  roslaunch limo_navigation limo_navigation_amcl_ackerman.launch
  ```
- Or start navigation with Four-diff mode
  ```
  roslaunch limo_navigation limo_navigation_amcl_diff.launch
  ```
- To send the robot a goal, use the interactive 2D Nav Goal tool in RVIZ or send a message on the topic: /move_base_simple/goal topic !ADD EXAMPLE MESSAGE!

#### RTABMap
##### Create a map
- Start RTABMap`
  ```
  roslaunch limo_navigation limo_rtabmap.launch
  ```
- Visualize the mapping progress
  ```
  roslaunch limo_viz view_navigation_rtabmap.launch
  ```
- To save the map you can just stop the process from step one and the map will get saved in ~/.ros as rtabmap.db

##### Autonomous Navigation
- Start Navigation with Ackerman mode
  ```
  roslaunch limo_navigation limo_navigation_rtabmap_ackerman.launch
  ```
- Or start navigation with Four-diff mode
  ```
  roslaunch limo_navigation limo_navigation_rtabmap_diff.launch
  ```
- To send the robot a goal, use the interactive 2D Nav Goal tool in RVIZ or send a message on the topic: /move_base_simple/goal topic !ADD EXAMPLE MESSAGE!

#### Some remarks
these methodes will use the sensors (GMapping only the lidar, RTABMap both lidar and camera) data to estimate its location on the map. This methode is not foolproof but moving aroud with the robot will increase the precision, this is something that has to be done once and then it can drive around on its own without issue.

## Important files and folders

