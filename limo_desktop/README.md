# Limo Desktop

![Rtab](https://limo-agx.github.io/_images/rtabmap.png)

This package contains a standard setup for using RVIZ with your Limo

## Set Up

Clone this repo into your workspace, install dependencies, build and source

    git clone https://github.com/limo-agx/limo.git
    git clone https://github.com/limo-agx/limo_desktop.viz
    cd ..
    rosdep install --from-paths src --ignore-src -y
    catkin_make
    source devel/setup.bash

## Launch Files

* view_ackerman_model.launch - Basic launch file for viewing the Ackerman URDF - See [Customizing URDF](https://limo-agx.github.io/custom_urdf.html)
* view_four_diff_model.launch - Basic launch file for viewing the differential-drive URDF - See [Customizing URDF](https://limo-agx.github.io/custom_urdf.html)
* view_navigation_gmapping.launch - RVIZ setup for common Gmapping Topics  - See [Gmapping](https://limo-agx.github.io/navigation_gmapping.html)
* view_navigation_rtabmap.launch - RVIZ setup for common RTABMap Topics  - See [RTABMap](https://limo-agx.github.io/navigation_rtabmap.html)
* view_robot.launch - Generic Rviz setup for monitoring your robot and sensors
