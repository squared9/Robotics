# 3D Segmentation in ROS on RGB-D data

This project performs 3D segmentation on data obtained from a RGB-D camera.

*To run this project, copy "sensor_stick" directory to "src" directory of your catkin workspace (typically "catkin_ws"), then run catkin_make, source devel/setup.bash, then execute "roslaunch sensor_stick robot_spawn.launch".*

The original project dataset looks like this:
![image](images/3D_Point_Cloud.PNG)

In the first step, RGB-D point cloud is downsampled using Voxel grid filter.

Then a passthrough filter is used to focus only on area above table (table desk included)

[RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) is used to perform segmentation around table desktop plane.

The table is extracted as positive segments:

![image](images/3D_Table.PNG)

The objects are extracted as negative segments:

![image](images/3D_Objects.PNG)

In the end, Euclidean clustering utilizing Kd-tree is used to partition object mesh into individual 3D objects and visualized in RViz: 
![image](images/3D_clustering.png)
