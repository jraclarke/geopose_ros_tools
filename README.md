# geopose_ros_tools
Tools for the support of GeoPose standard implementations in ROS.

# What is GeoPose?

When a real or digital object's pose is defined relative to a geographical frame of reference, it will be called a geographically-anchored pose, or ''GeoPose'' for short. All physical world objects have a geographically-anchored pose. Digital objects may be assigned/attributed a GeoPose. 

![geopose](https://github.com/jraclarke/geopose_ros_tools/assets/20532947/96234418-65e5-4bdd-b69d-9859732d5a5d)

# GeoPose as a Standard

OGC GeoPose Standard defines the encodings for the real world position and orientation of a real or a digital object in a machine-readable form.
Using GeoPose enables the easy integration of digital elements on and in relation to the surface of the planet. 

# Run the Demo

1) Make sure you have cloned the ogc_geopose_msgs [repo](https://github.com/jraclarke/ogc_geopose_msgs) to your ROS/ROS2 workspace:
   <br>``` git clone https://github.com/jraclarke/ogc_geopose_msgs.git```
2) Clone this repo to the same workspace:
   <br>``` git clone https://github.com/jraclarke/geopose_ros_tools.git```
3) Build within your workspace:
   <br>``` colcon build```
4) Launch the demo:
   <br>``` ros2 launch geopose_ros_tools launch.py```
