# ros_mongo
Interaction trace logger for robot I/O subsystem 

This is a suerp-minimal logger of interaction state for the humanoid robot. It is useful
only for debugging failed human-robot interactions.

It subscribes to several ROS topics:

* The blender face expression and gesture nodes (i.e. the stream of animations that
the robot s performing, and

* The face-detection node, wich publishes when a face is observed and tracked, and
it's 3D location. (Facial recognition is not used; names of faces are not recorded).

The messages are then recorded to a Mongo DB instance.

These two ROS topics are very highly specific to the Hanson Robotics animation
infrastructure, and are not useful in any general setting.

## Status
As of 2018, this system is not in use.

#### Copyright (c) 2017-2018 Hanson Robotics, Ltd.
