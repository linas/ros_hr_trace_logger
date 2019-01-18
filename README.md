# ros_hr_trace_logger
Interaction trace logger for Sophia robot I/O subsystem 

This is a super-minimal logger of interaction state for the humanoid robot. It is useful
only for debugging failed (or successful) human-robot interactions.

It subscribes to several ROS topics:

* The blender face expression and gesture nodes (i.e. the stream of animations that
the robot s performing), and

* The face-detection node, which publishes when a face is observed and tracked, and
it's 3D location. (Facial recognition is not used; names of faces are not recorded).

* Chat content is not logged.

The messages are then recorded to a Mongo DB instance.

These two ROS topics are very highly specific to the Hanson Robotics animation
infrastructure, and are not useful in any general setting.

## Status
As of 2018, this system is not in use.

#### Copyright (c) 2017-2018 Hanson Robotics, Ltd.
