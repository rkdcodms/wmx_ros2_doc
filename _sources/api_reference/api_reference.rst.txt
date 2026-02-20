API Reference
=============

Complete reference for all ROS2 interfaces exposed by the WMX ROS2 packages.

**ROS2 Services** documents the request/response interfaces for axis control
(servo on/off, jog, move to position), gripper control (open/close via
EtherCAT digital I/O), and system management (device creation, EtherCAT
scan, communication start/stop).

**ROS2 Topics** covers the published data streams: ``/joint_states`` at 500 Hz
for MoveIt2 feedback, ``/wmx/axis/state`` for detailed per-axis status, and
``/wmx/io/state`` for digital I/O monitoring.

**ROS2 Actions** describes the ``FollowJointTrajectory`` action server that
receives multi-waypoint trajectories from MoveIt2 and executes them on the
robot via WMX3 spline interpolation.

.. toctree::
   :maxdepth: 2

   ros2_services
   ros2_topics
   ros2_actions
