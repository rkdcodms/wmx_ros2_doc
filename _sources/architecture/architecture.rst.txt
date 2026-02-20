Architecture & Design
=====================

The WMX ROS2 application follows a layered architecture separating hardware
control (WMX3/EtherCAT), ROS2 communication, and motion planning.

**System Overview** describes the three-layer design: the WMX3 motion control
engine at the bottom, three ROS2 nodes in the middle (general control, joint
state publishing, trajectory execution), and MoveIt2/Isaac cuMotion planners
at the top.

**Communication** details all ROS2 topics, services, and actions used for
inter-node communication — including joint state publishing at 500 Hz,
axis control services, gripper I/O, and the ``FollowJointTrajectory`` action
interface.

**Flowcharts** illustrate the key execution paths: node startup and WMX3
device initialization, MoveIt2 trajectory execution pipeline, and the
gripper open/close sequence.

.. toctree::
   :maxdepth: 2

   system_overview
   communication
   flowcharts
