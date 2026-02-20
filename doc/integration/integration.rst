Integration Scenarios
=====================

WMX ROS2 supports multiple motion planning backends and can be extended with
custom planners and applications. Choose the integration that fits your use case:

- **MoveIt2** -- Standard ROS2 motion planning via ``FollowJointTrajectory``
- **Isaac cuMotion** -- NVIDIA GPU-accelerated collision-aware planning
- **Custom Planner** -- Integrate your own planning algorithm
- **Custom Application** -- Build standalone ROS2 apps using WMX services

.. toctree::
   :maxdepth: 2

   moveit2_integration
   cumotion_integration
   custom_planner
   custom_application
