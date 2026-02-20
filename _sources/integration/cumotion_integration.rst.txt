Isaac cuMotion Accelerated Planning
=====================================

Overview
--------

NVIDIA Isaac cuMotion provides GPU-accelerated trajectory planning as an
alternative to the default MoveIt2 OMPL planners. It can be used with the
WMX ROS2 application as a drop-in replacement for the MoveIt2 planner
backend.

Prerequisites
-------------

- NVIDIA GPU with CUDA support
- NVIDIA Isaac ROS packages installed
- MoveIt2 configured to use cuMotion as the planning pipeline

The ``manipulator_state`` node publishes to ``/joint_states`` and optionally
to ``/isaacsim/joint_command`` (configured via the ``isaacsim_joint_topic``
parameter in the Orin config).

Integration
-----------

cuMotion integrates through the same ``FollowJointTrajectory`` action
interface as MoveIt2. No changes to the WMX ROS2 nodes are required --
cuMotion plans the trajectory and sends it to the
``follow_joint_trajectory_server`` action server.

See :doc:`../api_reference/ros2_actions` for action server details and
:doc:`../api_reference/ros2_topics` for the joint state topics used by
the planner.

.. note::

   The Orin platform config (``orin_manipulator_config_cr3a.yaml``) includes
   the ``isaacsim_joint_topic`` parameter for Isaac Sim integration. See
   :doc:`../packages/wmx_ros2_package` for configuration details.

.. todo::

   Add step-by-step cuMotion setup and launch instructions once the
   integration is fully validated.
