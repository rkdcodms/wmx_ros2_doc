MoveIt2 Motion Planning
========================

Overview
--------

MoveIt2 is the primary motion planning integration for the WMX ROS2
application. It provides collision-aware trajectory planning, and the
``follow_joint_trajectory_server`` node executes the planned trajectories
on real hardware via the WMX3 cubic spline engine.

Architecture
------------

.. code-block:: text

   MoveIt2 Planner
       │
       ├── Reads /joint_states (current robot state)
       │     └── Published by manipulator_state @ 500 Hz
       │
       ├── Plans collision-free trajectory
       │
       └── Sends FollowJointTrajectory Action Goal
             │
             └── follow_joint_trajectory_server
                   │
                   ├── Converts trajectory to WMX3 CSplinePos command
                   ├── Executes via AdvancedMotion API
                   └── Returns result (success/failure)
                         │
                         EtherCAT → Servo Drives → Robot Motion

Prerequisites
-------------

MoveIt2 must be installed as part of the workspace dependencies:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-moveit*

See :doc:`../getting_started` for the complete dependency list.

Configuration
-------------

The ``follow_joint_trajectory_server`` node must be configured with the
correct action name that MoveIt2 expects. This is set in the config YAML:

.. code-block:: yaml

   follow_joint_trajectory_server:
     ros__parameters:
       joint_number: 6
       joint_trajectory_action: /movensys_manipulator_arm_controller/follow_joint_trajectory

The action name must match the controller configuration in the MoveIt2
setup for your robot.

The ``manipulator_state`` node must publish to ``/joint_states`` so MoveIt2
can read the current robot state:

.. code-block:: yaml

   manipulator_state:
     ros__parameters:
       encoder_joint_topic: /joint_states

Usage
-----

1. Launch the WMX ROS2 manipulator nodes (see
   :doc:`../getting_started`):

   .. code-block:: bash

      # Intel platform example
      sudo --preserve-env=PATH \
           --preserve-env=AMENT_PREFIX_PATH \
           --preserve-env=COLCON_PREFIX_PATH \
           --preserve-env=PYTHONPATH \
           --preserve-env=LD_LIBRARY_PATH \
           --preserve-env=ROS_DISTRO \
           --preserve-env=ROS_VERSION \
           --preserve-env=ROS_PYTHON_VERSION \
           --preserve-env=ROS_DOMAIN_ID \
           --preserve-env=RMW_IMPLEMENTATION \
           bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
                    source ~/wmx_ros2_ws/install/setup.bash && \
                    ros2 launch wmx_ros2_package \
                      wmx_ros2_intel_manipulator_cr3a.launch.py \
                      use_sim_time:=false"

2. Launch MoveIt2 with your robot's MoveIt configuration package.

3. Plan and execute trajectories through the MoveIt2 interface (RViz plugin
   or programmatic API).

Trajectory Execution Details
-----------------------------

When MoveIt2 sends a trajectory, the ``follow_joint_trajectory_server``
processes it as described in the :doc:`../api_reference/ros2_actions` page:

- Maximum 1000 waypoints per trajectory
- Cubic spline interpolation via ``AdvancedMotion::StartCSplinePos()``
- The server blocks until motion completes
- Returns ``error_code = 0`` on success

See :doc:`../architecture/flowcharts` for the detailed motion execution flow
diagram.
