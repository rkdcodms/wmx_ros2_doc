Launching with Mock Hardware
============================

The WMX ROS2 application nodes communicate directly with the WMX3 motion control
engine over EtherCAT. There is no built-in mock hardware mode in the current
launch files -- the nodes will attempt to connect to real hardware on startup.

This page describes how to test parts of the system without a physical robot.

.. note::

   The launch files support a ``use_sim_time`` argument (default: ``false``),
   which controls clock source but does not enable a simulated hardware backend.

Option 1: Test the General Node (Standalone)
---------------------------------------------

The ``wmx_ros2_general_node`` can start and create services even if the WMX3 engine
is not fully communicating. This allows you to verify the ROS2 interface layer:

.. code-block:: bash

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
        bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                 source ~/wmx_ros2_ws/install/setup.bash && \
                 ros2 launch wmx_ros2_package wmx_ros2_general.launch.py"

.. note::

   ``sudo`` is required because the WMX3 engine needs root access for device
   creation. The ``--preserve-env`` flags pass your ROS2 environment into the
   root shell.

If no EtherCAT hardware is connected, the node will log retry attempts during
device creation and may report errors, but the ROS2 service endpoints will still
be registered.

**In a separate terminal, verify the services exist:**

.. code-block:: bash

   ros2 service list | grep wmx

Expected output:

.. code-block:: text

   /wmx/axis/clear_alarm
   /wmx/axis/homing
   /wmx/axis/set_gear_ratio
   /wmx/axis/set_mode
   /wmx/axis/set_on
   /wmx/axis/set_polarity
   /wmx/engine/get_status
   /wmx/engine/set_comm
   /wmx/engine/set_device

**Check the axis state topic:**

.. code-block:: bash

   ros2 topic list | grep wmx

Expected output:

.. code-block:: text

   /wmx/axis/position
   /wmx/axis/position/relative
   /wmx/axis/state
   /wmx/axis/velocity

Option 2: Publish Simulated Joint States
------------------------------------------

If you need to test downstream components (MoveIt2, RViz visualization) without
real encoder feedback, you can manually publish fake joint states.

**Start a joint state publisher:**

.. code-block:: bash

   ros2 topic pub /joint_states sensor_msgs/msg/JointState \
     "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
       name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
              'picker_1_joint', 'picker_2_joint'],
       position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
       velocity: [],
       effort: []}" --rate 50

This publishes zeroed joint states at 50 Hz, simulating the
``manipulator_state`` node's output. Adjust the ``position`` array to test
different joint configurations.

Option 3: Simulation Environments
-----------------------------------

The ``manipulator_state`` node publishes to multiple topic targets for
simulation compatibility. The available simulator topic depends on the
platform configuration:

.. list-table::
   :header-rows: 1
   :widths: 25 30 25 20

   * - Simulator
     - Joint State Topic
     - Message Type
     - Platform
   * - MoveIt2 / RViz
     - ``/joint_states``
     - ``sensor_msgs/JointState``
     - Both
   * - NVIDIA Isaac Sim
     - ``/isaacsim/joint_command``
     - ``sensor_msgs/JointState``
     - Orin
   * - Gazebo
     - ``/gazebo_position_controller/commands``
     - ``std_msgs/Float64MultiArray``
     - Intel

The Intel config sets ``gazebo_joint_topic`` while the Orin config sets
``isaacsim_joint_topic``. The ``/joint_states`` topic is always published on
both platforms.

To use the simulated time source with any launch file:

.. code-block:: bash

   ros2 launch wmx_ros2_package wmx_ros2_intel_manipulator_cr3a.launch.py \
     use_sim_time:=true

Verification Commands
---------------------

Use these commands to inspect the running system:

**List all active nodes:**

.. code-block:: bash

   ros2 node list

Expected nodes (manipulator launch):

.. code-block:: text

   /follow_joint_trajectory_server
   /manipulator_state
   /wmx_ros2_general_node

**List all active topics:**

.. code-block:: bash

   ros2 topic list

Expected topics (manipulator launch):

.. code-block:: text

   /joint_states
   /isaacsim/joint_command
   /gazebo_position_controller/commands
   /wmx/axis/position
   /wmx/axis/position/relative
   /wmx/axis/state
   /wmx/axis/velocity
   /wmx/set_gripper

**List all active services:**

.. code-block:: bash

   ros2 service list

**List all active actions:**

.. code-block:: bash

   ros2 action list

Expected action:

.. code-block:: text

   /movensys_manipulator_arm_controller/follow_joint_trajectory

**Monitor joint state data:**

.. code-block:: bash

   ros2 topic echo /joint_states

**Check topic publishing rate:**

.. code-block:: bash

   ros2 topic hz /joint_states

Expected: approximately 500 Hz (configured via ``joint_feedback_rate`` parameter).

**Query engine status:**

.. code-block:: bash

   ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger

Returns the engine state (``Idle``, ``Running``, ``Communicating``, etc.).
