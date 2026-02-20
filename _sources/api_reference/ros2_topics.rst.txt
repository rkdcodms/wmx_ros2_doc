ROS2 Topics
============

The WMX ROS2 application uses topics for real-time data streaming: encoder
feedback from the robot and motion commands to the WMX3 engine. Topics are
divided into published (output) and subscribed (input) categories.

.. list-table:: Topic Summary
   :header-rows: 1
   :widths: 30 25 15 15 15

   * - Topic
     - Message Type
     - Direction
     - Rate
     - Node
   * - ``/joint_states``
     - ``sensor_msgs/JointState``
     - Published
     - 500 Hz
     - ``manipulator_state``
   * - ``/isaacsim/joint_command``
     - ``sensor_msgs/JointState``
     - Published
     - 500 Hz
     - ``manipulator_state``
   * - ``/gazebo_.../commands``
     - ``std_msgs/Float64MultiArray``
     - Published
     - 500 Hz
     - ``manipulator_state``
   * - ``/wmx/axis/state``
     - ``wmx_ros2_message/AxisState``
     - Published
     - 100 Hz
     - ``wmx_ros2_general_node``
   * - ``/wmx/axis/velocity``
     - ``wmx_ros2_message/AxisVelocity``
     - Subscribed
     - On demand
     - ``wmx_ros2_general_node``
   * - ``/wmx/axis/position``
     - ``wmx_ros2_message/AxisPose``
     - Subscribed
     - On demand
     - ``wmx_ros2_general_node``
   * - ``/wmx/axis/position/relative``
     - ``wmx_ros2_message/AxisPose``
     - Subscribed
     - On demand
     - ``wmx_ros2_general_node``

.. contents:: Topic Categories
   :local:
   :depth: 1

Custom Message Types
--------------------

Before reviewing the topics, here are the custom message type definitions
from ``wmx_ros2_message``.

**wmx_ros2_message/msg/AxisState**

.. code-block:: text

   int32[] amp_alarm          # Amplifier alarm active per axis
   int32[] servo_on           # Servo enabled per axis
   int32[] home_done          # Homing completed per axis
   int32[] in_pos             # At target position per axis
   int32[] negative_ls        # Negative limit switch active per axis
   int32[] positive_ls        # Positive limit switch active per axis
   int32[] home_switch        # Home switch active per axis
   float64[] pos_cmd         # Commanded position per axis (radians)
   float64[] velocity_cmd    # Commanded velocity per axis (rad/s)
   float64[] actual_pos      # Actual encoder position per axis (radians)
   float64[] actual_velocity # Actual velocity per axis (rad/s)
   float64[] actual_torque   # Actual torque per axis (Nm)

**wmx_ros2_message/msg/AxisPose**

.. code-block:: text

   int32[] index         # Axis indices to command
   float64[] target      # Target positions (radians)
   string profile        # Motion profile type (reserved)
   float64[] velocity    # Motion velocity per axis (rad/s)
   float64[] acc         # Acceleration per axis (rad/s^2)
   float64[] dec         # Deceleration per axis (rad/s^2)

**wmx_ros2_message/msg/AxisVelocity**

.. code-block:: text

   int32[] index         # Axis indices to command
   string profile        # Motion profile type (reserved)
   float64[] velocity    # Target velocity per axis (rad/s)
   float64[] acc         # Acceleration per axis (rad/s^2)
   float64[] dec         # Deceleration per axis (rad/s^2)

Published Topics
----------------

Joint State Feedback
^^^^^^^^^^^^^^^^^^^^^

/joint_states
""""""""""""""

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``sensor_msgs/msg/JointState``
   * - **Publisher**
     - ``manipulator_state``
   * - **Rate**
     - 500 Hz (configurable via ``joint_feedback_rate`` parameter)
   * - **Configurable**
     - Topic name set via ``encoder_joint_topic`` parameter
   * - **Source**
     - ``manipulator_state.cpp:encoderJointStep()``

The primary joint state topic used by MoveIt2, RViz, and other ROS2
consumers. Publishes encoder feedback for all 6 robot joints plus 2 gripper
finger joints.

.. list-table:: Message Fields
   :header-rows: 1
   :widths: 25 20 55

   * - Field
     - Type
     - Description
   * - ``header.stamp``
     - ``builtin_interfaces/Time``
     - Current ROS2 time (set before publishing)
   * - ``header.frame_id``
     - ``string``
     - Empty (default)
   * - ``name``
     - ``string[]``
     - 8 joint names: ``["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "picker_1_joint", "picker_2_joint"]``
   * - ``position``
     - ``float64[]``
     - 8 values: 6 joint positions (``actualPos`` from encoder, radians) + 2 gripper positions (from I/O bit)
   * - ``velocity``
     - ``float64[]``
     - 8 values: 6 joint velocities (``actualVelocity`` from encoder, rad/s) + 2 gripper velocities (always ``0.0``)
   * - ``effort``
     - ``float64[]``
     - Empty (not populated)

The gripper positions (indices 6 and 7) are derived from the EtherCAT
digital I/O output bit 0 via ``Io::GetOutBit(0, 0)``. When the gripper is
closed (bit=1), they report the ``gripper_close_value`` parameter (default
``0.045``); when open (bit=0), the ``gripper_open_value`` parameter (default
``0.00``).

**Example -- Monitor joint states:**

.. code-block:: bash

   ros2 topic echo /joint_states

**Example -- Check publishing rate:**

.. code-block:: bash

   ros2 topic hz /joint_states

Expected: approximately 500 Hz.

**Example -- View only joint positions:**

.. code-block:: bash

   ros2 topic echo /joint_states --field position

Simulator Integration Topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

/isaacsim/joint_command
""""""""""""""""""""""""

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``sensor_msgs/msg/JointState``
   * - **Publisher**
     - ``manipulator_state``
   * - **Rate**
     - 500 Hz (same timer as ``/joint_states``)
   * - **Configurable**
     - Topic name set via ``isaacsim_joint_topic`` parameter
   * - **Platform**
     - Orin config only (Intel config does not set this topic)
   * - **Purpose**
     - Mirror joint states for NVIDIA Isaac Sim integration

Same data content as ``/joint_states`` (8 joint names, positions, velocities).

.. note::

   This topic is published **before** the ``header.stamp`` is set in the
   ``encoderJointStep()`` callback. The timestamp in the Isaac Sim message
   will be unset (zero). This is an implementation detail of the publish
   order in the source code.

**Example:**

.. code-block:: bash

   ros2 topic echo /isaacsim/joint_command

/gazebo_position_controller/commands
""""""""""""""""""""""""""""""""""""""

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``std_msgs/msg/Float64MultiArray``
   * - **Publisher**
     - ``manipulator_state``
   * - **Rate**
     - 500 Hz (same timer as ``/joint_states``)
   * - **Configurable**
     - Topic name set via ``gazebo_joint_topic`` parameter
   * - **Platform**
     - Intel config only (Orin config does not set this topic)
   * - **Purpose**
     - Mirror joint positions for Gazebo simulation integration

Publishes 8 position values (6 joint positions + 2 gripper positions) as a
flat ``Float64MultiArray``. The array is pre-sized to 8 elements.

.. list-table:: Message Fields
   :header-rows: 1
   :widths: 25 20 55

   * - Field
     - Type
     - Description
   * - ``data``
     - ``float64[]``
     - 8 values: 6 joint positions in radians (``actualPos``) + 2 gripper positions (``gripper_close_value`` or ``gripper_open_value``)

**Example:**

.. code-block:: bash

   ros2 topic echo /gazebo_position_controller/commands

Axis State Monitoring
^^^^^^^^^^^^^^^^^^^^^^

/wmx/axis/state
"""""""""""""""""

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``wmx_ros2_message/msg/AxisState``
   * - **Publisher**
     - ``wmx_ros2_general_node``
   * - **Rate**
     - 100 Hz (hardcoded ``rate_=100``)
   * - **Axis Count**
     - 2 (hardcoded ``axisCount_=2``)
   * - **Source**
     - ``wmx_ros2_core_motion.cpp:axisStateStep()``
   * - **Purpose**
     - Detailed per-axis status from the WMX3 CoreMotion API

Publishes comprehensive axis status including servo state, alarm status,
limit switches, commanded vs. actual positions, velocities, and torques.

.. important::

   The axis count is hardcoded to **2** in ``wmx_ros2_general.hpp``
   (``axisCount_=2``). This means the arrays in ``AxisState`` contain 2
   elements each, monitoring axes 0 and 1 only.

.. list-table:: Message Fields
   :header-rows: 1
   :widths: 25 15 60

   * - Field
     - Type
     - Description
   * - ``amp_alarm``
     - ``int32[]``
     - Amplifier alarm active per axis (``1`` = fault)
   * - ``servo_on``
     - ``int32[]``
     - Servo drive enabled per axis (``1`` = on)
   * - ``home_done``
     - ``int32[]``
     - Homing procedure completed per axis (``1`` = done)
   * - ``in_pos``
     - ``int32[]``
     - Axis has reached commanded position (``1`` = in position)
   * - ``negative_ls``
     - ``int32[]``
     - Negative limit switch triggered per axis (``1`` = triggered)
   * - ``positive_ls``
     - ``int32[]``
     - Positive limit switch triggered per axis (``1`` = triggered)
   * - ``home_switch``
     - ``int32[]``
     - Home switch triggered per axis (``1`` = triggered)
   * - ``pos_cmd``
     - ``float64[]``
     - Commanded position per axis (radians)
   * - ``velocity_cmd``
     - ``float64[]``
     - Commanded velocity per axis (rad/s)
   * - ``actual_pos``
     - ``float64[]``
     - Actual encoder position per axis (radians)
   * - ``actual_velocity``
     - ``float64[]``
     - Actual velocity per axis (rad/s)
   * - ``actual_torque``
     - ``float64[]``
     - Actual torque per axis (Nm)

The callback clears all vectors before each publish cycle and rebuilds them
from the ``CoreMotion::GetStatus()`` result.

**Example -- Monitor axis state:**

.. code-block:: bash

   ros2 topic echo /wmx/axis/state

**Example -- Check for alarms:**

.. code-block:: bash

   ros2 topic echo /wmx/axis/state --field amp_alarm

Subscribed Topics (Motion Commands)
------------------------------------

These topics accept motion commands and are subscribed to by the
``wmx_ros2_general_node`` (source: ``wmx_ros2_core_motion.cpp``). Publishing
to these topics directly controls the WMX3 engine and moves real motors.

.. warning::

   Publishing to these topics will cause immediate physical motion. Ensure
   the robot workspace is clear and all safety precautions are in place.

/wmx/axis/velocity
^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``wmx_ros2_message/msg/AxisVelocity``
   * - **Subscriber**
     - ``wmx_ros2_general_node``
   * - **Callback**
     - ``axisVelCallback()``
   * - **Purpose**
     - Command continuous velocity motion on specified axes

For each axis in ``msg->index``, the callback:

1. Sets ``velocity_.axis`` to the axis index
2. Sets the velocity profile to ``ProfileType::Trapezoidal``
3. Sets velocity, acceleration, and deceleration from the message
4. Calls ``CoreMotion::StartVel(&velocity_)``

The axis must be in velocity mode (see :doc:`ros2_services` for
``/wmx/axis/set_mode`` with ``data=[1]``).

**Example -- Rotate axis 0 at 1.0 rad/s:**

.. code-block:: bash

   ros2 topic pub --once /wmx/axis/velocity wmx_ros2_message/msg/AxisVelocity \
     "{index: [0], profile: '', velocity: [1.0], acc: [10.0], dec: [10.0]}"

**Example -- Stop axis 0:**

.. code-block:: bash

   ros2 topic pub --once /wmx/axis/velocity wmx_ros2_message/msg/AxisVelocity \
     "{index: [0], profile: '', velocity: [0.0], acc: [10.0], dec: [10.0]}"

/wmx/axis/position
^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``wmx_ros2_message/msg/AxisPose``
   * - **Subscriber**
     - ``wmx_ros2_general_node``
   * - **Callback**
     - ``axisPoseCallback()``
   * - **Purpose**
     - Command absolute position motion on specified axes

For each axis in ``msg->index``, the callback:

1. Sets ``position_.axis`` to the axis index
2. Sets ``position_.target`` to the target position
3. Sets the profile to ``ProfileType::Trapezoidal``
4. Sets velocity, acceleration, and deceleration from the message
5. Calls ``CoreMotion::StartPos(&position_)`` -- **absolute positioning**

The ``target`` values are absolute positions in radians relative to the home
position.

**Example -- Move axis 0 to 1.5 radians:**

.. code-block:: bash

   ros2 topic pub --once /wmx/axis/position wmx_ros2_message/msg/AxisPose \
     "{index: [0], target: [1.5], profile: '', velocity: [5.0], acc: [10.0], dec: [10.0]}"

/wmx/axis/position/relative
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``wmx_ros2_message/msg/AxisPose``
   * - **Subscriber**
     - ``wmx_ros2_general_node``
   * - **Callback**
     - ``axisPoseRelativeCallback()``
   * - **Purpose**
     - Command relative position motion on specified axes

For each axis in ``msg->index``, the callback:

1. Sets ``position_.axis`` to the axis index
2. Sets ``position_.target`` to the relative displacement
3. Sets the profile to ``ProfileType::Trapezoidal``
4. Sets velocity, acceleration, and deceleration from the message
5. Calls ``CoreMotion::StartMov(&position_)`` -- **relative movement**

The ``target`` values are relative displacements in radians from the current
position.

**Example -- Move axis 0 by +0.5 radians from current position:**

.. code-block:: bash

   ros2 topic pub --once /wmx/axis/position/relative wmx_ros2_message/msg/AxisPose \
     "{index: [0], target: [0.5], profile: '', velocity: [5.0], acc: [10.0], dec: [10.0]}"

Differential Drive Topics (Optional)
--------------------------------------

The ``diff_drive_controller`` node provides topics for mobile base control.
This node is currently disabled in the build (commented out in
``CMakeLists.txt``).

.. note::

   The differential drive controller is not built by default. It must be
   enabled in ``CMakeLists.txt`` to use these topics.

/cmd_vel (Subscribed)
^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``geometry_msgs/msg/Twist``
   * - **Subscriber**
     - ``diff_drive_controller``
   * - **Configurable**
     - Topic name set via ``cmd_vel_topic`` parameter
   * - **Purpose**
     - Receive linear/angular velocity commands for the mobile base

The ``cmdCallback()`` stores the incoming Twist message and republishes it.
The ``cmdVelStep()`` timer (at ``rate`` Hz) then:

1. Checks engine state is ``Communicating``
2. Checks for amplifier alarms and servo status on both wheel axes
3. Computes wheel angular velocities using differential drive kinematics:

   - ``left_omega = (2 * linear.x - angular.z * wheel_to_wheel) / (2 * wheel_radius)``
   - ``right_omega = (2 * linear.x + angular.z * wheel_to_wheel) / (2 * wheel_radius)``

4. Calls ``CoreMotion::StartVel()`` for each wheel axis

/cmd_vel_check (Published)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``geometry_msgs/msg/Twist``
   * - **Publisher**
     - ``diff_drive_controller``
   * - **Configurable**
     - Topic name set via ``encoder_vel_topic`` parameter
   * - **Purpose**
     - Echo the received ``/cmd_vel`` for debugging

/velocity_controller/commands (Published)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``std_msgs/msg/Float64MultiArray``
   * - **Publisher**
     - ``diff_drive_controller``
   * - **Configurable**
     - Topic name set via ``encoder_omega_topic`` parameter
   * - **Purpose**
     - Publish actual wheel angular velocities from encoder feedback

Publishes 2 values: ``[left_velocity, right_velocity]`` read from the
``CoreMotion::GetStatus()`` result for the left and right wheel axes.

/odom_enc (Published)
^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 75

   * - **Message Type**
     - ``nav_msgs/msg/Odometry``
   * - **Publisher**
     - ``diff_drive_controller``
   * - **Configurable**
     - Topic name set via ``encoder_odometry_topic`` parameter
   * - **Purpose**
     - Publish encoder-based odometry (velocity only)

Computes odometry from wheel angular velocities:

- ``linear_vel = (right_omega * wheel_radius + left_omega * wheel_radius) / 2``
- ``angular_vel = (right_omega * wheel_radius - left_omega * wheel_radius) / wheel_to_wheel``

Publishes in ``twist.twist.linear.x`` and ``twist.twist.angular.z``. The
``pose`` fields are not populated.

See Also
--------

- :doc:`ros2_services` -- Service API for engine management and axis control
- :doc:`ros2_actions` -- FollowJointTrajectory action for MoveIt2 integration
- :doc:`../packages/wmx_ros2_message` -- Custom message type definitions
- :doc:`../packages/wmx_ros2_package` -- Node documentation with parameters
