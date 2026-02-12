Communication
=============

This section documents all ROS2 communication interfaces and hardware communication
details, derived from the actual source code of the WMX ROS2 application.

ROS2 Topics
-----------

Published Topics
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 28 25 22 25

   * - Topic Name
     - Message Type
     - Publisher Node
     - Description
   * - ``/wmx/axis/state``
     - ``wmx_ros2_message/msg/AxisState``
     - ``wmx_ros2_general_node``
     - Periodic axis status at 100 Hz. Contains alarm states, servo on/off,
       homing status, limit switches, commanded and actual position/velocity/torque
       for all configured axes.
   * - ``/joint_states`` :sup:`(configurable)`
     - ``sensor_msgs/msg/JointState``
     - ``manipulator_state``
     - Joint encoder feedback for MoveIt2 and ``robot_state_publisher``. Includes
       6 joint positions, velocities, and 2 gripper finger positions.
       Published at the configurable ``joint_feedback_rate`` (default: 500 Hz).
   * - ``/isaacsim/joint_command`` :sup:`(configurable)`
     - ``sensor_msgs/msg/JointState``
     - ``manipulator_state``
     - Same joint state data formatted for NVIDIA Isaac Sim. Topic name set via
       ``isaacsim_joint_topic`` parameter.
   * - ``/gazebo_position_controller/commands`` :sup:`(configurable)`
     - ``std_msgs/msg/Float64MultiArray``
     - ``manipulator_state``
     - Joint positions as a float array for Gazebo position controller plugin.
       Contains 8 values (6 joints + 2 gripper fingers).

.. note::

   Topics marked :sup:`(configurable)` have their names set via ROS2 parameters in
   the YAML configuration files. The names shown are the defaults from
   ``intel_manipulator_config_cr3a.yaml``.

Subscribed Topics
^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 28 28 22 22

   * - Topic Name
     - Message Type
     - Subscriber Node
     - Description
   * - ``/wmx/axis/velocity``
     - ``wmx_ros2_message/msg/AxisVelocity``
     - ``wmx_ros2_general_node``
     - Velocity motion commands. Starts velocity mode motion for each specified
       axis with trapezoidal profile.
   * - ``/wmx/axis/position``
     - ``wmx_ros2_message/msg/AxisPose``
     - ``wmx_ros2_general_node``
     - Absolute position motion commands. Moves each specified axis to the target
       position with trapezoidal profile (``StartPos``).
   * - ``/wmx/axis/position/relative``
     - ``wmx_ros2_message/msg/AxisPose``
     - ``wmx_ros2_general_node``
     - Relative position motion commands. Moves each specified axis by the
       target offset relative to current position (``StartMov``).

ROS2 Services
-------------

.. list-table::
   :header-rows: 1
   :widths: 27 25 20 28

   * - Service Name
     - Service Type
     - Server Node
     - Description
   * - ``/wmx/engine/set_device``
     - ``wmx_ros2_message/srv/SetEngine``
     - ``wmx_ros2_general_node``
     - Create (``data=true``) or close (``data=false``) a WMX3 device.
       Requires ``path`` (e.g., ``/opt/lmx/``) and ``name`` fields.
   * - ``/wmx/engine/set_comm``
     - ``std_srvs/srv/SetBool``
     - ``wmx_ros2_general_node``
     - Start (``data=true``) or stop (``data=false``) EtherCAT communication
       with a 10-second timeout.
   * - ``/wmx/engine/get_status``
     - ``std_srvs/srv/Trigger``
     - ``wmx_ros2_general_node``
     - Query the current WMX3 engine state. Returns one of: ``Idle``,
       ``Running``, ``Communicating``, ``Shutdown``, ``Unknown``.
   * - ``/wmx/axis/set_on``
     - ``wmx_ros2_message/srv/SetAxis``
     - ``wmx_ros2_general_node``
     - Enable (``data=[1]``) or disable (``data=[0]``) servo drives for
       specified axis indices.
   * - ``/wmx/axis/clear_alarm``
     - ``wmx_ros2_message/srv/SetAxis``
     - ``wmx_ros2_general_node``
     - Clear amplifier alarms on the specified axes.
   * - ``/wmx/axis/set_mode``
     - ``wmx_ros2_message/srv/SetAxis``
     - ``wmx_ros2_general_node``
     - Set axis command mode: ``0`` = Position mode, ``1`` = Velocity mode.
   * - ``/wmx/axis/set_polarity``
     - ``wmx_ros2_message/srv/SetAxis``
     - ``wmx_ros2_general_node``
     - Set axis polarity: ``1`` = normal, ``-1`` = reversed. Used to correct
       motor rotation direction.
   * - ``/wmx/axis/set_gear_ratio``
     - ``wmx_ros2_message/srv/SetAxisGearRatio``
     - ``wmx_ros2_general_node``
     - Configure encoder gear ratio per axis using numerator/denominator values.
       Converts between encoder counts and engineering units (e.g., radians).
   * - ``/wmx/axis/homing``
     - ``wmx_ros2_message/srv/SetAxis``
     - ``wmx_ros2_general_node``
     - Execute homing procedure for specified axes. Uses ``CurrentPos`` home type
       (sets current position as home). Blocks until homing completes.
   * - ``/wmx/set_gripper`` :sup:`(configurable)`
     - ``std_srvs/srv/SetBool``
     - ``follow_joint_trajectory_server``
     - Open (``data=false``) or close (``data=true``) the gripper via digital
       I/O bit 0 on channel 0. Topic name set via ``wmx_gripper_topic`` parameter.

ROS2 Actions
------------

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Action Name
     - Action Type
     - Server Node
   * - ``/movensys_manipulator_arm_controller/follow_joint_trajectory`` :sup:`(configurable)`
     - ``control_msgs/action/FollowJointTrajectory``
     - ``follow_joint_trajectory_server``

**Action Details:**

The ``FollowJointTrajectory`` action is the primary interface between motion planners
(MoveIt2, cuMotion) and the robot hardware.

**Goal:**

- Receives a ``trajectory_msgs/JointTrajectory`` containing an ordered list of
  waypoints with joint positions and timestamps.

**Execution:**

1. Validates that the number of trajectory points does not exceed 1000
2. Extracts joint positions and time intervals from each trajectory point
3. Constructs a WMX3 cubic spline command (``PointTimeSplineCommand``) with:

   - ``dimensionCount`` = number of joints (typically 6)
   - Position array for each point across all joints
   - Time array in milliseconds for each point

4. Adjusts timing: ensures first point starts at time 0, and ignores the last
   point if its time interval from the previous point is less than 1 ms
5. Executes via ``StartCSplinePos`` on spline buffer 0
6. Blocks on ``motion->Wait`` until all axes reach their final positions

**Result:**

- On success: ``error_code = 0``
- On failure: ``error_code`` set to the WMX3 error code, goal is aborted

**Cancel:**

- Cancel requests are accepted but the current implementation uses a blocking
  ``Wait`` call, so cancellation during spline execution is not yet fully supported.

.. note::

   The action name is configurable via the ``joint_trajectory_action`` parameter.
   The default value ``/movensys_manipulator_arm_controller/follow_joint_trajectory``
   matches the controller name expected by the MoveIt2 configuration.

WMX3 Hardware Communication
----------------------------

The WMX3 API layer provides real-time communication with servo drives over EtherCAT.
All three active nodes (``manipulator_state``, ``follow_joint_trajectory_server``,
``wmx_ros2_general_node``) create independent WMX3 device handles to the same
shared motion engine.

Device Initialization
^^^^^^^^^^^^^^^^^^^^^^

Each node creates a WMX3 device connection on startup:

.. code-block:: cpp

   wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, timeout);
   wmx3Lib_.SetDeviceName("NodeName");

All nodes use a retry mechanism (5 attempts, 2-second intervals) to handle lock
contention when multiple nodes start simultaneously.

WMX3 API Libraries
^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Library
     - API Class
     - Used By
   * - ``coremotionapi``
     - ``CoreMotion``
     - All nodes -- position/velocity commands, axis status, servo control,
       configuration, homing
   * - ``advancedmotionapi``
     - ``AdvancedMotion``
     - ``follow_joint_trajectory_server`` -- cubic spline trajectory execution
       (``StartCSplinePos``)
   * - ``ioapi``
     - ``Io``
     - ``manipulator_state``, ``follow_joint_trajectory_server`` -- digital I/O
       for gripper control (``SetOutBit``, ``GetOutBit``)
   * - ``ecapi``
     - ``Ecat``
     - ``manipulator_state`` -- EtherCAT network scanning (``ScanNetwork``)
   * - ``wmx3api``
     - ``WMX3Api``
     - All nodes -- device lifecycle (create, close), engine start/stop,
       communication start/stop, error handling
   * - ``imdll``
     - (internal)
     - All nodes -- internal WMX3 dependency

EtherCAT Communication Cycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   manipulator_state (startup sequence):

   CreateDevice("/opt/lmx/")  ──►  ScanNetwork(masterId=0)
                                            │
                                    StartCommunication(10s timeout)
                                            │
                                    ImportAndSetAll(wmx_params.xml)
                                            │
                                    ClearAmpAlarm + SetServoOn (per joint)
                                            │
                                    ┌───────┴───────┐
                                    │  Cyclic Loop  │
                                    │  GetStatus()  │──► Publish /joint_states
                                    └───────────────┘

Once communication is started, the WMX3 engine maintains a real-time cyclic exchange
with all connected servo drives. The ``GetStatus()`` call reads the latest process
data (positions, velocities, torques, status bits) from shared memory.

Gripper I/O
^^^^^^^^^^^^

The gripper is controlled via digital output bit 0 on EtherCAT I/O channel 0:

- **Close gripper:** ``Io::SetOutBit(0, 0, 1)``
- **Open gripper:** ``Io::SetOutBit(0, 0, 0)``
- **Read state:** ``Io::GetOutBit(0, 0, &data)`` -- returns 1 if closed, 0 if open

The ``manipulator_state`` node reads the gripper state each cycle and publishes
simulated gripper finger positions (``gripper_open_value`` / ``gripper_close_value``)
as part of the joint state message.
