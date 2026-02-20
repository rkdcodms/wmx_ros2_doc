System Overview
===============

The WMX ROS2 application provides a robot-agnostic ROS2 interface for controlling 6-DOF
robotic manipulators through the WMX3 motion control platform. The architecture bridges
MoveIt2 motion planning with real-time servo control via EtherCAT fieldbus communication.

Currently, the system supports the **Dobot CR series** (e.g., CR3A), but the layered
design allows adaptation to any 6-DOF robot by modifying configuration files and
WMX3 parameter mappings.

Architecture Diagram
--------------------

.. image:: /_static/images/architecture_diagram.png
   :alt: WMX ROS2 System Architecture - showing ROS2 Application Layer, WMX ROS2 Package with API modules, WMX3 Motion Engine, EtherCAT Master, and Hardware Layer
   :width: 100%
   :align: center

Package Overview
----------------

The system consists of two ROS2 packages. See :doc:`../packages/packages`
for complete per-package documentation.

wmx_ros2_message
^^^^^^^^^^^^^^^^^

A C++ (ament_cmake) interface definition package containing custom ROS2 message and
service types for motor control communication.
See :doc:`../packages/wmx_ros2_message` for full details.

.. list-table:: Custom Messages
   :header-rows: 1
   :widths: 25 75

   * - Message
     - Description
   * - ``AxisPose.msg``
     - Multi-axis absolute/relative position command with motion profile parameters
       (index, target, profile, velocity, acceleration, deceleration)
   * - ``AxisState.msg``
     - Comprehensive axis status feedback including alarm state, servo status,
       homing status, limit switches, commanded/actual position, velocity, and torque
   * - ``AxisVelocity.msg``
     - Multi-axis velocity command with motion profile parameters
       (index, profile, velocity, acceleration, deceleration)

.. list-table:: Custom Services
   :header-rows: 1
   :widths: 25 75

   * - Service
     - Description
   * - ``SetEngine.srv``
     - Create or close a WMX3 device with specified path and device name
   * - ``SetAxis.srv``
     - Generic axis control (servo on/off, mode, polarity, homing, clear alarm)
       using axis index and data arrays
   * - ``SetAxisGearRatio.srv``
     - Set encoder gear ratio (numerator/denominator) per axis

wmx_ros2_package
^^^^^^^^^^^^^^^^^

The main C++ (ament_cmake) application package containing all ROS2 nodes, launch files,
and configuration. It depends on ``wmx_ros2_message`` for custom interface types and
links against the WMX3 motion control libraries at ``/opt/lmx/``.
See :doc:`../packages/wmx_ros2_package` for full details.

.. list-table:: Node Executables
   :header-rows: 1
   :widths: 30 70

   * - Executable
     - Description
   * - ``manipulator_state``
     - Reads joint encoder positions via WMX3 CoreMotion API and publishes
       ``sensor_msgs/JointState`` for MoveIt2, Isaac Sim, and Gazebo. Handles
       full hardware initialization (EtherCAT scan, communication start,
       parameter loading, servo enable).
   * - ``follow_joint_trajectory_server``
     - Implements the ``FollowJointTrajectory`` action server that MoveIt2
       uses for trajectory execution. Converts trajectory points into WMX3
       cubic spline commands (``StartCSplinePos``). Also provides a gripper
       control service via digital I/O.
   * - ``wmx_ros2_general_node``
     - Provides low-level WMX3 engine management and axis control through
       ROS2 services and topics. Publishes periodic axis state at 100 Hz.
       Accepts velocity and position commands via topic subscriptions.
   * - ``wmx_ros2_general_example``
     - Client example demonstrating the full workflow: engine start,
       communication, gear ratio setup, servo enable, homing, velocity/position
       motion commands, and shutdown.
   * - ``diff_drive_controller``
     - Differential drive controller for mobile base applications (currently
       disabled in build). Converts ``cmd_vel`` twist commands to individual
       wheel velocities.

Node Roles
----------

manipulator_state
^^^^^^^^^^^^^^^^^^

This node is the **hardware initialization master** for manipulator applications. On
startup it performs the complete initialization sequence:

1. Creates a WMX3 device at ``/opt/lmx/``
2. Scans the EtherCAT network for connected servo drives
3. Starts real-time EtherCAT communication
4. Loads robot-specific WMX3 parameters (gear ratios, polarities, encoder modes)
5. Clears servo alarms and enables servo drives for all joints
6. Begins periodic encoder feedback publishing

It publishes joint states to three configurable topics simultaneously, enabling
real hardware feedback to reach MoveIt2 (via ``/joint_states``), Isaac Sim, and
Gazebo concurrently. Gripper state is read via digital I/O and appended to joint
state messages.

follow_joint_trajectory_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This node bridges MoveIt2 trajectory planning with real-time servo execution. When
MoveIt2 (or any planner) sends a ``FollowJointTrajectory`` action goal:

1. The trajectory point list is validated (max 1000 points)
2. Joint positions and timing are extracted from trajectory points
3. A WMX3 cubic spline buffer is constructed (``PointTimeSplineCommand``)
4. The spline is executed via ``StartCSplinePos`` for smooth, interpolated motion
5. The node waits for motion completion and reports success/failure

It also provides a gripper open/close service through WMX3 digital I/O.

wmx_ros2_general_node
^^^^^^^^^^^^^^^^^^^^^^

This node provides **standalone axis control** independent of MoveIt2. It exposes
ROS2 services for engine lifecycle management (create device, start/stop communication)
and individual axis operations (servo on/off, mode selection, polarity, gear ratio,
homing, alarm clearing). It also accepts motion commands via topic subscriptions
for direct velocity and position control.

Communication Protocols
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Protocol
     - Layer
     - Description
   * - ROS2 DDS
     - ROS2 Interface Layer
     - All inter-node communication uses ROS2 topics, services, and actions
       over DDS (Data Distribution Service). Configured middleware is
       CycloneDDS (``rmw_cyclonedds_cpp``).
   * - EtherCAT
     - Hardware Layer
     - Real-time fieldbus communication between WMX3 and servo drives.
       Managed by the WMX3 engine via ``WMX3Api`` and ``EcApi``. Provides
       deterministic, low-latency servo control.
   * - WMX3 API
     - Interface to Hardware
     - C++ shared libraries (``coremotionapi``, ``advancedmotionapi``,
       ``wmx3api``, ``ioapi``, ``imdll``) installed at ``/opt/lmx/``.
       Provides motion commands, I/O control, and EtherCAT management.

.. note::

   The system does **not** use TCP/IP to communicate with the robot controller directly.
   Instead, the WMX3 motion control platform manages real-time communication over
   EtherCAT. The ROS2 nodes interface with the WMX3 API through shared memory and
   function calls on the same machine.

Configuration
-------------

Robot-specific behavior is controlled through YAML configuration files and WMX3 XML
parameter files:

.. list-table:: Configuration Files
   :header-rows: 1
   :widths: 35 65

   * - File
     - Purpose
   * - ``intel_manipulator_config_cr3a.yaml``
     - ROS2 parameters for Intel-based platform running CR3A manipulator
   * - ``orin_manipulator_config_cr3a.yaml``
     - ROS2 parameters for NVIDIA Jetson Orin platform running CR3A manipulator
   * - ``cr3a_wmx_parameters.xml``
     - WMX3 axis configuration (gear ratios, polarities, encoder modes) for Dobot CR3A
   * - ``diff_drive_controller_config.yaml``
     - Differential drive parameters (wheel geometry, axis mapping, topics)

To support a different 6-DOF robot, create new YAML and WMX3 XML parameter files
with the appropriate joint count, gear ratios, encoder settings, and topic names.
