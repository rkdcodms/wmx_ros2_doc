Flowcharts
==========

Launch Sequence
---------------

Manipulator Launch (Intel / Orin)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``wmx_ros2_intel_manipulator_cr3a.launch.py`` and
``wmx_ros2_orin_manipulator_cr3a.launch.py`` launch files start three nodes
simultaneously with platform-specific configuration.

.. code-block:: text

   wmx_ros2_intel_manipulator_cr3a.launch.py
   (or wmx_ros2_orin_manipulator_cr3a.launch.py)
       │
       ├── Load config: intel_manipulator_config_cr3a.yaml
       │   (or orin_manipulator_config_cr3a.yaml)
       │
       ├──► [Node 1] manipulator_state
       │        │
       │        ├── Declare & read ROS2 parameters
       │        │     (joint_number, feedback_rate, joint_names, topics, wmx_param_path)
       │        │
       │        ├── CreateDevice("/opt/lmx/") with retry (5x, 2s interval)
       │        ├── ScanNetwork(masterId=0)          ← EtherCAT discovery
       │        ├── StartCommunication(10s timeout)  ← Begin real-time cycle
       │        ├── ImportAndSetAll(wmx_params.xml)  ← Load axis configuration
       │        │
       │        ├── For each joint (0..5):
       │        │     ├── ClearAmpAlarm(joint)
       │        │     └── SetServoOn(joint)
       │        │
       │        └── Start timer: encoderJointStep() @ joint_feedback_rate Hz
       │              └── GetStatus() → Publish /joint_states
       │
       ├──► [Node 2] follow_joint_trajectory_server
       │        │
       │        ├── Declare & read ROS2 parameters
       │        │     (joint_number, joint_trajectory_action, wmx_gripper_topic)
       │        │
       │        ├── CreateDevice("/opt/lmx/") with retry (5x, 2s interval)
       │        ├── Initialize CoreMotion, AdvancedMotion, Io APIs
       │        ├── CreateSplineBuffer(0, 1000)     ← Allocate trajectory buffer
       │        │
       │        ├── Create action server: FollowJointTrajectory
       │        └── Create service server: /wmx/set_gripper
       │
       └──► [Node 3] wmx_ros2_general_node
                │
                ├── CreateDevice("/opt/lmx/") with retry (5x, 2s interval)
                │
                ├── Create service servers:
                │     /wmx/engine/set_device
                │     /wmx/engine/set_comm
                │     /wmx/engine/get_status
                │     /wmx/axis/set_on
                │     /wmx/axis/clear_alarm
                │     /wmx/axis/set_mode
                │     /wmx/axis/set_polarity
                │     /wmx/axis/set_gear_ratio
                │     /wmx/axis/homing
                │
                ├── Create publisher:  /wmx/axis/state
                ├── Create subscribers: /wmx/axis/velocity
                │                       /wmx/axis/position
                │                       /wmx/axis/position/relative
                │
                └── Start timer: axisStateStep() @ 100 Hz
                      └── GetStatus() → Publish /wmx/axis/state

General Launch
^^^^^^^^^^^^^^^

The ``wmx_ros2_general.launch.py`` starts only the general node for standalone
axis control without MoveIt2 integration:

.. code-block:: text

   wmx_ros2_general.launch.py
       │
       └──► [Node] wmx_ros2_general_node
                └── (same initialization as above)

Diff Drive Launch
^^^^^^^^^^^^^^^^^^

The ``wmx_ros2_diff_drive_controller.launch.py`` starts the differential drive
controller for mobile base applications:

.. code-block:: text

   wmx_ros2_diff_drive_controller.launch.py
       │
       ├── Load config: diff_drive_controller_config.yaml
       │
       └──► [Node] diff_drive_controller
                │
                ├── Declare & read ROS2 parameters
                │     (left_axis, right_axis, rate, wheel params, topics)
                │
                ├── CreateDevice("/opt/lmx/")
                ├── StartCommunication()
                ├── ImportAndSetAll(wmx_params.xml)
                ├── ClearAlarm + SetServoOn (left & right axes)
                │
                ├── Subscribe: /cmd_vel (geometry_msgs/Twist)
                ├── Publish:   /cmd_vel_check, /velocity_controller/commands, /odom_enc
                │
                └── Start timers @ configured rate:
                      ├── cmdVelStep()          ← Convert cmd_vel to wheel velocities
                      ├── encoderOmegaStep()    ← Read & publish wheel angular velocities
                      └── encoderOdometryStep() ← Compute & publish odometry

Motion Execution Flow
---------------------

MoveIt2 Trajectory Execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This flow shows the complete path from a MoveIt2 plan request to physical robot
motion:

.. image:: /_static/images/moveIt2_trajectory_flow.png
   :alt: MoveIt2 Trajectory Execution
   :width: 100%
   :align: center 


Direct Axis Control Flow
^^^^^^^^^^^^^^^^^^^^^^^^^

For standalone operation via ``wmx_ros2_general_node`` without MoveIt2:

.. code-block:: text

   ┌────────────────────────────┐
   │  Client Application        │
   │  (e.g., general_example)   │
   └─────────────┬──────────────┘
                 │
    ┌────────────┼─────────────────────────────────────────┐
    │            │          Setup Phase                     │
    │            ▼                                          │
    │  /wmx/engine/set_device  (create WMX3 device)        │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/engine/set_comm    (start EtherCAT comm)       │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/axis/set_gear_ratio (configure encoders)       │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/axis/set_polarity   (set rotation direction)   │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/axis/clear_alarm    (clear servo faults)       │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/axis/set_mode       (position or velocity)     │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/axis/set_on         (enable servo drives)      │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/axis/homing         (set home position)        │
    └────────────┼─────────────────────────────────────────┘
                 │
    ┌────────────┼─────────────────────────────────────────┐
    │            │          Motion Phase                    │
    │            ▼                                          │
    │  Publish /wmx/axis/velocity   (velocity commands)    │
    │       or /wmx/axis/position   (absolute position)    │
    │       or /wmx/axis/position/relative (relative pos)  │
    │            │                                          │
    │            ▼                                          │
    │  Subscribe /wmx/axis/state    (monitor feedback)     │
    └────────────┼─────────────────────────────────────────┘
                 │
    ┌────────────┼─────────────────────────────────────────┐
    │            │          Shutdown Phase                  │
    │            ▼                                          │
    │  /wmx/axis/set_on (data=[0])  (disable servos)       │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/engine/set_comm (false) (stop communication)   │
    │            │                                          │
    │            ▼                                          │
    │  /wmx/engine/set_device (false) (close device)       │
    └──────────────────────────────────────────────────────┘

Error Handling Flow
-------------------

WMX3 Device Creation Errors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All nodes implement a retry mechanism for device creation to handle race conditions
when multiple nodes start simultaneously:

.. code-block:: text

   CreateDevice("/opt/lmx/")
        │
        ├── Success (ErrorCode::None)
        │     └── Continue initialization
        │
        └── Failure
              │
              ├── Error 297 (CreateDeviceLockError)
              │     └── Log warning: "Device lock error"
              │         Wait 2 seconds → Retry (up to 5 attempts)
              │
              └── Other error
                    └── Log warning with error code & description
                        Wait 2 seconds → Retry (up to 5 attempts)
                              │
                              └── All retries exhausted
                                    └── Log ERROR: "Failed to create device
                                        after N attempts"

Trajectory Execution Errors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   FollowJointTrajectory Goal Received
        │
        ├── Point count > 1000?
        │     └── YES → Log warning → goal_handle->abort() → Return
        │
        ├── Point count == 0 (after timing adjustment)?
        │     └── YES → Log info: "Already at target" → goal_handle->succeed()
        │
        ├── StartCSplinePos() error?
        │     └── YES → Log ERROR with WMX3 error string
        │               Set result.error_code = wmx3_error
        │               goal_handle->abort() → Return
        │
        ├── motion->Wait() error?
        │     └── YES → Log ERROR with WMX3 error string
        │               Set result.error_code = wmx3_error
        │               goal_handle->abort() → Return
        │
        └── All steps succeed
              └── result.error_code = 0
                  goal_handle->succeed() → Return

Communication Errors
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   StartCommunication(timeout) / StopCommunication(timeout)
        │
        ├── Success (ErrorCode::None)
        │     └── Log INFO: "Communication started/stopped"
        │         Response: success=true
        │
        └── Failure
              └── ErrorToString(err) → human-readable message
                  Log ERROR with error code & description
                  Response: success=false, message=error_description

Servo and Axis Control Errors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All axis control operations (servo on/off, mode set, polarity, homing, alarm clear)
follow a consistent per-axis error handling pattern:

.. code-block:: text

   For each axis in request:
        │
        ├── Operation succeeds
        │     └── Log INFO: "Operation on axis N"
        │         Append to message stream
        │
        └── Operation fails
              └── ErrorToString(err)
                  Log ERROR: "Failed operation on axis N. Error=code (description)"
                  Set all_success = false
                  Append to message stream

   Response:
     success = all_success (true only if ALL axes succeeded)
     message = concatenated results for each axis

Diff Drive Controller Safety
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The differential drive controller implements a safety check hierarchy before
sending velocity commands:

.. code-block:: text

   cmdVelStep() timer callback
        │
        ├── Engine state != Communicating?
        │     └── Log WARN: "Communication or engine off"
        │         Skip command → Wait 1 second
        │
        ├── Any amplifier alarm active?
        │     └── Log WARN: "Servo alarm on. Clear alarm"
        │         Skip command → Wait 1 second
        │
        ├── Any servo not enabled?
        │     └── Log WARN: "Servo off. Enable servo"
        │         Skip command → Wait 1 second
        │
        └── All checks pass
              └── Calculate wheel velocities from cmd_vel
                  SetVelocity(left_axis, omega_left)
                  SetVelocity(right_axis, omega_right)
