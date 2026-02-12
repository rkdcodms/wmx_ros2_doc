wmx_ros2_package
=================

Overview
--------

The ``wmx_ros2_package`` is the main application package of the WMX ROS2
system. It contains all executable nodes, launch files, and configuration
files for operating a 6-DOF manipulator (Dobot CR3A) and an optional
differential drive mobile base through the WMX3 motion control engine.

**Package Metadata:**

.. list-table::
   :widths: 25 75

   * - **Package Name**
     - ``wmx_ros2_package``
   * - **Version**
     - 0.0.0
   * - **Maintainer**
     - mfikih15 (lp02781@gmail.com)
   * - **Build Type**
     - ``ament_cmake``
   * - **C++ Standard**
     - C++17

Package Structure
-----------------

.. code-block:: text

   wmx_ros2_package/
   ├── CMakeLists.txt
   ├── package.xml
   ├── wmx_ros2_package/
   │   └── __init__.py
   ├── include/
   │   └── wmx_general_header/
   │       ├── wmx_ros2_general.hpp          # WmxRos2General class definition
   │       ├── wmx_ros2_general.cpp          # Constructor, destructor, init
   │       ├── wmx_ros2_engine.cpp           # Engine lifecycle methods
   │       └── wmx_ros2_core_motion.cpp      # Motion control & axis methods
   ├── src/
   │   ├── wmx_ros2_general_node.cpp         # General node entry point
   │   ├── manipulator_state.cpp             # Joint state publisher node
   │   ├── follow_joint_trajectory_server.cpp # Trajectory action server node
   │   └── diff_drive_controller.cpp         # Diff drive node (disabled)
   ├── example/
   │   └── wmx_ros2_general_example.cpp      # Full API demo client
   ├── launch/
   │   ├── wmx_ros2_general.launch.py
   │   ├── wmx_ros2_intel_manipulator_cr3a.launch.py
   │   ├── wmx_ros2_orin_manipulator_cr3a.launch.py
   │   └── wmx_ros2_diff_drive_controller.launch.py
   └── config/
       ├── intel_manipulator_config_cr3a.yaml
       ├── orin_manipulator_config_cr3a.yaml
       ├── diff_drive_controller_config.yaml
       ├── cr3a_wmx_parameters.xml           # WMX3 axis params for CR3A
       └── baymax_wmx_parameters.xml         # WMX3 axis params for Baymax

Dependencies
------------

Package Dependencies (package.xml)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 35 20 45

   * - Dependency
     - Type
     - Purpose
   * - ``ament_cmake``
     - buildtool
     - CMake build system
   * - ``rclcpp``
     - build + exec
     - ROS2 C++ client library
   * - ``std_srvs``
     - depend
     - Standard service types (``SetBool``, ``Trigger``)
   * - ``wmx_ros2_message``
     - depend
     - Custom message and service definitions
   * - ``ros2launch``
     - exec
     - Launch system
   * - ``joint_state_publisher``
     - exec
     - Joint state publishing utilities
   * - ``robot_state_publisher``
     - exec
     - Robot TF broadcasting
   * - ``rviz``
     - exec
     - Visualization
   * - ``xacro``
     - exec
     - URDF preprocessing

CMake Dependencies (find_package)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Package
     - Purpose
   * - ``rclcpp``
     - ROS2 C++ client library
   * - ``rclcpp_action``
     - Action server/client support
   * - ``std_msgs``
     - Standard message types (``Float64MultiArray``)
   * - ``std_srvs``
     - Standard service types (``SetBool``, ``Trigger``)
   * - ``geometry_msgs``
     - Twist messages (diff drive)
   * - ``nav_msgs``
     - Odometry messages (diff drive)
   * - ``sensor_msgs``
     - JointState messages
   * - ``control_msgs``
     - FollowJointTrajectory action type
   * - ``trajectory_msgs``
     - JointTrajectory messages
   * - ``wmx_ros2_message``
     - Custom WMX3 interfaces

WMX3 Libraries (External)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

All executables link against the WMX3 shared libraries at ``/opt/lmx/lib/``:

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Library
     - Executable(s)
     - Purpose
   * - ``libcoremotionapi.so``
     - All nodes
     - Position, velocity, axis control
   * - ``libadvancedmotionapi.so``
     - ``follow_joint_trajectory_server``
     - Cubic spline trajectory execution
   * - ``libioapi.so``
     - ``manipulator_state``, ``follow_joint_trajectory_server``
     - Digital I/O (gripper control)
   * - ``libecapi.so``
     - ``manipulator_state``
     - EtherCAT network scanning
   * - ``libwmx3api.so``
     - All nodes
     - Core WMX3 device and engine management
   * - ``libimdll.so``
     - All nodes
     - Internal WMX3 dependency

Nodes
-----

manipulator_state
^^^^^^^^^^^^^^^^^^

The primary joint state publisher. Performs the full hardware initialization
sequence (device creation, EtherCAT scan, communication start, parameter
loading, servo enable) and then publishes encoder feedback at high frequency.

**Source:** ``src/manipulator_state.cpp``

**Class:** ``ManipulatorState`` (inherits ``rclcpp::Node``)

**Node name:** ``manipulator_state``

Parameters
"""""""""""

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``joint_number``
     - ``int``
     - ``0``
     - Number of robot joints (typically 6)
   * - ``joint_feedback_rate``
     - ``int``
     - ``0``
     - Encoder publish rate in Hz (typically 500)
   * - ``gripper_open_value``
     - ``double``
     - ``0.0``
     - Joint position value when gripper is open
   * - ``gripper_close_value``
     - ``double``
     - ``0.0``
     - Joint position value when gripper is closed
   * - ``joint_name``
     - ``string[]``
     - ``["j1"..."j6"]``
     - Joint names for the JointState message
   * - ``encoder_joint_topic``
     - ``string``
     - (required)
     - Topic name for primary joint state output
   * - ``isaacsim_joint_topic``
     - ``string``
     - (optional)
     - Topic name for Isaac Sim joint command output
   * - ``gazebo_joint_topic``
     - ``string``
     - (required)
     - Topic name for Gazebo position controller output
   * - ``wmx_param_file_path``
     - ``string``
     - (required)
     - Absolute path to the WMX3 axis parameter XML file

Published Topics
"""""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 35 30 15 20

   * - Topic
     - Message Type
     - Rate
     - Description
   * - ``encoder_joint_topic`` (param)
     - ``sensor_msgs/msg/JointState``
     - ``joint_feedback_rate`` Hz
     - Joint positions + gripper state
   * - ``isaacsim_joint_topic`` (param)
     - ``sensor_msgs/msg/JointState``
     - ``joint_feedback_rate`` Hz
     - Mirror for Isaac Sim (no timestamp)
   * - ``gazebo_joint_topic`` (param)
     - ``std_msgs/msg/Float64MultiArray``
     - ``joint_feedback_rate`` Hz
     - Joint positions for Gazebo

The JointState message contains ``joint_number`` joint positions from
encoder feedback plus 2 gripper finger positions derived from the EtherCAT
digital I/O bit (``Io::GetOutBit(0, 0)``).

Initialization Sequence
""""""""""""""""""""""""

1. Declare and read all ROS2 parameters
2. ``CreateDevice("/opt/lmx/")`` with retry (5 attempts, 2s interval)
3. ``Ecat::ScanNetwork(masterId=0)`` -- EtherCAT device discovery
4. ``StartCommunication(timeout=10s)`` -- begin real-time cycle
5. ``config->ImportAndSetAll(wmx_param_file_path)`` -- load axis parameters
6. For each joint: ``ClearAmpAlarm()`` then ``SetServoOn()``
7. Start encoder feedback timer

Shutdown Sequence
""""""""""""""""""

1. Disable all servos (``SetServoOn(axis, 0)`` for each joint)
2. Stop EtherCAT communication
3. Close WMX3 device
4. Wait 3 seconds for cleanup

follow_joint_trajectory_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Action server for executing MoveIt2-planned trajectories on the physical
robot using WMX3 cubic spline interpolation.

**Source:** ``src/follow_joint_trajectory_server.cpp``

**Class:** ``FollowJointTrajectoryServer`` (inherits ``rclcpp::Node``)

**Node name:** ``follow_joint_trajectory_server``

Parameters
"""""""""""

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``joint_number``
     - ``int``
     - ``0``
     - Number of robot joints
   * - ``joint_trajectory_action``
     - ``string``
     - (required)
     - Action server name for FollowJointTrajectory
   * - ``wmx_gripper_topic``
     - ``string``
     - (required)
     - Service name for gripper open/close

Action Server
""""""""""""""

.. list-table::
   :widths: 25 75

   * - **Action Name**
     - From ``joint_trajectory_action`` parameter
   * - **Action Type**
     - ``control_msgs/action/FollowJointTrajectory``

The server processes trajectories as follows:

- Validates point count (max 1000 points)
- Adjusts timing (first point time = 0, removes points <1 ms apart)
- Builds a WMX3 ``CSplinePosData`` structure for all joints
- Executes via ``AdvancedMotion::StartCSplinePos()``
- Blocks on ``AdvancedMotion::Wait()`` until complete
- Returns ``error_code = 0`` on success, or WMX3 error code on failure

A spline buffer of 1000 points is allocated in the constructor via
``AdvancedMotion::CreateSplineBuffer(0, 1000)`` and freed in the destructor.

Service Server
"""""""""""""""

.. list-table::
   :widths: 25 75

   * - **Service Name**
     - From ``wmx_gripper_topic`` parameter
   * - **Service Type**
     - ``std_srvs/srv/SetBool``

Controls the pneumatic gripper via ``Io::SetOutBit(0, 0, value)``.
``true`` = close (bit=1), ``false`` = open (bit=0).

Initialization
"""""""""""""""

1. ``CreateDevice("/opt/lmx/")`` with retry (5 attempts, 2s interval)
2. Initialize CoreMotion and AdvancedMotion APIs
3. ``CreateSplineBuffer(0, 1000)`` -- allocate trajectory buffer
4. Create action server and gripper service

wmx_ros2_general_node
^^^^^^^^^^^^^^^^^^^^^^

Standalone WMX3 engine and axis control node. Provides service-based access
to all WMX3 engine lifecycle and axis control operations, plus topic-based
motion commands.

**Source:** ``src/wmx_ros2_general_node.cpp`` (entry point),
``include/wmx_general_header/wmx_ros2_general.hpp`` (class definition),
``include/wmx_general_header/wmx_ros2_general.cpp`` (constructor),
``include/wmx_general_header/wmx_ros2_engine.cpp`` (engine methods),
``include/wmx_general_header/wmx_ros2_core_motion.cpp`` (motion methods)

**Class:** ``WmxRos2General`` (inherits ``rclcpp::Node``)

**Node name:** ``wmx_ros2_general_node``

**Internal axis count:** 2 (hardcoded in ``wmx_ros2_general.hpp``)

Parameters
"""""""""""

This node does not declare any ROS2 parameters. All interface names are
hardcoded in the class member initializers.

Services
"""""""""

**Engine Management:**

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Service
     - Type
     - Description
   * - ``/wmx/engine/set_device``
     - ``wmx_ros2_message/srv/SetEngine``
     - Create or close the WMX3 device handle
   * - ``/wmx/engine/set_comm``
     - ``std_srvs/srv/SetBool``
     - Start or stop EtherCAT communication
   * - ``/wmx/engine/get_status``
     - ``std_srvs/srv/Trigger``
     - Query engine state (Idle/Running/Communicating/Shutdown)

**Axis Control:**

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Service
     - Type
     - Description
   * - ``/wmx/axis/set_on``
     - ``wmx_ros2_message/srv/SetAxis``
     - Enable/disable servo drives
   * - ``/wmx/axis/clear_alarm``
     - ``wmx_ros2_message/srv/SetAxis``
     - Clear amplifier faults
   * - ``/wmx/axis/set_mode``
     - ``wmx_ros2_message/srv/SetAxis``
     - Set position (0) or velocity (1) mode
   * - ``/wmx/axis/set_polarity``
     - ``wmx_ros2_message/srv/SetAxis``
     - Set rotation direction (1 or -1)
   * - ``/wmx/axis/set_gear_ratio``
     - ``wmx_ros2_message/srv/SetAxisGearRatio``
     - Configure encoder gear ratio
   * - ``/wmx/axis/homing``
     - ``wmx_ros2_message/srv/SetAxis``
     - Set current position as home (zero)

Published Topics
"""""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 25 35 15 25

   * - Topic
     - Message Type
     - Rate
     - Description
   * - ``/wmx/axis/state``
     - ``wmx_ros2_message/msg/AxisState``
     - 100 Hz
     - Full axis status (12 fields)

Subscribed Topics
""""""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Topic
     - Message Type
     - Description
   * - ``/wmx/axis/velocity``
     - ``wmx_ros2_message/msg/AxisVelocity``
     - Velocity commands → ``CoreMotion::StartVel()``
   * - ``/wmx/axis/position``
     - ``wmx_ros2_message/msg/AxisPose``
     - Absolute position → ``CoreMotion::StartPos()``
   * - ``/wmx/axis/position/relative``
     - ``wmx_ros2_message/msg/AxisPose``
     - Relative position → ``CoreMotion::StartMov()``

Initialization
"""""""""""""""

1. ``CreateDevice("/opt/lmx/")`` with retry (5 attempts, 2s interval)
2. Create all service servers, publishers, and subscribers (in class member
   initializers)
3. Start 100 Hz timer for ``axisStateStep()``

diff_drive_controller (Disabled)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Differential drive controller for mobile base applications. Converts
``geometry_msgs/Twist`` commands into individual wheel velocity commands via
the WMX3 engine.

.. note::

   This node is **commented out** in ``CMakeLists.txt`` and is not built
   by default. It is documented here for reference.

**Source:** ``src/diff_drive_controller.cpp``

**Class:** ``DiffDriveController`` (inherits ``rclcpp::Node``)

**Node name:** ``diff_drive_controller``

Parameters
"""""""""""

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``left_axis``
     - ``int``
     - ``0``
     - WMX3 axis index for left wheel
   * - ``right_axis``
     - ``int``
     - ``1``
     - WMX3 axis index for right wheel
   * - ``rate``
     - ``int``
     - ``10``
     - Control loop rate in Hz
   * - ``acc_time``
     - ``double``
     - ``1.0``
     - Acceleration time (seconds)
   * - ``dec_time``
     - ``double``
     - ``1.0``
     - Deceleration time (seconds)
   * - ``wheel_radius``
     - ``double``
     - ``0.09``
     - Wheel radius in meters
   * - ``wheel_to_wheel``
     - ``double``
     - ``0.55``
     - Distance between wheels in meters
   * - ``cmd_vel_topic``
     - ``string``
     - (required)
     - Input velocity command topic
   * - ``encoder_vel_topic``
     - ``string``
     - (required)
     - Output velocity echo topic
   * - ``encoder_omega_topic``
     - ``string``
     - (required)
     - Output wheel angular velocity topic
   * - ``encoder_odometry_topic``
     - ``string``
     - (required)
     - Output odometry topic
   * - ``wmx_param_file_path``
     - ``string``
     - (required)
     - Path to WMX3 parameter XML file

Subscribed Topics
""""""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Topic
     - Message Type
     - Description
   * - ``cmd_vel_topic`` (param)
     - ``geometry_msgs/msg/Twist``
     - Linear/angular velocity commands

Published Topics
"""""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 35 30 35

   * - Topic
     - Message Type
     - Description
   * - ``encoder_vel_topic`` (param)
     - ``geometry_msgs/msg/Twist``
     - Echo of received cmd_vel
   * - ``encoder_omega_topic`` (param)
     - ``std_msgs/msg/Float64MultiArray``
     - Wheel angular velocities
   * - ``encoder_odometry_topic`` (param)
     - ``nav_msgs/msg/Odometry``
     - Encoder-based odometry

Timers
"""""""

Three timers run at the configured ``rate`` (Hz):

- ``cmdVelStep()`` -- Convert ``Twist`` to wheel velocities and send to WMX3.
  Includes safety checks: verifies engine is communicating, no amplifier
  alarms, and servos are enabled before sending commands.
- ``encoderOmegaStep()`` -- Read actual wheel velocities from WMX3 and
  publish as ``Float64MultiArray``
- ``encoderOdometryStep()`` -- Compute and publish odometry from wheel
  encoder feedback

Example Application
--------------------

wmx_ros2_general_example
^^^^^^^^^^^^^^^^^^^^^^^^^

A standalone ROS2 client that demonstrates the full WMX3 control workflow
through the ``wmx_ros2_general_node`` services and topics.

**Source:** ``example/wmx_ros2_general_example.cpp``

**Node name:** ``wmx_ros2_general_example``

**This is NOT a long-running node** -- it executes a scripted demo sequence
and exits.

Execution Sequence
"""""""""""""""""""

**Phase 1 -- Setup:**

1. Create device: ``/wmx/engine/set_device`` (path="/opt/lmx/")
2. Start communication: ``/wmx/engine/set_comm`` (data=true)
3. Set gear ratios: ``/wmx/axis/set_gear_ratio`` (8388608.0 / 6.28319)
4. Set polarities: ``/wmx/axis/set_polarity`` (axis 0 = 1, axis 1 = -1)
5. Clear alarms: ``/wmx/axis/clear_alarm``
6. Set velocity mode: ``/wmx/axis/set_mode`` (data=[1, 1])
7. Enable servos: ``/wmx/axis/set_on`` (data=[1, 1])
8. Home axes: ``/wmx/axis/homing``

**Phase 2 -- Velocity motion test:**

9. Publish 1.0 rad/s velocity for 10 seconds
10. Stop (0 rad/s) for 5 seconds

**Phase 3 -- Position mode switch:**

11. Disable servos, switch to position mode, re-enable, re-home

**Phase 4 -- Absolute position test:**

12. Move to 5.0 rad, wait 10 seconds
13. Move to -2.0 rad, wait 10 seconds

**Phase 5 -- Relative position test:**

14. Home axes
15. Move +5.0 rad relative, wait 10 seconds
16. Move -2.0 rad relative, wait 10 seconds

**Phase 6 -- Shutdown:**

17. Disable servos
18. Stop communication
19. Close device

Usage
""""""

.. code-block:: bash

   # Ensure wmx_ros2_general_node is running first
   ros2 run wmx_ros2_package wmx_ros2_general_example

.. warning::

   This example moves real motors. Ensure the robot workspace is clear.

Launch Files
------------

wmx_ros2_general.launch.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Starts only the ``wmx_ros2_general_node`` for standalone axis control
without MoveIt2 integration.

**Nodes launched:** ``wmx_ros2_general_node``

**Arguments:** None

**Config loaded:** None (node uses hardcoded defaults)

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
        bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
                 source ~/wmx_ros2_ws/install/setup.bash && \
                 ros2 launch wmx_ros2_package wmx_ros2_general.launch.py"

wmx_ros2_intel_manipulator_cr3a.launch.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Full manipulator launch for Intel x86_64 platforms.

**Nodes launched:**

1. ``manipulator_state``
2. ``follow_joint_trajectory_server``
3. ``wmx_ros2_general_node``

**Arguments:**

.. list-table::
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``use_sim_time``
     - ``false``
     - Use simulated clock source

**Config loaded:** ``config/intel_manipulator_config_cr3a.yaml``

All three nodes receive parameters from the same YAML file plus the
``use_sim_time`` argument.

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
        bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
                 source ~/wmx_ros2_ws/install/setup.bash && \
                 ros2 launch wmx_ros2_package \
                   wmx_ros2_intel_manipulator_cr3a.launch.py \
                   use_sim_time:=false"

wmx_ros2_orin_manipulator_cr3a.launch.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Full manipulator launch for NVIDIA Jetson Orin platforms.

**Nodes launched:**

1. ``manipulator_state``
2. ``follow_joint_trajectory_server``
3. ``wmx_ros2_general_node``

**Arguments:**

.. list-table::
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``use_sim_time``
     - ``false``
     - Use simulated clock source

**Config loaded:** ``config/orin_manipulator_config_cr3a.yaml``

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
        bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
                 source ~/wmx_ros2_ws/install/setup.bash && \
                 ros2 launch wmx_ros2_package \
                   wmx_ros2_orin_manipulator_cr3a.launch.py \
                   use_sim_time:=false"

wmx_ros2_diff_drive_controller.launch.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

   The ``diff_drive_controller`` executable is commented out in
   ``CMakeLists.txt``. This launch file will fail unless the build
   is modified to include it.

**Nodes launched:** ``diff_drive_controller``

**Arguments:** None

**Config loaded:** ``config/diff_drive_controller_config.yaml``

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
        bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
                 source ~/wmx_ros2_ws/install/setup.bash && \
                 ros2 launch wmx_ros2_package \
                   wmx_ros2_diff_drive_controller.launch.py"

Configuration Files
-------------------

intel_manipulator_config_cr3a.yaml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameters for the Intel x86_64 manipulator deployment.

.. code-block:: yaml

   manipulator_state:
     ros__parameters:
       joint_number: 6
       joint_feedback_rate: 500
       gripper_open_value: 0.00
       gripper_close_value: 0.045
       joint_name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                     "picker_1_joint", "picker_2_joint"]
       encoder_joint_topic: /joint_states
       gazebo_joint_topic: /gazebo_position_controller/commands
       wmx_param_file_path: /home/<user>/wmx_ros2_ws/src/wmx_ros2_application/
                            wmx_ros2_package/config/cr3a_wmx_parameters.xml

   follow_joint_trajectory_server:
     ros__parameters:
       joint_number: 6
       wmx_gripper_topic: /wmx/set_gripper
       joint_trajectory_action: /movensys_manipulator_arm_controller/
                                follow_joint_trajectory

.. important::

   Update ``wmx_param_file_path`` to match your actual home directory.
   The default references the machine username ``mvsk``.

orin_manipulator_config_cr3a.yaml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameters for the NVIDIA Jetson Orin manipulator deployment. Identical to
the Intel config with two differences:

- Adds ``isaacsim_joint_topic: /isaacsim/joint_command`` for Isaac Sim
  integration
- The ``wmx_param_file_path`` references the Orin machine path (username
  ``mic-733ao``)

.. code-block:: yaml

   manipulator_state:
     ros__parameters:
       joint_number: 6
       joint_feedback_rate: 500
       gripper_open_value: 0.00
       gripper_close_value: 0.045
       joint_name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                     "picker_1_joint", "picker_2_joint"]
       encoder_joint_topic: /joint_states
       isaacsim_joint_topic: /isaacsim/joint_command
       wmx_param_file_path: /home/<user>/wmx_ros2_ws/src/wmx_ros2_application/
                            wmx_ros2_package/config/cr3a_wmx_parameters.xml

   follow_joint_trajectory_server:
     ros__parameters:
       joint_number: 6
       wmx_gripper_topic: /wmx/set_gripper
       joint_trajectory_action: /movensys_manipulator_arm_controller/
                                follow_joint_trajectory

diff_drive_controller_config.yaml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameters for the differential drive mobile base controller.

.. code-block:: yaml

   diff_drive_controller:
     ros__parameters:
       left_axis: 0
       right_axis: 1
       rate: 100
       acc_time: 1.0
       dec_time: 1.0
       wheel_radius: 0.09
       wheel_to_wheel: 0.55
       cmd_vel_topic: /cmd_vel
       encoder_vel_topic: /cmd_vel_check
       encoder_omega_topic: /velocity_controller/commands
       encoder_odometry_topic: /odom_enc
       wmx_param_file_path: /home/<user>/wmx_ros2_ws/src/wmx_ros2_application/
                            wmx_ros2_package/config/baymax_wmx_parameters.xml

WMX3 Parameter XML Files
^^^^^^^^^^^^^^^^^^^^^^^^^^

Two robot-specific WMX3 parameter files define per-axis motor configuration:

- ``cr3a_wmx_parameters.xml`` -- Parameters for the Dobot CR3A manipulator
  (gear ratios, polarities, limits, home positions for 6 joints)
- ``baymax_wmx_parameters.xml`` -- Parameters for the Baymax mobile base
  (gear ratios, limits for 2 wheel axes)

These files are loaded at runtime by the ``manipulator_state`` node via
``CoreMotion::config->ImportAndSetAll(path)``.

Building the Package
--------------------

This package depends on ``wmx_ros2_message`` and must be built after it:

.. code-block:: bash

   cd ~/wmx_ros2_ws

   # Stage 1: Build message package
   colcon build --packages-select wmx_ros2_message
   source install/setup.bash

   # Stage 2: Build application package
   colcon build --packages-select wmx_ros2_package
   source install/setup.bash

Verify executables are available:

.. code-block:: bash

   ros2 pkg executables wmx_ros2_package

Expected:

.. code-block:: text

   wmx_ros2_package follow_joint_trajectory_server
   wmx_ros2_package manipulator_state
   wmx_ros2_package wmx_ros2_general_example
   wmx_ros2_package wmx_ros2_general_node
