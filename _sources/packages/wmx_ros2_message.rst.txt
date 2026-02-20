wmx_ros2_message
=================

Overview
--------

The ``wmx_ros2_message`` package defines custom ROS2 message and service
interfaces used by all nodes in the WMX ROS2 application. It contains no
executable nodes -- only interface definitions that are compiled into C++
and Python bindings by ``rosidl``.

**Package Metadata:**

.. list-table::
   :widths: 25 75

   * - **Package Name**
     - ``wmx_ros2_message``
   * - **Version**
     - 0.0.0
   * - **Maintainer**
     - mfikih15 (lp02781@gmail.com)
   * - **Build Type**
     - ``ament_cmake``
   * - **C++ Standard**
     - C++17
   * - **C Standard**
     - C99

Package Structure
-----------------

.. code-block:: text

   wmx_ros2_message/
   ├── CMakeLists.txt
   ├── package.xml
   ├── msg/
   │   ├── AxisPose.msg
   │   ├── AxisState.msg
   │   └── AxisVelocity.msg
   └── srv/
       ├── SetAxis.srv
       ├── SetAxisGearRatio.srv
       └── SetEngine.srv

Dependencies
------------

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Dependency
     - Type
     - Purpose
   * - ``ament_cmake``
     - buildtool
     - CMake build system
   * - ``rclcpp``
     - build + exec
     - ROS2 C++ client library
   * - ``rosidl_default_generators``
     - build
     - Message/service code generation
   * - ``rosidl_default_runtime``
     - exec
     - Runtime message type support
   * - ``ament_lint_auto``
     - test
     - Automated linting
   * - ``ament_lint_common``
     - test
     - Common lint rules

The package is a member of the ``rosidl_interface_packages`` group, which
allows other packages to discover its generated interfaces at build time.

Message Definitions
-------------------

AxisPose
^^^^^^^^^

Used for absolute and relative position motion commands. Published to
``/wmx/axis/position`` and ``/wmx/axis/position/relative``.

.. code-block:: text

   int32[]   index       # Axis indices to command
   float64[] target      # Target positions (radians)
   string    profile     # Motion profile type (reserved for future use)
   float64[] velocity    # Motion velocity per axis (rad/s)
   float64[] acc         # Acceleration per axis (rad/s²)
   float64[] dec         # Deceleration per axis (rad/s²)

All array fields are parallel -- element ``i`` in each array corresponds to
the same axis. The ``profile`` field is currently unused by the subscriber
nodes.

**Used by:**

- ``wmx_ros2_general_node`` -- subscribes on ``/wmx/axis/position`` and
  ``/wmx/axis/position/relative``
- ``wmx_ros2_general_example`` -- publishes position and relative position
  commands

AxisVelocity
^^^^^^^^^^^^^

Used for continuous velocity motion commands. Published to
``/wmx/axis/velocity``.

.. code-block:: text

   int32[]   index       # Axis indices to command
   string    profile     # Motion profile type (reserved for future use)
   float64[] velocity    # Target velocity per axis (rad/s)
   float64[] acc         # Acceleration per axis (rad/s²)
   float64[] dec         # Deceleration per axis (rad/s²)

**Used by:**

- ``wmx_ros2_general_node`` -- subscribes on ``/wmx/axis/velocity``
- ``wmx_ros2_general_example`` -- publishes velocity commands

AxisState
^^^^^^^^^^

Comprehensive per-axis status feedback published at 100 Hz by the
``wmx_ros2_general_node`` on ``/wmx/axis/state``.

.. code-block:: text

   # Status flags (per axis)
   int32[]   amp_alarm        # Amplifier alarm active (1 = fault)
   int32[]   servo_on         # Servo enabled (1 = on)
   int32[]   home_done        # Homing completed (1 = done)
   int32[]   in_pos           # At target position (1 = in position)
   int32[]   negative_ls      # Negative limit switch (1 = triggered)
   int32[]   positive_ls      # Positive limit switch (1 = triggered)
   int32[]   home_switch      # Home switch (1 = triggered)

   # Command and feedback values (per axis)
   float64[] pos_cmd          # Commanded position (radians)
   float64[] velocity_cmd     # Commanded velocity (rad/s)
   float64[] actual_pos       # Actual encoder position (radians)
   float64[] actual_velocity  # Actual velocity (rad/s)
   float64[] actual_torque    # Actual torque (Nm)

The status flags use ``int32`` (not ``bool``) to match the WMX3
``CoreMotionStatus`` structure directly.

**Used by:**

- ``wmx_ros2_general_node`` -- publishes on ``/wmx/axis/state`` at 100 Hz

Service Definitions
-------------------

SetEngine
^^^^^^^^^^

Controls the WMX3 engine lifecycle (device creation and communication).

.. code-block:: text

   # Request
   bool   data      # true = create/start, false = close/stop
   string path      # WMX3 device path (e.g., "/opt/lmx/")
   string name      # Device name identifier
   ---
   # Response
   bool   success   # true if operation succeeded
   string message   # Human-readable result description

**Used by service:**

- ``/wmx/engine/set_device`` (on ``wmx_ros2_general_node``)

SetAxis
^^^^^^^^

Generic per-axis control operations. The meaning of the ``data`` field
depends on which service uses this type.

.. code-block:: text

   # Request
   int32[] index    # Axis indices (e.g., [0, 1, 2, 3, 4, 5])
   int32[] data     # Per-axis values (meaning varies by service)
   ---
   # Response
   bool   success   # true only if ALL axes succeeded
   string message   # Concatenated per-axis result messages

**Used by services:**

- ``/wmx/axis/set_on`` -- ``data``: 1 = enable, 0 = disable
- ``/wmx/axis/clear_alarm`` -- ``data``: not used (pass zeros)
- ``/wmx/axis/set_mode`` -- ``data``: 0 = position, 1 = velocity
- ``/wmx/axis/set_polarity`` -- ``data``: 1 = normal, -1 = reversed
- ``/wmx/axis/homing`` -- ``data``: not used (pass zeros)

SetAxisGearRatio
^^^^^^^^^^^^^^^^^

Configures the encoder gear ratio for each axis, mapping encoder counts to
physical units (radians).

.. code-block:: text

   # Request
   int32[]   index         # Axis indices
   float64[] numerator     # Gear ratio numerators
   float64[] denumerator   # Gear ratio denominators
   ---
   # Response
   bool   success          # true only if ALL axes succeeded
   string message          # Concatenated per-axis result messages

The effective gear ratio per axis is ``numerator / denumerator``.

**Used by service:**

- ``/wmx/axis/set_gear_ratio`` (on ``wmx_ros2_general_node``)

Building the Package
--------------------

This package must be built **before** ``wmx_ros2_package`` because the main
package depends on the generated message headers:

.. code-block:: bash

   cd ~/wmx_ros2_ws
   colcon build --packages-select wmx_ros2_message
   source install/setup.bash

Verify the interfaces are registered:

.. code-block:: bash

   ros2 interface list | grep wmx

Expected:

.. code-block:: text

   wmx_ros2_message/msg/AxisPose
   wmx_ros2_message/msg/AxisState
   wmx_ros2_message/msg/AxisVelocity
   wmx_ros2_message/srv/SetAxis
   wmx_ros2_message/srv/SetAxisGearRatio
   wmx_ros2_message/srv/SetEngine

Inspect a specific interface:

.. code-block:: bash

   ros2 interface show wmx_ros2_message/msg/AxisState
