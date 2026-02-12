System Requirements
===================

Operating System
----------------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - OS
     - Version
     - Notes
   * - Ubuntu
     - 22.04 LTS (Jammy)
     - Primary development and testing platform
   * - Ubuntu
     - 24.04 LTS (Noble)
     - Supported with ROS2 Jazzy

.. note::

   Root (sudo) access is required at runtime. The WMX3 motion control engine
   and EtherCAT communication require kernel-level access to the network
   interface.

ROS2 Distribution
-----------------

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Distribution
     - Ubuntu Version
     - Platform
   * - ROS2 Humble Hawksbill
     - Ubuntu 22.04
     - NVIDIA Jetson Orin, Intel x86_64
   * - ROS2 Jazzy Jalisco
     - Ubuntu 24.04
     - Intel x86_64

The C++ standard required is **C++17** (set in ``CMakeLists.txt``).

Hardware Requirements
---------------------

Compute Platform
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Component
     - Minimum
     - Recommended
   * - CPU
     - x86_64 or ARM64 (aarch64)
     - Intel Core i7 or NVIDIA Jetson Orin
   * - RAM
     - 4 GB
     - 8 GB or more
   * - Storage
     - 10 GB free
     - 20 GB free (including ROS2 + MoveIt2)
   * - GPU
     - Not required for base operation
     - NVIDIA GPU with CUDA for Isaac cuMotion

**Tested Platforms:**

- **Intel x86_64** -- Desktop/industrial PC (referenced as ``mvsk`` in configs)
- **NVIDIA Jetson Orin** -- Edge AI platform (referenced as ``mic-733ao`` in configs)

EtherCAT Network Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The WMX3 engine communicates with servo drives over EtherCAT, which requires a
dedicated Ethernet port:

- **Dedicated Ethernet NIC** for the EtherCAT fieldbus (cannot be shared with
  regular network traffic)
- The EtherCAT port connects directly to the first servo drive in the daisy chain
- WMX3 manages the EtherCAT master internally

.. warning::

   The EtherCAT Ethernet port is **not** a standard TCP/IP connection. Do not
   configure it with a regular IP address. The WMX3 engine controls this
   interface at the raw Ethernet level.

Robot Hardware
^^^^^^^^^^^^^^^

Currently supported:

- **Dobot CR3A** -- 6-DOF collaborative robot with EtherCAT servo drives and
  pneumatic gripper

The architecture is designed to be robot-agnostic for any 6-DOF manipulator with
EtherCAT-connected servo drives. See :doc:`../architecture/system_overview` for
details on adapting to other robots.

Software Dependencies
---------------------

WMX3 / LMX Runtime (Required)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The WMX3 motion control libraries must be pre-installed at ``/opt/lmx/``. This is
a proprietary runtime that provides:

- EtherCAT master functionality
- Real-time motion control engine
- CoreMotion, AdvancedMotion, I/O, and EtherCAT APIs

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Component
     - Location
   * - WMX3 headers
     - ``/opt/lmx/include/``
   * - WMX3 sample headers
     - ``/opt/lmx/sample/Header/``
   * - WMX3 shared libraries
     - ``/opt/lmx/lib/``
   * - WMX3 engine version
     - v3.5.0.0 (or compatible)

.. list-table:: WMX3 Shared Libraries
   :header-rows: 1
   :widths: 30 70

   * - Library
     - Purpose
   * - ``libwmx3api.so``
     - Core WMX3 device and engine management
   * - ``libcoremotionapi.so``
     - Position, velocity, and axis control
   * - ``libadvancedmotionapi.so``
     - Spline trajectory execution
   * - ``libioapi.so``
     - Digital I/O control (gripper)
   * - ``libecapi.so``
     - EtherCAT network management
   * - ``libimdll.so``
     - Internal WMX3 dependency

.. note::

   LMX installation is provided separately by Movensys. Contact your Movensys
   representative for the LMX installer package.

ROS2 APT Dependencies
^^^^^^^^^^^^^^^^^^^^^^

The following ROS2 packages must be installed via ``apt``:

.. code-block:: bash

   sudo apt install -y ros-${ROS_DISTRO}-graph-msgs \
                       ros-${ROS_DISTRO}-moveit* \
                       ros-${ROS_DISTRO}-ros2-control \
                       ros-${ROS_DISTRO}-ros2-controllers \
                       ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

**Breakdown:**

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Package
     - Purpose
   * - ``ros-${ROS_DISTRO}-graph-msgs``
     - Graph visualization messages
   * - ``ros-${ROS_DISTRO}-moveit*``
     - MoveIt2 motion planning framework (all packages)
   * - ``ros-${ROS_DISTRO}-ros2-control``
     - ros2_control hardware abstraction framework
   * - ``ros-${ROS_DISTRO}-ros2-controllers``
     - Standard ros2_control controllers
   * - ``ros-${ROS_DISTRO}-rmw-cyclonedds-cpp``
     - CycloneDDS middleware (used instead of default Fast DDS)

Package Build Dependencies (from package.xml)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These are resolved automatically by ``rosdep`` and ``colcon build``:

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Dependency
     - Type
     - Source
   * - ``ament_cmake``
     - buildtool
     - Both packages
   * - ``rclcpp``
     - build + exec
     - Both packages
   * - ``rclcpp_action``
     - build (CMake)
     - ``wmx_ros2_package``
   * - ``rosidl_default_generators``
     - build
     - ``wmx_ros2_message``
   * - ``rosidl_default_runtime``
     - exec
     - ``wmx_ros2_message``
   * - ``std_msgs``
     - build (CMake)
     - ``wmx_ros2_package``
   * - ``std_srvs``
     - depend
     - ``wmx_ros2_package``
   * - ``sensor_msgs``
     - build (CMake)
     - ``wmx_ros2_package``
   * - ``geometry_msgs``
     - build (CMake)
     - ``wmx_ros2_package``
   * - ``nav_msgs``
     - build (CMake)
     - ``wmx_ros2_package``
   * - ``control_msgs``
     - build (CMake)
     - ``wmx_ros2_package``
   * - ``trajectory_msgs``
     - build (CMake)
     - ``wmx_ros2_package``
   * - ``ros2launch``
     - exec
     - ``wmx_ros2_package``
   * - ``joint_state_publisher``
     - exec
     - ``wmx_ros2_package``
   * - ``robot_state_publisher``
     - exec
     - ``wmx_ros2_package``
   * - ``rviz``
     - exec
     - ``wmx_ros2_package``
   * - ``xacro``
     - exec
     - ``wmx_ros2_package``

Environment Variables
---------------------

The following environment variables must be set (typically in ``~/.bashrc``):

.. code-block:: bash

   # ROS2 domain ID (must match across all machines in the network)
   export ROS_DOMAIN_ID=70

   # Use CycloneDDS instead of default Fast DDS
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

   # ROS2 base setup
   source /opt/ros/${ROS_DISTRO}/setup.bash

   # Workspace overlay
   source ~/wmx_ros2_ws/install/setup.bash
