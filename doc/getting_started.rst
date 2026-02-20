Getting Started
===============

This guide walks you through the complete setup — from system requirements to
launching your first robot control session with WMX ROS2.

.. contents:: On this page
   :local:
   :depth: 1

System Requirements
-------------------

Operating System
^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^

**Compute Platform:**

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

**EtherCAT Network Interface:**

- **Dedicated Ethernet NIC** for the EtherCAT fieldbus (cannot be shared with
  regular network traffic)
- The EtherCAT port connects directly to the first servo drive in the daisy chain

.. warning::

   The EtherCAT Ethernet port is **not** a standard TCP/IP connection. Do not
   configure it with a regular IP address. The WMX3 engine controls this
   interface at the raw Ethernet level.

**Supported Robot:**

- **Dobot CR3A** -- 6-DOF collaborative robot with EtherCAT servo drives and
  pneumatic gripper

The architecture is robot-agnostic for any 6-DOF manipulator with EtherCAT
servo drives. See :doc:`architecture/architecture` for details on adapting
to other robots.

Supported Models & Platforms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 15 15 15 55

   * - Model
     - DOF
     - Payload
     - Status
   * - Dobot CR3A
     - 6
     - 16.5 kg
     - Fully supported. Configuration files and WMX3 parameters included.

.. list-table::
   :header-rows: 1
   :widths: 30 25 25 20

   * - Platform
     - Architecture
     - ROS2 Distro
     - Config File
   * - Intel x86_64 PC
     - x86_64
     - Humble / Jazzy
     - ``intel_manipulator_config_cr3a.yaml``
   * - NVIDIA Jetson Orin
     - aarch64
     - Humble
     - ``orin_manipulator_config_cr3a.yaml``

Install ROS2
------------

ROS2 Humble (Ubuntu 22.04)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow the official guide:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Quick summary:

.. code-block:: bash

   # Set locale
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   # Setup sources
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install
   sudo apt update && sudo apt upgrade
   sudo apt install ros-humble-desktop

   # Source
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc

ROS2 Jazzy (Ubuntu 24.04)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow the official guide:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Quick summary:

.. code-block:: bash

   # Set locale
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   # Setup sources
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install
   sudo apt update && sudo apt upgrade
   sudo apt install ros-jazzy-desktop

   # Source
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc

Verify ROS2
^^^^^^^^^^^^

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

In another terminal:

.. code-block:: bash

   ros2 run demo_nodes_cpp listener

If messages are being received, ROS2 is working correctly.

Install Dependencies
--------------------

Install the required ROS2 packages:

.. code-block:: bash

   sudo apt update
   sudo apt install -y ros-${ROS_DISTRO}-graph-msgs \
                       ros-${ROS_DISTRO}-moveit* \
                       ros-${ROS_DISTRO}-ros2-control \
                       ros-${ROS_DISTRO}-ros2-controllers \
                       ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
   sudo apt install -y python3-colcon-common-extensions python3-rosdep

Verify LMX Installation
^^^^^^^^^^^^^^^^^^^^^^^^^

The WMX3 runtime must be pre-installed at ``/opt/lmx/``:

.. code-block:: bash

   ls /opt/lmx/include/WMX3Api.h
   ls /opt/lmx/lib/libwmx3api.so

Both files must exist. If not, contact your Movensys representative for the
LMX installer package.

Create Workspace and Build
--------------------------

.. code-block:: bash

   mkdir -p ~/wmx_ros2_ws/src
   cd ~/wmx_ros2_ws/src
   git clone git@bitbucket.org:mvs_app/wmx_ros2_application.git

Install rosdep dependencies:

.. code-block:: bash

   sudo rosdep init   # only needed once per system
   rosdep update
   cd ~/wmx_ros2_ws
   rosdep install --from-paths src --ignore-src -y

Build in two stages (``wmx_ros2_package`` depends on ``wmx_ros2_message``):

.. code-block:: bash

   cd ~/wmx_ros2_ws

   # Stage 1: Build the message package first
   colcon build --packages-select wmx_ros2_message
   source install/setup.bash

   # Stage 2: Build all remaining packages
   colcon build
   source install/setup.bash

Configure Environment
---------------------

Add the following to your ``~/.bashrc``:

.. code-block:: bash

   # ROS2 setup
   source /opt/ros/${ROS_DISTRO}/setup.bash
   source ~/wmx_ros2_ws/install/setup.bash

   # ROS2 domain ID (must match across all machines)
   export ROS_DOMAIN_ID=70

   # Use CycloneDDS middleware
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Apply the changes:

.. code-block:: bash

   source ~/.bashrc

Verify Installation
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 pkg list | grep wmx

Expected:

.. code-block:: text

   wmx_ros2_message
   wmx_ros2_package

.. code-block:: bash

   ros2 pkg executables wmx_ros2_package

Expected:

.. code-block:: text

   wmx_ros2_package follow_joint_trajectory_server
   wmx_ros2_package manipulator_state
   wmx_ros2_package wmx_ros2_general_example
   wmx_ros2_package wmx_ros2_general_node

Launch with Mock Hardware
-------------------------

The WMX ROS2 nodes communicate directly with the WMX3 engine over EtherCAT.
There is no built-in mock hardware mode — nodes will attempt to connect to
real hardware on startup. Below are ways to test without a physical robot.

Test the General Node (Standalone)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

If no EtherCAT hardware is connected, the node will log retry attempts but
ROS2 service endpoints will still be registered.

Publish Simulated Joint States
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To test MoveIt2 or RViz without real encoder feedback:

.. code-block:: bash

   ros2 topic pub /joint_states sensor_msgs/msg/JointState \
     "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
       name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
              'picker_1_joint', 'picker_2_joint'],
       position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
       velocity: [],
       effort: []}" --rate 50

Simulation Environments
^^^^^^^^^^^^^^^^^^^^^^^^

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

Launch with Physical Hardware
------------------------------

Prerequisites
^^^^^^^^^^^^^^

- The WMX ROS2 workspace is built and sourced
- LMX (WMX3 runtime) is installed at ``/opt/lmx/``
- EtherCAT cable is connected between compute platform and first servo drive
- Robot and all servo drives are powered on
- You have ``sudo`` privileges

EtherCAT Wiring
^^^^^^^^^^^^^^^^^

.. code-block:: text

   ┌──────────────┐    Ethernet     ┌──────────┐    ┌──────────┐         ┌──────────┐
   │  Compute PC  │────(EtherCAT)──►│ Servo J1 │───►│ Servo J2 │── ... ──│ Servo J6 │
   │  (dedicated  │                 └──────────┘    └──────────┘         └──────────┘
   │   NIC port)  │                     │
   └──────────────┘              Gripper I/O module

- Use a **dedicated Ethernet port** for EtherCAT
- Servo drives are daisy-chained (each drive has IN and OUT ports)
- The I/O module for gripper control is part of the same chain

Configuration
^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Platform
     - Config File
     - Launch File
   * - Intel x86_64
     - ``intel_manipulator_config_cr3a.yaml``
     - ``wmx_ros2_intel_manipulator_cr3a.launch.py``
   * - NVIDIA Jetson Orin
     - ``orin_manipulator_config_cr3a.yaml``
     - ``wmx_ros2_orin_manipulator_cr3a.launch.py``

.. important::

   Update the ``wmx_param_file_path`` in the config YAML to match your actual
   home directory and workspace path.

Launch Commands
^^^^^^^^^^^^^^^^

**Intel Platform:**

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
                 ros2 launch wmx_ros2_package wmx_ros2_intel_manipulator_cr3a.launch.py \
                 use_sim_time:=false"

**NVIDIA Jetson Orin Platform:**

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
                 ros2 launch wmx_ros2_package wmx_ros2_orin_manipulator_cr3a.launch.py \
                 use_sim_time:=false"

Startup Sequence
^^^^^^^^^^^^^^^^^

When launched, three nodes initialize in parallel:

1. **Device creation** -- Each node creates a WMX3 device handle
2. **EtherCAT scan** -- Network scan discovers all servo drives
3. **Communication start** -- Real-time EtherCAT communication begins
4. **Parameter loading** -- Gear ratios and axis polarities loaded from XML
5. **Servo enable** -- All 6 joint servos cleared and enabled
6. **Ready** -- All nodes report ready

Verifying the Connection
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 node list          # Expect: 3 nodes
   ros2 topic hz /joint_states   # Expect: ~500 Hz
   ros2 topic echo /joint_states --field position  # Live joint positions
   ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger  # Expect: "Communicating"
   ros2 action list        # Expect: follow_joint_trajectory

Gripper Control
^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Close gripper
   ros2 service call /wmx/set_gripper std_srvs/srv/SetBool "{data: true}"

   # Open gripper
   ros2 service call /wmx/set_gripper std_srvs/srv/SetBool "{data: false}"

Shutdown
^^^^^^^^^

Press ``Ctrl+C`` in the launch terminal. The nodes will automatically disable
servos, stop EtherCAT communication, and close the WMX3 device.
