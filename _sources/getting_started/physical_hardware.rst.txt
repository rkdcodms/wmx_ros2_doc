Launch with Physical Hardware
==============================

Prerequisites
-------------

- The WMX ROS2 workspace is built and sourced
- LMX (WMX3 runtime) is installed at ``/opt/lmx/``
- EtherCAT cable is connected between compute platform and first servo drive
- Robot and all servo drives are powered on
- You have ``sudo`` privileges

EtherCAT Wiring
-----------------

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
--------------

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
----------------

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
-----------------

When launched, three nodes initialize in parallel:

1. **Device creation** -- Each node creates a WMX3 device handle
2. **EtherCAT scan** -- Network scan discovers all servo drives
3. **Communication start** -- Real-time EtherCAT communication begins
4. **Parameter loading** -- Gear ratios and axis polarities loaded from XML
5. **Servo enable** -- All 6 joint servos cleared and enabled
6. **Ready** -- All nodes report ready

Verifying the Connection
-------------------------

.. code-block:: bash

   ros2 node list          # Expect: 3 nodes
   ros2 topic hz /joint_states   # Expect: ~500 Hz
   ros2 topic echo /joint_states --field position  # Live joint positions
   ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger  # Expect: "Communicating"
   ros2 action list        # Expect: follow_joint_trajectory

Gripper Control
----------------

.. code-block:: bash

   # Close gripper
   ros2 service call /wmx/set_gripper std_srvs/srv/SetBool "{data: true}"

   # Open gripper
   ros2 service call /wmx/set_gripper std_srvs/srv/SetBool "{data: false}"

Shutdown
---------

Press ``Ctrl+C`` in the launch terminal. The nodes will automatically disable
servos, stop EtherCAT communication, and close the WMX3 device.
