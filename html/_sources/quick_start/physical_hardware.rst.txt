Launching with Physical Hardware
=================================

This guide covers connecting to and launching the WMX ROS2 application with a
physical robot (Dobot CR3A or compatible 6-DOF manipulator with EtherCAT servo drives).

Prerequisites
-------------

Before launching, ensure:

- The WMX ROS2 workspace is built and sourced (see :doc:`install_dependencies`)
- LMX (WMX3 runtime) is installed at ``/opt/lmx/``
- The EtherCAT network cable is connected between the compute platform and the
  first servo drive
- The robot and all servo drives are powered on
- You have ``sudo`` privileges

Hardware Connection
-------------------

EtherCAT Wiring
^^^^^^^^^^^^^^^^^

The WMX3 engine communicates with servo drives over a dedicated EtherCAT Ethernet
connection. This is **not** a standard TCP/IP network.

.. code-block:: text

   ┌──────────────┐    Ethernet     ┌──────────┐    ┌──────────┐         ┌──────────┐
   │  Compute PC  │────(EtherCAT)──►│ Servo J1 │───►│ Servo J2 │── ... ──│ Servo J6 │
   │  (dedicated  │                 └──────────┘    └──────────┘         └──────────┘
   │   NIC port)  │                     │
   └──────────────┘              Gripper I/O module
                                 (if applicable)

- Use a **dedicated Ethernet port** for EtherCAT -- do not share it with your
  regular network
- Servo drives are daisy-chained (each drive has IN and OUT EtherCAT ports)
- The I/O module for gripper control is part of the same EtherCAT chain

.. warning::

   Do not assign an IP address to the EtherCAT network interface. The WMX3
   engine manages this port at the raw Ethernet level.

EtherCAT Network Information (ENI)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``eni/`` directory in the repository contains EtherCAT Network Information
files for supported servo drives. These files describe the PDO mapping and
communication parameters for each device type. The WMX3 engine uses these
during network scanning.

Configuration
-------------

Select the correct configuration for your platform:

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

Before launching, verify the ``wmx_param_file_path`` in the config YAML points to
the correct WMX3 parameter file for your robot.

**Intel config** (``intel_manipulator_config_cr3a.yaml``):

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
       wmx_param_file_path: /home/<user>/wmx_ros2_ws/src/wmx_ros2_application/wmx_ros2_package/config/cr3a_wmx_parameters.xml

   follow_joint_trajectory_server:
     ros__parameters:
       joint_number: 6
       wmx_gripper_topic: /wmx/set_gripper
       joint_trajectory_action: /movensys_manipulator_arm_controller/follow_joint_trajectory

**Orin config** (``orin_manipulator_config_cr3a.yaml``):

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
       wmx_param_file_path: /home/<user>/wmx_ros2_ws/src/wmx_ros2_application/wmx_ros2_package/config/cr3a_wmx_parameters.xml

   follow_joint_trajectory_server:
     ros__parameters:
       joint_number: 6
       wmx_gripper_topic: /wmx/set_gripper
       joint_trajectory_action: /movensys_manipulator_arm_controller/follow_joint_trajectory

.. important::

   Update the ``wmx_param_file_path`` to match your actual home directory and
   workspace path. The default paths reference specific machine usernames
   (``mvsk``, ``mic-733ao``) that may differ on your system.

.. note::

   The Intel config publishes simulator feedback to a Gazebo topic
   (``gazebo_joint_topic``), while the Orin config publishes to an Isaac Sim
   topic (``isaacsim_joint_topic``). Both platforms always publish to the
   primary ``/joint_states`` topic for MoveIt2.

Launching the Manipulator
--------------------------

The launch commands require ``sudo`` with environment variable preservation to
allow the WMX3 engine to access EtherCAT hardware while maintaining the ROS2
environment.

Intel Platform
^^^^^^^^^^^^^^^

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

NVIDIA Jetson Orin Platform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

When the launch file starts, three nodes initialize in parallel. The expected
console output shows the following sequence:

1. **Device creation** -- Each node creates a WMX3 device handle (with up to 5
   retries if other nodes are contending for the lock):

   .. code-block:: text

      [manipulator_state] Created a device (attempt 1)
      [follow_joint_trajectory_server] Created a device (attempt 1)
      [wmx_ros2_general_node] Created a device (attempt 1)

2. **EtherCAT scan** -- The ``manipulator_state`` node scans the EtherCAT network:

   .. code-block:: text

      [manipulator_state] Scan network operation done!

3. **Communication start** -- EtherCAT real-time communication begins:

   .. code-block:: text

      [manipulator_state] Start communication

4. **Parameter loading** -- WMX3 axis parameters (gear ratios, polarities) are
   loaded from the XML file:

   .. code-block:: text

      [manipulator_state] Success to set WMX params

5. **Servo enable** -- All 6 joint servos are cleared of alarms and enabled:

   .. code-block:: text

      [manipulator_state] Clear alarm axis 0
      [manipulator_state] Servo 0 on
      ...
      [manipulator_state] Clear alarm axis 5
      [manipulator_state] Servo 5 on

6. **Ready** -- All nodes report ready:

   .. code-block:: text

      [manipulator_state] manipulator_state is ready
      [follow_joint_trajectory_server] follow_joint_trajectory_server is ready
      [wmx_ros2_general_node] wmx_ros2_general_node is ready

Verifying the Connection
-------------------------

Once launched, verify the system is running correctly in a separate terminal.

**Check nodes are running:**

.. code-block:: bash

   ros2 node list

Expected:

.. code-block:: text

   /follow_joint_trajectory_server
   /manipulator_state
   /wmx_ros2_general_node

**Check joint states are publishing:**

.. code-block:: bash

   ros2 topic hz /joint_states

Expected: approximately 500 Hz.

**View live joint positions:**

.. code-block:: bash

   ros2 topic echo /joint_states --field position

This should show 8 values (6 joint positions + 2 gripper finger positions) updating
in real time. Manually move the robot (if in freedrive mode) to confirm the values
change.

**Check engine status:**

.. code-block:: bash

   ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger

Expected response:

.. code-block:: text

   success: True
   message: "Communicating"

The ``Communicating`` state confirms the EtherCAT link is active.

**Check the action server is ready:**

.. code-block:: bash

   ros2 action list

Expected:

.. code-block:: text

   /movensys_manipulator_arm_controller/follow_joint_trajectory

Gripper Control
----------------

Test the gripper service:

**Close gripper:**

.. code-block:: bash

   ros2 service call /wmx/set_gripper std_srvs/srv/SetBool "{data: true}"

**Open gripper:**

.. code-block:: bash

   ros2 service call /wmx/set_gripper std_srvs/srv/SetBool "{data: false}"

Running the Example Client
---------------------------

The ``wmx_ros2_general_example`` node demonstrates the full low-level control
workflow (engine setup, axis configuration, velocity/position motion, and shutdown).

First, ensure the ``wmx_ros2_general_node`` is running (either via the manipulator
launch or the general launch). Then in a separate terminal:

.. code-block:: bash

   ros2 run wmx_ros2_package wmx_ros2_general_example

This will sequentially:

1. Start the WMX3 engine and communication
2. Configure gear ratios and axis polarities
3. Clear alarms and enable servos
4. Execute velocity motion (1 rad/s for 10 seconds)
5. Execute absolute position motion (5 rad, then -2 rad)
6. Execute relative position motion
7. Disable servos and shut down

.. warning::

   The example moves real motors. Ensure the robot workspace is clear before
   running.

Shutdown
--------

To cleanly shut down the system:

1. Press ``Ctrl+C`` in the launch terminal
2. The nodes will automatically:

   - Disable all servo drives (``SetServoOn(axis, 0)``)
   - Stop EtherCAT communication
   - Close the WMX3 device
   - Wait 3 seconds for cleanup

.. code-block:: text

   [manipulator_state] Servo 0 off
   ...
   [manipulator_state] Servo 5 off
   [manipulator_state] Communication stopped
   [manipulator_state] Device stopped

Troubleshooting
----------------

**"Failed to create device" errors:**

- Verify LMX is installed at ``/opt/lmx/``
- Ensure you are running with ``sudo``
- Check that no other WMX3 application is running (lock contention)

**"Failed to scan network" errors:**

- Verify the EtherCAT cable is connected to the correct Ethernet port
- Ensure all servo drives are powered on
- Check the ``eni/`` directory has the correct device descriptor files

**"Failed to start communication" errors:**

- EtherCAT network scan may have failed (check previous errors)
- Servo drives may not be properly connected in the daisy chain
- The EtherCAT Ethernet port may be configured with an IP address (remove it)

**Joint state values all zero:**

- Verify EtherCAT communication is in ``Communicating`` state
- Check that servo drives are enabled (no amp alarms)
- Verify the ``wmx_param_file_path`` points to the correct XML file

**Servo alarm errors:**

- Use ``ros2 service call /wmx/axis/clear_alarm`` to clear alarms
- Check for physical obstructions or overcurrent conditions on the robot
