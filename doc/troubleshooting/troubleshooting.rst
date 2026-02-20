Troubleshooting
===============

Device Creation Failures
------------------------

**Symptom:** ``Failed to create device after N attempts``

**Solutions:**

- Verify LMX is installed: ``ls /opt/lmx/lib/libwmx3api.so``
- Ensure you are running with ``sudo``
- Check that no other WMX3 application is running (lock contention causes
  error code 297)
- If the error persists after stopping other applications, reboot to clear
  stale device locks

EtherCAT Scan Failures
-----------------------

**Symptom:** ``Failed to scan network``

**Solutions:**

- Verify the EtherCAT cable is connected to the correct dedicated Ethernet
  port
- Ensure all servo drives are powered on
- Check the ``eni/`` directory has the correct EtherCAT Network Information
  files for your servo drives
- Verify the EtherCAT Ethernet port does **not** have an IP address assigned

Communication Start Failures
-----------------------------

**Symptom:** ``Failed to start communication``

**Solutions:**

- Check that the EtherCAT network scan succeeded (look for earlier scan
  errors in the log)
- Verify servo drives are properly daisy-chained (IN port to OUT port)
- Remove any IP address from the EtherCAT network interface
- Check the WMX3 engine status:

  .. code-block:: bash

     ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger

Joint States All Zero
---------------------

**Symptom:** ``/joint_states`` publishes but all position values are zero.

**Solutions:**

- Verify engine is in ``Communicating`` state (see above)
- Check that servo drives are enabled (no amplifier alarms):

  .. code-block:: bash

     ros2 topic echo /wmx/axis/state --field amp_alarm

- Verify ``wmx_param_file_path`` in the config YAML points to the correct
  WMX3 parameter XML file for your robot
- Clear alarms and re-enable servos:

  .. code-block:: bash

     ros2 service call /wmx/axis/clear_alarm wmx_ros2_message/srv/SetAxis \
       "{index: [0,1,2,3,4,5], data: [0,0,0,0,0,0]}"

Servo Alarm Errors
------------------

**Symptom:** Servo drives report amplifier alarms; motion commands are
rejected.

**Solutions:**

- Clear alarms via the service:

  .. code-block:: bash

     ros2 service call /wmx/axis/clear_alarm wmx_ros2_message/srv/SetAxis \
       "{index: [0,1,2,3,4,5], data: [0,0,0,0,0,0]}"

- Check for physical obstructions or overcurrent conditions on the robot
- Verify gear ratios and polarities match the physical servo configuration

Trajectory Execution Failures
------------------------------

**Symptom:** ``FollowJointTrajectory`` goal is aborted with a WMX3 error
code.

**Solutions:**

- Check that the trajectory has at most 1000 waypoints
- Verify all servos are enabled and in the correct mode
- Check the WMX3 error description in the action result ``error_string``
- See :doc:`../architecture/flowcharts` for the error handling flow

Gripper Not Responding
-----------------------

**Symptom:** Gripper open/close service calls succeed but the gripper
does not move.

**Solutions:**

- Verify the I/O module is part of the EtherCAT daisy chain
- Check EtherCAT communication is active (engine state = ``Communicating``)
- Test the I/O directly:

  .. code-block:: bash

     ros2 service call /wmx/set_gripper std_srvs/srv/SetBool "{data: true}"

Nodes Not Found
---------------

**Symptom:** ``ros2 pkg list | grep wmx`` returns nothing.

**Solutions:**

- Source the workspace: ``source ~/wmx_ros2_ws/install/setup.bash``
- Rebuild if needed: ``cd ~/wmx_ros2_ws && colcon build``
- Check the two-stage build was done correctly (message package first).
  See :doc:`../getting_started`.

Build Errors
------------

**Symptom:** Linker errors referencing ``coremotionapi``, ``wmx3api``, or
other WMX3 libraries.

**Solutions:**

- Verify LMX libraries exist: ``ls /opt/lmx/lib/``
- Ensure ``LD_LIBRARY_PATH`` includes ``/opt/lmx/lib/``:

  .. code-block:: bash

     export LD_LIBRARY_PATH=/opt/lmx/lib:$LD_LIBRARY_PATH

- Install missing ROS2 dependencies:

  .. code-block:: bash

     cd ~/wmx_ros2_ws
     rosdep install --from-paths src --ignore-src -y

Getting Help
------------

For additional support, contact your Movensys representative or file an
issue on the project repository.
