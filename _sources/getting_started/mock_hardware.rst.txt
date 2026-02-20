Launch with Mock Hardware
=========================

The WMX ROS2 nodes communicate directly with the WMX3 engine over EtherCAT.
There is no built-in mock hardware mode — nodes will attempt to connect to
real hardware on startup. Below are ways to test without a physical robot.

Test the General Node (Standalone)
-----------------------------------

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
--------------------------------

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
------------------------

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
