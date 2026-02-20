Configure Environment
=====================

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
-------------------

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
