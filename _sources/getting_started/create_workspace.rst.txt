Create Workspace and Build
==========================

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
