Install Dependencies
====================

This guide walks through the complete installation process for the WMX ROS2
application, from ROS2 setup to building the workspace.

Prerequisites
-------------

Before starting, ensure you have:

- Ubuntu 22.04 (for ROS2 Humble) or Ubuntu 24.04 (for ROS2 Jazzy)
- LMX (WMX Linux runtime) installed at ``/opt/lmx/`` -- contact Movensys for the installer
- Internet access for downloading packages
- ``sudo`` privileges

Step 1: Install ROS2
---------------------

If ROS2 is not yet installed, follow the official installation guide for your
distribution:

**ROS2 Humble (Ubuntu 22.04):**

See the official docs: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

**ROS2 Jazzy (Ubuntu 24.04):**

See the official docs: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

After installation, source the ROS2 setup:

.. code-block:: bash

   source /opt/ros/${ROS_DISTRO}/setup.bash

Step 2: Install ROS2 Dependencies
-----------------------------------

Install the required ROS2 packages via ``apt``:

.. code-block:: bash

   sudo apt update
   sudo apt install -y ros-${ROS_DISTRO}-graph-msgs \
                       ros-${ROS_DISTRO}-moveit* \
                       ros-${ROS_DISTRO}-ros2-control \
                       ros-${ROS_DISTRO}-ros2-controllers \
                       ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

Also install the standard development tools if not already present:

.. code-block:: bash

   sudo apt install -y python3-colcon-common-extensions python3-rosdep

Step 3: Verify LMX Installation
---------------------------------

Confirm that the WMX3 runtime is properly installed:

.. code-block:: bash

   ls /opt/lmx/include/WMX3Api.h
   ls /opt/lmx/lib/libwmx3api.so

Both files must exist. If not, install the LMX package from Movensys before
continuing.

Step 4: Create Workspace and Clone
------------------------------------

Create the ROS2 workspace and clone the application repository:

.. code-block:: bash

   mkdir -p ~/wmx_ros2_ws/src
   cd ~/wmx_ros2_ws/src
   git clone git@bitbucket.org:mvs_app/wmx_ros2_application.git

This clones the repository containing both packages:

- ``wmx_ros2_application/wmx_ros2_message`` -- Custom message and service definitions
- ``wmx_ros2_application/wmx_ros2_package`` -- Main application nodes

Step 5: Install rosdep Dependencies
-------------------------------------

Initialize ``rosdep`` (if not done previously) and install package dependencies:

.. code-block:: bash

   sudo rosdep init   # only needed once per system
   rosdep update

   cd ~/wmx_ros2_ws
   rosdep install --from-paths src --ignore-src -y

.. note::

   ``rosdep`` will resolve standard ROS2 dependencies declared in
   ``package.xml`` files. The WMX3 libraries at ``/opt/lmx/`` are not managed
   by ``rosdep`` and must be installed separately.

Step 6: Build the Workspace
-----------------------------

The build must be done in two stages because ``wmx_ros2_package`` depends on
``wmx_ros2_message`` for the generated message headers:

.. code-block:: bash

   cd ~/wmx_ros2_ws

   # Stage 1: Build the message package first
   colcon build --packages-select wmx_ros2_message
   source install/setup.bash

   # Stage 2: Build all remaining packages
   colcon build
   source install/setup.bash

.. note::

   If you encounter linker errors related to ``coremotionapi``, ``wmx3api``,
   or other WMX3 libraries, verify that ``/opt/lmx/lib/`` contains the required
   shared libraries and that the ``LD_LIBRARY_PATH`` includes ``/opt/lmx/lib/``.

Step 7: Configure Environment
-------------------------------

Add the following to your ``~/.bashrc`` to persist the environment across
terminal sessions:

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

Step 8: Verify Installation
-----------------------------

Run the following checks to confirm the installation is correct:

**Check that packages are found:**

.. code-block:: bash

   ros2 pkg list | grep wmx

Expected output:

.. code-block:: text

   wmx_ros2_message
   wmx_ros2_package

**Check that executables are available:**

.. code-block:: bash

   ros2 pkg executables wmx_ros2_package

Expected output:

.. code-block:: text

   wmx_ros2_package follow_joint_trajectory_server
   wmx_ros2_package manipulator_state
   wmx_ros2_package wmx_ros2_general_example
   wmx_ros2_package wmx_ros2_general_node

**Check that custom message types are registered:**

.. code-block:: bash

   ros2 interface list | grep wmx

Expected output:

.. code-block:: text

   wmx_ros2_message/msg/AxisPose
   wmx_ros2_message/msg/AxisState
   wmx_ros2_message/msg/AxisVelocity
   wmx_ros2_message/srv/SetAxis
   wmx_ros2_message/srv/SetAxisGearRatio
   wmx_ros2_message/srv/SetEngine

**Check that launch files are found:**

.. code-block:: bash

   ros2 launch wmx_ros2_package --show-args wmx_ros2_general.launch.py

This should print the launch file arguments without errors.

If all checks pass, the installation is complete. Proceed to
:doc:`physical_hardware` to connect to a robot or :doc:`mock_hardware` for
testing without hardware.

Rebuilding After Changes
-------------------------

When you modify source code, rebuild with:

.. code-block:: bash

   cd ~/wmx_ros2_ws
   colcon build
   source install/setup.bash

To rebuild only a specific package:

.. code-block:: bash

   colcon build --packages-select wmx_ros2_package
   source install/setup.bash
