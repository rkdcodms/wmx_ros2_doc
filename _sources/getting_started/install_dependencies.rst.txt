Install Dependencies
====================

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
-------------------------

The WMX3 runtime must be pre-installed at ``/opt/lmx/``:

.. code-block:: bash

   ls /opt/lmx/include/WMX3Api.h
   ls /opt/lmx/lib/libwmx3api.so

Both files must exist. If not, contact your Movensys representative for the
LMX installer package.
