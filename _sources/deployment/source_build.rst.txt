Source Build
============

This page provides a quick reference for building from source. For the
complete step-by-step guide with dependency installation, see
:doc:`../quick_start/install_dependencies`.

Prerequisites
-------------

- Ubuntu 22.04 or 24.04
- ROS2 Humble or Jazzy installed
- LMX (WMX3 runtime) installed at ``/opt/lmx/``
- ROS2 dependencies installed (see :doc:`../environment/system_requirements`)

Clone and Build
---------------

.. code-block:: bash

   mkdir -p ~/wmx_ros2_ws/src
   cd ~/wmx_ros2_ws/src
   git clone git@bitbucket.org:mvs_app/wmx_ros2_application.git

   cd ~/wmx_ros2_ws

   # Stage 1: Build message package first
   colcon build --packages-select wmx_ros2_message
   source install/setup.bash

   # Stage 2: Build all packages
   colcon build
   source install/setup.bash

.. note::

   The two-stage build is required because ``wmx_ros2_package`` depends on
   the generated message headers from ``wmx_ros2_message``.

Verify
------

.. code-block:: bash

   ros2 pkg list | grep wmx

Expected:

.. code-block:: text

   wmx_ros2_message
   wmx_ros2_package

Rebuilding
----------

After modifying source code:

.. code-block:: bash

   cd ~/wmx_ros2_ws
   colcon build
   source install/setup.bash

To rebuild a single package:

.. code-block:: bash

   colcon build --packages-select wmx_ros2_package
   source install/setup.bash
