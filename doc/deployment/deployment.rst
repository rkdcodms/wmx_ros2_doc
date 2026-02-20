Deployment
==========

Source Build
------------

This section provides a quick reference for building from source. For the
complete step-by-step guide with dependency installation, see
:doc:`../getting_started`.

Prerequisites
^^^^^^^^^^^^^

- Ubuntu 22.04 or 24.04
- ROS2 Humble or Jazzy installed
- LMX (WMX3 runtime) installed at ``/opt/lmx/``
- ROS2 dependencies installed (see :doc:`../getting_started`)

Clone and Build
^^^^^^^^^^^^^^^

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
^^^^^^

.. code-block:: bash

   ros2 pkg list | grep wmx

Expected:

.. code-block:: text

   wmx_ros2_message
   wmx_ros2_package

Rebuilding
^^^^^^^^^^

After modifying source code:

.. code-block:: bash

   cd ~/wmx_ros2_ws
   colcon build
   source install/setup.bash

To rebuild a single package:

.. code-block:: bash

   colcon build --packages-select wmx_ros2_package
   source install/setup.bash

Debian Package Install
----------------------

.. note::

   Debian packages are not yet available. The WMX ROS2 application must
   currently be built from source.

.. todo::

   Create ``apt``-installable Debian packages for ``wmx_ros2_message``
   and ``wmx_ros2_package`` for streamlined deployment.

Docker
------

.. note::

   Docker support is planned for future releases. The WMX ROS2 application
   requires access to EtherCAT network interfaces and the WMX3 runtime at
   ``/opt/lmx/``, which requires special Docker configuration (host
   networking, device passthrough, volume mounts for ``/opt/lmx/``).

.. todo::

   Create a Docker image with ROS2, WMX3 runtime, and pre-built
   ``wmx_ros2_application`` packages. Will require ``--net=host``,
   ``--privileged``, and ``-v /opt/lmx:/opt/lmx`` for EtherCAT access.
