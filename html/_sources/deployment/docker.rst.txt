Docker
======

.. note::

   Docker support is planned for future releases. The WMX ROS2 application
   requires access to EtherCAT network interfaces and the WMX3 runtime at
   ``/opt/lmx/``, which requires special Docker configuration (host
   networking, device passthrough, volume mounts for ``/opt/lmx/``).

See :doc:`source_build` for the current installation method.

.. todo::

   Create a Docker image with ROS2, WMX3 runtime, and pre-built
   ``wmx_ros2_application`` packages. Will require ``--net=host``,
   ``--privileged``, and ``-v /opt/lmx:/opt/lmx`` for EtherCAT access.
