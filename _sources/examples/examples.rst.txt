Example Applications
=====================

Overview
--------

This section contains complete application examples showing how to use the
WMX ROS2 packages with different frameworks and hardware setups. Each example
is a **standalone workspace** that depends on the core ``wmx_ros2_message``
and ``wmx_ros2_package`` packages.

These examples demonstrate the **robot-agnostic design** of the system -- the
same WMX ROS2 nodes and interfaces are used regardless of the specific robot
model or compute platform. Only the configuration files (YAML parameters,
WMX3 XML parameters, and launch files) differ between setups.

.. list-table:: Example Summary
   :header-rows: 1
   :widths: 30 30 40

   * - Example
     - Platform
     - Description
   * - :doc:`isaac_manipulator`
     - NVIDIA Jetson Orin
     - Manipulator control with Isaac Sim integration
   * - :doc:`intel_manipulator`
     - Intel x86_64
     - Manipulator control with Gazebo integration

Prerequisites
-------------

All examples require:

- The core WMX ROS2 packages built and sourced
  (see :doc:`../getting_started/index`)
- LMX (WMX3 runtime) installed at ``/opt/lmx/``
- EtherCAT hardware connected (for physical robot operation)

Each example page describes any additional dependencies specific to that setup.

.. toctree::
   :maxdepth: 2

   isaac_manipulator
   intel_manipulator
