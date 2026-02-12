Supported Models
================

The WMX ROS2 application is designed for 6-DOF manipulators with EtherCAT
servo drives. The architecture is robot-agnostic -- adapting to a new robot
requires creating new WMX3 parameter files and configuration YAML.

Currently Supported
-------------------

.. list-table::
   :header-rows: 1
   :widths: 15 15 15 55

   * - Model
     - DOF
     - Payload
     - Status
   * - Dobot CR3A
     - 6
     - 3 kg
     - Fully supported. Configuration files and WMX3 parameters included.

The Dobot CR3A uses EtherCAT-connected servo drives (one per joint) plus
an I/O module for gripper control.

Platform Support
----------------

.. list-table::
   :header-rows: 1
   :widths: 30 25 25 20

   * - Platform
     - Architecture
     - ROS2 Distro
     - Config File
   * - Intel x86_64 PC
     - x86_64
     - Humble / Jazzy
     - ``intel_manipulator_config_cr3a.yaml``
   * - NVIDIA Jetson Orin
     - aarch64
     - Humble
     - ``orin_manipulator_config_cr3a.yaml``

See :doc:`../quick_start/install_dependencies` for setup instructions and
:doc:`../packages/wmx_ros2_package` for configuration file details.

Adapting to Other Robots
------------------------

To add support for a new 6-DOF manipulator:

1. Create a WMX3 parameter XML file with the correct gear ratios, encoder
   settings, and axis polarities for the new servo drives
2. Create a ROS2 config YAML specifying joint names, feedback rate, and the
   path to the parameter XML
3. Create a launch file that loads the new config

See :doc:`../architecture/system_overview` for the layered architecture
that enables this separation.
