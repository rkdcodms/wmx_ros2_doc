System Requirements
===================

Operating System
----------------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - OS
     - Version
     - Notes
   * - Ubuntu
     - 22.04 LTS (Jammy)
     - Primary development and testing platform
   * - Ubuntu
     - 24.04 LTS (Noble)
     - Supported with ROS2 Jazzy

.. note::

   Root (sudo) access is required at runtime. The WMX3 motion control engine
   and EtherCAT communication require kernel-level access to the network
   interface.

ROS2 Distribution
-----------------

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Distribution
     - Ubuntu Version
     - Platform
   * - ROS2 Humble Hawksbill
     - Ubuntu 22.04
     - NVIDIA Jetson Orin, Intel x86_64
   * - ROS2 Jazzy Jalisco
     - Ubuntu 24.04
     - Intel x86_64

The C++ standard required is **C++17** (set in ``CMakeLists.txt``).

Hardware Requirements
---------------------

**Compute Platform:**

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Component
     - Minimum
     - Recommended
   * - CPU
     - x86_64 or ARM64 (aarch64)
     - Intel Core i7 or NVIDIA Jetson Orin
   * - RAM
     - 4 GB
     - 8 GB or more
   * - Storage
     - 10 GB free
     - 20 GB free (including ROS2 + MoveIt2)
   * - GPU
     - Not required for base operation
     - NVIDIA GPU with CUDA for Isaac cuMotion

**Tested Platforms:**

- **Intel x86_64** -- Desktop/industrial PC (referenced as ``mvsk`` in configs)
- **NVIDIA Jetson Orin** -- Edge AI platform (referenced as ``mic-733ao`` in configs)

**EtherCAT Network Interface:**

- **Dedicated Ethernet NIC** for the EtherCAT fieldbus (cannot be shared with
  regular network traffic)
- The EtherCAT port connects directly to the first servo drive in the daisy chain

.. warning::

   The EtherCAT Ethernet port is **not** a standard TCP/IP connection. Do not
   configure it with a regular IP address. The WMX3 engine controls this
   interface at the raw Ethernet level.

**Supported Robot:**

- **Dobot CR3A** -- 6-DOF collaborative robot with EtherCAT servo drives and
  pneumatic gripper

The architecture is robot-agnostic for any 6-DOF manipulator with EtherCAT
servo drives. See :doc:`../architecture/architecture` for details on adapting
to other robots.

Supported Models & Platforms
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 15 15 15 55

   * - Model
     - DOF
     - Payload
     - Status
   * - Dobot CR3A
     - 6
     - 16.5 kg
     - Fully supported. Configuration files and WMX3 parameters included.

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
