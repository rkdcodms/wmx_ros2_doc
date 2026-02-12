WMX ROS2 Documentation
=======================

Overview
--------

Welcome to the WMX ROS2 documentation. This project provides ROS2 packages
for controlling 6-DOF robotic manipulators through the WMX3 motion control
platform over EtherCAT. The current supported robot is the **Dobot CR3A**.

**Key capabilities:**

- Real-time joint control via WMX3 EtherCAT servo drives
- MoveIt2 trajectory execution through the ``FollowJointTrajectory`` action
- NVIDIA Isaac cuMotion GPU-accelerated planning support
- Standalone axis control via ROS2 services and topics
- Gripper control via EtherCAT digital I/O
- Multi-simulator output (MoveIt2/RViz, Isaac Sim, Gazebo)

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   environment/environment
   quick_start/quick_start

.. toctree::
   :maxdepth: 2
   :caption: Packages

   packages/packages

.. toctree::
   :maxdepth: 2
   :caption: Integration Scenarios

   integration/integration

.. toctree::
   :maxdepth: 2
   :caption: Architecture & Design

   architecture/architecture

.. toctree::
   :maxdepth: 2
   :caption: Performance

   performance/performance

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api_reference/api_reference

.. toctree::
   :maxdepth: 2
   :caption: Example Applications

   examples/examples

.. toctree::
   :maxdepth: 2
   :caption: Deployment

   deployment/deployment

.. toctree::
   :maxdepth: 2
   :caption: Support

   troubleshooting/troubleshooting
