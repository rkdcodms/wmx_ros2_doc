Performance
===========

Non-Functional Requirements
---------------------------

The following requirements are derived from the system architecture and
configuration defaults.

.. list-table::
   :header-rows: 1
   :widths: 20 30 25 25

   * - Category
     - Metric
     - Target
     - Source
   * - Latency
     - Joint state feedback rate
     - 500 Hz (2 ms period)
     - ``joint_feedback_rate`` parameter
   * - Latency
     - Axis state feedback rate
     - 100 Hz (10 ms period)
     - Hardcoded in ``wmx_ros2_general_node``
   * - Throughput
     - Max trajectory waypoints
     - 1000 points
     - ``MAX_TRAJ_POINTS`` in ``follow_joint_trajectory_server``
   * - Reliability
     - Device creation retries
     - 5 attempts, 2s interval
     - All nodes
   * - Concurrency
     - Simultaneous WMX3 devices
     - 3 (one per node)
     - Lock-based access with retry
   * - Timing
     - Min waypoint interval
     - 1 ms
     - Trajectory server timing filter

.. todo::

   Add measured values for:

   - EtherCAT round-trip latency
   - End-to-end MoveIt2 command-to-motion latency
   - Maximum sustained joint state publish rate under load
   - Emergency stop response time

Benchmarks
----------

Test Environment
^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Component
     - Configuration
   * - **Intel Platform**
     - x86_64 desktop PC (referenced as ``mvsk`` in configs)
   * - **Orin Platform**
     - NVIDIA Jetson Orin (referenced as ``mic-733ao`` in configs)
   * - **Robot**
     - Dobot CR3A (6-DOF, EtherCAT servo drives)
   * - **ROS2**
     - Humble (Ubuntu 22.04) / Jazzy (Ubuntu 24.04)
   * - **Middleware**
     - CycloneDDS (``rmw_cyclonedds_cpp``)

Measurement Commands
^^^^^^^^^^^^^^^^^^^^

Use these commands to benchmark a running system:

**Joint state publish rate:**

.. code-block:: bash

   ros2 topic hz /joint_states

Expected: ~500 Hz (configured via ``joint_feedback_rate``).

**Axis state publish rate:**

.. code-block:: bash

   ros2 topic hz /wmx/axis/state

Expected: ~100 Hz.

**Topic latency (requires two terminals):**

.. code-block:: bash

   ros2 topic delay /joint_states

.. todo::

   Add benchmark results:

   - Joint state rate stability over 1 hour
   - Trajectory execution timing accuracy (planned vs actual)
   - CPU and memory usage per node on Intel and Orin platforms
   - Network bandwidth consumed by EtherCAT communication
