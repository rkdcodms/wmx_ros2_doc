Benchmarks
==========

Test Environment
----------------

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
--------------------

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
