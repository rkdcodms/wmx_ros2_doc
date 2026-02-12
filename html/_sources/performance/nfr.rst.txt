Non-Functional Requirements
===========================

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
