Intel Manipulator Integration
==============================

Overview
--------

The **movensys_intel_ws** workspace is a standalone ROS 2 application that
integrates MoveIt2 with WMX ROS2 for manipulator control on Intel x86_64
platforms. It provides a progressive 3-stage workflow that takes the Dobot CR3A
from basic trajectory validation through vision-guided pick-and-place with
real-time obstacle avoidance -- all using CPU-based algorithms.

The workspace is located at::

   ~/workspaces/movensys_intel_ws/

.. list-table:: Stage Summary
   :header-rows: 1
   :widths: 10 25 35 30

   * - Stage
     - Capability
     - Description
     - Key Component
   * - 1
     - Basic Trajectory
     - Cartesian/joint motion with MoveIt2 OMPL planner
     - MoveIt2 (OMPL/CHOMP/Pilz)
   * - 2
     - AprilTag Pick & Place
     - Vision-based detection and autonomous manipulation
     - ``pyapriltags`` + Intel RealSense
   * - 3
     - OctoMap Obstacle Avoidance
     - 3D voxel mapping for collision-free planning
     - OctoMap + Intel RealSense depth

Each stage can run in three environments:

- **Simulation** -- Gazebo provides physics simulation and sensor emulation.
  The ``simulation_action`` bridge node forwards MoveIt2 trajectories to the
  ``/gazebo_position_controller/commands`` topic (8 values: 6 joints + 2
  gripper).
- **Hardware-in-the-Loop (HiL)** -- Gazebo acts as a digital twin while the
  real robot is driven by WMX ROS2 over EtherCAT. Both systems share the same
  ``/joint_states`` topic.
- **Real Robot** -- No simulator. WMX ROS2 drives the physical robot directly
  and MoveIt2 plans in real time.


Architecture
------------

Packages
^^^^^^^^

The workspace contains five ROS 2 packages (plus copies of the core WMX ROS2
packages):

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Package
     - Purpose
   * - ``movensys_manipulator_description``
     - URDF/Xacro robot model for the Dobot CR3A with table, stand, bracket,
       gripper, and picker links. Provides STL mesh assets, Gazebo simulation
       support (``gz_ros2_control``), RViz launch files, and the
       ``gazebo_position_controller`` configuration.
   * - ``movensys_manipulator_moveit_config``
     - MoveIt2 configuration: SRDF, joint limits, KDL kinematics solver,
       OMPL/CHOMP/Pilz planners, controller configuration, stage executables
       (``simulation_action``, ``stage1_trajectory_cpp``,
       ``stage2_apriltag_cpp``, ``stage3_octomap_cpp``), the ``MoveIt2Client``
       C++ library, RealSense camera configs, OctoMap ``sensors_3d`` config,
       and AprilTag detection script.
   * - ``movensys_manipulator_intel_config``
     - Intel-platform-specific bringup launch files, additional demo
       executables (``trajectory_demo_cpp``, ``pick_and_place_cpp``),
       AprilTag detection configuration, and vision pipeline launch files.
   * - ``wmx_ros2_package``
     - Core WMX ROS2 driver package (copied into the workspace).
   * - ``wmx_ros2_message``
     - Core WMX ROS2 message definitions (copied into the workspace).


Kinematic Chain
^^^^^^^^^^^^^^^

::

   world -> world_manipulator -> table -> stand -> Link0
     -> joint1 (revolute) -> Link1
       -> joint2 (revolute) -> Link2
         -> joint3 (revolute) -> Link3
           -> joint4 (revolute) -> Link4
             -> joint5 (revolute) -> Link5
               -> joint6 (revolute) -> Link6
                 -> bracket (fixed) -> gripper (fixed)
                   -> picker_1_joint (prismatic, 0--0.045 m) -> picker_1
                   -> picker_2_joint (prismatic, 0--0.045 m) -> picker_2

The arm group ``movensys_manipulator_arm`` is defined as the chain from
``Link0`` to ``Link6`` (6 revolute joints: ``joint1`` through ``joint6``, each
with limits of ``-3.14`` to ``3.14`` rad). The end-effector group
``movensys_manipulator_eef`` includes the ``bracket`` and ``gripper`` links.

The URDF is composed from four xacro files:

- ``stage.xacro`` -- World frame, table, and stand (static links)
- ``cr3a.xacro`` -- CR3A 6-DOF arm macro with link inertials and STL meshes
- ``control.xacro`` -- ``gz_ros2_control`` hardware interface (position command
  for all 8 joints)
- ``transmission.xacro`` -- ``SimpleTransmission`` for each joint


Source Files
^^^^^^^^^^^^

**movensys_manipulator_moveit_config** executables:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Source File
     - Description
   * - ``src/simulation_action.cpp``
     - Gazebo bridge: subscribes to ``/joint_states``, serves
       ``FollowJointTrajectory`` action on
       ``/movensys_manipulator_arm_controller/follow_joint_trajectory``, and
       publishes 8-value ``Float64MultiArray`` to
       ``/gazebo_position_controller/commands``. Handles gripper via
       ``/wmx/set_gripper`` service (``SetBool``). Trajectory points are
       replayed with timing from ``time_from_start`` intervals.
   * - ``src/moveit2_client.cpp``
     - Shared ``MoveIt2Client`` library wrapping ``MoveGroupInterface``.
       Provides: ``absoluteBaseEefCartesian`` (Cartesian path with TOTG time
       parameterization), ``absoluteBaseEefJointMovement`` (IK + joint-space
       plan), ``relativeBaseEefCartesian``, ``relativeToolEefCartesian``,
       ``jointMovement``, ``setGripper``, ``lookupTF``, ``getCurrentEefPose``.
   * - ``src/stage1_trajectory.cpp``
     - Stage 1 demo: absolute Cartesian, relative base Cartesian, relative
       tool Cartesian, joint movement, and gripper open/close.
       ``vel_scale=0.3``, ``acc_scale=0.3``, ``planning_time=1.0``.
   * - ``src/stage2_apriltag.cpp``
     - Stage 2 demo: AprilTag visual servoing pick-and-place. Tracks 4 tags
       (``tag36h11:5``, ``tag36h11:9``, ``tag36h11:7``, ``tag36h11:11``) using
       iterative ``lookupTF`` + ``relativeToolEefCartesian`` convergence loop
       (tolerance: 0.03 m position, 0.05 rad orientation). Places objects in
       predefined box positions. ``vel_scale=0.7``, ``acc_scale=0.7``.
   * - ``src/stage3_octomap.cpp``
     - Stage 3 demo: OctoMap obstacle-aware planning. Phase 1 scans 4 poses to
       build OctoMap (3 s dwell per pose). Phase 2-4 execute Cartesian, joint,
       and relative movements with OMPL planning around obstacles.
       ``vel_scale=0.3``, ``acc_scale=0.3``, ``planning_time=5.0``,
       ``planning_attempts=10``.
   * - ``scripts/apriltag_detector.py``
     - Python ROS 2 node using ``pyapriltags`` for CPU-based AprilTag
       detection. Subscribes to RealSense camera image and camera_info topics,
       broadcasts TF for each detected tag as ``tag36h11:<id>``.

**movensys_manipulator_intel_config** executables:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Source File
     - Description
   * - ``src/trajectory_demo.cpp``
     - Trajectory demo: absolute Cartesian, relative base/tool Cartesian,
       joint movement, and gripper operation. ``vel_scale=1.0``,
       ``acc_scale=1.0``, ``planning_attempts=5``.
   * - ``src/pick_and_place.cpp``
     - 8-step pick-and-place demo: home -> open gripper -> pre-pick ->
       pick (Cartesian descent, ``max_step=0.01``) -> close gripper -> retract
       -> pre-place -> place -> open -> home. ``vel_scale=0.5``,
       ``acc_scale=0.5``, ``planning_time=2.0``, ``planning_attempts=10``.


Data Flow
^^^^^^^^^

**Simulation mode** (Gazebo)::

   MoveIt2 (OMPL/CHOMP/Pilz)
     -> FollowJointTrajectory action
       -> simulation_action node
         -> /gazebo_position_controller/commands (Float64MultiArray, 8 values)
           -> Gazebo gz_ros2_control
             -> /joint_states -> MoveIt2

**Hardware-in-the-Loop (Gazebo + WMX ROS2)**::

   MoveIt2 (OMPL/CHOMP/Pilz)
     -> FollowJointTrajectory action
       -> WMX ROS2 action server -> EtherCAT -> Dobot CR3A
       -> simulation_action -> Gazebo (digital twin)
     <- /joint_states <- WMX ROS2

**Real Robot (WMX ROS2 only)**::

   MoveIt2 (OMPL/CHOMP/Pilz)
     -> FollowJointTrajectory action
       -> WMX ROS2 action server -> EtherCAT -> Dobot CR3A
     <- /joint_states <- WMX ROS2


Prerequisites
-------------

Hardware
^^^^^^^^

- Intel x86_64 PC (Ubuntu 22.04 or 24.04)
- Dobot CR3A 6-DOF collaborative robot arm
- WMX3 EtherCAT motion controller (for physical robot operation)
- Intel RealSense camera (for Stages 2 and 3 -- mounted on Link6)

Software
^^^^^^^^

- ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- MoveIt2 (``moveit_ros_planning_interface``, ``moveit_msgs``)
- Gazebo (``ros_gz_sim``, ``gz_ros2_control``) for simulation mode
- CycloneDDS middleware (``rmw_cyclonedds_cpp``)
- LMX (WMX3 runtime) installed at ``/opt/lmx/`` for hardware control
- Python packages: ``pyapriltags``, ``scipy`` (for Stage 2)
- ``realsense2_camera`` ROS 2 package (for Stages 2 and 3)


Installation
------------

1. **Create the workspace:**

   .. code-block:: bash

      mkdir -p ~/workspaces/movensys_intel_ws/src
      cd ~/workspaces/movensys_intel_ws/src

2. **Clone the repositories:**

   .. code-block:: bash

      git clone git@bitbucket.org:mvs_app/movensys_intel_manipulator.git
      git clone git@bitbucket.org:mvs_app/movensys_manipulator_intel_config.git
      git clone git@bitbucket.org:mvs_app/wmx_ros2_message.git
      git clone git@bitbucket.org:mvs_app/wmx_ros2_package.git

3. **Install ROS 2 dependencies:**

   .. code-block:: bash

      cd ~/workspaces/movensys_intel_ws
      rosdep install --from-paths src --ignore-src -r -y

4. **Install Python dependencies (for AprilTag detection):**

   .. code-block:: bash

      pip install pyapriltags scipy

5. **Configure the environment (add to** ``~/.bashrc`` **):**

   .. code-block:: bash

      source /opt/ros/${ROS_DISTRO}/setup.bash
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

6. **Build the workspace:**

   .. code-block:: bash

      cd ~/workspaces/movensys_intel_ws
      colcon build --symlink-install
      source install/setup.bash


Configuration
-------------

MoveIt2 Configuration
^^^^^^^^^^^^^^^^^^^^^

The MoveIt2 configuration is in ``movensys_manipulator_moveit_config/config/``:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - File
     - Description
   * - ``movensys_manipulator.srdf``
     - Arm group ``movensys_manipulator_arm`` (chain ``Link0`` -> ``Link6``),
       end-effector group ``movensys_manipulator_eef`` (bracket + gripper),
       named states: ``initial`` (ready pose), ``zero`` (all zeros), ``test``.
       Full collision disable matrix for adjacent and non-colliding link pairs.
   * - ``movensys_manipulator.urdf.xacro``
     - MoveIt URDF: includes the description xacro and adds
       ``mock_components/GenericSystem`` ros2_control for planning without
       hardware.
   * - ``joint_limits.yaml``
     - All joints: ``max_velocity=2.5`` rad/s,
       ``max_acceleration=2.5`` rad/s\ :sup:`2`.
   * - ``kinematics.yaml``
     - KDL kinematics solver: resolution 0.005, timeout 0.1 s.
   * - ``moveit_controllers.yaml``
     - ``MoveItSimpleControllerManager`` with
       ``movensys_manipulator_arm_controller`` (``FollowJointTrajectory``
       action for ``joint1`` through ``joint6``).
       ``allowed_execution_duration_scaling=1.0``,
       ``allowed_start_tolerance=0.01``.
   * - ``sensors_3d.yaml``
     - OctoMap sensor plugin:
       ``occupancy_map_monitor/PointCloudOctomapUpdater`` subscribing to
       ``/camera_hand/depth/color/points``, ``max_range=2.0``,
       ``max_update_rate=1.0``, ``padding_offset=0.03``.

RealSense Camera Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Two RealSense configurations are provided in ``movensys_manipulator_moveit_config/config/``:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - File
     - Description
   * - ``realsense_hand.yaml``
     - Color only: 640x480 at 15 fps. Depth disabled. Used for AprilTag
       detection (Stage 2).
   * - ``realsense_hand_depth.yaml``
     - Color + depth: 640x480 at 15 fps for both streams. Pointcloud and
       depth alignment enabled. Used for OctoMap (Stage 3).

Gazebo Simulation Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Gazebo controller is configured in ``movensys_manipulator_description/urdf/control.yaml``:

- Controller manager update rate: **200 Hz**
- ``gazebo_position_controller``: ``JointGroupPositionController`` for all 8
  joints (6 arm + 2 gripper) with per-joint PID gains
- ``joint_state_broadcaster``: publishes ``/joint_states``

AprilTag Configuration
^^^^^^^^^^^^^^^^^^^^^^

The AprilTag configuration for the Intel platform is in
``movensys_manipulator_intel_config/config/apriltag_config.yaml``:

- Tag family: ``tag36h11``
- Tag size: 0.04 m (40 mm)
- Minimum detections for stability: 5
- Detector parameters: ``nthreads=2``, ``quad_decimate=2.0``,
  ``refine_edges=true``
- Camera mount: Link6 parent, z offset 0.04 m, roll and yaw 90 degrees


Launch Files
^^^^^^^^^^^^

**movensys_manipulator_description** launches (Gazebo simulation):

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Launch File
     - Description
   * - ``simulation_1.launch.py``
     - Full Gazebo simulation: spawns robot, loads
       ``joint_state_broadcaster`` and ``gazebo_position_controller``,
       bridges ``/clock``.
   * - ``hil_1.launch.py``
     - Hardware-in-the-loop: Gazebo with ``gazebo_position_controller``
       only (no ``joint_state_broadcaster`` -- WMX ROS2 provides
       ``/joint_states``).
   * - ``demo_1.launch.py``
     - Identical to ``hil_1.launch.py``. Used for real robot demos.

**movensys_manipulator_moveit_config** launches:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Launch File
     - Description
   * - ``movensys_manipulator_moveit.launch.py``
     - MoveIt2: ``move_group`` node with OMPL/CHOMP/Pilz planners,
       ``robot_state_publisher``, RViz with MotionPlanning plugin.
   * - ``camera_hand.launch.py``
     - RealSense camera driver with configurable camera mount TF
       (Link6 -> camera_hand_link). Default: z offset 0.05 m.
   * - ``stage1_trajectory.launch.py``
     - Launches ``stage1_trajectory_cpp`` with MoveIt config parameters.
   * - ``stage2_apriltag.launch.py``
     - Launches ``stage2_apriltag_cpp`` with MoveIt config parameters.
   * - ``stage3_octomap.launch.py``
     - Full OctoMap pipeline: RealSense (depth + pointcloud),
       camera mount TF, ``move_group`` with OctoMap
       (``octomap_resolution=0.025``), RViz, and ``stage3_octomap_cpp``
       demo node. Configurable ``launch_demo`` argument.

**movensys_manipulator_intel_config** launches:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Launch File
     - Description
   * - ``bringup.launch.py``
     - System bringup: includes MoveIt2 launch and WMX ROS2 driver
       (``wmx_ros2_manipulator_dobot_cr3a.launch.py``). Args:
       ``launch_wmx``, ``launch_rviz``, ``use_sim_time``.
   * - ``moveit_only.launch.py``
     - MoveIt2 with ``joint_state_publisher_gui`` (no hardware).
       For testing planning without hardware.
   * - ``trajectory_demo.launch.py``
     - Launches ``trajectory_demo_cpp`` with MoveIt config.
   * - ``pick_and_place.launch.py``
     - Launches ``pick_and_place_cpp`` with MoveIt config.
   * - ``bringup_with_vision.launch.py``
     - Full stack: MoveIt2 + WMX driver + RealSense + AprilTag detection.
       Args: ``tag_size`` (default 0.04), ``tag_family`` (tag36h11),
       ``camera_serial``, ``launch_vision``.
   * - ``apriltag_detection.launch.py``
     - RealSense camera + AprilTag detector + camera mount static TF.
       Configurable: color/depth resolution, camera serial, tag parameters,
       camera mount position (parent: Link6, default z: 0.04 m).
   * - ``webcam_apriltag.launch.py``
     - USB webcam AprilTag detection for testing without RealSense.


Usage
-----

Stage 1: Basic Trajectory
^^^^^^^^^^^^^^^^^^^^^^^^^

Stage 1 validates motion planning and execution with MoveIt2. It exercises all
motion types: absolute Cartesian, relative base/tool Cartesian, joint movement,
and gripper control.

**1a. Simulation (Gazebo):**

Terminal 1 -- Start Gazebo simulation:

.. code-block:: bash

   ros2 launch movensys_manipulator_description simulation_1.launch.py

Terminal 2 -- Start the simulation bridge:

.. code-block:: bash

   ros2 run movensys_manipulator_moveit_config simulation_action

Terminal 3 -- Start MoveIt2:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config movensys_manipulator_moveit.launch.py use_sim_time:=true

Terminal 4 -- Run Stage 1 trajectory:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config stage1_trajectory.launch.py use_sim_time:=true

**1b. Hardware-in-the-Loop (Gazebo + WMX ROS2):**

Terminal 1 -- Start Gazebo HIL:

.. code-block:: bash

   ros2 launch movensys_manipulator_description hil_1.launch.py

Terminal 2 -- Start WMX ROS2 driver:

.. code-block:: bash

   sudo --preserve-env ros2 launch wmx_ros2_package wmx_ros2_manipulator_dobot_cr3a.launch.py

Terminal 3 -- Start MoveIt2:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config movensys_manipulator_moveit.launch.py

Terminal 4 -- Run Stage 1 trajectory:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config stage1_trajectory.launch.py

**1c. Real Robot (WMX ROS2 only):**

Terminal 1 -- Start Gazebo for visualization:

.. code-block:: bash

   ros2 launch movensys_manipulator_description demo_1.launch.py

Terminal 2 -- Start WMX ROS2 driver:

.. code-block:: bash

   sudo --preserve-env ros2 launch wmx_ros2_package wmx_ros2_manipulator_dobot_cr3a.launch.py

Terminal 3 -- Start MoveIt2:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config movensys_manipulator_moveit.launch.py

Terminal 4 -- Run Stage 1 trajectory:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config stage1_trajectory.launch.py

**Alternative demos from movensys_manipulator_intel_config:**

Using the bringup launch (combines MoveIt2 + WMX driver):

.. code-block:: bash

   # Terminal 1: Bringup (MoveIt2 + WMX ROS2 driver)
   sudo --preserve-env ros2 launch movensys_manipulator_intel_config bringup.launch.py

   # Terminal 2: Trajectory demo
   ros2 launch movensys_manipulator_intel_config trajectory_demo.launch.py

   # -- OR --

   # Terminal 2: Pick-and-place demo
   ros2 launch movensys_manipulator_intel_config pick_and_place.launch.py


Stage 2: AprilTag Pick-and-Place
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Stage 2 adds vision-guided manipulation using Intel RealSense camera and
CPU-based AprilTag detection via ``pyapriltags``. The robot detects
``tag36h11`` tags, performs visual servoing to center the end-effector over
each tag, picks the object, and places it in a predefined box position.

The visual servoing loop works as follows:

1. Move to scan pose
2. Look up TF from ``camera_hand_color_optical_frame`` to ``tag36h11:<id>``
3. If tag offset exceeds tolerance (0.03 m position, 0.05 rad orientation),
   execute ``relativeToolEefCartesian`` correction
4. Repeat until converged
5. Apply tag-to-target offset (``x=0.01``, ``y=-0.065``)
6. Descend 0.21 m, close gripper, retract
7. Move to box position, release

**Prerequisites:**

.. code-block:: bash

   pip install pyapriltags scipy
   sudo apt install ros-${ROS_DISTRO}-realsense2-camera

**2a. Hardware-in-the-Loop:**

Terminal 1 -- Start Gazebo HIL:

.. code-block:: bash

   ros2 launch movensys_manipulator_description hil_1.launch.py

Terminal 2 -- Start WMX ROS2 driver:

.. code-block:: bash

   sudo --preserve-env ros2 launch wmx_ros2_package wmx_ros2_manipulator_dobot_cr3a.launch.py

Terminal 3 -- Start MoveIt2:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config movensys_manipulator_moveit.launch.py

Terminal 4 -- Start RealSense camera:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config camera_hand.launch.py

Terminal 5 -- Start AprilTag detector:

.. code-block:: bash

   ros2 run movensys_manipulator_moveit_config apriltag_detector.py

Terminal 6 -- Run Stage 2 pick-and-place:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config stage2_apriltag.launch.py

**2b. Real Robot:**

Same as 2a but replace the Gazebo HIL terminal with the demo launch:

.. code-block:: bash

   ros2 launch movensys_manipulator_description demo_1.launch.py

**Alternative (using bringup_with_vision):**

.. code-block:: bash

   # Terminal 1: Full stack (MoveIt2 + WMX + RealSense + AprilTag)
   sudo --preserve-env ros2 launch movensys_manipulator_intel_config bringup_with_vision.launch.py

   # Terminal 2: Stage 2 pick-and-place
   ros2 launch movensys_manipulator_moveit_config stage2_apriltag.launch.py


Stage 3: OctoMap Obstacle Avoidance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Stage 3 adds 3D obstacle awareness using OctoMap (the CPU-based equivalent of
NVIDIA NvBlox). A depth camera mounted on the end-effector builds a 3D voxel
map of the workspace. MoveIt2 OMPL planner uses this map to generate
collision-free trajectories.

The demo runs in four phases:

1. **Workspace scan** -- Move to 4 scan poses (overview, left, right, center),
   dwell 3 seconds at each to accumulate depth data into OctoMap
2. **Obstacle-aware Cartesian** -- Execute Cartesian moves through the
   workspace, OMPL plans around obstacles
3. **Obstacle-aware joint** -- Joint-space movements considering OctoMap
   obstacles
4. **Relative movements** -- Relative base-frame moves with obstacle avoidance

**Prerequisites:**

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-realsense2-camera

**All-in-one launch (includes camera, OctoMap, MoveIt2, demo):**

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config stage3_octomap.launch.py

The ``stage3_octomap.launch.py`` launch starts:

1. RealSense camera with depth + pointcloud
   (config: ``realsense_hand_depth.yaml``)
2. Static TF for camera mount (Link6 -> camera_hand_link)
3. Robot state publisher
4. MoveIt2 ``move_group`` with OctoMap enabled
   (``octomap_resolution=0.025``, ``octomap_frame=world_manipulator``)
5. RViz with OctoMap visualization
6. ``stage3_octomap_cpp`` demo node

To skip the demo node and use interactively:

.. code-block:: bash

   ros2 launch movensys_manipulator_moveit_config stage3_octomap.launch.py launch_demo:=false

With WMX ROS2 hardware, start the driver first:

.. code-block:: bash

   # Terminal 1: WMX ROS2 driver
   sudo --preserve-env ros2 launch wmx_ros2_package wmx_ros2_manipulator_dobot_cr3a.launch.py

   # Terminal 2: OctoMap pipeline
   ros2 launch movensys_manipulator_moveit_config stage3_octomap.launch.py


Comparison: Isaac vs Intel
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 37 38

   * - Feature
     - Isaac (Jetson Orin)
     - Intel (x86_64)
   * - Motion planner
     - cuMotion (GPU-accelerated)
     - OMPL / CHOMP / Pilz (CPU)
   * - AprilTag detection
     - ``isaac_ros_apriltag`` (GPU)
     - ``pyapriltags`` (CPU)
   * - 3D reconstruction
     - NvBlox (GPU TSDF)
     - OctoMap (CPU voxels)
   * - Depth sensor topic
     - ``/front/stereo_camera/...``
     - ``/camera_hand/depth/color/points``
   * - Simulator
     - Isaac Sim (Omniverse)
     - Gazebo (Open Source)
   * - Trajectory bridge
     - ``/joint_command`` (Float64MultiArray)
     - ``/gazebo_position_controller/commands`` (Float64MultiArray)
   * - Bridge values
     - 6 values (arm only)
     - 8 values (6 arm + 2 gripper)


How It Connects to WMX ROS2
----------------------------

The Intel manipulator workspace follows the same integration pattern as all
WMX ROS2 applications. The connection points are:

**FollowJointTrajectory action** -- MoveIt2 sends planned trajectories to the
WMX ROS2 action server at
``/movensys_manipulator_arm_controller/follow_joint_trajectory``. In simulation,
``simulation_action`` handles this; with hardware, the WMX ROS2 driver's action
server takes over.

**Joint states** -- The ``/joint_states`` topic (``sensor_msgs/JointState``)
provides real-time joint feedback. In simulation, Gazebo publishes via
``joint_state_broadcaster``; with hardware, the WMX ROS2 driver publishes from
EtherCAT feedback.

**Gripper control** -- The ``/wmx/set_gripper`` service (``std_srvs/SetBool``)
controls the gripper via EtherCAT digital I/O. In simulation,
``simulation_action`` handles this by setting gripper positions in the 8-value
command array (indices 6 and 7: ``0.045`` = closed, ``0.000`` = open).

**WMX ROS2 driver launch** -- The bringup launch file includes the WMX ROS2
driver via:

.. code-block:: python

   IncludeLaunchDescription(
       PythonLaunchDescriptionSource([
           FindPackageShare('wmx_ros2_package'),
           '/launch/wmx_ros2_manipulator_dobot_cr3a.launch.py'
       ])
   )

This requires ``sudo --preserve-env`` because the WMX3 EtherCAT driver needs
kernel-level access for real-time communication.

**Switching between simulation and hardware** requires no code changes -- only
the launch configuration changes:

- ``launch_wmx:=true`` enables the WMX ROS2 driver (hardware)
- ``launch_wmx:=false`` disables it (simulation with ``simulation_action``)
- ``use_sim_time:=true`` enables Gazebo clock synchronization
