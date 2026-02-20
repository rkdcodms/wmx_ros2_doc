Install ROS2
============

ROS2 Humble (Ubuntu 22.04)
---------------------------

Follow the official guide:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Quick summary:

.. code-block:: bash

   # Set locale
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   # Setup sources
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install
   sudo apt update && sudo apt upgrade
   sudo apt install ros-humble-desktop

   # Source
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc

ROS2 Jazzy (Ubuntu 24.04)
--------------------------

Follow the official guide:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Quick summary:

.. code-block:: bash

   # Set locale
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   # Setup sources
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install
   sudo apt update && sudo apt upgrade
   sudo apt install ros-jazzy-desktop

   # Source
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc

Verify ROS2
------------

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

In another terminal:

.. code-block:: bash

   ros2 run demo_nodes_cpp listener

If messages are being received, ROS2 is working correctly.
