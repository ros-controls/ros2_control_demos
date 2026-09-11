:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_19/doc/userdoc.rst

.. _ros2_control_demos_example_19_userdoc:

Example 19: DiffBot with Optionally Chained controllers using ``launch_ros2_control``
=====================================================================================

This example demonstrates the use of ``launch_ros2_control`` on several *DiffBot* scenarios.
This is done in three parts:

1. Reimplement :ref:`Example 2: DiffBot <ros2_control_demos_example_2_userdoc>`
2. Reimplement :ref:`Example 16: DiffBot with Chained Controllers <ros2_control_demos_example_16_userdoc>`
3. *DiffBot* with Conditional Chained Controllers

All these launch files are available as ``XML``, ``YAML`` and ``Python`` launch files.

.. include:: ../../doc/run_from_docker.rst


Tutorial steps
--------------
This example is split up in three parts to build up to the optional chained controller setup.

.. contents:: Parts
   :depth: 2
   :local:


Part 1: Reimplementing :ref:`Example 2 <ros2_control_demos_example_2_userdoc>`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. To start *DiffBot* in a configuration equivalent to Example 2 open a terminal, source your ROS2-workspace and the following launch file with

   .. tabs::

      .. group-tab:: XML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2.launch.xml

      .. group-tab:: YAML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2.launch.yaml

      .. group-tab:: Python

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz* in a similar fashion to :ref:`Example 2 <ros2_control_demos_example_2_userdoc>`.
   To validate the full tutorial of *example_2* can be followed starting from step 3.

2. The used launch file looks as follows:

   .. tabs::

      .. group-tab:: XML

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.xml
            :language: xml

      .. group-tab:: YAML

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.yaml
            :language: yaml

      .. group-tab:: Python

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.py
            :language: python

   The arguments are defined at the start of the launch file.
   These are both defined with a default to make them optional.

   .. tabs::

      .. group-tab:: XML

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            The ``value`` attribute is separated over multiple lines in order to improve readability.

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.xml
            :language: xml
            :lines: 8-13
            :dedent:

         Then the controller config filepath is stored in the ``controller_config_file`` launch configuration after which the ``robot_state_publisher`` and ``controller_manager`` nodes are started.

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.xml
            :language: xml
            :lines: 15-23
            :dedent:

         Then the ``diffbot_base_controller`` and the ``joint_state_broadcaster`` are spawned using the ``<spawn_controller>`` action.
         The two controllers are spawned with a single spawner, and only the ``cmd_vel`` topic of the ``diffbot_base_controller``` is remapped.
         In this example the parameters for the ``joint_state_broadcaster`` have been omitted, as they are already loaded when loading the parameters for the ``diffbot_base_controller``

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.xml
            :language: xml
            :lines: 24-34
            :dedent:


      .. group-tab:: YAML

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            The ``value`` attribute is separated over multiple lines in order to improve readability.

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.yaml
            :language: yaml
            :lines: 12-18
            :dedent:

         Then the controller config filepath is stored in the ``controller_config_file`` launch configuration after which the ``robot_state_publisher`` and ``controller_manager`` nodes are started.

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.yaml
            :language: yaml
            :lines: 20-36
            :dedent:

         Then the ``diffbot_base_controller`` and the ``joint_state_broadcaster`` are spawned using the ``spawn_controller`` action.
         The two controllers are spawned with a single spawner, and only the ``cmd_vel`` topic of the ``diffbot_base_controller``` is remapped.
         In this example the parameters for the ``joint_state_broadcaster`` have been omitted, as they are already loaded when loading the parameters for the ``diffbot_base_controller``

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.yaml
            :language: yaml
            :lines: 38-47
            :dedent:


      .. group-tab:: Python

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            An extra ``demo_package_dir`` variable is defined, containing the substitution for the ``share`` directory path of *example_2*.

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.py
            :language: python
            :lines: 49-63
            :dedent:

         Then the controller config filepath is stored in the ``controller_config_file`` launch configuration after which the ``robot_state_publisher`` and ``controller_manager`` nodes are started.

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.py
            :language: python
            :lines: 65-90
            :dedent:

         Then the ``diffbot_base_controller`` and the ``joint_state_broadcaster`` are spawned using the ``SpawnControllers`` action.
         The two controllers are spawned with a single spawner, and only the ``cmd_vel`` topic of the ``diffbot_base_controller``` is remapped.
         In this example the parameters for the ``joint_state_broadcaster`` have been omitted, as they are already loaded when loading the parameters for the ``diffbot_base_controller``

         .. literalinclude:: ../bringup/launch/diffbot_example_2.launch.py
            :language: python
            :lines: 92-105
            :dedent:


   All launch files end with starting *RViz*, which has been omitted for brevity.


Part 2: Reimplementing :ref:`Example 16 <ros2_control_demos_example_16_userdoc>`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. To start *DiffBot* in a configuration equivalent to Example 16 open a terminal, source your ROS2-workspace and the following launch file with

   .. tabs::

      .. group-tab:: XML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_16.launch.xml

      .. group-tab:: YAML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_16.launch.yaml

      .. group-tab:: Python

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_16.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz* in a similar fashion to :ref:`Example 16 <ros2_control_demos_example_16_userdoc>`.
   To validate the full tutorial of *example_16* can be followed starting from step 2.

2. The used launch file looks as follows:

   .. tabs::

      .. group-tab:: XML

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.xml
            :language: xml

      .. group-tab:: YAML

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.yaml
            :language: yaml

      .. group-tab:: Python

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.py
            :language: python


   The arguments are defined at the start of the launch file.
   These are both defined with a default to make them optional.

   .. tabs::

      .. group-tab:: XML

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            The ``value`` attribute is separated over multiple lines in order to improve readability.

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.xml
            :language: xml
            :lines: 10-15
            :dedent:

         Then the controller config filepath is stored in the ``controller_config_file`` launch configuration after which the ``robot_state_publisher`` and ``controller_manager`` nodes are started.

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.xml
            :language: xml
            :lines: 17-26
            :dedent:

         Then the ``diffbot_base_controller``, the ``joint_state_broadcaster`` and both PID controllers are spawned using the ``<spawn_controller>`` action.
         The four controllers are spawned with a single spawner, and only the ``cmd_vel`` topic of the ``diffbot_base_controller``` is remapped.
         In this example the parameters for the ``joint_state_broadcaster`` have been omitted, as they are already loaded when loading the parameters for the ``diffbot_base_controller``

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.xml
            :language: xml
            :lines: 27-47
            :dedent:


      .. group-tab:: YAML

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            The ``value`` attribute is separated over multiple lines in order to improve readability.

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.yaml
            :language: yaml
            :lines: 15-22
            :dedent:

         Then the controller config filepath is stored in the ``controller_config_file`` launch configuration after which the ``robot_state_publisher`` and ``controller_manager`` nodes are started.

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.yaml
            :language: yaml
            :lines: 24-40
            :dedent:

         Then the ``diffbot_base_controller``, the ``joint_state_broadcaster`` and both PID controllers are spawned using the ``spawn_controller`` action.
         The four controllers are spawned with a single spawner, and only the ``cmd_vel`` topic of the ``diffbot_base_controller``` is remapped.
         In this example the parameters for the ``joint_state_broadcaster`` have been omitted, as they are already loaded when loading the parameters for the ``diffbot_base_controller``

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.yaml
            :language: yaml
            :lines: 41-61
            :dedent:


      .. group-tab:: Python

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            An extra ``demo_package_dir`` variable is defined, containing the substitution for the ``share`` directory path of *example_16*.

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.py
            :language: python
            :lines: 55-69
            :dedent:

         Then the controller config filepath is stored in the ``controller_config_file`` launch configuration after which the ``robot_state_publisher`` and ``controller_manager`` nodes are started.

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.py
            :language: python
            :lines: 71-97
            :dedent:

         Then the ``diffbot_base_controller``, the ``joint_state_broadcaster`` and both PID controllers are spawned using the ``SpawnControllers`` action.
         The four controllers are spawned with a single spawner, and only the ``cmd_vel`` topic of the ``diffbot_base_controller``` is remapped.
         In this example the parameters for the ``joint_state_broadcaster`` have been omitted, as they are already loaded when loading the parameters for the ``diffbot_base_controller``

         .. literalinclude:: ../bringup/launch/diffbot_example_16.launch.py
            :language: python
            :lines: 99-124
            :dedent:


   All launch files end with starting *RViz*, which has been omitted for brevity.


Part 3: *DiffBot* with Conditional Chained Controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. To start the conditional *DiffBot* example in a configuration equivalent to Example 2 open a terminal, source your ROS2-workspace and the following launch file with:

   .. tabs::

      .. group-tab:: XML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2_16.launch.xml use_pid:=False

      .. group-tab:: YAML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2_16.launch.yaml use_pid:=False

      .. group-tab:: Python

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2_16.launch.py use_pid:=False

   The launch file loads and starts the robot hardware, controllers and opens *RViz* in a similar fashion to :ref:`Example 2 <ros2_control_demos_example_2_userdoc>`.
   To validate the full tutorial of *example_2* can be followed starting from step 3.


2. Stop the previous launch file by pressing :kbd:`Ctrl-C`.
   To now start the conditional *DiffBot* example in a configuration equivalent to Example 16 in the same terminal start following launch file with:

   .. tabs::

      .. group-tab:: XML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2_16.launch.xml use_pid:=True

      .. group-tab:: YAML

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2_16.launch.yaml use_pid:=True

      .. group-tab:: Python

         .. code-block:: shell

            ros2 launch ros2_control_demo_example_19 diffbot_example_2_16.launch.py use_pid:=True

   The launch file loads and starts the robot hardware, controllers and opens *RViz* in a similar fashion to :ref:`Example 16 <ros2_control_demos_example_16_userdoc>`.
   To validate the full tutorial of *example_16* can be followed starting from step 2.


3. The used launch file looks as follows:

   .. tabs::

      .. group-tab:: XML

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.xml
            :language: xml

      .. group-tab:: YAML

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.yaml
            :language: yaml

      .. group-tab:: Python

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.py
            :language: python

   The strategy used to achieve the toggle-able PID wheel controllers will be summarized here.
   A new launch argument ``use_pid`` is introduced, which when set to some Truthy value, will enable the these extra controllers.
   This is achieved by loading the base parameters from *example_2* as global parameters for the launch file.
   This will make sure they are loaded by each node/controller, **before** the node/controller specific overrides are loaded.
   The parameters for the PID controllers are only loaded when they are spawned, however they can also load parameters for other controllers.
   This can be used to override the ``wheel_name`` parameters of the ``diffbot_base_controller`` to ensure the PID controller command interfaces are used.

   This will now be highlighted in detail.

   .. tabs::

      .. group-tab:: XML

         First, some general launch arguments are defined, after which the ``use_pid`` launch argument is defined.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.xml
            :language: xml
            :lines: 4-10
            :emphasize-lines: 7
            :dedent:

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            The ``value`` attribute is separated over multiple lines in order to improve readability.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.xml
            :language: xml
            :lines: 12-17
            :dedent:

         Then the ``robot_state_publisher`` and ``controller_manager`` nodes are started.
         The ``controller_manager`` loads the parameters from the PID-less setup.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.xml
            :language: xml
            :lines: 19-26
            :emphasize-lines: 3
            :dedent:

         After which the PID-less controller config is loaded into the global parameters.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.xml
            :language: xml
            :lines: 27-29
            :dedent:

         Then all controllers are spawned using a single ``<spawn_controller>`` action.
         The PID controllers are spawned **only** when the ``if`` attribute resolves to a truthy value.
         These controllers load ``diffbot_pid_controllers.yaml`` which get combined with the global parameters from earlier.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.xml
            :language: xml
            :lines: 31-51
            :dedent:


      .. group-tab:: YAML

         First, some general launch arguments are defined, after which the ``use_pid`` launch argument is defined.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.yaml
            :language: yaml
            :lines: 4-18
            :emphasize-lines: 13-
            :dedent:

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. note::
            The ``value`` attribute is separated over multiple lines in order to improve readability.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.yaml
            :language: yaml
            :lines: 20-26
            :dedent:

         Then the ``robot_state_publisher`` and ``controller_manager`` nodes are started.
         The ``controller_manager`` loads the parameters from the PID-less setup.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.yaml
            :language: yaml
            :lines: 27-42
            :emphasize-lines: 7
            :dedent:

         After which the PID-less controller config is loaded into the global parameters.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.yaml
            :language: yaml
            :lines: 43-45
            :dedent:

         Then all controllers are spawned using a single ``spawn_controller`` action.
         The PID controllers are spawned **only** when the ``if`` attribute resolves to a truthy value.
         These controllers load ``diffbot_pid_controllers.yaml`` which get combined with the global parameters from earlier.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.yaml
            :language: yaml
            :lines: 47-67
            :dedent:


      .. group-tab:: Python

         First, some general launch arguments are defined, after which the ``use_pid`` launch argument is defined.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.py
            :language: python
            :lines: 32-55
            :emphasize-lines: 20-
            :dedent:

         Some additional variables for the previously defined launch configurations are defined in addition to a ``IfCondition`` for the use of ``use_pid``.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.py
            :language: python
            :lines: 57-62
            :emphasize-lines: 6
            :dedent:

         After which the robot description is expanded and stored in the ``robot_description_content`` launch configuration.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.py
            :language: python
            :lines: 64-78
            :dedent:

         Then the ``robot_state_publisher`` and ``controller_manager`` nodes are started.
         The ``controller_manager`` loads the parameters from the PID-less setup.

         .. note::
            Additional variables to contain (partial) launch configurations/substitutions are defined throughout the launch file to prevent repetition.
            Most of these will be left out of this analysis for readability.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.py
            :language: python
            :lines: 82-102
            :emphasize-lines: 1-3,10
            :dedent:

         After which the PID-less controller config is loaded into the global parameters.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.py
            :language: python
            :lines: 110-111
            :dedent:

         Then all controllers are spawned using a single ``SpawnController`` action.
         The PID controllers are spawned **only** when the supplied ``condition`` argument resolves to a truthy value.
         These controllers load ``diffbot_pid_controllers.yaml`` which get combined with the global parameters from earlier.

         .. literalinclude:: ../bringup/launch/diffbot_example_2_16.launch.py
            :language: python
            :lines: 113-144
            :dedent:


   All launch files end with starting *RViz*, which has been omitted for brevity.

   The content of the conditionally loaded parameter file ``diffbot_pid_controllers.yaml`` is:

   .. literalinclude:: ../bringup/config/diffbot_pid_controllers.yaml
      :language: yaml


Files used for this demo
------------------------

Files of Part 1: Reimplementing Example 2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
* Launch files:

  * `diffbot_example_2.launch.xml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_2.launch.xml>`__
  * `diffbot_example_2.launch.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_2.launch.yaml>`__
  * `diffbot_example_2.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_2.launch.py>`__

* Controllers yaml: `diffbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/config/diffbot_controllers.yaml>`__
* URDF file: `diffbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/urdf/diffbot.urdf.xacro>`__

  * Description: `diffbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/urdf/diffbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `diffbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/ros2_control/diffbot.ros2_control.xacro>`__

* RViz configuration: `diffbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/rviz/diffbot.rviz>`__

* Hardware interface plugin: `diffbot_system.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/hardware/diffbot_system.cpp>`__


Files of Part 2: Reimplementing Example 16
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
* Launch files:

  * `diffbot_example_16.launch.xml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_16.launch.xml>`__
  * `diffbot_example_16.launch.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_16.launch.yaml>`__
  * `diffbot_example_16.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_16.launch.py>`__

* Controllers yaml: `diffbot_chained_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_16/bringup/config/diffbot_chained_controllers.yaml>`__
* URDF file: `diffbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_16/description/urdf/diffbot.urdf.xacro>`__

  * Description: `diffbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/urdf/diffbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `diffbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_16/description/ros2_control/diffbot.ros2_control.xacro>`__

* RViz configuration: `diffbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/rviz/diffbot.rviz>`__

* Hardware interface plugin: `diffbot_system.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_16/hardware/diffbot_system.cpp>`__


Files of Part 3: *DiffBot* with Conditional Chained Controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
* Launch files:

  * `diffbot_example_2_16.launch.xml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_2_16.launch.xml>`__
  * `diffbot_example_2_16.launch.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_2_16.launch.yaml>`__
  * `diffbot_example_2_16.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/diffbot_example_2_16.launch.py>`__

* Controllers yaml:

  * `diffbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/config/diffbot_controllers.yaml>`__
  * `diffbot_pid_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/config/diffbot_pid_controllers.yaml>`__
* URDF file: `diffbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_16/description/urdf/diffbot.urdf.xacro>`__

  * Description: `diffbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/urdf/diffbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `diffbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_16/description/ros2_control/diffbot.ros2_control.xacro>`__

* RViz configuration: `diffbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/rviz/diffbot.rviz>`__

* Hardware interface plugin: `diffbot_system.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_16/hardware/diffbot_system.cpp>`__


Controllers from this demo
--------------------------

* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
* ``Diff Drive Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/diff_drive_controller>`__): :ref:`doc <diff_drive_controller_userdoc>`
* ``pid_controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/pid_controller>`__): :ref:`doc <pid_controller_userdoc>`
