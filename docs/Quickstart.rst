==========
Quickstart
==========

.. _minimal-example:

Minimal example
---------------
This is a minimal example that shows how to use the collision avoidance algorithm. Here the own ship (OS)
is facing an head-on encounter with one target ship (TS).

.. literalinclude:: ../asv_path_planner/min_example.py
    :name: min-example
    :caption: min_example.py
    :linenos:

1. Import the classes VO and TS from the velobst_class.py and ts_class.py as dependencies. In this example
the .py files are in the same folder as the minimal example.

.. code-block:: python
    :name: imports
    :linenos:
    :lineno-start: 7
    
    from velobst_class import VO
    from ts_class import TS

2. Define the properties of the own ship (OS): current velocity of the OS and the desired velocity of the OS.
The velocity is always a vector consiting of speed and orientation angle.

.. code-block:: python
    :name: vo
    :linenos:
    :lineno-start: 11

    vel_OS = [3.0, 90]
    vel_des = [3.0, 90]
    os = [vel_OS, vel_des]

3. Define the properties of the target ship (TS) obtained by sensors or the automatic identification system (AIS) as an object of the TS class:
relative position of TS to OS, length and width of TS, current speed and orientation angle (velocity) of TS

.. code-block:: python
    :name: ts
    :linenos:
    :lineno-start: 17

    ts = TS()
    ts.pos = [100,0]
    ts.length = 6.0
    ts.width = 3.0
    ts.speed = 10.0
    ts.ang = 270

4. Add all objects of the TS class (all obstacles) in a list:

.. code-block:: python
    :name: all-ts
    :linenos:
    :lineno-start: 25

    all_ts = [ts]

5. Declare and object of the VO class with initial parameters for the collision avoidance algorithm. The 
initial parameters are: length and width of OS, maximum speed of OS, time to collision, time threshhold to collision,
safety factor, uncertainties in speed and orientation of TS, resolution of speed and orientation of the velocity space.

.. code-block:: python
    :name: vo-init
    :linenos:
    :lineno-start: 29

    vo = VO(3.0,1.5,5.0,120, 60, 5, 0.15, 1.5, 0.25, 3)

6. Use the calc_vel_final() function of the VO class to calculate the new velocity to avoid a collision. 

.. code-block:: python
    :name: new-vel
    :linenos:
    :lineno-start: 32

    new_vel = vo.calc_vel_final(all_ts, os, False)

.. _marus:

Evaluation of the collision avoidance algorithm with MARUSimulator
------------------------------------------------------------------

1. Build and source colcon workspace. Position yourself in the root directory of the workspace (e.g. ``cd ~/ros_ws/src``) and run:

.. code:: bash

    $ source /opt/ros/{ROS_DISTRO}/setup.bash
    $ source ~/ros_ws/install/setup.bash
    $ colcon build

2. Run the ROS server in a new terminal with:

.. code:: bash

    $ source /opt/ros/{ROS_DISTRO}/setup.bash
    $ ros2 launch grpc_ros_adapter ros2_server_launch.py

3. Open the marus-example project in Unity and select the example scene from ``Assets/Scenes/Head-on_Left_Right_fast.unity``. 

.. _marussimu_1:
.. figure:: img/MARUS_1.png
    :width: 100%
    :align: center

    Image of the unity user interface and the console output that verifies the ROS connection


4. Start the scene by pressing the play button. Make sure the connection to ROS is established by checking the console output of Unity.

.. _marussimu_2:
.. figure:: img/MARUS.png
    :width: 100%
    :align: center

    Image of the unity user interface and the console output that verifies the ROS connection

5. For each scene a seperate folder can be found in ``Assets/Scenes/``. Here are the results of the first simulation stored as well as the parameters for the collision avoidance as a ``params.yaml`` file. Copy and paste the content of the .yaml file for your loaded unity scene and insert it into ``asv_path_planner/config/params.yaml``. Make sure to save the file again so that the "Date modified" is up-to-date. (To change parameters like speed of OS and TSs or detection range of the OS for your own simulation scenes, change the parameters inside the ``/asv_path_planner/config/params.yaml`` file.)

6. Run the collision avoidance algorithm with:

.. code:: bash

    $ ros2 run asv_path_planner ros_script --ros-args --params-file ~/ros2_ws/src/asv_path_planner/config/params.yaml

6. (Optional) After changing the params.yaml file the package has to be build once more. Position yourself in the root directory of the workspace and run:

.. code:: bash

    $ colcon build --packages-above asv_path_planner

7. (Optional) Run the collision avoidance algorithm with:

.. code:: bash

    $ ros2 launch asv_path_planner simulation.launch.py

8. All data collected during the simulation and all figures are store after shutdown in ``asv_path_planner/Simulation output``.
