==========
Quickstart
==========
Here Quickstart guide

Requirements
------------

- ROS2 humble
- Ubuntu 22.04
- Unity 2021.3.15f1
- MARUS simulator
- requirement.txt

Change directory to asv_path_planner and install the requirements from the requirements.txt file:

.. code:: bash
    
    $ pip install -r requirements.txt


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

3. Define the properties of the target ship (TS) obtained by sensors or AIS as an object of the TS class:
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

Integration in ROS2 and use with MARUS simulatior
-------------------------------------------------