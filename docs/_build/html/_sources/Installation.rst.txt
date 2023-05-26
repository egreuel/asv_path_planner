============
Installation
============
In the following two types of installations are displayed. The first one for the minimal example, 
which allows to use the algorithm in any personal project. The second describes the installation for 
the use of the MARUSimulator with ROS2 where the collision avoidance algorithm can be tested with up to 3
target ships. With small changes in the code it can be tested with even more target ships.

Installation minimal example
----------------------------
1. This installation guide is valid for linux and windows.
2. Clone asv_path_planner repository to your preferred folder:

.. code:: bash

    $ git clone https://github.com/egreuel/asv_path_planner.git

3. Change directory to asv_path_planner where you should find a requirements.txt file. Install python dependencies by running:

.. code:: bash

    $ pip install -r requirements.txt

With this all requirements for the minimal example are done and you can check out the quickstart guide for the :ref:`minimal-example`.

Installation for the use of the MARUSimulator with ROS2
---------------------------------------------------------
1. This installation guide is valid for linux and windows.
2. For the installation for MARUSimulator in Unity and the ROS backend please follow the instructions on: https://github.com/MARUSimulator/marus-example/wiki/2.-Installation. Please make sure to use ROS2 humble or galactic for this package. **Instead of using the marus-example repository from git@github.com:MARUSimulator/marus-example.git please use the following:**

.. code:: bash

    $ git clone https://github.com/egreuel/marus-example.git

The example scene for the test of the collision avoidance algorithm with 3 target ships then can be found in ``Assets/Scenes/Head-on_Left_Right_fast.unity``.

3. Clone asv_path_planner repository in the ``src`` folder of the ROS workspace (e.g. ~/ros_ws/src):

.. code:: bash

    $ git clone https://github.com/egreuel/asv_path_planner.git

4. Change directory to asv_path_planner where you should find a requirements.txt file. Install the python dependencies by running:

.. code:: bash

    $ pip install -r requirements.txt

With this the installation is completed and and you can hop into the :ref:`quickstart guide <marus>` and use the MARUSimulator to test the algorithm.