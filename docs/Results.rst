=======
Results
=======
This section presents some results of the VO algorithm tested in the MARUSimulator. Several different encounters with different amount of TS has been tested. In the simulation
all TSs do not take any action even if they are supposed to. The speed of the TSs is constant and the OS can in most situations only detect TSs that are at a distance of 50 m or closer.

The collision avoidance algorithm was tested beforehand in a less realistic and simpler simulation. In the turtlesim simulation provided by ROS. This was used to test the basic functionality of the algorithm and
is not part of this documentation. 

For each simulation the distance between the ships, the trajectory of the ships, and the current and desired speed and course of the OS are recorded and plotted. Also it is checked if at any point during the maneuver
the OS enters the defined :ref:`safety area <safety area>` around the TS. The avoidance maneuver is only successful if the safety area is untouched. 

Head-on
-------
For the head-on scenario four situations are presented here which shows the effect of different speeds of the TS on the avoidance maneuver as well as different detection ranges of the OS.

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_1
    

    .. image:: img/Head-on_1/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_1/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_1/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_1/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_2
    

    .. image:: img/Head-on_2/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_2/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_2/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_2/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_3
    

    .. image:: img/Head-on_3/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_3/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_3/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_3/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_4
    

    .. image:: img/Head-on_4/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_4/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_4/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_4/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS


Right-crossing
--------------
For the right-crossing scenario three situation are displayed here in which each TS is coming from a different direction towards the OS. 

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Right-crossing_1
    

    .. image:: img/Right-crossing_1/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Right-crossing_1/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Right-crossing_1/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Right-crossing_1/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Right-crossing_2
    

    .. image:: img/Right-crossing_2/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Right-crossing_2/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Right-crossing_2/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Right-crossing_2/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Right-crossing_3
    

    .. image:: img/Right-crossing_3/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Right-crossing_3/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Right-crossing_3/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Right-crossing_3/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

Overtaking + static obstacle
----------------------------
For the overtaking scenarios two situations are displayed here. One where the TS is moving slower then the OS on the same course over ground and one where the TS is not moving at all and so being a static obstacle.

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Overtaking_1


    .. image:: img/Overtaking_1/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Overtaking_1/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Overtaking_1/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Overtaking_1/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Overtaking_2


    .. image:: img/Overtaking_2/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Overtaking_2/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Overtaking_2/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Overtaking_2/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS


Left-crossing
-------------
For the left-crossing scenario three situation are displayed here in which each TS is coming from a different direction towards the OS. 

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Left-crossing_1


    .. image:: img/Left-crossing_1/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Left-crossing_1/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Left-crossing_1/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Left-crossing_1/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS
    

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Left-crossing_2


    .. image:: img/Left-crossing_2/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Left-crossing_2/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Left-crossing_2/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Left-crossing_2/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Left-crossing_3


    .. image:: img/Left-crossing_3/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Left-crossing_3/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Left-crossing_3/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Left-crossing_3/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS


Being-overtaken
---------------
For the being-overtaken scenario two situations are displayed here. Both situations are similar in the course of the OS and TS but differs in the speed of the involved ships.

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 0px
    :subcaptions: above
    :name: Being-overtaken_1
    

    .. image:: img/Being-overtaken_1/Distance_new.svg
        :alt: (A)

    .. image:: img/Being-overtaken_1/Speed.svg
        :alt: (B)

    .. image:: img/Being-overtaken_1/Orientation.svg
        :alt: (C)
    
    .. image:: img/Being-overtaken_1/Trajectory_new.svg
        :alt: (D)


    Results of the simulation for a being-overtaken situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 0px
    :subcaptions: above
    :name: Being-overtaken_2
    

    .. image:: img/Being-overtaken_2/Distance_new.svg
        :alt: (A)

    .. image:: img/Being-overtaken_2/Speed.svg
        :alt: (B)

    .. image:: img/Being-overtaken_2/Orientation.svg
        :alt: (C)
    
    .. image:: img/Being-overtaken_2/Trajectory_new.svg
        :alt: (D)


    Results of the simulation for a being-overtaken situation with a fast moving OS

Multiple ship encounters
------------------------
For multiple ship encounters several scenes have been simulated. Head-on scenarios with multiple TSs and Head-on scnearios with additional crossing encounters.

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_double


    .. image:: img/Head-on_double/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_double/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_double/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_double/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_double_2


    .. image:: img/Head-on_double_2/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_double_2/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_double_2/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_double_2/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_Left-crossing


    .. image:: img/Head-on_Left-crossing/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_Left-crossing/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_Left-crossing/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_Left-crossing/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_Right-crossing


    .. image:: img/Head-on_Right-crossing/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_Right-crossing/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_Right-crossing/Orientation-cropped_new.svg
        :alt: (C)
    
    .. image:: img/Head-on_Right-crossing/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS

.. subfigure:: AB|CD
    :layout-sm: A|B|C|D
    :align: center
    :gap: 2px
    :subcaptions: above
    :name: Head-on_Left_Right_1


    .. image:: img/Head-on_Left_Right_1/Distance-cropped-1.svg
        :alt: (A)

    .. image:: img/Head-on_Left_Right_1/Speed-cropped-1.svg
        :alt: (B)

    .. image:: img/Head-on_Left_Right_1/Orientation-cropped-1.svg
        :alt: (C)
    
    .. image:: img/Head-on_Left_Right_1/Trajectory-cropped-1.svg
        :alt: (D)


    Results of the simulation for a head-on situation with a slow moving OS



