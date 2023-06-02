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
For the head-on scenario four situations are presented here which shows the effect of different speeds of the TS on the avoidance maneuver as well as different detection ranges of the OS. In :numref:`Head-on_1` below
both OS and TS are moving with 3 m/s on a reciprocal course. The detection range for obstacles of the OS is set here to 50 m. It is set to 50 m because the ASV the algorithm is dedicated to is going to have a detection
around 50 m. At about 25 seconds the velocity of the OS penetrates the velocity obstacle formed by the TS. As seen in subfigure (C) the desired orientation changes about 30° from the inital course.
Due to the fact that the new course cannot be taken immediately, further calculations of the new speed will determine that an even greater course change is necessary. Once the collision avoidance maneuver is over,
the OS turns back to its next point as seen in subfigure (D). As described in the COLREGs, a change in speed was avoided.

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


    Results of the simulation for a head-on situation (vel_OS\: 3 m/s, vel_TS\: 3 m/s, detection range: 50 m)

In :numref:`Head-on_2` and :numref:`Head-on_3` the speed of the TS is increased to 5 and 7.5 m/s but the detection range was kept at 50 m. This results first in an even greater change of the orientation of the OS as seen in subfigure (C). In :numref:`Head-on_3`
the course change alone was not sufficient any more to avoid the collision that is why the speed changes as well as seen in subfigure (B). 

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


    Results of the simulation for a head-on situation (vel_OS\: 3 m/s, vel_TS\: **5 m/s**, detection range: 50 m)

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


    Results of the simulation for a head-on situation (vel_OS\: 3 m/s, vel_TS\: **7.5 m/s**, detection range: 50 m)

For the last simulation on the head-on scenarios the detection range of the OS was increased to 200 m and the speed of the TS was kept at 7.5 m/s. As seen in :numref:`Head-on_4`, the earlier a TS can be detected the
better the OS can avoid the collision. With 50 m detection range and a speed of the TS of 7.5 m/s the OS had to change its course drastically and also its speed. Once the TS was detected in a range of
200 m the OS was able to avoid the collision again by just changing the course by 30° without changing the speed.

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


    Results of the simulation for a head-on situation (vel_OS\: 3 m/s, vel_TS\: 5 m/s, detection range: **200 m**)

In all shown head-on situations the OS acts according to COLREGs by turning at least 30° to starboard to avoid the collision. It is also avoiding a change in speed and prefering a change in course as long it is
possible. At all time during the maneuver the OS is keeping a safe distance to the TS. This was confirmed by checking if the OS touches the safety area defined in the collision avoidance algorithm, which was not
the case in any situation.


Right-crossing
--------------
For the right-crossing scenario three situation are displayed here in which each TS is coming from a different direction towards the OS. Here the speed of OS and TS was both 3 m/s for each situation.
In all three simulations the OS is able to avoid the collision with the OS in compliance with COLREGs by avoiding to starboard and crossing behind the TS. No change in speed was necessary and the safety
distance was always maintained. In :numref:`Right-crossing_3` in the subfigure (C) it can be seen, that the orientation changes back to the goal position at about second 25 because the desired velocity
was not inside the velocity obstacle formed by the TS anymore. Due to the fact that the controller of the OS cannot adjust the course directly, the algorithm detects another possible collision later on
and adjusts the course again.

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


    Results of the simulation for a right-crossing situation with a course of TS of 0° and speed of 3 m/s

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


    Results of the simulation for a right-crossing situation with a course of TS of 330° and speed of 3 m/s

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


    Results of the simulation for a right-crossing situation with a course of TS of 20° and speed of 3 m/s

Overtaking + static obstacle
----------------------------
For the overtaking scenarios two situations are displayed here. One where the TS is moving slower then the OS on the same course over ground and one where the TS is not moving at all and so being a static obstacle.
According to COLREGs the OS is free to choose on which side to pass the TS on. Due to the :ref:`cost function <cost function>` implemented in the algorithm, the OS will choose the side on which the new course is
closest to the course to the goal or next waypoint. The two simulations in :numref:`Overtaking_1` and :numref:`Overtaking_2` below show that the OS can perform avoidance maneuvers for overtaking scenarios even 
if the obstacle is not moving. If the obstacle is not moving the OS can also choose on which side to pass. The algorithm can be therefor used for static and dynamic collision avoidance.

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


    Results of the simulation for an overtaking situation with a slow moving TS on the same course as the OS

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


    Results of the simulation for an situation with a non-moving/static TS


Left-crossing
-------------
For the left-crossing scenario three situation are displayed here in which each TS is coming from a different direction towards the OS. In these scenarios we assume, that the TS is not taking action even if it
is supposed according to COLREGs. The OS will first to try to stand-on course until a threshold is reached and it will act anyways. Because the OS reacts later than usual, you can see in subfigure (B) that the
speed has to be changed as well to avoid the collision. These avoidance maneuvers are also in accordance with COLREGs. Instead of a time to collision threshold here it is probably a good idea to have a treshhold like:
if there are are less then ten free velocities with no speed change in the velocity space, take an action. Else it can happen that the OS tries to avoid a collision at a point where already are no free velocity in 
the velocity space.

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


    Results of the simulation for a left-crossing situation with a course of TS of 180° and speed of 3 m/s

    

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


    Results of the simulation for a left-crossing situation with a course of TS of 210° and speed of 3 m/s


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


    Results of the simulation for a left-crossing situation with a course of TS of 160° and speed of 3 m/s



Being-overtaken
---------------
For the being-overtaken scenario two situations are displayed here. Both situations are similar in the course of the OS and TS but differs in the speed of the involved ships. As before in the Lfet-crossing situations,
the OS is not supposed to take actions but has to do it eventually. Which here also results in a change of speed of the OS to avoid the collision. The phenomenon described in the section before, that the threshold for
stand-on vessels could result in a fully occupied velocity space is in these scenarios even worse. That is why the speed is changing even more.

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


    Results of the simulation for a being-overtaken situation with a slow moving OS (1 m/s)

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


    Results of the simulation for a being-overtaken situation with a fast moving OS (3 m/s)

Multiple ship encounters
------------------------
For multiple ship encounters several scenes have been simulated. Head-on scenarios with multiple TSs and Head-on scenarios with additional crossing encounters. These simulations show how the OS reacts to several TSs
in at the same time.

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


    Results of the simulation for a head-on situation with two TSs besides each other coming in reciprocal courses towards the OS

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


    Results of the simulation for a head-on situation with two TSs besides each other coming in reciprocal courses towards the OS, but with an offset in the starting position

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


    Results of the simulation for a situation with two TSs, one head-on and one in a left-crossing

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


    Results of the simulation for a situation with two TSs, one head-on and one in a right-crossing

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


    Results of the simulation for a situation with three target ships, pne head-on, one left-crossing and one right-crossing



