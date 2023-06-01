============
Introduction
============
This chapter gives an overview of the reasons for the developed collision avoidance algorithm and clarifies some basics which are important for the understanding.
Also it explains the functional principle of the algorithm and which extra features have been implemented. At the beginning of this chapter some vocabulary, which will
be used in the further course, is explained and defined.

Vocabulary
----------

- **Own ship (OS)** - the autonomous surface vehicle using the collision avoidance algorithm
- **Target ship (TS)** - the obstacle that the OS has to avoid (e.g. other vessels, static objects, etc.)
- **Heading** - the compass direction in which the vessels bow or nose is pointed --> not equal to the direction the vessel acutally travels
- **Course** - intended path of a vessels over the surface of the Earth
- **Course over ground (COG)** - acutal direction the vessel travels over the surface of earth
- **Course, heading angles** - North = 0°, East = 90°, South = 180°, West = 270°
- **Collision Cone (CC)** - set of realtive velocities of the OS to TS leading to a collision at any time
- **Velocity Obstacle (VO)** - set of absolute velocities of OS leading to a collision at any time
- **Velocity** - a vector with magnitude as speed [m/s] and direction as COG [°]
- **desired velocity** - the velocity the OS to move from the current position to the next waypoint (given by the global path planner)

.. _motivation:

Motivation
----------
The interest in automating processes has increased in the last decade to support or even substitute humans in their work. There are several
reasons to automate processes. To increase safety by assisting humans in dangerous task or by removing the human factor completely. In the
course of this, great attention was paid to autonomous robots in military and civil purposes. Some examples are autonomous cars, lawn movers
or vacuum cleaners. Autonomous robots use sensors and actuators to interact with the physical surroundings and take the right course of action.
Therefor they have to operate in static but also changing dynamic environments.  Of increasing interest here are autonomous ships and autonomous
surface vehicles (ASVs). The complex environment ships are moving in is quite challenging. An ASV has to be able to detect static and dynamic obstacles
like other vehicles, shallow waters or islands. Furthermore ASVs has to follow an optimal path while avoiding obstacles. While humans may not be able
to react and choose an optimal path fast enough, this could be possible with USVs. The optimizations here can be related to energy consumption, time
consumption or safety in form of collision avoidance. Due to the reason, that for now there will be also human driven vehicles on the sea, an ASV has
to be able not just to choose the optimal path, but also choose an action in compliance with the COLREGs. According to the Annual overview of
marine casualties 2022 the most accidents are caused by human actions with 59.6\,\% of all incidences. Collision at sea cost lives, money and often have
a major impact on the local environment. Besides this it is also estimated that there will be a lack of over approx. 147,500 qualified personnel by the year 2025.
An autonomous system offers the possibility of increasing safety, reducing costs and counteracting the lack of personnel. To obtain a safe autonomous system a reliable
guidance, navigation and control (GNC) system is required. Due to the aforementioned points, the collision avoidance algorithm plays an important part in making the GNC system reliable.

.. _colregs:

COLREGs
-------
The International Regulations for Preventing Collisions at Sea 1972 (COLREGs) are published
by the International Maritime Organization (IMO) and define, among other things, the "rules of the road"
or navigation rules to be followed by ships and other vessels at sea to prevent collisions between two or more vessels.
The most important rules for collision avoidance are defined in Part B - Streering and sailing rules.
For this collision avoidance algorithm rules 8 and 13-17 are considered. These are the rules that are describing
which actions should be taken in case of an encounter between two vessels. It follows a short description of each rule:

- **Rule 8** (Action to avoid a collision): actions should be made in ample time, alteration of course shall be obvious to the other vessels, alteration of course is preferred, collision avoidance action shall result in passing at a safe distance
- **Rule 13** (Overtaking): Vessel coming from a direction more than 22.5° abaft your own vessels beam --> Own vessel is being overtaken, other vessel is overtaking
- **Rule 14** (Head-on situation): Vessels meeting on reciprocal or nearly reciprocal courses shall both alter their course to starboard
- **Rule 15** (Crossing situation): Vessels having the other vessel on their starboard side shall keep out of the way and so are the give-away vessel
- **Rule 16** (Action by give-away vessel): Vessels which are directed to keep out of the way shall take early and substantial actions to avoid the collision
- **Rule 17** (Action by stand-on vessel): Vessels which are not directed to keep out of the way shall keep course and speed but take actions as soon as it becomes apparent that the other vessel is not taking any actions to avoid the collision

In general when a vessel is in any doubt as wether it is in a specific situation, it shall assume it is the case. Also these rules only apply if the current course involve risk
of collision.

The COLREG rules are quite vague which make it hard to be implemented in autonomous systems. The only quantified value is defined in Rule 13, which is the 22.5° abaft beam. In this algorithm an area +- 15° from
the bow of the OS is defnied for head-on encounters. Furthermore a substantial action and obvious alteration of course is defined at a value of at least 30° from the initial course. These values were derived from
official law cases of collisions on sea. The safe distance at which the own vessel should pass during the avoidance maneuver is defined as a multiple of the own vessel size. How many ship sizes considerd to be safe depends 
on the vessels properties like speed, size, turning speed, etc.
The above defined areas for each rule results in the following sectors for rule selection:

.. _colregclass:
.. figure:: img/COLREGclassification.png
    :width: 70%
    :align: center

    COLREG rule selection sectors (Source: Xia et al., 2020, DOI:10.1109/ACCESS.2020.3038187)

.. _velocityobstacle:

Velocity Obstacle algorithm
---------------------------
The Velocity Obstacle (VO) algorithm was introduced by Fiorini and Schiller in 1996 for motion planning in dynamic environments. In the VO algorithm, obstacles are represented as cones in the velocity
space. The cones, called velocity obstacles, represent the set of constant velocities causing a collision between the vehicle and an obstacle at some future time. Maintaining a velocity outside of the velocity
obstacle guarantees a collision-free trajectory of the vessel. Navigation among multiple moving obstacles is possible by avoiding the velocities inside the union of all individual velocity obstacles. Besides the VO
other contrains can be added to the velocity space to obtain a COLREG compliant trajectory.

To explain the principle of the algorithm we are looking at a scenario with a single target ship that the own ship has to avoid. (see :numref:`expanded`)  

.. _safety area:

The first step in calculating the Velocity Obstacle is to expand the obstacle size with the size of the own ship so that for the calcualtions the OS can be seen as a point. This
is done with the Minkowski sum which adds all points of one shape to the shape of the other (see :numref:`expanded`). As mentioned before in chapter :ref:`colregs`, Rule 8 states that the OS
shall pass at a safe distance. This safe distance here is defined as a multiple of the OS shape and is also added to the shape of the TS with the help of the Minkowski sum.

.. _expanded:
.. figure:: img/3_row_expanded_cropped.svg

    Expension of TS with OS shape and safety area

In the next step the collision cone (CC) is calculated. It is the area formed by the two tanget lines formed from the center of OS to the expanded shape of TS (see :numref:`CC`).
The CC is a set of all relative velocities that would lead to a collision with the TS at any time in the future. The CC can be further reduced by removing all velocities that would
lead to a collision in the distant future. If the time to collision of a velocity is greater than the set time to collision (TTC), it is removed from the CC. This reduces the risk
that the velocity space is completely occupied by the CC in case there are many obstacles around the OS. Furthermore, this prevents the OS from trying to avoid obstacles where a
collision would only take place in 2 hours, for example, since the situation can still change completely during this time.

$$CC = \{v_r\,|\,v_rt\,\cap\,T\,\neq\,\emptyset,\,\forall_t\,\geq\,TTC\}$$



.. _CC:
.. figure:: img/2_row_collision_cone_cropped.svg
    :width: 75%
    :align: center

    Collision Cone visualisation

The collision cone is specific only to a pair of OS/TS. To take into account several TS it is necessary to create an object comparable to the CC, which takes into account absolute velocities.
This can be easily achieved by adding the velocity of the TS (V\ :sub:`TS`) to each velocity in the CC or equivalent translating the CC by V\ :sub:`TS` as seen in :numref:`velobst`.
This is the so called Velocity Obstacle (VO) and is a set of absolute velocities leading to a collision at any time in the future greater then the TTC.

$$VO = CC\,\oplus\,V_{TS}$$

.. _velobst:
.. figure:: img/2_row_velocity_obstacle_cropped.svg
    :width: 75%
    :align: center

    Velocity obstacle visualisation

In the most cases the velocity of target ships is measured by sensor or is calculated based on measurements obtained by sensors. This and the fact that the target ship movement is not
always constant as assumed by the VO algorithm, there are uncertainties in the velocity of the TS. TO account for these uncertainties, they can be added to the VO in same fashion as
it was done with expansion of the TS with the OS. With the Minkowsi sum the uncertainties can be added to the VO as seen in :numref:`uncert`.

.. _uncert:
.. figure:: img/VO_unc_2_cropped.svg
    :width: 35%
    :align: center

    Extension of the VO with uncertainties in the velocity of TS

From this point on it would be possible to choose a speed outside the VO to avoid a collision. However, the further rules of the COLREGs must still be considered. Which specify, among
other things, on which side an obstacle must be avoided. Conveniently, the VO already divides the velocity space into three areas (see :numref:`colregcon`), which we can use for the implementation of the COLREGs.

- V\ :sub:`1`: All velocities resulting in passing with TS on starboard side
- V\ :sub:`2`: All velocities resulting in passing with TS on port side
- V\ :sub:`3`: All velocities resulting in moving away from TS

The velocities inside V\ :sub:`1` are equal to the velocities that should be avoided according to COLREG in case the OS is the give-away vessel and also in cases where the OS is the stand-on vessel but the
TS is not taking any actions at all to avoid the collision. All velocities outside the VO and outside the COLREG constrains can be choosen by the OS to avoid a collision in compliance with COLREGs. The COLREG
contrains are only calculated if the current velocity of the OS enters the VO of a TS (= risk of collison) and the situation requires to apply the COLREG contrains. For example in an overtaking scenario no
COLREG constrains have to be applied, because Rule 13 does not state on which side the TS shall be overtaken.

.. _colregcon:
.. figure:: img/Colreg_cons_2_cropped.svg
    :width: 50%
    :align: center

    Velocity Obstacle dividing the velocity space in three areas

From the hard constrains, the VO and the COLREG contrains, we determined a space in the velocity space in which any choosen velocity avoids the collision. The velocity space is discretized to select the optimal
velocity. This is done in velocity steps from 0 to the maximum possible speed of the OS and a course angle of 0-360 degrees. The so-called discretized velocity space. The finer the steps are, the more velocities
are obtained which have an effect on the speed of the algorithm. For every velocity in the discretized velocity space it is checked wether they are inside the VO and COLREG constrains or outside. Every velocity outside
the constrains is a velocity which assures that a collision is avoided. To select the optimal velocity out of those, a cost function is implemented (see :ref:`cost function <cost function>`). The more the speed deviates from the current speed, the higher the cost.
The more the course deviates from the desired course, the higher the cost. As mentioned before, an evasive maneuver should be clearly visible. We have defined a course change of 30 degrees here. The further the course
difference is from the new speed to the current speed, the higher the cost:

.. _cost function:

$$J = w_1\cdot\Delta(\theta_{des}-\theta_{free}) + w_2\cdot\Delta(v_{des}-v_{free}) + w_3\cdot\Delta(\theta_{OS+30}-\theta_{free})$$ 

$w_1$, $w_2$ and $w_3$ are the weights to set the cost function and were determined experimentally. The first time the velocity of the OS enters the VO, it is stored as the inital velocity of the OS. This velocity
is then used during the avoidance maneuver to calculate the COLREG rule and is used in the cost function to calculate the optimal velocity. The avoidance maneuver ends, once the current velocity of the OS and the 
desired velocity of the OS are outside the velocity Obstacle. 
With this cost function the new velocity for the OS is calculated. In :numref:`discrete` the new velocity is displayed as a blue arrow. In this example it is a right crossing 
scenario and the OS has to avoid the collision by changing its course to starboard and crossing behind the TS. 

.. _discrete:
.. figure:: img/2_row_discrete_vel_space_cropped.svg
    :width: 100%
    :align: center

    Discretized velocity space with optimal velocity displayed

The Velocity Obstacle algorithm can be used with multiple obstacles as well. Therefor all individual COLREG constrains and VOs are combined. All velocities outside the union of each
individual constrains are the velocities avoiding a collision. In :numref:`multiple` the OS can be seen surrounded by four TSs. The velocity of the OS is inside three velcity obstacles.
One is a left-crossing, one head-on and one right-crossing scenario. For the Head-on and the right-crossing scenario COLRGE constrains has to be calculated and are added to the velocity space.

.. _multiple:
.. figure:: img/VO_4_TS_cropped.svg
    :width: 50%
    :align: center

    Multiple target ships around the own ship

The Velocity Obstacle algorithm can be used not only to make fully autonomous systems and thus replace humans, but also offers the possibility to support humans. For example,
a display can be created that shows the velocity space and gives a suggestion for a new velocity (see :numref:`display`).

.. _display:
.. figure:: img/VO_4_TS_disp_cropped.svg
    :width: 50%
    :align: center

    Display of the result of the VO algorithm to assist humans
