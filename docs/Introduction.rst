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
official law cases of collisions on sea. The safe distance at which the own vessel should pass during the avoidance manouver is defined as a multiple of the own vessel size. How many ship sizes considerd to be safe depends 
on the vessels properties like speed, size, turning speed, etc.
The above defined areas for each rule result in the following sectors for rule selection:

.. figure:: img/COLREGclassification.png

    COLREG rule selection sectors (Source: Xia et al., 2020, DOI:10.1109/ACCESS.2020.3038187)

Velocity Obstacle algorithm
---------------------------
The Velocity Obstacle (VO) algorithm was introduced by Fiorini and Schiller in 1996 for motion planning in dynamic environments. In the VO algorithm, obstacles are represented as cones in the velocity
space. The cones, called velocity obstacles, represent the set of constant velocities causing a collision between the vehicle and an obstacle at some future time. Maintaining a velocity outside of the velocity
obstacle guarantees a collision-free trajectory of the vessel. Navigation among multiple moving obstacles is possible by avoiding the velocities inside the union of all individual velocity obstacles. Besides the VO
other contrains can be added to the velocity space to obtain a COLREG compliant trajectory.

To explain the principle of the algorithm we are looking at a scenario with a single target ship that the own ship has to avoid.  

!!! Here VO algorithm explained !!!

