============
Introduction
============
This chapter gives an overview of the reasons for the developed collision avoidance algorithm and clarifies some basics which are important for the understanding.
Also it explains the functional principle of the algorithm and which extra features have been implemented. At the beginning of this chapter some vocabulary, which will
be used in the further course, is explained and defined.

Motivation
----------

- a lot of collisions
- many fatalities due to collision
- most collisions are due to human errors
- shortage of qualified labor
- reduce risk
- reduce costs

COLREGs
-------
The International Regulations for Preventing Collisions at Sea 1972 (COLREGs) are published
by the International Maritime Organization (IMO) and define, among other things, the "rules of the road"
or navigation rules to be followed by ships and other vessels at sea to prevent collisions between two or more vessels.
The most important rules for collision avoidance are defined in Part B - Streering and sailing rules.
For this collision avoidance algorithm rules 8 and 13-17 are considered. These are the rules that are describing
which actions should be taken in case of an encounter between two vessels. It follows a short description of each rule:

- **Rule 8** (Action to avoid a collision): actions should be made in ample time, alteration of course shall be obvious to the other vessels, alteration of course is preferred
- **Rule 13** (Overtaking): Vessel coming from a direction more than 22.5° abaft your own vessels beam --> Own vessel is being overtaken, other vessel is overtaking
- **Rule 14** (Head-on situation): Vessels meeting on reciprocal or nearly reciprocal courses shall both alter their course to starboard
- **Rule 15** (Crossing situation): Vessels having the other vessel on their starboard side shall keep out of the way and so are the give-away vessel
- **Rule 16** (Action by give-away vessel): Vessels which are directed to keep out of the way shall take early and substantial actions to avoid the collision
- **Rule 17** (Action by stand-on vessel): Vessels which are not directed to keep out of the way shall keep course and speed but take actions as soon as it becomes apparent that the other vessel is not taking any actions to avoid the collision

In general when a vessel is in any doubt as wether it is in a specific situation, it shall assume it is the case. Also these rules only apply if the current course involve risk
of collision. 

Velocity Obstacle algorithm
---------------------------
The velocity obstacle (VO) algorithm was introduced by Fiorini and Schiller in 1996 for motion planning in dynamic environments. 


