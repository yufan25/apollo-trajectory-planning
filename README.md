# apollo-trajectory-planning
decouple the trajectory planning part from the apollo v5.0 and run in ros system.

tested in ubuntu 18.04 with ros melodic.

in version 1.0 the lattice planning with static obstacle is realized.  

- rosrun PNC_MapPub to publish the reference line of road
- rosrun PNC_ObsPub to publish the obstacle
- rosrun lattice_planning to run the lattice planning
