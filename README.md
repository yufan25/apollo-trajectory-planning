# apollo-trajectory-planning
Decouple the trajectory planning part from the apollo v5.0 and run in ros system.

Tested in ubuntu 18.04 with ros melodic.

In version 1.0 the lattice planning with static obstacle is realized.  

- rosrun PNC_MapPub to publish the reference line of road
- rosrun PNC_ObsPub to publish the obstacle
- rosrun lattice_planning to run the lattice planning

For lateral optimization osqp solver is also an option. (Eigen needed) 
