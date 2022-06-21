# social-rrt-demo

## Contribution defined

In general, the contribution is related to include social cues into a sampling based planning method. In this case, the main planning method is RRT*. As a basis a modified risk related RRT* modification from Juan Hernandez is used.

The only social cue in which we will focus will be social comfort. Meaning that proxemics will be considered, leading to the following objective:

- Implementing a modified version of RRT\* capable of creating feasible dynamic paths for robot navigation around moving people.

Several stages will be carried on, step by step to accomplish the objective.

### Stage 1

For this part the following will be considered:

1. Only static people will be considered in simulation.
2. A comfort zone will be defined to interact around each people.
3. The planned solution path will have to consider path cost affected by comfort zone.

#### How to do it?

1. Static people has already been implemented in simulation using pedsim_ros. This is done by defining just a single waypoint.

2. The interaction for static and dynamic agents.

![](https://i.imgur.com/kaL4ZpR.png)

![](https://i.imgur.com/8D9VzFD.png)

3. Change the integration of the cost using the interaction of agents as defined above.

   In brief the cost of the path is modified as the integral of the interaction zone between the agent and the robot with the planned path in a way that the modification should be related to the cost optimization function.

![](https://i.imgur.com/FYW02Ie.png)

![](https://i.imgur.com/HJEfEql.png)

### Stage 2

For this part the following is considered:

1. Agents move around an specific space.

In this case not much changes will be made. Simply in simulation agents will be put to move around waypoints and the robot shall replan its path depending on the movements of the agents.

### Stage 3

For this part the following is considered:

1. There are more complex interactions of people. For example groups and interactions with interesting objects.

#### How to do it?

GOFF will be used as in Ngo, H. Q. T., Le, V. N., Thien, V. D. N., Nguyen, T. P., & Nguyen, H. (2020). Develop the socially human-aware navigation system using dynamic window approach and optimize cost function for autonomous medical robot. Advances in Mechanical Engineering, 12(12), 1–17. https://doi.org/10.1177/1687814020979430

And this will influence just as with static single agents.
