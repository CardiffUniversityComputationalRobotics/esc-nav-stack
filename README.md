# Extended Social Comfort

This is an online social robot navigation framework for indoor social scenarios. From this work a paper conference was submitted [Towards Online Socially Acceptable Robot Navigation](https://ieeexplore.ieee.org/document/9926686).

It is composed of four different packages:

- `esc_move_base_control`: it is a simple differential control method.
- `esc_move_base_mapping`: is in charge of the perception of the space.
- `esc_move_base_msgs`: contains the messages needed for the framework and the start-goal queries.
- `esc_move_base_planning`: responsible for finding solution paths for the navigation query requested.

To understand more about how the framework works, you are highly encouraged to take a look at the paper.

To run the framework, there is a launch file example in every package with example configurations.

![FrameworkConnections](https://i.imgur.com/0YOMmiD.png)

## World Modeling (`esc_move_base_mapping`)

This package is in charge of the perception for the robot. For that, a depth camera is used, which helps to generate a 3D map of the space.

This 3D map is combined in a multilayer GridMap with the position of the social agents detected in the space.

## Subscribers

## Publishers
