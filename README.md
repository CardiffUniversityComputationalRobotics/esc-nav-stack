# Extended Social Comfort

This is an online social robot navigation framework. From this work a paper conference was submitted: **S. Silva, D. Paillacho, N. Verdezoto, J.D. Hernández. "Towards Online Socially Acceptable Robot Navigation". To be published in IEEE International Conference on Automation Science and Engineering (CASE) 2022**.

It is divided in the following packages:

- `esc_move_base_control`: it is a simple differential control method.
- `esc_move_base_mapping`: is in charged of doing the mapping of the space. (not actually used so far)
- `esc_move_base_msgs`: contains the messages needed for the framework and the start-goal queries.
- `esc_move_base_planning`: this is in charge of finding the path solution for the queries.

Every package has a launch file as an example.

As dependencies have in mind the following packages:

- pedsim_ros
- octomapping
