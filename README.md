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

This package is in charge of the perception for the robot. For that, a depth camera is used, which helps to generate a 3D map of the space by using Octomap.

This 3D map is combined in a multilayer GridMap with the position of the social agents detected in the space.

### Parameters

- resolution (double, default: 1.0)

  Octomap and Grid Map resolution.

- map_frame (string, default: "map")

- fixed_frame (string, default: "fixed_frame")

  Fixed frame considered for the mapping, can be `odom` for example.

- robot_frame (string, default: "/robot_frame")

  Base frame considered from the robot for the mapping.

- oflline_octomap_path (string, default: "")

  If defined, then the given map is used and no ther mapping is done.

- visualize_free_space (bool, default: True)

  Wether you would like to see the 3D map and grid_map. If `False`, nothing is published at `/esc_move_base_mapping/social_grid_map` and `/esc_move_base_mapping/octomap_map`.

- mapping_max_range (double, default: 5.0)

  Maximum distance to generate the 3D map from the obtained depth points.

- odometry_topic (string, default: "/odometry_topic")

- rviz_timer (double, default: 0.0)

  The time delay to publish messages to RViz. Not used if `visualize_free_space` is `False`.

- point_cloud_topics (list: string, default: empty)

  List of pointcloud topics used to generate the 3D map.

- point_cloud_frames (list: string, default: empty)

  List of frames from pointcloud topics in order respectively.

- social_agents_topic (string, default: "/pedsim_simulator/simulated_agents")

  Topic with social agents states.

- social_agent_radius (double, default: 0.4)

  Radius considered for the social agents.

- social_relevance_validity_checking (bool, default: False)

  Wether or not it is desired to only consider the relevant agents.

- robot_distance_view_max (double, default: 6.0)

  Maximum distance for the robot to consider social agents.

- robot_distance_view_min (double, default: 1.5)

  Mininum distance for the robot to consider social agents.

- robot_angle_view (double, default: 1.57)

  Maximum angle in which the social agents are considered from the field of view of the robot. Defined in radians.

- robot_velocity_threshold (double, default: 0.3)

  Maximum velocity that the robot can have.

### Subscribers

- /pedsim_simulator/simulated_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/pedsim_msgs/msg/AgentStates.msg))

  Position, orientation, velocity and other states of the social agents.

- /pepper/camera/depth/points ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))

  Depth points from depth camera.

- /pepper/odom_groundtruth ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

  Robot odometry (in this case it was Pepper robot).

### Publishers

- /esc_move_base_mapping/octomap_map ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))

  Visual representation of the 3D map created.

- /esc_move_base_mapping/relevant_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/pedsim_msgs/msg/AgentStates.msg))

  If parameter `social_relevance_validity_checking` is `true`, then only the filtered relevant agents are published. Otherwise, all agents are re-published.

- /esc_move_base_mapping/social_grid_map ([grid_map_msgs/GridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/msg/GridMap.html))

  Grid map with 2 layers, one for obstacles and another for social agents.

### Services

- /esc_move_base_mapping/get_grid_map ([grid_map_msgs/GetGridMap](http://docs.ros.org/en/indigo/api/grid_map_msgs/html/srv/GetGridMap.html))

  Fetches the grid map from the World Modeling.

## Online Social Robot Path Planning (`esc_move_base_planning`)

This package is in charge of finding socially acceptable solution paths for the robot to follow.

### Parameters

- world_frame (string)
- planner_name (string)
- planning_bounds_x (list: [x_min, x_max])
- planning_bounds_y (list: [y_min, y_max])
- dynamic_bounds (bool)
- start_state (list, [X, Y, Yaw])

### Subscribers

- /esc_move_base_planner/query_goal ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- /pepper/odom_groundtruth ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

  Robot odometry.

- /esc_move_base_mapping/relevant_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/pedsim_msgs/msg/AgentStates.msg))

  Agents considered by the mapping module.

### Publishers

- /esc_move_base_planner/esc_move_base_solution_path ([esc_move_base_msgs/Path2D](https://github.com/CardiffUniversityComputationalRobotics/esc-nav-stack/blob/world_modeling/esc_move_base_msgs/msg/Path2D.msg))

  Solution path found by the planner. It is passed to the control module.

- /esc_move_base_planner/esc_num_nodes ([std_msgs/Int32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32.html))

  Number of valid nodes sampled by the planner.

- /esc_move_base_planner/query_goal_pose_rviz ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

  Shows in RViz the query pose requested to the planner.

- /esc_move_base_planner/query_goal_radius_rviz ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))

  Shows in RViz the radius of the query requested to the planner.

- /esc_move_base_planner/solution_path ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))

  Visual solution path to be seen in RViz.

### Actions

- `/goto_action` ([esc_move_base_msgs/GoTo2D](https://github.com/CardiffUniversityComputationalRobotics/esc-nav-stack/blob/world_modeling/esc_move_base_msgs/action/Goto2D.action))

  Action to request navigation query. The name of the action server depends on the parameter `goto_action`.

## Path Following Control (`esc_move_base_control`)

### Subscribers

### Publishers

### Parameters
