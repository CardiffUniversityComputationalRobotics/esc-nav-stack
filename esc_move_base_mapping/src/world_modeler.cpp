/*! \file octomap_laser_scan.cpp
 * \brief Merge data from different laser_scan messages to incrementally build
 * an WorldModeler.
 *
 * \date November 07, 2022
 * \author Juan David Hernandez Vega, HernandezVegaJ@cardiff.ac.uk
 * \author Steven Alexander Silva Mendoza, silvas1@cardiff.ac.uk
 *
 * \details Purpose: Merge data from different laser_scan messages to
 * incrementally build an Octomap.
 *  Based on laser_octomap (Guillem Vallicrosa & J.D. Hernandez Vega, University
 * of Girona)
 */

#include <world_modeler.hpp>

//!  Constructor.
/*!
 * Load map parameters.
 * Subscribers to odometry and laser scan
 * Publishers to visualize the WorldModeler.
 */
WorldModeler::WorldModeler()
    : Node("world_modeler_node"),
      fixed_frame_("fixed_frame"),
      robot_frame_("robot_frame"),
      map_frame_("map"),
      odometry_topic_("/odometry_topic"),
      offline_octomap_path_(""),
      erase_map_topic_("erase_map"),
      octree_(NULL),
      octree_resol_(1.0),
      mapping_max_range_(5.0),
      initialized_(false),
      visualize_free_space_(false),
      rviz_timer_(0.0),
      robot_distance_view_max_(6.0),
      robot_distance_view_min_(1.5),
      robot_angle_view_(1.57),
      robot_velocity_threshold_(0.3),
      social_agent_radius_(0.4),
      social_agents_topic_("/pedsim_simulator/simulated_agents"),
      social_relevance_validity_checking_(false),
      min_z_pc_(0.05),
      max_z_pc_(1.0),
      social_comfort_amplitude_(6.0),
      orientation_drift_(0.0),
      position_drift_(0.0),
      apply_filter_(false),
      add_max_ranges_(false),
      add_rays_(false),
      skip_sensor_callbacks_after_erase_(false),
      minimum_range_(-1.0)
{
  //=======================================================================
  // Get parameters
  //=======================================================================
  this->declare_parameter("add_rays", add_rays_);
  this->declare_parameter("resolution", octree_resol_);
  this->declare_parameter("map_frame", map_frame_);
  this->declare_parameter("fixed_frame", fixed_frame_);
  this->declare_parameter("robot_frame", robot_frame_);
  this->declare_parameter("offline_octomap_path", offline_octomap_path_);
  this->declare_parameter("visualize_free_space", visualize_free_space_);
  this->declare_parameter("odometry_topic", odometry_topic_);
  this->declare_parameter("rviz_timer", rviz_timer_);
  this->declare_parameter("laser_scan_topic", laser_scan_topic_);
  this->declare_parameter("laser_scan_frame", laser_scan_frame_);
  this->declare_parameter("point_cloud_topic", point_cloud_topic_);
  this->declare_parameter("point_cloud_frame", point_cloud_frame_);
  this->declare_parameter("mapping_max_range", mapping_max_range_);
  this->declare_parameter("robot_distance_view_max", robot_distance_view_max_);
  this->declare_parameter("robot_distance_view_min", robot_distance_view_min_);
  this->declare_parameter("robot_angle_view", robot_angle_view_);
  this->declare_parameter("robot_velocity_threshold", robot_velocity_threshold_);
  this->declare_parameter("social_agent_radius", social_agent_radius_);
  this->declare_parameter("social_agents_topic", social_agents_topic_);
  this->declare_parameter("erase_map_topic", erase_map_topic_);
  this->declare_parameter("social_relevance_validity_checking", social_relevance_validity_checking_);
  this->declare_parameter("min_z_pc", min_z_pc_);
  this->declare_parameter("max_z_pc", max_z_pc_);
  this->declare_parameter("social_comfort_amplitude", social_comfort_amplitude_);
  this->declare_parameter("apply_filter", apply_filter_);
  this->declare_parameter("add_max_ranges", add_max_ranges_);
  this->declare_parameter("minimum_range", minimum_range_);

  this->get_parameter("add_rays", add_rays_);
  this->get_parameter("resolution", octree_resol_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("fixed_frame", fixed_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("offline_octomap_path", offline_octomap_path_);
  this->get_parameter("visualize_free_space", visualize_free_space_);
  this->get_parameter("odometry_topic", odometry_topic_);
  this->get_parameter("rviz_timer", rviz_timer_);
  this->get_parameter("laser_scan_topic", laser_scan_topic_);
  this->get_parameter("laser_scan_frame", laser_scan_frame_);
  this->get_parameter("point_cloud_topic", point_cloud_topic_);
  this->get_parameter("point_cloud_frame", point_cloud_frame_);
  this->get_parameter("mapping_max_range", mapping_max_range_);
  this->get_parameter("robot_distance_view_max", robot_distance_view_max_);
  this->get_parameter("robot_distance_view_min", robot_distance_view_min_);
  this->get_parameter("robot_angle_view", robot_angle_view_);
  this->get_parameter("robot_velocity_threshold", robot_velocity_threshold_);
  this->get_parameter("social_agent_radius", social_agent_radius_);
  this->get_parameter("social_agents_topic", social_agents_topic_);
  this->get_parameter("erase_map_topic", erase_map_topic_);
  this->get_parameter("social_relevance_validity_checking", social_relevance_validity_checking_);
  this->get_parameter("min_z_pc", min_z_pc_);
  this->get_parameter("max_z_pc", max_z_pc_);
  this->get_parameter("social_comfort_amplitude", social_comfort_amplitude_);
  this->get_parameter("apply_filter", apply_filter_);
  this->get_parameter("add_max_ranges", add_max_ranges_);
  this->get_parameter("minimum_range", minimum_range_);

  this->set_parameter(rclcpp::Parameter("use_sim_time", true));

  //=======================================================================
  // Transforms TF and catch the static transform from vehicle to laser_scan
  // sensor
  //=======================================================================
  int count(0);
  rclcpp::Time t;
  geometry_msgs::msg::TransformStamped transform;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(false);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());

  tf_buffer_->setCreateTimerInterface(create_timer_interface);

  initialized_ = false;
  do
  {
    try
    {
      transform = tf_buffer_->lookupTransform(robot_frame_, point_cloud_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
      initialized_ = true;
    }
    catch (tf2::TransformException &ex)
    {

      count++;
      RCLCPP_WARN(this->get_logger(), "Cannot find tf from %s to %s: %s",
                  robot_frame_.c_str(), point_cloud_frame_.c_str(), ex.what());
    }
    if (count > 10)
    {
      RCLCPP_ERROR(this->get_logger(), "No transform found. Aborting...");
      exit(-1);
    }
  } while (rclcpp::ok() && !initialized_);

  RCLCPP_WARN(this->get_logger(), "tf from %s to %s OK", robot_frame_.c_str(), point_cloud_frame_.c_str());

  //=======================================================================
  // Octree
  //=======================================================================
  if (offline_octomap_path_.size() > 0)
    octree_ = new octomap::OcTree(offline_octomap_path_);
  else
    octree_ = new octomap::OcTree(octree_resol_);
  RCLCPP_WARN(this->get_logger(), "Loaded octree");

  octree_->setProbHit(0.7);
  octree_->setProbMiss(0.4);
  octree_->setClampingThresMin(0.1192);
  octree_->setClampingThresMax(0.971);

  //=======================================================================
  // Gridmap
  //=======================================================================

  initializeGridMap();

  //=======================================================================
  // Publishers
  //=======================================================================
  octomap_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("octomap_map", 2);
  grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("social_grid_map", 1);

  //=======================================================================
  // Subscribers
  //=======================================================================

  // Agent states callback
  agent_states_sub_ = this->create_subscription<pedsim_msgs::msg::AgentStates>(
      social_agents_topic_, 1, std::bind(&WorldModeler::agentStatesCallback, this, std::placeholders::_1));

  // Odometry data (feedback)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 1, std::bind(&WorldModeler::odomCallback, this, std::placeholders::_1));

  // Map erase command
  erase_map_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      erase_map_topic_, 1, std::bind(&WorldModeler::eraseMapCallback, this, std::placeholders::_1));

  nav_sts_available_ = false;
  if (!nav_sts_available_)
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry.");

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok() && !nav_sts_available_)
  {
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
  RCLCPP_WARN(this->get_logger(), "Odometry received.");

  if (offline_octomap_path_.size() == 0)
  {
    // POINTCLOUD
    std::vector<std::string> pc_need_frames;
    pc_need_frames.push_back(point_cloud_frame_);
    pc_need_frames.push_back(fixed_frame_);
    pc_need_frames.push_back(robot_frame_);
    point_cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, point_cloud_topic_);
    point_cloud_mn_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *tf_buffer_,
        point_cloud_frame_,
        5,
        this->get_node_logging_interface(),
        this->get_node_clock_interface());
    point_cloud_mn_->connectInput(*point_cloud_sub_);
    point_cloud_mn_->setTargetFrames(pc_need_frames);
    point_cloud_mn_->registerCallback(&WorldModeler::pointCloudCallback, this);

    // LASERSCAN
    std::vector<std::string> laser_need_frames;
    laser_need_frames.push_back(point_cloud_frame_);
    laser_need_frames.push_back(fixed_frame_);
    laser_need_frames.push_back(robot_frame_);

    laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, laser_scan_topic_);
    laser_scan_mn_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *tf_buffer_,
        laser_scan_frame_,
        5,
        this->get_node_logging_interface(),
        this->get_node_clock_interface());
    laser_scan_mn_->connectInput(*laser_scan_sub_);
    laser_scan_mn_->setTargetFrames(laser_need_frames);
    laser_scan_mn_->registerCallback(&WorldModeler::laserScanCallback, this);
  }

  //=======================================================================
  // Services
  //=======================================================================
  save_binary_octomap_srv_ = this->create_service<std_srvs::srv::Empty>(
      "save_binary", std::bind(&WorldModeler::saveBinaryOctomapSrv, this, std::placeholders::_1, std::placeholders::_2));
  save_full_octomap_srv_ = this->create_service<std_srvs::srv::Empty>(
      "save_full", std::bind(&WorldModeler::saveFullOctomapSrv, this, std::placeholders::_1, std::placeholders::_2));
  get_binary_octomap_srv_ = this->create_service<octomap_msgs::srv::GetOctomap>(
      "get_binary", std::bind(&WorldModeler::getBinaryOctomapSrv, this, std::placeholders::_1, std::placeholders::_2));
  get_grid_map_srv_ = this->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_grid_map", std::bind(&WorldModeler::getGridMapSrv, this, std::placeholders::_1, std::placeholders::_2));

  // Timer for publishing
  if (rviz_timer_ > 0.0)
  {
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(rviz_timer_), std::bind(&WorldModeler::timerCallback, this));
  }
}

//! Destructor.
WorldModeler::~WorldModeler()
{
  RCLCPP_INFO(this->get_logger(), "Octree has been deleted.");
  delete octree_;
}

void WorldModeler::initializeGridMap()
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  grid_map_ = grid_map::GridMap();
  grid_map_.setFrameId(map_frame_);
  grid_map_.add("full");
  grid_map_.add("comfort");
  grid_map_.setGeometry(grid_map::Length(1, 1), octree_resol_);
}

void WorldModeler::eraseMap()
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  octree_->clear();
  initializeGridMap();

  relevant_agent_states_.agent_states.clear();
  social_agents_in_radius_.agent_states.clear();
  social_agents_in_radius_vector_.clear();
  orientation_drift_ = 0.0;
  position_drift_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Map erased. Starting a fresh map.");
}

void WorldModeler::eraseMapCallback(const std_msgs::msg::Bool::SharedPtr erase_map_msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  if (!erase_map_msg->data)
  {
    return;
  }

  eraseMap();
  skip_sensor_callbacks_after_erase_ = true;
  skip_sensor_callbacks_until_ = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  RCLCPP_WARN(this->get_logger(), "Skipping sensor callbacks for 3 seconds after erase_map");
}

//! Laserscan callback.
/*!
 * Callback for receiving the laserscan data (taken from octomap_server)
 */
void WorldModeler::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  if (skip_sensor_callbacks_after_erase_)
  {
    if (std::chrono::steady_clock::now() < skip_sensor_callbacks_until_)
    {
      return;
    }

    skip_sensor_callbacks_after_erase_ = false;
    RCLCPP_WARN(this->get_logger(), "Sensor callbacks resumed after erase_map cooldown");
  }

  geometry_msgs::msg::TransformStamped tf_robot_to_laser_scan, tf_fixed_to_robot, tf_map_to_fixed;

  try
  {
    // Check drift
    tf_map_to_fixed = tf_buffer_->lookupTransform(map_frame_, fixed_frame_, tf2::TimePointZero);
    tf2::Matrix3x3 m_map_to_fixed(tf2::Quaternion(
        tf_map_to_fixed.transform.rotation.x,
        tf_map_to_fixed.transform.rotation.y,
        tf_map_to_fixed.transform.rotation.z,
        tf_map_to_fixed.transform.rotation.w));
    tf2::Vector3 p_map_to_fixed(
        tf_map_to_fixed.transform.translation.x,
        tf_map_to_fixed.transform.translation.y,
        tf_map_to_fixed.transform.translation.z);

    double roll, pitch, yaw;
    m_map_to_fixed.getRPY(roll, pitch, yaw);
    orientation_drift_ += std::abs(prev_map_to_fixed_yaw_ - yaw);
    prev_map_to_fixed_yaw_ = yaw;
    position_drift_ += std::sqrt(std::pow(prev_map_to_fixed_pos_.x() - p_map_to_fixed.x(), 2.0) +
                                 std::pow(prev_map_to_fixed_pos_.y() - p_map_to_fixed.y(), 2.0));
    prev_map_to_fixed_pos_ = p_map_to_fixed;

    if (orientation_drift_ > 0.05 || position_drift_ > 0.05)
    {
      orientation_drift_ = 0.0;
      position_drift_ = 0.0;
      octree_->clear();
      // mergeGlobalMapToOctomap();
    }

    tf_robot_to_laser_scan = tf_buffer_->lookupTransform(robot_frame_, laser_scan_msg->header.frame_id, tf2::TimePointZero);
    tf_fixed_to_robot = tf_buffer_->lookupTransform(fixed_frame_, robot_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  // Editable message
  sensor_msgs::msg::LaserScan laser_scan = *laser_scan_msg;

  // Max range flags
  std::vector<bool> rngflags;
  rngflags.resize(laser_scan.ranges.size());
  for (int i = 0; i < laser_scan.ranges.size(); i++)
  {
    rngflags[i] = false;
    if (!rclcpp::ok())
      break;
  }

  // Use zero ranges as max ranges
  double new_max = laser_scan.range_max * 0.95;
  if (add_max_ranges_)
  {
    // Flag readings as maxrange
    for (int i = 0; i < laser_scan.ranges.size(); i++)
    {
      if (laser_scan.ranges[i] == 0.0 || laser_scan.ranges[i] >= laser_scan.range_max)
      {
        laser_scan.ranges[i] = new_max;
        rngflags[i] = true;
      }
    }
  }

  // Apply single outliers removal filter
  if (apply_filter_)
  {
    // Copy message for editing, filter and project to (x,y,z)
    sensor_msgs::msg::LaserScan laser_scan = *laser_scan_msg;
    filterSingleOutliers(laser_scan, rngflags);
  }

  // Project to (x,y,z)
  laser_scan_projector_.projectLaser(laser_scan, cloud_, laser_scan.range_max);

  // Compute origin of sensor in world frame (filters laser_scan_msg->ranges[i] > 0.0)
  geometry_msgs::msg::PoseStamped orig;
  tf2::doTransform(orig, orig, tf_fixed_to_robot);
  tf2::doTransform(orig, orig, tf_robot_to_laser_scan);
  octomap::point3d origin(orig.pose.position.x, orig.pose.position.y, orig.pose.position.z);

  // ! AGENTS FILTERING

  // =============================

  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(cloud_, pc);

  float minX = -12, minY = -0.6, minZ = -0.6;
  float maxX = 12, maxY = 0.6, maxZ = 0.6;

  geometry_msgs::msg::TransformStamped transform;

  if (social_agents_in_radius_.agent_states.size() > 0)
  {
    for (int i = 0; i < social_agents_in_radius_.agent_states.size(); i++)
    {
      std::string agent_frame = "agent_" + std::to_string(social_agents_in_radius_.agent_states[i].id);
      try
      {
        transform = tf_buffer_->lookupTransform(laser_scan_msg->header.frame_id, agent_frame, laser_scan_msg->header.stamp);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "Transform error of sensor data: %s. Getting the latest obtained transform.", ex.what());
        transform = tf_buffer_->lookupTransform(laser_scan_msg->header.frame_id, agent_frame, tf2::TimePointZero);
      }

      // Z -> X
      // X -> Y
      // Y -> -Z
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0));
      boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 0));
      boxFilter.setInputCloud(pc.makeShared());
      boxFilter.setTranslation(Eigen::Vector3f(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z));
      // boxFilter.setRotation(Eigen::Vector3f(roll, pitch + 90, yaw));
      boxFilter.setNegative(true);
      boxFilter.filter(pc);
    }
  }

  geometry_msgs::msg::TransformStamped sensorToWorldTf;
  try
  {
    sensorToWorldTf = tf_buffer_->lookupTransform(fixed_frame_, laser_scan_msg->header.frame_id, laser_scan_msg->header.stamp);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, quitting callback", ex.what());
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  // set up filter for height range, also removes NANs:
  // pcl::PassThrough<PCLPoint> pass_x;
  // pass_x.setFilterFieldName("x");
  // pass_x.setFilterLimits(0.15, 4.0);
  // pcl::PassThrough<PCLPoint> pass_y;
  // pass_y.setFilterFieldName("y");
  // pass_y.setFilterLimits(0.15, 4.0);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(min_z_pc_, max_z_pc_); // TODO

  PCLPointCloud pc_ground;    // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  // directly transform to map frame:
  pcl::transformPointCloud(pc, pc, sensorToWorld);

  // just filter height range:
  // pass_x.setInputCloud(pc.makeShared());
  // pass_x.filter(pc);
  // pass_y.setInputCloud(pc.makeShared());
  // pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

  pc_nonground = pc;

  // pc_nonground is empty without ground segmentation
  pc_ground.header = pc.header;
  pc_nonground.header = pc.header;

  insertScan(sensorToWorldTf.transform.translation, pc_ground, pc_nonground, mapping_max_range_, minimum_range_);
}

//! Pointcloud callback.
/*!
 * Callback for receiving the pointcloud data (taken from octomap_server)
 */
void WorldModeler::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  if (skip_sensor_callbacks_after_erase_)
  {
    if (std::chrono::steady_clock::now() < skip_sensor_callbacks_until_)
    {
      return;
    }

    skip_sensor_callbacks_after_erase_ = false;
    RCLCPP_WARN(this->get_logger(), "Sensor callbacks resumed after erase_map cooldown");
  }

  // ROS_INFO_STREAM("PROCESSING POINTCLOUD");
  //
  // ground filtering in base frame
  //
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  float minX = -0.6, minY = -0.6, minZ = -12;
  float maxX = 0.6, maxY = 0.6, maxZ = 12;

  // ROS_INFO_STREAM("ABOUT TO PROCESS AGENTS");

  if (social_agents_in_radius_.agent_states.size() > 0)
  {
    for (int i = 0; i < social_agents_in_radius_.agent_states.size(); i++)
    {
      std::string err = "cannot find transform from agent to camera frame";

      geometry_msgs::msg::TransformStamped transform;
      rclcpp::Time t;

      try
      {
        transform = tf_buffer_->lookupTransform(cloud->header.frame_id, "agent_" + std::to_string(social_agents_in_radius_.agent_states[i].id), cloud->header.stamp);

        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Z -> X
        // X -> Y
        // Y -> -Z
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0));
        boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 0));
        boxFilter.setInputCloud(pc.makeShared());
        boxFilter.setTranslation(Eigen::Vector3f(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z));
        boxFilter.setRotation(Eigen::Vector3f(roll, pitch, yaw));
        boxFilter.setNegative(true);
        boxFilter.filter(pc);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, quitting callback", ex.what());
        return;
      }
    }
  }
  // ROS_INFO_STREAM("ABOUT TO PROCESS AGENTS FINISHED");

  rclcpp::Time t;
  std::string err = "cannot find transform from robot_frame to scan frame";

  geometry_msgs::msg::TransformStamped sensorToWorldTf;
  try
  {
    //        tf_listener_.lookupTransform(robot_frame_,
    //        laser_scan_msg->header.frame_id, t,
    //                                     tf_robot_to_laser_scan);
    sensorToWorldTf = tf_buffer_->lookupTransform(fixed_frame_, cloud->header.frame_id, cloud->header.stamp);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, quitting callback", ex.what());
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  // set up filter for height range, also removes NANs:
  // pcl::PassThrough<PCLPoint> pass_x;
  // pass_x.setFilterFieldName("x");
  // pass_x.setFilterLimits(0.15, 4.0);
  // pcl::PassThrough<PCLPoint> pass_y;
  // pass_y.setFilterFieldName("y");
  // pass_y.setFilterLimits(0.15, 4.0);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(min_z_pc_, max_z_pc_); // TODO

  PCLPointCloud pc_ground;    // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  // directly transform to map frame:
  pcl::transformPointCloud(pc, pc, sensorToWorld);

  // just filter height range:
  // pass_x.setInputCloud(pc.makeShared());
  // pass_x.filter(pc);
  // pass_y.setInputCloud(pc.makeShared());
  // pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

  pc_nonground = pc;

  // pc_nonground is empty without ground segmentation
  pc_ground.header = pc.header;
  pc_nonground.header = pc.header;

  // ROS_INFO_STREAM("INSERT SCAN START");

  insertScan(sensorToWorldTf.transform.translation, pc_ground, pc_nonground, mapping_max_range_, minimum_range_);
  //
  //    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  //    ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts
  //    (ground/nonground), %f sec)",
  //              pc_ground.size(), pc_nonground.size(), total_elapsed);
  //
  //    publishAll(cloud->header.stamp);

  defineSocialGridMap();
}

void WorldModeler::insertScan(const geometry_msgs::msg::Vector3 &sensorOriginTf,
                              const PCLPointCloud &ground,
                              const PCLPointCloud &nonground, const double &max_range, const double &min_range)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  octomap::point3d sensorOrigin(sensorOriginTf.x, sensorOriginTf.y, sensorOriginTf.z);

  if (!octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMin) ||
      !octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    RCLCPP_ERROR(this->get_logger(), "Could not generate Key for origin %f %f %f",
                 sensorOrigin.x(), sensorOrigin.y(), sensorOrigin.z());
  }

#ifdef COLOR_OCTOMAP_SERVER
  unsigned char *colors = new unsigned char[3];
#endif

  // instead of direct scan insertion, compute update to filter ground:
  octomap::KeySet free_cells, occupied_cells;

  // all other points: free on ray, occupied on endpoint:
  double m_maxRange(max_range); // TODO
  double m_minRange(min_range);
  for (PCLPointCloud::const_iterator it = nonground.begin();
       it != nonground.end(); ++it)
  {
    octomap::point3d point(it->x, it->y, it->z); // TODO
    double point_distance = (point - sensorOrigin).norm();

    if (m_minRange >= 0.0 && point_distance < m_minRange)
    {
      continue;
    }

    // maxrange check
    if ((m_maxRange < 0.0) || (point_distance <= m_maxRange))
    {

      // free cells
      if (octree_->computeRayKeys(sensorOrigin, point, m_keyRay))
      {
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(point, key))
      {
        occupied_cells.insert(key);

        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);
      }
    }
    else
    {
      // ray longer than maxrange:;
      octomap::point3d new_end =
          sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (octree_->computeRayKeys(sensorOrigin, new_end, m_keyRay))
      {
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (octree_->coordToKeyChecked(new_end, endKey))
        {
          free_cells.insert(endKey);
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Could not generate Key for endpoint %f %f %f",
                       new_end.x(), new_end.y(), new_end.z());
        }
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (octomap::KeySet::iterator it = free_cells.begin(),
                                 end = free_cells.end();
       it != end; ++it)
  {
    if (occupied_cells.find(*it) == occupied_cells.end())
    {
      octree_->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (octomap::KeySet::iterator it = occupied_cells.begin(),
                                 end = occupied_cells.end();
       it != end; it++)
  {
    octree_->updateNode(*it, true);
  }
}

//! Odometry callback.
/*!
 * Callback for getting updated vehicle odometry.
 */
void WorldModeler::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  if (!nav_sts_available_)
    nav_sts_available_ = true;

  robot_odometry_ = odom_msg;
}

//! Agent States callback.
/*!
 * Callback for getting updated agent states.
 */
void WorldModeler::agentStatesCallback(const pedsim_msgs::msg::AgentStates::SharedPtr agent_states_msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  if (nav_sts_available_)
  {
    agent_states_ = agent_states_msg;

    std::vector<pedsim_msgs::msg::AgentState> agent_state_vector;

    social_agents_in_radius_vector_.clear();

    double robot_velocity =
        std::sqrt(std::pow(robot_odometry_->twist.twist.linear.x, 2) + std::pow(robot_odometry_->twist.twist.linear.y, 2));

    actual_fov_distance_ = robot_distance_view_max_ / robot_velocity_threshold_ * robot_velocity;

    if (actual_fov_distance_ < robot_distance_view_min_)
    {
      actual_fov_distance_ = robot_distance_view_min_;
    }
    else if (actual_fov_distance_ > robot_distance_view_max_)
    {
      actual_fov_distance_ = robot_distance_view_max_;
    }

    for (int i = 0; i < agent_states_msg->agent_states.size(); i++)
    {
      if (this->isAgentInRFOV(agent_states_msg->agent_states[i]))
      {
        agent_state_vector.push_back(agent_states_msg->agent_states[i]);
      }
    }

    relevant_agent_states_.agent_states = agent_state_vector;

    social_agents_in_radius_.agent_states = social_agents_in_radius_vector_;
  }
}

bool WorldModeler::isAgentInRFOV(const pedsim_msgs::msg::AgentState agent_state)
{
  if (!social_relevance_validity_checking_)
  {
    return true;
  }

  double d_robot_agent = std::sqrt(std::pow(agent_state.pose.position.x - robot_odometry_->pose.pose.position.x, 2) +
                                   std::pow(agent_state.pose.position.y - robot_odometry_->pose.pose.position.y, 2));

  if (d_robot_agent < mapping_max_range_ + 2)
  {
    social_agents_in_radius_vector_.push_back(agent_state);
  }

  if (d_robot_agent > actual_fov_distance_)
  {
    return false;
  }

  double tetha_robot_agent = atan2((agent_state.pose.position.y - robot_odometry_->pose.pose.position.y),
                                   (agent_state.pose.position.x - robot_odometry_->pose.pose.position.x));

  if (tetha_robot_agent < 0)
  {
    tetha_robot_agent = 2 * M_PI + tetha_robot_agent;
  }

  tf2::Quaternion q(robot_odometry_->pose.pose.orientation.x, robot_odometry_->pose.pose.orientation.y,
                    robot_odometry_->pose.pose.orientation.z, robot_odometry_->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double robot_angle = yaw;

  if (robot_angle < 0)
  {
    robot_angle = 2 * M_PI + robot_angle;
  }

  if (tetha_robot_agent > (robot_angle + M_PI))
    tetha_robot_agent = abs(robot_angle + 2 * M_PI - tetha_robot_agent);
  else if (robot_angle > (tetha_robot_agent + M_PI))
    tetha_robot_agent = abs(tetha_robot_agent + 2 * M_PI - robot_angle);
  else
    tetha_robot_agent = abs(tetha_robot_agent - robot_angle);

  return abs(tetha_robot_agent) < robot_angle_view_;
}

bool WorldModeler::isRobotInFront(const pedsim_msgs::msg::AgentState agent_state, const grid_map::Position position)
{
  double tetha_agent_robot = atan2((position[1] - agent_state.pose.position.y),
                                   (position[0] - agent_state.pose.position.x));
  if (tetha_agent_robot < 0)
  {
    tetha_agent_robot = 2 * M_PI + tetha_agent_robot;
  }

  tf2::Quaternion q(agent_state.pose.orientation.x, agent_state.pose.orientation.y,
                    agent_state.pose.orientation.z, agent_state.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double agent_angle = yaw;

  if (agent_angle < 0)
  {
    agent_angle = 2 * M_PI + agent_angle;
  }

  if (tetha_agent_robot > (agent_angle + M_PI))
    tetha_agent_robot = abs(agent_angle + 2 * M_PI - tetha_agent_robot);
  else if (agent_angle > (tetha_agent_robot + M_PI))
    tetha_agent_robot = abs(tetha_agent_robot + 2 * M_PI - agent_angle);
  else
    tetha_agent_robot = abs(tetha_agent_robot - agent_angle);

  if (abs(tetha_agent_robot) < robot_angle_view_)
    return true;

  return false;
}

// !CALCULATE COMFORT AT AN SPECIFIC POSITION IN THE GRIDMAP

double WorldModeler::getExtendedPersonalSpace(const pedsim_msgs::msg::AgentState agent_state, const grid_map::Position position)
{
  double distance_robot_agent = std::sqrt(std::pow(agent_state.pose.position.x - position[0], 2) +
                                          std::pow(agent_state.pose.position.y - position[1], 2));

  double tetha_robot_agent = atan2((position[1] - agent_state.pose.position.y),
                                   (position[0] - agent_state.pose.position.x));

  if (tetha_robot_agent < 0)
  {
    tetha_robot_agent = 2 * M_PI + tetha_robot_agent;
  }

  double tetha_orientation;
  if (abs(agent_state.twist.linear.x) > 0 || abs(agent_state.twist.linear.y) > 0)
  {
    tetha_orientation = atan2(agent_state.twist.linear.y, agent_state.twist.linear.x);
  }
  else
  {
    tf2::Quaternion q(agent_state.pose.orientation.x, agent_state.pose.orientation.y,
                      agent_state.pose.orientation.z, agent_state.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tetha_orientation = yaw;
  }

  if (tetha_orientation < 0)
  {
    tetha_orientation = 2 * M_PI + tetha_orientation;
  }

  bool robot_in_front = false;
  bool robot_in_fov = false;
  double mod_sigma_y;
  double agent_velocity;

  agent_velocity =
      std::sqrt(std::pow(agent_state.twist.linear.x, 2) + std::pow(agent_state.twist.linear.y, 2));

  robot_in_front = this->isRobotInFront(agent_state, position);

  if (robot_in_front)
  {
    if (robot_in_fov)
      mod_sigma_y = (1 + agent_velocity * fv + fFront + fFieldOfView) * sigma_y;
    else
      mod_sigma_y = (1 + agent_velocity * fv + fFront) * sigma_y;
  }
  else
  {
    mod_sigma_y = sigma_y;
  }

  double basic_personal_space_value =
      social_comfort_amplitude_ *
      std::exp(-(
          std::pow(distance_robot_agent * std::sin(tetha_robot_agent - tetha_orientation) / (std::sqrt(2) * sigma_x),
                   2) +
          std::pow(distance_robot_agent * std::cos(tetha_robot_agent - tetha_orientation) / (std::sqrt(2) * mod_sigma_y),
                   2)));

  return basic_personal_space_value;
}

//! Time callback.
/*!
 * Callback for publishing the map periodically using the WorldModeler RViz plugin.
 */
void WorldModeler::timerCallback()
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  if (offline_octomap_path_.size() != 0)
  {
    defineSocialGridMap();
  }

  // Declare message
  octomap_msgs::msg::Octomap msg;
  octomap_msgs::binaryMapToMsg(*octree_, msg);
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = fixed_frame_;

  if (visualize_free_space_)
  {
    publishMap();

    grid_map_.setTimestamp(this->get_clock()->now().nanoseconds());
    std::shared_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(grid_map_);
    grid_map_pub_->publish(*message);
  }
}

//! Save binary service
/*!
 * Service for saving the binary Octomap into the home folder
 */
bool WorldModeler::saveBinaryOctomapSrv(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  // Saves current octree_ in home folder
  std::string fpath(getenv("HOME"));
  octree_->writeBinary(fpath + "/map_laser_octomap.bt");
  return true;
}

//! Save binary service
/*!
 * Service for saving the full Octomap into the home folder
 */
bool WorldModeler::saveFullOctomapSrv(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  // Saves current octree_ in home folder (full probabilities)
  std::string fpath(getenv("HOME"));
  octree_->write(fpath + "/map_laser_octomap.ot");
  return true;
}

//! Get binary service
/*!
 * Service for getting the binary Octomap
 */
bool WorldModeler::getBinaryOctomapSrv(
    const std::shared_ptr<OctomapSrv::Request> req,
    std::shared_ptr<OctomapSrv::GetOctomap::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  RCLCPP_INFO(get_logger(), "Sending binary map data on service request");

  res->map.header.frame_id = fixed_frame_;
  res->map.header.stamp = this->get_clock()->now();

  if (!octomap_msgs::binaryMapToMsg(*octree_, res->map))
    return false;

  return true;
}

//! Get binary service
/*!
 * Service for getting the binary Octomap
 */
bool WorldModeler::getGridMapSrv(
    const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> req,
    std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  RCLCPP_INFO(get_logger(), "Sending grid map data on service");

  grid_map_.setTimestamp(this->get_clock()->now().nanoseconds());

  res->map = *grid_map::GridMapRosConverter::toMessage(grid_map_);

  return true;
}

//! Publish the map
/*!
 * Service for saving the binary of the Octomap into the home folder
 */
void WorldModeler::publishMap()
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  // Declare message and resize
  visualization_msgs::msg::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(octree_->getTreeDepth() + 1);

  // Octree limits for height map
  double min_x, min_y, max_x, max_y, min_z_, max_z_;
  octree_->getMetricMin(min_x, min_y, min_z_);
  octree_->getMetricMax(max_x, max_y, max_z_);

  // Traverse all leafs in the tree:
  for (octomap::OcTree::iterator it = octree_->begin(octree_->getTreeDepth()),
                                 end = octree_->end();
       it != end; ++it)
  {
    if (octree_->isNodeOccupied(*it))
    {
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();

      // Create marker:
      unsigned idx = it.getDepth();
      assert(idx < occupiedNodesVis.markers.size());

      geometry_msgs::msg::Point cubeCenter;
      cubeCenter.x = x;
      cubeCenter.y = y;
      cubeCenter.z = z;

      occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
    }

    if (!rclcpp::ok())
      break;
  }
  // Finish Headers and options
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
  {
    double size = octree_->getNodeSize(i);

    occupiedNodesVis.markers[i].header.frame_id = fixed_frame_;
    occupiedNodesVis.markers[i].header.stamp = this->get_clock()->now();
    occupiedNodesVis.markers[i].ns = fixed_frame_;
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color.r = 0.5;
    occupiedNodesVis.markers[i].color.g = 0.5;
    occupiedNodesVis.markers[i].color.b = 0.5;
    occupiedNodesVis.markers[i].color.a = 1.0;

    if (occupiedNodesVis.markers[i].points.size() > 0)
      occupiedNodesVis.markers[i].action = visualization_msgs::msg::Marker::ADD;
    else
      occupiedNodesVis.markers[i].action = visualization_msgs::msg::Marker::DELETE;

    if (!rclcpp::ok())
      break;
  }

  // Publish it
  octomap_marker_pub_->publish(occupiedNodesVis);
}

void WorldModeler::filterSingleOutliers(sensor_msgs::msg::LaserScan &laser_scan_msg, std::vector<bool> &rngflags)
{
  int n(laser_scan_msg.ranges.size());
  double thres(laser_scan_msg.range_max / 10.0);
  std::vector<int> zeros;
  for (int i = 1; i < n - 1; i++)
  {
    if ((std::abs(laser_scan_msg.ranges[i - 1] - laser_scan_msg.ranges[i]) > thres) &&
        (std::abs(laser_scan_msg.ranges[i + 1] - laser_scan_msg.ranges[i]) > thres))
    {
      laser_scan_msg.ranges[i] = 0.0;
      zeros.push_back(i);
    }

    if (!rclcpp::ok())
      break;
  }

  // Delete unnecessary flags
  for (std::vector<int>::reverse_iterator rit = zeros.rbegin();
       rit < zeros.rend(); rit++)
  {
    rngflags.erase(rngflags.begin() + *rit);

    if (!rclcpp::ok())
      break;
  }
}

void WorldModeler::defineSocialGridMap()
{
  std::lock_guard<std::recursive_mutex> lock(map_state_mutex_);

  // ! OCTOMAP PREPARATION

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;

  grid_map_.clearAll();

  octree_->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  octree_->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

  grid_map::GridMapOctomapConverter::fromOctomap(*octree_, "full", grid_map_, &min_bound, &max_bound);
  if (!grid_map_.exists("full"))
  {
    RCLCPP_WARN(this->get_logger(), "No full layer after fromOctomap, skipping defineSocialGridMap");
    return;
  }
  if (!grid_map_.exists("comfort"))
  {
    grid_map_.add("comfort");
  }

  grid_map_["full"] = 150 * grid_map_["full"];

  grid_map::Matrix &full_grid_map = grid_map_["full"];
  grid_map::Matrix &comfort_grid_map = grid_map_["comfort"];

  // !SOCIAL AGENTS GRID MAP PREPARATION

  for (int i = 0; i < relevant_agent_states_.agent_states.size(); i++)
  {
    grid_map::Position center(relevant_agent_states_.agent_states[i].pose.position.x, relevant_agent_states_.agent_states[i].pose.position.y);

    for (grid_map::CircleIterator iterator(grid_map_, center, social_agent_radius_);
         !iterator.isPastEnd(); ++iterator)
    {
      try
      {
        grid_map::Index index(*iterator);
        full_grid_map(index(0), index(1)) = 100;
      }
      catch (const std::out_of_range &oor)
      {
        RCLCPP_ERROR(this->get_logger(), "TRIED TO DEFINE AN AGENT OUT OF RANGE");
      }
    }

    for (grid_map::CircleIterator iterator(grid_map_, center, 2.25);
         !iterator.isPastEnd(); ++iterator)
    {
      try
      {

        grid_map::Position temp_pos;

        grid_map_.getPosition(*iterator, temp_pos);

        grid_map::Index index(*iterator);

        double last_val = comfort_grid_map(index(0), index(1));

        if (std::isnan(last_val))
        {
          last_val = getExtendedPersonalSpace(relevant_agent_states_.agent_states[i], temp_pos);
        }
        else
        {
          last_val += getExtendedPersonalSpace(relevant_agent_states_.agent_states[i], temp_pos);
        }

        comfort_grid_map(index(0), index(1)) = last_val;
      }
      catch (const std::out_of_range &oor)
      {
        RCLCPP_ERROR(this->get_logger(), "TRIED TO DEFINE COMFORT OUT OF RANGE");
      }
    }
  }

  grid_map_["full"] = full_grid_map;
  grid_map_["comfort"] = comfort_grid_map;
}

//! Main function
int main(int argc, char **argv)
{
  //=======================================================================
  // Override SIGINT handler
  //=======================================================================
  signal(SIGINT, stopNode);

  // Init ROS node
  rclcpp::init(argc, argv);

  // Spin
  rclcpp::spin(std::make_shared<WorldModeler>());

  rclcpp::shutdown();

  // Exit main function without errors
  return 0;
}
