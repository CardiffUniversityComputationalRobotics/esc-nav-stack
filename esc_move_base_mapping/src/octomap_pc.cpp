/*! \file octomap_laser_scan.cpp
 * \brief Merge data from different laser_scan messages to incrementally build
 * an Octomap.
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

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ROS LaserScan tools
#include <laser_geometry/laser_geometry.h>

// ROS messages
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>

// ROS services
#include <std_srvs/Empty.h>

// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
typedef octomap_msgs::GetOctomap OctomapSrv;
#include <octomap/Pointcloud.h>
#include <octomap_ros/conversions.h>

// grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>

// PEDSIM
#include <pedsim_msgs/AgentStates.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef octomap::OcTree OcTreeT;

#include <signal.h>

void stopNode(int sig)
{
  ros::shutdown();
  exit(0);
}

struct PointCloudExtended
{
  std::string topic;
  std::string frame;
  tf::StampedTransform tf_robot_to_point_cloud;
  ros::Subscriber sub;
};

//!  Octomap class.
/*!
 * Autopilot Laser Octomap.
 * Create an Octomap using information from laser scans.
 */
class Octomap
{
public:
  //! Constructor
  Octomap();
  //! Destructor
  virtual ~Octomap();
  //! Callback for getting the point_cloud data
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
  //! Callback for getting current vehicle odometry
  void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
  //! Callback for getting current agent states
  void agentStatesCallback(const pedsim_msgs::AgentStatesConstPtr &agent_states_msg);
  bool isAgentInRFOV(const pedsim_msgs::AgentState agent_state);
  //! Periodic callback to publish the map for visualization.
  void timerCallback(const ros::TimerEvent &e);
  void insertScan(const tf::Point &sensorOriginTf, const PCLPointCloud &ground,
                  const PCLPointCloud &nonground);
  //! Service to get grid map
  bool getGridMapSrv(grid_map_msgs::GetGridMap::Request &req,
                     grid_map_msgs::GetGridMap::Response &res);

  //! Publish the Octomap
  void publishMap();

private:
  // ROS
  ros::NodeHandle nh_, local_nh_;
  ros::Publisher octomap_marker_pub_, grid_map_pub_, relevant_agents_pub_;
  ros::Subscriber odom_sub_, agent_states_sub_;
  ros::ServiceServer get_grid_map_srv_;
  ros::Timer timer_;

  // ROS tf
  tf::TransformListener tf_listener_;

  // Names
  std::string map_frame_, fixed_frame_, robot_frame_, offline_octomap_path_,
      odometry_topic_, social_agents_topic_;

  // Point Clouds
  std::vector<std::string> point_cloud_topics_, point_cloud_frames_;
  std::vector<PointCloudExtended *> point_clouds_info_;

  // ROS Messages
  sensor_msgs::PointCloud cloud_;

  nav_msgs::OdometryConstPtr robot_odometry_;

  // pedsim messages
  pedsim_msgs::AgentStatesConstPtr agent_states_;
  pedsim_msgs::AgentStates relevant_agent_states_;

  pedsim_msgs::AgentStates social_agents_in_radius_;
  std::vector<pedsim_msgs::AgentState> social_agents_in_radius_vector_;

  double social_agent_radius_;

  // Octree
  octomap::OcTree *octree_;
  double octree_resol_, rviz_timer_;
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;
  octomap::KeyRay m_keyRay; // temp storage for ray casting

  // grid map
  grid_map::GridMap grid_map_;

  grid_map_msgs::GridMap grid_map_msg_;

  // social relevance validity checking constants
  double robot_distance_view_;
  double robot_velocity_threshold_;
  double robot_angle_view_;

  // Flags
  bool initialized_;
  bool nav_sts_available_;
  bool visualize_free_space_;
  bool social_relevance_validity_checking_;

protected:
  inline static void updateMinKey(const octomap::OcTreeKey &in,
                                  octomap::OcTreeKey &min)
  {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };
  inline static void updateMaxKey(const octomap::OcTreeKey &in,
                                  octomap::OcTreeKey &max)
  {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };
};

//!  Constructor.
/*!
 * Load map parameters.
 * Subscribers to odometry and laser scan
 * Publishers to visualize the Octomap.
 */
Octomap::Octomap()
    : nh_(),
      local_nh_("~"),
      fixed_frame_("/fixed_frame"),
      robot_frame_("/robot_frame"),
      odometry_topic_("/odometry_topic"),
      offline_octomap_path_(""),
      octree_(NULL),
      octree_resol_(1.0),
      initialized_(false),
      visualize_free_space_(false),
      rviz_timer_(0.0),
      robot_distance_view_(6.0),
      robot_angle_view_(1.57),
      robot_velocity_threshold_(0.3),
      social_agent_radius_(0.4),
      social_agents_topic_("/pedsim_simulator/simulated_agents"),
      social_relevance_validity_checking_(true)
{
  //=======================================================================
  // Get parameters
  //=======================================================================
  local_nh_.param("resolution", octree_resol_, octree_resol_);
  local_nh_.param("map_frame", map_frame_, map_frame_);
  local_nh_.param("fixed_frame", fixed_frame_, fixed_frame_);
  local_nh_.param("robot_frame", robot_frame_, robot_frame_);
  local_nh_.param("offline_octomap_path", offline_octomap_path_,
                  offline_octomap_path_);
  local_nh_.param("visualize_free_space", visualize_free_space_,
                  visualize_free_space_);
  local_nh_.param("odometry_topic", odometry_topic_, odometry_topic_);
  local_nh_.param("rviz_timer", rviz_timer_, rviz_timer_);
  local_nh_.param("point_cloud_topics", point_cloud_topics_,
                  point_cloud_topics_);
  local_nh_.param("point_cloud_frames", point_cloud_frames_,
                  point_cloud_frames_);
  local_nh_.param("robot_distance_view", robot_distance_view_, robot_distance_view_);
  local_nh_.param("robot_angle_view", robot_angle_view_, robot_angle_view_);
  local_nh_.param("robot_velocity_threshold", robot_velocity_threshold_, robot_velocity_threshold_);
  local_nh_.param("social_agent_radius", social_agent_radius_, social_agent_radius_);
  local_nh_.param("social_agents_topic", social_agents_topic_, social_agents_topic_);
  local_nh_.param("social_relevance_validity_checking", social_relevance_validity_checking_, social_relevance_validity_checking_);

  // Transforms TF and catch the static transform from vehicle to laser_scan
  // sensor
  // tf_listener_.setExtrapolationLimit(ros::Duration(0.2));
  int count(0);
  ros::Time t;
  std::string err = "";

  if (point_cloud_frames_.size() == point_cloud_topics_.size())
  {
    for (unsigned int i = 0; i < point_cloud_frames_.size(); i++)
    {
      PointCloudExtended *point_cloud_info = new PointCloudExtended();
      point_cloud_info->frame = point_cloud_frames_[i];
      point_cloud_info->topic = point_cloud_topics_[i];

      // Get the corresponding tf
      count = 0;
      err = "cannot find tf from " + robot_frame_ + "to " +
            point_cloud_info->frame;

      tf_listener_.getLatestCommonTime(robot_frame_, point_cloud_info->frame, t,
                                       &err);

      initialized_ = false;
      do
      {
        try
        {
          tf_listener_.lookupTransform(
              robot_frame_, point_cloud_info->frame, t,
              point_cloud_info->tf_robot_to_point_cloud);
          initialized_ = true;
        }
        catch (std::exception e)
        {
          tf_listener_.waitForTransform(robot_frame_, point_cloud_info->frame,
                                        ros::Time::now(), ros::Duration(1.0));
          tf_listener_.getLatestCommonTime(robot_frame_,
                                           point_cloud_info->frame, t, &err);
          count++;
          ROS_WARN("%s:\n\tCannot find tf from %s to %s\n",
                   ros::this_node::getName().c_str(), robot_frame_.c_str(),
                   point_cloud_info->frame.c_str());
        }
        if (count > 10)
        {
          ROS_ERROR("%s\n\tNo transform found. Aborting...",
                    ros::this_node::getName().c_str());
          exit(-1);
        }
      } while (ros::ok() && !initialized_);
      ROS_WARN("%s:\n\ttf from %s to %s OK\n",
               ros::this_node::getName().c_str(), robot_frame_.c_str(),
               point_cloud_info->frame.c_str());

      point_clouds_info_.push_back(point_cloud_info);
    }
  }

  //=======================================================================
  // Octree
  //=======================================================================
  if (offline_octomap_path_.size() > 0)
    octree_ = new octomap::OcTree(offline_octomap_path_);
  else
    octree_ = new octomap::OcTree(octree_resol_);
  ROS_WARN("%s:\n\tLoaded\n", ros::this_node::getName().c_str());
  octree_->setProbHit(0.7);
  octree_->setProbMiss(0.4);
  octree_->setClampingThresMin(0.1192);
  octree_->setClampingThresMax(0.971);

  //=======================================================================
  // Gridmap
  //=======================================================================

  grid_map_.setFrameId(map_frame_);
  grid_map_.add("obstacles");
  grid_map_.add("agents");
  grid_map_.setGeometry(grid_map::Length(1, 1), octree_resol_);

  //=======================================================================
  // Publishers
  //=======================================================================
  octomap_marker_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>(
      "octomap_map", 2, true);
  grid_map_pub_ = local_nh_.advertise<grid_map_msgs::GridMap>("social_grid_map", 1, true);
  relevant_agents_pub_ = local_nh_.advertise<pedsim_msgs::AgentStates>("relevant_agents", 1, true);

  //=======================================================================
  // Subscribers
  //=======================================================================

  // Agent states callback
  agent_states_sub_ = nh_.subscribe(social_agents_topic_, 1,
                                    &Octomap::agentStatesCallback, this);

  // Odometry data (feedback)
  odom_sub_ =
      nh_.subscribe(odometry_topic_, 1, &Octomap::odomCallback, this);
  nav_sts_available_ = false;
  if (!nav_sts_available_)
    ROS_WARN("%s:\n\tWaiting for odometry\n",
             ros::this_node::getName().c_str());
  ros::Rate loop_rate(10);
  while (ros::ok() && !nav_sts_available_)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_WARN("%s:\n\tOdometry received\n", ros::this_node::getName().c_str());

  if (offline_octomap_path_.size() == 0)
  {

    for (std::vector<PointCloudExtended *>::iterator point_cloud_it =
             point_clouds_info_.begin();
         point_cloud_it != point_clouds_info_.end(); point_cloud_it++)
    {
      PointCloudExtended *point_cloud_info = *point_cloud_it;
      point_cloud_info->sub = nh_.subscribe(
          point_cloud_info->topic, 10, &Octomap::pointCloudCallback, this);
    }
  }

  //=======================================================================
  // Services
  //=======================================================================
  get_grid_map_srv_ = local_nh_.advertiseService(
      "get_grid_map", &Octomap::getGridMapSrv, this);

  // Timer for publishing
  if (rviz_timer_ > 0.0)
  {
    timer_ = nh_.createTimer(ros::Duration(rviz_timer_),
                             &Octomap::timerCallback, this);
  }
}

//! Destructor.
Octomap::~Octomap()
{
  ROS_INFO("%s:\n\tOctree has been deleted\n",
           ros::this_node::getName().c_str());
  delete octree_;
}

//! LaserScan callback.
/*!
 * Callback for receiving the laser scan data (taken from octomap_server)
 */
void Octomap::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
  //
  // ground filtering in base frame
  //
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  float minX = -0.6, minY = -12, minZ = -0.6;
  float maxX = 0.6, maxY = 12, maxZ = 0.6;

  if (social_agents_in_radius_.agent_states.size() > 0)
  {
    for (int i = 0; i < social_agents_in_radius_.agent_states.size(); i++)
    {

      tf::StampedTransform transform;

      tf_listener_.lookupTransform(cloud->header.frame_id, "agent_" + std::to_string(social_agents_in_radius_.agent_states[i].id),
                                   ros::Time(0), transform);

      // Z -> X
      // X -> Y
      // Y -> -Z
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0));
      boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 0));
      boxFilter.setInputCloud(pc.makeShared());
      boxFilter.setTranslation(Eigen::Vector3f(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()));
      boxFilter.setNegative(true);
      boxFilter.filter(pc);
    }
  }

  ros::Time t;
  std::string err = "cannot find transform from robot_frame to scan frame";

  tf::StampedTransform sensorToWorldTf;
  try
  {
    tf_listener_.getLatestCommonTime(fixed_frame_, cloud->header.frame_id, t,
                                     &err);
    //        tf_listener_.lookupTransform(robot_frame_,
    //        laser_scan_msg->header.frame_id, t,
    //                                     tf_robot_to_laser_scan);

    tf_listener_.lookupTransform(fixed_frame_, cloud->header.frame_id, t,
                                 sensorToWorldTf);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: "
                     << ex.what() << ", quitting callback");
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
  pass_z.setFilterLimits(0.1, 0.7); // TODO

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

  insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);
  //
  //    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  //    ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts
  //    (ground/nonground), %f sec)",
  //              pc_ground.size(), pc_nonground.size(), total_elapsed);
  //
  //    publishAll(cloud->header.stamp);

  // ! OCTOMAP PREPARATION

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;

  octree_->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  octree_->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

  grid_map::GridMapOctomapConverter::fromOctomap(*octree_, "obstacles", grid_map_, &min_bound, &max_bound);

  grid_map_["obstacles"] = 150 * grid_map_["obstacles"];

  // !SOCIAL AGENTS GRID MAP PREPARATION

  for (int i = 0; i < relevant_agent_states_.agent_states.size(); i++)
  {
    grid_map::Position center(relevant_agent_states_.agent_states[i].pose.position.x, relevant_agent_states_.agent_states[i].pose.position.y);
    double radius = social_agent_radius_;

    for (grid_map::CircleIterator iterator(grid_map_, center, radius);
         !iterator.isPastEnd(); ++iterator)
    {
      try
      {
        grid_map_.at("agents", *iterator) = 0.6;
      }
      catch (const std::out_of_range &oor)
      {
        ROS_ERROR("TRIED TO DEFINE AN AGENT OUT OF RANGE");
      }
    }
  }

  grid_map_["agents"] = 150 * grid_map_["agents"];
}

void Octomap::insertScan(const tf::Point &sensorOriginTf,
                         const PCLPointCloud &ground,
                         const PCLPointCloud &nonground)
{
  octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

  if (!octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMin) ||
      !octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
  }

#ifdef COLOR_OCTOMAP_SERVER
  unsigned char *colors = new unsigned char[3];
#endif

  // instead of direct scan insertion, compute update to filter ground:
  octomap::KeySet free_cells, occupied_cells;

  // all other points: free on ray, occupied on endpoint:
  double m_maxRange(5.0); // TODO
  for (PCLPointCloud::const_iterator it = nonground.begin();
       it != nonground.end(); ++it)
  {
    octomap::point3d point(it->x, it->y, it->z); // TODO
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange))
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
          ROS_ERROR_STREAM("Could not generate Key for endpoint " << new_end);
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
void Octomap::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
  if (!nav_sts_available_)
    nav_sts_available_ = true;

  robot_odometry_ = odom_msg;
}

//! Agent States callback.
/*!
 * Callback for getting updated agent states.
 */
void Octomap::agentStatesCallback(const pedsim_msgs::AgentStatesConstPtr &agent_states_msg)
{

  if (nav_sts_available_)
  {
    agent_states_ = agent_states_msg;

    std::vector<pedsim_msgs::AgentState> agent_state_vector;

    social_agents_in_radius_vector_.clear();

    for (int i = 0; i < agent_states_msg->agent_states.size(); i++)
    {
      if (this->isAgentInRFOV(agent_states_msg->agent_states[i]))
      {
        agent_state_vector.push_back(agent_states_msg->agent_states[i]);
      }
    }

    relevant_agent_states_.agent_states = agent_state_vector;

    social_agents_in_radius_.agent_states = social_agents_in_radius_vector_;

    relevant_agent_states_.header.stamp = ros::Time::now();
    relevant_agent_states_.header.frame_id = map_frame_;
  }

  relevant_agents_pub_.publish(relevant_agent_states_);
}

bool Octomap::isAgentInRFOV(const pedsim_msgs::AgentState agent_state)
{
  double d_robot_agent = std::sqrt(std::pow(agent_state.pose.position.x - robot_odometry_->pose.pose.position.x, 2) +
                                   std::pow(agent_state.pose.position.y - robot_odometry_->pose.pose.position.y, 2));

  double robot_velocity =
      std::sqrt(std::pow(robot_odometry_->twist.twist.linear.x, 2) + std::pow(robot_odometry_->twist.twist.linear.y, 2));

  double actual_fov_distance = robot_distance_view_ / robot_velocity_threshold_ * robot_velocity;

  if (actual_fov_distance < 1.5)
  {
    actual_fov_distance = 1.5;
  }

  if (d_robot_agent < 10)
  {
    social_agents_in_radius_vector_.push_back(agent_state);
  }

  if (!social_relevance_validity_checking_)
  {
    return true;
  }

  if (d_robot_agent > actual_fov_distance)
  {
    return false;
  }

  double tetha_robot_agent = atan2((agent_state.pose.position.y - robot_odometry_->pose.pose.position.y),
                                   (agent_state.pose.position.x - robot_odometry_->pose.pose.position.x));

  if (tetha_robot_agent < 0)
  {
    tetha_robot_agent = 2 * M_PI + tetha_robot_agent;
  }

  tf::Quaternion q(robot_odometry_->pose.pose.orientation.x, robot_odometry_->pose.pose.orientation.y,
                   robot_odometry_->pose.pose.orientation.z, robot_odometry_->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
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

//! Time callback.
/*!
 * Callback for publishing the map periodically using the Octomap RViz plugin.
 */
void Octomap::timerCallback(const ros::TimerEvent &e)
{
  // Declare message
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(*octree_, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = fixed_frame_;
  if (visualize_free_space_)
    publishMap();

  grid_map_.setTimestamp(ros::Time::now().toNSec());

  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(grid_map_, message);

  grid_map_pub_.publish(message);
}

//! Get binary service
/*!
 * Service for getting the binary Octomap
 */
bool Octomap::getGridMapSrv(grid_map_msgs::GetGridMap::Request &req,
                            grid_map_msgs::GetGridMap::Response &res)
{
  ROS_INFO("%s:\n\tSending grid map data on service\n",
           ros::this_node::getName().c_str());

  grid_map::GridMapRosConverter::toMessage(grid_map_, res.map);

  return true;
}

//! Publish the map
/*!
 * Service for saving the binary of the Octomap into the home folder
 */
void Octomap::publishMap()
{
  // Declare message and resize
  visualization_msgs::MarkerArray occupiedNodesVis;
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

      geometry_msgs::Point cubeCenter;
      cubeCenter.x = x;
      cubeCenter.y = y;
      cubeCenter.z = z;

      occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
    }

    if (!ros::ok())
      break;
  }
  // Finish Headers and options
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
  {
    double size = octree_->getNodeSize(i);

    occupiedNodesVis.markers[i].header.frame_id = fixed_frame_;
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = fixed_frame_;
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color.r = 0.5;
    occupiedNodesVis.markers[i].color.g = 0.5;
    occupiedNodesVis.markers[i].color.b = 0.5;
    occupiedNodesVis.markers[i].color.a = 1.0;

    if (occupiedNodesVis.markers[i].points.size() > 0)
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;

    if (!ros::ok())
      break;
  }

  // Publish it
  octomap_marker_pub_.publish(occupiedNodesVis);
}

//! Main function
int main(int argc, char **argv)
{
  //=======================================================================
  // Override SIGINT handler
  //=======================================================================
  signal(SIGINT, stopNode);

  // Init ROS node
  ros::init(argc, argv, "octomap_laser_scan");
  ros::NodeHandle private_nh("~");

  // Constructor
  Octomap mapper;

  // Spin
  ros::spin();

  // Exit main function without errors
  return 0;
}
