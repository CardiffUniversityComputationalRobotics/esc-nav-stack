/*! \file octomap_laser_scan.cpp
 * \brief Merge data from different laser_scan messages to incrementally build
 * an Octomap.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
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
//#include <octomap_msgs/OctomapBinary.h>
#include <octomap_msgs/GetOctomap.h>
typedef octomap_msgs::GetOctomap OctomapSrv;
//#include <autopilot_laser_octomap/BoundingBoxQuery.h>
// typedef autopilot_laser_octomap::BoundingBoxQuery BBXSrv;
#include <octomap/Pointcloud.h>
#include <octomap_ros/conversions.h>

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

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef octomap::OcTree OcTreeT;

#include <signal.h>

void stopNode(int sig)
{
  ros::shutdown();
  exit(0);
}

struct LaserScanExtended
{
  std::string topic;
  std::string frame;
  tf::StampedTransform tf_robot_to_laser_scan;
  ros::Subscriber sub;
};

struct PointCloudExtended
{
  std::string topic;
  std::string frame;
  tf::StampedTransform tf_robot_to_point_cloud;
  ros::Subscriber sub;
};

//!  LaserOctomap class.
/*!
 * Autopilot Laser Octomap.
 * Create an Octomap using information from laser scans.
 */
class LaserOctomap
{
public:
  //! Constructor
  LaserOctomap();
  //! Destructor
  virtual ~LaserOctomap();
  //! Callback for getting the laser_scan data
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr &laser_scan_msg);
  //! Callback for getting the point_cloud data
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
  //! Callback for getting current vehicle odometry
  void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
  //! Callback for getting global map
  void globalMapCallback(const nav_msgs::OccupancyGridPtr &map_msg);
  //! Periodic callback to publish the map for visualization.
  void timerCallback(const ros::TimerEvent &e);

  void mergeGlobalMapToOctomap();
  void insertScan(const tf::Point &sensorOriginTf, const PCLPointCloud &ground,
                  const PCLPointCloud &nonground);

  void filterGroundPlane(const PCLPointCloud &pc, PCLPointCloud &ground,
                         PCLPointCloud &nonground) const;

  //! Service to save a binary Octomap (.bt)
  bool saveBinaryOctomapSrv(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res);
  //! Service to save a full Octomap (.ot)
  bool saveFullOctomapSrv(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res);
  //! Service to get a binary Octomap
  bool getBinaryOctomapSrv(OctomapSrv::Request &req,
                           OctomapSrv::GetOctomap::Response &res);

  //! Service to save a binary Octomap (.bt)
  bool mergeGlobalMapToOctomapSrv(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

  //! Publish the Octomap
  void publishMap();

private:
  //! Filter outliers
  void filterSingleOutliers(sensor_msgs::LaserScan &laser_scan_msg,
                            std::vector<bool> &rngflags);

  // ROS
  ros::NodeHandle nh_, local_nh_;
  ros::Publisher octomap_marker_pub_, octomap_plugin_pub_;
  ros::Subscriber odom_sub_, laser_scan_sub_, mission_flag_sub_;
  ros::ServiceServer save_binary_octomap_srv_, save_full_octomap_srv_,
      get_binary_octomap_srv_, merge_global_map_to_octomap_srv_;
  ros::Timer timer_;

  // ROS tf
  tf::StampedTransform tf_robot_to_laser_scan_;
  tf::TransformListener tf_listener_;

  // Names
  std::string map_frame_, fixed_frame_, robot_frame_, offline_octomap_path_,
      laser_scan_topic_, odometry_topic_, global_map_topic_;

  // LaserScan => (x,y,z)
  laser_geometry::LaserProjection laser_scan_projector_;

  // Laser scans
  std::vector<std::string> laser_scan_frames_, laser_scan_topics_;
  std::vector<LaserScanExtended *> laser_scans_info_;
  // Point Clouds
  std::vector<std::string> point_cloud_topics_, point_cloud_frames_;
  std::vector<PointCloudExtended *> point_clouds_info_;

  // ROS Messages
  sensor_msgs::PointCloud cloud_;

  // Octree
  octomap::OcTree *octree_;
  double octree_resol_, minimum_range_, rviz_timer_;
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;
  octomap::KeyRay m_keyRay; // temp storage for ray casting

  // Flags
  tf::Vector3 prev_map_to_fixed_pos_;
  double orientation_drift_, prev_map_to_fixed_yaw_, position_drift_;
  bool add_max_range_measures_;
  bool apply_filter_;
  bool initialized_;
  bool nav_sts_available_;
  bool visualize_free_space_;
  bool projection_2d_;
  bool add_rays_;
  bool global_map_available_;

  // Global map
  nav_msgs::OccupancyGrid global_map_;

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
LaserOctomap::LaserOctomap()
    : nh_(),
      local_nh_("~"),
      fixed_frame_("/fixed_frame"),
      robot_frame_("/robot_frame"),
      laser_scan_topic_("/laser_scan_topic"),
      odometry_topic_("/odometry_topic"),
      offline_octomap_path_(""),
      global_map_topic_("/map"),
      octree_(NULL),
      octree_resol_(1.0),
      add_max_range_measures_(false),
      apply_filter_(false),
      initialized_(false),
      visualize_free_space_(false),
      projection_2d_(false),
      add_rays_(false),
      minimum_range_(-1.0),
      rviz_timer_(0.0),
      orientation_drift_(0.0),
      prev_map_to_fixed_yaw_(0.0),
      position_drift_(0.0),
      global_map_available_(false)
{
  //=======================================================================
  // Get parameters
  //=======================================================================
  local_nh_.param("add_rays", add_rays_, add_rays_);
  local_nh_.param("resolution", octree_resol_, octree_resol_);
  local_nh_.param("apply_filter", apply_filter_, apply_filter_);
  local_nh_.param("add_max_ranges", add_max_range_measures_,
                  add_max_range_measures_);
  local_nh_.param("map_frame", map_frame_, map_frame_);
  local_nh_.param("fixed_frame", fixed_frame_, fixed_frame_);
  local_nh_.param("robot_frame", robot_frame_, robot_frame_);
  local_nh_.param("offline_octomap_path", offline_octomap_path_,
                  offline_octomap_path_);
  local_nh_.param("visualize_free_space", visualize_free_space_,
                  visualize_free_space_);
  local_nh_.param("projection_2d", projection_2d_, projection_2d_);
  local_nh_.param("laser_scan_topic", laser_scan_topic_, laser_scan_topic_);
  local_nh_.param("odometry_topic", odometry_topic_, odometry_topic_);
  local_nh_.param("global_map_topic", global_map_topic_, global_map_topic_);
  local_nh_.param("minimum_range", minimum_range_, minimum_range_);
  local_nh_.param("rviz_timer", rviz_timer_, rviz_timer_);

  local_nh_.param("laser_scan_topics", laser_scan_topics_, laser_scan_topics_);
  local_nh_.param("laser_scan_frames", laser_scan_frames_, laser_scan_frames_);

  local_nh_.param("point_cloud_topics", point_cloud_topics_,
                  point_cloud_topics_);
  local_nh_.param("point_cloud_frames", point_cloud_frames_,
                  point_cloud_frames_);

  // Transforms TF and catch the static transform from vehicle to laser_scan
  // sensor
  // tf_listener_.setExtrapolationLimit(ros::Duration(0.2));
  int count(0);
  ros::Time t;
  std::string err = "";

  if (laser_scan_frames_.size() == laser_scan_topics_.size())
  {
    for (unsigned int i = 0; i < laser_scan_frames_.size(); i++)
    {
      LaserScanExtended *laser_scan_info = new LaserScanExtended();
      laser_scan_info->frame = laser_scan_frames_[i];
      laser_scan_info->topic = laser_scan_topics_[i];

      // Get the corresponding tf
      count = 0;
      err = "cannot find tf from " + robot_frame_ + "to " +
            laser_scan_info->frame;

      tf_listener_.getLatestCommonTime(robot_frame_, laser_scan_info->frame, t,
                                       &err);

      initialized_ = false;
      do
      {
        try
        {
          tf_listener_.lookupTransform(robot_frame_, laser_scan_info->frame, t,
                                       laser_scan_info->tf_robot_to_laser_scan);
          initialized_ = true;
        }
        catch (std::exception e)
        {
          tf_listener_.waitForTransform(robot_frame_, laser_scan_info->frame,
                                        ros::Time::now(), ros::Duration(1.0));
          tf_listener_.getLatestCommonTime(robot_frame_, laser_scan_info->frame,
                                           t, &err);
          count++;
          ROS_WARN("%s:\n\tCannot find tf from %s to %s\n",
                   ros::this_node::getName().c_str(), robot_frame_.c_str(),
                   laser_scan_info->frame.c_str());
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
               laser_scan_info->frame.c_str());

      laser_scans_info_.push_back(laser_scan_info);
    }
  }

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
  // Publishers
  //=======================================================================
  octomap_marker_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>(
      "octomap_map", 2, true);
  octomap_plugin_pub_ =
      local_nh_.advertise<octomap_msgs::Octomap>("octomap_map_plugin", 2, true);

  //=======================================================================
  // Subscribers
  //=======================================================================
  // Odometry data (feedback)
  odom_sub_ =
      nh_.subscribe(odometry_topic_, 1, &LaserOctomap::odomCallback, this);
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
    for (std::vector<LaserScanExtended *>::iterator laser_scan_it =
             laser_scans_info_.begin();
         laser_scan_it != laser_scans_info_.end(); laser_scan_it++)
    {
      LaserScanExtended *laser_scan_info = *laser_scan_it;
      laser_scan_info->sub = nh_.subscribe(
          laser_scan_info->topic, 10, &LaserOctomap::laserScanCallback, this);
    }

    for (std::vector<PointCloudExtended *>::iterator point_cloud_it =
             point_clouds_info_.begin();
         point_cloud_it != point_clouds_info_.end(); point_cloud_it++)
    {
      PointCloudExtended *point_cloud_info = *point_cloud_it;
      point_cloud_info->sub = nh_.subscribe(
          point_cloud_info->topic, 10, &LaserOctomap::pointCloudCallback, this);
    }
  }

  // Global map
  odom_sub_ = nh_.subscribe(global_map_topic_, 1,
                            &LaserOctomap::globalMapCallback, this);
  global_map_available_ = false;
  if (!global_map_available_)
    ROS_WARN("%s:\n\tWaiting for global map\n",
             ros::this_node::getName().c_str());
  while (ros::ok() && !global_map_available_)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_WARN("%s:\n\tGlobal map received\n", ros::this_node::getName().c_str());

  mergeGlobalMapToOctomap();

  //=======================================================================
  // Services
  //=======================================================================
  save_binary_octomap_srv_ = local_nh_.advertiseService(
      "save_binary", &LaserOctomap::saveBinaryOctomapSrv, this);
  save_full_octomap_srv_ = local_nh_.advertiseService(
      "save_full", &LaserOctomap::saveFullOctomapSrv, this);
  get_binary_octomap_srv_ = local_nh_.advertiseService(
      "get_binary", &LaserOctomap::getBinaryOctomapSrv, this);
  merge_global_map_to_octomap_srv_ = local_nh_.advertiseService(
      "clean_merge_octomap", &LaserOctomap::mergeGlobalMapToOctomapSrv, this);

  // Timer for publishing
  if (rviz_timer_ > 0.0)
    timer_ = nh_.createTimer(ros::Duration(rviz_timer_),
                             &LaserOctomap::timerCallback, this);

  // Info
  ROS_INFO(
      "%s:\n\tAutopilot Laser Octomap node initialized_.\n\tResolution = "
      "%f\n\tmax_ranges = %d \n",
      ros::this_node::getName().c_str(), octree_resol_,
      add_max_range_measures_);
  for (std::vector<LaserScanExtended *>::iterator laser_scan_it =
           laser_scans_info_.begin();
       laser_scan_it != laser_scans_info_.end(); laser_scan_it++)
  {
    LaserScanExtended *laser_scan_info = *laser_scan_it;
    ROS_INFO(
        "%s:\n\tFixed frame = %s\n\tRobot frame = %s\n\tlaser_scan frame = "
        "%s\n",
        ros::this_node::getName().c_str(), fixed_frame_.c_str(),
        robot_frame_.c_str(), laser_scan_info->frame.c_str());
  }
}

//! Destructor.
LaserOctomap::~LaserOctomap()
{
  ROS_INFO("%s:\n\tOctree has been deleted\n",
           ros::this_node::getName().c_str());
  delete octree_;
}

void LaserOctomap::mergeGlobalMapToOctomap()
{
  ros::Time t;
  std::string err = "";
  tf::StampedTransform tf_map_to_fixed;
  tf_listener_.getLatestCommonTime("map", "odom", t, &err);
  tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);
  tf::Point occupied_point;
  occupied_point.setZ(0.2); // TODO
  octree_->clear();

  for (int x = 0; x < global_map_.info.height; x++)
    for (int y = 0; y < global_map_.info.width; y++)
    {
      if (global_map_.data[x + global_map_.info.height * y] > 0)
      {
        occupied_point.setX((x - 2000) * global_map_.info.resolution);
        occupied_point.setY((y - 2000) * global_map_.info.resolution);

        occupied_point = tf_map_to_fixed.inverse() * occupied_point;

        octree_->updateNode(occupied_point.getX(), occupied_point.getY(), 0.25,
                            true); // integrate 'occupied' measurement
      }
    }
}

void LaserOctomap::globalMapCallback(
    const nav_msgs::OccupancyGridPtr &map_msg)
{
  global_map_.header = map_msg->header;
  global_map_.info = map_msg->info;
  global_map_.data.clear();

  for (auto map_data : map_msg->data)
    global_map_.data.push_back(map_data);

  global_map_available_ = true;
}

//! LaserScan callback.
/*!
 * Callback for receiving the laser scan data (taken from octomap_server)
 */
void LaserOctomap::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
  //
  // ground filtering in base frame
  //
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);
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
  //    pcl::PassThrough<PCLPoint> pass_x;
  //    pass_x.setFilterFieldName("x");
  //    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
  //    pcl::PassThrough<PCLPoint> pass_y;
  //    pass_y.setFilterFieldName("y");
  //    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.3, 0.7); // TODO

  PCLPointCloud pc_ground;    // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  bool m_filterGroundPlane(false); // TODO
  if (m_filterGroundPlane)
  {
    tf::StampedTransform sensorToBaseTf, baseToWorldTf;
    try
    {
      tf_listener_.getLatestCommonTime("base_link", cloud->header.frame_id, t,
                                       &err);

      //            tf_listener_.waitForTransform("base_link",
      //            cloud->header.frame_id,
      //            cloud->header.stamp,
      //                                          ros::Duration(0.2));
      tf_listener_.lookupTransform("base_link", cloud->header.frame_id, t,
                                   sensorToBaseTf);

      tf_listener_.getLatestCommonTime(fixed_frame_, "base_link", t, &err);
      tf_listener_.lookupTransform(fixed_frame_, "base_link", t, baseToWorldTf);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR_STREAM("Transform error for ground plane filter: "
                       << ex.what() << ", quitting callback.\n"
                                       "You need to set the "
                                       "base_frame_id or disable "
                                       "filter_ground.");
    }

    Eigen::Matrix4f sensorToBase, baseToWorld;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToBase);
    //        pass_x.setInputCloud(pc.makeShared());
    //        pass_x.filter(pc);
    //        pass_y.setInputCloud(pc.makeShared());
    //        pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);
    filterGroundPlane(pc, pc_ground, pc_nonground);

    //        // transform clouds to world frame for insertion
    //        pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
    //        pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
  }
  else
  {
    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    // just filter height range:
    //        pass_x.setInputCloud(pc.makeShared());
    //        pass_x.filter(pc);
    //        pass_y.setInputCloud(pc.makeShared());
    //        pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }

  insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);
  //
  //    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  //    ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts
  //    (ground/nonground), %f sec)",
  //              pc_ground.size(), pc_nonground.size(), total_elapsed);
  //
  //    publishAll(cloud->header.stamp);
}

void LaserOctomap::insertScan(const tf::Point &sensorOriginTf,
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
  // insert ground points only as free:
  //    for (PCLPointCloud::const_iterator it = ground.begin(); it !=
  //    ground.end(); ++it)
  //    {
  //        point3d point(it->x, it->y, it->z);
  //        // maxrange check
  //        if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() >
  //        m_maxRange))
  //        {
  //            point = sensorOrigin + (point - sensorOrigin).normalized() *
  //            m_maxRange;
  //        }
  //
  //        // only clear space (ground points)
  //        if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
  //        {
  //            free_cells.insert(m_keyRay.begin(), m_keyRay.end());
  //        }
  //
  //        octomap::OcTreeKey endKey;
  //        if (m_octree->coordToKeyChecked(point, endKey))
  //        {
  //            updateMinKey(endKey, m_updateBBXMin);
  //            updateMaxKey(endKey, m_updateBBXMax);
  //        }
  //        else
  //        {
  //            ROS_ERROR_STREAM("Could not generate Key for endpoint " <<
  //            point);
  //        }
  //    }
  //
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

#ifdef COLOR_OCTOMAP_SERVER // NB: Only read and interpret color if it's an
                            // occupied node
        const int rgb =
            *reinterpret_cast<const int *>(&(it->rgb)); // TODO: there are
        other ways to
            // encode color
            than this one colors[0] = ((rgb >> 16) & 0xff);
        colors[1] = ((rgb >> 8) & 0xff);
        colors[2] = (rgb & 0xff);
        m_octree->averageNodeColor(it->x, it->y, it->z, colors[0], colors[1],
                                   colors[2]);
#endif
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
  //
  //    // TODO: eval lazy+updateInner vs. proper insertion
  //    // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  //    // m_octree->updateInnerOccupancy();
  //    octomap::point3d minPt, maxPt;
  //    ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] <<
  //    " " << m_updateBBXMin[1]
  //    << " "
  //                                                    << m_updateBBXMin[2] <<
  //                                                    " / " <<
  //                                                    m_updateBBXMax[0]
  //                                                    << " "
  //                                                    << m_updateBBXMax[1] <<
  //                                                    " " <<
  //                                                    m_updateBBXMax[2]);
  //
  //    // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
  //    //   if (m_maxTreeDepth < 16)
  //    //   {
  //    //      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth);
  //    // this should give us
  //    the first
  //    //      key at depth m_maxTreeDepth that is smaller or equal to
  //    m_updateBBXMin (i.e. lower left
  //    in 2D grid
  //    //      coordinates) OcTreeKey tmpMax = getIndexKey(m_updateBBXMax,
  //    m_maxTreeDepth); // see
  //    above, now add
  //    //      something to find upper right tmpMax[0]+= m_octree->getNodeSize(
  //    m_maxTreeDepth ) - 1;
  //    tmpMax[1]+=
  //    //      m_octree->getNodeSize( m_maxTreeDepth ) - 1; tmpMax[2]+=
  //    m_octree->getNodeSize(
  //    m_maxTreeDepth ) -
  //    //      1; m_updateBBXMin = tmpMin; m_updateBBXMax = tmpMax;
  //    //   }
  //
  //    // TODO: we could also limit the bbx to be within the map bounds here
  //    (see publishing check)
  //    minPt = m_octree->keyToCoord(m_updateBBXMin);
  //    maxPt = m_octree->keyToCoord(m_updateBBXMax);
  //    ROS_DEBUG_STREAM("Updated area bounding box: " << minPt << " - " <<
  //    maxPt);
  //    ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << "
  //    " << m_updateBBXMin[1]
  //    << "
  //    "
  //                                                   << m_updateBBXMin[2] << "
  //                                                   / " <<
  //                                                   m_updateBBXMax[0] << " "
  //                                                   << m_updateBBXMax[1] << "
  //                                                   " <<
  //                                                   m_updateBBXMax[2]);
  //
  //    if (m_compressMap)
  //        m_octree->prune();
  //
  //#ifdef COLOR_OCTOMAP_SERVER
  //    if (colors)
  //    {
  //        delete[] colors;
  //        colors = NULL;
  //    }
  //#endif
}

void LaserOctomap::filterGroundPlane(const PCLPointCloud &pc,
                                     PCLPointCloud &ground,
                                     PCLPointCloud &nonground) const
{
  ground.header = pc.header;
  nonground.header = pc.header;
  //
  //    if (pc.size() < 50)
  //    {
  //        ROS_WARN("Pointcloud in OctomapServer too small, skipping ground
  //        plane extraction");
  //        nonground = pc;
  //    }
  //    else
  //    {
  //        // plane detection for ground plane removal:
  //        pcl::ModelCoefficients::Ptr coefficients(new
  //        pcl::ModelCoefficients);
  //        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  //
  //        // Create the segmentation object and set up:
  //        pcl::SACSegmentation<PCLPoint> seg;
  //        seg.setOptimizeCoefficients(true);
  //        // TODO: maybe a filtering based on the surface normals might be
  //        more robust / accurate?
  //        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  //        seg.setMethodType(pcl::SAC_RANSAC);
  //        seg.setMaxIterations(200);
  //        seg.setDistanceThreshold(m_groundFilterDistance);
  //        seg.setAxis(Eigen::Vector3f(0, 0, 1));
  //        seg.setEpsAngle(m_groundFilterAngle);
  //
  //        PCLPointCloud cloud_filtered(pc);
  //        // Create the filtering object
  //        pcl::ExtractIndices<PCLPoint> extract;
  //        bool groundPlaneFound = false;
  //
  //        while (cloud_filtered.size() > 10 && !groundPlaneFound)
  //        {
  //            seg.setInputCloud(cloud_filtered.makeShared());
  //            seg.segment(*inliers, *coefficients);
  //            if (inliers->indices.size() == 0)
  //            {
  //                ROS_INFO("PCL segmentation did not find any plane.");
  //
  //                break;
  //            }
  //
  //            extract.setInputCloud(cloud_filtered.makeShared());
  //            extract.setIndices(inliers);
  //
  //            if (std::abs(coefficients->values.at(3)) <
  //            m_groundFilterPlaneDistance)
  //            {
  //                ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f
  //                %f %f",
  //                inliers->indices.size(),
  //                          cloud_filtered.size(), coefficients->values.at(0),
  //                          coefficients->values.at(1),
  //                          coefficients->values.at(2),
  //                          coefficients->values.at(3));
  //                extract.setNegative(false);
  //                extract.filter(ground);
  //
  //                // remove ground points from full pointcloud:
  //                // workaround for PCL bug:
  //                if (inliers->indices.size() != cloud_filtered.size())
  //                {
  //                    extract.setNegative(true);
  //                    PCLPointCloud cloud_out;
  //                    extract.filter(cloud_out);
  //                    nonground += cloud_out;
  //                    cloud_filtered = cloud_out;
  //                }
  //
  //                groundPlaneFound = true;
  //            }
  //            else
  //            {
  //                ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu
  //                inliers. Coeff: %f %f %f
  //                %f",
  //                          inliers->indices.size(), cloud_filtered.size(),
  //                          coefficients->values.at(0),
  //                          coefficients->values.at(1),
  //                          coefficients->values.at(2),
  //                          coefficients->values.at(3));
  //                pcl::PointCloud<PCLPoint> cloud_out;
  //                extract.setNegative(false);
  //                extract.filter(cloud_out);
  //                nonground += cloud_out;
  //                // debug
  //                //            pcl::PCDWriter writer;
  //                //
  //                writer.write<PCLPoint>("nonground_plane.pcd",cloud_out,
  //                false);
  //
  //                // remove current plane from scan for next iteration:
  //                // workaround for PCL bug:
  //                if (inliers->indices.size() != cloud_filtered.size())
  //                {
  //                    extract.setNegative(true);
  //                    cloud_out.points.clear();
  //                    extract.filter(cloud_out);
  //                    cloud_filtered = cloud_out;
  //                }
  //                else
  //                {
  //                    cloud_filtered.points.clear();
  //                }
  //            }
  //        }
  //        // TODO: also do this if overall starting pointcloud too small?
  //        if (!groundPlaneFound)
  //        {  // no plane found or remaining points too small
  //            ROS_WARN("No ground plane found in scan");
  //
  //            // do a rough fitlering on height to prevent spurious obstacles
  //            pcl::PassThrough<PCLPoint> second_pass;
  //            second_pass.setFilterFieldName("z");
  //            second_pass.setFilterLimits(-m_groundFilterPlaneDistance,
  //            m_groundFilterPlaneDistance);
  //            second_pass.setInputCloud(pc.makeShared());
  //            second_pass.filter(ground);
  //
  //            second_pass.setFilterLimitsNegative(true);
  //            second_pass.filter(nonground);
  //        }
  //
  //        // debug:
  //        //        pcl::PCDWriter writer;
  //        //        if (pc_ground.size() > 0)
  //        //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
  //        //        if (pc_nonground.size() > 0)
  //        //          writer.write<PCLPoint>("nonground.pcd",pc_nonground,
  //        false);
  //    }
}

//! LaserScan callback.
/*!
 * Callback for receiving the laser scan data
 */
void LaserOctomap::laserScanCallback(
    const sensor_msgs::LaserScanConstPtr &laser_scan_msg)
{
  ros::Time t;
  tf::StampedTransform tf_robot_to_laser_scan, tf_fixed_to_robot,
      tf_map_to_fixed;
  std::string err = "cannot find transform from robot_frame to scan frame";

  // check drift
  tf_listener_.getLatestCommonTime(map_frame_, fixed_frame_, t, &err);
  tf_listener_.lookupTransform(map_frame_, fixed_frame_, t, tf_map_to_fixed);
  tf::Matrix3x3 m_map_to_fixed = tf_map_to_fixed.getBasis();
  tf::Vector3 p_map_to_fixed = tf_map_to_fixed.getOrigin();
  double roll, pitch, yaw;
  m_map_to_fixed.getRPY(roll, pitch, yaw);
  orientation_drift_ += abs(prev_map_to_fixed_yaw_ - yaw);
  prev_map_to_fixed_yaw_ = yaw;
  position_drift_ +=
      sqrt(pow(prev_map_to_fixed_pos_.getX() - p_map_to_fixed.getX(), 2.0) +
           pow(prev_map_to_fixed_pos_.getY() - p_map_to_fixed.getY(), 2.0));
  prev_map_to_fixed_pos_ = p_map_to_fixed;

  if (orientation_drift_ > 0.05 || position_drift_ > 0.05)
  {
    orientation_drift_ = 0.0;
    position_drift_ = 0.0;
    octree_->clear();
    mergeGlobalMapToOctomap();
  }

  tf_listener_.getLatestCommonTime(robot_frame_,
                                   laser_scan_msg->header.frame_id, t, &err);
  tf_listener_.lookupTransform(robot_frame_, laser_scan_msg->header.frame_id, t,
                               tf_robot_to_laser_scan);

  tf_listener_.getLatestCommonTime(fixed_frame_, robot_frame_, t, &err);
  tf_listener_.lookupTransform(fixed_frame_, robot_frame_, t,
                               tf_fixed_to_robot);

  // Editable message
  sensor_msgs::LaserScan laser_scan = *laser_scan_msg;

  // Max range flags
  std::vector<bool> rngflags;
  rngflags.resize(laser_scan.ranges.size());
  for (int i = 0; i < laser_scan.ranges.size(); i++)
  {
    rngflags[i] = false;
    if (!ros::ok())
      break;
  }

  // Use zero ranges as max ranges
  double new_max = laser_scan.range_max * 0.95;
  if (add_max_range_measures_)
  {
    // Flag readings as maxrange
    for (int i = 0; i < laser_scan.ranges.size(); i++)
    {
      if (laser_scan.ranges[i] == 0.0 ||
          laser_scan.ranges[i] >= laser_scan.range_max)
      {
        laser_scan.ranges[i] = new_max;
        rngflags[i] = true;
      }
      if (!ros::ok())
        break;
    }
  }

  // Apply single outliers removal filter
  if (apply_filter_)
  {
    // Copy message for editing, filter and project to (x,y,z)
    sensor_msgs::LaserScan laser_scan = *laser_scan_msg;
    filterSingleOutliers(laser_scan, rngflags);
  }

  // Project to (x,y,z)
  laser_scan_projector_.projectLaser(laser_scan, cloud_, laser_scan.range_max);

  // Compute origin of sensor in world frame (filters laser_scan_msg->ranges[i]
  // > 0.0)
  tf::Vector3 orig(0, 0, 0);
  orig = tf_fixed_to_robot * tf_robot_to_laser_scan * orig;
  octomap::point3d origin(orig.getX(), orig.getY(), orig.getZ());
  // std::cout << "origin: " << orig.getX() << ", " << orig.getY() << "," <<
  // orig.getZ() << std::endl;

  // Trick for max ranges not occupied
  // new_max *= 0.95;
  for (unsigned i = 0; i < cloud_.points.size(); i++)
  {
    if ((i % 1) == 0)
    {
      // Transform readings
      tf::Vector3 scanpt(cloud_.points[i].x, cloud_.points[i].y,
                         cloud_.points[i].z);
      scanpt = tf_fixed_to_robot * tf_robot_to_laser_scan * scanpt;
      octomap::point3d end;
      end = octomap::point3d(scanpt.getX(), scanpt.getY(), scanpt.getZ());

      // Insert readings
      // if(scanpt.getZ()>0.0)
      double point_distance =
          sqrt(pow(end.z() - origin.z(), 2.0) + pow(end.y() - origin.y(), 2.0) +
               pow(end.x() - origin.x(), 2.0));
      if (point_distance > minimum_range_)
      {
        if (add_rays_)
          octree_->insertRay(origin, end,
                             new_max); // integrate 'occupied' measurement
        else
          octree_->updateNode(scanpt.getX(), scanpt.getY(), scanpt.getZ(),
                              true); // integrate 'occupied' measurement
      }
    }

    if (!ros::ok())
      break;
  }
}

//! Odometry callback.
/*!
 * Callback for getting updated vehicle odometry.
 */
void LaserOctomap::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
  if (!nav_sts_available_)
    nav_sts_available_ = true;
}

//! Time callback.
/*!
 * Callback for publishing the map periodically using the Octomap RViz plugin.
 */
void LaserOctomap::timerCallback(const ros::TimerEvent &e)
{
  // Declare message
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(*octree_, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = fixed_frame_;
  octomap_plugin_pub_.publish(msg);
  if (visualize_free_space_)
    publishMap();
}

//! Save binary service
/*!
 * Service for saving the binary Octomap into the home folder
 */
bool LaserOctomap::saveBinaryOctomapSrv(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  // Saves current octree_ in home folder
  std::string fpath(getenv("HOME"));
  octree_->writeBinary(fpath + "/map_laser_octomap.bt");
  return true;
}

//! MergeGlobalMap
/*!
 * Service for cleaning and merging the octomap
 */
bool LaserOctomap::mergeGlobalMapToOctomapSrv(std_srvs::Empty::Request &req,
                                              std_srvs::Empty::Response &res)
{
  this->mergeGlobalMapToOctomap();
  return true;
}

//! Save binary service
/*!
 * Service for saving the full Octomap into the home folder
 */
bool LaserOctomap::saveFullOctomapSrv(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res)
{
  // Saves current octree_ in home folder (full probabilities)
  std::string fpath(getenv("HOME"));
  octree_->write(fpath + "/map_laser_octomap.ot");
  return true;
}

//! Get binary service
/*!
 * Service for getting the binary Octomap
 */
bool LaserOctomap::getBinaryOctomapSrv(OctomapSrv::Request &req,
                                       OctomapSrv::GetOctomap::Response &res)
{
  ROS_INFO("%s:\n\tSending binary map data on service request\n",
           ros::this_node::getName().c_str());
  res.map.header.frame_id = fixed_frame_;
  res.map.header.stamp = ros::Time::now();

  if (!octomap_msgs::binaryMapToMsg(*octree_, res.map))
    return false;

  return true;
}

//! Publish the map
/*!
 * Service for saving the binary of the Octomap into the home folder
 */
void LaserOctomap::publishMap()
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

//! Filter outliers
void LaserOctomap::filterSingleOutliers(sensor_msgs::LaserScan &laser_scan_msg,
                                        std::vector<bool> &rngflags)
{
  int n(laser_scan_msg.ranges.size());
  double thres(laser_scan_msg.range_max / 10.0);
  std::vector<int> zeros;
  for (int i = 1; i < n - 1; i++)
  {
    if ((std::abs(laser_scan_msg.ranges[i - 1] - laser_scan_msg.ranges[i]) >
         thres) &&
        (std::abs(laser_scan_msg.ranges[i + 1] - laser_scan_msg.ranges[i]) >
         thres))
    {
      laser_scan_msg.ranges[i] = 0.0;
      zeros.push_back(i);
    }

    if (!ros::ok())
      break;
  }

  // Delete unnecessary flags
  for (std::vector<int>::reverse_iterator rit = zeros.rbegin();
       rit < zeros.rend(); rit++)
  {
    rngflags.erase(rngflags.begin() + *rit);

    if (!ros::ok())
      break;
  }
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
  LaserOctomap mapper;

  // Spin
  ros::spin();

  // Exit main function without errors
  return 0;
}
