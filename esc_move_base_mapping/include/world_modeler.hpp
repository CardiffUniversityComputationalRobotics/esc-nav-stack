#ifndef WORLD_MODELER_HPP
#define WORLD_MODELER_HPP

// SOCIAL HEATMAP
#include "social_heatmap.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>

// ROS2 messages
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int8.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ROS2 services
#include <std_srvs/srv/empty.hpp>

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include "message_filters/subscriber.h"
#include <tf2_ros/create_timer_ros.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/srv/get_octomap.hpp>
typedef octomap_msgs::srv::GetOctomap OctomapSrv;
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>

// grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>

// PEDSIM
#include <pedsim_msgs/msg/agent_states.hpp>

#include <signal.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef octomap::OcTree OcTreeT;

void stopNode(int sig)
{
    rclcpp::shutdown();
    exit(0);
}

//!  WorldModeler class.
/*!
 * Autopilot Laser WorldModeler.
 * Create a WorldModeler using information from laser scans.
 */
class WorldModeler : public rclcpp::Node
{
public:
    //! Constructor
    WorldModeler();
    //! Destructor
    virtual ~WorldModeler();
    //! Callback for getting the point_cloud data
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    //! Callback for getting current agent states
    void agentStatesCallback(const pedsim_msgs::msg::AgentStates::SharedPtr agent_states_msg);
    bool isAgentInRFOV(const pedsim_msgs::msg::AgentState agent_state);
    //! Check if robot is in front of agent.
    bool isRobotInFront(pedsim_msgs::msg::AgentState agent_state, grid_map::Position position);
    //! Calculate comfort in specific position in gridmap
    double getExtendedPersonalSpace(pedsim_msgs::msg::AgentState agent_state, grid_map::Position position);
    //! Periodic callback to publish the map for visualization.
    void timerCallback();
    void insertScan(const geometry_msgs::msg::Vector3 &sensorOriginTf, const PCLPointCloud &ground,
                    const PCLPointCloud &nonground);
    //! Service to save a binary Octomap (.bt)
    bool saveBinaryOctomapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                              std::shared_ptr<std_srvs::srv::Empty::Response> res);
    //! Service to save a full Octomap (.ot)
    bool saveFullOctomapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                            std::shared_ptr<std_srvs::srv::Empty::Response> res);
    //! Service to get a binary Octomap
    bool getBinaryOctomapSrv(const std::shared_ptr<OctomapSrv::Request> req,
                             std::shared_ptr<OctomapSrv::Response> res);
    //! Service to get grid map
    bool getGridMapSrv(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> req,
                       std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> res);

    //! Publish the WorldModeler
    void publishMap();
    //! Redefine the gridmap with octomap
    void defineSocialGridMap();

private:
    // ROS2
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr octomap_marker_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<pedsim_msgs::msg::AgentStates>::SharedPtr relevant_agents_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_states_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> point_cloud_sub_;

    // SERVICES
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_binary_octomap_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_full_octomap_srv_;
    rclcpp::Service<OctomapSrv>::SharedPtr get_binary_octomap_srv_;
    rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr get_grid_map_srv_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> point_cloud_mn_;

    // tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Names
    std::string map_frame_, fixed_frame_, robot_frame_, offline_octomap_path_,
        odometry_topic_, social_agents_topic_;

    // Point Clouds
    std::string point_cloud_topic_, point_cloud_frame_;

    // ROS Messages
    sensor_msgs::msg::PointCloud cloud_;

    nav_msgs::msg::Odometry::SharedPtr robot_odometry_;

    SocialHeatmap social_heatmap_ = SocialHeatmap();

    // pedsim messages
    pedsim_msgs::msg::AgentStates::SharedPtr agent_states_;
    pedsim_msgs::msg::AgentStates relevant_agent_states_;

    pedsim_msgs::msg::AgentStates social_agents_in_radius_;
    std::vector<pedsim_msgs::msg::AgentState> social_agents_in_radius_vector_;

    double social_agent_radius_;

    // Octree
    octomap::OcTree *octree_;
    double octree_resol_, rviz_timer_;
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    octomap::KeyRay m_keyRay; // temp storage for ray casting
    double mapping_max_range_, min_z_pc_, max_z_pc_;

    // grid map
    grid_map::GridMap grid_map_;

    grid_map_msgs::msg::GridMap grid_map_msg_;

    // social relevance validity checking constants
    double robot_distance_view_max_, robot_distance_view_min_, robot_velocity_threshold_, robot_angle_view_, actual_fov_distance_;
    double social_heatmap_decay_factor_;

    // Basic social personal space parameters defined
    double social_comfort_amplitude_ = 4;
    double sigma_x = 0.45;
    double sigma_y = 0.45;
    double fv = 0.8;
    double fFront = 0.2;
    double fFieldOfView = 0.0;

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

#endif // WORLD_MODELER_HPP
