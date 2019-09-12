//
// Created by d_hayashi on 9/10/19.
//

#ifndef ROS_LANE_DETECTOR_H
#define ROS_LANE_DETECTOR_H

#include <cmath>
#include <deque>
#include <stdlib.h>
#include <string>
#include <iostream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <polynomial.h>


namespace lane_detector {

  struct pose
  {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };

  class LaneDetector {
  public:
    LaneDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~LaneDetector();
    void MainLoop();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // publisher
    ros::Publisher white_line_publisher_;
    ros::Publisher lane_width_publisher_;
    ros::Publisher lane_offset_publisher_;
    ros::Publisher ego_lane_index_publisher_;
    ros::Publisher road_shape_polynomial_coefficients_publisher_;

    // variables
    bool debug_mode_;
    std::string topic_pointcloud_;
    ros::Time previous_scan_time_;
    double lane_width_, lane_offset_, average_z_;
    std::vector<double> polynomial_coeffs_;
    std::vector<bool> line_is_alive_left_, line_is_alive_right_;

    // Default values
    int intensity_threshold_ = 10;           // points with intensity higher than this will be used to detect lanes
    int max_num_lines_left_ = 4;             // maximum number of lanes on the left
    int max_num_lines_right_ = 4;            // maximum number of lanes on the right
    int polynomial_order_ = 3;               // the order of polynomial functions
    double initial_lane_width_ = 3.5;        // lane width (initial value)
    double initial_lane_offset_ = 0.0;       // distance of the ego-vehicle from its lane center (initial value)
    double distance_threshold_ = 1.0;        // threshold of distance between white line and points
    double marker_range_ = 200;              // range to visualize marker in meters
    double marker_interval_ = 1;             // interval to visualize marker in meters

    // callbacks
    void callback_points(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // internal functions
    void detectLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr &points);
    void filterPoints(
        pcl::PointCloud<pcl::PointXYZI>::Ptr &points,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &points_filtered
        );
    void analyzeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr &points);
    void publishWhiteLines(const std_msgs::Header &header);
    void publishLaneInfo(const std_msgs::Header &header);
    visualization_msgs::Marker createMarker(
        Polynomial &polynomial, const std_msgs::Header &header
        );
    void getParameters();
    void resetStates();

  };

} // namespace lane_detector


#endif //ROS_LANE_DETECTOR_H
