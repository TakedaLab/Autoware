//
// Created by d_hayashi on 6/20/19.
//

#ifndef ROS_POINTS_ACCUMULATOR_H
#define ROS_POINTS_ACCUMULATOR_H

#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <boost/format.hpp>
#include <stdlib.h>
#include <string>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


namespace points_accumulator {

  struct pose
  {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };

  class PointsAccumulator {
  public:
    PointsAccumulator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~PointsAccumulator();
    void MainLoop();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // publisher
    ros::Publisher accumulated_points_publisher_;

    int number_of_accumulations_ = 5;
    bool use_current_velocity_, remove_points_negative_z_;
    std::string topic_pointcloud_, topic_current_velocity_;
    std::string output_frame_id_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr previous_scan_ptr_;
    pose previous_pose_diff_, guess_pose_diff_, guess_pose_diff_current_velocity_, diff_pose_diff_, current_pose_diff_;
    geometry_msgs::TwistStamped previous_current_velocity_;
    ros::Time previous_scan_time_;
    std::deque<std::pair<pcl::PointCloud<pcl::PointXYZI>, Eigen::Matrix4f> > points_and_mats_;

    // Default values
    int max_iter_ = 30;        // Maximum iterations
    float ndt_res_ = 1.0;      // Resolution
    double step_size_ = 0.1;   // Step size
    double trans_eps_ = 0.01;  // Transformation epsilon
    double voxel_leaf_size_ = 2.0; // Leaf size of VoxelGrid filter.

    // tf
    tf::TransformListener tf_listener_;

    // callbacks
    void callback_current_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void callback_points(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // internal functions
    Eigen::Matrix4f estimateDifference(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void estimatePoseFromCurrentVelocity(ros::Time time);
    void accumulatePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &points_accumulated);
    double wrapToPm(double a_num, const double a_max);
    double wrapToPmPi(double a_angle_rad);
    void getParameters();
    void resetStates();

  };

} // namespace points_accumulator


#endif //ROS_POINTS_ACCUMULATOR_H
