/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 Points Accumulator

 Daiki HAYASHI
 */


#include <ros/ros.h>
#include <points_accumulator.h>

namespace points_accumulator {

  PointsAccumulator::PointsAccumulator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    nh_ = nh;
    pnh_ = pnh;

    // initialize variables here
    resetStates();

  }

  PointsAccumulator::~PointsAccumulator()
  {
    // deconstruct
  }

  void PointsAccumulator::callback_current_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    std::cout << __func__ << std::endl;
    previous_current_velocity_ = *msg;
  }

  void PointsAccumulator::callback_points(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
//    std::cout << __func__ << std::endl;

    // Check timestamp
    if (msg->header.stamp < previous_scan_time_)
      resetStates();

    // Transform to output_frame_id_
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    tf_listener_.waitForTransform(output_frame_id_, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
    pcl_ros::transformPointCloud(output_frame_id_, *cloud, *cloud, tf_listener_);

    // Apply NDT and calculate transformation
    ros::Duration dt = msg->header.stamp - previous_scan_time_;
    Eigen::Matrix4f mat_diff = estimateDifference(cloud);

    // Calculate velocity
    double vx = 0.0, vy = 0.0, vz = 0.0;
    try {
      vx = mat_diff(0, 3) / dt.toSec();
      vy = mat_diff(1, 3) / dt.toSec();
      vz = mat_diff(2, 3) / dt.toSec();
    } catch (char *str) {
      std::cerr << str;
    }

    // Convert to PointCloud
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    pcl::fromROSMsg(*cloud, pointcloud);

    // Store to deque
    std::pair<pcl::PointCloud<pcl::PointXYZI>, Eigen::Matrix4f> points_and_mat;
    points_and_mat.first = pointcloud;
    points_and_mat.second = mat_diff;
    points_and_mats_.pop_back();
    points_and_mats_.push_front(points_and_mat);

    std::cout
      << "dx: " << mat_diff(0, 3)
      << ", dy: " << mat_diff(1, 3)
      << ", dz: " << mat_diff(2, 3)
      << std::endl;

    // Accumulate points
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_accumulated(new pcl::PointCloud<pcl::PointXYZI>());
    accumulatePoints(points_accumulated);

    // Publish points
    sensor_msgs::PointCloud2::Ptr cloud_to_publish(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*points_accumulated, *cloud_to_publish);
    cloud_to_publish->header = msg->header;
    accumulated_points_publisher_.publish(*cloud_to_publish);

    // Publish velocity
    geometry_msgs::TwistStamped velocity;
    velocity.header.stamp = msg->header.stamp;
    velocity.twist.linear.x = vx;
    velocity.twist.linear.y = vy;
    velocity.twist.linear.z = vz;
    estimated_velocity_publisher_.publish(velocity);

  }

//  void PointsAccumulator::estimateDifference(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
  Eigen::Matrix4f PointsAccumulator::estimateDifference(
          const sensor_msgs::PointCloud2::ConstPtr& msg
          )
  {
//    std::cout << __func__ << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // remove points of z<0
    if (remove_points_negative_z_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
      for (auto &point: cloud->points) {
        if (point.z >= 0) {
          cloud2->points.push_back(point);
        }
      }
      cloud = cloud2;
    }

    // Apply voxelgrid filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_scan_ptr->is_dense = false;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid_filter.setInputCloud(previous_scan_ptr_);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    // Remove NaN and Inf from point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan2_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_scan_ptr->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*filtered_scan_ptr, *filtered_scan2_ptr, indices);
    filtered_scan_ptr = filtered_scan2_ptr;

    if (previous_scan_ptr_->points.empty()) {
      previous_scan_ptr_ = cloud;
      Eigen::Matrix4f mat_identity(Eigen::Matrix4f::Identity());
      return mat_identity;
    }

    // Apply NDT to analyze moved distance
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setTransformationEpsilon(trans_eps_);
    ndt.setStepSize(step_size_);
    ndt.setResolution(ndt_res_);
    ndt.setMaximumIterations(max_iter_);
    ndt.setInputSource(filtered_scan_ptr);
    ndt.setInputTarget(cloud);

    // Setup initial guess
    guess_pose_diff_.x = previous_pose_diff_.x + diff_pose_diff_.x;
    guess_pose_diff_.y = previous_pose_diff_.y + diff_pose_diff_.y;
    guess_pose_diff_.z = previous_pose_diff_.z + diff_pose_diff_.z;
    guess_pose_diff_.roll = previous_pose_diff_.roll;
    guess_pose_diff_.pitch = previous_pose_diff_.pitch;
    guess_pose_diff_.yaw = previous_pose_diff_.yaw + diff_pose_diff_.yaw;

    pose guess_pose_diff_for_ndt = guess_pose_diff_;
    if (use_current_velocity_) {
      estimatePoseFromCurrentVelocity(msg->header.stamp);
      guess_pose_diff_for_ndt = guess_pose_diff_current_velocity_;
    }

    Eigen::AngleAxisf init_rotation_x(guess_pose_diff_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose_diff_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose_diff_for_ndt.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(guess_pose_diff_for_ndt.x, guess_pose_diff_for_ndt.y, guess_pose_diff_for_ndt.z);

    Eigen::Matrix4f init_guess =
        (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt.align(*output_cloud, init_guess);
    double fitness_score = ndt.getFitnessScore();

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    t_localizer = ndt.getFinalTransformation();

    tf::Matrix3x3 mat_l;
    mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    // Update localizer_pose.
    pose ndt_pose;
    ndt_pose.x = t_localizer(0, 3);
    ndt_pose.y = t_localizer(1, 3);
    ndt_pose.z = t_localizer(2, 3);
    mat_l.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    // Update states
    current_pose_diff_.x = ndt_pose.x;
    current_pose_diff_.y = ndt_pose.y;
    current_pose_diff_.z = ndt_pose.z;
    current_pose_diff_.roll = ndt_pose.roll;
    current_pose_diff_.pitch = ndt_pose.pitch;
    current_pose_diff_.yaw = ndt_pose.yaw;

    // Calculate the offset (curren_pos - previous_pos)
    diff_pose_diff_.x = current_pose_diff_.x - previous_pose_diff_.x;
    diff_pose_diff_.y = current_pose_diff_.y - previous_pose_diff_.y;
    diff_pose_diff_.z = current_pose_diff_.z - previous_pose_diff_.z;
    diff_pose_diff_.roll = current_pose_diff_.roll - previous_pose_diff_.roll;
    diff_pose_diff_.pitch = current_pose_diff_.pitch - previous_pose_diff_.pitch;
    diff_pose_diff_.yaw = current_pose_diff_.yaw - previous_pose_diff_.yaw;

    // Update position and posture. current_pos -> previous_pos
    previous_pose_diff_.x = current_pose_diff_.x;
    previous_pose_diff_.y = current_pose_diff_.y;
    previous_pose_diff_.z = current_pose_diff_.z;
    previous_pose_diff_.roll = current_pose_diff_.roll;
    previous_pose_diff_.pitch = current_pose_diff_.pitch;
    previous_pose_diff_.yaw = current_pose_diff_.yaw;

    previous_scan_time_ = msg->header.stamp;
    previous_scan_ptr_ = cloud;

    return t_localizer;
  }

  void PointsAccumulator::estimatePoseFromCurrentVelocity(ros::Time time)
  {
    double time_diff = (time - previous_scan_time_).toSec();
    if (time_diff < 0) {
      std::cerr << "message jumped backward, so skipped estimating pose" << std::endl;
      return;
    }

    // Estimate pose
    guess_pose_diff_current_velocity_.x = (-1.0) * previous_current_velocity_.twist.linear.x * time_diff;
    guess_pose_diff_current_velocity_.y = (-1.0) * previous_current_velocity_.twist.linear.y * time_diff;
    guess_pose_diff_current_velocity_.z = (-1.0) * previous_current_velocity_.twist.linear.z * time_diff;
    guess_pose_diff_current_velocity_.roll = (-1.0) * previous_current_velocity_.twist.angular.x * time_diff;
    guess_pose_diff_current_velocity_.pitch = (-1.0) * previous_current_velocity_.twist.angular.y * time_diff;
    guess_pose_diff_current_velocity_.yaw = (-1.0) * previous_current_velocity_.twist.angular.z * time_diff;
  }

  void PointsAccumulator::accumulatePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &points_accumulated)
  {
//    pcl::PointCloud<pcl::PointXYZI>::Ptr points_accumulated(new pcl::PointCloud<pcl::PointXYZI>());
    int count = 0;
    Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
    for(auto itr = points_and_mats_.begin(); itr != points_and_mats_.end(); ++itr) {
//      std::cout << "count: " << count << std::endl;
      if (itr->first.points.size() == 0)
        continue;
      if (count > 0) {
        transformation = itr->second * transformation;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr points_transformed(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(itr->first, *points_transformed, transformation);
      *points_accumulated += *points_transformed;
      count++;
    }
  }

  double PointsAccumulator::wrapToPm(double a_num, const double a_max)
  {
    if (a_num >= a_max)
    {
      a_num -= 2.0 * a_max;
    }
    return a_num;
  }

  double PointsAccumulator::wrapToPmPi(double a_angle_rad)
  {
    return wrapToPm(a_angle_rad, M_PI);
  }

  void PointsAccumulator::getParameters()
  {
    pnh_.param<int>("num_accumulations", number_of_accumulations_, 5);
    pnh_.param<std::string>("topic_pointcloud", topic_pointcloud_, "/points_raw");
    pnh_.param<bool>("use_current_velocity", use_current_velocity_, false);
    pnh_.param<bool>("remove_points_negative_z", remove_points_negative_z_, false);
    pnh_.param<std::string>("topic_current_velocity", topic_current_velocity_, "/current_velocity");
    pnh_.param<std::string>("output_frame_id", output_frame_id_, "/velodyne");
  }

  void PointsAccumulator::resetStates()
  {
    previous_scan_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    previous_current_velocity_ = geometry_msgs::TwistStamped();

    // initialize deque
    std::deque<std::pair<pcl::PointCloud<pcl::PointXYZI>, Eigen::Matrix4f> > temp(number_of_accumulations_);
    points_and_mats_ = temp;

    previous_pose_diff_.x = 0.0;
    previous_pose_diff_.y = 0.0;
    previous_pose_diff_.z = 0.0;
    previous_pose_diff_.roll = 0.0;
    previous_pose_diff_.pitch = 0.0;
    previous_pose_diff_.yaw = 0.0;

    current_pose_diff_.x = 0.0;
    current_pose_diff_.y = 0.0;
    current_pose_diff_.z = 0.0;
    current_pose_diff_.roll = 0.0;
    current_pose_diff_.pitch = 0.0;
    current_pose_diff_.yaw = 0.0;

    guess_pose_diff_.x = 0.0;
    guess_pose_diff_.y = 0.0;
    guess_pose_diff_.z = 0.0;
    guess_pose_diff_.roll = 0.0;
    guess_pose_diff_.pitch = 0.0;
    guess_pose_diff_.yaw = 0.0;

    guess_pose_diff_current_velocity_.x = 0.0;
    guess_pose_diff_current_velocity_.y = 0.0;
    guess_pose_diff_current_velocity_.z = 0.0;
    guess_pose_diff_current_velocity_.roll = 0.0;
    guess_pose_diff_current_velocity_.pitch = 0.0;
    guess_pose_diff_current_velocity_.yaw = 0.0;

    diff_pose_diff_.x = 0.0;
    diff_pose_diff_.y = 0.0;
    diff_pose_diff_.z = 0.0;
    diff_pose_diff_.roll = 0.0;
    diff_pose_diff_.pitch = 0.0;
    diff_pose_diff_.yaw = 0.0;

    previous_scan_time_ = ros::Time(0);
  }

  void PointsAccumulator::MainLoop()
  {

    // debug mode
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }

    // get parameters
    getParameters();

    // reset states
    resetStates();

    // prepare publisher
    accumulated_points_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("points_accumulated", 1);
    estimated_velocity_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("estimated_velocity", 1);

    // subscribe points_raw and current_velocity
    ros::Subscriber points_sub = nh_.subscribe<sensor_msgs::PointCloud2>
        (topic_pointcloud_, 1000, &PointsAccumulator::callback_points, this);
    ros::Subscriber current_velocity_sub = nh_.subscribe<geometry_msgs::TwistStamped>
        (topic_current_velocity_, 1000, &PointsAccumulator::callback_current_velocity, this);

    ros::Rate loop_rate(10);
    ros::spin();

  }



} // end of namespace: points_accumulator
