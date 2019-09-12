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
 Lane Detector

 Daiki HAYASHI
 */


#include <ros/ros.h>
#include <lane_detector.h>

namespace lane_detector {

  LaneDetector::LaneDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    nh_ = nh;
    pnh_ = pnh;

    // initialize variables here
    resetStates();
  }

  LaneDetector::~LaneDetector()
  {
    // deconstruct
  }

  void LaneDetector::callback_points(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    std::cout << __func__ << std::endl;

    // Check timestamp
    if (msg->header.stamp < previous_scan_time_)
      resetStates();

    // Convert to PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *pointcloud);

    // Debug
    if (debug_mode_) {
      analyzeIntensity(pointcloud);
    }

    // Filter points by intensity
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filterPoints(pointcloud, points_filtered);

    // TODO: detect curbs and filter the points inside the curbs

    // Detect lanes
    detectLanes(points_filtered);

    // Publish messages
    publishWhiteLines(msg->header);
    publishLaneInfo(msg->header);

  }

  void LaneDetector::detectLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr &points)
  {

    // initialize
    lane_width_ = initial_lane_width_;
    lane_offset_ = initial_lane_offset_;
    std::vector<double> coeffs;
    for (int i = 0; i < polynomial_order_ + 1; i++)
      coeffs.push_back(0.0);
    std::vector<bool> is_alive_left, is_alive_right;

    int num_iteration = 100;
    double learning_rate = 0.1;
    double threshold_convergence = 0.25;

    for (int itr=0; itr<num_iteration; itr++) {
      std::vector<Polynomial> polynomials;
      is_alive_left.clear();
      is_alive_right.clear();

      // white lines on the left
      for (int i = 0; i < max_num_lines_left_; i++) {
        Polynomial new_polynomial(polynomial_order_);
        coeffs[0] = lane_width_ * (float(i) + 0.5) + lane_offset_;
        new_polynomial.setCoefficients(coeffs);
        polynomials.push_back(new_polynomial);
        is_alive_left.push_back(false);
      }

      // white lines on the right
      for (int i = 0; i < max_num_lines_right_; i++) {
        Polynomial new_polynomial(polynomial_order_);
        coeffs[0] = (-1.0) * lane_width_ * (float(i) + 0.5) + lane_offset_;
        new_polynomial.setCoefficients(coeffs);
        polynomials.push_back(new_polynomial);
        is_alive_right.push_back(false);
      }

      // fitting iteration
      int poly_index = 0;
      long num_points_all = 0;
      double distance_averaged_all = 0.0, z_averaged_all = 0.0;
      for (auto &poly: polynomials) {
        if (poly_index >= 0 && poly_index < max_num_lines_left_) {
          poly_index++;
        }else if (poly_index == max_num_lines_left_) {
          poly_index = -1;
        }else {
          poly_index--;
        }

        std::vector<double> xs, ys;
        float distance;

        // extract points which are close to the line
        for (auto &point: points->points) {
          distance = point.y - poly.predict(point.x);
          if (fabs(distance) > distance_threshold_)
            continue;
          xs.push_back(point.x);
          ys.push_back(point.y);
          distance_averaged_all += distance;
          z_averaged_all += point.z;
        }
        if (xs.size() < ((polynomial_order_ + 1) * 2)) {
          continue;
        }

        // is_alive flag
        if (poly_index > 0) {
          is_alive_left[poly_index - 1] = true;
        }else {
          is_alive_right[abs(poly_index) - 1] = true;
        }

        num_points_all += xs.size();
        double ratio = double(xs.size()) / double(points->points.size());

        // estimate coefficients
        poly.fit(xs, ys);
        std::vector<double> estimated_coeffs = poly.getCoefficients();
        assert (coeffs.size() == estimated_coeffs.size());

        // calculate L2 norm
        double l2_norm = 0.0;
        for (int i = 1; i < coeffs.size(); i++) {
          l2_norm += pow(estimated_coeffs[i], 2);
        }
        if (l2_norm > 10.0) {
          printf("OVERFIT (iteration stopped): %f + %f*x + %f*x^2 + %f*x^3\n",
              estimated_coeffs[0], estimated_coeffs[1], estimated_coeffs[2], estimated_coeffs[3]);
          continue;
        }

        // learn coefficients
        for (int i = 1; i < coeffs.size(); i++) {
          coeffs[i] +=  learning_rate * ratio * estimated_coeffs[i];
        }

        // skip learning lane width in case of road edge
        if (abs(poly_index) != 1 && poly_index != max_num_lines_left_ && poly_index != -max_num_lines_right_) {
          if (poly_index > 0 && abs(poly_index) < line_is_alive_left_.size()) {
            if (!line_is_alive_left_[abs(poly_index)])
              continue;
          }
          if (poly_index < 0 && abs(poly_index) < line_is_alive_right_.size()){
            if (!line_is_alive_right_[abs(poly_index)])
              continue;
          }
        }

        // learn lane width
        double supposed_coeff_0;
        double error;
        if (poly_index > 0) {
          supposed_coeff_0 = lane_width_ * (float(poly_index) - 0.5) + lane_offset_;
        }else {
          supposed_coeff_0 = lane_width_ * (float(poly_index) + 0.5) + lane_offset_;
        }
        error = estimated_coeffs[0] - supposed_coeff_0;
        if (fabs(error) < 1.0)
          lane_width_ += learning_rate * ratio * error;

        // learn lane offset
        if (poly_index == 1) {
          if (fabs(estimated_coeffs[0]) < lane_width_)
            lane_offset_ += learning_rate * ratio * error;

          if (lane_offset_ > lane_width_)
            lane_offset_ -= lane_width_;
          if (lane_offset_ < (-1.0) * lane_width_)
            lane_offset_ += lane_width_;
        }
      }

      if (num_points_all == 0) {
        continue;
      }

      distance_averaged_all /= num_points_all;
      z_averaged_all /= num_points_all;
      average_z_ = z_averaged_all;

      // if converged
      if (fabs(distance_averaged_all) < threshold_convergence)
        break;

    }
    printf("%f*x + %f*x^2 + %f*x^3\n", coeffs[1], coeffs[2], coeffs[3]);
    std::cout << "lane offset: " << lane_offset_ << ", lane width: " << lane_width_ << std::endl;

    polynomial_coeffs_ = coeffs;
    line_is_alive_left_ = is_alive_left;
    line_is_alive_right_ = is_alive_right;
  }

  void LaneDetector::filterPoints(
      pcl::PointCloud<pcl::PointXYZI>::Ptr &points,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &points_filtered
      )
  {
    // threshold points by intensity
    for (auto &point: points->points) {
      if (point.intensity < intensity_threshold_)
        continue;
      points_filtered->points.push_back(point);
    }
  }

  void LaneDetector::analyzeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr &points)
  {
    int intensities[26];
    for (int i=0; i<26; i++) {
      intensities[i] = 0;
    }

    int intensity_index = 0;
    for (auto &point: points->points) {
      intensity_index = int(point.intensity / 10);
      intensities[intensity_index]++;
    }

    std::cout << "---" << std::endl;
    for (int i=0; i<26; i++) {
      std::cout << "intensity[" << i << "]: " << intensities[i] << std::endl;
    }
  }

  void LaneDetector::publishWhiteLines(const std_msgs::Header &header)
  {
    std::vector<Polynomial> polynomials;
    std::vector<double> coeffs(polynomial_coeffs_);
    int num_lines_left = 0, num_lines_right = 0;

    // white lines on the left
    for (int i = 0; i < max_num_lines_left_; i++) {
      if (!line_is_alive_left_[i])
        continue;
      Polynomial new_polynomial(polynomial_order_);
      coeffs[0] = lane_width_ * (float(i) + 0.5) + lane_offset_;
      new_polynomial.setCoefficients(coeffs);
      polynomials.push_back(new_polynomial);
      num_lines_left++;
    }

    // white lines on the right
    for (int i = 0; i < max_num_lines_right_; i++) {
      if (!line_is_alive_right_[i])
        continue;
      Polynomial new_polynomial(polynomial_order_);
      coeffs[0] = (-1.0) * lane_width_ * (float(i) + 0.5) + lane_offset_;
      new_polynomial.setCoefficients(coeffs);
      polynomials.push_back(new_polynomial);
      num_lines_right++;
    }

    // create message
    visualization_msgs::MarkerArray marker_array;
    uint32_t poly_index = 0;
    for (auto &polynomial: polynomials) {
      visualization_msgs::Marker marker = createMarker(polynomial, header);
      marker.ns = "white_line";
      marker.id = poly_index;
      marker.scale.x = 0.3;
      marker_array.markers.push_back(marker);
      poly_index++;
    }

    // publish
    white_line_publisher_.publish(marker_array);
  }

  void LaneDetector::publishLaneInfo(const std_msgs::Header &header)
  {
    // Lane width
    std_msgs::Float32 lane_width_msg;
    lane_width_msg.data = lane_width_;
    lane_width_publisher_.publish(lane_width_msg);

    // Lane offset
    std_msgs::Float32 lane_offset_msg;
    lane_offset_msg.data = lane_offset_;
    lane_offset_publisher_.publish(lane_offset_msg);

    // Ego lane index
    std_msgs::Int8 ego_lane_index_msg;
    for (int i=0; i<line_is_alive_left_.size(); i++) {
      if (line_is_alive_left_[i])
        ego_lane_index_msg.data = i;
    }
    if (ego_lane_index_msg.data == 0)
      ego_lane_index_msg.data = 1;
    ego_lane_index_publisher_.publish(ego_lane_index_msg);

    // Road shape polynomial coefficients
    std_msgs::Float64MultiArray road_shape_polynomial_coefficients_msg;
    for (int i = 1; i < polynomial_coeffs_.size(); i++) {
      road_shape_polynomial_coefficients_msg.data.push_back(polynomial_coeffs_[i]);
    }
    road_shape_polynomial_coefficients_publisher_.publish(road_shape_polynomial_coefficients_msg);
  }

  visualization_msgs::Marker LaneDetector::createMarker(
      Polynomial &polynomial, const std_msgs::Header &header
      )
  {
    // generate points
    std::vector<double> xs, ys;
    for (int i = 0; i < (int(marker_range_ / marker_interval_)); i++) {
      double x = double(i) * marker_interval_ - (marker_range_ / 2.0);
      xs.push_back(x);
    }
    polynomial.predict(xs, ys);
    assert(xs.size() == ys.size());

    // create message
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.lifetime = ros::Duration(0, 100000000);
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;
    marker.color = color;
    for (long i = 0; i < xs.size(); i++) {
      geometry_msgs::Point point;
      point.x = xs[i];
      point.y = ys[i];
      point.z = average_z_;
      marker.points.push_back(point);
    }

    return marker;
  }

  void LaneDetector::getParameters()
  {
    pnh_.param<std::string>("topic_pointcloud", topic_pointcloud_, "/points_ground");
    pnh_.param<bool>("debug_mode", debug_mode_, false);
    pnh_.param<int>("intensity_threshold", intensity_threshold_, 10);
    pnh_.param<double>("distance_threshold", distance_threshold_, 1.0);
    pnh_.param<int>("max_num_lines_left", max_num_lines_left_, 5);
    pnh_.param<int>("max_num_lines_right", max_num_lines_right_, 5);
    pnh_.param<int>("polynomial_order", polynomial_order_, 2);
    pnh_.param<double>("initial_lane_width", initial_lane_width_, 3.5);
    pnh_.param<double>("initial_lane_offset", initial_lane_offset_, 0.0);
    pnh_.param<double>("marker_range", marker_range_, 200.0);
    pnh_.param<double>("marker_interval", marker_interval_, 1.0);
  }

  void LaneDetector::resetStates()
  {
    std::cout << "reset states" << std::endl;

    previous_scan_time_ = ros::Time();

    lane_width_ = initial_lane_width_;
    lane_offset_ = initial_lane_offset_;
    polynomial_coeffs_.clear();
    line_is_alive_left_.clear();
    line_is_alive_right_.clear();
    average_z_ = 0.0;
  }

  void LaneDetector::MainLoop()
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
    white_line_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("white_lines", 1);
    lane_width_publisher_ = nh_.advertise<std_msgs::Float32>("lane_width", 1);
    lane_offset_publisher_ = nh_.advertise<std_msgs::Float32>("lane_offset", 1);
    ego_lane_index_publisher_ = nh_.advertise<std_msgs::Int8>("ego_lane_index", 1);
    road_shape_polynomial_coefficients_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>
        ("road_shape_polynomial_coefficients", 1);

    // subscribe points_raw and current_velocity
    ros::Subscriber points_sub = nh_.subscribe<sensor_msgs::PointCloud2>
        (topic_pointcloud_, 1000, &LaneDetector::callback_points, this);

    ros::Rate loop_rate(10);
    ros::spin();

  }



} // end of namespace: lane_detector
