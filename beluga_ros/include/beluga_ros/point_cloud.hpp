// Copyright 2024 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BELUGA_ROS_POINT_CLOUD_HPP
#define BELUGA_ROS_POINT_CLOUD_HPP


#include <range/v3/view/iota.hpp>

#include <beluga/sensor/data/point_cloud.hpp>
#include <beluga/views/take_evenly.hpp>
#include <beluga_ros/messages.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Dense>

/**
 * \file
 * \brief Implementation of `sensor_msgs/PointCloud2` wrapper type.
 */

namespace beluga_ros {

template<typename T>
struct DataType;

template<>
struct DataType<beluga_ros::msg::PointFieldU8> {
  using type = std::uint8_t;
};

template<>
struct DataType<beluga_ros::msg::PointFieldU16> {
  using type = std::uint16_t;
};

template<>
struct DataType<beluga_ros::msg::PointFieldU32> {
  using type = std::uint32_t;
};

template<>
struct DataType<beluga_ros::msg::PointFieldF32> {
  using type = float;
};

template<>
struct DataType<beluga_ros::msg::PointFieldF32> {
  using type = double;
};

/// Thin wrapper type for 3D `sensor_msgs/PointCloud2` messages.
/// Assumes an XYZ... type message.
/// Each field must have the same datatype.
/// The point cloud can't have invalid values, i.e., it must be dense.
class PointCloud3 : public beluga::BasePointCloud<PointCloud3> {
 public:
  /// Range type.
  using Scalar = double;

  /// Constructor.
  ///
  /// \param cloud Point cloud message.
  /// \param origin Point cloud frame origin in the filter frame.
  explicit PointCloud3(
      beluga_ros::msg::PointCloud2ConstSharedPtr cloud,
      Sophus::SE3d origin = Sophus::SE3d())
      : cloud_(std::move(cloud)),
        origin_(std::move(origin)),
        stride_(0),
        max_row_(3) {
          // Check there are not invalid values
          if (!cloud_->is_dense) throw std::invalid_argument("PointCloud is not dense");
          // Check if point cloud is 3D
          if (cloud_->fields.size() < 3) throw std::invalid_argument("PointCloud is not 3D");
          // Check point cloud is XYZ... type
          if (cloud_->fields.at(0).name != "x" && cloud_->fields.at(1).name != "y" && cloud_->fields.at(2).name != "z") throw std::invalid_argument("PointCloud not XYZ...");
          // Check all datatype is the same
          for (size_t i = 3; i < cloud_->fields.size(); ++i) {
            stride_ += sizeof(cloud_->fields.at(i).datatype);
          }
          stride_ = sizeof(float) + sizeof(std::uint16_t) + sizeof(float);
          beluga_ros::msg::PointCloud2ConstIterator<float> iterPoints(*cloud_, "x");
          initial_point_ = reinterpret_cast<uintptr_t>(&iterPoints[0]);
          max_col_ = cloud_->width * cloud_->height;
    assert(cloud_ != nullptr);
  }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get the unorganized 3D point collection as an Eigen Map.
  [[nodiscard]] auto point(size_t row, size_t col) const {
    return *reinterpret_cast<float*>(initial_point_ + row*sizeof(float) + col*stride_+ col*3*sizeof(float));
  }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& cols() const { return max_col_; }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& rows() const { return max_row_; }

 private:
  beluga_ros::msg::PointCloud2ConstSharedPtr cloud_;
  Sophus::SE3d origin_;
  uintptr_t initial_point_;
  int stride_;
  int max_row_;
  int max_col_;
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_POINT_CLOUD_HPP
