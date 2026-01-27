//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  while (ros::ok()) { usleep(10000); }

  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: rclcpp_lifecycle::LifecycleNode("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);
  DRIVER_INFO(*this, "[Lifecycle] DriverNode created (unconfigured). Declaring parameters...");

  this->declare_parameter("xfer_format", static_cast<int>(kPointCloud2Msg));
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", static_cast<int>(kSourceRawLidar));
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", static_cast<int>(kOutputToRos));
  this->declare_parameter("frame_id", std::string("frame_default"));
  this->declare_parameter("user_config_path", std::string("path_default"));
  this->declare_parameter("cmdline_input_bd_code", std::string("000000000000001"));
  this->declare_parameter("lvx_file_path", std::string("/home/livox/livox_test.lvx"));

  // TODO(lifecycle-orchestrator): Remove the auto-transition block below once a
  // lifecycle manager (e.g. nav2_lifecycle_manager or a custom orchestrator) is
  // responsible for coordinating node startup order across the system.
  // At that point, the orchestrator will call configure -> activate externally
  // via the /livox_driver_node/change_state service.
  DRIVER_INFO(*this, "[Lifecycle] Auto-transitioning: configure -> activate...");
  auto configure_result = trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    DRIVER_ERROR(*this, "[Lifecycle] Auto-configure failed!");
    throw std::runtime_error("Lifecycle auto-configure failed");
  }
  auto activate_result = trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    DRIVER_ERROR(*this, "[Lifecycle] Auto-activate failed!");
    throw std::runtime_error("Lifecycle auto-activate failed");
  }
  DRIVER_INFO(*this, "[Lifecycle] Auto-transition complete. Node is ACTIVE.");
}

DriverNode::CallbackReturn DriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  DRIVER_INFO(*this, "[Lifecycle] Configuring...");

  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0;
  int output_type = kOutputToRos;
  std::string frame_id;

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  }

  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
      return CallbackReturn::FAILURE;
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
    return CallbackReturn::FAILURE;
  }

  DRIVER_INFO(*this, "[Lifecycle] Configuration complete.");
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  DRIVER_INFO(*this, "[Lifecycle] Activating...");

  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->CleanRequestExit();
  }

  active_.store(true);
  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);

  DRIVER_INFO(*this, "[Lifecycle] Node is now ACTIVE.");
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  DRIVER_INFO(*this, "[Lifecycle] Deactivating...");

  active_.store(false);

  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->RequestExit();
    // Wake threads blocked on semaphore Wait() so they can check active_ and exit
    lddc_ptr_->lds_->pcd_semaphore_.Signal();
    lddc_ptr_->lds_->imu_semaphore_.Signal();
  }

  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
  pointclouddata_poll_thread_.reset();
  imudata_poll_thread_.reset();

  if (lddc_ptr_) {
    lddc_ptr_->ResetPublishers();
  }

  DRIVER_INFO(*this, "[Lifecycle] Node is now INACTIVE.");
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  DRIVER_INFO(*this, "[Lifecycle] Cleaning up...");

  if (lddc_ptr_) {
    lddc_ptr_->PrepareExit();
    lddc_ptr_.reset();
  }

  DRIVER_INFO(*this, "[Lifecycle] Cleanup complete.");
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  DRIVER_INFO(*this, "[Lifecycle] Shutting down...");

  active_.store(false);
  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->RequestExit();
    lddc_ptr_->lds_->pcd_semaphore_.Signal();
    lddc_ptr_->lds_->imu_semaphore_.Signal();
  }
  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
  pointclouddata_poll_thread_.reset();
  imudata_poll_thread_.reset();
  if (lddc_ptr_) {
    lddc_ptr_->PrepareExit();
    lddc_ptr_.reset();
  }

  DRIVER_INFO(*this, "[Lifecycle] Shutdown complete.");
  return CallbackReturn::SUCCESS;
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()
{
  std::this_thread::sleep_for(std::chrono::seconds(3));
  while (active_.load()) {
    lddc_ptr_->DistributePointCloudData();
  }
}

void DriverNode::ImuDataPollThread()
{
  std::this_thread::sleep_for(std::chrono::seconds(3));
  while (active_.load()) {
    lddc_ptr_->DistributeImuData();
  }
}





















