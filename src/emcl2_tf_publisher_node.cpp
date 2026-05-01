// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/emcl2_tf_publisher_node.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <functional>

namespace emcl2
{
EMcl2TfPublisherNode::EMcl2TfPublisherNode()
: Node("emcl2_tf_publisher_node"),
  latest_map_to_odom_(tf2::Transform::getIdentity()),
  odom_freq_(20),
  transform_tolerance_(0.2),
  has_transform_(false),
  has_pending_pose_(false),
  pending_pose_uses_latest_odom_(false)
{
	declareParameter();
	initCommunication();
	initTF();
}

void EMcl2TfPublisherNode::declareParameter()
{
	this->declare_parameter("global_frame_id", std::string("map"));
	this->declare_parameter("footprint_frame_id", std::string("base_footprint"));
	this->declare_parameter("odom_frame_id", std::string("odom"));
	this->declare_parameter("odom_freq", 20);
	this->declare_parameter("transform_tolerance", 0.2);
}

void EMcl2TfPublisherNode::initCommunication()
{
	initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
	  "initialpose", 2,
	  std::bind(&EMcl2TfPublisherNode::initialPoseReceived, this, std::placeholders::_1));
	mcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
	  "mcl_pose", 2,
	  std::bind(&EMcl2TfPublisherNode::mclPoseReceived, this, std::placeholders::_1));
	final_transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
	  "final_map_to_odom_transform", rclcpp::QoS(1).transient_local().reliable());

	this->get_parameter("global_frame_id", global_frame_id_);
	this->get_parameter("footprint_frame_id", footprint_frame_id_);
	this->get_parameter("odom_frame_id", odom_frame_id_);
	this->get_parameter("odom_freq", odom_freq_);
	this->get_parameter("transform_tolerance", transform_tolerance_);
	if (odom_freq_ <= 0) {
		RCLCPP_WARN(get_logger(), "Invalid odom_freq %d, fallback to 20 Hz.", odom_freq_);
		odom_freq_ = 20;
	}

	const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
	  std::chrono::duration<double>(1.0 / static_cast<double>(odom_freq_)));
	publish_timer_ = create_wall_timer(
	  period, std::bind(&EMcl2TfPublisherNode::publishLatestTransform, this));
}

void EMcl2TfPublisherNode::initTF()
{
	tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	  get_node_base_interface(), get_node_timers_interface(),
	  create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
	tf_->setCreateTimerInterface(timer_interface);
	tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
	tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void EMcl2TfPublisherNode::initialPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
	last_pose_ = *msg;
	has_pending_pose_ = true;
	pending_pose_uses_latest_odom_ = true;

	if (updateTransformFromPose(*msg, true)) {
		RCLCPP_INFO(get_logger(), "Updated map->odom transform from initialpose.");
	}
}

void EMcl2TfPublisherNode::mclPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
	last_pose_ = *msg;
	has_pending_pose_ = true;
	pending_pose_uses_latest_odom_ = false;

	if (updateTransformFromPose(*msg, false)) {
		RCLCPP_DEBUG(get_logger(), "Updated map->odom transform from mcl_pose.");
	}
}

bool EMcl2TfPublisherNode::normalizePoseToGlobalFrame(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  geometry_msgs::msg::PoseStamped & pose)
{
	geometry_msgs::msg::PoseStamped source_pose;
	source_pose.header = msg.header;
	source_pose.pose = msg.pose.pose;
	if (source_pose.header.frame_id.empty()) {
		source_pose.header.frame_id = global_frame_id_;
	}

	if (source_pose.header.frame_id == global_frame_id_) {
		pose = source_pose;
		return true;
	}

	try {
		tf_->transform(source_pose, pose, global_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN_THROTTLE(
		  get_logger(), *get_clock(), 5000, "Failed to transform pose from %s to %s (%s)",
		  source_pose.header.frame_id.c_str(), global_frame_id_.c_str(), e.what());
		return false;
	}
	return true;
}

bool EMcl2TfPublisherNode::updateTransformFromPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg, bool use_latest_odom)
{
	geometry_msgs::msg::PoseStamped map_to_base_pose;
	if (!normalizePoseToGlobalFrame(msg, map_to_base_pose)) {
		return false;
	}

	tf2::Transform map_to_base;
	tf2::convert(map_to_base_pose.pose, map_to_base);

	geometry_msgs::msg::PoseStamped base_to_map_pose;
	base_to_map_pose.header.frame_id = footprint_frame_id_;
	if (use_latest_odom || rclcpp::Time(map_to_base_pose.header.stamp).nanoseconds() == 0) {
		base_to_map_pose.header.stamp = rclcpp::Time(0);
	} else {
		base_to_map_pose.header.stamp = map_to_base_pose.header.stamp;
	}
	tf2::toMsg(map_to_base.inverse(), base_to_map_pose.pose);

	geometry_msgs::msg::PoseStamped odom_to_map_pose;
	try {
		tf_->transform(base_to_map_pose, odom_to_map_pose, odom_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN_THROTTLE(
		  get_logger(), *get_clock(), 5000, "Failed to compute map->odom transform (%s)",
		  e.what());
		return false;
	}

	tf2::Transform odom_to_map;
	tf2::convert(odom_to_map_pose.pose, odom_to_map);
	latest_map_to_odom_ = odom_to_map.inverse();
	has_transform_ = true;
	publishLatestTransform();
	return true;
}

void EMcl2TfPublisherNode::publishLatestTransform()
{
	if (!has_transform_) {
		if (has_pending_pose_) {
			updateTransformFromPose(last_pose_, pending_pose_uses_latest_odom_);
		}
		return;
	}

	geometry_msgs::msg::TransformStamped transform;
	transform.header.frame_id = global_frame_id_;
	transform.header.stamp =
	  get_clock()->now() + rclcpp::Duration::from_seconds(transform_tolerance_);
	transform.child_frame_id = odom_frame_id_;
	tf2::convert(latest_map_to_odom_, transform.transform);

	tfb_->sendTransform(transform);
	final_transform_pub_->publish(transform);
}

}  // namespace emcl2

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<emcl2::EMcl2TfPublisherNode>());
	rclcpp::shutdown();
	return 0;
}
