// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__EMCL2_TF_PUBLISHER_NODE_H_
#define EMCL2__EMCL2_TF_PUBLISHER_NODE_H_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace emcl2
{
class EMcl2TfPublisherNode : public rclcpp::Node
{
      public:
	EMcl2TfPublisherNode();

      private:
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
	  initial_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
	  mcl_pose_sub_;
	rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr final_transform_pub_;
	rclcpp::TimerBase::SharedPtr publish_timer_;

	std::shared_ptr<tf2_ros::Buffer> tf_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;

	std::string global_frame_id_;
	std::string odom_frame_id_;
	std::string footprint_frame_id_;

	tf2::Transform latest_map_to_odom_;
	geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;

	int odom_freq_;
	double transform_tolerance_;
	bool has_transform_;
	bool has_pending_pose_;
	bool pending_pose_uses_latest_odom_;

	void declareParameter();
	void initCommunication();
	void initTF();

	void initialPoseReceived(
	  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
	void mclPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

	bool updateTransformFromPose(
	  const geometry_msgs::msg::PoseWithCovarianceStamped & msg, bool use_latest_odom);
	bool normalizePoseToGlobalFrame(
	  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
	  geometry_msgs::msg::PoseStamped & pose);
	void publishLatestTransform();
};

}  // namespace emcl2

#endif	// EMCL2__EMCL2_TF_PUBLISHER_NODE_H_
