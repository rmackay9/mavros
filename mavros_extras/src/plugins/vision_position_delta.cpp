/**
 * @brief VisionPositionDelta plugin
 * @file vision_position_delta.cpp
 * @author Randy Mackay <rmackay9@yahoo.com>
 *
 * @addtogroup plugin
 */
/*
 * Copyright 2017 Randy Mackay
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace extra_plugins{
/**
 * @brief Vision angle and position delta publishing plugin
 *
 * Retrieves odometry information from ZED camera and republishes to ArduPilot
 * using MAVLink's VISION_POSITION_DELTA message
 *
 */
class VisionPositionDeltaPlugin : public plugin::PluginBase,
	private plugin::TF2ListenerMixin<VisionPositionDeltaPlugin> {
public:
	VisionPositionDeltaPlugin() : PluginBase()
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

        // subscribe to odometry updates from zed camera
        odom_sub = sp_nh.subscribe("/zed/odom", 30, &VisionPositionDeltaPlugin::odom_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;
	ros::Subscriber odom_sub;
	ros::Time last_update;

	/* send vision_position_delta message to ardupilot */
	void vision_position_delta_send(
            uint64_t time_usec,                 // Timestamp (microseconds, synced to UNIX time or since system boot)
            uint64_t time_delta_usec,           // Time in microseconds since the last reported camera frame
			Eigen::Vector3d &angle_delta,       // Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientation
            Eigen::Vector3d &position_delta,    // Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right, 2=down)
			float confidence)                   // normalised confidence value from 0 to 100
	{
		mavlink::ardupilotmega::msg::VISION_POSITION_DELTA vpd{};

		vpd.time_usec = time_usec;
        vpd.time_delta_usec = time_delta_usec;
        vpd.angle_delta = angle_delta;
        vpd.position_delta = position_delta;
        vpd.confidence = confidence;
		UAS_FCU(m_uas)->send_message_ignore_drop(vpd);
	}

	/* callback when odometry is updated, triggers send to ardupilot */
	void odom_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
        // get time of this message
        // calculate time_delta_usec using last_update (sanity check time difference)
        // store time of this message in last_update
        uint64_t time_usec = 0;
        uint64_t time_delta_usec = 0;
		Eigen::Vector3d angle_delta;
        Eigen::Vector3d position_delta;
        float confidence = 0;
		vision_position_delta_send(time_usec, time_delta_usec, angle_delta, position_delta, confidence);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisionPositionDelta, mavros::plugin::PluginBase)
