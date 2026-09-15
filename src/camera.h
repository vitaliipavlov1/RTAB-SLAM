#pragma once

// Intel RealSense D435i capture: the newest RGB-D frameset for the main loop and
// an IMU orientation from RTAB-Map's own Madgwick filter.

#include "options.h"

#include <librealsense2/rs.hpp>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/core/Transform.h>

#include <opencv2/core.hpp>

#include <cmath>
#include <memory>
#include <mutex>

namespace rtabmap_minimal {

// Shared between the RealSense callback thread and the main loop.
struct Capture
{
	std::mutex mutex;
	rs2::frameset frames;
	bool hasFrames = false;

	std::unique_ptr<rtabmap::IMUFilter> imuFilter;   // Madgwick filter from RTAB-Map
	cv::Vec3d gyro, accel;
	bool hasAccel = false;
	bool imuReady = false;
};

struct Camera
{
	rs2::pipeline pipe;
	rs2::pipeline_profile profile;
	rtabmap::CameraModel model;
	rtabmap::Transform imuLocalTransform = rtabmap::Transform::getIdentity();
	float depthScale = 0.001f;
	bool depthInMillimeters = true;
};

// rs2_extrinsics rotation is column-major.
inline rtabmap::Transform fromRealSense(const rs2_extrinsics & e)
{
	return rtabmap::Transform(
		e.rotation[0], e.rotation[3], e.rotation[6], e.translation[0],
		e.rotation[1], e.rotation[4], e.rotation[7], e.translation[1],
		e.rotation[2], e.rotation[5], e.rotation[8], e.translation[2]);
}

// Video framesets and IMU samples arrive on the same callback thread; the main
// loop always takes the newest frameset and drops the rest (weak CPU friendly).
inline bool startCamera(const Options & options, Capture & capture, Camera & camera)
{
	rs2::config config;
	config.enable_stream(RS2_STREAM_COLOR, options.width, options.height, RS2_FORMAT_BGR8, options.fps);
	config.enable_stream(RS2_STREAM_DEPTH, options.width, options.height, RS2_FORMAT_Z16, options.fps);
	if(options.useImu)
	{
		config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
		config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	}

	const bool useImu = options.useImu;
	auto callback = [&capture, useImu](const rs2::frame & frame)
	{
		if(rs2::frameset frameset = frame.as<rs2::frameset>())
		{
			const std::lock_guard<std::mutex> lock(capture.mutex);
			capture.frames = frameset;
			capture.hasFrames = true;
			return;
		}
		if(!useImu)
		{
			return;
		}
		const rs2::motion_frame motion = frame.as<rs2::motion_frame>();
		if(!motion)
		{
			return;
		}
		const rs2_vector v = motion.get_motion_data();
		const std::lock_guard<std::mutex> lock(capture.mutex);
		if(motion.get_profile().stream_type() == RS2_STREAM_ACCEL)
		{
			capture.accel = cv::Vec3d(v.x, v.y, v.z);
			capture.hasAccel = true;
		}
		else if(capture.hasAccel)
		{
			capture.gyro = cv::Vec3d(v.x, v.y, v.z);
			capture.imuFilter->update(
					capture.gyro[0], capture.gyro[1], capture.gyro[2],
					capture.accel[0], capture.accel[1], capture.accel[2],
					motion.get_timestamp() / 1000.0);
			capture.imuReady = true;
		}
	};

	try
	{
		camera.profile = camera.pipe.start(config, callback);
	}
	catch(const rs2::error & e)
	{
		printf("Failed to start the RealSense pipeline: %s\n", e.what());
		return false;
	}

	const rs2::video_stream_profile colorProfile =
			camera.profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	const rs2_intrinsics intrinsics = colorProfile.get_intrinsics();
	camera.model = rtabmap::CameraModel(
			"d435i", intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
			rtabmap::CameraModel::opticalRotation(), 0, cv::Size(intrinsics.width, intrinsics.height));

	if(options.useImu)
	{
		// IMU pose in the same base frame as the camera model.
		camera.imuLocalTransform = rtabmap::CameraModel::opticalRotation() *
				fromRealSense(camera.profile.get_stream(RS2_STREAM_GYRO).get_extrinsics_to(colorProfile));

		// Image and IMU stamps must share a clock, which is what global time gives us.
		for(rs2::sensor & sensor : camera.profile.get_device().query_sensors())
		{
			if(sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED) &&
					sensor.get_option(RS2_OPTION_GLOBAL_TIME_ENABLED) == 0.0f)
			{
				printf("Warning: global time is disabled on '%s', IMU and image stamps may not match.\n",
						sensor.get_info(RS2_CAMERA_INFO_NAME));
			}
		}
	}

	camera.depthScale = camera.profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
	camera.depthInMillimeters = std::fabs(camera.depthScale - 0.001f) < 1e-6f;

	printf("D435i %s fw %s | %dx%d @ %d fps | fx=%.1f fy=%.1f cx=%.1f cy=%.1f | IMU %s\n",
			camera.profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
			camera.profile.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
			intrinsics.width, intrinsics.height, options.fps,
			intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
			options.useImu ? "on" : "off");
	return true;
}
} // namespace rtabmap_minimal
