#pragma once

// Intel RealSense D435i capture.
//
// Owns the pipeline, the depth-to-colour alignment and RTAB-Map's Madgwick IMU
// filter, and hands the main loop ready-made RGB-D frames. Only the newest
// frameset is kept: on a slow machine the older ones are dropped instead of
// piling up latency.

#include <librealsense2/rs.hpp>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>

#include <opencv2/core.hpp>

#include <memory>
#include <mutex>

namespace rtabmap_minimal {

struct CaptureConfig
{
	int width = 640;
	int height = 480;
	int fps = 15;
	bool useImu = true;
};

class RealSenseCapture
{
public:
	explicit RealSenseCapture(const CaptureConfig & config);
	~RealSenseCapture();

	RealSenseCapture(const RealSenseCapture &) = delete;
	RealSenseCapture & operator=(const RealSenseCapture &) = delete;

	// Starts streaming; the parameters configure RTAB-Map's IMU filter.
	// Prints what the camera reports and returns false if it cannot start.
	bool start(const rtabmap::ParametersMap & parameters);
	void stop();

	// Newest frame, aligned to colour. Returns false when nothing new arrived:
	// depth is CV_16UC1 in millimetres or CV_32FC1 in metres, whichever the
	// camera's depth scale calls for, and imu is empty when IMU is off.
	bool nextFrame(cv::Mat & rgb, cv::Mat & depth, double & stamp, rtabmap::IMU & imu);

	const rtabmap::CameraModel & model() const { return model_; }

private:
	void onFrame(const rs2::frame & frame);   // runs on the librealsense thread

	CaptureConfig config_;
	rs2::pipeline pipe_;
	rs2::pipeline_profile profile_;
	rs2::align alignToColor_;
	rtabmap::CameraModel model_;
	rtabmap::Transform imuLocalTransform_;
	float depthScale_ = 0.001f;
	bool depthInMillimeters_ = true;
	bool started_ = false;

	std::mutex mutex_;
	rs2::frameset frames_;
	bool hasFrames_ = false;
	std::unique_ptr<rtabmap::IMUFilter> imuFilter_;
	cv::Vec3d gyro_;
	cv::Vec3d accel_;
	bool hasAccel_ = false;
	bool imuReady_ = false;
};

} // namespace rtabmap_minimal
