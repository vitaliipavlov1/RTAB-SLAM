#include "realsense_capture.h"

#include <cmath>
#include <cstdio>

namespace rtabmap_minimal {
namespace {

// Noise fed to RTAB-Map with every IMU sample. The orientation term weights the
// gravity constraint; it is deliberately tighter than Optimizer/GravitySigma so
// that the graph, not the filter, decides how much gravity is trusted.
constexpr double kImuVariance = 1e-4;

// rs2_extrinsics rotation is column-major.
rtabmap::Transform fromRealSense(const rs2_extrinsics & e)
{
	return rtabmap::Transform(
		e.rotation[0], e.rotation[3], e.rotation[6], e.translation[0],
		e.rotation[1], e.rotation[4], e.rotation[7], e.translation[1],
		e.rotation[2], e.rotation[5], e.rotation[8], e.translation[2]);
}

} // namespace

RealSenseCapture::RealSenseCapture(const CaptureConfig & config) :
	config_(config),
	alignToColor_(RS2_STREAM_COLOR)
{
}

RealSenseCapture::~RealSenseCapture()
{
	stop();
}

bool RealSenseCapture::start(const rtabmap::ParametersMap & parameters)
{
	imuFilter_.reset(rtabmap::IMUFilter::create(parameters));

	rs2::config config;
	config.enable_stream(RS2_STREAM_COLOR, config_.width, config_.height, RS2_FORMAT_BGR8, config_.fps);
	config.enable_stream(RS2_STREAM_DEPTH, config_.width, config_.height, RS2_FORMAT_Z16, config_.fps);
	if(config_.useImu)
	{
		config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
		config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	}

	try
	{
		profile_ = pipe_.start(config, [this](const rs2::frame & frame){ onFrame(frame); });
	}
	catch(const rs2::error & e)
	{
		printf("Failed to start the RealSense pipeline: %s\n", e.what());
		return false;
	}
	started_ = true;

	const rs2::video_stream_profile colorProfile =
			profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	const rs2_intrinsics intrinsics = colorProfile.get_intrinsics();
	model_ = rtabmap::CameraModel(
			"d435i", intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
			rtabmap::CameraModel::opticalRotation(), 0, cv::Size(intrinsics.width, intrinsics.height));

	if(config_.useImu)
	{
		// IMU pose expressed in the same base frame as the camera model.
		imuLocalTransform_ = rtabmap::CameraModel::opticalRotation() *
				fromRealSense(profile_.get_stream(RS2_STREAM_GYRO).get_extrinsics_to(colorProfile));

		// Image and IMU stamps must share a clock, which is what global time gives us.
		for(rs2::sensor & sensor : profile_.get_device().query_sensors())
		{
			if(sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED) &&
					sensor.get_option(RS2_OPTION_GLOBAL_TIME_ENABLED) == 0.0f)
			{
				printf("Warning: global time is disabled on '%s', IMU and image stamps may not match.\n",
						sensor.get_info(RS2_CAMERA_INFO_NAME));
			}
		}
	}

	depthScale_ = profile_.get_device().first<rs2::depth_sensor>().get_depth_scale();
	depthInMillimeters_ = std::fabs(depthScale_ - 0.001f) < 1e-6f;

	printf("D435i %s fw %s | %dx%d @ %d fps | fx=%.1f fy=%.1f cx=%.1f cy=%.1f | IMU %s\n",
			profile_.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
			profile_.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
			intrinsics.width, intrinsics.height, config_.fps,
			intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
			config_.useImu ? "on" : "off");
	return true;
}

void RealSenseCapture::stop()
{
	if(!started_)
	{
		return;
	}
	started_ = false;
	try
	{
		pipe_.stop();
	}
	catch(const rs2::error & e)
	{
		printf("Camera already stopped: %s\n", e.what());
	}
}

void RealSenseCapture::onFrame(const rs2::frame & frame)
{
	if(rs2::frameset frameset = frame.as<rs2::frameset>())
	{
		const std::lock_guard<std::mutex> lock(mutex_);
		frames_ = frameset;
		hasFrames_ = true;
		return;
	}
	if(!config_.useImu)
	{
		return;
	}
	const rs2::motion_frame motion = frame.as<rs2::motion_frame>();
	if(!motion)
	{
		return;
	}
	const rs2_vector v = motion.get_motion_data();
	const std::lock_guard<std::mutex> lock(mutex_);
	if(motion.get_profile().stream_type() == RS2_STREAM_ACCEL)
	{
		accel_ = cv::Vec3d(v.x, v.y, v.z);
		hasAccel_ = true;
	}
	else if(hasAccel_)
	{
		// The filter needs both, so the first gyro sample after an accel one drives it.
		gyro_ = cv::Vec3d(v.x, v.y, v.z);
		imuFilter_->update(gyro_[0], gyro_[1], gyro_[2],
				accel_[0], accel_[1], accel_[2], motion.get_timestamp() / 1000.0);
		imuReady_ = true;
	}
}

bool RealSenseCapture::nextFrame(cv::Mat & rgb, cv::Mat & depth, double & stamp, rtabmap::IMU & imu)
{
	rs2::frameset frames;
	{
		const std::lock_guard<std::mutex> lock(mutex_);
		if(!hasFrames_)
		{
			return false;
		}
		frames = frames_;
		hasFrames_ = false;
		if(config_.useImu && imuReady_)
		{
			double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
			imuFilter_->getOrientation(qx, qy, qz, qw);
			const cv::Mat variance = cv::Mat::eye(3, 3, CV_64FC1) * kImuVariance;
			imu = rtabmap::IMU(cv::Vec4d(qx, qy, qz, qw), variance,
					gyro_, variance, accel_, variance, imuLocalTransform_);
		}
	}

	const rs2::frameset aligned = alignToColor_.process(frames);
	const rs2::video_frame colorFrame = aligned.get_color_frame();
	const rs2::depth_frame depthFrame = aligned.get_depth_frame();
	if(!colorFrame || !depthFrame)
	{
		return false;   // the very first frameset of a D435i may have no depth
	}

	rgb = cv::Mat(cv::Size(colorFrame.get_width(), colorFrame.get_height()),
			CV_8UC3, const_cast<void*>(colorFrame.get_data())).clone();
	const cv::Mat depth16 = cv::Mat(cv::Size(depthFrame.get_width(), depthFrame.get_height()),
			CV_16UC1, const_cast<void*>(depthFrame.get_data()));
	if(depthInMillimeters_)
	{
		depth = depth16.clone();   // RTAB-Map reads CV_16UC1 as millimetres
	}
	else
	{
		depth16.convertTo(depth, CV_32FC1, depthScale_);   // ... and CV_32FC1 as metres
	}
	stamp = colorFrame.get_timestamp() / 1000.0;
	return true;
}

} // namespace rtabmap_minimal
