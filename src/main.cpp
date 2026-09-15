// rtabmap_minimal - minimal working RTAB-Map SLAM demo for the Intel RealSense D435i.
//
// Pipeline:
//   D435i (RGB + Depth aligned to RGB + IMU)
//     -> rtabmap::Odometry   (visual odometry, provided by RTAB-Map)
//     -> rtabmap::Rtabmap    (memory, loop closure, graph optimization)
//     -> three live windows (camera, 3D cloud, 2D occupancy grid + pose graph)
//     -> .db + assembled point cloud (PCL) + trajectory
//
// Nothing here re-implements SLAM: features, matching, PnP, keyframes, loop
// closure and the pose graph all live inside RTAB-Map.
//
// This file keeps the orchestration only; the pieces live next to it:
//   control.h     stop/save flags and the signal handler
//   options.h     tuning constants, command line, RTAB-Map parameters, database
//   camera.h      RealSense capture and the IMU filter
//   views.h       the three windows and the HUD
//   export_map.h  saving the map as .pcd + trajectory

#include "camera.h"
#include "control.h"
#include "export_map.h"
#include "options.h"
#include "views.h"

#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <chrono>
#include <exception>
#include <memory>
#include <thread>

namespace rtabmap_minimal {

struct Session
{
	const Options & options;
	Capture & capture;
	Camera & camera;
	rtabmap::Odometry & odometry;
	rtabmap::Rtabmap & rtabmap;
	Windows & windows;
	Hud hud;
};

// Takes the newest frameset, feeds RTAB-Map, refreshes the windows. Returns
// false when there was nothing new to process.
bool processOneFrame(Session & session, rs2::align & alignToColor)
{
	rs2::frameset frames;
	rtabmap::IMU imu;
	{
		const std::lock_guard<std::mutex> lock(session.capture.mutex);
		if(session.capture.hasFrames)
		{
			frames = session.capture.frames;
			session.capture.hasFrames = false;
			if(session.options.useImu && session.capture.imuReady)
			{
				double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
				session.capture.imuFilter->getOrientation(qx, qy, qz, qw);
				const cv::Mat variance = cv::Mat::eye(3, 3, CV_64FC1) * kImuVariance;
				imu = rtabmap::IMU(
						cv::Vec4d(qx, qy, qz, qw), variance,
						session.capture.gyro, variance,
						session.capture.accel, variance,
						session.camera.imuLocalTransform);
			}
		}
	}
	if(!frames)
	{
		return false;
	}

	const rs2::frameset aligned = alignToColor.process(frames);
	const rs2::video_frame colorFrame = aligned.get_color_frame();
	const rs2::depth_frame depthFrame = aligned.get_depth_frame();
	if(!colorFrame || !depthFrame)
	{
		return false;   // the very first frameset of a D435i may have no depth
	}

	UTimer workTimer;
	const cv::Mat rgb = cv::Mat(cv::Size(colorFrame.get_width(), colorFrame.get_height()),
			CV_8UC3, const_cast<void*>(colorFrame.get_data())).clone();
	const cv::Mat depth16 = cv::Mat(cv::Size(depthFrame.get_width(), depthFrame.get_height()),
			CV_16UC1, const_cast<void*>(depthFrame.get_data()));
	cv::Mat depth;
	if(session.camera.depthInMillimeters)
	{
		depth = depth16.clone();   // RTAB-Map reads CV_16UC1 as millimeters
	}
	else
	{
		depth16.convertTo(depth, CV_32FC1, session.camera.depthScale);   // ... and CV_32FC1 as meters
	}

	rtabmap::SensorData data(rgb, depth, session.camera.model,
			++session.hud.frames, colorFrame.get_timestamp() / 1000.0);
	if(!imu.empty())
	{
		data.setIMU(imu);
	}

	// 1) odometry (RTAB-Map)
	rtabmap::OdometryInfo info;
	const rtabmap::Transform pose = session.odometry.process(data, &info);

	// 2) mapping + loop closure (RTAB-Map). It throttles itself to Rtabmap/DetectionRate.
	if(pose.isNull())
	{
		++session.hud.lost;
	}
	else if(session.rtabmap.process(data, pose, info.reg.covariance))
	{
		session.hud.mapNodes = static_cast<int>(session.rtabmap.getLocalOptimizedPoses().size());
		if(session.windows.map3d)
		{
			updateLiveCloud(session.windows, session.rtabmap, data, pose);
		}
		if(session.windows.map2d)
		{
			updateGraphView(session.windows, session.rtabmap);
		}
		// Loop/Id covers both appearance-based loop closure and proximity detection.
		const int loopId = static_cast<int>(uValue(session.rtabmap.getStatistics().data(),
				rtabmap::Statistics::kLoopId(), 0.0f));
		if(loopId > 0)
		{
			session.hud.lastLoopId = loopId;
			++session.hud.loopClosures;
			printf("Loop closure / proximity link with node %d (total %d)\n",
					loopId, session.hud.loopClosures);
		}
	}

	session.hud.frameMs = workTimer.elapsed() * 1000.0;

	if(session.windows.camera)
	{
		session.windows.camera->setImage(uCvMat2QImage(renderHud(rgb, pose, info, session.hud)));
	}
	if(session.windows.map3d && !pose.isNull())
	{
		session.windows.map3d->updateCameraTargetPosition(pose);   // also grows the trajectory
	}
	if(session.windows.map2d && !pose.isNull())
	{
		session.windows.map2d->updateReferentialPosition(pose);
	}
	return true;
}

// Wall time between two processed frames, which is the rate the user actually
// gets - capture and alignment included, not just the SLAM part.
void updateFps(Hud & hud, double period)
{
	if(period <= 0.0)
	{
		return;
	}
	const double instant = 1.0 / period;
	hud.fps = hud.fps == 0.0 ? instant : (1.0 - kFpsSmoothing) * hud.fps + kFpsSmoothing * instant;
}

void runLoop(Session & session)
{
	rs2::align alignToColor(RS2_STREAM_COLOR);
	UTimer loopTimer, viewerTimer, statusTimer;

	while(!g_stop)
	{
		if(!processOneFrame(session, alignToColor))
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			continue;
		}
		updateFps(session.hud, loopTimer.restart());

		if(session.windows.map3d && viewerTimer.elapsed() > kViewerPeriod)
		{
			viewerTimer.restart();
			session.windows.map3d->refreshView();
		}
		if(session.windows.app)
		{
			session.windows.app->processEvents();
			if(!session.windows.anyVisible())
			{
				g_stop = true;   // every window closed
			}
		}

		if(g_save)
		{
			g_save = false;
			exportMap(session.rtabmap, session.options);
		}
		else if(statusTimer.elapsed() > kStatusPeriod)
		{
			statusTimer.restart();
			printf("frames %d (lost %d) | %.1f Hz | nodes %d | loops %d\n",
					session.hud.frames, session.hud.lost, session.hud.fps,
					session.hud.mapNodes, session.hud.loopClosures);
		}
	}
}
} // namespace rtabmap_minimal

int main(int argc, char ** argv)
{
	using namespace rtabmap_minimal;

	Options options;
	switch(parseArgs(argc, argv, options))
	{
		case ParseResult::kHelp: return 0;
		case ParseResult::kError: return 1;
		case ParseResult::kOk: break;
	}

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	const rtabmap::ParametersMap parameters = loadParameters(options);
	printBackends(parameters);

	Capture capture;
	capture.imuFilter.reset(rtabmap::IMUFilter::create(parameters));
	Camera camera;
	if(!startCamera(options, capture, camera))
	{
		return 1;
	}

	std::unique_ptr<rtabmap::Odometry> odometry(rtabmap::Odometry::create(parameters));
	rtabmap::Rtabmap rtabmap;
	prepareDatabase(options);
	rtabmap.init(parameters, options.dbPath);

	Windows windows;
	createWindows(options, argc, argv, parameters, windows);

	installSignalHandler();
	printf("\nRunning. Move the camera slowly. [q] quit and save, [s] save now.\n\n");

	Session session{options, capture, camera, *odometry, rtabmap, windows, Hud()};
	int exitCode = 0;
	try
	{
		runLoop(session);
	}
	catch(const std::exception & e)
	{
		// Do not lose the map: fall through to the shutdown path below.
		printf("\nAborted: %s\n", e.what());
		exitCode = 1;
	}
	catch(...)
	{
		printf("\nAborted: unknown error\n");
		exitCode = 1;
	}

	printf("\nStopping...\n");
	try
	{
		camera.pipe.stop();
	}
	catch(const rs2::error & e)
	{
		printf("Camera already stopped: %s\n", e.what());
	}
	exportMap(rtabmap, options);
	rtabmap.close(true);
	printf("database saved          : %s\n", options.dbPath.c_str());
	printf("inspect it with         : rtabmap-databaseViewer %s\n", options.dbPath.c_str());
	return exitCode;
}
