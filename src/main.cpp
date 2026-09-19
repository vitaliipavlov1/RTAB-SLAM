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
// This file parses the command line and runs the loop; the parts it wires
// together are independent modules:
//   realsense_capture.h  the camera: newest RGB-D frame + IMU orientation
//   rtabmap_app.h        RTAB-Map: parameters, database, odometry, mapping
//   viewer.h             the three windows and the HUD
//   map_export.h         saving the map as .pcd + trajectory

#include "map_export.h"
#include "realsense_capture.h"
#include "rtabmap_app.h"
#include "viewer.h"

#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <thread>

namespace rtabmap_minimal {
namespace {

constexpr double kStatusPeriod = 2.0;   // s, console status line
constexpr double kFpsSmoothing = 0.1;   // exponential moving average weight

static_assert(std::atomic<bool>::is_always_lock_free,
		"the signal handler below may only touch lock-free atomics");
std::atomic<bool> g_stop(false);

void onSignal(int) { g_stop = true; }

void installSignalHandler()
{
	struct sigaction action;
	std::memset(&action, 0, sizeof(action));
	action.sa_handler = onSignal;
	sigaction(SIGINT, &action, nullptr);
	sigaction(SIGTERM, &action, nullptr);
}

struct Options
{
	std::string configPath = "config/rtabmap_minimal.ini";
	CaptureConfig capture;
	AppConfig app;
	ViewerConfig viewer;
	ExportPaths paths;
};

enum class ParseResult { kOk, kHelp, kError };

void printUsage(const char * program)
{
	printf("Usage: %s [--config f.ini] [--db out.db] [--cloud out.pcd] [--continue]\n"
	       "          [--no-imu] [--no-gui] [--no-view] [--no-map3d] [--no-map2d]\n"
	       "          [--fps 15] [--size 640 480]\n", program);
}

ParseResult parseArgs(int argc, char ** argv, Options & options)
{
	for(int i = 1; i < argc; ++i)
	{
		const std::string arg = argv[i];
		if(arg == "--config" && i + 1 < argc) options.configPath = argv[++i];
		else if(arg == "--db" && i + 1 < argc) options.app.databasePath = argv[++i];
		else if(arg == "--cloud" && i + 1 < argc) options.paths.cloudPath = argv[++i];
		else if(arg == "--continue") options.app.continueMapping = true;
		else if(arg == "--no-imu") options.capture.useImu = false;
		else if(arg == "--no-gui") options.viewer = ViewerConfig{false, false, false};
		else if(arg == "--no-view") options.viewer.camera = false;
		else if(arg == "--no-map3d") options.viewer.map3d = false;
		else if(arg == "--no-map2d") options.viewer.map2d = false;
		else if(arg == "--fps" && i + 1 < argc) options.capture.fps = atoi(argv[++i]);
		else if(arg == "--size" && i + 2 < argc)
		{
			options.capture.width = atoi(argv[++i]);
			options.capture.height = atoi(argv[++i]);
		}
		else
		{
			printUsage(argv[0]);
			return arg == "--help" || arg == "-h" ? ParseResult::kHelp : ParseResult::kError;
		}
	}

	// The camera throws a rather cryptic error for impossible modes, so check here.
	const CaptureConfig & capture = options.capture;
	if(capture.width < 160 || capture.height < 120 || capture.width > 1920 || capture.height > 1080)
	{
		printf("Unsupported resolution %dx%d.\n", capture.width, capture.height);
		return ParseResult::kError;
	}
	if(capture.fps < 5 || capture.fps > 90)
	{
		printf("Unsupported frame rate %d fps.\n", capture.fps);
		return ParseResult::kError;
	}
	return ParseResult::kOk;
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

void runLoop(const Options & options, RealSenseCapture & capture, RtabmapApp & app, Viewer & viewer)
{
	Hud hud;
	UTimer loopTimer, workTimer, statusTimer;

	while(!g_stop && !viewer.quitRequested())
	{
		cv::Mat rgb, depth;
		double stamp = 0.0;
		rtabmap::IMU imu;
		if(!capture.nextFrame(rgb, depth, stamp, imu))
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			continue;
		}

		workTimer.restart();
		rtabmap::SensorData data(rgb, depth, capture.model(), ++hud.frames, stamp);
		if(!imu.empty())
		{
			data.setIMU(imu);
		}

		const FrameResult result = app.process(data);
		if(result.pose.isNull())
		{
			++hud.lost;
		}
		if(result.nodeAdded)
		{
			hud.mapNodes = app.mapNodes();
			viewer.onNewNode(app.rtabmap(), data, result.pose);
			if(result.loopId > 0)
			{
				hud.lastLoopId = result.loopId;
				++hud.loopClosures;
				printf("Loop closure / proximity link with node %d (total %d)\n",
						result.loopId, hud.loopClosures);
			}
		}

		hud.frameMs = workTimer.elapsed() * 1000.0;
		updateFps(hud, loopTimer.restart());

		viewer.showFrame(rgb, result.pose, result.odometry, hud);
		// The map windows draw the optimized graph, while odometry keeps running in
		// its own frame: every loop closure moves one away from the other. The map
		// correction is what RTAB-Map uses to express an odometry pose in the map,
		// without it the position marker drifts off the map it belongs to.
		viewer.followCamera(app.rtabmap().getMapCorrection() * result.pose);
		viewer.processEvents();

		if(viewer.takeSaveRequest())
		{
			exportMap(app.rtabmap(), options.paths);
		}
		else if(statusTimer.elapsed() > kStatusPeriod)
		{
			statusTimer.restart();
			printf("frames %d (lost %d) | %.1f Hz | nodes %d | loops %d\n",
					hud.frames, hud.lost, hud.fps, hud.mapNodes, hud.loopClosures);
		}
	}
}

} // namespace
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

	const rtabmap::ParametersMap parameters =
			loadParameters(options.configPath, options.capture.useImu, options.viewer.map2d);
	printBackends(parameters);

	RealSenseCapture capture(options.capture);
	if(!capture.start(parameters))
	{
		return 1;
	}

	RtabmapApp app(parameters, options.app);
	Viewer viewer(options.viewer, parameters, argc, argv);

	installSignalHandler();
	printf("\nRunning. Move the camera slowly. [q] quit and save, [s] save now.\n\n");

	int exitCode = 0;
	try
	{
		runLoop(options, capture, app, viewer);
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
	capture.stop();
	exportMap(app.rtabmap(), options.paths);
	app.close();
	printf("database saved          : %s\n", options.app.databasePath.c_str());
	printf("inspect it with         : rtabmap-databaseViewer %s\n", options.app.databasePath.c_str());
	return exitCode;
}
