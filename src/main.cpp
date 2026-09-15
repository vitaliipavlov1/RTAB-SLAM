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

#include <librealsense2/rs.hpp>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>
#include <rtabmap/core/util3d.h>

#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/gui/GraphViewer.h>
#include <rtabmap/gui/ImageView.h>

#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UCv2Qt.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <QApplication>
#include <QRect>
#include <QScreen>
#include <QShortcut>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <exception>
#include <string>
#include <thread>

namespace {

// ---------------------------------------------------------------- constants

// Live 3D map: one cloud per map node, kept cheap on purpose.
constexpr int kLiveDecimation = 8;
constexpr float kLiveMaxDepth = 4.0f;

// Saved map: denser than the live view, still bounded.
constexpr int kExportDecimation = 4;
constexpr float kExportMaxDepth = 4.0f;
constexpr float kExportVoxelSize = 0.03f;

// Depth closer than this is unreliable on the D435i.
constexpr float kMinDepth = 0.3f;

constexpr double kViewerPeriod = 0.2;   // s, how often the map windows repaint
constexpr double kStatusPeriod = 2.0;   // s, console status line in --no-gui
constexpr double kFpsSmoothing = 0.1;   // exponential moving average weight
constexpr unsigned int kTrajectorySize = 10000;

// Occupancy grid palette: the usual SLAM look.
constexpr unsigned char kGridFree = 255;
constexpr unsigned char kGridOccupied = 0;
constexpr unsigned char kGridUnknown = 128;

// IMU noise fed to RTAB-Map. The orientation term is what weights the gravity
// constraint; it is deliberately tighter than Optimizer/GravitySigma so the
// graph, not the filter, decides how much gravity is trusted.
constexpr double kImuVariance = 1e-4;

static_assert(std::atomic<bool>::is_always_lock_free,
		"the signal handler below may only touch lock-free atomics");
std::atomic<bool> g_stop(false);
std::atomic<bool> g_save(false);

void onSignal(int) { g_stop = true; }

// ------------------------------------------------------------- command line

struct Options
{
	std::string configPath = "config/rtabmap_minimal.ini";
	std::string dbPath = "rtabmap_minimal.db";
	std::string cloudPath = "rtabmap_minimal_cloud.pcd";
	std::string trajectoryPath = "rtabmap_minimal_trajectory.txt";
	bool useImu = true;
	bool cameraWindow = true;
	bool map3dWindow = true;
	bool map2dWindow = true;
	bool continueMap = false;   // keep mapping an existing database
	int width = 640;
	int height = 480;
	int fps = 15;

	bool anyWindow() const { return cameraWindow || map3dWindow || map2dWindow; }
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
		else if(arg == "--db" && i + 1 < argc) options.dbPath = argv[++i];
		else if(arg == "--cloud" && i + 1 < argc) options.cloudPath = argv[++i];
		else if(arg == "--continue") options.continueMap = true;
		else if(arg == "--no-imu") options.useImu = false;
		else if(arg == "--no-gui") options.cameraWindow = options.map3dWindow = options.map2dWindow = false;
		else if(arg == "--no-view") options.cameraWindow = false;
		else if(arg == "--no-map3d") options.map3dWindow = false;
		else if(arg == "--no-map2d") options.map2dWindow = false;
		else if(arg == "--fps" && i + 1 < argc) options.fps = atoi(argv[++i]);
		else if(arg == "--size" && i + 2 < argc)
		{
			options.width = atoi(argv[++i]);
			options.height = atoi(argv[++i]);
		}
		else
		{
			printUsage(argv[0]);
			return arg == "--help" || arg == "-h" ? ParseResult::kHelp : ParseResult::kError;
		}
	}

	// The camera throws a rather cryptic error for impossible modes, so check here.
	if(options.width < 160 || options.height < 120 || options.width > 1920 || options.height > 1080)
	{
		printf("Unsupported resolution %dx%d.\n", options.width, options.height);
		return ParseResult::kError;
	}
	if(options.fps < 5 || options.fps > 90)
	{
		printf("Unsupported frame rate %d fps.\n", options.fps);
		return ParseResult::kError;
	}
	return ParseResult::kOk;
}

// -------------------------------------------------------------- parameters

rtabmap::ParametersMap loadParameters(const Options & options)
{
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	if(UFile::exists(options.configPath))
	{
		rtabmap::Parameters::readINI(options.configPath, parameters);
		printf("Parameters loaded from %s\n", options.configPath.c_str());
	}
	else
	{
		printf("No config file at %s, using RTAB-Map defaults.\n", options.configPath.c_str());
	}

	if(!options.useImu)
	{
		// Gravity constraints make sense only when IMU orientation is fed in.
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerGravitySigma(), "0"));
	}
	if(!options.map2dWindow)
	{
		// Nobody looks at the local occupancy grids, so do not pay for them.
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kRGBDCreateOccupancyGrid(), "false"));
	}
	else
	{
		// The 2D window is fed from the published statistics of the last node.
		for(const auto & key : {rtabmap::Parameters::kRtabmapPublishStats(),
				rtabmap::Parameters::kRtabmapPublishLastSignature()})
		{
			if(uStr2Bool(uValue(parameters, key, std::string("true"))) == false)
			{
				printf("Warning: %s=false, the 2D map window will stay empty.\n", key.c_str());
			}
		}
	}
	return parameters;
}

void printBackends(const rtabmap::ParametersMap & parameters)
{
	printf("RTAB-Map %s | optimizers: TORO=%d g2o=%d GTSAM=%d Ceres=%d | using strategy %s\n",
			RTABMAP_VERSION,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeTORO) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeG2O) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeGTSAM) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeCeres) ? 1 : 0,
			uValue(parameters, rtabmap::Parameters::kOptimizerStrategy(), std::string("?")).c_str());
}

// ------------------------------------------------------------------ capture

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
rtabmap::Transform fromRealSense(const rs2_extrinsics & e)
{
	return rtabmap::Transform(
		e.rotation[0], e.rotation[3], e.rotation[6], e.translation[0],
		e.rotation[1], e.rotation[4], e.rotation[7], e.translation[1],
		e.rotation[2], e.rotation[5], e.rotation[8], e.translation[2]);
}

// Video framesets and IMU samples arrive on the same callback thread; the main
// loop always takes the newest frameset and drops the rest (weak CPU friendly).
bool startCamera(const Options & options, Capture & capture, Camera & camera)
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

// ------------------------------------------------------------------ windows

// All three windows are RTAB-Map's own Qt widgets, the same ones that
// rtabmap-databaseViewer is built from. PCLVisualizer is not an option here:
// its X11 interactor segfaults on the first spinOnce() with VTK 9.1, which
// reproduces in a 30-line program without RTAB-Map.
struct Windows
{
	std::unique_ptr<QApplication> app;
	std::unique_ptr<rtabmap::ImageView> camera;
	std::unique_ptr<rtabmap::CloudViewer> map3d;
	std::unique_ptr<rtabmap::GraphViewer> map2d;

	rtabmap::LocalGridCache gridCache;              // must outlive occupancyGrid
	std::unique_ptr<rtabmap::OccupancyGrid> occupancyGrid;
	std::map<int, std::string> shownClouds;         // map node id -> cloud id

	bool anyVisible() const
	{
		return (camera && camera->isVisible()) ||
			(map3d && map3d->isVisible()) ||
			(map2d && map2d->isVisible());
	}
};

// Put a window so that its *frame* occupies exactly the given rectangle, so the
// three windows touch each other without overlapping.
void placeWindow(QWidget * widget, const QRect & target)
{
	widget->setGeometry(target);
	widget->show();
	// The window manager reports the decoration size only once the window is
	// mapped, so let it answer before correcting for the frame.
	for(int i = 0; i < 20 && widget->frameGeometry() == widget->geometry(); ++i)
	{
		QApplication::processEvents();
	}
	const QRect frame = widget->frameGeometry();
	const QRect inner = widget->geometry();
	widget->setGeometry(
			target.x() + (inner.x() - frame.x()),
			target.y() + (inner.y() - frame.y()),
			target.width() - (frame.width() - inner.width()),
			target.height() - (frame.height() - inner.height()));
}

void createWindows(const Options & options, int & argc, char ** argv,
		const rtabmap::ParametersMap & parameters, Windows & windows)
{
	if(!options.anyWindow())
	{
		return;
	}
	windows.app = std::make_unique<QApplication>(argc, argv);

	// Layout: camera top-left, 2D map top-right, 3D map centred below them.
	QRect screen(0, 0, 1280, 720);
	if(QApplication::primaryScreen())
	{
		screen = QApplication::primaryScreen()->availableGeometry();
	}
	const int halfW = screen.width() / 2;
	const int halfH = screen.height() / 2;

	if(options.cameraWindow)
	{
		windows.camera = std::make_unique<rtabmap::ImageView>();
		windows.camera->setWindowTitle("rtabmap_minimal - camera");
		windows.camera->setBackgroundColor(QColor(30, 30, 30));
		placeWindow(windows.camera.get(), QRect(screen.x(), screen.y(), halfW, halfH));
	}
	if(options.map2dWindow)
	{
		windows.map2d = std::make_unique<rtabmap::GraphViewer>();
		windows.map2d->setWindowTitle("rtabmap_minimal - 2D map");
		windows.map2d->setGridMapVisible(true);
		windows.map2d->setNodeColor(QColor(0, 0, 255));          // trajectory nodes
		windows.map2d->setNeighborColor(QColor(0, 0, 255));      // odometry links
		windows.map2d->setNeighborMergedColor(QColor(0, 0, 255));
		windows.map2d->setGlobalLoopClosureColor(QColor(255, 0, 0));
		windows.map2d->setLocalLoopClosureColor(QColor(255, 0, 0));
		windows.map2d->setNodeRadius(0.04f);
		windows.map2d->setLinkWidth(0.02f);
		windows.map2d->setEnsureFrameVisible(true);
		windows.map2d->setBackgroundBrush(QBrush(QColor(kGridUnknown, kGridUnknown, kGridUnknown)));
		placeWindow(windows.map2d.get(), QRect(screen.x() + halfW, screen.y(),
				screen.width() - halfW, halfH));
		windows.occupancyGrid = std::make_unique<rtabmap::OccupancyGrid>(&windows.gridCache, parameters);
	}
	if(options.map3dWindow)
	{
		windows.map3d = std::make_unique<rtabmap::CloudViewer>();
		windows.map3d->setWindowTitle("rtabmap_minimal - map");
		windows.map3d->setBackgroundColor(QColor(30, 30, 30));
		windows.map3d->setGridShown(true);
		windows.map3d->setTrajectorySize(kTrajectorySize);
		placeWindow(windows.map3d.get(), QRect(screen.x() + halfW / 2, screen.y() + halfH,
				halfW, screen.height() - halfH));
	}

	for(QWidget * widget : {static_cast<QWidget*>(windows.camera.get()),
			static_cast<QWidget*>(windows.map2d.get()),
			static_cast<QWidget*>(windows.map3d.get())})
	{
		if(widget)
		{
			QObject::connect(new QShortcut(QKeySequence("s"), widget), &QShortcut::activated,
					[](){ g_save = true; });
			QObject::connect(new QShortcut(QKeySequence("q"), widget), &QShortcut::activated,
					[](){ g_stop = true; });
		}
	}
}

// ----------------------------------------------------------- live map views

// One cloud per map node. Loop closure moves old nodes: the clouds are re-posed,
// never rebuilt. Nodes that RTAB-Map moved out of working memory are dropped
// here as well, which is what keeps this bounded on a long session.
void updateLiveCloud(Windows & windows, const rtabmap::Rtabmap & rtabmap,
		const rtabmap::SensorData & data, const rtabmap::Transform & odomPose)
{
	const int nodeId = rtabmap.getLastLocationId();
	if(nodeId > 0 && windows.shownClouds.find(nodeId) == windows.shownClouds.end())
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
				rtabmap::util3d::cloudRGBFromSensorData(data, kLiveDecimation, kLiveMaxDepth, kMinDepth);
		if(!cloud->empty())
		{
			const std::string cloudId = uFormat("node%d", nodeId);
			windows.map3d->addCloud(cloudId, cloud, odomPose);
			windows.shownClouds.insert(std::make_pair(nodeId, cloudId));
		}
	}

	const std::map<int, rtabmap::Transform> & optimized = rtabmap.getLocalOptimizedPoses();
	for(auto iter = windows.shownClouds.begin(); iter != windows.shownClouds.end(); )
	{
		const auto jter = optimized.find(iter->first);
		if(jter == optimized.end())
		{
			windows.map3d->removeCloud(iter->second);
			iter = windows.shownClouds.erase(iter);
		}
		else
		{
			if(!jter->second.isNull())
			{
				windows.map3d->updateCloudPose(iter->second, jter->second);
			}
			++iter;
		}
	}
}

// RTAB-Map's occupancy grid (-1 unknown, 0 empty, 100 occupied) as a grey image:
// white = free, black = obstacle, grey = unknown.
cv::Mat gridToImage(const cv::Mat & map8S)
{
	cv::Mat image(map8S.size(), CV_8UC1, cv::Scalar(kGridUnknown));
	for(int y = 0; y < map8S.rows; ++y)
	{
		const char * src = map8S.ptr<char>(y);
		unsigned char * dst = image.ptr<unsigned char>(y);
		for(int x = 0; x < map8S.cols; ++x)
		{
			dst[x] = src[x] == 0 ? kGridFree : (src[x] == 100 ? kGridOccupied : kGridUnknown);
		}
	}
	return image;
}

void updateGraphView(Windows & windows, const rtabmap::Rtabmap & rtabmap)
{
	const rtabmap::Statistics & stats = rtabmap.getStatistics();
	const rtabmap::Signature & last = stats.getLastSignatureData();
	if(last.id() > 0 && last.sensorData().gridCellSize() > 0.0f)
	{
		// Local occupancy grids are computed by RTAB-Map itself
		// (RGBD/CreateOccupancyGrid), we only assemble and show them.
		windows.gridCache.add(last.id(),
				last.sensorData().gridGroundCellsRaw(),
				last.sensorData().gridObstacleCellsRaw(),
				last.sensorData().gridEmptyCellsRaw(),
				last.sensorData().gridCellSize(),
				last.sensorData().gridViewPoint());
	}

	const std::map<int, rtabmap::Transform> & poses =
			stats.poses().empty() ? rtabmap.getLocalOptimizedPoses() : stats.poses();
	if(windows.occupancyGrid->update(poses))
	{
		float xMin = 0.0f, yMin = 0.0f;
		const cv::Mat map8S = windows.occupancyGrid->getMap(xMin, yMin);
		if(!map8S.empty())
		{
			windows.map2d->updateMap(gridToImage(map8S), windows.occupancyGrid->getCellSize(), xMin, yMin);
		}
	}

	std::map<int, int> mapIds;   // single session: every node belongs to map 0
	for(const auto & [nodeId, pose] : poses)
	{
		(void)pose;
		mapIds.insert(mapIds.end(), std::make_pair(nodeId, 0));
	}
	windows.map2d->updateGraph(poses, stats.constraints(), mapIds);
}

// ---------------------------------------------------------------------- HUD

struct Hud
{
	int frames = 0;
	int lost = 0;
	int mapNodes = 0;
	int loopClosures = 0;
	int lastLoopId = 0;
	double fps = 0.0;
	double frameMs = 0.0;
};

void putLine(cv::Mat & image, const std::string & text, int line)
{
	const cv::Point at(8, 20 + 18 * line);
	cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
	cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

// Draws on a copy: the original frame is still referenced by the SensorData we
// handed to RTAB-Map, and the map must not get the overlay burned into it.
cv::Mat renderHud(const cv::Mat & rgb, const rtabmap::Transform & pose,
		const rtabmap::OdometryInfo & info, const Hud & hud)
{
	cv::Mat display = rgb.clone();
	float x = 0.0f, y = 0.0f, z = 0.0f;
	if(!pose.isNull())
	{
		pose.getTranslation(x, y, z);
	}
	putLine(display, uFormat("%s  features %d  inliers %d",
			pose.isNull() ? "LOST" : "TRACKING", info.features, info.reg.inliers), 0);
	putLine(display, uFormat("frames %d (lost %d)  %.1f Hz  %.0f ms/frame",
			hud.frames, hud.lost, hud.fps, hud.frameMs), 1);
	putLine(display, uFormat("map nodes %d  loop closures %d  last id %d",
			hud.mapNodes, hud.loopClosures, hud.lastLoopId), 2);
	putLine(display, uFormat("xyz %.2f %.2f %.2f", x, y, z), 3);
	for(const auto & [wordId, keypoint] : info.words)
	{
		(void)wordId;
		cv::circle(display, keypoint.pt, 2, cv::Scalar(0, 255, 0), -1);
	}
	return display;
}

// ------------------------------------------------------------------- export

// Assemble the optimized map as a PCL cloud, filter it, print statistics, save it.
// Note: the whole graph is materialized in memory, which is fine for a demo-sized
// map; for a long session use rtabmap-export on the .db instead.
void exportMap(rtabmap::Rtabmap & rtabmap, const Options & options)
{
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	std::map<int, rtabmap::Signature> signatures;
	rtabmap.getGraph(poses, links, true /*optimized*/, true /*global*/, &signatures, true /*images*/);

	if(poses.empty())
	{
		printf("Map is empty, nothing to export.\n");
		return;
	}

	std::map<int, double> stamps;
	for(const auto & [nodeId, signature] : signatures)
	{
		stamps.insert(std::make_pair(nodeId, signature.getStamp()));
	}
	// Format 11 = "id x y z qx qy qz qw" (RGBD-SLAM style, needs a stamp per pose).
	rtabmap::graph::exportPoses(options.trajectoryPath,
			stamps.size() == poses.size() ? 11 : 0, poses, links, stamps);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembled(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(const auto & [nodeId, pose] : poses)
	{
		const auto signature = signatures.find(nodeId);
		if(signature == signatures.end() || pose.isNull())
		{
			continue;
		}
		rtabmap::SensorData data = signature->second.sensorData();
		data.uncompressData();
		if(data.imageRaw().empty() || data.depthRaw().empty())
		{
			continue;
		}
		// RGB-D -> point cloud (RTAB-Map's own converter, returns a PCL cloud in base frame)
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
				rtabmap::util3d::cloudRGBFromSensorData(data, kExportDecimation, kExportMaxDepth, kMinDepth);
		if(cloud->empty())
		{
			continue;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud, *transformed, pose.toEigen4f());
		*assembled += *transformed;
	}

	if(assembled->empty())
	{
		printf("Assembled cloud is empty (no depth data kept in the map).\n");
		return;
	}

	const std::size_t rawSize = assembled->size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
	voxel.setInputCloud(assembled);
	voxel.setLeafSize(kExportVoxelSize, kExportVoxelSize, kExportVoxelSize);
	voxel.filter(*filtered);

	pcl::PointXYZRGB min, max;
	pcl::getMinMax3D(*filtered, min, max);

	filtered->is_dense = false;
	pcl::io::savePCDFileBinary(options.cloudPath, *filtered);

	printf("\n--- map ---\n");
	printf("nodes (optimized poses) : %d\n", static_cast<int>(poses.size()));
	printf("links (graph edges)     : %d\n", static_cast<int>(links.size()));
	printf("cloud points            : %d raw -> %d after %.0f cm voxel filter\n",
			static_cast<int>(rawSize), static_cast<int>(filtered->size()), kExportVoxelSize * 100.0f);
	printf("bounding box [m]        : x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]\n",
			min.x, max.x, min.y, max.y, min.z, max.z);
	printf("cloud saved             : %s\n", options.cloudPath.c_str());
	printf("trajectory saved        : %s\n", options.trajectoryPath.c_str());
}

// --------------------------------------------------------------- main loop

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

void prepareDatabase(const Options & options)
{
	if(!UFile::exists(options.dbPath))
	{
		return;
	}
	if(options.continueMap)
	{
		printf("Continuing the existing map in %s (a new session is appended).\n", options.dbPath.c_str());
	}
	else
	{
		printf("Replacing the existing database %s (pass --continue to keep mapping it).\n",
				options.dbPath.c_str());
		UFile::erase(options.dbPath);
	}
}

void installSignalHandler()
{
	struct sigaction action;
	std::memset(&action, 0, sizeof(action));
	action.sa_handler = onSignal;
	sigaction(SIGINT, &action, nullptr);
	sigaction(SIGTERM, &action, nullptr);
}

} // namespace

int main(int argc, char ** argv)
{
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
