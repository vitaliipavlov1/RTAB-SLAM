// rtabmap_minimal - minimal working RTAB-Map SLAM demo for the Intel RealSense D435i.
//
// Pipeline:
//   D435i (RGB + Depth aligned to RGB + IMU)
//     -> rtabmap::Odometry   (visual odometry, provided by RTAB-Map)
//     -> rtabmap::Rtabmap    (memory, loop closure, graph optimization)
//     -> .db + assembled point cloud (PCL) + trajectory
//
// Nothing here re-implements SLAM: features, matching, PnP, keyframes, loop
// closure and the pose graph all live inside RTAB-Map.

#include <librealsense2/rs.hpp>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/gui/ImageView.h>

#include <QApplication>
#include <QShortcut>

#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UCv2Qt.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/imgproc.hpp>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_stop(false);
std::atomic<bool> g_save(false);
void onSignal(int) { g_stop = true; }


// rs2_extrinsics rotation is column-major.
rtabmap::Transform fromRealSense(const rs2_extrinsics & e)
{
	return rtabmap::Transform(
		e.rotation[0], e.rotation[3], e.rotation[6], e.translation[0],
		e.rotation[1], e.rotation[4], e.rotation[7], e.translation[1],
		e.rotation[2], e.rotation[5], e.rotation[8], e.translation[2]);
}

// Shared between the RealSense callback thread and the main loop.
struct Capture
{
	std::mutex mutex;
	rs2::frameset frames;
	bool hasFrames = false;

	rtabmap::IMUFilter * imuFilter = 0;   // Madgwick filter from RTAB-Map
	cv::Vec3d gyro, accel;
	bool hasAccel = false;
	bool imuReady = false;
};

void putLine(cv::Mat & img, const std::string & text, int line)
{
	const cv::Point p(8, 20 + 18 * line);
	cv::putText(img, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
	cv::putText(img, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

// Assemble the optimized map as a PCL cloud, filter it, print statistics, save it.
void exportMap(
		rtabmap::Rtabmap & rtabmap,
		const std::string & cloudPath,
		const std::string & trajectoryPath,
		int decimation,
		float maxDepth,
		float voxelSize)
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
	for(std::map<int, rtabmap::Signature>::const_iterator iter = signatures.begin(); iter != signatures.end(); ++iter)
	{
		stamps.insert(std::make_pair(iter->first, iter->second.getStamp()));
	}
	// Format 11 = "id x y z qx qy qz qw" (RGBD-SLAM style, needs a stamp per pose).
	rtabmap::graph::exportPoses(trajectoryPath,
			stamps.size() == poses.size() ? 11 : 0, poses, links, stamps);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembled(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
	{
		std::map<int, rtabmap::Signature>::iterator jter = signatures.find(iter->first);
		if(jter == signatures.end() || iter->second.isNull())
		{
			continue;
		}
		rtabmap::SensorData data = jter->second.sensorData();
		data.uncompressData();
		if(data.imageRaw().empty() || data.depthRaw().empty())
		{
			continue;
		}
		// RGB-D -> point cloud (RTAB-Map's own converter, returns a PCL cloud in base frame)
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
			rtabmap::util3d::cloudRGBFromSensorData(data, decimation, maxDepth, 0.3f);
		if(cloud->empty())
		{
			continue;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud, *transformed, iter->second.toEigen4f());
		*assembled += *transformed;
	}

	if(assembled->empty())
	{
		printf("Assembled cloud is empty (no depth data kept in the map).\n");
		return;
	}

	const size_t rawSize = assembled->size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
	voxel.setInputCloud(assembled);
	voxel.setLeafSize(voxelSize, voxelSize, voxelSize);
	voxel.filter(*filtered);

	pcl::PointXYZRGB min, max;
	pcl::getMinMax3D(*filtered, min, max);

	filtered->is_dense = false;
	pcl::io::savePCDFileBinary(cloudPath, *filtered);

	printf("\n--- map ---\n");
	printf("nodes (optimized poses) : %d\n", (int)poses.size());
	printf("links (graph edges)     : %d\n", (int)links.size());
	printf("cloud points            : %d raw -> %d after %.0f cm voxel filter\n",
			(int)rawSize, (int)filtered->size(), voxelSize * 100.0f);
	printf("bounding box [m]        : x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]\n",
			min.x, max.x, min.y, max.y, min.z, max.z);
	printf("cloud saved             : %s\n", cloudPath.c_str());
	printf("trajectory saved        : %s\n", trajectoryPath.c_str());
}

} // namespace

int main(int argc, char ** argv)
{
	std::string configPath = "config/rtabmap_minimal.ini";
	std::string dbPath = "rtabmap_minimal.db";
	std::string cloudPath = "rtabmap_minimal_cloud.pcd";
	std::string trajectoryPath = "rtabmap_minimal_trajectory.txt";
	bool useImu = true;
	bool gui = true;
	bool liveMap = true;
	int width = 640, height = 480, fps = 15;

	for(int i = 1; i < argc; ++i)
	{
		const std::string a = argv[i];
		if(a == "--config" && i + 1 < argc) configPath = argv[++i];
		else if(a == "--db" && i + 1 < argc) dbPath = argv[++i];
		else if(a == "--cloud" && i + 1 < argc) cloudPath = argv[++i];
		else if(a == "--no-imu") useImu = false;
		else if(a == "--no-gui") { gui = false; liveMap = false; }
		else if(a == "--no-view") gui = false;
		else if(a == "--no-map") liveMap = false;
		else if(a == "--fps" && i + 1 < argc) fps = atoi(argv[++i]);
		else if(a == "--size" && i + 2 < argc) { width = atoi(argv[++i]); height = atoi(argv[++i]); }
		else
		{
			printf("Usage: %s [--config f.ini] [--db out.db] [--cloud out.pcd]\n"
			       "          [--no-imu] [--no-gui] [--no-map] [--fps 15] [--size 640 480]\n", argv[0]);
			return a == "--help" ? 0 : 1;
		}
	}

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	// ---------------- RTAB-Map parameters ----------------
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	if(UFile::exists(configPath))
	{
		rtabmap::Parameters::readINI(configPath, parameters);
		printf("Parameters loaded from %s\n", configPath.c_str());
	}
	else
	{
		printf("No config file at %s, using RTAB-Map defaults.\n", configPath.c_str());
	}
	if(!useImu)
	{
		// Gravity constraints make sense only when IMU orientation is fed in.
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerGravitySigma(), "0"));
	}

	printf("RTAB-Map %s | optimizers: TORO=%d g2o=%d GTSAM=%d Ceres=%d | using strategy %s\n",
			RTABMAP_VERSION,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeTORO) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeG2O) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeGTSAM) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeCeres) ? 1 : 0,
			parameters.at(rtabmap::Parameters::kOptimizerStrategy()).c_str());

	// ---------------- RealSense D435i ----------------
	Capture capture;
	capture.imuFilter = rtabmap::IMUFilter::create(parameters);

	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
	cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
	if(useImu)
	{
		cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
		cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	}

	// Video framesets and IMU samples arrive on the same callback thread; the main
	// loop always takes the newest frameset and drops the rest (weak CPU friendly).
	auto callback = [&capture, useImu](const rs2::frame & frame)
	{
		if(rs2::frameset fs = frame.as<rs2::frameset>())
		{
			std::lock_guard<std::mutex> lock(capture.mutex);
			capture.frames = fs;
			capture.hasFrames = true;
			return;
		}
		if(!useImu) return;
		rs2::motion_frame mf = frame.as<rs2::motion_frame>();
		if(!mf) return;
		const rs2_vector v = mf.get_motion_data();
		std::lock_guard<std::mutex> lock(capture.mutex);
		if(mf.get_profile().stream_type() == RS2_STREAM_ACCEL)
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
					mf.get_timestamp() / 1000.0);
			capture.imuReady = true;
		}
	};

	rs2::pipeline_profile profile;
	try
	{
		profile = pipe.start(cfg, callback);
	}
	catch(const rs2::error & e)
	{
		printf("Failed to start the RealSense pipeline: %s\n", e.what());
		return 1;
	}

	rs2::video_stream_profile colorProfile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	const rs2_intrinsics intrinsics = colorProfile.get_intrinsics();
	const rtabmap::CameraModel model(
			"d435i", intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
			rtabmap::CameraModel::opticalRotation(), 0, cv::Size(intrinsics.width, intrinsics.height));

	// IMU pose in the same base frame as the camera model.
	rtabmap::Transform imuLocalTransform = rtabmap::Transform::getIdentity();
	if(useImu)
	{
		imuLocalTransform = rtabmap::CameraModel::opticalRotation() *
				fromRealSense(profile.get_stream(RS2_STREAM_GYRO).get_extrinsics_to(colorProfile));
	}

	const float depthScale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
	const bool depthInMillimeters = fabs(depthScale - 0.001f) < 1e-6f;

	printf("D435i %s fw %s | %dx%d @ %d fps | fx=%.1f fy=%.1f cx=%.1f cy=%.1f | IMU %s\n",
			profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
			profile.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
			intrinsics.width, intrinsics.height, fps,
			intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
			useImu ? "on" : "off");

	// ---------------- SLAM ----------------
	rtabmap::Odometry * odometry = rtabmap::Odometry::create(parameters);
	rtabmap::Rtabmap rtabmap;
	UFile::erase(dbPath); // fresh session; remove this line to keep mapping an existing map
	rtabmap.init(parameters, dbPath);

	// Live 3D map window: one PCL cloud per map node, re-posed whenever RTAB-Map
	// optimizes the graph. Updated only when a node is added (<= Rtabmap/DetectionRate),
	// so it costs almost nothing between nodes.
	// Both windows are RTAB-Map's own Qt widgets (the same CloudViewer that
	// rtabmap-databaseViewer uses). PCLVisualizer is not usable here: its X11
	// interactor segfaults on the first spinOnce with VTK 9.1 on this system.
	std::unique_ptr<QApplication> app;
	std::unique_ptr<rtabmap::ImageView> cameraViewer;
	std::unique_ptr<rtabmap::CloudViewer> viewer;
	std::map<int, std::string> shownNodes;
	if(gui || liveMap)
	{
		app.reset(new QApplication(argc, argv));
	}
	if(gui)
	{
		cameraViewer.reset(new rtabmap::ImageView());
		cameraViewer->setWindowTitle("rtabmap_minimal - camera");
		cameraViewer->resize(640, 480);
		cameraViewer->show();
	}
	if(liveMap)
	{
		viewer.reset(new rtabmap::CloudViewer());
		viewer->setWindowTitle("rtabmap_minimal - map");
		viewer->setBackgroundColor(QColor(30, 30, 30));
		viewer->setGridShown(true);
		viewer->setTrajectorySize(10000);
		viewer->resize(800, 600);
		viewer->show();
	}
	for(int i = 0; i < 2; ++i)
	{
		QWidget * w = i == 0 ? (QWidget*)cameraViewer.get() : (QWidget*)viewer.get();
		if(w)
		{
			QObject::connect(new QShortcut(QKeySequence("s"), w), &QShortcut::activated,
					[](){ g_save = true; });
			QObject::connect(new QShortcut(QKeySequence("q"), w), &QShortcut::activated,
					[](){ g_stop = true; });
		}
	}

	rs2::align alignToColor(RS2_STREAM_COLOR);
	signal(SIGINT, onSignal);
	printf("\nRunning. Move the camera slowly. [q] quit and save, [s] save now.\n\n");

	int frameId = 0, lostCount = 0, loopClosures = 0, lastLoopId = 0, mapNodes = 0;
	double displayFps = 0.0;
	UTimer timer, statusTimer, viewerTimer;

	while(!g_stop)
	{
		rs2::frameset frames;
		rtabmap::IMU imu;
		{
			std::lock_guard<std::mutex> lock(capture.mutex);
			if(capture.hasFrames)
			{
				frames = capture.frames;
				capture.hasFrames = false;
				if(useImu && capture.imuReady)
				{
					double qx, qy, qz, qw;
					capture.imuFilter->getOrientation(qx, qy, qz, qw);
					imu = rtabmap::IMU(
							cv::Vec4d(qx, qy, qz, qw), cv::Mat::eye(3, 3, CV_64FC1) * 0.0001,
							capture.gyro, cv::Mat::eye(3, 3, CV_64FC1) * 0.0001,
							capture.accel, cv::Mat::eye(3, 3, CV_64FC1) * 0.0001,
							imuLocalTransform);
				}
			}
		}
		if(!frames)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			continue;
		}

		rs2::frameset aligned = alignToColor.process(frames);
		rs2::video_frame colorFrame = aligned.get_color_frame();
		rs2::depth_frame depthFrame = aligned.get_depth_frame();
		if(!colorFrame || !depthFrame)
		{
			continue; // the very first frameset of a D435i may have no depth
		}

		timer.restart();
		cv::Mat rgb = cv::Mat(cv::Size(colorFrame.get_width(), colorFrame.get_height()),
				CV_8UC3, (void*)colorFrame.get_data()).clone();
		cv::Mat depth16 = cv::Mat(cv::Size(depthFrame.get_width(), depthFrame.get_height()),
				CV_16UC1, (void*)depthFrame.get_data());
		cv::Mat depth;
		if(depthInMillimeters)
		{
			depth = depth16.clone(); // RTAB-Map reads CV_16UC1 as millimeters
		}
		else
		{
			depth16.convertTo(depth, CV_32FC1, depthScale); // ... and CV_32FC1 as meters
		}

		rtabmap::SensorData data(rgb, depth, model, ++frameId, colorFrame.get_timestamp() / 1000.0);
		if(!imu.empty())
		{
			data.setIMU(imu);
		}

		// 1) odometry (RTAB-Map)
		rtabmap::OdometryInfo info;
		const rtabmap::Transform pose = odometry->process(data, &info);

		// 2) mapping + loop closure (RTAB-Map). It internally throttles to Rtabmap/DetectionRate.
		if(!pose.isNull())
		{
			if(rtabmap.process(data, pose, info.reg.covariance))
			{
				mapNodes = (int)rtabmap.getLocalOptimizedPoses().size();
				if(viewer)
				{
					const int nodeId = rtabmap.getLastLocationId();
					if(nodeId > 0 && shownNodes.find(nodeId) == shownNodes.end())
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
								rtabmap::util3d::cloudRGBFromSensorData(data, 8, 4.0f, 0.3f);
						if(!cloud->empty())
						{
							const std::string id = uFormat("node%d", nodeId);
							viewer->addCloud(id, cloud, pose);
							shownNodes.insert(std::make_pair(nodeId, id));
						}
					}
					// Loop closure moves old nodes: re-pose the clouds, never rebuild them.
					const std::map<int, rtabmap::Transform> & optimized = rtabmap.getLocalOptimizedPoses();
					for(std::map<int, std::string>::const_iterator iter = shownNodes.begin();
							iter != shownNodes.end(); ++iter)
					{
						std::map<int, rtabmap::Transform>::const_iterator jter = optimized.find(iter->first);
						if(jter != optimized.end() && !jter->second.isNull())
						{
							viewer->updateCloudPose(iter->second, jter->second);
						}
					}
				}
				// Loop/Id covers both appearance-based loop closure and proximity detection.
				const int loopId = (int)uValue(rtabmap.getStatistics().data(),
						rtabmap::Statistics::kLoopId(), 0.0f);
				if(loopId > 0)
				{
					lastLoopId = loopId;
					++loopClosures;
					printf("Loop closure / proximity link with node %d (total %d)\n",
							lastLoopId, loopClosures);
				}
			}
		}
		else
		{
			++lostCount;
		}

		const double elapsed = timer.elapsed();
		displayFps = displayFps == 0.0 ? 1.0 / elapsed : 0.9 * displayFps + 0.1 / elapsed;

		if(viewer && viewerTimer.elapsed() > 0.2)
		{
			viewerTimer.restart();
			if(!pose.isNull())
			{
				viewer->updateCameraTargetPosition(pose); // also grows the trajectory
			}
			viewer->refreshView();
		}

		if(cameraViewer)
		{
			float x = 0.f, y = 0.f, z = 0.f;
			if(!pose.isNull())
			{
				pose.getTranslation(x, y, z);
			}
			putLine(rgb, uFormat("%s  odom %s  features %d  inliers %d",
					pose.isNull() ? "LOST" : "TRACKING", pose.isNull() ? "-" : "ok",
					info.features, info.reg.inliers), 0);
			putLine(rgb, uFormat("frames %d (lost %d)  %.1f Hz  %.0f ms/frame",
					frameId, lostCount, displayFps, elapsed * 1000.0), 1);
			putLine(rgb, uFormat("map nodes %d  loop closures %d  last id %d",
					mapNodes, loopClosures, lastLoopId), 2);
			putLine(rgb, uFormat("xyz %.2f %.2f %.2f", x, y, z), 3);
			for(std::multimap<int, cv::KeyPoint>::const_iterator iter = info.words.begin();
					iter != info.words.end(); ++iter)
			{
				cv::circle(rgb, iter->second.pt, 2, cv::Scalar(0, 255, 0), -1);
			}
			cameraViewer->setImage(uCvMat2QImage(rgb));
		}

		if(app)
		{
			app->processEvents();
			const bool cameraOpen = cameraViewer && cameraViewer->isVisible();
			const bool mapOpen = viewer && viewer->isVisible();
			if(!cameraOpen && !mapOpen)
			{
				g_stop = true; // both windows closed
			}
		}

		if(g_save)
		{
			g_save = false;
			exportMap(rtabmap, cloudPath, trajectoryPath, 4, 4.0f, 0.03f);
		}
		else if(statusTimer.elapsed() > 2.0)
		{
			statusTimer.restart();
			printf("frames %d (lost %d) | %.1f Hz | nodes %d | loops %d\n",
					frameId, lostCount, displayFps, mapNodes, loopClosures);
		}
	}

	printf("\nStopping...\n");
	pipe.stop();
	exportMap(rtabmap, cloudPath, trajectoryPath, 4, 4.0f, 0.03f);
	rtabmap.close(true);
	delete odometry;
	delete capture.imuFilter;
	printf("database saved          : %s\n", dbPath.c_str());
	printf("inspect it with         : rtabmap-databaseViewer %s\n", dbPath.c_str());
	return 0;
}
