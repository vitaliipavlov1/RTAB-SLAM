#pragma once

// The three live windows - camera image with the HUD, 3D cloud map, 2D occupancy
// grid with the pose graph - all built from RTAB-Map's own Qt widgets.

#include "control.h"
#include "options.h"

#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/OdometryInfo.h>
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

#include <QApplication>
#include <QRect>
#include <QScreen>
#include <QShortcut>

#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <string>

namespace rtabmap_minimal {

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
inline void placeWindow(QWidget * widget, const QRect & target)
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

inline void createWindows(const Options & options, int & argc, char ** argv,
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
inline void updateLiveCloud(Windows & windows, const rtabmap::Rtabmap & rtabmap,
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
inline cv::Mat gridToImage(const cv::Mat & map8S)
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

inline void updateGraphView(Windows & windows, const rtabmap::Rtabmap & rtabmap)
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

inline void putLine(cv::Mat & image, const std::string & text, int line)
{
	const cv::Point at(8, 20 + 18 * line);
	cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
	cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

// Draws on a copy: the original frame is still referenced by the SensorData we
// handed to RTAB-Map, and the map must not get the overlay burned into it.
inline cv::Mat renderHud(const cv::Mat & rgb, const rtabmap::Transform & pose,
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
} // namespace rtabmap_minimal
