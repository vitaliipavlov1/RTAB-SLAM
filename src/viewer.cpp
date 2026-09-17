#include "viewer.h"

#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Statistics.h>
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

#include <clocale>

namespace rtabmap_minimal {
namespace {

// Live 3D map: one cloud per map node, kept cheap on purpose.
constexpr int kLiveDecimation = 8;
constexpr float kLiveMaxDepth = 4.0f;
constexpr float kLiveMinDepth = 0.3f;   // closer than this the D435i is unreliable

constexpr double kRefreshPeriod = 0.2;   // s, how often the 3D view repaints
constexpr unsigned int kTrajectorySize = 10000;

// Occupancy grid palette: the usual SLAM look.
constexpr unsigned char kGridFree = 255;
constexpr unsigned char kGridOccupied = 0;
constexpr unsigned char kGridUnknown = 128;

// RTAB-Map's occupancy grid (-1 unknown, 0 empty, 100 occupied) as a grey image.
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

void putLine(cv::Mat & image, const std::string & text, int line)
{
	const cv::Point at(8, 20 + 18 * line);
	cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
	cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

} // namespace

Viewer::Viewer(const ViewerConfig & config, const rtabmap::ParametersMap & parameters,
		int & argc, char ** argv) :
	config_(config),
	lastRefresh_(std::chrono::steady_clock::now())
{
	if(!config_.any())
	{
		return;
	}
	app_ = std::make_unique<QApplication>(argc, argv);
	// QApplication calls setlocale(LC_ALL, ""), so a locale like ru_RU/es_ES
	// makes printf and the trajectory export write "0,15" instead of "0.15".
	// Numbers we write are data, not UI text: keep them in the C locale.
	std::setlocale(LC_NUMERIC, "C");

	// Layout: camera top-left, 2D map top-right, 3D map centred below them.
	QRect screen(0, 0, 1280, 720);
	if(QApplication::primaryScreen())
	{
		screen = QApplication::primaryScreen()->availableGeometry();
	}
	const int halfW = screen.width() / 2;
	const int halfH = screen.height() / 2;

	if(config_.camera)
	{
		camera_ = std::make_unique<rtabmap::ImageView>();
		camera_->setWindowTitle("rtabmap_minimal - camera");
		camera_->setBackgroundColor(QColor(30, 30, 30));
		place(camera_.get(), screen.x(), screen.y(), halfW, halfH);
		bindShortcuts(camera_.get());
	}
	if(config_.map2d)
	{
		map2d_ = std::make_unique<rtabmap::GraphViewer>();
		map2d_->setWindowTitle("rtabmap_minimal - 2D map");
		map2d_->setGridMapVisible(true);
		map2d_->setNodeColor(QColor(0, 0, 255));          // trajectory nodes
		map2d_->setNeighborColor(QColor(0, 0, 255));      // odometry links
		map2d_->setNeighborMergedColor(QColor(0, 0, 255));
		map2d_->setGlobalLoopClosureColor(QColor(255, 0, 0));
		map2d_->setLocalLoopClosureColor(QColor(255, 0, 0));
		map2d_->setNodeRadius(0.04f);
		map2d_->setLinkWidth(0.02f);
		map2d_->setEnsureFrameVisible(true);
		map2d_->setBackgroundBrush(QBrush(QColor(kGridUnknown, kGridUnknown, kGridUnknown)));
		place(map2d_.get(), screen.x() + halfW, screen.y(), screen.width() - halfW, halfH);
		bindShortcuts(map2d_.get());
		occupancyGrid_ = std::make_unique<rtabmap::OccupancyGrid>(&gridCache_, parameters);
	}
	if(config_.map3d)
	{
		map3d_ = std::make_unique<rtabmap::CloudViewer>();
		map3d_->setWindowTitle("rtabmap_minimal - map");
		map3d_->setBackgroundColor(QColor(30, 30, 30));
		map3d_->setGridShown(true);
		map3d_->setTrajectorySize(kTrajectorySize);
		place(map3d_.get(), screen.x() + halfW / 2, screen.y() + halfH, halfW, screen.height() - halfH);
		bindShortcuts(map3d_.get());
	}
}

Viewer::~Viewer() = default;

// Places a window so that its *frame* occupies exactly the given rectangle, so
// the three windows touch each other without overlapping.
void Viewer::place(QWidget * widget, int x, int y, int width, int height)
{
	const QRect target(x, y, width, height);
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

void Viewer::bindShortcuts(QWidget * widget)
{
	QObject::connect(new QShortcut(QKeySequence("s"), widget), &QShortcut::activated,
			[this](){ save_ = true; });
	QObject::connect(new QShortcut(QKeySequence("q"), widget), &QShortcut::activated,
			[this](){ quit_ = true; });
}

bool Viewer::anyVisible() const
{
	return (camera_ && camera_->isVisible()) ||
		(map3d_ && map3d_->isVisible()) ||
		(map2d_ && map2d_->isVisible());
}

bool Viewer::takeSaveRequest()
{
	const bool requested = save_;
	save_ = false;
	return requested;
}

void Viewer::showFrame(const cv::Mat & rgb, const rtabmap::Transform & pose,
		const rtabmap::OdometryInfo & info, const Hud & hud)
{
	if(!camera_)
	{
		return;
	}
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
	camera_->setImage(uCvMat2QImage(display));
}

void Viewer::onNewNode(const rtabmap::Rtabmap & rtabmap, const rtabmap::SensorData & data,
		const rtabmap::Transform & odomPose)
{
	if(map3d_)
	{
		updateCloud(rtabmap, data, odomPose);
	}
	if(map2d_)
	{
		updateGraph(rtabmap);
	}
}

// One cloud per map node. Loop closure moves old nodes: the clouds are re-posed,
// never rebuilt. Nodes that RTAB-Map moved out of working memory are dropped
// here as well, which is what keeps this bounded on a long session.
void Viewer::updateCloud(const rtabmap::Rtabmap & rtabmap, const rtabmap::SensorData & data,
		const rtabmap::Transform & odomPose)
{
	const int nodeId = rtabmap.getLastLocationId();
	if(nodeId > 0 && shownClouds_.find(nodeId) == shownClouds_.end())
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
				rtabmap::util3d::cloudRGBFromSensorData(data, kLiveDecimation, kLiveMaxDepth, kLiveMinDepth);
		if(!cloud->empty())
		{
			const std::string cloudId = uFormat("node%d", nodeId);
			map3d_->addCloud(cloudId, cloud, odomPose);
			shownClouds_.insert(std::make_pair(nodeId, cloudId));
		}
	}

	const std::map<int, rtabmap::Transform> & optimized = rtabmap.getLocalOptimizedPoses();
	for(auto iter = shownClouds_.begin(); iter != shownClouds_.end(); )
	{
		const auto jter = optimized.find(iter->first);
		if(jter == optimized.end())
		{
			map3d_->removeCloud(iter->second);
			iter = shownClouds_.erase(iter);
		}
		else
		{
			if(!jter->second.isNull())
			{
				map3d_->updateCloudPose(iter->second, jter->second);
			}
			++iter;
		}
	}
}

void Viewer::updateGraph(const rtabmap::Rtabmap & rtabmap)
{
	const rtabmap::Statistics & stats = rtabmap.getStatistics();
	const rtabmap::Signature & last = stats.getLastSignatureData();
	if(last.id() > 0 && last.sensorData().gridCellSize() > 0.0f)
	{
		// Local occupancy grids are computed by RTAB-Map itself
		// (RGBD/CreateOccupancyGrid), we only assemble and show them.
		gridCache_.add(last.id(),
				last.sensorData().gridGroundCellsRaw(),
				last.sensorData().gridObstacleCellsRaw(),
				last.sensorData().gridEmptyCellsRaw(),
				last.sensorData().gridCellSize(),
				last.sensorData().gridViewPoint());
	}

	const std::map<int, rtabmap::Transform> & poses =
			stats.poses().empty() ? rtabmap.getLocalOptimizedPoses() : stats.poses();
	if(occupancyGrid_->update(poses))
	{
		float xMin = 0.0f, yMin = 0.0f;
		const cv::Mat map8S = occupancyGrid_->getMap(xMin, yMin);
		if(!map8S.empty())
		{
			map2d_->updateMap(gridToImage(map8S), occupancyGrid_->getCellSize(), xMin, yMin);
		}
	}

	std::map<int, int> mapIds;   // single session: every node belongs to map 0
	for(const auto & [nodeId, pose] : poses)
	{
		(void)pose;
		mapIds.insert(mapIds.end(), std::make_pair(nodeId, 0));
	}
	map2d_->updateGraph(poses, stats.constraints(), mapIds);
}

void Viewer::followCamera(const rtabmap::Transform & pose)
{
	if(pose.isNull())
	{
		return;
	}
	if(map3d_)
	{
		map3d_->updateCameraTargetPosition(pose);   // also grows the trajectory
	}
	if(map2d_)
	{
		map2d_->updateReferentialPosition(pose);
	}

	const auto now = std::chrono::steady_clock::now();
	if(map3d_ && std::chrono::duration<double>(now - lastRefresh_).count() > kRefreshPeriod)
	{
		lastRefresh_ = now;
		map3d_->refreshView();
	}
}

void Viewer::processEvents()
{
	if(!app_)
	{
		return;
	}
	app_->processEvents();
	if(!anyVisible())
	{
		quit_ = true;   // every window closed
	}
}

} // namespace rtabmap_minimal
