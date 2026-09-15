#pragma once

// The three live windows: camera image with the HUD, 3D cloud map, 2D occupancy
// grid with the pose graph.
//
// All of them are RTAB-Map's own Qt widgets, the same ones rtabmap-databaseViewer
// is built from. PCLVisualizer is not an option here: its X11 interactor
// segfaults on the first spinOnce() with VTK 9.1, which reproduces in a 30-line
// program without RTAB-Map.

#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>

#include <opencv2/core.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>

class QApplication;
class QWidget;

namespace rtabmap {
class CloudViewer;
class GraphViewer;
class ImageView;
}

namespace rtabmap_minimal {

struct ViewerConfig
{
	bool camera = true;
	bool map3d = true;
	bool map2d = true;

	bool any() const { return camera || map3d || map2d; }
};

// What the camera window prints over the image.
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

class Viewer
{
public:
	// Creates the windows that the config asks for and lays them out on screen.
	// With every window disabled nothing Qt-related is constructed at all.
	Viewer(const ViewerConfig & config, const rtabmap::ParametersMap & parameters,
			int & argc, char ** argv);
	~Viewer();

	Viewer(const Viewer &) = delete;
	Viewer & operator=(const Viewer &) = delete;

	bool enabled() const { return config_.any(); }
	bool anyVisible() const;

	// [q] in any window, or every window closed.
	bool quitRequested() const { return quit_; }
	// [s] in any window; reading it clears the request.
	bool takeSaveRequest();

	// Camera window: the frame is copied before the HUD is drawn, so the map
	// never gets the overlay burned into it.
	void showFrame(const cv::Mat & rgb, const rtabmap::Transform & pose,
			const rtabmap::OdometryInfo & info, const Hud & hud);

	// Map windows: called when RTAB-Map created a node. The 3D view gets that
	// node's cloud, the 2D view the reassembled occupancy grid and pose graph.
	void onNewNode(const rtabmap::Rtabmap & rtabmap, const rtabmap::SensorData & data,
			const rtabmap::Transform & odomPose);

	// Moves the current-position markers and repaints, throttled internally.
	void followCamera(const rtabmap::Transform & pose);

	void processEvents();

private:
	void place(QWidget * widget, int x, int y, int width, int height);
	void bindShortcuts(QWidget * widget);
	void updateCloud(const rtabmap::Rtabmap & rtabmap, const rtabmap::SensorData & data,
			const rtabmap::Transform & odomPose);
	void updateGraph(const rtabmap::Rtabmap & rtabmap);

	ViewerConfig config_;
	std::unique_ptr<QApplication> app_;          // must outlive the widgets below
	std::unique_ptr<rtabmap::ImageView> camera_;
	std::unique_ptr<rtabmap::CloudViewer> map3d_;
	std::unique_ptr<rtabmap::GraphViewer> map2d_;

	rtabmap::LocalGridCache gridCache_;          // must outlive occupancyGrid_
	std::unique_ptr<rtabmap::OccupancyGrid> occupancyGrid_;
	std::map<int, std::string> shownClouds_;     // map node id -> cloud id

	std::chrono::steady_clock::time_point lastRefresh_;
	bool quit_ = false;
	bool save_ = false;
};

} // namespace rtabmap_minimal
