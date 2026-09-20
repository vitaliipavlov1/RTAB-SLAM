#pragma once

// RTAB-Map as the SLAM backend.
//
// One frame in, one pose and a bit of bookkeeping out: odometry, memory, loop
// closure and graph optimization are all RTAB-Map's, this class only feeds it
// and reports what happened.

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>

#include <memory>
#include <string>

namespace rtab_slam {

struct SlamConfig
{
	std::string databasePath = "rtab_slam.db";
	bool continueMapping = false;   // keep an existing database instead of replacing it
};

struct FrameResult
{
	rtabmap::Transform pose;          // null when odometry lost tracking
	rtabmap::OdometryInfo odometry;   // features, inliers and tracked words for the HUD
	bool nodeAdded = false;           // RTAB-Map created a map node from this frame
	int loopId = 0;                   // >0: loop closure or proximity link with that node
};

// Reads config/rtab_slam.ini when it exists, falls back to RTAB-Map's
// defaults, and applies the two switches that depend on the command line.
rtabmap::ParametersMap loadParameters(const std::string & iniPath, bool useImu, bool occupancyGrid);

// Prints which optimizers this build of RTAB-Map actually has.
void printBackends(const rtabmap::ParametersMap & parameters);

class SlamBackend
{
public:
	SlamBackend(const rtabmap::ParametersMap & parameters, const SlamConfig & config);
	~SlamBackend();

	SlamBackend(const SlamBackend &) = delete;
	SlamBackend & operator=(const SlamBackend &) = delete;

	// Runs odometry and, when odometry holds, mapping and loop closure.
	FrameResult process(rtabmap::SensorData & data);

	// Saves and closes the database. Safe to call once; the destructor does it too.
	void close();

	rtabmap::Rtabmap & rtabmap() { return rtabmap_; }
	const rtabmap::Rtabmap & rtabmap() const { return rtabmap_; }
	int mapNodes() const { return static_cast<int>(rtabmap_.getLocalOptimizedPoses().size()); }

private:
	std::unique_ptr<rtabmap::Odometry> odometry_;
	rtabmap::Rtabmap rtabmap_;
	double detectionPeriod_ = 0.0;    // seconds between mapping steps, from Rtabmap/DetectionRate
	double lastProcessStamp_ = 0.0;
	bool open_ = false;
};

} // namespace rtab_slam
