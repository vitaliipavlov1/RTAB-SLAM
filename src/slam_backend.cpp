#include "rtab_slam/slam_backend.hpp"

#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Statistics.h>

#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>

#include <cstdio>

namespace rtab_slam {
namespace {

// Wipes or keeps the previous map, and says which one it did.
void prepareDatabase(const SlamConfig & config)
{
	if(!UFile::exists(config.databasePath))
	{
		return;
	}
	if(config.continueMapping)
	{
		printf("Continuing the existing map in %s (a new session is appended).\n",
				config.databasePath.c_str());
	}
	else
	{
		printf("Replacing the existing database %s (pass --continue to keep mapping it).\n",
				config.databasePath.c_str());
		UFile::erase(config.databasePath);
	}
}

} // namespace

rtabmap::ParametersMap loadParameters(const std::string & iniPath, bool useImu, bool occupancyGrid)
{
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	if(UFile::exists(iniPath))
	{
		rtabmap::Parameters::readINI(iniPath, parameters);
		printf("Parameters loaded from %s\n", iniPath.c_str());
	}
	else
	{
		printf("No config file at %s, using RTAB-Map defaults.\n", iniPath.c_str());
	}

	if(!useImu)
	{
		// Gravity constraints make sense only when IMU orientation is fed in.
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerGravitySigma(), "0"));
	}
	if(!occupancyGrid)
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
			if(!uStr2Bool(uValue(parameters, key, std::string("true"))))
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

SlamBackend::SlamBackend(const rtabmap::ParametersMap & parameters, const SlamConfig & config) :
	odometry_(rtabmap::Odometry::create(parameters))
{
	const float rate = uStr2Float(uValue(parameters,
			rtabmap::Parameters::kRtabmapDetectionRate(), std::string("1.0")));
	detectionPeriod_ = rate > 0.0f ? 1.0 / rate : 0.0;

	prepareDatabase(config);
	rtabmap_.init(parameters, config.databasePath);
	open_ = true;
}

SlamBackend::~SlamBackend()
{
	close();
}

FrameResult SlamBackend::process(rtabmap::SensorData & data)
{
	FrameResult result;
	result.pose = odometry_->process(data, &result.odometry);
	if(result.pose.isNull())
	{
		return result;   // tracking lost, nothing to map
	}

	// Rtabmap::process() runs the whole mapping step - dictionary, loop closure
	// search, graph optimization - on every call. The standalone rtabmap program
	// drops frames in its own thread to honour Rtabmap/DetectionRate; a direct
	// library call does not, so the rate is applied here. Without it mapping eats
	// the time odometry needs and the camera ends up tracked a few times a second.
	if(data.stamp() - lastProcessStamp_ < detectionPeriod_)
	{
		return result;   // odometry only, this frame is not offered to the map
	}
	lastProcessStamp_ = data.stamp();

	result.nodeAdded = rtabmap_.process(data, result.pose, result.odometry.reg.covariance);
	if(result.nodeAdded)
	{
		// Loop/Id covers both appearance-based loop closure and proximity detection.
		result.loopId = static_cast<int>(uValue(rtabmap_.getStatistics().data(),
				rtabmap::Statistics::kLoopId(), 0.0f));
	}
	return result;
}

void SlamBackend::close()
{
	if(open_)
	{
		open_ = false;
		rtabmap_.close(true);
	}
}

} // namespace rtab_slam
