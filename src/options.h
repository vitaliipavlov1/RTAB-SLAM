#pragma once

// Everything the run is configured with: tuning constants that are not worth an
// ini entry, the command line, the RTAB-Map parameter set and the output database.

#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>

#include <cstdio>
#include <cstdlib>
#include <string>

namespace rtabmap_minimal {

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

inline void printUsage(const char * program)
{
	printf("Usage: %s [--config f.ini] [--db out.db] [--cloud out.pcd] [--continue]\n"
	       "          [--no-imu] [--no-gui] [--no-view] [--no-map3d] [--no-map2d]\n"
	       "          [--fps 15] [--size 640 480]\n", program);
}

inline ParseResult parseArgs(int argc, char ** argv, Options & options)
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

inline rtabmap::ParametersMap loadParameters(const Options & options)
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

inline void printBackends(const rtabmap::ParametersMap & parameters)
{
	printf("RTAB-Map %s | optimizers: TORO=%d g2o=%d GTSAM=%d Ceres=%d | using strategy %s\n",
			RTABMAP_VERSION,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeTORO) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeG2O) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeGTSAM) ? 1 : 0,
			rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeCeres) ? 1 : 0,
			uValue(parameters, rtabmap::Parameters::kOptimizerStrategy(), std::string("?")).c_str());
}

inline void prepareDatabase(const Options & options)
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
} // namespace rtabmap_minimal
