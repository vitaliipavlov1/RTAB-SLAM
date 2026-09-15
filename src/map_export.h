#pragma once

// Saving the result.
//
// The optimized graph is assembled into a single PCL cloud, voxel filtered,
// measured and written next to the trajectory. This is where PCL does real
// work; RTAB-Map's own converter provides the per-node clouds.

#include <rtabmap/core/Rtabmap.h>

#include <string>

namespace rtabmap_minimal {

struct ExportPaths
{
	std::string cloudPath = "rtabmap_minimal_cloud.pcd";
	std::string trajectoryPath = "rtabmap_minimal_trajectory.txt";
};

// Prints a short summary of what was written. Returns false when the map is
// still empty. Note that the whole graph is materialized in memory, which is
// fine for a demo-sized map; for a long session use rtabmap-export on the .db.
bool exportMap(rtabmap::Rtabmap & rtabmap, const ExportPaths & paths);

} // namespace rtabmap_minimal
