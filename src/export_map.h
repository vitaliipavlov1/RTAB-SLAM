#pragma once

// Saving the result: the optimized graph assembled into one PCL cloud, filtered,
// measured and written next to the trajectory.

#include "options.h"

#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/util3d.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cstdio>
#include <map>

namespace rtabmap_minimal {

// Assemble the optimized map as a PCL cloud, filter it, print statistics, save it.
// Note: the whole graph is materialized in memory, which is fine for a demo-sized
// map; for a long session use rtabmap-export on the .db instead.
inline void exportMap(rtabmap::Rtabmap & rtabmap, const Options & options)
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
} // namespace rtabmap_minimal
