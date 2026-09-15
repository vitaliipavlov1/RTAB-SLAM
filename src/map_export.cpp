#include "map_export.h"

#include <rtabmap/core/Graph.h>
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
namespace {

// Saved map: denser than the live view, still bounded.
constexpr int kDecimation = 4;
constexpr float kMaxDepth = 4.0f;
constexpr float kMinDepth = 0.3f;
constexpr float kVoxelSize = 0.03f;

} // namespace

bool exportMap(rtabmap::Rtabmap & rtabmap, const ExportPaths & paths)
{
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	std::map<int, rtabmap::Signature> signatures;
	rtabmap.getGraph(poses, links, true /*optimized*/, true /*global*/, &signatures, true /*images*/);

	if(poses.empty())
	{
		printf("Map is empty, nothing to export.\n");
		return false;
	}

	std::map<int, double> stamps;
	for(const auto & [nodeId, signature] : signatures)
	{
		stamps.insert(std::make_pair(nodeId, signature.getStamp()));
	}
	// Format 11 = "id x y z qx qy qz qw" (RGBD-SLAM style, needs a stamp per pose).
	rtabmap::graph::exportPoses(paths.trajectoryPath,
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
				rtabmap::util3d::cloudRGBFromSensorData(data, kDecimation, kMaxDepth, kMinDepth);
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
		return false;
	}

	const std::size_t rawSize = assembled->size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
	voxel.setInputCloud(assembled);
	voxel.setLeafSize(kVoxelSize, kVoxelSize, kVoxelSize);
	voxel.filter(*filtered);

	pcl::PointXYZRGB min, max;
	pcl::getMinMax3D(*filtered, min, max);

	filtered->is_dense = false;
	pcl::io::savePCDFileBinary(paths.cloudPath, *filtered);

	printf("\n--- map ---\n");
	printf("nodes (optimized poses) : %d\n", static_cast<int>(poses.size()));
	printf("links (graph edges)     : %d\n", static_cast<int>(links.size()));
	printf("cloud points            : %d raw -> %d after %.0f cm voxel filter\n",
			static_cast<int>(rawSize), static_cast<int>(filtered->size()), kVoxelSize * 100.0f);
	printf("bounding box [m]        : x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]\n",
			min.x, max.x, min.y, max.y, min.z, max.z);
	printf("cloud saved             : %s\n", paths.cloudPath.c_str());
	printf("trajectory saved        : %s\n", paths.trajectoryPath.c_str());
	return true;
}

} // namespace rtabmap_minimal
