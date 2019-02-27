#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>

int main (int argc, char** argv)
{
	srand ((unsigned int) time (NULL));

	// Octree resolution - side length of octree voxels
	float resolution = 0.01f;

	// Instantiate octree-based point cloud change detection class
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

	//Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> ());
	//Load bun0.pcd -- shoukd be available on PCL archives
	pcl::io::loadPCDFile("../pcd/bun0.pcd", *cloudA);

	// Add points from cloudA to octree
	octree.setInputCloud (cloudA);
	octree.addPointsFromInputCloud ();

	// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
	octree.switchBuffers ();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );

	//Load bun0.pcd -- should be available on PCL archives
	pcl::io::loadPCDFile("../pcd/bun0-mls.pcd", *cloudB);

	// Add points from cloudB to octree
	octree.setInputCloud (cloudB);
	octree.addPointsFromInputCloud ();

	std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

	// Output points
	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	for (size_t i = 0; i < newPointIdxVector.size (); ++i)
	std::cout << i << "# Index:" << newPointIdxVector[i]
		      << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
		      << cloudB->points[newPointIdxVector[i]].y << " "
		      << cloudB->points[newPointIdxVector[i]].z << std::endl;

}