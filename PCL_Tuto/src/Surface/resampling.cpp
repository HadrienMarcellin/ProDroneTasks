#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


int main(int argc, char **argv)
{
	//Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	//Load bun0.pcd -- shoukd be available on PCL archives
	pcl::io::loadPCDFile("../pcd/bun0.pcd", *cloud);
	
	//Create a KD-tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	
	//Output has the PointNormal Typein in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;
	
	//Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	
	mls.setComputeNormals(true);
	//Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);


	//Reconstruct
	mls.process(mls_points);
	
	//Save output
	pcl::io::savePCDFile("../pcd/bun0-mls.pcd", mls_points);
	
	return 1;

}
