#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/filters/project_inliers.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

using namespace std;
int main(int args, char ** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(argv[1], * cloud);
 

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (atof(argv[2]));

  // Compute the features
  ne.compute (*cloud_normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

  
  pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
  pcl::io::savePCDFile<pcl::PointNormal>("CloudWithNormal.pcd", *cloud_with_normals );
  return 0;

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}
