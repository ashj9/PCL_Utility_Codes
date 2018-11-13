#include<iostream>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include<pcl/common/common.h>
#include<eigen3/Eigen/Core>
#include<pcl/common/centroid.h>
#include<pcl/common/common.h>


//using namespace pcl;
using namespace std;


int main(int argc, char ** argv )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1) 
    {
      std::cout << "Cloud reading failed." << std::endl;
      return (-1);
    }
    Eigen::Matrix3f covariance_mat;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::computeCovarianceMatrix(*cloud, centroid, covariance_mat);
    
    
    cout<<covariance_mat<<endl;
          
    return 0;
}
