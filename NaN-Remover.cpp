#include<stdlib.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <iostream>
#include <vector>
#include <string>
#include<sstream>
#include<pcl/filters/filter.h>
#include <sstream>
using namespace std;
using namespace pcl;

int main(int argc, char ** argv){
    vector<int> mapping;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[1], *cloud_in);
    pcl::removeNaNFromPointCloud (*cloud_in, *cloud_out, mapping);  
   
  stringstream ss;
  ss<<argv[1]<<"_out.pcd";
  pcl::io::savePCDFile(ss.str(), * cloud_out);
  return 0;

}
