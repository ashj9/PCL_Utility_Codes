/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KD_Radius_Search.cpp
 * Author: ashj
 *
 * Created on 13 December, 2018, 10:30 AM
 */

#include <cstdlib>
#include<iostream>
#include<vector>
#include<fstream>
#include<sstream>


#include<pcl/PointIndices.h>
#include<pcl/point_types.h>

#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/kdtree/kdtree_flann.h>


using namespace std;

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::io::loadPCDFile(argv[1], *cloud);
    pcl::io::loadPCDFile(argv[2], *keypoints_cloud);
    pcl::io::loadPCDFile(argv[3], *merged_cloud);
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    
    vector< vector<int> > globalNeighbourPointIdx;
    vector< vector<int> > globalRadiusSquaredDistance;
    
    //float radius = 256.0f * rand () / (RAND_MAX + 1.0f);
    float radius = atof(argv[4]);
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(merged_cloud);
       
    assert(keypoints_cloud->size() == merged_cloud->size() - cloud->size());
    int kp_count = keypoints_cloud->size();
    pcl::PointXYZ searchPoint;
    
    
    //writing the neighbourest neighbours
    stringstream ss;
    fstream fs;
    fs.open("NN.txt", ios::out);
    for(int i=0; i<20; ++i){
        searchPoint = keypoints_cloud->points[i];
        cout<<i<<": "<<endl;
        fs<<i<<"\n";
        if( kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
            
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                cout<<pointIdxRadiusSearch[i]<<",";
                fs<<pointIdxRadiusSearch[i]<<",";
            }
        }
        fs<<"\n";
        cout<<"\n";
    }
    fs.close();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree.radiusSearch(keypoints_cloud->points[20], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    out_cloud->height = 1;
    out_cloud->width = pointIdxRadiusSearch.size();
    out_cloud->is_dense = false;
    out_cloud->points.resize (out_cloud->width);
    
    for(int i = 0; i<out_cloud->width; ++i){
        out_cloud->points[i] = merged_cloud->points[pointIdxRadiusSearch[i]];
    }
    
    pcl::io::savePCDFile("OutNeighbours.pcd", *out_cloud);
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr kp(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree.radiusSearch(keypoints_cloud->points[20], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    kp->height = 1;
    kp->width = 1;
    kp->is_dense = false;
    kp->points.resize (kp->width);
    
    for(int i = 0; i<kp->width; ++i){
        kp->points[i] = keypoints_cloud->points[20];
    }
    
    pcl::io::savePCDFile("KP.pcd", *kp);
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(out_cloud, 200, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (out_cloud, single_color1, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud1");
    

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(kp, 0, 0, 150);
    viewer->addPointCloud<pcl::PointXYZ> (kp, single_color2, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud2");


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}
