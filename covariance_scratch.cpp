#include <cstdlib>
#include<iostream>
#include<vector>
#include<pcl/PointIndices.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;

using namespace std;

void computeCovarianceMatrix(vector<int> point_Indices, cloudPtr src_cloud) {
	double covarianceMatrix[3][3];
	
	double means[3] = {0, 0, 0};
       
	for (int i = 0; i < point_Indices.size(); i++){
    pcl::PointXYZ inst_point = cloudPtr->points(point_Indices[i]);
		means[0] += inst_point[i].x,
		means[1] += inst_point[i].y,
		means[2] += inst_point[i].z;
  }
        
	means[0] /= point_Indices.size();
        means[1] /= point_Indices.size();
        means[2] /= point_Indices.size();
        
	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {                        
			covarianceMatrix[i][j] = 0.0;
			for (int k = 0; k < point_Indices.size(); k++){
                            pcl::PointXYZ inst_point = cloudPtr->points[point_Indices[k]];
                            covarianceMatrix[i][j] += (means[i] - inst_point.data[i]) *
                                                          (means[j] - inst_point.data[j]);
			covarianceMatrix[i][j] /= point_Indices.size() - 1;
		}	
  }
}
