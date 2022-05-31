/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "ransac3d.h"
using namespace lidar_obstacle_detection;

template<typename PointT>
Ransac<PointT>::~Ransac(){}

template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Ransac3d(PtCdtr<PointT> cloud){
	std::unordered_set<int> inliersResult;
	//srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers


	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
			inliers.insert(rand()%num_points);
	
		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		auto itr = inliers.begin();
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		z1=cloud->points[*itr].z;
		itr++;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;
		z2=cloud->points[*itr].z;
		itr++;
		x3=cloud->points[*itr].x;
		y3=cloud->points[*itr].y;
		z3=cloud->points[*itr].z;
		
		float a = x2-x1;
		float b = y2-y1;
		float c = z2-z1;
		float d = x3-x1;
		float e = y3-y1;
		float f = z3-z1;
		float v_i = (b*f)-(c*e);
		float v_j = (c*d)-(a*f);
		float v_k = (a*e)-(b*d);
		float A = v_i;
		float B = v_j;
		float C = v_k;
		float D = -(v_i*x1+v_j*y1+v_k*z1);

		float sqrt_abc;
		sqrt_abc = sqrt(A*A + B*B + C*C);

		for (int ind =0;ind<num_points;ind++){
			if(inliers.count(ind)>0){
				continue;
			}
			PointT point = cloud->points[ind];
			float x = point.x;
			float y = point.y;
			float z = point.z;
			float dist = fabs(A*x + B*y + C*z + D) / sqrt_abc;

			if(dist < distanceTol){
				inliers.insert(ind);
			}
			if(inliers.size()>inliersResult.size()){
				inliersResult = inliers;
			}
		}
	}
	return inliersResult;
}