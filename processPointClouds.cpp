// PCL lib Functions for processing point clouds 
#include <unordered_set>
#include "processPointClouds.h"

using namespace lidar_obstacle_detection;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
//void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
void ProcessPointClouds<PointT>::numPoints(PtCdtr<PointT> cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::RansacSegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));

    int num_points = cloud->points.size();
    auto cloud_points = cloud->points;
    Ransac<PointT> RansacSeg(maxIterations,distanceTol,num_points);

    std::unordered_set<int> inliersResult = RansacSeg.Ransac3d(cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
    std::cout<<"plane ransac-segment took"<<elapsedTime.count()<<"milliseconds"<<std::endl;

    PtCdtr<PointT> out_plane(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> in_plane(new pcl::PointCloud<PointT>());

    for(int i = 0;i<num_points;i++)
    {
        PointT pt = cloud_points[i];
        if(inliersResult.count(i))
        {
            out_plane->points.push_back(pt);
        }else{
            in_plane->points.push_back(pt);
        }
    }
    return std::pair<PtCdtr<PointT>, PtCdtr<PointT>>(in_plane,out_plane);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
//PtCdtr<PointT> ProcessPointClouds<PointT>::FilterCloud(PtCdtr<PointT> cloud,float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    //PtCdtr<PointT> cloud_filtered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter (*cloud_filtered);
    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
//std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdtr<PointT> cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    //typename pcl::PointCloud<PointT>::Ptr obstCloud (new );
    //typename pcl::PointCloud<PointT>::Ptr obstCloud {new pcl::PointCloud<PointT>()};
    PtCdtr<PointT> obstCloud(new pcl::PointCloud<PointT>());

    //typename pcl::PointCloud<PointT>::Ptr PlaneCloud {new pcl::PointCloud<PointT>()};
    PtCdtr<PointT> planeCloud(new pcl::PointCloud<PointT>());
    for(int index:inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);


    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, PlaneCloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
//std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::SegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
/*
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
    

    
    while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
			inliers.insert(rand()%(cloud->points.size()));
	
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
		
		
		//float a = y1-y2;
		//float b = x2-x1;
		//float c = x1*y2-x2*y1;
        

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

		for(int index =0;index<cloud->points.size();index++)
		{
			if(inliers.count(index)>0)
				continue;
		
			//pcl::PointXYZ point = cloud->points[index];
            PointT point = cloud->points[index];
			float x4=point.x;
			float y4=point.y;
			float z4=point.z;

			float dist = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);

			if(dist<=distanceThreshold)
				inliers.insert(index);
		}
		if(inliers.size()>inliersResult.size())
		{
			inliersResult=inliers;
		}

	}

*/
 
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::SACSegmentation<PointT> seg;
    // TODO:: Fill in this function to find inliers for the cloud.
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size()==0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
        //break;
    }

    
    /*    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

        for (int index=0;index<cloud->points.size();index++)
        {
            PointT point = cloud->points[index];
            if (inliersResult.count(index))
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point);
        }*/
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult = SeparateClouds(inliers,cloud);//(cloudOutliers,cloudInliers);
    //return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> (cloudOutliers,cloudInliers);

    return segResult;
}


template<typename PointT>
//std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::Clustering(PtCdtr<PointT> cloud,float clusterTolerance,int minSize,int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<PtCdtr<PointT>> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        //typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        PtCdtr<PointT> cloudCluster(new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }
        


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::EuclideanClustering(PtCdtr<PointT> cloud,float clusterTolerance, int minsize, int maxsize)
{
    auto startTime = std::chrono::steady_clock::now();
    ClusterPts<PointT> clusterPoints(cloud->points.size(),clusterTolerance,minsize,maxsize);
    std::vector<PtCdtr<PointT>> clusters = clusterPoints.EuclidCluster(cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
    std::cout<<"KDTree clustering took"<<elapsedTime.count()<<"milliseconds and found"<<clusters.size()<<"clusters"<<std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(PtCdtr<PointT> cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(PtCdtr<PointT> cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    PtCdtr<PointT> cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}