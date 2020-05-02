// PCL lib Functions for processing point clouds 
#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>());
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filteredCloud);

    // select region of interest pcl::CropBox<pcl::PointXYZI>::Ptr
    typename pcl::PointCloud<PointT>::Ptr regionCloud (new pcl::PointCloud<PointT>());
    typename pcl::CropBox<PointT>::Ptr roi  (new pcl::CropBox<PointT>(true));
    roi->setMax(maxPoint);
    roi->setMin(minPoint);
    roi->setInputCloud(filteredCloud);
    roi->filter(*regionCloud);

    // To filter out the ego car points we need to get the indices of within that box we will filter out
    std::vector<int> indices;
    typename pcl::CropBox<PointT>::Ptr roi_roof  (new pcl::CropBox<PointT>(true));
    roi->setMax(Eigen::Vector4f(3,2,0.2,1));
    roi->setMin(Eigen::Vector4f(-3,-2,-2,1));
    roi->setInputCloud(regionCloud);
    roi->filter(indices);

    // extract these roof points from the point cloud
    // you cannot use a simple cropbox again as you need to be able to extract the point within the box
    pcl::ExtractIndices<PointT> extract_roof;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // convert the vector of ints to vector of indices
    for(int index: indices)
    {
        inliers->indices.push_back(index);
    }

    extract_roof.setInputCloud(regionCloud);
    extract_roof.setIndices(inliers);
    extract_roof.setNegative(true);
    extract_roof.filter(*regionCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr objects (new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road);

    extract.setNegative(true);
    extract.filter(*objects);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road, objects);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliers;
    auto startTime = std::chrono::steady_clock::now();

	srand(time(NULL));
	
	// TODO: Fill in this function
	// Randomly sample subset and fit line 
	//std::cout << "the cloud has a size of: " << cloud->size() << endl;
	// For max iterations 
	for( int i = 0; i<maxIterations; i++)
	{
		// Select two indices in the range of cloud size
		while(inliers.size()<3)
		{
			inliers.insert(rand()%cloud->size());
		}
		// print the indices
		//int j = 0;
		//for(auto it = inliers.begin(); it!=inliers.end(); it++)
		//{
		//	std::cout << "cycle " << j << " index: " << *it << endl;
		//	j++;
		//}
		// extract the points
		PointT p1, p2, p3;
		auto it = inliers.begin();
		p1 = cloud->points[*it];
		it++;
		p2 = cloud->points[*it];
		it++;
		p3 = cloud->points[*it];
		// print the point data
		//std::cout << "p1 x: " << p1.x << " y: " << p1.y << " z: " << p1.z << endl;
		//std::cout << "p2 x: " << p2.x << " y: " << p2.y << " z: " << p2.z << endl;
		//std::cout << "p3 x: " << p3.x << " y: " << p3.y << " z: " << p3.z << endl;

		// define the vectors on the plane with p1 as ref
		std::vector<float> v1 = {p2.x-p1.x, p2.y-p1.x, p2.z-p1.z};
		std::vector<float> v2 = {p3.x-p1.x, p3.y-p1.x, p3.z-p1.z};

		// find the normal vector to the plane taking cross product of v1 x v2
		// Measure distance between every point and fitted line, im just calculating the factors I need
		float A=0, B=0, C=0, D=0;
		A = (p2.y - p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		B = (p2.z - p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		C = (p2.x - p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
		D = -(A*p1.x+B*p1.y+C*p1.z);
		//std::cout << "A: " << A << " B: " << B << " C: " << C << " D: " << D << endl;

		for(int i = 0; i < cloud->size(); i++)
		{
			// check if point is not one of the two samples
			if (inliers.find(i)!=inliers.end())
			{
				continue;
			}
			PointT p4 = cloud->points[i];
			float distance = fabs(A*p4.x + B*p4.y + C*p4.z + D) / sqrt(pow(A,2)+pow(B,2)+pow(C,2));
			//std::cout << "distance p3: " << distance << std::endl;

			// If distance is smaller than threshold count it as inlier
			if(distance < distanceTol)
			{
				inliers.insert(i);
			}
		}
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
		inliers.clear();
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inlier
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // inliers_set contains the indices of the points in plane
    std::unordered_set<int> inliers_set = Ransac(cloud, maxIterations, distanceThreshold);
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int i = 0; i < cloud->points.size(); i++)
    {
        // check if the current point in the cloud is in the unordered set
        if(inliers_set.count(i))
        {
            cloudInliers->points.push_back(cloud->points[i]);
        }
        else
        {
            cloudOutliers->points.push_back(cloud->points[i]);
        }
    }  
    typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>  cloud_pair(cloudInliers, cloudOutliers);
    return cloud_pair;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0){
        std::cerr << "Could not estimate planar model for the given dataset" << std::endl;
    }
    else
    {
        std::cout << "Found plane" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(uint index, const std::vector<std::vector<float>>* points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearest = tree->search((*points)[index], distanceTol);

  	// add every point that the tree has found that is near initial point
	for(int id: nearest)
	{
		if(!processed[id])
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>* points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	// vector of booleans with size same as point cloud
	std::vector<bool> processed(points->size(), false);

	std::cout << "we have " << points->size() << " points." << std::endl;
	uint i = 0;
	while(i<points->size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		// this is proximity function in pseudo code
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
	}

	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //create KDTree
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_ind;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_ind);

    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_ind.begin(); it!=cluster_ind.end(); it++)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>());
        for(auto pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the cluster: " << cloud_cluster->width << " data points." << std::endl;
        // no need to write data away
        //std::sstream ss;
        //ss << "cloud_cluster_"  << j << ".pcd"

        clusters.push_back(cloud_cluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

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