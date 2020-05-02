/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliers;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// Randomly sample subset and fit line 
	std::cout << "the cloud has a size of: " << cloud->size() << endl;
	// For max iterations 
	for( int i = 0; i<maxIterations; i++)
	{
		// Select two indices in the range of cloud size
		while(inliers.size()<2)
		{
			inliers.insert(rand()%cloud->size());
		}
		// print the indices
		int j = 0;
		for(auto it = inliers.begin(); it!=inliers.end(); it++)
		{
			std::cout << "cycle " << j << " index: " << *it << endl;
			j++;
		}
		// extract the points
		pcl::PointXYZ p1, p2;
		auto it = inliers.begin();
		p1 = cloud->points[*it];
		it++;
		p2 = cloud->points[*it];
		// print the point data
		std::cout << "p1 x: " << p1.x << " y: " << p1.y << " z: " << p1.z << endl;
		std::cout << "p2 x: " << p2.x << " y: " << p2.y << " z: " << p2.z << endl;
	
		// Measure distance between every point and fitted line
		float A=0, B=0, C=0;
		A = p1.y - p2.y;
		B = p2.x - p1.x;
		C = p1.x*p2.y - p2.x*p1.y;
		std::cout << "A: " << A << " B: " << B << " C: " << C << endl;

		for(int i = 0; i < cloud->size(); i++)
		{
			// check if point is not one of the two samples
			if (inliers.find(i)!=inliers.end())
			{
				continue;
			}
			pcl::PointXYZ p3 = cloud->points[i];
			float distance = fabs(A*p3.x + B*p3.y + C) / sqrt(pow(A,2)+pow(B,2));
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


	// Return indicies of inliers from fitted line with most inlier
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliers;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// Randomly sample subset and fit line 
	std::cout << "the cloud has a size of: " << cloud->size() << endl;
	// For max iterations 
	for( int i = 0; i<maxIterations; i++)
	{
		// Select two indices in the range of cloud size
		while(inliers.size()<3)
		{
			inliers.insert(rand()%cloud->size());
		}
		// print the indices
		int j = 0;
		for(auto it = inliers.begin(); it!=inliers.end(); it++)
		{
			std::cout << "cycle " << j << " index: " << *it << endl;
			j++;
		}
		// extract the points
		pcl::PointXYZ p1, p2, p3;
		auto it = inliers.begin();
		p1 = cloud->points[*it];
		it++;
		p2 = cloud->points[*it];
		it++;
		p3 = cloud->points[*it];
		// print the point data
		std::cout << "p1 x: " << p1.x << " y: " << p1.y << " z: " << p1.z << endl;
		std::cout << "p2 x: " << p2.x << " y: " << p2.y << " z: " << p2.z << endl;
		std::cout << "p3 x: " << p3.x << " y: " << p3.y << " z: " << p3.z << endl;

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
		std::cout << "A: " << A << " B: " << B << " C: " << C << " D: " << D << endl;

		for(int i = 0; i < cloud->size(); i++)
		{
			// check if point is not one of the two samples
			if (inliers.find(i)!=inliers.end())
			{
				continue;
			}
			pcl::PointXYZ p4 = cloud->points[i];
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


	// Return indicies of inliers from fitted line with most inlier
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
