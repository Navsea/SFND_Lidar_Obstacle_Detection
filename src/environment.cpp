/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // create lidar sensor exersize
    Lidar* myLidar = new Lidar(cars, 0);
    auto myPointCloud = myLidar->scan();
    //renderRays(viewer, myLidar->position, myPointCloud);

    // segmenting exersize
    auto myPointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedPlane;

    segmentedPlane = myPointProcessor->SegmentPlane(myPointCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentedPlane.first, "Road", Color(0,1,0));
    //renderPointCloud(viewer, segmentedPlane.second, "Obstacles", Color(1,0,0));

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloud)
{
    //pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, pointCloud, "loaded_data");

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(pointCloud, 0.2, Eigen::Vector4f(-10,-6,-2,1), Eigen::Vector4f(30,6,5,1));
    std::cout << "After filtering: " << filteredCloud->points.size () << " data points" << std::endl;
    //renderPointCloud(viewer, filteredCloud, "filteredCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPlane;
    segmentedPlane = pointProcessor->SegmentPlaneRansac(filteredCloud, 50, 0.16);
    renderPointCloud(viewer, segmentedPlane.first, "Road", Color(0,1,0));
    renderPointCloud(viewer, segmentedPlane.second, "Obstacles", Color(1,0,0));

  // convert the pointcloud of objects to a vector of (vector of floats=point coords) to do clustering
  std::vector<std::vector<float>>* pointCloud_v = new std::vector<std::vector<float>>();
  // run through every point in the cloud
  for ( int i=0; i < segmentedPlane.second->points.size(); i++)
  {
    pcl::PointXYZI point = segmentedPlane.second->points[i];
    std::vector<float> point_v = { point.x, point.y, point.z };
  	pointCloud_v->push_back(point_v);
  }
    // print out sizes of point cloud and of vector of points
    std::cout << "segmentedPlane.second: " << segmentedPlane.second->points.size() << " pointCloud_v: " << pointCloud_v->size() << std::endl;

    
//     build kdtree
    KdTree* tree = new KdTree;
  
    for (int i=0; i<segmentedPlane.second->points.size(); i++) 
    	tree->insert((*pointCloud_v)[i],i); 

//     clustering  
//     Time segmentation process
   	auto startTime = std::chrono::steady_clock::now();
  	
   	std::vector<std::vector<int>> clusters = pointProcessor->euclideanCluster(pointCloud_v, tree, 0.6);
  	
   	auto endTime = std::chrono::steady_clock::now();
   	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
   	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
  
  
//     Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster)
        {
            pcl::PointXYZI newPoint = pcl::PointXYZI();
            newPoint.x = (*pointCloud_v)[indice][0];
            newPoint.y = (*pointCloud_v)[indice][1];
            newPoint.z = (*pointCloud_v)[indice][2];
            newPoint.intensity = pointCloud->points[indice].intensity;
  			clusterCloud->points.push_back(newPoint);
        }
        Box bbox = pointProcessor->BoundingBox(clusterCloud);
        viewer->addCube(bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max, bbox.z_min, bbox.z_max, colors[clusterId%3].r, colors[clusterId%3].g, colors[clusterId%3].b, "bbox"+std::to_string(clusterId));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,segmentedPlane.second,"data");
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    // returns a vector of chronologically ordered pcd files names
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    //cityBlock(viewer, pointProcessor, inputCloud);

    while (!viewer->wasStopped ())
    {
        //viewer->spinOnce ();

        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd and run obstacle detection process
        inputCloud = pointProcessor->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloud);

        streamIterator++;
        // restart recording
        if(streamIterator==stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}