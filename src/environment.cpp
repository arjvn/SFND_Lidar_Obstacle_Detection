/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "3Dkdtree.h"

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &rawCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

// RENDERING PARAMETERS
  bool render_PointCloud = false;
  bool render_inputPointCloud = false;
  bool render_obst = false;
  bool render_plane = true;
  bool render_cluster = true;
  bool render_box = true;

// FilterCloud PARAMETERS
  float filterRes = 0.3;
  Eigen::Vector4f minPoint(-20, -6, -2, 1);
  Eigen::Vector4f maxPoint(40, 7, 2, 1);

// RANSAC PARAMETERS
  int MaxIterations = 50;
  float DistanceTol = 0.3;

// CLUSTERING PARAMETERS
  float ClusterTol = 0.45;
  int minSize = 5;
  int maxSize = 300;

  // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  // pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  // STEP 1: Filter Cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->FilterCloud(rawCloud, filterRes, minPoint, maxPoint);
  if (render_PointCloud) {
      renderPointCloud(viewer, rawCloud, "rawCloud", Color(1, 1, 1));
  }
  if (render_inputPointCloud) {
      renderPointCloud(viewer, inputCloud, "inputCloud");
  }

  // STEP 2: Apply RANSAC
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->PlaneRansac(inputCloud, MaxIterations, DistanceTol);

  if (render_obst) {
      renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(0, 1, 1));
  }
  if (render_plane) {
      renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(1, 0, 0));
  }

  // STEP 3: Clustering
  // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, ClusterTol, minSize, maxSize);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EclideanClustering(segmentCloud.first, ClusterTol, minSize, maxSize);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(0.3, 1, 1), Color(1, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
      if (render_cluster) {
          renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
      }
      if (render_box) {
          Box box = pointProcessorI->BoundingBox(cluster);
          renderBox(viewer, box, clusterId);
      }
      ++clusterId;
  }

  std::cout << "render complete" << '\n';
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_PointCloud = true;
    bool render_obst = true;
    bool render_plane = true;
    bool render_cluster = true;
    bool render_box = true;


    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0); //Pass LiDAR struct the vector <car> and grd incline
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    if (render_PointCloud) {
        renderPointCloud(viewer, inputCloud, "inputCloud");
    }

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    if (render_obst) {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    }
    if (render_plane) {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2.0, 3, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        if (render_cluster) {
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }
        if (render_box) {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }

    std::cout << "render complete" << '\n';
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

// For single PCD files
    // simpleHighway(viewer);
    // cityBlock(viewer);
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // }

  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloud;

  while (!viewer->wasStopped()) {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();
      // Load pcd and run obstacle detection process
      rawCloud = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, rawCloud);
      streamIterator++;
      if (streamIterator == stream.end()) {
          streamIterator = stream.begin();
      }
      viewer->spinOnce();
  }
}
