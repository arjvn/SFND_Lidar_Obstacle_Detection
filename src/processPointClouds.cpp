// PCL lib Functions for processing point clouds
#include <unordered_set>
#include <cstdlib>
#include <chrono>
#include <string>
#include <pcl/impl/point_types.hpp>
#include <vector>

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
    pcl::VoxelGrid<PointT> Voxel;
    typename pcl::PointCloud<PointT>::Ptr filterCloud (new pcl::PointCloud<PointT>);
    Voxel.setInputCloud (cloud);
    Voxel.setLeafSize (filterRes, filterRes, filterRes);
    Voxel.filter (*filterCloud);

    // apply crop box
    // typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> InterestRegion(true);
    InterestRegion.setMax(maxPoint);
    InterestRegion.setMin(minPoint);
    InterestRegion.setInputCloud(filterCloud);
    InterestRegion.filter(*filterCloud);

    std::vector<int> indices;
    pcl::CropBox<PointT> EgoRoof(true);
    EgoRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    EgoRoof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    EgoRoof.setInputCloud(filterCloud);
    EgoRoof.filter(indices);
    numPoints(filterCloud);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filterCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filterCloud);

    numPoints(filterCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filterCloud;

}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);

  for(int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Extract to get obstacles
    // Inliers are the road points

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    // TODO:: Fill in this function to find inliers for the cloud.



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  // TODO: Fill in this function
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while (maxIterations--) {
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) { //we want to create two unique points. use set for uniqueness
			inliers.insert(rand() % (cloud->points.size()));
		}
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		// Randomly pick two consecutive points
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++; // this iterates through inliers array hence giving us the 2 randomly generted points
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++; // this iterates through inliers array hence giving us the 2 randomly generted points
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		// Generate line between these points
		float a = (y2 - y1)*(z3 - z1)-(y3 - y1)*(z2 - z1);
		float b = (z2 - z1)*(x3 - x1)-(z3 - z1)*(x2 - x1);
		float c = (x2 - x1)*(y3 - y1)-(x3 - x1)*(y2 - y1);
		float d = -(a*x1 + b*y1 + c*z1);

		for (int index = 0; index < cloud->points.size(); index++) {
			if (inliers.count(index) > 0) {
				//inliers.count() => This function returns 1 if the element is present in the container otherwise it returns 0.
				continue;
			}
			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			// Measure distance between every point and fitted line
			float dist = (fabs(a*x4 + b*y4 + c*z4 + d))/(sqrt(a*a + b*b + c*c)); //use fabs() instead of abs() so as to account for the floats
			// If distance is smaller than threshold count it as inlier
			if (dist <= distanceTol) {
				inliers.insert(index);
			}
			// Return indicies of inliers from fitted line with most inliers
			if (inliers.size() > inliersResult.size()) {
				inliersResult = inliers;
			}
		}
	}
  typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>);

  for(int index = 0; index < cloud->points.size(); index++)
  {
    PointT point = cloud->points[index];
    if(inliersResult.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

  return std::pair< typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // For each cluster indice
    for (pcl::PointIndices getIndices: cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (int index:getIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }
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
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{
  processed[indice] = true;
  cluster.push_back(indice);

  std::vector<int> nearest = tree->search3D(cloud->points[indice], distanceTol);
  for (int id:nearest) {
    if (!processed[id]) {
        clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
    }
  }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::vector<bool> processed(cloud->points.size(), false);
	KdTree* tree (new KdTree());

  for (int i = 0; i < cloud->points.size(); i++)
  	tree->insert3D(cloud->points[i],i);

  for (int i = 0; i < cloud->points.size(); i++) {
      if (processed[i]) {
          continue;
      }
      std::vector<int> idxCluster;
      typename pcl::PointCloud<PointT>::Ptr identifiedCluster (new pcl::PointCloud<PointT>);
      clusterHelper(i, cloud, idxCluster, processed, tree, clusterTolerance);

      int cluster_size = idxCluster.size();
      if (cluster_size >= minSize && cluster_size <= maxSize) {
        for (int j = 0; j < cluster_size; j++) {
          identifiedCluster->points.push_back(cloud->points[idxCluster[j]]);
        }
        identifiedCluster->width = identifiedCluster->points.size();
        identifiedCluster->height = 1;
        clusters.push_back(identifiedCluster);
      }
}
  auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout <<'\n' << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  delete tree;
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
