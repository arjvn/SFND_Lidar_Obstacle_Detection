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
	return pointProcessor.loadPcd("../../../sensors/data/pcd/data_1/0000000000.pcd");
	// return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
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

	// TODO: Fill in this function
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while (maxIterations--) {
		std::cout <<'\n' << "iteration: " << maxIterations << '\n';
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while (inliers.size() < 2) { //we want to create two unique points. use set for uniqueness
			std::cout << "randomly created inlier array: ";
			inliers.insert(rand() % (cloud->points.size()));
			for (auto& elm: inliers){
				std::cout << elm << " ";
			}
			std::cout << endl;
		}
		float x1, y1, x2, y2;
		auto itr = inliers.begin();
		// Randomly pick two consecutive points
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		std::cout << "itr 1: "<< *itr << '\n';
		std::cout << "x1, y1: " << x1 << ", " << y1 << '\n';
		itr++; // this iterates through inliers array hence giving us the 2 randomly generted points
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		std::cout << "itr 2: "<< *itr << '\n';
		std::cout << "x2, y2: " << x2 << ", " << y2 << '\n';
		// Generate line between these points
		float a = (y1 - y2);
		float b = (x2 - x1);
		float c = (x1 * y2 - x2 *y1);

		for (int index = 0; index < cloud->points.size(); index++) {
			if (inliers.count(index) > 0) {
				//inliers.count() => This function returns 1 if the element is present in the container otherwise it returns 0.
				std::cout << "index: " << index << " has already been tested..? continue" <<'\n';
				continue;
			}

			std::cout << "index: " << index << " testing" <<'\n';
			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			// Measure distance between every point and fitted line
			float d = (fabs(a*x3 + b*y3 + c))/(sqrt(a*a+b*b)); //use fabs() instead of abs() so as to account for the floats
			std::cout << "distance to line: " << d << '\n';
			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol) {
				inliers.insert(index);
				std::cout << "INLIER FOUND. x, y: "  << x3 << ", " << y3 << '\n';
				std::cout << "inliers size: " << inliers.size() << '\n';
				std::cout << "inlierResult size: " << inliersResult.size() << '\n';
			}
			else {
				std::cout << "INLIER NOT FOUND" << '\n';
			}
			// Return indicies of inliers from fitted line with most inliers
			if (inliers.size() > inliersResult.size()) {
				std::cout << "inlier array updated" << '\n';
				inliersResult = inliers;
			}
		}
	}

	return inliersResult;

}

std::unordered_set<int> PlaneRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	// TODO: Fill in this function
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while (maxIterations--) {
		std::cout <<'\n' << "iteration: " << maxIterations << '\n';
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) { //we want to create two unique points. use set for uniqueness
			std::cout << "randomly created inlier array: ";
			inliers.insert(rand() % (cloud->points.size()));
			for (auto& elm: inliers){
				std::cout << elm << " ";
			}

			std::cout << endl;
		}
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		// Randomly pick two consecutive points
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		std::cout << "itr 1: "<< *itr << '\n';
		std::cout << "x1, y1, z1: " << x1 << ", " << y1 << ", " << z1 << '\n';
		itr++; // this iterates through inliers array hence giving us the 2 randomly generted points
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		std::cout << "itr 2: "<< *itr << '\n';
		std::cout << "x2, y2, z2: " << x2 << ", " << y2 << ", " << z2 << '\n';
		itr++; // this iterates through inliers array hence giving us the 2 randomly generted points
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		std::cout << "itr 3: "<< *itr << '\n';
		std::cout << "x3, y3, z3: " << x3 << ", " << y3 << ", " << z3 << '\n';
		// Generate line between these points
		float a = (y2 - y1)*(z3 - z1)-(y3 - y1)*(z2 - z1);
		float b = (z2 - z1)*(x3 - x1)-(z3 - z1)*(x2 - x1);
		float c = (x2 - x1)*(y3 - y1)-(x3 - x1)*(y2 - y1);
		float d = -(a*x1 + b*y1 + c*z1);

		for (int index = 0; index < cloud->points.size(); index++) {
			if (inliers.count(index) > 0) {
				//inliers.count() => This function returns 1 if the element is present in the container otherwise it returns 0.
				std::cout << "index: " << index << " has already been tested. continue" <<'\n';
				continue;
			}

			std::cout << "index: " << index << " testing" <<'\n';
			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			// Measure distance between every point and fitted line
			float dist = (fabs(a*x4 + b*y4 + c*z4 + d))/(sqrt(a*a + b*b + c*c)); //use fabs() instead of abs() so as to account for the floats
			std::cout << "distance to line: " << dist << '\n';
			// If distance is smaller than threshold count it as inlier
			if (dist <= distanceTol) {
				inliers.insert(index);
				std::cout << "INLIER FOUND. x, y: "  << x4 << ", " << y4 << '\n';
				std::cout << "inliers size: " << inliers.size() << '\n';
				std::cout << "inlierResult size: " << inliersResult.size() << '\n';
			}
			else {
				std::cout << "INLIER NOT FOUND" << '\n';
			}
			// Return indicies of inliers from fitted line with most inliers
			if (inliers.size() > inliersResult.size()) {
				std::cout << "inlier array updated" << '\n';
				inliersResult = inliers;
			}
		}
	}

	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = PlaneRansac(cloud, 10, 0.5);

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
