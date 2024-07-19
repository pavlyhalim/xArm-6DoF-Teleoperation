#include "pclFilter.h"
#include <pcl/filters/crop_box.h>



/*
 * Filters pointcloud. Cuts all points outside a freely rotated and positioned cuboid.
 *
 * Input:
 * origin: Midpoint of cuboid.
 * orientation: vector of the cuboids orientation from the midpoint.
 * min(0) to  max(0): length
 * min(1) to  max(1): width
 * min(2) to  max(2): depth
 *
 */

void cuboidFilterCustom(Eigen::Vector3d origin, Eigen::Vector3d orientation, Eigen::Vector4d min, Eigen::Vector4d max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr) {
 
   Eigen::Matrix3d rotMat; 
   orientation.normalize();

   rotMat(0,2) = orientation(0);
   rotMat(1,2) = orientation(1);
   rotMat(2,2) = orientation(2);
   Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
   Eigen::Vector3d normal;
   normal = zAxis.cross(orientation);
   normal.normalize();
   rotMat(0,0) = normal(0);
   rotMat(1,0) = normal(1);
   rotMat(2,0) = normal(2);

   Eigen::Vector3d normal2;
   normal2 = orientation.cross(normal);

   rotMat(0,1) = normal2(0);
   rotMat(1,1) = normal2(1);
   rotMat(2,1) = normal2(2);

   double alpha; 
   double beta; 
   double gamma;

   beta = atan2(-rotMat(2,0), sqrt(rotMat(0,0)*rotMat(0,0) + rotMat(1,0)*rotMat(1,0))); 

   alpha = atan2(rotMat(1,0)/cos(beta), rotMat(0,0)/cos(beta)); 

   gamma = atan2(rotMat(2,1)/cos(beta), rotMat(2,2)/cos(beta));

   Eigen::Vector4f minF = min.cast<float>();
   Eigen::Vector4f maxF = max.cast<float>();
   Eigen::Vector3f originF = origin.cast<float>();

   pcl::CropBox<pcl::PointXYZRGB> cropFilter;
   cropFilter.setInputCloud (cloudPtr);
   cropFilter.setMin(minF);
   cropFilter.setMax(maxF);
   cropFilter.setKeepOrganized(true);
   cropFilter.setTranslation(originF);

   Eigen::Vector3d rotation(gamma, beta, alpha);
   Eigen::Vector3f rotationF = rotation.cast<float>();
   cropFilter.setRotation(rotationF);

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   cropFilter.filter(*cloudOutPtr);

   *cloudPtr = *cloudOutPtr;
}
