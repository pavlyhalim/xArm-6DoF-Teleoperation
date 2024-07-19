//#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

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
void cuboidFilterCustom(Eigen::Vector3d origin, Eigen::Vector3d orientation, Eigen::Vector4d min, Eigen::Vector4d max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr);
