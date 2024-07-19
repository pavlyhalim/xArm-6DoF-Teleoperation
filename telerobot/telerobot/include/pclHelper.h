#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/gicp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "eigenHelper.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class pclHelper
{
public:

    static bool isValid(const PointCloud::Ptr &cloud)
    {
        return (cloud->size() > 0) && cloud->is_dense;
    }

    static void removeOutliers(const PointCloud::ConstPtr &cloudIn, PointCloud::Ptr &cloudOut, double radius, int minNeighbours)
    {
        pcl::RadiusOutlierRemoval<Point> sor;
        sor.setInputCloud(cloudIn);
        sor.setRadiusSearch(radius);
        sor.setMinNeighborsInRadius(minNeighbours);
        sor.filter(*cloudOut);
    }

    static void downsample(const PointCloud::ConstPtr &cloudIn, PointCloud::Ptr &cloudOut, float leafSize)
    {
        pcl::VoxelGrid<Point> sor;
        sor.setInputCloud (cloudIn);
        sor.setLeafSize (leafSize, leafSize, leafSize);
        sor.filter (*cloudOut);
    }

    static void registerGICP(const PointCloud::ConstPtr &cloudSource, const PointCloud::Ptr &cloudTarget, Eigen::Vector3f *offset, Eigen::Quaternionf *rotation)
    {
        pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
        gicp.setInputSource(cloudSource);
        gicp.setInputTarget(cloudTarget);
        gicp.setMaxCorrespondenceDistance(0.2);
        PointCloud dummy;
        gicp.align(dummy);

        Eigen::Matrix4f transformation = gicp.getFinalTransformation();

        eigenHelper::toVectorQuaternion(transformation,offset,rotation);
    }

    static void transform(const PointCloud::Ptr &cloudIn, PointCloud::Ptr &cloudOut, std::string source_frame_id, std::string target_frame_id)
    {
        tf::TransformListener tf_listener;
        tf::StampedTransform transform;

        if(tf_listener.waitForTransform(target_frame_id, source_frame_id,ros::Time(0),ros::Duration(1.0)))
        {
            tf_listener.lookupTransform(target_frame_id, source_frame_id,ros::Time(0), transform);
            Eigen::Vector3f offset;
            offset << transform.getOrigin().x(),
                      transform.getOrigin().y(),
                      transform.getOrigin().z();

            Eigen::Quaternionf rotation;
            rotation = Eigen::Quaternionf(transform.getRotation().w(),\
                                          transform.getRotation().x(),\
                                          transform.getRotation().y(),\
                                          transform.getRotation().z());

            pcl::transformPointCloud(*cloudIn,*cloudOut,offset,rotation);
        }
    }

    static void transformSet(const std::vector<PointCloud::Ptr> cloudsIn, std::vector<PointCloud::Ptr> *cloudsOut, std::string source_frame_id, std::string target_frame_id)
    {
        for(int i = 0; i<cloudsIn.size(); i++)
        {
            PointCloud::Ptr cloud_transformed (new PointCloud);
            pclHelper::transform(cloudsIn.at(i),cloud_transformed,source_frame_id,target_frame_id);

            cloudsOut->push_back(cloud_transformed);
        }
    }

    static void color(PointCloud::Ptr &cloud, int red, int green, int blue)
    {
        PointCloud::iterator it;

        for(it = cloud->points.begin(); it < cloud->points.end(); it++){
            it->r = red;
            it->g = green;
            it->b = blue;
        }
    }

    static void estimateGroundPlaneTransformation(const PointCloud::Ptr &cloud, Eigen::Matrix4f *transformation)
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<Point> seg;

        // Optional
        seg.setOptimizeCoefficients (true);

        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.02);
        seg.setAxis(Eigen::Vector3f::UnitZ());

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cout << "Ground plane not found ..." << std::endl;
        }

        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];

        eigenHelper::transformationBetweenTwoPlanes(a,b,c,d,0,0,1,0,transformation);
    }
};


