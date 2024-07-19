#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "GetOpenniTrackerData.h"
#include "pclFilter.h"

//#include <boost/foreach.hpp>

//#define PI 3.1415926

#include "pclHelper.h"
#include "camNames.h"

using namespace ros;
//using namespace sensor_msgs;

void getTransform(const ros::NodeHandle &node_handle, const std::string &pre, Eigen::Matrix4f &transf){

    Eigen::Matrix3f rot;
    transf = Eigen::Matrix4f::Identity(4,4);

    std::cout << pre + "/translation/x" <<std::endl;

    double translation_x;
    node_handle.getParam(pre + "/translation/x", translation_x);
    double translation_y;
    node_handle.getParam(pre + "/translation/y", translation_y);
    double translation_z;
    node_handle.getParam(pre + "/translation/z", translation_z);

    double rotation_x;
    node_handle.getParam(pre + "/rotation/x", rotation_x);
    double rotation_y;
    node_handle.getParam(pre + "/rotation/y", rotation_y);
    double rotation_z;
    node_handle.getParam(pre + "/rotation/z", rotation_z);
    double rotation_w;
    node_handle.getParam(pre + "/rotation/w", rotation_w);

    Eigen::Quaternionf quat(rotation_w, rotation_x, rotation_y, rotation_z );
    rot = quat.toRotationMatrix();

    for(int i=0; i<3;i++) {
        for(int j = 0; j<3;j++) {
         transf(i,j) = rot(i,j);
        }
    }
    transf(0,3) = translation_x;
    transf(1,3) = translation_y;
    transf(2,3) = translation_z;
}

Eigen::Vector3d originShoulder(0.0, 0.0, 0.0);
Eigen::Vector3d directionShoulder(0.0, 0.0, 0.0);
Eigen::Vector3d originElbow(0.0, 0.0, 0.0);
Eigen::Vector3d directionElbow(0.0, 0.0, 0.0);
Eigen::Vector3d originHand(0.0, 0.0, 0.0);

/*
 * Class to receive Point cloud data.
 */
class processPoint {

    ros::Publisher *publisher;

public:
    //Transformation-Matrizes from calibration files
    Eigen::Matrix4f transf_Kinect1;
    Eigen::Matrix4f transf_Kinect2;

    //frames
    std::string full_frame_id_;
    std::string asus_1_frame_id_;
    std::string asus_2_frame_id_;

    //publisher
    ros::Publisher full_pub_;
    ros::Publisher asus_1_pub_;
    ros::Publisher asus_2_pub_;

    //subscriber
    ros::Subscriber asus_1_sub_;
    ros::Subscriber asus_2_sub_;

    //pointclouds
    PointCloud::Ptr asus_1_cloud_calib_;
    PointCloud::Ptr asus_2_cloud_calib_;

    processPoint() {
    }

    ~processPoint() {
     }

    void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud );
    void publishClouds();

    //callbacks
    void asus1Callback(const PointCloud::ConstPtr& cloud);
    void asus2Callback(const PointCloud::ConstPtr& cloud);

};

void processPoint::asus1Callback(const PointCloud::ConstPtr &cloud)
{

    PointCloud::Ptr cloud_temp(new PointCloud());

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setKeepOrganized(true);
    pass.setInputCloud(cloud); //cloud_downsampled
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,2.0); // unit : meter
    pass.filter(*asus_1_cloud_calib_);


    //pclHelper::downsample(cloud, asus_1_cloud_calib_, 0.01f);

    //pclHelper::removeOutliers(cloud_temp,asus_1_cloud_calib_,0.15,10);

    //pclHelper::color(asus_1_cloud_calib_,255,0,0);


}

void processPoint::asus2Callback(const PointCloud::ConstPtr &cloud)
{
    PointCloud::Ptr cloud_temp(new PointCloud());

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setKeepOrganized(true);
    pass.setInputCloud(cloud); //cloud_downsampled
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,2.0); // unit : meter
    pass.filter(*asus_2_cloud_calib_);

    //pclHelper::downsample(cloud, asus_2_cloud_calib_, 0.01f);

    //pclHelper::removeOutliers(cloud_temp,asus_2_cloud_calib_,0.15,10);

    //pclHelper::color(asus_2_cloud_calib_,0,0,255);
}

void processPoint::publishClouds()
{
    //transform pointclouds in full frame
    PointCloud::Ptr asus_1_cloud_transformed(new PointCloud());
    //pclHelper::transform(asus_1_cloud_calib_,asus_1_cloud_transformed,asus_1_frame_id_,full_frame_id_);
    pcl::transformPointCloud(*asus_1_cloud_calib_,*asus_1_cloud_transformed,transf_Kinect2.inverse()*transf_Kinect1);
    processCloud(asus_1_cloud_transformed);

    //PointCloud::Ptr asus_2_cloud_transformed(new PointCloud());
    //pclHelper::transform(asus_2_cloud_calib_,asus_2_cloud_transformed,asus_2_frame_id_,full_frame_id_);
    //pcl::transformPointCloud(*asus_2_cloud_calib_,*asus_2_cloud_transformed,transf_Kinect2);
    //*asus_2_cloud_transformed = *asus_2_cloud_calib_ ;
    //processCloud(asus_2_cloud_transformed);

    //add pointclouds
    PointCloud::Ptr full_cloud_calib(new PointCloud());
    *full_cloud_calib = *asus_1_cloud_transformed;// + *asus_2_cloud_transformed;

    //publish full cloud
    full_cloud_calib->header.frame_id = "/camera_rgb_optical_frame";
    full_pub_.publish(full_cloud_calib);

    //publish asus 1 cloud
    //asus_1_cloud_calib_->header.frame_id = asus_1_frame_id_;
    //asus_1_pub_.publish(asus_1_cloud_calib_);

    //publish asus 2 cloud
    //asus_2_cloud_calib_->header.frame_id = asus_2_frame_id_;
    //asus_2_pub_.publish(asus_2_cloud_calib_);
}

void processPoint::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud ) {

//    std::cout << "Size of Input Point-Cloud: " << cloud->points.size () << std::endl;

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new 	pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new 	pcl::PointCloud<pcl::PointXYZRGB> ());

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassFilter (new 	pcl::PointCloud<pcl::PointXYZRGB> ());
//    cloudPassFilter = cloud_filtered;

//    pcl::PassThrough<pcl::PointXYZRGB> pass;
//    pass.setKeepOrganized(true);
//    pass.setInputCloud(cloud); //cloud_downsampled
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(0.0,2.0); // unit : meter
//    pass.filter(*cloud_filtered);

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCuboidFilter (new 	pcl::PointCloud<pcl::PointXYZRGB> ());
//    cloudCuboidFilter = cloud_filtered;

    Eigen::Vector4d max_pt(0.20, 0.20, 0.20, 1.0);
    Eigen::Vector4d min_pt(-0.20, -0.20, -0.20, 1.0);
    cuboidFilterCustom(originHand, directionElbow, min_pt, max_pt, cloud);
//    cloud_filtered = cloudCuboidFilter;

//    std::cout << "Size of Cuboid-Filtered Point-Cloud: " << cloud_filtered->points.size () << std::endl;

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudBilateralFilter (new 	pcl::PointCloud<pcl::PointXYZRGB> ());
//    cloudBilateralFilter = cloud_filtered;

//    pcl::FastBilateralFilter<pcl::PointXYZRGB> fbFilter;
//    fbFilter.setInputCloud(cloudBilateralFilter);
//    fbFilter.setSigmaR(0.03f); // standard deviation of the Gaussian for the intensity difference = 0.05f (-->pixel is downweighted)
//    fbFilter.setSigmaS(4.5f); // spatial neighborhood/window = 15.0f
//    fbFilter.applyFilter(*cloud_filtered);

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVoxelFilter (new 	pcl::PointCloud<pcl::PointXYZRGB> ());
//    cloudVoxelFilter = cloud_filtered;

//    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//    sor.setInputCloud (cloudVoxelFilter);
//    sor.setLeafSize (0.008f, 0.008f, 0.008f);
//    sor.filter (*cloud_filtered);

    //transform to origin
    Eigen::Matrix4d transformMat;

    Eigen::Vector3d handCS_Z;

    handCS_Z <<  directionElbow(0), directionElbow(1), directionElbow(2);
    handCS_Z.normalize();

    Eigen::Vector3d handCS_X = (directionShoulder).cross(directionElbow).normalized();

    Eigen::Vector3d handCS_Y = (handCS_Z).cross(handCS_X).normalized();

    transformMat << handCS_X(0), handCS_Y(0), handCS_Z(0), originHand(0),
                    handCS_X(1), handCS_Y(1), handCS_Z(1), originHand(1),
                    handCS_X(2), handCS_Y(2), handCS_Z(2), originHand(2),
                    0,           0,           0,           1;

    Eigen::Matrix4d rotationX90Mat;
    rotationX90Mat << 1.0, 0.0,  0.0, 0.0,
                      0.0, 0.0, -1.0, 0.0,
                      0.0, 1.0,  0.0, 0.0,
                      0.0, 0.0,  0.0, 1.0;

    Eigen::Matrix4d rotationZ90Mat;
    rotationZ90Mat << 0.0, -1.0, 0.0, 0.0,
                      1.0,  0.0, 0.0, 0.0,
                      0.0,  0.0, 1.0, 0.0,
                      0.0,  0.0, 0.0, 1.0;

    Eigen::Matrix4d scale;
    scale <<          0.9,  0.0, 0.0, 0.0,
                      0.0,  0.9, 0.0, 0.0,
                      0.0,  0.0, 0.9, 0.0,
                      0.0,  0.0, 0.0, 1.0;

    Eigen::Matrix4d translatMat;
    translatMat <<    1.0,  0.0, 0.0,   0.0,
                      0.0,  1.0, 0.0,   -0.1,
                      0.0,  0.0, 1.0,   0.0,
                      0.0,  0.0, 0.0,   1.0;

    Eigen::Matrix4d G_temp;
    G_temp = translatMat*rotationX90Mat*rotationZ90Mat*transformMat.inverse();
    //G_temp = identity;

    Eigen::Matrix4f G = G_temp.cast <float> ();

    //pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, G);
    pcl::transformPointCloud(*cloud, *cloud_transformed, G);
    *cloud = *cloud_transformed;

    //cloud_transformed->header.frame_id = "/camera_rgb_optical_frame";
    //publisher->publish(cloud_transformed);

//    std::cout << "Size of Output Point-Cloud: " << cloud->points.size () << std::endl;

}

int main(int argc, char **argv) {
    
    init(argc, argv, "processPoint");

    NodeHandle nh;

    processPoint processP;

    //ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("hp_cloud", 1);
    //ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, &processPoint::processCloud, &processP);

    //ros::Subscriber sub = nh.subscribe("/Full/depth/points/calibrated", 1, &processPoint::processCloud, &processP);

    //processP.setPublisher(&pub);

    /* Get Calibration poses */
    std::string pre_Kinect1 = "/poses/" + CAM1;
    getTransform(nh, pre_Kinect1, processP.transf_Kinect1);

    std::string pre_Kinect2 = "/poses/" + CAM2;
    getTransform(nh, pre_Kinect2, processP.transf_Kinect2);

    /* Init Subscriber and Publisher */
    //frames
    processP.full_frame_id_ = "Full";
    processP.asus_1_frame_id_ = CAM1;
    processP.asus_2_frame_id_ = CAM2;

    //subscriber
    processP.asus_1_sub_ = nh.subscribe<PointCloud>("/" + processP.asus_1_frame_id_ + "/depth/points", 1, &processPoint::asus1Callback, &processP);
    //processP.asus_2_sub_ = nh.subscribe<PointCloud>("/" + processP.asus_2_frame_id_ + "/depth/points", 1, &processPoint::asus2Callback, &processP);

    //publisher
    processP.full_pub_ = nh.advertise<PointCloud>("/hp_cloud", 1);
    processP.asus_1_pub_ = nh.advertise<PointCloud>("/" + processP.asus_1_frame_id_ + "/depth/points/calibrated", 1);
    processP.asus_2_pub_ = nh.advertise<PointCloud>("/" + processP.asus_2_frame_id_ + "/depth/points/calibrated", 1);

    //pointclouds
    processP.asus_1_cloud_calib_ = PointCloud::Ptr(new PointCloud());
    processP.asus_2_cloud_calib_ = PointCloud::Ptr(new PointCloud());


    GetOpenniTrackerData getOpenniData;

    Eigen::Vector3d shoulderPoint;
    Eigen::Vector3d elbowPoint;
    Eigen::Vector3d handPoint;

    Eigen::Vector3d upperarm;
    Eigen::Vector3d forearm;

    Eigen::Matrix3d mapTF;
    mapTF << 0, -1, 0,
             0,  0, -1,
             1,  0,  0;

    Rate spin_rate(30);

    while( ok() ) {

     shoulderPoint = getOpenniData.getShoulderPoint();
     elbowPoint = getOpenniData.getElbowPoint();
     handPoint = getOpenniData.getHandPoint();

     shoulderPoint = mapTF*shoulderPoint;
     elbowPoint = mapTF*elbowPoint;
     handPoint = mapTF*handPoint;

     upperarm = -shoulderPoint + elbowPoint;
     forearm = -elbowPoint + handPoint;

     originShoulder = shoulderPoint;
     directionShoulder = upperarm;
     originElbow = elbowPoint;
     directionElbow = forearm;
     originHand = handPoint;

     processP.publishClouds();

     spinOnce();

     spin_rate.sleep();

    }



return 0;

}
