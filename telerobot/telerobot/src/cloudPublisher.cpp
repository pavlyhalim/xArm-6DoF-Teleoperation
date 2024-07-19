#include "camNames.h"
#include "cloudPublisher.h"

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

cloudPublisher::cloudPublisher()
{
    //frames
    full_frame_id_ = "Full";
    asus_1_frame_id_ = CAM1;
    asus_2_frame_id_ = CAM2;

    //subscriber
    asus_1_sub_ = cloudPublisher_nh_.subscribe<PointCloud>\
            ("/" + asus_1_frame_id_ + "/depth/points", 1, &cloudPublisher::asus1Callback,this);
    asus_2_sub_ = cloudPublisher_nh_.subscribe<PointCloud >\
            ("/" + asus_2_frame_id_ + "/depth/points", 1, &cloudPublisher::asus2Callback,this);

    //publisher
    full_pub_ = cloudPublisher_nh_.advertise<PointCloud>("/" + full_frame_id_ + "/depth/points/calibrated", 1);
    asus_1_pub_ = cloudPublisher_nh_.advertise<PointCloud>("/" + asus_1_frame_id_ + "/depth/points/calibrated", 1);
    asus_2_pub_ = cloudPublisher_nh_.advertise<PointCloud>("/" + asus_2_frame_id_ + "/depth/points/calibrated", 1);

    //pointclouds
    asus_1_cloud_calib_ = PointCloud::Ptr(new PointCloud());
    asus_2_cloud_calib_ = PointCloud::Ptr(new PointCloud());
}

void cloudPublisher::publishClouds()
{
    //transform pointclouds in full frame
    PointCloud::Ptr asus_1_cloud_transformed(new PointCloud());
    //pclHelper::transform(asus_1_cloud_calib_,asus_1_cloud_transformed,asus_1_frame_id_,full_frame_id_);
    pcl::transformPointCloud(*asus_1_cloud_calib_,*asus_1_cloud_transformed,transf_Kinect2.inverse()*transf_Kinect1);

    PointCloud::Ptr asus_2_cloud_transformed(new PointCloud());
    //pclHelper::transform(asus_2_cloud_calib_,asus_2_cloud_transformed,asus_2_frame_id_,full_frame_id_);
    //pcl::transformPointCloud(*asus_2_cloud_calib_,*asus_2_cloud_transformed,transf_Kinect2);
    *asus_2_cloud_transformed = *asus_2_cloud_calib_ ;

    //add pointclouds
    PointCloud::Ptr full_cloud_calib(new PointCloud());
    *full_cloud_calib = *asus_1_cloud_transformed + *asus_2_cloud_transformed;

    //publish full cloud
    full_cloud_calib->header.frame_id = full_frame_id_;
    full_pub_.publish(full_cloud_calib);

    //publish asus 1 cloud
    asus_1_cloud_calib_->header.frame_id = asus_1_frame_id_;
    asus_1_pub_.publish(asus_1_cloud_calib_);

    //publish asus 2 cloud
    asus_2_cloud_calib_->header.frame_id = asus_2_frame_id_;
    asus_2_pub_.publish(asus_2_cloud_calib_);
}

void cloudPublisher::asus1Callback(const PointCloud::ConstPtr &cloud)
{

    PointCloud::Ptr cloud_temp(new PointCloud());
    pclHelper::downsample(cloud, asus_1_cloud_calib_, 0.01f);

    //pclHelper::removeOutliers(cloud_temp,asus_1_cloud_calib_,0.15,10);

    //pclHelper::color(asus_1_cloud_calib_,255,0,0);


}

void cloudPublisher::asus2Callback(const PointCloud::ConstPtr &cloud)
{
    PointCloud::Ptr cloud_temp(new PointCloud());
    pclHelper::downsample(cloud, asus_2_cloud_calib_, 0.01f);

    //pclHelper::removeOutliers(cloud_temp,asus_2_cloud_calib_,0.15,10);

    //pclHelper::color(asus_2_cloud_calib_,0,0,255);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "cloudPublisher");

    cloudPublisher cP;

    std::string pre_Kinect1 = "/poses/" + CAM1;
    getTransform(cP.cloudPublisher_nh_, pre_Kinect1, cP.transf_Kinect1);

    std::string pre_Kinect2 = "/poses/" + CAM2;
    getTransform(cP.cloudPublisher_nh_, pre_Kinect2, cP.transf_Kinect2);

    ros::Rate rate(30);

    while(ros::ok())
    {
        cP.publishClouds();
        ros::spinOnce();
        rate.sleep();
    }
}
