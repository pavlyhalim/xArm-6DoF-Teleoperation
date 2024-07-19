#include "camNames.h"
#include "calibrationRefinement.h"

//Passthrough
double CAM1_Z;
double CAM2_Z;
bool registrationStart;

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

calibrationRefinement::calibrationRefinement()
{
    //frames
    full_frame_id_ = "Full";
    asus_1_frame_id_ = CAM1;
    asus_2_frame_id_ = CAM2;

    //subscriber
    asus_1_sub_ = calibrationRefinement_nh_.subscribe<PointCloud>\
            ("/" + asus_1_frame_id_ + "/depth/points", 1, &calibrationRefinement::asus1Callback,this);
    asus_2_sub_ = calibrationRefinement_nh_.subscribe<PointCloud >\
            ("/" + asus_2_frame_id_ + "/depth/points", 1, &calibrationRefinement::asus2Callback,this);

    //publisher
    full_pub_ = calibrationRefinement_nh_.advertise<PointCloud>("/" + full_frame_id_ + "/depth/points/calibrated", 1);
    asus_1_pub_ = calibrationRefinement_nh_.advertise<PointCloud>("/" + asus_1_frame_id_ + "/depth/points/calibrated", 1);
    asus_2_pub_ = calibrationRefinement_nh_.advertise<PointCloud>("/" + asus_2_frame_id_ + "/depth/points/calibrated", 1);

    //pointclouds
    asus_1_cloud_calib_ = PointCloud::Ptr(new PointCloud());
    asus_2_cloud_calib_ = PointCloud::Ptr(new PointCloud());
}

void calibrationRefinement::publishClouds()
{
    //transform pointclouds in full frame
    PointCloud::Ptr asus_1_cloud_transformed(new PointCloud());
    pcl::transformPointCloud(*asus_1_cloud_calib_,*asus_1_cloud_transformed,transf_Kinect1);

    PointCloud::Ptr asus_2_cloud_transformed(new PointCloud());
    pcl::transformPointCloud(*asus_2_cloud_calib_,*asus_2_cloud_transformed,transf_Kinect2);

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

void calibrationRefinement::registerClouds()
{

    //transform pointclouds in full frame
    PointCloud::Ptr asus_1_cloud_transformed(new PointCloud());
    pcl::transformPointCloud(*asus_1_cloud_calib_,*asus_1_cloud_transformed,transf_Kinect1);

    PointCloud::Ptr asus_2_cloud_transformed(new PointCloud());
    pcl::transformPointCloud(*asus_2_cloud_calib_,*asus_2_cloud_transformed,transf_Kinect2);

    pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
    gicp.setInputSource(asus_1_cloud_transformed);
    gicp.setInputTarget(asus_2_cloud_transformed);
    gicp.setMaxCorrespondenceDistance(0.3);
    PointCloud dummy;
    gicp.align(dummy);

    transf_Kinect1 = gicp.getFinalTransformation()*transf_Kinect1;

    saveTransformation();

}

void calibrationRefinement::asus1Callback(const PointCloud::ConstPtr &cloud)
{

    PointCloud::Ptr cloud_temp(new PointCloud());
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setKeepOrganized(true);
    pass.setInputCloud(cloud); //cloud_downsampled
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,CAM1_Z); // unit : meter
    pass.filter(*cloud_temp);

    PointCloud::Ptr cloud_temp2(new PointCloud());
    pclHelper::downsample(cloud_temp, cloud_temp2, 0.05f);

    pclHelper::removeOutliers(cloud_temp2,asus_1_cloud_calib_,0.15,10);
}

void calibrationRefinement::asus2Callback(const PointCloud::ConstPtr &cloud)
{
    PointCloud::Ptr cloud_temp(new PointCloud());
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setKeepOrganized(true);
    pass.setInputCloud(cloud); //cloud_downsampled
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,CAM2_Z); // unit : meter
    pass.filter(*cloud_temp);

    PointCloud::Ptr cloud_temp2(new PointCloud());
    pclHelper::downsample(cloud_temp, cloud_temp2, 0.05f);

    pclHelper::removeOutliers(cloud_temp2,asus_2_cloud_calib_,0.15,10);
}

void calibrationRefinement::saveTransformation()
{
    std::string file_name = ros::package::getPath("telerobot") + "/conf/camera_poses.yaml";
    std::ofstream file;
    file.open(file_name.c_str());

    if (file.is_open())
    {
        ros::Time time = ros::Time::now();
        file << "# Auto generated file." << std::endl;
        file << "calibration_id: " << time.sec << std::endl << std::endl;

        // Write TF transforms between cameras and world frame
        file << "# Poses w.r.t. the \"world\" reference frame" << std::endl;
        file << "poses:" << std::endl;

        file << "  " << asus_1_frame_id_ << ":" << std::endl;

        Eigen::Vector3f* translation = new Eigen::Vector3f;
        Eigen::Quaternionf* rotation = new Eigen::Quaternionf;
        eigenHelper::toVectorQuaternion(transf_Kinect1,translation,rotation);

        file << "    translation:" << std::endl
             << "      x: " << (*translation)(0) << std::endl
             << "      y: " << (*translation)(1) << std::endl
             << "      z: " << (*translation)(2) << std::endl;

        file << "    rotation:" << std::endl
             << "      x: " << rotation->x() << std::endl
             << "      y: " << rotation->y() << std::endl
             << "      z: " << rotation->z() << std::endl
             << "      w: " << rotation->w() << std::endl;

        file << "  " << asus_2_frame_id_ << ":" << std::endl;

        eigenHelper::toVectorQuaternion(transf_Kinect2,translation,rotation);

        file << "    translation:" << std::endl
             << "      x: " << (*translation)(0) << std::endl
             << "      y: " << (*translation)(1) << std::endl
             << "      z: " << (*translation)(2) << std::endl;

        file << "    rotation:" << std::endl
             << "      x: " << rotation->x() << std::endl
             << "      y: " << rotation->y() << std::endl
             << "      z: " << rotation->z() << std::endl
             << "      w: " << rotation->w() << std::endl;

      }

      file.close();

}

void reconfigure_callback(telerobot::calibrationRefinementConfig &config, uint32_t level) {
    CAM1_Z = config.CAM1_Z;
    CAM2_Z = config.CAM2_Z;
    registrationStart = config.registrationStart;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "calibrationRefinement");

    calibrationRefinement cR;

    std::string pre_Kinect1 = "/poses/" + CAM1;
    getTransform(cR.calibrationRefinement_nh_, pre_Kinect1, cR.transf_Kinect1);

    std::string pre_Kinect2 = "/poses/" + CAM2;
    getTransform(cR.calibrationRefinement_nh_, pre_Kinect2, cR.transf_Kinect2);

    dynamic_reconfigure::Server<telerobot::calibrationRefinementConfig> server;
    dynamic_reconfigure::Server<telerobot::calibrationRefinementConfig>::CallbackType f;
    f = boost::bind(&reconfigure_callback, _1, _2);
    server.setCallback(f);

    ros::Rate rate(30);

    while(!registrationStart)
    {
        cR.publishClouds();
        ros::spinOnce();
        rate.sleep();
    }

    cR.registerClouds();

}
