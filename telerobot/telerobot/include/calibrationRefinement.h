#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <telerobot/calibrationRefinementConfig.h>
#include <ros/package.h>

#include "pclHelper.h"

class calibrationRefinement
{
    public:

        //constructor
        calibrationRefinement();

        //Visualization
        void publishClouds();

        //Registration
        void registerClouds();

        //TransformationMatrix
        Eigen::Matrix4f transf_Kinect1;
        Eigen::Matrix4f transf_Kinect2;

        //nodehandle
        ros::NodeHandle calibrationRefinement_nh_;

    private:
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

        //callbacks
        void asus1Callback(const PointCloud::ConstPtr& cloud);
        void asus2Callback(const PointCloud::ConstPtr& cloud);

        //transform listener
        tf::TransformListener tf_Listener_;

        void saveTransformation();

};
