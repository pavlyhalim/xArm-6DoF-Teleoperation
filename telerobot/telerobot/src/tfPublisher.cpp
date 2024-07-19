#include <ros/ros.h>
#include "camNames.h"
#include <tf/transform_broadcaster.h>

void getTransform(const ros::NodeHandle &node_handle, const std::string &frame_id, tf::Transform &transform){

    std::string pre = "/poses/" + frame_id;

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

    transform.setOrigin(tf::Vector3(translation_x,translation_y,translation_z));
    transform.setRotation(tf::Quaternion(rotation_x,rotation_y,rotation_z,rotation_w));

    return;

}

int main(int argc, char** argv){

    ros::init(argc, argv, "tfPublisher");

    //nodehandle
    ros::NodeHandle tfPublisher_nh;

    //frames
    std::string full_frame_id = "Full";
    std::string asus_1_frame_id = CAM2;
    std::string asus_2_frame_id = CAM1;

    //transformation
    tf::TransformBroadcaster br;
    tf::Transform asus_1_transform;
    tf::Transform asus_2_transform;

    ros::Rate rate(10);

    while (ros::ok())
    {

        getTransform(tfPublisher_nh, asus_1_frame_id, asus_1_transform);
        getTransform(tfPublisher_nh, asus_2_frame_id, asus_2_transform);

        br.sendTransform(tf::StampedTransform(asus_1_transform, ros::Time::now(), full_frame_id, asus_1_frame_id));
        br.sendTransform(tf::StampedTransform(asus_2_transform, ros::Time::now(), full_frame_id, asus_2_frame_id));

        ros::spinOnce();
        rate.sleep();
    }

}
