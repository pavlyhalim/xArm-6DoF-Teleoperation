#include "GetOpenniTrackerData.h"


/*
 * Class to communicate with the Opennitracker ros-node.
 *
 */


   GetOpenniTrackerData::GetOpenniTrackerData() {

   }

   bool GetOpenniTrackerData::invalidValues() {
        //std::cout << "Input Value: " << std::endl << getShoulderPoint() << std::endl;
        Eigen::Vector3d shoulder = getShoulderPoint();
        double treshold = 0.0001;
        return (abs(shoulder(0))<treshold) && (abs(shoulder(1))<treshold) && (abs(shoulder(2))<treshold);
   }

   Eigen::Vector3d GetOpenniTrackerData::getShoulderPoint () {

      try{
         listener.lookupTransform("/openni_depth_frame", "/left_shoulder_1", ros::Time(0), transformShoulder);
      } catch (tf::TransformException ex){
         //ROS_ERROR("%s",ex.what());
      }
      
      Eigen::Vector3d shoulderPoint(transformShoulder.getOrigin().x(), transformShoulder.getOrigin().y(), transformShoulder.getOrigin().z());
      return shoulderPoint;
   }

   Eigen::Vector3d GetOpenniTrackerData::getElbowPoint () {

      try{
         listener.lookupTransform("/openni_depth_frame", "/left_elbow_1", ros::Time(0), transformElbow);
      } catch (tf::TransformException ex){
         //ROS_ERROR("%s",ex.what());
      }

      Eigen::Vector3d elbowPoint(transformElbow.getOrigin().x(), transformElbow.getOrigin().y(), transformElbow.getOrigin().z());
      return elbowPoint;
   }

   Eigen::Vector3d GetOpenniTrackerData::getHandPoint () {

      try{
         listener.lookupTransform("/openni_depth_frame", "/left_hand_1", ros::Time(0), transformHand);
      } catch (tf::TransformException ex){
         //ROS_ERROR("%s",ex.what());
      }

      Eigen::Vector3d handPoint(transformHand.getOrigin().x(), transformHand.getOrigin().y(), transformHand.getOrigin().z());
      return handPoint;
   }	

