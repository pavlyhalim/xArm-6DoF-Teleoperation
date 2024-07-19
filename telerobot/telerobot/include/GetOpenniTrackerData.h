#include <tf/transform_listener.h>
#include <Eigen/Dense>

/*
 * Class to communicate with the Opennitracker ros-node.
 *
 */
class GetOpenniTrackerData {
public:
   tf::TransformListener listener;
   tf::StampedTransform transformShoulder;
   tf::StampedTransform transformElbow;
   tf::StampedTransform transformHand;

   GetOpenniTrackerData();

   bool invalidValues();

   Eigen::Vector3d getShoulderPoint ();
   Eigen::Vector3d getElbowPoint ();
   Eigen::Vector3d getHandPoint ();
};
