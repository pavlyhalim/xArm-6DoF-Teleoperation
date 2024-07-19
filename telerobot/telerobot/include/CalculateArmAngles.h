#include <Eigen/Dense>


/*
 * Class calculates four arm angles. Input: 3D Position of shoulder, elbow and hand.
 */
class CalculateArmAngles {
public:
   	Eigen::Vector3d shoulderPoint;
  	Eigen::Vector3d elbowPoint;
   	Eigen::Vector3d handPoint;	
   	Eigen::Vector3d upperarm;
   	Eigen::Vector3d forearm;

	CalculateArmAngles();

	void setArmData(Eigen::Vector3d shoulder, Eigen::Vector3d elbow, Eigen::Vector3d hand);
	double getAlpha();
	double getBeta();
	double getGamma();
	double getDelta();
};
