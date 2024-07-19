#include "CalculateArmAngles.h"

/*
 * Class calculates four arm angles. Input: 3D Position of shoulder, elbow and hand.
 */

	CalculateArmAngles::CalculateArmAngles() {

	}

	void CalculateArmAngles::setArmData(Eigen::Vector3d shoulder, Eigen::Vector3d elbow, Eigen::Vector3d hand) {
		shoulderPoint = shoulder;
		elbowPoint = elbow;
		handPoint = hand;
		upperarm = -shoulderPoint + elbowPoint;
        forearm = -elbowPoint + handPoint;
	}

	double CalculateArmAngles::getAlpha() {
		return atan2(upperarm(1),upperarm(0))*(180/M_PI);
	}

	double CalculateArmAngles::getBeta() {
		Eigen::Vector3d shouldCS_Z;
        shouldCS_Z <<  upperarm(0), upperarm(1), 0.0;
        shouldCS_Z.normalize();
   		Eigen::Vector3d shouldCS_Y;
        shouldCS_Y << 0.0, 0.0, 1.0;
        Eigen::Vector3d shouldCS_X = shouldCS_Z.cross(shouldCS_Y).normalized();
        Eigen::Matrix4d shouldCS;

    		shouldCS << shouldCS_Y(0), shouldCS_X(0), shouldCS_Z(0), shoulderPoint(0),
                	    shouldCS_Y(1), shouldCS_X(1), shouldCS_Z(1), shoulderPoint(1),
                	    shouldCS_Y(2), shouldCS_X(2), shouldCS_Z(2), shoulderPoint(2),
                            0,            0,            0,               1;
    		Eigen::Matrix4d B;
    		B = shouldCS.inverse();

    		Eigen::Vector4d elbowPoint_homogen(elbowPoint(0), elbowPoint(1), elbowPoint(2), 1);
    		Eigen::Vector4d elbowPoint_projected_temp = B*elbowPoint_homogen;
    		Eigen::Vector2d elbowPoint_projected(elbowPoint_projected_temp(0), elbowPoint_projected_temp(2));

    		return atan2(elbowPoint_projected(1), elbowPoint_projected(0))*(180/M_PI);
	}

	double CalculateArmAngles::getGamma() {
		return acos(-(upperarm.dot(forearm))/((upperarm.norm())*(forearm.norm())))*(180/M_PI);
	}

	double CalculateArmAngles::getDelta() {
            Eigen::Vector3d elbCS_Z = upperarm.normalized();
    		Eigen::Vector3d elbCS_Y = upperarm.cross(forearm).normalized();
    		Eigen::Vector3d elbCS_X = elbCS_Z.cross(elbCS_Y).normalized();

    		Eigen::Matrix4d elbCS;
    		elbCS << elbCS_Y(0), elbCS_X(0), elbCS_Z(0), elbowPoint(0),
	     	         elbCS_Y(1), elbCS_X(1), elbCS_Z(1), elbowPoint(1),
	     		 elbCS_Y(2), elbCS_X(2), elbCS_Z(2), elbowPoint(2),
	              	 0,          0,        0,             1;

    		Eigen::Matrix4d A;
    		A = elbCS.inverse();

    		Eigen::Vector4d handPoint_homogen(handPoint(0), handPoint(1), handPoint(2), 1);
    		Eigen::Vector4d handPoint_projected_temp = A*handPoint_homogen;
    		Eigen::Vector2d handPoint_projected(handPoint_projected_temp(0), handPoint_projected_temp(1));

    		Eigen::Vector4d Z(elbowPoint(0), elbowPoint(1), elbowPoint(2) + 1.0, 1.0);
    		Eigen::Vector4d Z_projected_temp = A*Z;
    		Eigen::Vector2d Z_projected(Z_projected_temp(0), Z_projected_temp(1));

    		return atan2(Z_projected(1), Z_projected(0))*(180/M_PI);
	}

