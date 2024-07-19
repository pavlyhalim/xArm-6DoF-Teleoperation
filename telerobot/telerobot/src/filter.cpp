#include "filter.h"
#include <stdlib.h>

/*
 * Constructor to init angles Vector to the basic position (-90 -90 0 -90 0 0 0).
 *
 */
movingAverage::movingAverage(int dequeS, int nrAngles){
   
	dequeSize = dequeS;
   	
	int i;
   	for(i = 0; i<dequeSize; i++){
      		Eigen::VectorXd zero(nrAngles);
		int j;
		for(j = 0; j<nrAngles; j++){
      			zero(j) = 0.0;
		}
        zero(0) = -90.0;
        zero(1) = -90.0;
        zero(3) = -90.0;
        anglesVector.push_back(zero);
   	}
}

/*
 * Store new value.
 *
 */
void movingAverage::addNewValues(Eigen::VectorXd val){

	anglesVector.pop_front();
	anglesVector.push_back(val);

}

/*
 * Calculate moving average.
 *
 */
Eigen::VectorXd movingAverage::getFilteredValues(){

	return averageCoeffWise(anglesVector);

}

// Calculate sin of vector coefficient wise.
Eigen::VectorXd sinXd(Eigen::VectorXd vec){
   Eigen::VectorXd res(vec.size());
   int i;
   for(i=0;i<vec.size();i++){
      res(i) = sin(vec(i)*(M_PI / 180.0));
   }
   return res;	
}

// Calculate cos of vector coefficient wise.
Eigen::VectorXd cosXd(Eigen::VectorXd vec){
   Eigen::VectorXd res(vec.size());
   int i;
   for(i=0;i<vec.size();i++){
      res(i) = cos(vec(i)*(M_PI / 180.0));
   }
   return res;	
}

// Calculate atan2 of vector coefficient wise.
Eigen::VectorXd atan2Xd(Eigen::VectorXd y, Eigen::VectorXd x){
   Eigen::VectorXd res(y.size());
   int i;
   for(i=0;i<y.size();i++){
      res(i) = atan2(y(i),x(i));
   }
   return res;	
}

/*
 * Method to calculate the complex! moving average for each element of a vector.
 *
 */
Eigen::VectorXd averageCoeffWise(std::deque<Eigen::VectorXd> anglesVector){
    int i;
    int vectorSize = anglesVector[0].size();

    Eigen::VectorXd sum_x(vectorSize);
    Eigen::VectorXd sum_y(vectorSize);
    Eigen::VectorXd x(vectorSize);
    Eigen::VectorXd y(vectorSize);
    sum_x = Eigen::VectorXd::Zero(vectorSize);
    sum_y = Eigen::VectorXd::Zero(vectorSize);

    for(i=0; i<anglesVector.size(); i++){

        //calculate kartesic coordiantes
        x = cosXd(anglesVector[i]);
        y = sinXd(anglesVector[i]);

        sum_x = sum_x + x;
        sum_y = sum_y + y;

    }

    Eigen::VectorXd average_x(vectorSize);
    Eigen::VectorXd average_y(vectorSize);

    average_x = sum_x/anglesVector.size();
    average_y = sum_y/anglesVector.size();

    Eigen::VectorXd result(vectorSize);
    result = atan2Xd(average_y,average_x);
    return result * (180 / M_PI);
}

/*
 * Method to limit the change of an angle to the value given by maxAngularChange.
 *
 */
double limitAngularMovement(double alpha, double alphaOld, double maxAngularChange){
    double deltaAlpha = alpha - alphaOld;
    if (std::min(std::abs(deltaAlpha),(360.0-std::abs(deltaAlpha)))>maxAngularChange){
    	if (deltaAlpha > 0.0 && deltaAlpha <= 180.0){
	    return fmod((alphaOld + maxAngularChange + 180.0),360) - 180;
    	}else if (deltaAlpha > -180.0 && deltaAlpha <= 0.0){
	    return fmod((alphaOld - maxAngularChange + 180.0),360) - 180;
    	}else if (deltaAlpha > 180 && deltaAlpha <= 360){
            return fmod((alphaOld - maxAngularChange + 180.0),360) - 180;
    	}else if (deltaAlpha > -360 && deltaAlpha <= -180){
            return fmod((alphaOld + maxAngularChange + 180.0),360) - 180;
    	}
    }
    
    return alpha;
}

/*
 * Mehtod to limit an angle to the interval given by [min, max].
 *
 */
double limitAngle(double angle, double min, double max){

	return std::max(std::min(angle,max),min);

}

/*
 * Hysteresis Filter
 */

hysteresis::hysteresis(double maxVal, double minVal) {
    max = maxVal;
    min = minVal;
    state=false;
}

bool hysteresis::filter(double val) {
    if(val<max && state==false) {
        state=false;
    }
    if(val>max && state==false) {
        state=true;
    }
    if(val>min && state==true) {
        state=true;
    }
    if(val<min && state==true) {
        state=false;
    }
    return state;
}


