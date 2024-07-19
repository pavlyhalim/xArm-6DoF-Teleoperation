#include <Eigen/Dense>
#include <deque>

/*
 * Method to calculate the complex! moving average for each element of a vector.
 *
 */
Eigen::VectorXd averageCoeffWise(std::deque<Eigen::VectorXd> anglesVector);

/*
 * Method to limit the change of an angle to the value given by maxAngularChange.
 *
 */
double limitAngularMovement(double alpha, double alphaOld, double maxAngularChange);

/*
 * Mehtod to limit an angle to the interval given by [min, max].
 *
 */
double limitAngle(double angle, double min, double max);


/*
 * Class stores values and calculates moving average for each element of a vector.
 * In this case the seven arm angles.
 *
 */
class movingAverage {

public:
	
	movingAverage(int dequeS, int nrAngles);
	void addNewValues(Eigen::VectorXd val);
	Eigen::VectorXd getFilteredValues();
private:

	int dequeSize;
	std::deque<Eigen::VectorXd> anglesVector;
};

class hysteresis {
public:
    hysteresis(double maxVal, double minVal);
    bool filter(double val);
private:
    bool state;
    double max;
    double min;

};
