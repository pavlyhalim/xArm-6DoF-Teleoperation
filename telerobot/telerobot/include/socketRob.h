#include <Eigen/Dense>
#include <iostream>


/*
 * Comunicates with Robot or Simulator.
 *
 */
class socketRob {
public:
	socketRob(std::string hostName, int port);
    void sendJointsSimulator(Eigen::VectorXd angles);
	void sendString(std::string str);
	void sendJointsKuka(Eigen::VectorXd angles);

};
