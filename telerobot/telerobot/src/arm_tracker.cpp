#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <Eigen/Dense>

#include "GetOpenniTrackerData.h"
#include "CalculateArmAngles.h"
#include "filter.h"
#include "socketRob.h"
#include "handSkeleton.h"

typedef Eigen::Matrix<double,8,1> Vector8d;

double epsilonTemp;
double zetaTemp;
double etaTemp;
double indexThetaMCP2Temp;

void initKuka(socketRob &socketRobot, GetOpenniTrackerData &getOpenniData, ros::NodeHandle &node) {

    sleep(1);
    socketRobot.sendString("Hello Robot");
    sleep(1);
    socketRobot.sendString("SS 40");
    sleep(1);
    socketRobot.sendString("SA 10");
    sleep(1);
    socketRobot.sendString("STPF");
    sleep(1);
    socketRobot.sendString("SJ -90 -90 0 -90 0 0 0");
    sleep(5);

    while (getOpenniData.invalidValues() && node.ok()) {
        std::cout << "No valid tracker data !!!" << std::endl;
        sleep(0.01);
    }
    socketRobot.sendString("SS 100");
    sleep(5);

}

void callback(tracking_server_msgs::handSkeleton handMsg) {
    epsilonTemp = handMsg.params[0]*(180.0/M_PI);
    zetaTemp = handMsg.params[1]*(180.0/M_PI);
    etaTemp = handMsg.params[2]*(180.0/M_PI);
    indexThetaMCP2Temp = handMsg.params[19]*(180.0/M_PI);
//    std::cout << "  Yaw: " << epsilonTemp << std::endl;
//    std::cout << "Pitch: " << zetaTemp << std::endl;
//    std::cout << " Roll: " << etaTemp << std::endl;
//    std::cout << " indexThetaMCP1: " << indexThetaMCP2Temp << std::endl;
//    std::cout << " ----------- " << std::endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "arm_tracker");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/tracking/hand/eval/skeleton", 1, callback);

    GetOpenniTrackerData getOpenniData;

    CalculateArmAngles armAnglesCalculator;

    int dequeSize = 15; // number of values used for moving average filter
    int nrAngles = 8; // number of robots joints

    movingAverage movingAverageFilter(dequeSize,nrAngles);

    hysteresis hysteresisFilter(50.0, 20.0);

    socketRob socketRobot("127.0.0.1", 12345); // SIMULATOR!!!
    //socketRob socketRobot("141.83.19.40", 8000); // KUKA LWR!!!
    //initKuka(socketRobot, getOpenniData, node);

    double alpha;
    double beta;
    double gamma;
    double delta;
    double epsilon;
    double zeta;
    double eta;
    double indexThetaMCP2;

    double alphaOld;
    double betaOld;
    double gammaOld;
    double deltaOld;
    double epsilonOld;
    double zetaOld;
    double etaOld;
    double indexThetaMCP2Old;

    Eigen::VectorXd angles(nrAngles);

    ros::Rate rate(30.0);

    while (node.ok()){

        armAnglesCalculator.setArmData(getOpenniData.getShoulderPoint(), getOpenniData.getElbowPoint(), getOpenniData.getHandPoint());

        //Rueckwaertsrechnung
        double yaw = epsilonTemp * (M_PI/180.0);
        double pitch = zetaTemp * (M_PI/180.0);
        double roll = etaTemp * (M_PI/180.0);

//        std::cout << "epsilon: " << epsilon << std::endl;
//        std::cout << "zeta: " << zeta << std::endl;
//        std::cout << "eta: " << eta << std::endl;

        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());

        Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

        Eigen::Matrix3d mapTF;
        mapTF << 1,  0, 0,
                 0,  0, 1,
                 0, -1, 0;

        Eigen::Matrix3d G = mapTF * q.matrix();

//        double R33 = cos(pitch)*cos(roll);
//        double R31 = -sin(pitch);
//        double R13 = sin(pitch)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
//        double R32 = cos(pitch)*sin(roll);
//        double R23 = -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);

//        double beta_inverse = atan2(R33, sqrt(1-(R33*R33)))*(180.0/M_PI);
//        double gamma_inverse = atan2(R13, R23) * (180.0/M_PI);
//        double alpha_inverse = atan2(-R31, R32) * (180.0/M_PI);

        Eigen::Vector3d ea = G.eulerAngles(2, 1, 2);

        epsilon = ea[2] * (180.0/M_PI);
        zeta = -ea[0] * (180.0/M_PI);
        eta = -ea[1] * (180.0/M_PI);

        //std::cout << "    R13: " << R13 << std::endl;
        //std::cout << "    R23: " << R23 << std::endl;
        //std::cout << "epsilon: " << epsilon << std::endl;

        // Store last Value
        alphaOld = alpha;
        betaOld = beta;
        gammaOld = gamma;
        deltaOld = delta;
        epsilonOld = epsilon;
        zetaOld = zeta;
        etaOld = eta;
        indexThetaMCP2Old = indexThetaMCP2;

        //Add Offsets and normalize to [0...pi]
        alpha = (armAnglesCalculator.getAlpha() - 180.0); // offset = -180;
        alpha = fmod((alpha +  180.0),360) - 180;

        beta = -armAnglesCalculator.getBeta() + 0.0; // sign

        gamma = armAnglesCalculator.getGamma() -180; // offset = -180
        gamma = fmod((gamma + 180.0),360) - 180;

        delta = - armAnglesCalculator.getDelta() - 90.0; //offset = -90
                                                         //sign
        delta = fmod((delta + 180.0),360) - 180;

        epsilon = epsilon;
        epsilon = fmod((epsilon + 180.0),360) - 180;

        zeta = zeta - 0.0;
        zeta = fmod((zeta + 180.0),360) - 180;

        eta = eta - 0.0;
        eta = fmod((eta + 180.0),360) - 180;

        indexThetaMCP2 = indexThetaMCP2Temp - 0.0;
        indexThetaMCP2 = fmod((indexThetaMCP2 + 180.0),360) - 180;

        //limit velocity to 5 degr per iteration
        alpha = limitAngularMovement(alpha, alphaOld, 5);
        beta = limitAngularMovement(beta, betaOld, 5);
        gamma = limitAngularMovement(gamma, gammaOld, 5);
        delta = limitAngularMovement(delta, deltaOld, 5);
        epsilon = limitAngularMovement(epsilon, epsilonOld, 5);
        zeta = limitAngularMovement(zeta, zetaOld, 5);
        eta = limitAngularMovement(eta, etaOld, 5);
        indexThetaMCP2 = limitAngularMovement(indexThetaMCP2, indexThetaMCP2Old, 5);

        //limit angle
        // alpha = limitAngle(alpha, -170.0, 170.0);
        // beta = limitAngle(beta, -120.0, 120.0);
        // gamma = limitAngle(gamma, -120.0, 120.0);
        // delta = limitAngle(delta, -170.0, 170.0);

        //average filter
        angles << alpha, beta, delta, gamma, epsilon, zeta, eta, indexThetaMCP2;
        movingAverageFilter.addNewValues(angles);

        angles = movingAverageFilter.getFilteredValues();

        bool indexThetaMCP2filtered = hysteresisFilter.filter(-angles(7));
        if (indexThetaMCP2filtered) {
            angles(7) = -60.0;
        } else {
            angles(7) = 0.0;
        }

        //send Angles to robot
        socketRobot.sendJointsSimulator(angles); // SIMULATOR!!!
        //socketRobot.sendJointsKuka(movingAverageFilter.getFilteredValues()); // KUKA LWR!!!

        //std::cout<<"Running..."<<std::endl;

        ros::spinOnce();
        rate.sleep();

    }

    return 0;

};
