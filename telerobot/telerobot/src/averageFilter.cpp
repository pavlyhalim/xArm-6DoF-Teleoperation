#include "averageFilter.h"

Eigen::VectorXd sinXd(Eigen::VectorXd vec){
   Eigen::VectorXd res(vec.size());
   int i;
   for(i=0;i<vec.size();i++){
      res(i) = sin(vec(i)*(M_PI / 180.0));
   }
   return res;	
}

Eigen::VectorXd cosXd(Eigen::VectorXd vec){
   Eigen::VectorXd res(vec.size());
   int i;
   for(i=0;i<vec.size();i++){
      res(i) = cos(vec(i)*(M_PI / 180.0));
   }
   return res;	
}

Eigen::VectorXd atan2Xd(Eigen::VectorXd y, Eigen::VectorXd x){
   Eigen::VectorXd res(y.size());
   int i;
   for(i=0;i<y.size();i++){
      res(i) = atan2(y(i),x(i));
   }
   return res;	
}

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
