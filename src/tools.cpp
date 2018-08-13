#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    Eigen::VectorXd RMSE(4);
    RMSE<<0,0,0,0;
    
    if (estimations.size()==0||estimations.size()!=ground_truth.size()) {
        cout<<"invalid estimations or ground_truth data"<<endl;
        return RMSE;
    }
    
    for (int i=0; i<estimations.size(); ++i) {
        Eigen::VectorXd residual = estimations[i]-ground_truth[i];
        residual = residual.array()*residual.array();
        RMSE+=residual;
    }
    //get the mean;
    RMSE = RMSE/estimations.size();
    
    //calculate the squared root
    RMSE = RMSE.array().sqrt();
    
    return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    Eigen::MatrixXd Hj(3,4);
    float px = x_state(0);
    float py  = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = c1*c2;
    
    if(fabs(c1)<0.0001){
        cout<<"error: devided by zero"<<endl;
    }
    
    Hj<<(px/c2),(py/c2),0,0,
        (-py/c1),(px/c1),0,0,
    (py*(vx*py-vy*px)/c3),(px*(vy*px-vx*py)/c3),(px/c2),(py/c2);
    
    return Hj;
    
}
