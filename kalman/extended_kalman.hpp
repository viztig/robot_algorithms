#include "eigen/Eigen/Dense"

/*
A:state transition matrix
H:measurement matrix

*/
class Kalman
{
private:
    Eigen::MatrixXd A, W, Q, P, H, R, I;
    int m, n, l;
    double t0, t, dt;
    Eigen::VectorXd x_, x_new, u_;

public:
    Kalman(
        double dt,
        double t0,
        const Eigen::MatrixXd &)
};