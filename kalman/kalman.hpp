#include "eigen/Eigen/Dense"
#pragma once
/*
x[t]=A[t]x[t-1]+B[t]u[t]+w[t];P(w)
z[t]=C[t]x[t]+v[t];

x:state vector (n,1)
u:control vector (l,1)
z:observation vector (m,1)

A:state transition matrix (n,n)
B:(n,l)
C:(m,1)
Q=measurement noise covariance matrix::v
R=process noise covariance matrix::w

p:predicted covariance=uncertainity of belief
*/
class Kalman
{
private:
    Eigen::MatrixXd A, B, C, Q, R, P, K;
    int n, l, m;
    double t0, t, dt;
    Eigen::MatrixXd I; // identity matrix
    Eigen::VectorXd x_old, x_new, z_, u_;

public:
    // constructor
    Kalman(
        double dt,
        double t0,
        const Eigen::MatrixXd &A,
        const Eigen::MatrixXd &B,
        const Eigen::MatrixXd &C,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R,
        const Eigen::MatrixXd &P,
        const Eigen::VectorXd &x_0,
        const Eigen::VectorXd &u_0,
        const Eigen::VectorXd &z_0);
    Kalman();
    ~Kalman(); // destructor
    // void predict(const Eigen::MatrixXd &y); // predict new state vector at time t+1
    void update(const Eigen::VectorXd &u, const Eigen::VectorXd &z); // update measurement vector z at time t+1
    void update(const Eigen::VectorXd &u, const Eigen::VectorXd &z, double dt, const Eigen::MatrixXd A, const Eigen::MatrixXd B);
    Eigen::VectorXd state() { return x_old; };
    double time() { return t; };
};
