#include "eigen/Eigen/Dense"
#pragma once

/*
x[k+1]=f(x[k],us[k])+w[k]
z[k]=g(h[k],um[k])+v[k]
w[k]~N(0,Q[k])
v[k]~N(0,R[k])

ul:control vector
um:sensor vector
x:state vector
z:measurement vector
w:sensor noise
v:measurement noise
Q:Process noise covariance matrix
R:mesurement noise covariance matrix

f,g:nonlinear functions
*/

class UKF
{
private:
    int n; // number of states
    int m; // no of sensor measurements
    int l;
    float alpha;
    float beta;
    float lambda;
    float gamma;
    float kappa;            // no of sensor controls
    int total_sigma_points; // total number of sigma points
    Eigen::MatrixXd P;      // state covariance matrix:covariance of prior state estimate
    Eigen::MatrixXd R;      // measurement error covariance
    Eigen::MatrixXd Q;      // prior error covariance
    Eigen::VectorXd x_;     // state vector
    Eigen::VectorXd x_hat;
    Eigen::VectorXd x_hat_sigma; // predicted(estimated)sigma point matrix
    Eigen::VectorXd Wm;          // first order weight vector
    Eigen::VectorXd Wc;          // second order weight vector
    Eigen::MatrixXd K;           // kalman gain
    Eigen::VectorXd Z_hat;

public:
    UKF(
        int n,
        int m,
        int l,
        float alpha, float beta, float kappa,
        int total_sigma_points,
        Eigen::MatrixXd &P0,
        Eigen::MatrixXd &R0,
        Eigen::MatrixXd &Q0,
        Eigen::VectorXd &x_0);
    virtual ~UKF();
    Eigen::MatrixXd Dx;  // delta x=[x[k]-x_hat[k|k-1]]
    Eigen::MatrixXd Pxy; // cross covariance matrix
    Eigen::MatrixXd Py;
    Eigen::MatrixXd x_sigma;
    bool update(const Eigen::MatrixXd &, const Eigen::MatrixXd &);
    bool calculate_sigma_point(void);
    bool fn_f(Eigen::VectorXd &v)
    {
        for (int i = 0; i < n; i++)
        {
            v[i] *= v[i];
        }
        return true;
    }
    bool fn_h(Eigen::VectorXd &w)
    {
        for (int i = 0; i < n; i++)
        {
            w[i] *= w[i];
        }
        return true;
    }
    bool transform();
};