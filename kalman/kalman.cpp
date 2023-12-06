#include <iostream>
#include <stdexcept>
#include "kalman.hpp"

Kalman::Kalman(
    double dt,
    double t0,
    const Eigen::MatrixXd &A0,
    const Eigen::MatrixXd &B0,
    const Eigen::MatrixXd &C0,
    const Eigen::MatrixXd &Q0,
    const Eigen::MatrixXd &R0,
    const Eigen::MatrixXd &P0,
    const Eigen::VectorXd &x_0,
    const Eigen::VectorXd &u_0,
    const Eigen::VectorXd &z_0)
    : A(A0), B(B0), C(C0), Q(Q0), R(R0), P(P0), t0(t0), t(t0), m(C.rows()), n(A.rows()), l(B.cols()), dt(dt), I(n, n), x_old(x_0), x_new(x_0), u_(u_0), z_(z_0)
{
    I.setIdentity();
}
Kalman::Kalman() {}
Kalman::~Kalman() {}
void Kalman::update(const Eigen::VectorXd &u, const Eigen::VectorXd &z)
{
    // Prediction step
    Eigen::VectorXd x_hat = A * (this->x_old) + B * u;
    Eigen::MatrixXd P_hat = A * (this->P) * A.transpose() + R;

    // Correction step
    K = P_hat * C.transpose() * (C * P_hat * C.transpose() + Q).inverse();
    this->x_new = x_hat + K * (z - C * x_hat);
    this->P = (I - K * C) * P_hat;

    this->x_old = x_new;
    this->t += this->dt;
}
void Kalman::update(const Eigen::VectorXd &u, const Eigen::VectorXd &z, double dt, const Eigen::MatrixXd A, const Eigen::MatrixXd B)
{
    this->A = A;
    this->B = B;
    this->dt = dt;
    update(u, z);
}