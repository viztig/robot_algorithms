#include <iostream>
#include <cmath>
#include "ukf.h"

UKF::UKF(
    int n, int m, int l,
    float alpha, float beta, float kappa,
    int total_sigma_points,
    Eigen::MatrixXd &P0,
    Eigen::MatrixXd &R0,
    Eigen::MatrixXd &Q0,
    Eigen::VectorXd &x_0)
    : P(P0), x_(x_0), R(R0), Q(Q0), n(n), m(m),
      l(l), alpha(alpha), beta(beta), kappa(kappa),
      Wm((2 * n + 1)), Dx(n, 2 * n + 1), x_sigma((n, 2 * n + 1))
{
    lambda = alpha * alpha * (n - kappa) - n;
    gamma = sqrt((n + this->lambda));
    Wm[0] = lambda / (n + lambda);
    for (int i = 1; i <= 2 * n; i++)
    {
        Wc[i] = 0.5 / (n + lambda);
    }
    Wc = Wm;
    Wc[0] += (1 - alpha * alpha + beta);
}

bool UKF::update(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &U)
{
    Eigen::MatrixXd Dy = Dx;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < 2 * n + 1; j++)
        {
            Dx.row(i)[j] *= Wc[j];
        }
    }
    Pxy = Dx * Dy.transpose();
    K = Pxy * Py.inverse(); // kalman gain;
    x_ = x_ + K * (Z - Z_hat);
    P = P - (K * Py * K.transpose());
    return true;
}
bool UKF::calculate_sigma_point()
{
    Eigen::MatrixXd P_sqrt(P.llt().matrixL());
    P_sqrt = P_sqrt * gamma;
    x_sigma.col(0) = x_;
    Eigen::MatrixXd y_(n, n);
    for (int i = 0; i < n; i++)
    {
        y_.col(i) = x_;
    }
    for (int i = 1; i <= n; i++)
    {
        x_sigma.col(i) = (y_ + P_sqrt).col(i - 1);
    }
    for (int i = 1 + n; i <= 2 * n + 1; i++)
    {
        x_sigma.col(i) = (y_ - P_sqrt).col(i - 1);
    }
    return true;
}
bool UKF::transform()
{
    for (int i = 0; i < 2 * n + 1; i++)
    {
        fn_f(x_sigma.col(i));
    }
}
