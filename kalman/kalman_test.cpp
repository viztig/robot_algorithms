#include <iostream>
#include <vector>
#include "eigen/Eigen/Dense"

#include "kalman.cpp"
using namespace std;

int main(int argc, char *argv[])
{

    int n = 3; // Number of states
    int m = 1; // Number of measurements
    int l = 1; // Number of controls

    double dt = 1.0 / 30; // Time step
    double t0 = 0;

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n);
    Eigen::MatrixXd B(n, l); // Output matrix
    Eigen::MatrixXd R(n, n); // Measurement noise covariance
    Eigen::MatrixXd Q(m, m); // Process noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance
    // Discrete LTI projectile motion, measuring position only
    A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
    C << 1, 0, 0;
    B << 1, 0, 0;
    // Reasonable covariance matrices
    R << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    Q << 5;
    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
    std::cout << "A: \n"
              << A << std::endl;
    std::cout << "B: \n"
              << B << std::endl;
    std::cout << "C: \n"
              << C << std::endl;
    std::cout << "Q: \n"
              << Q << std::endl;
    std::cout << "R: \n"
              << R << std::endl;
    std::cout << "P: \n"
              << P << std::endl;

    // List of noisy position measurements (y)
    std::vector<double> measurements = {
        1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
        1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
        2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
        2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
        2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
        2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
        2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
        1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
        0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813};

    // Best guess of initial states
    Eigen::VectorXd x0(n);
    Eigen::VectorXd u0(l);
    Eigen::VectorXd z0(m);

    double t = 0;
    x0 << measurements[0], 0, -9.81;
    std::cout << "x0: " << x0 << std::endl;
    u0 << 0.1;
    std::cout << "u0: " << u0 << std::endl;
    z0 << measurements[0];
    std::cout << "z0: " << z0 << std::endl;

    std::cout << "initial states constructed!!!" << std::endl;
    // Feed measurements into filter, output estimated states

    // Construct the filter
    std::cout << "constructing filter:" << std::endl;
    Kalman kf(dt, t0, A, B, C, Q, R, P, x0, u0, z0);
    std::cout << "t = " << t << ", "
              << "x_[0]: " << kf.state().transpose() << std::endl;
    for (int i = 0; i < measurements.size(); i++)
    {
        t += dt;
        z0 << measurements[i];
        kf.update(u0, z0);
        std::cout << "t = " << t << ", "
                  << "z[" << i << "] = " << z0.transpose()
                  << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
    }

    return 0;
}
