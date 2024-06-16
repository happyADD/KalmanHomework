#include "kalman/kalman.hpp"

// sateSize状态量个数
// uSize输入的维度
using namespace std;

Kalman::Kalman(double _time): T(_time) {
    A << 1, T,
            0, 1;
    H << 1, 0;
    P.setIdentity();
    x.setZero();
}

void Kalman::Q_set(double qx) {
    Eigen::Vector2d G;
    G << T * T * 0.5, T;
    Q = G * qx * G.transpose();
}

void Kalman::R_set(double rx) {
    R << rx;
}

Eigen::VectorXd Kalman::predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
    return x;
}

Eigen::VectorXd Kalman::update(const Eigen::VectorXd &z_meas) {
    if(fabs(z_meas(0) - x(0))>1) {
        Eigen::Vector3d X = {x(0),x(1),0};
        return X;
    }

    auto K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x = x + K * (z_meas - H * x);
    P = (Eigen::Matrix2d::Identity() - K * H) * P;
    Eigen::Vector3d X = {x(0),x(1),0};
    return X;
}
