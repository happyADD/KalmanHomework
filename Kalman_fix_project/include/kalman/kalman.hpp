#ifndef KALMAN_HPP
#define KALMAN_HPP

#define QUEEN_LENGTH 20 //计算方差的窗口，窗口越大，滞后越大

#include <cmath>

#include <iostream>
#include <vector>
#include <atomic>

#include <Eigen/Dense>
#include <../angles/angles/angles.h>

#include <rclcpp/rclcpp.hpp>

/**
 * X_k = [P_k
 *        v_k]
 * A = [1 delta_T
 *      0   1   ]
 */
class Kalman {
private:

    //温馨提示,匀加速和匀速模型的维数不一样哦

    Eigen::Matrix2d A;
    Eigen::Matrix<double,1,2> H;

    Eigen::Matrix2d P;

    Eigen::Matrix2d Q;
    Eigen::Matrix<double,1,1> R;

    Eigen::Vector2d x;


    double T;

public:
    //初始化A，H，P矩阵等
    Kalman(double _time);
    void Q_set(double qx);
    void R_set(double rx);

    Eigen::VectorXd predict();
    Eigen::VectorXd update(const Eigen::VectorXd &z_meas);
};

#endif
