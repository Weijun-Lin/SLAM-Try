#include "BA.h"
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <iostream>


// ------------ 参数化实现 ---------------
bool SE3LocalParameterization::Plus(
        const double* x,
        const double* delta,
        double* x_plus_delta) const {
    const Vector6d lie(x);
    const Vector6d d_lie(delta);
    // 左乘更新
    Sophus::SE3d T = Sophus::SE3d::exp(lie) * Sophus::SE3d::exp(d_lie);
    Vector6d update = T.log();
    // 赋值
    ceres::MatrixRef(x_plus_delta, 6, 1) = update;
    return true;
}

// 这里是局部偏导 但我们给出的是 (TP)/(kxi) 这里本质应该是 (T)/(kxi)
// 我们直接在 CostFunction 中计算雅可比矩阵，所以这里不需要了，直接返回一个单位阵
bool SE3LocalParameterization::ComputeJacobian(const double* x, double* jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

// 返回大小
int SE3LocalParameterization::GlobalSize() const { return 6; }
int SE3LocalParameterization::LocalSize() const { return 6; }

// --------------- 损失函数构建 ----------------------
BACostFunction::BACostFunction(const Eigen::Vector2d &ui, const Eigen::Matrix3d &K):
    _ui(ui), _K(K) {}

// 计算损失
bool BACostFunction::Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
    // 计算投影点
    Vector6d pose(parameters[0]);
    Eigen::Matrix4d T = Sophus::SE3d::exp(pose).matrix();
    Eigen::Vector3d P(parameters[1]);
    Eigen::Vector4d P4(P, 1);
    // 相机坐标系下的空间点
    Eigen::Vector3d P_ = (T*P4).head(3);
    Eigen::Vector3d proj = _K*P_;
    proj /= proj[2];
    // 损失更新
    residuals[0] = _ui[0] - proj[0];
    residuals[1] = _ui[1] - proj[1];
    
    // 获取雅可比
    // 首先计算对空间点的导数
    double fx = _K(0, 0), fy = _K(0, 1);
    double X = P_[0], Y = P_[1], Z = P_[2];
    Eigen::Matrix<double, 2, 3> partialP_;
    P_ << fx/Z  ,0      ,-fx*X/(Z*Z),
          0     ,fy/Z   ,-fy*Y/(Z*Z);
    P_ = -P_;
    // 然后计算对位姿的导数，根据扰动模型 3*6
    Eigen::Matrix<double, 3, 6> partialP_2pose;
    partialP_2pose << Eigen::Matrix3d::Identity(), -Sophus::SO3d::hat(P_);
    // 合并得到关于位姿的导数
    Eigen::Matrix<double, 2, 6> partial_pose = partialP_ * partialP_2pose;

    // 然后计算关于空间点的导数
    Eigen::Matrix3d R = T.topLeftCorner(3, 3);
    Eigen::Matrix<double, 2, 3> partialP = partialP_ * R;

    // 获取最后的雅可比矩阵 2*9
    Eigen::Matrix<double, 2, 9> partial;
    partial << partial_pose, partialP;

    // 更新雅可比
    ceres::MatrixRef(jacobians[0], 1, 6) = partial.row(0);
    ceres::MatrixRef(jacobians[1], 1, 6) = partial.row(1);
    
    return true;
}



// ---------------- BA 问题的构造 -----------------
BA::BA(const Points3D& pts3d, const Points2D& pts2d, const Eigen::Matrix3d &K):
    _points3d(pts3d), _points2d(pts2d), _K(K) {
        _p_nums = pts3d.size();
    }

void BA::solve(Eigen::Matrix3d &R, Eigen::Vector3d &T, Points3D &pts3d) {
    // 设置参数 位姿李代数以及空间点
    double *pose = new double[6];
    double *pts = new double[3*_p_nums];
    ceres::Problem problem;
    // 添加残差
    for(int i = 0;i < _p_nums;i++) {
        ceres::CostFunction *costFunction = new BACostFunction(_points2d[i], _K);
        problem.AddResidualBlock(costFunction, nullptr, pose, pts + 3*i);
    }
    // 求解
    // 配置求解器
    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_QR;
    option.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    std::cout << summary.BriefReport() << std::endl;
    // 输出对应结果
    Eigen::Map<Vector6d> vec(pose);
    Eigen::Matrix4d T_ = Sophus::SE3d::exp(vec).matrix();
    for(int i = 0;i < _p_nums;i++) {
        pts3d[i] << pts[i*3 + 0], pts[i*3 + 1], pts[i*3 + 2];
    }
    // 更新数据
    R = T_.block(0, 0, 3, 3);
    T = T_.block(0, 3, 3, 1);
    for(int i = 0;i < _p_nums;i++) {
        pts3d[i] << pts[3*i + 0], pts[3*i + 1], pts[3*i + 2];
    }
    // 释放内存
    delete pose;
    delete pts;
}