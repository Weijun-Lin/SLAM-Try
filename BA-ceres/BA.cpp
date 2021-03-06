/**
 * @file BA.cpp
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief BA 问题类实现，Ceres 损失函数配置
 * @version 0.2
 * @date 2021-12-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "BA.h"

#include <iostream>
#include <sophus/se3.hpp>

// ------------ 参数化实现 ---------------
bool SE3LocalParameterization::Plus(const double* x, const double* delta,
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
// 我们直接在 CostFunction
// 中计算雅可比矩阵，所以这里不需要了，直接返回一个单位阵
bool SE3LocalParameterization::ComputeJacobian(const double* x,
                                               double* jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

// 返回大小
int SE3LocalParameterization::GlobalSize() const { return 6; }
int SE3LocalParameterization::LocalSize() const { return 6; }

// --------------- 损失函数构建 ----------------------
BACostFunction::BACostFunction(const Eigen::Vector2d& ui,
                               const Eigen::Matrix3d& K)
    : _ui(ui), _K(K) {}

// 计算损失
bool BACostFunction::Evaluate(double const* const* parameters,
                              double* residuals, double** jacobians) const {
    // 从数组构建位姿和空间点
    Vector6d pose(parameters[0]);
    Eigen::Matrix4d T = Sophus::SE3d::exp(pose).matrix();
    Eigen::Vector3d P(parameters[1]);
    Eigen::Vector4d P4;
    P4 << P, 1;
    // 相机投影模型获得重投影点
    Eigen::Vector3d P_ = (T * P4).head(3);
    Eigen::Vector3d proj = _K * P_;
    proj /= proj[2];
    // 损失更新 计算重投影误差
    residuals[0] = _ui[0] - proj[0];
    residuals[1] = _ui[1] - proj[1];
    // 获取雅可比
    // 首先计算对空间点的导数
    double fx = _K(0, 0), fy = _K(1, 1);
    double X = P_[0], Y = P_[1], Z = P_[2];
    Eigen::Matrix<double, 2, 3> partialP_;
    partialP_ << fx / Z, 0, -fx * X / (Z * Z), 0, fy / Z, -fy * Y / (Z * Z);
    partialP_ = -partialP_;
    // 然后计算对位姿的导数，根据扰动模型 3*6
    Eigen::Matrix<double, 3, 6> partialP_2pose;
    partialP_2pose << Eigen::Matrix3d::Identity(), -Sophus::SO3d::hat(P_);
    // 合并得到关于位姿的导数
    Eigen::Matrix<double, 2, 6> partial_pose = partialP_ * partialP_2pose;
    // 然后计算关于空间点的导数
    Eigen::Matrix3d R = T.topLeftCorner(3, 3);
    Eigen::Matrix<double, 2, 3> partialP = partialP_ * R;
    // 更新雅可比 注意查看 ceres 雅可比存储规则
    // 首先需要判断雅可比指针是否为空，nullptr 代表 ceres
    // 在某个阶段不需要这一项雅可比
    if (jacobians != nullptr && jacobians[0] != nullptr) {
        ceres::MatrixRef(jacobians[1], 2, 3) = partialP;
    }
    if (jacobians != nullptr && jacobians[1] != nullptr) {
        ceres::MatrixRef(jacobians[0], 2, 6) = partial_pose;
    }

    return true;
}

void solveBA(Vector6d& se_T, Points3d& pts3d, Points2d& pts2d,
             const Eigen::Matrix3d& K, bool isSchur) {
    assert(pts3d.size() == pts2d.size());
    int nums = pts3d.size();
    double* pose = new double[6];
    double* pts = new double[3 * nums];
    ceres::MatrixRef(pose, 6, 1) = se_T;
    // 使用空间点初始化
    for (int i = 0; i < nums; i++) {
        pts[i * 3 + 0] = pts3d[i](0);
        pts[i * 3 + 1] = pts3d[i](1);
        pts[i * 3 + 2] = pts3d[i](2);
    }
    ceres::Problem problem;
    // 舒尔求解顺序
    ceres::ParameterBlockOrdering* blockOrdering =
        new ceres::ParameterBlockOrdering();
    if (isSchur) {
        blockOrdering->AddElementToGroup(pose, 0);
    }
    // 添加残差
    for (int i = 0; i < nums; i++) {
        ceres::CostFunction* costFunction = new BACostFunction(pts2d[i], K);
        problem.AddResidualBlock(costFunction, nullptr, pose, pts + 3 * i);
        if (isSchur) {
            blockOrdering->AddElementToGroup(pts + 3 * i, 1);
        }
    }
    // 添加位姿的参数化
    problem.SetParameterization(pose, new SE3LocalParameterization());
    // 配置求解器
    ceres::Solver::Options option;
    if (isSchur) {
        // 配置舒尔求解
        option.linear_solver_type = ceres::DENSE_SCHUR;
        option.linear_solver_ordering.reset(blockOrdering);
    } else {
        // 直接使用矩阵分解
        option.linear_solver_type = ceres::DENSE_QR;
    }
    // 其它配置
    option.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    // 输出对应结果
    Eigen::Map<Vector6d> pose_vec(pose);
    se_T = pose_vec;
    for (int i = 0; i < nums; i++) {
        pts3d[i] << pts[i * 3 + 0], pts[i * 3 + 1], pts[i * 3 + 2];
    }
    // 释放内存
    delete[] pose;
    delete[] pts;
}