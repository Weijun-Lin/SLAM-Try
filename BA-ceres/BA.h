/**
 * @file BA.h
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief BA 问题类定义
 * @version 0.2
 * @date 2021-12-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __BA_H__
#define __BA_H__

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <vector>

#include "utils.h"

// SE3
// 的参数化，可参考：https://blog.csdn.net/u012348774/article/details/84144084
class SE3LocalParameterization : public ceres::LocalParameterization {
   public:
    // 扰动模型更新
    virtual bool Plus(const double* x, const double* delta,
                      double* x_plus_delta) const;
    // 计算局部雅可比 \partial x / \partial \delta
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;

    // Size of x.
    virtual int GlobalSize() const;

    // Size of delta.
    virtual int LocalSize() const;
};

// BA 的损失函数构建 损失为2个维度，参数块分别为位姿 6 和一个空间点 3
class BACostFunction : public ceres::SizedCostFunction<2, 6, 3> {
   public:
    BACostFunction(const Eigen::Vector2d& ui, const Eigen::Matrix3d& K);
    virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const;

   private:
    Eigen::Vector2d _ui;
    Eigen::Matrix3d _K;
};


/**
 * @brief 光束法平差求解函数
 *
 * @param se_T 相机初始位姿
 * @param pts3d 待优化点云
 * @param pts2d 对应的 2D 匹配点
 * @param K 相机内参
 * @param isSchur 是否使用舒尔补
 */
void solveBA(Vector6d& se_T, Points3d& pts3d, Points2d& pts2d,
             const Eigen::Matrix3d& K, bool isSchur = false);

#endif