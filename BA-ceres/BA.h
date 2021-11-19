/**
 * @file BA.h
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief BA 问题类定义
 * @version 0.1
 * @date 2021-11-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __BA_H__
#define __BA_H__

#include "utils.h"
#include <Eigen/Core>
#include <vector>
#include <ceres/ceres.h>

// SE3 的参数化，可参考：https://blog.csdn.net/u012348774/article/details/84144084
class SE3LocalParameterization: public ceres::LocalParameterization {
public:
    // 扰动模型更新
    virtual bool Plus(const double* x,
                const double* delta,
                double* x_plus_delta) const;
    // 计算局部雅可比 \partial x / \partial \delta
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;

    // Size of x.
    virtual int GlobalSize() const;

    // Size of delta.
    virtual int LocalSize() const;
};

// BA 的损失函数构建 损失为2个维度，参数块分别为位姿 6 和一个空间点 3
class BACostFunction: public ceres::SizedCostFunction<2, 6, 3> {
public:
    BACostFunction(const Eigen::Vector2d &ui, const Eigen::Matrix3d &K);
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const;
private:
    Eigen::Vector2d _ui;
    Eigen::Matrix3d _K;
};

// BA 问题求解类
class BA {
public:
    BA(const Points3d& pts3d, const Points2d& pts2d, const Eigen::Matrix3d &K);
    /**
     * @brief 光束法平差求解位姿以及空间点
     * 
     * @param R 旋转矩阵
     * @param T 平移向量
     * @param pts3d 优化之后的空间点
     * @param isSchur 是否使用舒尔补优化
     */
    void solve(Eigen::Matrix3d &R, Eigen::Vector3d &T, Points3d &pts3d, bool isSchur = false);
private:
    // 3D 点
    Points3d _points3d;
    // 对应的投影点
    Points2d _points2d;
    // 点数
    size_t _nums;
    // 相机内参矩阵
    Eigen::Matrix3d _K;
};


#endif