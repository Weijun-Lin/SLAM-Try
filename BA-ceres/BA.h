#ifndef __BA_H__
#define __BA_H__

#include <Eigen/Core>
#include <vector>
#include <ceres/ceres.h>

typedef std::vector<Eigen::Vector3d> Points3D;
typedef std::vector<Eigen::Vector2d> Points2D;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

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
    Eigen::Vector3d _K;
};

// BA 问题求解类
class BA {
public:
    BA(const Points3D& pts3d, const Points2D& pts2d, const Eigen::Matrix3d &K);
    // 求解位姿以及空间点
    void solve(Eigen::Matrix3d &R, Eigen::Vector3d &T, Points3D &pts3d);
private:
    // 3D 点
    Points3D _points3d;
    // 对应的投影点
    Points2D _points2d;
    // 点数
    size_t _p_nums;
    // 相机内参矩阵
    Eigen::Matrix3d _K;
};


#endif