/**
 * @file main.cpp
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief 光束法平差 Ceres 实验
 * 自己生成 3D - 2D 匹配点
 * 然后调用 Ceres 优化
 * 优化思路参考 SLAMBOOK2 7.7 章节
 * 这里对位姿和空间点一起作了估计，位姿需要良好的初值，否则容易陷入局部最优
 * 这是非线性优化的固有问题
 * @version 0.1
 * @date 2021-11-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "utils.h"
#include "DataGenerator.h"
#include "BA.h"

int main(int argv, char **args) {
    // 构建相机内参矩阵 数据来自 slambook2
    Eigen::Matrix3d K;
    K << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
    // 定义位姿矩阵 欧式变换
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // 沿着 (1, 1, 1) 旋转 30°
    Eigen::AngleAxisd rotation_vec(3.14/6, Eigen::Vector3d(0, 0, 1).normalized());
    Eigen::Matrix3d R = rotation_vec.toRotationMatrix();
    // 平移向量
    Eigen::Vector3d t(15.8, -20.1, 30.0);
    // 确定位姿
    T.topLeftCorner(3, 3) = R;
    T.topRightCorner(3, 1) = t;

    // 数据生成
    DataGenerator dataGenerator(K, T);
    // 生成的点数量
    int nums = 20;
    Points3d pts3d(nums);
    Points2d pts2d(nums);
    dataGenerator.get3D_2D(nums, pts3d, pts2d);
    
    // 添加随机噪声
    cv::RNG rng;
    double sigma = 1.0;
    for(auto &p : pts3d) {
        p(0) += rng.gaussian(sigma);
        p(1) += rng.gaussian(sigma);
        p(2) += rng.gaussian(sigma);
    }

    // 这里使用 EPNP 估计初始位姿（也可以使用其它的）
    // 首先构造 OpenCV 形式的输入
    cv::Mat r_cv, t_cv, K_cv, R_cv;
    cv::eigen2cv(K, K_cv);
    std::vector<cv::Point3d> pts3d_cv;
    std::vector<cv::Point2d> pts2d_cv;
    for(int i = 0;i < nums;i++) {
        pts3d_cv.push_back(cv::Point3d(pts3d[i](0), pts3d[i](1), pts3d[i](2)));
        pts2d_cv.push_back(cv::Point2d(pts2d[i](0), pts2d[i](1)));
    }
    // EPNP 求解
    cv::solvePnP(pts3d_cv, pts2d_cv, K_cv, cv::Mat(), r_cv, t_cv, false, cv::SOLVEPNP_EPNP);
    cv::Rodrigues(r_cv, R_cv);

    std::cout.precision(5);
    std::cout << "真实位姿：\n旋转矩阵：\n" 
        << T.topLeftCorner(3,3) << "\n平移向量：" << t.transpose() << std::endl;

    // 问题求解
    BA ba(pts3d, pts2d, K);
    // 估计的位姿 非线性优化需要一个很好的初值，所以这里任意初值可能结果不是最优的
    // 所以使用 EPNP 估计的初始值
    Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_;
    cv::cv2eigen(t_cv, t_);
    cv::cv2eigen(R_cv, R_);
    std::cout << "EPNP 估计初始位姿：\n旋转矩阵：\n" << R_ << "\n平移向量：" << t_.transpose() << std::endl;
    // 这里连带空间点一起估计
    Points3d pts3d_ = pts3d;
    // 是否使用舒尔补求解 使用舒尔补在大量点的时候加速明显
    bool isSchur = true;
    ba.solve(R_, t_, pts3d_, isSchur);

    // 打印结果
    std::cout << "BA 估计位姿：\n旋转矩阵：\n" << R_ << "\n平移向量：" << t_.transpose() << std::endl;
    // 估计的空间点就不打印了

    return 0;
}
