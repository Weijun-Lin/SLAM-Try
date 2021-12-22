/**
 * @file main.cpp
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief 光束法平差 Ceres 实验
 * 自己生成 3D - 2D 匹配点
 * 然后调用 Ceres 优化
 * 优化思路参考 SLAMBOOK2 7.7 章节
 * 这里对位姿和空间点一起作了估计，位姿需要良好的初值，否则容易陷入局部最优
 * 这是非线性优化的固有问题
 * @version 0.2
 * @date 2021-12-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>

#include "BA.h"
#include "DataGenerator.h"
#include "utils.h"

void getTestData(Eigen::Matrix3d &K, Eigen::Matrix4d &T, Points3d &pts3d,
                 Points2d &pts2d, int nums) {
    // 构建外参矩阵
    T = Eigen::Matrix4d::Identity();
    T.topRightCorner(3, 1) = Eigen::Vector3d(0, 0, -20);
    // 构建内参矩阵 这里直接设置为单位矩阵
    K = Eigen::Matrix3d::Identity();
    // 数据生成
    DataGenerator dataGenerator(K, T);
    dataGenerator.getCircle3D_2D(nums, pts3d, pts2d);
    DataGenerator::out2File("3d_data_src.txt", pts3d);
}

int main(int argv, char **args) {
    // 构建相机内参矩阵 数据来自 slambook2
    Eigen::Matrix3d K;
    Eigen::Matrix4d T;

    // 生成的点数量
    int nums = 150;
    Points3d pts3d(nums);
    Points2d pts2d(nums);

    // 获取数据
    getTestData(K, T, pts3d, pts2d, nums);

    Sophus::SE3d SE_T(T);
    Vector6d se_T = SE_T.log();
    std::cout.precision(5);
    std::cout << "真实位姿：\n" << se_T.transpose() << std::endl;

    // 添加高斯噪声
    Points3d pts3d_noise = pts3d;
    Vector6d se_T_noise = se_T;
    cv::RNG rng;
    double p_sigma = 0.5;
    for (auto &p : pts3d_noise) {
        p(0) += rng.gaussian(p_sigma);
        p(1) += rng.gaussian(p_sigma);
        p(2) += rng.gaussian(p_sigma);
    }
    // 对位姿同样做噪声
    DataGenerator::out2File("3d_data_noise.txt", pts3d_noise);
    double r_sigma = 0.005;
    double t_sigma = 0.05;
    for (int i = 0; i < 3; i++) {
        se_T_noise(i) += rng.gaussian(t_sigma);
        se_T_noise(i + 3) += rng.gaussian(r_sigma);
    }
    se_T_noise = se_T_noise.array();
    std::cout.precision(5);
    std::cout << "噪声扰动后位姿：\n" << se_T_noise.transpose() << std::endl;

    // 这里连带空间点一起估计
    // 是否使用舒尔补求解 使用舒尔补在大量点的时候加速明显
    Points3d pts3d_opt = pts3d_noise;
    Vector6d se_T_opt = se_T_noise;
    bool isSchur = true;
    solveBA(se_T_opt, pts3d_opt, pts2d, K, isSchur);

    // 打印结果
    std::cout << "优化后位姿：\n" << se_T_opt.transpose() << std::endl;
    // 估计的空间点导出
    DataGenerator::out2File("3d_data_opt.txt", pts3d_opt);

    // 量化估计 估计欧氏距离

    // 位姿
    double SSE_T_noise = (se_T_noise - se_T).norm();
    double SSE_T_opt = (se_T_opt - se_T).norm();
    // 空间点
    double SSE_p_noise = 0;
    double SSE_p_opt = 0;
    for (int i = 0; i < nums; i++) {
        SSE_p_noise += (pts3d_noise[i] - pts3d[i]).norm();
        SSE_p_opt += (pts3d_opt[i] - pts3d[i]).norm();
    }
    std::cout << "位姿误差：\n"
              << "优化前：" << SSE_T_noise << "\n优化后：" << SSE_T_opt
              << std::endl;
    std::cout << "点云误差\n"
              << "优化前：" << SSE_p_noise / nums << "\n优化后："
              << SSE_p_opt / nums << std::endl;
    return 0;
}