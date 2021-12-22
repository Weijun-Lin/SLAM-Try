/**
 * @file DataGenerator.cpp
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief DataGenerator类 的实现
 * @version 0.2
 * @date 2021-12-22
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "DataGenerator.h"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

DataGenerator::DataGenerator(const Eigen::Matrix3d &K, const Eigen::Matrix4d &T)
    : _K(K), _T(T) {}

void DataGenerator::getCircle3D_2D(int nums, Points3d &pts3d, Points2d &pts2d) {
    // 首先判断数组大小是否一致
    assert(pts3d.size() == nums && pts2d.size() == nums);
    // 首先生成三维空间圆形，在 xy 平面生成单位圆，然后绕 x 轴旋转45°
    int r = 10;
    double delta = 2 * M_PI / nums;
    double theta = 0;
    // 然后绕 x 轴旋转 45°
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(1, 0, 0));
    for (auto &p : pts3d) {
        p << r*cos(theta), r*sin(theta), 0;
        theta += delta;
        p = rotation_vector * p;
    }
    // 投影到相机坐标
    for (int i = 0; i < nums; i++) {
        Eigen::Vector4d p_;
        p_ << pts3d[i], 1;
        Point3d p = _K * (_T * p_).head(3);
        p = p / p(2);
        pts2d[i] = p.head(2);
    }
}

void DataGenerator::out2File(const char *filename, const Points3d &pts3d) {
    std::ofstream out;
    out.open(filename, std::ios::out);
    if (!out.is_open()) {
        std::cout << "File " << filename << " Open Error\n";
        return;
    } else {
        std::cout << "File " << filename << " Open Successfully\n";
    }
    for (auto p : pts3d) {
        out << p(0) << "," << p(1) << "," << p(2) << "\n";
    }
    out.close();
}