/**
 * @file DataGenerator.cpp
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief DataGenerator类 的实现
 * @version 0.1
 * @date 2021-11-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "DataGenerator.h"
#include <Eigen/Dense>

DataGenerator::DataGenerator(const Eigen::Matrix3d &K, const Eigen::Matrix4d &T):
    _K(K), _T(T) {}

void DataGenerator::get3D_2D(int nums, Points3d &pts3d, Points2d &pts2d) {
    // 首先判断数组大小是否一致
    assert(pts3d.size() == nums && pts2d.size() == nums);
    // 获取图片的宽高
    int width = 2*_K(0, 2), height = 2*_K(1, 2);
    srand(nums);
    // 获取相机位姿的逆
    // 这里也可以使用 SE(3) 矩阵的固有特点求逆
    Eigen::Matrix4d inverseT = _T.inverse();
    for(int i = 0;i < nums;i++) {
        int s = rand() % width, t = rand() % height;
        pts2d[i] = Point2d(s, t);
        // 获取归一化相机坐标，赋予随机深度，获得完整三维空间点
        int depth = (rand()%5 + 1)*5;
        Eigen::Vector4d temp;
        temp << depth*pixel2cam(s, t, _K), 1;
        // 然后转换到世界坐标系
        pts3d[i] = (inverseT*temp).head(3);
    }
}