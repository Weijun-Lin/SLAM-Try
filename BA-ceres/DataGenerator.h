/**
 * @file DataGenerator.h
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief 生成测试数据 3D-2D
 * @version 0.1
 * @date 2021-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DATAGENERATOR_H__
#define __DATAGENERATOR_H__

#include "utils.h"
#include <iostream>

class DataGenerator {
public:
    /**
     * @brief Construct a new Data Generator object
     * 
     * @param K 相机内参矩阵
     * @param T 欧式变换矩阵，即相机位姿
     */
    DataGenerator(const Eigen::Matrix3d &K, const Eigen::Matrix4d &T);
    // 获取数据 3D to 2D
    /**
     * @brief 获取3D到2D的数据
     * 1. 通过相机内参获取像素空间范围（正确成像到照片
     * 2. 随机选取像素点
     * 3. 获得每一个像素坐标对应的归一化坐标，并乘以随机深度，生成相机坐标系下空间点
     * 4. 通过位姿逆变换到世界坐标系
     * 
     * @param nums 生成数据的数量
     * @param pts3d 空间点坐标
     * @param pts2d 对应像素坐标
     */
    void get3D_2D(int nums, Points3d &pts3d, Points2d &pts2d);
private:
    // 相机内参矩阵
    Eigen::Matrix3d _K;
    // 相机位姿（外参）
    Eigen::Matrix4d _T;
};

#endif