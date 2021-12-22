/**
 * @file DataGenerator.h
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief 生成测试数据 3D-2D
 * @version 0.2
 * @date 2021-12-22
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __DATAGENERATOR_H__
#define __DATAGENERATOR_H__

#include <iostream>

#include "utils.h"

class DataGenerator {
   public:
    /**
     * @brief Construct a new Data Generator object
     *
     * @param K 相机内参矩阵
     * @param T 欧式变换矩阵，即相机位姿
     */
    DataGenerator(const Eigen::Matrix3d &K, const Eigen::Matrix4d &T);

    /**
     * @brief Get the Circle3D 2D Data
     * 1. 三维空间生成倾斜的圆形，3D 数据是固定的
     * 2. 投影到相机坐标系
     *
     * @param nums 生成数据的数量
     * @param pts3d 空间点坐标
     * @param pts2d 对应的2D坐标 这里不保证都是正数
     */
    void getCircle3D_2D(int nums, Points3d &pts3d, Points2d &pts2d);

    /**
     * @brief 输出到文件
     *
     * @param filename 文件名
     * @param pts3d 三维点云
     */
    static void out2File(const char *filename, const Points3d &pts3d);

   private:
    // 相机内参矩阵
    Eigen::Matrix3d _K;
    // 相机位姿（外参）
    Eigen::Matrix4d _T;
};

#endif