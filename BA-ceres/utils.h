/**
 * @file utils.h
 * @author weijun-lin (weijun-lin@foxmail.com)
 * @brief 一些工具函数或类型定义
 * @version 0.1
 * @date 2021-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __UTILS_H_
#define __UTILS_H_

#include <vector>
#include <Eigen/Core>

#define SEE(x) std::cout << #x << ":\n" << x << std::endl;

typedef Eigen::Vector2d Point2d;
typedef Eigen::Vector3d Point3d;
typedef std::vector<Point3d> Points3d;
typedef std::vector<Point2d> Points2d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief 像素坐标转换到相机坐标系(x,y,1)
 * 
 * @param x x
 * @param y y
 * @param K 相机内参矩阵
 * @return Point3d 相机坐标系归一化坐标
 */
inline Point3d pixel2cam(size_t x, size_t y, const Eigen::Matrix3d &K) {
  return Point3d
    (
      (x - K(0, 2)) / K(0, 0),
      (y - K(1, 2)) / K(1, 1),
      1
    );
}

#endif