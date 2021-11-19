/**
 * @file ceres_test.cpp
 * @author Weijun-Lin (weijun-lin@foxmail.com)
 * @brief 使用 Ceres 求解 y = a*sin(bx+c)+d + noise
 * @version 0.1
 * @date 2021-11-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;
// 数据量（点数）
const int N = 10000;

/**
 * @brief 构建仿函数 定义残差计算
 * 
 */
struct MyCostFunc {
    MyCostFunc(double x, double y): x(x), y(y) {}

    // 重写括号运算符 使用当前估计的参数计算残差 y - a*a*sin(bx+c)+d
    template<typename T>
    bool operator()(const T* const params, T* residual) const {
        T a = params[0], b = params[1], c = params[2], d = params[3];
        residual[0] = T(y) - a * ceres::sin(b*x + c) - d;
        return true;
    }

    double x, y;
};

int main() {
    // 估计函数 y = a*sin(bx + c) + d + noise
    // 实际参数
    double abcd_r[] = {1.5, PI/4.0, PI/6.0, 2.0};
    // 预估参数
    double abcd_e[] = {0.5, 1.0, 0.0, 0.0};
    // 生成 N 个 0 - 2pi 的 x 数据
    Array<double, N, 1> x_data = ArrayXd::LinSpaced(N, 0, 15*PI);
    Matrix<double, N, 1> y_data;
    cv::RNG rng;
    double sigma = 1.0;
    y_data = abcd_r[0]*Eigen::sin(abcd_r[1]*x_data + abcd_r[2]) + abcd_r[3];
    for(int i = 0;i < N;i++) {
        y_data(i) = y_data(i) + rng.gaussian(sigma*sigma);
    }

    // 构建问题
    ceres::Problem problem;
    // 调用 Ceres 求解
    for(int i = 0;i < N;i++) {
        // 创建损失函数 使用自动求导
        ceres::CostFunction* costfunction = 
            new ceres::AutoDiffCostFunction<MyCostFunc, 1, 4>(
                new MyCostFunc(x_data(i), y_data(i))
            );
        // 添加到问题
        problem.AddResidualBlock(costfunction, nullptr, abcd_e);
    }
    // 配置求解器
    // 求解器配置 求解方式以及是否输出到标准输出
    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_QR;
    option.minimizer_progress_to_stdout = true;

    // 存储优化信息
    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(option, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // 打印所花时间
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
    // 打印报告
    cout << summary.BriefReport() << "\n" << endl;
    cout << "src\t:";
    for(auto i : abcd_r) {
        cout << i << " ";
    }
    cout << endl;
    cout << "final\t:";
    for(auto i : abcd_e) {
        cout << i << " ";
    }
    cout << endl;
    return 0;
}