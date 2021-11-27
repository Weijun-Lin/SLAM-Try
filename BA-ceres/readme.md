# Ceres-Solver 求解 Bundle Adjustment

## 扰动模型

> [Computing Jacobian, why error state?——优化中为何对误差状态求导 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/75714471)

扰动模型的求导会比直接使用李代数求导效率高。求导的本质是自变量的微小变化导致的因变量的变化，这里是对旋转求导，但是旋转并没有加法定义，所以一种思路是转换为李代数，在李代数上推导导数。而另外一种就是给旋转一个微小变动，对于旋转也就是乘法，旋转一个微小的角度，这个角度趋于 0 时，在某个意义上也就获得了导数。但在高斯牛顿法中自变量的更新也需要变换成乘法。

也可以参考：[Modeling Non-linear Least Squares — Ceres Solver (ceres-solver.org)](http://www.ceres-solver.org/nnls_modeling.html#localparameterization)

其实就是重定义了加法操作

## 实验思路

实验主要就是参考 SLAM 14 讲，第二版 7.7、7.8 两节（看到后面发现第九章也讲了对点的优化样例，但这里只有一个相机位姿，也很容易拓展）

作者给的是对位姿估计，但也可以同时对位姿和空间点估计。当然这会导致求解矩阵有很高的维数，所以一般利用舒尔补加速求解，可参考：[SLAM中的滑窗（Sliding window） - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/92100386)

因为使用扰动模型，所以需要重新配置 Ceres 的参数化 `LocalParameterization`，这里没有使用自动求导，所以需要重写 `SizedCostFunction`，舒尔补还需要对求解器做相关配置，主要涉及  `ParameterBlockOrdering`，`Options::minimizer_progress_to_stdout `推荐把 Ceres 这一部分的官方文档看一下 [Modeling Non-linear Least Squares — Ceres Solver (ceres-solver.org)](http://www.ceres-solver.org/nnls_modeling.html#) 中文也可参考：[Ceres Modeling Non-linear Least Squares - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/98051344)

主要类如下：

### 数据生成 DataGenerator

```c++
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
```

### Ceres 求解构造

参数化

```c++
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
```

BA 损失函数

```cpp
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
```

BA 问题求解

```cpp
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
```

## 结果

非线性优化初值要求很高，所以这里使用 EPNP 构建初值
20个点，DENSE_SCHUR 求解器，结果如下：

```text
真实位姿：
旋转矩阵：
 0.86616 -0.49977        0
 0.49977  0.86616        0
       0        0        1
平移向量： 15.8 -20.1    30
EPNP 估计初始位姿：
旋转矩阵：
   0.86103   -0.50822  -0.018548
   0.50823    0.86121 -0.0042374
  0.018127 -0.0057783    0.99982
平移向量： 15.902 -19.819  31.049
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  5.500836e+04    0.00e+00    1.88e+04   0.00e+00   0.00e+00  1.00e+04        0    9.56e-04    3.49e-03
   1  1.893635e+04    3.61e+04    9.21e+03   1.07e+01   6.56e-01  1.03e+04        1    2.74e-02    3.10e-02
   2  1.601201e+03    1.73e+04    1.93e+03   1.94e+00   9.15e-01  2.42e+04        1    4.10e-03    3.52e-02
   3  1.903984e+02    1.41e+03    5.60e+02   9.05e-01   8.81e-01  4.34e+04        1    1.76e-03    3.70e-02
   4  4.952934e+01    1.41e+02    2.86e+02   2.67e-01   7.40e-01  4.88e+04        1    1.92e-03    3.93e-02
   5  1.634827e+01    3.32e+01    1.55e+02   1.22e-01   6.70e-01  5.08e+04        1    1.85e-03    4.12e-02
   6  6.252770e+00    1.01e+01    9.27e+01   6.64e-02   6.18e-01  5.14e+04        1    2.23e-03    4.37e-02
   7  2.491544e+00    3.76e+00    6.04e+01   4.02e-02   6.02e-01  5.19e+04        1    1.41e-03    4.52e-02
   8  1.007743e+00    1.48e+00    3.91e+01   2.52e-02   5.96e-01  5.22e+04        1    2.09e-03    4.74e-02
   9  4.098408e-01    5.98e-01    2.52e+01   1.60e-02   5.93e-01  5.26e+04        1    2.37e-03    4.98e-02
  10  1.670566e-01    2.43e-01    1.61e+01   1.02e-02   5.92e-01  5.29e+04        1    1.45e-03    5.13e-02
  11  6.815992e-02    9.89e-02    1.46e+01   6.51e-03   5.92e-01  5.33e+04        1    1.69e-03    5.30e-02
  12  2.782098e-02    4.03e-02    1.01e+01   4.16e-03   5.92e-01  5.36e+04        1    1.53e-03    5.46e-02
  13  1.135767e-02    1.65e-02    5.51e+01   2.66e-03   5.92e-01  5.39e+04        1    1.49e-03    5.62e-02
  14  4.636970e-03    6.72e-03    2.94e+01   1.70e-03   5.92e-01  5.43e+04        1    1.97e-03    5.82e-02
  15  1.893163e-03    2.74e-03    4.17e+01   1.08e-03   5.92e-01  5.46e+04        1    1.92e-03    6.02e-02
  16  7.729350e-04    1.12e-03    1.59e+01   6.93e-04   5.92e-01  5.49e+04        1    1.29e-03    6.15e-02
  17  3.155706e-04    4.57e-04    5.02e+01   4.43e-04   5.92e-01  5.53e+04        1    1.51e-03    6.30e-02
  18  1.288393e-04    1.87e-04    4.27e+01   2.83e-04   5.92e-01  5.56e+04        1    2.08e-03    6.52e-02
  19  5.260151e-05    7.62e-05    5.30e+01   1.81e-04   5.92e-01  5.60e+04        1    1.30e-03    6.67e-02
  20  2.147568e-05    3.11e-05    1.71e+01   1.16e-04   5.92e-01  5.63e+04        1    1.37e-03    6.81e-02
  21  8.767877e-06    1.27e-05    4.63e+01   7.38e-05   5.92e-01  5.67e+04        1    1.39e-03    6.95e-02
  22  3.579656e-06    5.19e-06    1.02e+01   4.72e-05   5.92e-01  5.70e+04        1    1.39e-03    7.09e-02
  23  1.461463e-06    2.12e-06    5.08e+01   3.01e-05   5.92e-01  5.74e+04        1    2.22e-03    7.33e-02
  24  5.966695e-07    8.65e-07    3.98e+01   1.93e-05   5.92e-01  5.77e+04        1    2.26e-03    7.57e-02
  25  2.436014e-07    3.53e-07    2.13e+01   1.23e-05   5.92e-01  5.81e+04        1    1.30e-03    7.70e-02
  26  9.945472e-08    1.44e-07    1.31e+01   7.86e-06   5.92e-01  5.84e+04        1    1.34e-03    7.84e-02
  27  4.060421e-08    5.89e-08    8.62e+00   5.02e-06   5.92e-01  5.88e+04        1    1.34e-03    7.98e-02
  28  1.657741e-08    2.40e-08    5.61e+00   3.21e-06   5.92e-01  5.92e+04        1    1.70e-03    8.15e-02
  29  6.768026e-09    9.81e-09    3.62e+00   2.05e-06   5.92e-01  5.95e+04        1    1.70e-03    8.34e-02
Ceres Solver Report: Iterations: 30, Initial cost: 5.500836e+04, Final cost: 6.768026e-09, Termination: CONVERGENCE
BA 估计位姿：
旋转矩阵：
   0.86658   -0.49739  -0.040689
   0.49684    0.86753  -0.023285
   0.04688 -3.818e-05     0.9989
平移向量： 15.602 -20.125  31.647
```

