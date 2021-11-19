# Ceres-Solver 求解 Bundle Adjustment

## 扰动模型

> [Computing Jacobian, why error state?——优化中为何对误差状态求导 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/75714471)

扰动模型的求导会比直接使用李代数求导效率高。求导的本质是自变量的微小变化导致的因变量的变化，这里是对旋转求导，但是旋转并没有加法定义，所以一种思路是转换为李代数，在李代数上推导导数。而另外一种就是给旋转一个微小变动，对于旋转也就是乘法，旋转一个微小的角度，这个角度趋于 0 时，在某个意义上也就获得了导数。但在高斯牛顿法中自变量的更新也需要变换成乘法。

也可以参考：[Modeling Non-linear Least Squares — Ceres Solver (ceres-solver.org)](http://www.ceres-solver.org/nnls_modeling.html#localparameterization)

其实就是重定义了加法操作

## 实验思路

实验主要就是参考 SLAM 14 讲，第二版 7.7、7.8 两节

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

这里初值没设置好，所以位姿估计的不好
20个点，DENSE_SCHUR 求解器

```
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  3.424539e+07    0.00e+00    3.02e+06   0.00e+00   0.00e+00  1.00e+04        0    1.09e-03    1.23e-03
   1  5.453723e+06    2.88e+07    3.41e+05   6.02e+01   8.41e-01  1.46e+04        1    2.99e-03    4.49e-03
   2  7.552582e+05    4.70e+06    3.38e+04   5.21e+01   8.62e-01  2.35e+04        1    3.48e-03    8.04e-03
   3  6.608772e+04    6.89e+05    5.45e+03   4.79e+01   9.12e-01  5.36e+04        1    4.37e-03    1.25e-02
   4  1.661990e+04    4.95e+04    1.56e+03   1.27e+02   7.49e-01  6.11e+04        1    2.55e-03    1.52e-02
   5  3.439891e+03    1.32e+04    6.83e+02   6.07e+01   7.93e-01  7.66e+04        1    2.65e-03    1.80e-02
   6  1.106311e+04   -7.62e+03    6.83e+02   2.43e+02  -2.22e+00  3.83e+04        1    1.84e-03    1.99e-02
   7  1.106317e+04   -7.62e+03    6.83e+02   2.43e+02  -2.22e+00  9.57e+03        1    1.80e-03    2.19e-02
   8  1.106356e+04   -7.62e+03    6.83e+02   2.43e+02  -2.22e+00  1.20e+03        1    1.91e-03    2.39e-02
   9  1.106719e+04   -7.63e+03    6.83e+02   2.42e+02  -2.22e+00  7.48e+01        1    3.07e-03    2.72e-02
  10  1.113019e+04   -7.69e+03    6.83e+02   2.40e+02  -2.24e+00  2.34e+00        1    2.21e-03    2.98e-02
  11  1.430359e+04   -1.09e+04    6.83e+02   1.87e+02  -3.39e+00  3.65e-02        1    2.59e-03    3.25e-02
  12  3.113777e+03    3.26e+02    6.42e+02   1.22e+01   1.09e+00  1.10e-01        1    3.68e-03    3.63e-02
  13  1.371896e+03    1.74e+03    6.94e+02   5.60e+01   1.94e+00  3.29e-01        1    2.69e-03    3.91e-02
  14  3.484878e+02    1.02e+03    4.33e+02   1.11e+01   1.21e+00  9.86e-01        1    7.68e-03    4.69e-02
  15  2.663561e+01    3.22e+02    9.94e+01   4.06e+00   1.06e+00  2.96e+00        1    3.63e-03    5.07e-02
  16  8.842315e-01    2.58e+01    1.74e+01   8.53e-01   9.98e-01  8.87e+00        1    3.55e-03    5.44e-02
  17  3.381783e-02    8.50e-01    1.11e+01   9.89e-02   9.67e-01  2.66e+01        1    6.81e-03    6.14e-02
  18  2.711129e-04    3.35e-02    9.09e-01   1.25e-02   9.92e-01  7.98e+01        1    5.10e-03    6.79e-02
  19  7.835649e-06    2.63e-04    8.69e+00   1.07e-03   9.71e-01  2.40e+02        1    3.46e-03    7.14e-02
  20  3.083847e-07    7.53e-06    6.07e+00   2.50e-04   9.61e-01  7.19e+02        1    4.25e-03    7.58e-02
  21  7.568643e-10    3.08e-07    3.35e-01   4.89e-05   9.98e-01  2.16e+03        1    9.73e-03    8.64e-02
  22  1.761639e-10    5.81e-10    1.59e-01   2.22e-06   7.67e-01  2.54e+03        1    3.68e-03    9.02e-02
Ceres Solver Report: Iterations: 23, Initial cost: 3.424539e+07, Final cost: 1.761639e-10, Termination: CONVERGENCE
真实位姿：
0.866158 -0.49977        0
 0.49977 0.866158        0
       0        0        1
 15.8
-20.1
   30
估计位姿：
 0.979308  0.158111 -0.126323
-0.128427  0.967934  0.215893
 0.156408 -0.195203  0.968211
 2.68984
-11.9803
-4.72939
```

