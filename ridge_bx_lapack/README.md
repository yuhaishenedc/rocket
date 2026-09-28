# 120×120 单精度岭回归求解器

本项目仅保留 `float` 岭回归接口，面向 IAR Embedded Workbench for Arm 9.40.1 和 Cortex-A7。姿态、角速度、法方程、回归系数和预测热路径均使用单精度；业务代码中的绝对时间、时间差及预测时刻继续使用 `double`，避免长时间运行时丢失时标分辨率。

## 求解方法

对于 `X[120][120]`、`Y[rows][120]` 和 `B[rows][120]`，求解：

```text
min ||B*X-Y||_F² + lambda*||B||_F²，lambda > 0
```

程序构造：

```text
A = X*Xᵀ + lambda*I
C = X*Yᵀ
```

随后调用单精度 packed LAPACK 接口 `spptrf_` 和 `spptrs_`，对 `A` 做一次 Cholesky 分解并直接求解，不构造逆矩阵。对称矩阵只保存下三角，共 `RIDGE_PACKED_SIZE=7260` 个 `float` 元素。

## 接口

从原始 `X`、`Y` 构造并求解：

```c
int ridge_solve_f32(const float *x, const float *y, size_t rows,
                    float lambda, float *b, ridge_workspace_f32 *work);
```

业务代码已经构造法方程时，先取得 packed 矩阵区，再一次求解六个右端：

```c
static ridge_normal_workspace_f32 work;
float *packedXTX = ridge_normal_packed_matrix_f32(&work);

packedXTX[ridge_packed_lower_index(row, col)] = value;

int status = ridge_solve_normal_f32(&XTY[0][0], lambda,
                                    &coefficients[0][0], &work);
```

求解函数会把 `lambda` 加到实际参与分解的矩阵对角线上。分解或求解失败时不会发布新的系数矩阵。packed 矩阵会被 Cholesky 因子覆盖，因此每次求解前必须重新构建。

`IF_CombinedNavi.c` 另行保存一份未分解的 packed 法方程，并从第一条完整训练行出现后随每个新 100 ms 样本增量维护。边界平滑数据修改前先减去受影响训练行，修改后再加入新贡献；进入 method2 时只复制法方程并执行求解，不进行一次性全量初始化。没有新样本的 20 ms 控制周期不更新法方程。

返回值：

```text
 0  RIDGE_OK       成功
-1  RIDGE_EINVAL   参数或lambda无效
-2  RIDGE_ENUMERIC 输入或中间结果出现NaN、Inf或溢出
-3  RIDGE_ENOTSPD  Cholesky分解失败
-4  RIDGE_ELAPACK  LAPACK参数错误
```

## 加入 IAR 9.40.1 工程

通用求解接口与六右端法方程接口的最小源码集合为：

```text
src/ridge.c
src/xerbla.c
examples/main.c

third_party/clapack/SRC/spptrf.c
third_party/clapack/SRC/spptrs.c

third_party/clapack/BLAS/SRC/sdot.c
third_party/clapack/BLAS/SRC/sscal.c
third_party/clapack/BLAS/SRC/sspr.c
third_party/clapack/BLAS/SRC/stpsv.c
third_party/clapack/BLAS/SRC/lsame.c
```

头文件搜索路径：

```text
include
third_party/clapack/INCLUDE
```

预处理宏：

```text
NO_BLAS_WRAP
```

所有目标文件、运行库、浮点 ABI 和板级 FPU 初始化必须使用一致配置。接入实际程序时可删除 `examples/main.c`，继续使用现有启动代码和任务入口。

## 内存占用

算法不使用动态内存。在 32 位 ABI 下：

| 数据 | 占用 |
|---|---:|
| X | 57,600 字节 |
| Y | `480*rows` 字节 |
| B | `480*rows` 字节 |
| 通用求解工作区 | 约 30,012 字节 |
| 六右端法方程工作区 | 约 32,412 字节 |

工作区和大矩阵应使用静态存储或放入已初始化的 DDR，避免占用任务栈。单精度对特征尺度、矩阵条件数和很小的 `lambda` 更敏感，部署前应使用实际数据检查预测误差和法方程残差。

CLAPACK 源码来自 Netlib 官方 3.2.1 发布包：https://www.netlib.org/clapack/
