# 120×120 稠密矩阵岭回归求解器

本项目同时提供单精度 `float` 和双精度 `double` 两套接口，面向 IAR Embedded Workbench for Arm 9.40.1 和 Cortex-A7。项目只包含目标程序需要的 C 源码、头文件、Netlib CLAPACK 源码、许可证和示例，不需要 Python、CMake、PowerShell 或 Fortran 编译器。

所有 `.c` 和 `.h` 文件均采用 UTF-8 编码。`examples/main.c` 使用中文注释，其余源码保留上游或项目原来的英文注释。

## 求解公式

矩阵尺寸为：`X` 是 120×120，`Y` 是 m×120，`B` 是 m×120。两套接口都求解：

```text
min ||B*X-Y||_F^2 + lambda*||B||_F^2，lambda > 0
```

程序构造：

```text
A = X*X^T + lambda*I
C = X*Y^T
```

然后通过 Cholesky 分解求解 `A*B^T=C`，不会显式计算逆矩阵。

- 单精度接口调用 `spotf2_` 和 `spotrs_`。
- 双精度接口调用 `dpotf2_` 和 `dpotrs_`。

## 两个接口

```c
int ridge_solve_f32(const float *x, const float *y, size_t rows,
                    float lambda, float *b, ridge_workspace_f32 *work);

int ridge_solve_f64(const double *x, const double *y, size_t rows,
                    double lambda, double *b, ridge_workspace_f64 *work);
```

`f32` 是完整的单精度计算，包括 Gram 矩阵构造、Cholesky 分解和三角方程求解。`f64` 全程使用双精度。两者不是简单地在接口处转换类型。

所有数组均为 C 行优先排列：

```text
X[120][120]
Y[rows][120]
B[rows][120]
```

函数内部处理 LAPACK 的列优先布局，调用者不需要转置。`X`、`Y`、`B` 和工作区的内存不能重叠。返回值不是 `RIDGE_OK` 时不能使用 B。

## 调用示例

单精度：

```c
static float X[RIDGE_N][RIDGE_N];
static float Y[3][RIDGE_N];
static float B[3][RIDGE_N];
static ridge_workspace_f32 work;

int status = ridge_solve_f32(&X[0][0], &Y[0][0], 3,
                             0.1F, &B[0][0], &work);
```

双精度：

```c
static double X[RIDGE_N][RIDGE_N];
static double Y[3][RIDGE_N];
static double B[3][RIDGE_N];
static ridge_workspace_f64 work;

int status = ridge_solve_f64(&X[0][0], &Y[0][0], 3,
                             0.1, &B[0][0], &work);
```

返回值：

```text
 0  RIDGE_OK       成功
-1  RIDGE_EINVAL   参数或 lambda 无效
-2  RIDGE_ENUMERIC 输入或中间结果出现 NaN、Inf 或溢出
-3  RIDGE_ENOTSPD  Cholesky 分解失败
-4  RIDGE_ELAPACK  LAPACK 参数错误
```

## 加入 IAR 9.40.1 工程

将以下 20 个 `.c` 文件加入已有 Cortex-A7 工程：

```text
src/ridge.c
src/xerbla.c
examples/main.c

third_party/clapack/SRC/spotf2.c
third_party/clapack/SRC/spotrs.c
third_party/clapack/SRC/sisnan.c
third_party/clapack/SRC/slaisnan.c
third_party/clapack/SRC/dpotf2.c
third_party/clapack/SRC/dpotrs.c
third_party/clapack/SRC/disnan.c
third_party/clapack/SRC/dlaisnan.c

third_party/clapack/BLAS/SRC/sdot.c
third_party/clapack/BLAS/SRC/sgemv.c
third_party/clapack/BLAS/SRC/sscal.c
third_party/clapack/BLAS/SRC/strsm.c
third_party/clapack/BLAS/SRC/ddot.c
third_party/clapack/BLAS/SRC/dgemv.c
third_party/clapack/BLAS/SRC/dscal.c
third_party/clapack/BLAS/SRC/dtrsm.c
third_party/clapack/BLAS/SRC/lsame.c
```

增加头文件搜索路径：

```text
include
third_party/clapack/INCLUDE
```

增加预处理宏：

```text
NO_BLAS_WRAP
```

源码采用 C89/C90 写法。目标选择 Cortex-A7、小端；如果启用 VFPv4，所有目标文件、运行库、浮点 ABI 和板级 FPU 初始化必须使用一致配置。

`examples/main.c` 会依次运行单精度和双精度接口。接入实际程序时，可以删除该文件，继续使用已有的启动代码、任务入口和链接配置文件。

## 内存占用

算法不使用动态内存。在 32 位 IAR Arm ABI 下：

| 数据 | 单精度 | 双精度 |
|---|---:|---:|
| X | 57,600 字节 | 115,200 字节 |
| Y | `480*m` 字节 | `960*m` 字节 |
| B | `480*m` 字节 | `960*m` 字节 |
| 工作区 | 约 59,052 字节 | 约 118,104 字节 |

工作区和大矩阵应采用静态存储或放入已经初始化的 DDR，不要作为局部变量放入小栈中。实际应用只需要分配所调用接口对应的工作区。

单精度通常更快且占用约一半数据内存，但对病态矩阵和很小的 lambda 更敏感。应使用实际 X、Y 比较两个接口的 B 和残差，再决定最终采用哪种精度。

CLAPACK 源码来自 Netlib 官方 3.2.1 发布包：https://www.netlib.org/clapack/
