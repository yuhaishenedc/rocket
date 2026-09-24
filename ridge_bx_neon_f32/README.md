# Cortex-A7 FP32 NEON 岭回归求解器

本目录是独立的单精度版本，求解：

```text
min ||B*X-Y||_F^2 + lambda*||B||_F^2
```

其中 `X` 固定为 120×120 的行优先稠密矩阵，`Y` 和 `B` 为
`rows×120`。程序构造：

```text
A = X*X^T + lambda*I
C = X*Y^T
```

然后用 Netlib CLAPACK 的 `spotf2_` 和 `spotrs_` 求解
`A*B^T=C`。程序不会显式计算逆矩阵。

`src/ridge_neon_kernels.c` 是唯一显式使用 NEON intrinsic 的文件，负责
长度为 120 的 FP32 点积。其余求解流程和 CLAPACK 保持普通 C 实现。公开接口
只包含普通 C 类型，不会把 `float32x4_t` 传播到已有代码。

## 文件

```text
include/ridge_neon.h             公开接口和工作区
src/ridge_neon.c                 岭回归、参数检查和 LAPACK 调用
src/ridge_neon_kernels.c         NEON FP32 点积内核
src/ridge_neon_kernels.h         内部接口
src/xerbla.c                     CLAPACK 参数错误处理
examples/main.c                  无 RTOS 的中文示例
third_party/clapack              所需的官方 Netlib CLAPACK 源码
```

CLAPACK 源码来自 Netlib CLAPACK 3.2.1 发布包，许可证位于
`third_party/clapack/COPYING`。

## 接口

```c
int ridge_solve_f32_neon(const float *x,
                         const float *y,
                         size_t rows,
                         float lambda,
                         float *b,
                         ridge_neon_workspace_f32 *work);
```

所有矩阵均为行优先：

```text
X[120][120]
Y[rows][120]
B[rows][120]
```

`work` 必须使用静态存储或已初始化的全局/静态内存。首次调用前必须保证
`work->busy == 0`。每个可能并发的调用都必须有独立工作区；更推荐由一个固定
任务独占调用。函数不使用动态内存。

## 加入 IAR Embedded Workbench for Arm 9.40.1

将下列 13 个 C 文件加入工程：

```text
src/ridge_neon.c
src/ridge_neon_kernels.c
src/xerbla.c
examples/main.c

third_party/clapack/SRC/spotf2.c
third_party/clapack/SRC/spotrs.c
third_party/clapack/SRC/sisnan.c
third_party/clapack/SRC/slaisnan.c

third_party/clapack/BLAS/SRC/sdot.c
third_party/clapack/BLAS/SRC/sgemv.c
third_party/clapack/BLAS/SRC/sscal.c
third_party/clapack/BLAS/SRC/strsm.c
third_party/clapack/BLAS/SRC/lsame.c
```

如果已有工程已经定义自己的 `main()`，不要加入 `examples/main.c`。

增加头文件搜索路径：

```text
include
src
third_party/clapack/INCLUDE
```

增加预处理宏：

```text
NO_BLAS_WRAP
```

所有文件保持相同的浮点调用 ABI。若现有工程使用 VFP 寄存器传参，则所有新增
文件和预编译库都必须使用 `--aapcs=vfp`；不要混合 softfp 与 hardfp 对象。

### IAR 目标设置

在 `Project > Options > General Options > Target` 中确认：

```text
Core                    Cortex-A7
Execution state         32-bit AArch32（ARM 或 Thumb-2 沿用现有工程）
FPU                     VFPv4
D registers             32
Advanced SIMD           NEON
```

不要选择仅有 16 个 D 寄存器的配置。`ridge_neon_kernels.c` 包含编译期检查；
如果 IAR 没有定义 `__ARM_NEON`、`__ARM_NEON_FP` 或硬件 VFP 宏，该文件会停止
编译，而不会静默退化为标量实现。

## 让 NEON 只影响这个计算

推荐使用以下设置：

1. 工程目标允许 Cortex-A7 NEON，这是编译 intrinsic 的必要条件。
2. 在工程级 `C/C++ Compiler > Optimizations` 中关闭自动向量化。
3. 只有 `src/ridge_neon_kernels.c` 包含 `<arm_neon.h>` 并调用 NEON intrinsic。
4. 不要将 NEON 内核写入公共头文件，也不要开启跨模块内联。
5. 对已有模块查看反汇编，确认没有 `Q0-Q15`、`VLD1` 或 `VST1` 指令。

开启 Advanced SIMD 表示编译器可以接受 NEON 指令，并不等于所有文件都会自动
向量化。关闭自动向量化并只在一个 `.c` 文件中使用 intrinsic，可以把显式 NEON
代码限制在该目标文件。

如果必须从目标文件层面严格保证已有模块完全不含 NEON，可建立两个 IAR 工程：

```text
应用工程                 维持原来的非 NEON 编译选项
NEON 静态库工程          只编译 ridge_neon_kernels.c，目标为 Cortex-A7+NEON
```

应用工程中的 `ridge_neon.c` 和 CLAPACK 使用原有配置，再链接 NEON 静态库。两个
工程的 endian、ARM/Thumb 模式约束、`sizeof(int/long)`、VFP ABI 和运行库配置必须
一致。最终仍要让链接器的目标设备允许 Cortex-A7 NEON，并检查链接 Map 和反汇编。

## RTOS 集成

“每个控制周期只调用一次”只能避免算法重入，不能保护 NEON 寄存器。计算执行期间
仍可能发生任务切换或中断。NEON 的 `Q0-Q15` 与 VFP 的 `D0-D31` 是同一组物理
寄存器，因此应从以下两种方式选择一种。

### 方式 A：RTOS 保存完整 VFP/NEON 上下文，推荐

将求解器固定到一个计算任务，并通过所用 RTOS 的端口接口把该任务声明为浮点任务。
确认任务切换代码保存并恢复：

```text
D0-D31（等价于 Q0-Q15）
FPSCR
必要时的 FPEXC 状态
```

有些 RTOS 采用 lazy context switching，只保存当前浮点所有者的寄存器。这种机制
可以使用，但必须确认 Cortex-A7 端口支持完整的 32 个 D 寄存器，而不是只处理
`D0-D15`。任务可正常被抢占，不需要长时间关闭中断。

### 方式 B：调用期间禁止上下文切换

如果 RTOS 确实没有 VFP/NEON 上下文支持，可在调用前锁住调度器，调用结束后恢复：

```c
rtos_scheduler_lock();
status = ridge_solve_f32_neon(x, y, rows, lambda, b, &work);
rtos_scheduler_unlock();
```

此方式还要求所有可能打断该代码的 ISR 都不使用 VFP/NEON，并且 ISR 不调用会使用
浮点的库函数。如果做不到，需要在整个调用期间屏蔽这些中断，或者为中断实现完整
浮点上下文保存。求解 120×120 系统可能耗时较长，长时间锁调度器或关中断会增加
实时延迟，必须用硬件计时器测量最坏执行时间后再采用。

`workspace.busy` 只是诊断性重入检查，不是原子锁，不能替代 RTOS mutex、任务独占
或调度器锁。

### RTOS 验收测试

1. 在 NEON 求解期间，让高优先级任务频繁抢占计算任务。
2. 让其他任务运行普通整数负载；若系统允许其他任务使用浮点，再加入已知浮点负载。
3. 连续运行多个控制周期，比较每次输出和残差。
4. 如果一开启抢占结果就随机变化，优先检查 `D0-D31`、FPSCR 和 FPEXC 的保存恢复。
5. SMP 系统把计算任务固定在一个 Cortex-A7 核上，除非 RTOS 支持跨核迁移浮点上下文。

## 启动代码

进入任何 VFP/NEON C 代码之前必须完成：

```text
CPACR.CP10/CP11 = 完全访问
CPACR.ASEDIS    = 0
FPEXC.EN        = 1
```

Non-secure 程序还需要 Secure Monitor 设置 `NSACR.CP10/CP11`。如果 BSP 使用
CMSIS-Core(A)，可在启动阶段调用对应的 FPU enable 函数。工程选项只能生成 NEON
指令，不能代替处理器启动初始化。

## 确认实际使用 NEON

在 C-SPY 中打开 `ridge_neon_dot120_f32` 的 Disassembly。应看到类似：

```asm
VLD1.32     {...}
VMLA.F32    Q..., Q..., Q...
VADD.F32    Q..., Q..., Q...
```

使用 `Q` 寄存器、`VLD1`/`VST1` 是 NEON 向量代码的证据。只看到使用 `S` 寄存器的
`VMUL.F32` 或 `VADD.F32` 表示标量 VFP。

同时检查已有模块的反汇编。严格隔离时，除了
`ridge_neon_dot120_f32` 及其编译器生成的局部辅助代码，不应出现 NEON 指令。

## 计时

在 RTOS 任务中先预热一次，然后仅包围求解调用读取 Cortex-A7 Generic Timer 或
PMU cycle counter。至少记录 100 个控制周期的最小值、平均值和最大值。不要把输入
生成、日志和串口输出计入求解时间。若 CPU 会动态变频，Generic Timer 更适合测量
真实时间；PMU cycle counter 适合测量核心周期。

NEON 只优化 Gram 矩阵和右端项的点积，CLAPACK Cholesky 与三角求解仍是标量代码，
所以整体加速比不会等于四倍。最终应同时比较运行时间和结果误差。

## 参考资料

- IAR C/C++ Development Guide：<https://updates.iar.com/FileStore/STANDARD/001/003/381/arm/doc/EWARM_DevelopmentGuide.ENU.pdf>
- IAR 的 NEON intrinsic 示例：<https://www.iar.com/zh/knowledge/learn/how-to-use-arm-cortex-r52-neon-in-iar-embedded-workbench>
- Arm ACLE NEON intrinsics：<https://developer.arm.com/architectures/instruction-sets/intrinsics/>
