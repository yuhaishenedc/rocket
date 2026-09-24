#include "ridge.h"
#include <math.h>

#define OUTPUT_ROWS 3

/*
 * 单精度和双精度分别使用独立的输入、输出与工作区。所有大数组均采用静态存储，
 * 避免占用任务栈。实际项目可只保留需要的精度版本，以减少 RAM 和代码空间。
 */
static float x_f32[RIDGE_N * RIDGE_N];
static float y_f32[OUTPUT_ROWS * RIDGE_N];
static float b_f32[OUTPUT_ROWS * RIDGE_N];
static ridge_workspace_f32 workspace_f32;

static double x_f64[RIDGE_N * RIDGE_N];
static double y_f64[OUTPUT_ROWS * RIDGE_N];
static double b_f64[OUTPUT_ROWS * RIDGE_N];
static ridge_workspace_f64 workspace_f64;

/*
 * 可在 IAR Watch 窗口观察以下变量。两个状态值都应为 0；单精度误差应小于
 * 1e-4，双精度误差应小于 1e-11。本示例不依赖串口和 printf。
 */
volatile int demo_status_f32 = -99;
volatile int demo_status_f64 = -99;
volatile float demo_max_error_f32;
volatile double demo_max_error_f64;

int main(void)
{
    int i, j, r;
    float expected_f32, error_f32;
    double expected_f64, error_f64;

    /*
     * 同时构造两套相同的稠密测试矩阵：
     *
     *     X = I + 0.01*1*1^T
     *
     * 对角元素为 1.01，其余元素为 0.01。
     */
    for (i = 0; i < RIDGE_N; ++i) {
        for (j = 0; j < RIDGE_N; ++j) {
            x_f32[i * RIDGE_N + j] =
                (i == j ? 1.0F : 0.0F) + 0.01F;
            x_f64[i * RIDGE_N + j] =
                (i == j ? 1.0 : 0.0) + 0.01;
        }
    }

    /*
     * 第 r 行 Y=(r+1)*2.2*1^T。当 lambda=0.1 时，解析解为：
     *
     *     B = (r+1)*4.84/4.94*1^T
     */
    for (r = 0; r < OUTPUT_ROWS; ++r) {
        for (j = 0; j < RIDGE_N; ++j) {
            y_f32[r * RIDGE_N + j] = (float)(r + 1) * 2.2F;
            y_f64[r * RIDGE_N + j] = (double)(r + 1) * 2.2;
        }
    }

    /* 调用单精度接口。返回值非零时，不能使用 b_f32。 */
    demo_status_f32 = ridge_solve_f32(x_f32, y_f32, OUTPUT_ROWS,
                                      0.1F, b_f32, &workspace_f32);

    /* 调用双精度接口。返回值非零时，不能使用 b_f64。 */
    demo_status_f64 = ridge_solve_f64(x_f64, y_f64, OUTPUT_ROWS,
                                      0.1, b_f64, &workspace_f64);

    if (demo_status_f32 != RIDGE_OK || demo_status_f64 != RIDGE_OK)
        return 1;

    /* 分别计算单精度和双精度结果相对于解析解的最大绝对误差。 */
    demo_max_error_f32 = 0.0F;
    demo_max_error_f64 = 0.0;
    for (r = 0; r < OUTPUT_ROWS; ++r) {
        expected_f32 = (float)(r + 1) * 4.84F / 4.94F;
        expected_f64 = (double)(r + 1) * 4.84 / 4.94;
        for (j = 0; j < RIDGE_N; ++j) {
            error_f32 = b_f32[r * RIDGE_N + j] - expected_f32;
            if (error_f32 < 0.0F) error_f32 = -error_f32;
            error_f64 = fabs(b_f64[r * RIDGE_N + j] - expected_f64);
            if (error_f32 > demo_max_error_f32)
                demo_max_error_f32 = error_f32;
            if (error_f64 > demo_max_error_f64)
                demo_max_error_f64 = error_f64;
        }
    }

    /* 将误差超限转换成便于调试器观察的状态码。 */
    if (demo_max_error_f32 > 1e-4F) demo_status_f32 = -100;
    if (demo_max_error_f64 > 1e-11) demo_status_f64 = -100;

    if (demo_status_f32 != RIDGE_OK || demo_status_f64 != RIDGE_OK)
        return 1;
    return 0;
}
