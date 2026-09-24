#include "ridge_neon.h"

#define OUTPUT_ROWS 3

/*
 * 大数组使用静态存储，避免占用 RTOS 任务栈。实际应用中应由一个固定任务独占
 * 该工作区，并按照 README 配置该任务的 VFP/NEON 上下文。
 */
static float x_data[RIDGE_NEON_N * RIDGE_NEON_N];
static float y_data[OUTPUT_ROWS * RIDGE_NEON_N];
static float b_data[OUTPUT_ROWS * RIDGE_NEON_N];
static ridge_neon_workspace_f32 workspace;

/* 可在 IAR Watch 窗口观察这些变量，不依赖 printf 或串口。 */
volatile int demo_status = -99;
volatile float demo_max_error;

int main(void)
{
    int i, j, r;
    float expected;
    float error;

    /* 构造 X=I+0.01*1*1^T。 */
    for (i = 0; i < RIDGE_NEON_N; ++i) {
        for (j = 0; j < RIDGE_NEON_N; ++j) {
            x_data[i * RIDGE_NEON_N + j] =
                (i == j ? 1.0F : 0.0F) + 0.01F;
        }
    }

    /* 构造第 r 行 Y=(r+1)*2.2*1^T。 */
    for (r = 0; r < OUTPUT_ROWS; ++r) {
        for (j = 0; j < RIDGE_NEON_N; ++j)
            y_data[r * RIDGE_NEON_N + j] = (float)(r + 1) * 2.2F;
    }

    /*
     * 在真实 RTOS 中，在此调用前后使用 README 所述的任务浮点上下文接口。
     * 如果 RTOS 不保存 D0-D31，则必须保证调用期间不会切换任务，且中断不使用
     * VFP/NEON。这里只是无 RTOS 的最小示例。
     */
    demo_status = ridge_solve_f32_neon(x_data, y_data, OUTPUT_ROWS,
                                       0.1F, b_data, &workspace);
    if (demo_status != RIDGE_NEON_OK)
        return 1;

    /* 解析解为 B=(r+1)*4.84/4.94*1^T。 */
    demo_max_error = 0.0F;
    for (r = 0; r < OUTPUT_ROWS; ++r) {
        expected = (float)(r + 1) * 4.84F / 4.94F;
        for (j = 0; j < RIDGE_NEON_N; ++j) {
            error = b_data[r * RIDGE_NEON_N + j] - expected;
            if (error < 0.0F) error = -error;
            if (error > demo_max_error) demo_max_error = error;
        }
    }

    if (demo_max_error > 2e-4F) {
        demo_status = -100;
        return 1;
    }
    return 0;
}
