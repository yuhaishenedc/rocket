#include "ridge.h"

#define OUTPUT_ROWS 3

/*--------------------测试数据和工作区使用静态存储，避免占用任务栈--------------------*/
static float x[RIDGE_N * RIDGE_N];
static float y[OUTPUT_ROWS * RIDGE_N];
static float b[OUTPUT_ROWS * RIDGE_N];
static ridge_workspace_f32 workspace;

/*--------------------可在IAR Watch窗口观察状态码和最大误差--------------------*/
volatile int demo_status = -99;
volatile float demo_max_error;

int main(void)
{
    int i, j, r;
    float expected;
    float error;

    /*--------------------构造X=I+0.01*1*1^T--------------------*/
    for (i = 0; i < RIDGE_N; ++i) {
        for (j = 0; j < RIDGE_N; ++j) {
            x[i * RIDGE_N + j] =
                (i == j ? 1.0F : 0.0F) + 0.01F;
        }
    }

    /*--------------------构造具有已知解析解的三行Y--------------------*/
    for (r = 0; r < OUTPUT_ROWS; ++r) {
        for (j = 0; j < RIDGE_N; ++j) {
            y[r * RIDGE_N + j] = (float)(r + 1) * 2.2F;
        }
    }

    demo_status = ridge_solve_f32(x, y, OUTPUT_ROWS,
                                  0.1F, b, &workspace);
    if (demo_status != RIDGE_OK)
        return 1;

    /*--------------------检查单精度求解结果相对解析解的最大绝对误差--------------------*/
    demo_max_error = 0.0F;
    for (r = 0; r < OUTPUT_ROWS; ++r) {
        expected = (float)(r + 1) * 4.84F / 4.94F;
        for (j = 0; j < RIDGE_N; ++j) {
            error = b[r * RIDGE_N + j] - expected;
            if (error < 0.0F) error = -error;
            if (error > demo_max_error)
                demo_max_error = error;
        }
    }

    if (demo_max_error > 1e-4F) {
        demo_status = -100;
        return 1;
    }
    return 0;
}
