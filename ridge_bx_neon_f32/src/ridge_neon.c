#include "ridge_neon.h"
#include "ridge_neon_kernels.h"
#include <float.h>
#include "f2c.h"

extern int spotf2_(char *, integer *, real *, integer *, integer *);
extern int spotrs_(char *, integer *, integer *, real *, integer *,
                  real *, integer *, integer *);

typedef char ridge_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

static int finite_f32(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

int ridge_solve_f32_neon(const float *x, const float *y, size_t rows,
                         float lambda, float *b,
                         ridge_neon_workspace_f32 *work)
{
    size_t r, p;
    int i, j;
    integer n = RIDGE_NEON_N, nrhs = 1, info = 0;
    char lower = 'L';
    float sum;
    float *a, *rhs;

    if (!x || !y || !b || !work || rows == 0 ||
        rows > ((size_t)-1) / RIDGE_NEON_N / sizeof(float) ||
        !finite_f32(lambda) || lambda <= 0.0F)
        return RIDGE_NEON_EINVAL;

    /* This check detects accidental overlap but is not an RTOS lock. */
    if (work->busy != 0U)
        return RIDGE_NEON_EBUSY;
    work->busy = 1U;
    work->lapack_info = 0;

    for (p = 0; p < RIDGE_NEON_N * RIDGE_NEON_N; ++p) {
        if (!finite_f32(x[p])) {
            work->busy = 0U;
            return RIDGE_NEON_ENUMERIC;
        }
    }
    for (p = 0; p < rows * RIDGE_NEON_N; ++p) {
        if (!finite_f32(y[p])) {
            work->busy = 0U;
            return RIDGE_NEON_ENUMERIC;
        }
    }

    a = work->factor + RIDGE_NEON_N + 1;
    rhs = work->rhs + RIDGE_NEON_N + 1;

    /* Only this external kernel contains explicit NEON intrinsics. */
    for (j = 0; j < RIDGE_NEON_N; ++j) {
        for (i = j; i < RIDGE_NEON_N; ++i) {
            sum = ridge_neon_dot120_f32(x + i * RIDGE_NEON_N,
                                        x + j * RIDGE_NEON_N);
            if (i == j) sum += lambda;
            if (!finite_f32(sum)) {
                work->busy = 0U;
                return RIDGE_NEON_ENUMERIC;
            }
            a[i + j * RIDGE_NEON_N] = sum;
        }
    }

    spotf2_(&lower, &n, a, &n, &info);
    work->lapack_info = info;
    if (info > 0) {
        work->busy = 0U;
        return RIDGE_NEON_ENOTSPD;
    }
    if (info < 0) {
        work->busy = 0U;
        return RIDGE_NEON_ELAPACK;
    }

    for (j = 0; j < RIDGE_NEON_N; ++j) {
        for (i = j; i < RIDGE_NEON_N; ++i) {
            if (!finite_f32(a[i + j * RIDGE_NEON_N])) {
                work->busy = 0U;
                return RIDGE_NEON_ENUMERIC;
            }
        }
    }

    for (r = 0; r < rows; ++r) {
        for (i = 0; i < RIDGE_NEON_N; ++i) {
            sum = ridge_neon_dot120_f32(x + i * RIDGE_NEON_N,
                                        y + r * RIDGE_NEON_N);
            if (!finite_f32(sum)) {
                work->busy = 0U;
                return RIDGE_NEON_ENUMERIC;
            }
            rhs[i] = sum;
        }

        spotrs_(&lower, &n, &nrhs, a, &n, rhs, &n, &info);
        work->lapack_info = info;
        if (info != 0) {
            work->busy = 0U;
            return RIDGE_NEON_ELAPACK;
        }

        for (i = 0; i < RIDGE_NEON_N; ++i) {
            if (!finite_f32(rhs[i])) {
                work->busy = 0U;
                return RIDGE_NEON_ENUMERIC;
            }
            b[r * RIDGE_NEON_N + i] = rhs[i];
        }
    }

    work->busy = 0U;
    return RIDGE_NEON_OK;
}
