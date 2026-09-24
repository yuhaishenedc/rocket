#include "ridge.h"
#include <float.h>
#include "f2c.h"

extern int spotf2_(char *, integer *, real *, integer *, integer *);
extern int spotrs_(char *, integer *, integer *, real *, integer *,
                  real *, integer *, integer *);
extern int dpotf2_(char *, integer *, doublereal *, integer *, integer *);
extern int dpotrs_(char *, integer *, integer *, doublereal *, integer *,
                  doublereal *, integer *, integer *);

typedef char ridge_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];
typedef char ridge_double_must_be_64_bits[(sizeof(double) == 8) ? 1 : -1];

static int finite_f32(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static int finite_f64(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

int ridge_solve_f32(const float *x, const float *y, size_t rows,
                    float lambda, float *b, ridge_workspace_f32 *work)
{
    size_t r, p;
    int i, j, k;
    integer n = RIDGE_N, nrhs = 1, info = 0;
    char lower = 'L';
    float sum;
    float *a, *rhs;

    if (!x || !y || !b || !work || rows == 0 ||
        rows > ((size_t)-1) / RIDGE_N / sizeof(float) ||
        !finite_f32(lambda) || lambda <= 0.0F)
        return RIDGE_EINVAL;

    work->lapack_info = 0;
    for (p = 0; p < RIDGE_N * RIDGE_N; ++p)
        if (!finite_f32(x[p])) return RIDGE_ENUMERIC;
    for (p = 0; p < rows * RIDGE_N; ++p)
        if (!finite_f32(y[p])) return RIDGE_ENUMERIC;

    a = work->factor + RIDGE_N + 1;
    rhs = work->rhs + RIDGE_N + 1;

    /* Store the lower triangle of A=X*X^T+lambda*I in column-major layout. */
    for (j = 0; j < RIDGE_N; ++j) {
        for (i = j; i < RIDGE_N; ++i) {
            sum = 0.0F;
            for (k = 0; k < RIDGE_N; ++k)
                sum += x[i * RIDGE_N + k] * x[j * RIDGE_N + k];
            if (i == j) sum += lambda;
            if (!finite_f32(sum)) return RIDGE_ENUMERIC;
            a[i + j * RIDGE_N] = sum;
        }
    }

    /* Factor A=L*L^T once with the single-precision LAPACK routine. */
    spotf2_(&lower, &n, a, &n, &info);
    work->lapack_info = info;
    if (info > 0) return RIDGE_ENOTSPD;
    if (info < 0) return RIDGE_ELAPACK;

    for (j = 0; j < RIDGE_N; ++j)
        for (i = j; i < RIDGE_N; ++i)
            if (!finite_f32(a[i + j * RIDGE_N])) return RIDGE_ENUMERIC;

    /* Solve A*B^T=X*Y^T one output row at a time. */
    for (r = 0; r < rows; ++r) {
        for (i = 0; i < RIDGE_N; ++i) {
            sum = 0.0F;
            for (k = 0; k < RIDGE_N; ++k)
                sum += x[i * RIDGE_N + k] * y[r * RIDGE_N + k];
            if (!finite_f32(sum)) return RIDGE_ENUMERIC;
            rhs[i] = sum;
        }

        spotrs_(&lower, &n, &nrhs, a, &n, rhs, &n, &info);
        work->lapack_info = info;
        if (info != 0) return RIDGE_ELAPACK;

        for (i = 0; i < RIDGE_N; ++i) {
            if (!finite_f32(rhs[i])) return RIDGE_ENUMERIC;
            b[r * RIDGE_N + i] = rhs[i];
        }
    }

    return RIDGE_OK;
}

int ridge_solve_f64(const double *x, const double *y, size_t rows,
                    double lambda, double *b, ridge_workspace_f64 *work)
{
    size_t r, p;
    int i, j, k;
    integer n = RIDGE_N, nrhs = 1, info = 0;
    char lower = 'L';
    double sum;
    double *a, *rhs;

    if (!x || !y || !b || !work || rows == 0 ||
        rows > ((size_t)-1) / RIDGE_N / sizeof(double) ||
        !finite_f64(lambda) || lambda <= 0.0)
        return RIDGE_EINVAL;

    work->lapack_info = 0;
    for (p = 0; p < RIDGE_N * RIDGE_N; ++p)
        if (!finite_f64(x[p])) return RIDGE_ENUMERIC;
    for (p = 0; p < rows * RIDGE_N; ++p)
        if (!finite_f64(y[p])) return RIDGE_ENUMERIC;

    a = work->factor + RIDGE_N + 1;
    rhs = work->rhs + RIDGE_N + 1;

    /* Store the lower triangle of A=X*X^T+lambda*I in column-major layout. */
    for (j = 0; j < RIDGE_N; ++j) {
        for (i = j; i < RIDGE_N; ++i) {
            sum = 0.0;
            for (k = 0; k < RIDGE_N; ++k)
                sum += x[i * RIDGE_N + k] * x[j * RIDGE_N + k];
            if (i == j) sum += lambda;
            if (!finite_f64(sum)) return RIDGE_ENUMERIC;
            a[i + j * RIDGE_N] = sum;
        }
    }

    /* Factor A=L*L^T once with the double-precision LAPACK routine. */
    dpotf2_(&lower, &n, a, &n, &info);
    work->lapack_info = info;
    if (info > 0) return RIDGE_ENOTSPD;
    if (info < 0) return RIDGE_ELAPACK;

    for (j = 0; j < RIDGE_N; ++j)
        for (i = j; i < RIDGE_N; ++i)
            if (!finite_f64(a[i + j * RIDGE_N])) return RIDGE_ENUMERIC;

    /* Solve A*B^T=X*Y^T one output row at a time. */
    for (r = 0; r < rows; ++r) {
        for (i = 0; i < RIDGE_N; ++i) {
            sum = 0.0;
            for (k = 0; k < RIDGE_N; ++k)
                sum += x[i * RIDGE_N + k] * y[r * RIDGE_N + k];
            if (!finite_f64(sum)) return RIDGE_ENUMERIC;
            rhs[i] = sum;
        }

        dpotrs_(&lower, &n, &nrhs, a, &n, rhs, &n, &info);
        work->lapack_info = info;
        if (info != 0) return RIDGE_ELAPACK;

        for (i = 0; i < RIDGE_N; ++i) {
            if (!finite_f64(rhs[i])) return RIDGE_ENUMERIC;
            b[r * RIDGE_N + i] = rhs[i];
        }
    }

    return RIDGE_OK;
}
