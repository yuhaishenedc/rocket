#include "ridge.h"
#include <float.h>
#include "f2c.h"

extern int spptrf_(char *, integer *, real *, integer *);
extern int spptrs_(char *, integer *, integer *, real *, real *, integer *,
                  integer *);
typedef char ridge_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

static int finite_f32(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

size_t ridge_packed_lower_index(size_t row, size_t col)
{
    size_t swap;

    if (row < col) {
        swap = row;
        row = col;
        col = swap;
    }

    return col * (2 * RIDGE_N - col + 1) / 2 + row - col;
}

int ridge_solve_f32(const float *x, const float *y, size_t rows,
                    float lambda, float *b, ridge_workspace_f32 *work)
{
    size_t r, p;
    int i, j, k;
    integer n = RIDGE_N, nrhs = 1, ldb = RIDGE_N, info = 0;
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

    a = work->factor + 1;
    rhs = work->rhs + RIDGE_N + 1;

    /* Store the lower triangle of A=X*X^T+lambda*I in packed layout. */
    for (j = 0; j < RIDGE_N; ++j) {
        for (i = j; i < RIDGE_N; ++i) {
            sum = 0.0F;
            for (k = 0; k < RIDGE_N; ++k)
                sum += x[i * RIDGE_N + k] * x[j * RIDGE_N + k];
            if (i == j) sum += lambda;
            if (!finite_f32(sum)) return RIDGE_ENUMERIC;
            a[ridge_packed_lower_index((size_t)i, (size_t)j)] = sum;
        }
    }

    /* Factor A=L*L^T once with the packed single-precision LAPACK routine. */
    spptrf_(&lower, &n, a, &info);
    work->lapack_info = info;
    if (info > 0) return RIDGE_ENOTSPD;
    if (info < 0) return RIDGE_ELAPACK;

    for (p = 0; p < RIDGE_PACKED_SIZE; ++p)
        if (!finite_f32(a[p])) return RIDGE_ENUMERIC;

    /* Solve A*B^T=X*Y^T one output row at a time. */
    for (r = 0; r < rows; ++r) {
        for (i = 0; i < RIDGE_N; ++i) {
            sum = 0.0F;
            for (k = 0; k < RIDGE_N; ++k)
                sum += x[i * RIDGE_N + k] * y[r * RIDGE_N + k];
            if (!finite_f32(sum)) return RIDGE_ENUMERIC;
            rhs[i] = sum;
        }

        spptrs_(&lower, &n, &nrhs, a, rhs, &ldb, &info);
        work->lapack_info = info;
        if (info != 0) return RIDGE_ELAPACK;

        for (i = 0; i < RIDGE_N; ++i) {
            if (!finite_f32(rhs[i])) return RIDGE_ENUMERIC;
            b[r * RIDGE_N + i] = rhs[i];
        }
    }

    return RIDGE_OK;
}

float *ridge_normal_packed_matrix_f32(ridge_normal_workspace_f32 *work)
{
    if (!work) return (float *)0;
    return work->factor + 1;
}

int ridge_solve_normal_f32(const float *xty, float lambda,
                           float *coefficients,
                           ridge_normal_workspace_f32 *work)
{
    size_t p;
    int i, j;
    integer n = RIDGE_N;
    integer nrhs = RIDGE_NORMAL_RHS;
    integer info = 0;
    char lower = 'L';
    float value;
    float *factor;
    float *rhs;

    if (!xty || !coefficients || !work ||
        !finite_f32(lambda) || lambda <= 0.0F)
        return RIDGE_EINVAL;

    work->lapack_info = 0;
    factor = work->factor + 1;
    rhs = work->rhs + RIDGE_N + 1;

    /* Validate the packed matrix before changing its diagonal. */
    for (p = 0; p < RIDGE_PACKED_SIZE; ++p)
        if (!finite_f32(factor[p])) return RIDGE_ENUMERIC;

    /* Add ridge regularization directly to the packed diagonal. */
    for (i = 0; i < RIDGE_N; ++i) {
        p = ridge_packed_lower_index((size_t)i, (size_t)i);
        value = factor[p] + lambda;
        if (!finite_f32(value)) return RIDGE_ENUMERIC;
        factor[p] = value;
    }

    /* Pack all six right-hand sides in LAPACK column-major layout. */
    for (j = 0; j < RIDGE_NORMAL_RHS; ++j) {
        for (i = 0; i < RIDGE_N; ++i) {
            value = xty[i * RIDGE_NORMAL_RHS + j];
            if (!finite_f32(value)) return RIDGE_ENUMERIC;
            rhs[i + j * RIDGE_N] = value;
        }
    }

    spptrf_(&lower, &n, factor, &info);
    work->lapack_info = info;
    if (info > 0) return RIDGE_ENOTSPD;
    if (info < 0) return RIDGE_ELAPACK;

    for (p = 0; p < RIDGE_PACKED_SIZE; ++p)
        if (!finite_f32(factor[p])) return RIDGE_ENUMERIC;

    spptrs_(&lower, &n, &nrhs, factor, rhs, &n, &info);
    work->lapack_info = info;
    if (info != 0) return RIDGE_ELAPACK;

    /* Validate every result before publishing a new coefficient matrix. */
    for (j = 0; j < RIDGE_NORMAL_RHS; ++j)
        for (i = 0; i < RIDGE_N; ++i)
            if (!finite_f32(rhs[i + j * RIDGE_N]))
                return RIDGE_ENUMERIC;

    for (i = 0; i < RIDGE_N; ++i)
        for (j = 0; j < RIDGE_NORMAL_RHS; ++j)
            coefficients[i * RIDGE_NORMAL_RHS + j] =
                rhs[i + j * RIDGE_N];

    return RIDGE_OK;
}
