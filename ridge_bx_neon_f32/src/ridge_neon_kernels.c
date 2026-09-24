#include "ridge_neon_kernels.h"
#include <arm_neon.h>

#if !defined(__ICCARM__)
#error This source file requires IAR C/C++ Compiler for Arm.
#endif

#if !defined(__ARM_NEON)
#error Enable Advanced SIMD NEON for this source file.
#endif

#if !defined(__ARM_NEON_FP)
#error Enable NEON floating-point support for this source file.
#endif

#if !defined(__ARMVFP__)
#error Enable the hardware VFP unit.
#endif

#if defined(__ARMVFP_D16__)
#error Full NEON requires D0-D31, not the D16 register configuration.
#endif

/* RIDGE_NEON_N is fixed at 120, so the vector loop has no scalar tail. */
float ridge_neon_dot120_f32(const float *a, const float *b)
{
    float32x4_t sum0;
    float32x4_t sum1;
    float32x4_t va;
    float32x4_t vb;
    float32x4_t total;
    float32x2_t pair;
    unsigned int i;

    sum0 = vdupq_n_f32(0.0F);
    sum1 = vdupq_n_f32(0.0F);
    for (i = 0U; i < 120U; i += 8U) {
        va = vld1q_f32(a + i);
        vb = vld1q_f32(b + i);
        sum0 = vmlaq_f32(sum0, va, vb);

        va = vld1q_f32(a + i + 4U);
        vb = vld1q_f32(b + i + 4U);
        sum1 = vmlaq_f32(sum1, va, vb);
    }

    total = vaddq_f32(sum0, sum1);
    pair = vadd_f32(vget_low_f32(total), vget_high_f32(total));
    pair = vpadd_f32(pair, pair);
    return vget_lane_f32(pair, 0);
}
