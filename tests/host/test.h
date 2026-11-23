#pragma once

/* Minimal host unit-test helpers: every failed check prints file:line and is counted;
 * a test program ends with `return test_result();`. */

#include <math.h>
#include <stdio.h>

static int s_test_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);      \
            s_test_failures++;                                                   \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                    \
    do {                                                                         \
        double check_a_ = (double)(a);                                           \
        double check_b_ = (double)(b);                                           \
        if (!(fabs(check_a_ - check_b_) <= (double)(eps))) {                     \
            printf("%s:%d: CHECK_NEAR failed: %s = %.6f, %s = %.6f\n",           \
                   __FILE__, __LINE__, #a, check_a_, #b, check_b_);              \
            s_test_failures++;                                                   \
        }                                                                        \
    } while (0)

static inline int test_result(void)
{
    return s_test_failures ? 1 : 0;
}
