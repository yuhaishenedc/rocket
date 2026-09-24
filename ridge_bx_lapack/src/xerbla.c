#include "f2c.h"

/* This embedded replacement avoids Fortran I/O and STOP dependencies. The
 * ridge wrapper checks LAPACK INFO after every call. Internal BLAS arguments
 * are fixed by the wrapper and the selected LAPACK routines.
 */
int xerbla_(char *name, integer *argument)
{
    (void)name;
    (void)argument;
    return 0;
}
