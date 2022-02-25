#ifndef OSQP_CONFIGURE_H
# define OSQP_CONFIGURE_H

//#define __cplusplus

# ifdef __cplusplus
extern "C" {
# endif /* ifdef __cplusplus */

/* DEBUG */
#define DEBUG

/* Operating system */
#define IS_WINDOWS

/* EMBEDDED */
#undef EMBEDDED //(@EMBEDDED@)

/* PRINTING */
#define PRINTING

/* PROFILING */
#define PROFILING

/* CTRLC */
#define CTRLC

/* DFLOAT */
#define DFLOAT

/* DLONG */
#define DLONG

/* ENABLE_MKL_PARDISO */
#define ENABLE_MKL_PARDISO

/* MEMORY MANAGEMENT */
#undef OSQP_CUSTOM_MEMORY
#ifdef OSQP_CUSTOM_MEMORY
#include "@OSQP_CUSTOM_MEMORY@"
#endif



# ifdef __cplusplus
}
# endif /* ifdef __cplusplus */

#endif /* ifndef OSQP_CONFIGURE_H */
