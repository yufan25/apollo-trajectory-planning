#include <stdbool.h>

#ifndef QDLDL_TYPES_H
# define QDLDL_TYPES_H

# ifdef __cplusplus
extern "C" {
# endif /* ifdef __cplusplus */

#include <limits.h> //for the QDLDL_INT_TYPE_MAX

// QDLDL integer and float types

// typedef @QDLDL_INT_TYPE@    QDLDL_int;   /* for indices */
// typedef @QDLDL_FLOAT_TYPE@  QDLDL_float; /* for numerical values  */
// typedef @QDLDL_BOOL_TYPE@   QDLDL_bool;  /* for boolean values  */

typedef long long int    QDLDL_int;   /* for indices */
typedef float  QDLDL_float; /* for numerical values  */
typedef bool  QDLDL_bool;  /* for boolean values  */

//Maximum value of the signed type QDLDL_int.
#define QDLDL_INT_MAX 0x7FFF

# ifdef __cplusplus
}
# endif /* ifdef __cplusplus */

#endif /* ifndef QDLDL_TYPES_H */
