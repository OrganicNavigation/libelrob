remake_find_library(gsl gsl_blas.h)
remake_find_library(gslcblas gsl_blas.h PACKAGE gsl)

remake_add_directories(lib)
remake_add_directories(bin COMPONENT utils)
