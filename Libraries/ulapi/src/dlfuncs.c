/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*!
  \file dlfuncs.c

  \brief Test code for building a shared library that can be dynamically
  loaded.
*/

#include "ulapi.h"

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

ULAPI_DECL_SHARED int sum(int a, int b)
{
  ulapi_wait(500000000);

  return a+b;
}

ULAPI_DECL_SHARED int diff(int a, int b)
{
  ulapi_wait(500000000);

  return a-b;
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif
