///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:
//  Workfile:        portable.h
//  Revision:        1.0 - 21 January, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Variable configuration defintions and declarations for portability between
//  operating systems and compilers.  Also contains miscellaneous utility
//  functions.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef PORTABLE_H
#define PORTABLE_H

//! Define the library import/export 
#ifdef LIBRARY_IMPORTS
#define LIBRARY_API __declspec(dllimport)
// Changed to add GCC compatibility for linux
#elif defined(_MSC_VER)
#define LIBRARY_API __declspec(dllexport)
#elif defined(__GNUC__)
#define LIBRARY_API __attribute__((visibility("default")))
#include <cstddef>
//Linux equivelancy files to "windows.h"
#include <sys/types.h> 
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

/* FMP */
/*
  WIN32 'Sleep' is defined as:
  void Sleep(
  DWORD dwMilliseconds
  );

  We will map this to ulapi's implementation, 

  void ulapi_sleep(ulapi_real secs);
*/
#include "ulapi.h"
#define Sleep(x) ulapi_sleep(((double) (x)) * 0.001)

typedef union _LARGE_INTEGER {
  struct {
    unsigned long LowPart;
    long HighPart;
  };
  struct {
    unsigned long LowPart;
    long HighPart;
  } u;
  long long QuadPart;
} LARGE_INTEGER, *PLARGE_INTEGER;

typedef long long LONGLONG;

/* this differs from the C11 decl, which is
   errno_t strcpy_s(char *restrict dest, rsize_t destsz, const char *restrict src);
*/
inline int strcpy_s(char *dst, const char *src)
{
  strcpy(dst, src);
  return 0;
}

#endif

#ifdef WIN32
#include <windows.h>
//! JAM - This is to get rid of an annoying (but harmless) warning MSVC belches
//!       out whenever vectors are used as function parameters.
#pragma warning (disable: 4786)
#pragma warning (disable: 4996)
#pragma warning (disable: 4251)
#endif

//! @brief Enumerated CRCL return types
//enum CanonReturn {SUCCESS, FAILURE, REJECT};


//! @brief Enumerated learning algorithm types
//!
enum LearnType {NEURAL_NET, NEURAL_TISSUE, GENETIC_ALGORITHM};

//! Defines
#define GAGENEVERSION 1
#define GAPARAMCOUNT 20

#define INET_ADDR_LEN          128

#ifndef VERSION			/* FMP, defined in automake's config.h */
#define VERSION                1.0
#endif

/*
//! @brief Get the value of pi
//!
//! @return Pi to 54 decimal places
//!
inline double pi ()
{
  return 3.141592653589793238462643383279502884197169399375105820;
};
*/


inline double sgn (double a)
{
  return (a >= 0.0) ? 1.0 : -1.0;
}

#endif
