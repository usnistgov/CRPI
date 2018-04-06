/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef INIFILE_H
#define INIFILE_H

/*
  inifile.h

  Decls for INI file format functions
*/

#include <stdio.h>     /* FILE */

#define INIFILE_MAX_LINELEN 1024 /* max number of chars in a line */

#define COMMENT_CHAR ';'  /* signifies a comment */

#define INI_OK      0
#define INI_DEFAULT 1
#define INI_INVALID 2

typedef struct {
  char tag[INIFILE_MAX_LINELEN];
  char rest[INIFILE_MAX_LINELEN];
} INIFILE_ENTRY;

#ifdef __cplusplus
extern "C" {
#endif

extern const char * ini_find(FILE * fp,  /* already opened file ptr */
          const char * tag,  /* string to find */
          const char * section);  /* section it's in */

extern int ini_section(FILE * fp,  /* already opened file ptr */
           const char * section,  /* section you want */
           INIFILE_ENTRY array[],  /* entries to fill */
           int max);  /* how many you can hold */

extern int ini_match(const char * match, const char * me);

#ifdef __cplusplus
}
#endif

#endif
