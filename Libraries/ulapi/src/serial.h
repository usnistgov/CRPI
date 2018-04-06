/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*
  serial.h

  Serial interface declarations.
*/

#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>    /* struct termios */

/* put terminal into a raw mode, and save original settings */
extern int tty_raw(int fd, int baud, struct termios * save_termios);

/* just set the baud rate */
extern int tty_baud(int fd, int baud);

/* restore terminal's mode */
extern int tty_reset(int fd, struct termios * restore_termios);

/* flushes pending input and output */
extern int tty_flush(int fd);

#endif
