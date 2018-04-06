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
  serial.c

  Configures serial ports modes and bauds
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "serial.h"

/* Converts a baud rate number, say 9600, into its equivalent token,
   say B9600. */
static int to_bbaud(int baud)
{
  int maxbbaud;

  if (baud <= 50) return B50;
  if (baud <= 75) return B75;
  if (baud <= 110) return B110;
  if (baud <= 134) return B134;
  if (baud <= 150) return B150;
  if (baud <= 200) return B200;
  if (baud <= 300) return B300;
  if (baud <= 600) return B600;
  if (baud <= 1200) return B1200;
  if (baud <= 1800) return B1800;
  if (baud <= 2400) return B2400;
  if (baud <= 4800) return B4800;
  if (baud <= 9600) return B9600;
  if (baud <= 19200) return B19200;
  if (baud <= 38400) return B38400;
  if (baud <= 57600) return B57600;
  maxbbaud = B57600;
#ifdef B115200
  if (baud <= 115200) return B115200;
  maxbbaud = B115200;
#endif
#ifdef B230400
  if (baud <= 230400) return B230400;
  maxbbaud = B230400;
#endif
#ifdef B460800
  if (baud <= 460800) return B460800;
  maxbbaud = B460800;
#endif
  return maxbbaud;
}

/* flushes pending input and output */
int tty_flush(int fd)
{
  return tcflush(fd, TCIOFLUSH);
}

/* put terminal into a raw mode, and save original settings */
int tty_raw(int fd, int baud, struct termios * save_termios)
{
  struct termios buf;
  int status;

  if (tcgetattr(fd, save_termios) < 0) {
    return -1;
  }

  buf = *save_termios;	/* structure copy */

  baud = to_bbaud(baud);
  cfsetispeed(&buf, baud);
  cfsetospeed(&buf, baud);

  buf.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  /* echo off, canonical mode off, extended input
     processing off, signal chars off */

  buf.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  /* no SIGINT on BREAK, CR-to-NL off, input parity
     check off, don't strip 8th bit on input,
     output flow control off */

  buf.c_cflag &= ~(CSIZE | PARENB);
  /* clear size bits, parity checking off */
  buf.c_cflag |= CS8;
  /* set 8 bits/char */

  buf.c_oflag &= ~(OPOST);
  /* output processing off */

  buf.c_cc[VMIN] = 1;	/* Case B: 1 byte at a time, no timer */
  buf.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSAFLUSH, &buf) < 0) {
    return -1;
  }

  ioctl(fd, TIOCMGET, &status);
#if 1
  /* enable DTR and RTS */
  status &= ~(TIOCM_DTR | TIOCM_RTS);
#else
  /* disable DTR and RTS */
  status |= (TIOCM_DTR | TIOCM_RTS);
#endif
  ioctl(fd, TIOCMSET, &status);

  return 0;
}

int tty_baud(int fd, int baud)
{
  struct termios buf;

  if (tcgetattr(fd, &buf) < 0) {
    return -1;
  }

  baud = to_bbaud(baud);
  cfsetispeed(&buf, baud);
  cfsetospeed(&buf, baud);

  if (tcsetattr(fd, TCSAFLUSH, &buf) < 0) {
    return -1;
  }

  return 0;
}

/* restore terminal's mode */
int tty_reset(int fd, struct termios * restore_termios)
{
  if (tcsetattr(fd, TCSAFLUSH, restore_termios) < 0) {
    return -1;
  }

  return 0;
}

#ifdef MAIN

#include <stdio.h>		/* perror */
#include <string.h>		/* strlen */
#include <stdlib.h>		/* atoi */
#include <unistd.h>		/* open, close */
#include <fcntl.h>		/* O_RDWR */
#include <pthread.h>		/* pthread_xxx */

static void * read_thread(void * arg)
{
  enum {BUFFERLEN = 80};
  char buffer[BUFFERLEN];
  int fd;
  int nchars;
  int i;

  fd = *((int *) arg);

  for (;;) {
    nchars = read(fd, buffer, BUFFERLEN - 1);
    if (nchars > 0) {
      for (i = 0; i < nchars; i++) if (buffer[i] == '\r') buffer[i] = '\n';
      buffer[nchars] = 0;
      printf("%s", buffer);
      fflush(stdout);
    }
  }

  return NULL;
}

/* syntax: serial <port> <baud> */

int main(int argc, char * argv[])
{
  struct termios save_termios;
  enum {BUFFERLEN = 80};
  char buffer[BUFFERLEN];
  char * port;
  int baud;
  int fd;
  struct sched_param sched_param;
  pthread_t thread;
  pthread_attr_t attr;

  if (argc < 3) {
    fprintf(stderr, "syntax: serial <port> <baud>\n");
    return 1;
  }

  port = argv[1];
  baud = atoi(argv[2]);

  fd = open(port, O_RDWR);
  if (fd < 0) {
    perror("open");
    return 1;
  }

  tty_raw(fd, baud, &save_termios);
  tty_flush(fd);

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  sched_param.sched_priority = 1; /* 1-31, 1 is highest */
  pthread_attr_setschedparam(&attr, &sched_param);
  pthread_create(&thread, &attr, read_thread, &fd);
  pthread_setschedparam(thread, SCHED_OTHER, &sched_param);

  while (! feof(stdin)) {
    printf("> ");
    fflush(stdout);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) break;
    write(fd, buffer, strlen(buffer));
  }

  tty_reset(fd, &save_termios);

  close(fd);

  return 0;
}

#endif
