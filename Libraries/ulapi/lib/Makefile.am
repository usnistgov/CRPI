lib_LIBRARIES = libulapi.a

SOURCES = ../src/unix_ulapi.c ../src/ulapi.h ../src/inifile.c ../src/inifile.h ../src/ulapi_getopt.c ../src/ulapi_getopt.h ../src/unix_rtapi.c ../src/rtapi.h ../src/serial.c ../src/serial.h
libulapi_a_SOURCES = $(SOURCES)

if NO_DL
else

lib_LIBRARIES += libdlulapi.a
libdlulapi_a_SOURCES = $(SOURCES)
libdlulapi_a_CFLAGS = @DL_CFLAGS@

endif
