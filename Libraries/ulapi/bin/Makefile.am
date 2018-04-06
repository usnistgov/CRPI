bin_PROGRAMS = ultest semtest inifind sockettest serialtest

ultest_SOURCES = ../src/ultest.c
ultest_LDADD = -L../lib -lulapi @PTHREAD_LIBS@ @RTAI_LIBS@
ultest_DEPENDENCIES = ../lib/libulapi.a

semtest_SOURCES = ../src/semtest.c
semtest_LDADD = -L../lib -lulapi @PTHREAD_LIBS@ @RTAI_LIBS@
semtest_DEPENDENCIES = ../lib/libulapi.a

serialtest_SOURCES = ../src/serialtest.c
serialtest_LDADD = -L../lib -lulapi @PTHREAD_LIBS@ @RTAI_LIBS@
serialtest_DEPENDENCIES = ../lib/libulapi.a

inifind_SOURCES = ../src/inifile.c ../src/inifile.h
inifind_CFLAGS = -DINIFILE_MAIN

sockettest_SOURCES = ../src/sockettest.c
sockettest_LDADD = -L../lib -lulapi @PTHREAD_LIBS@ @RTAI_LIBS@
sockettest_DEPENDENCIES = ../lib/libulapi.a

if NO_DL
else

bin_PROGRAMS += dltest dlfuncs.so

dltest_SOURCES = ../src/dltest.c
dltest_LDADD = -L../lib -ldlulapi @PTHREAD_LIBS@ @RTAI_LIBS@
dltest_DEPENDENCIES = ../lib/libdlulapi.a

dlfuncs_so_SOURCES = ../src/dlfuncs.c
dlfuncs_so_CFLAGS = @DL_CFLAGS@
dlfuncs_so_LDADD = @DL_CFLAGS@ -L../lib -ldlulapi @RTAI_LIBS@
dlfuncs_so_DEPENDENCIES = ../lib/libdlulapi.a

endif

EXTRA_DIST = CMakeLists.txt Makefile.am
