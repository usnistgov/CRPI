include_directories(../src)

set(ROS_BUILD_TYPE Debug)

## The base library, 'libulapi.a'
add_library(ulapi
  ../src/inifile.c
  ../src/serial.c
  ../src/ulapi_getopt.c
  ../src/unix_rtapi.c
  ../src/unix_ulapi.c
  )

## The shared object function test file, 'libdlfuncs.so'
add_library(dlfuncs SHARED
  ../src/dlfuncs.c
  )

install(TARGETS ulapi dlfuncs DESTINATION lib)
