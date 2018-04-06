include_directories(../src)

## Declare a cpp executable
add_executable(ultest ../src/ultest.c)
add_executable(dltest ../src/dltest.c)
add_executable(semtest ../src/semtest.c)
add_executable(serialtest ../src/serialtest.c)
add_executable(sockettest ../src/sockettest.c)

target_link_libraries(ultest ulapi dl pthread)
target_link_libraries(dltest ulapi dl pthread)
target_link_libraries(semtest ulapi dl pthread)
target_link_libraries(serialtest ulapi dl pthread)
target_link_libraries(sockettest ulapi dl pthread)

install(FILES
  ../src/inifile.h
  ../src/rtapi.h
  ../src/rtapi_app.h
  ../src/ulapi.h
  DESTINATION include)
