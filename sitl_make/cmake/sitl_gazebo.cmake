include(ExternalProject)
ExternalProject_Add(sitl_gz
  SOURCE_DIR ${CF2_SITL_ROOT_DIR}/tools/simulators/gz/crazyflie_sitl_gz
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  BINARY_DIR ${CF2_SITL_BINARY_DIR}/build_crazyflie_sitl_gz
  INSTALL_COMMAND ""
  USES_TERMINAL_CONFIGURE true
  USES_TERMINAL_BUILD true
  EXCLUDE_FROM_ALL false
  BUILD_ALWAYS 1
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -- -j ${parallel_jobs}
)