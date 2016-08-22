remake_define(CPC_LIB_VERSION ${REMAKE_PROJECT_MAJOR}.${REMAKE_PROJECT_MINOR}
  QUOTED CACHE STRING "The CPC library version.")
remake_define(CPC_LIB_COMPAT_MODE ON
  CACHE BOOL "Compile with CPC compatibility mode.")
remake_define(CAN_MAX_DEVICE 10
  CACHE STRING "The maximum number of CPC CAN devices.")

# The -fgnu89-inline directive is needed to correctly interpret inlined functions in ethercan.c 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fgnu89-inline")

remake_add_library(cpc PREFIX OFF)
remake_add_headers(
  cpc.h
  cpclib.h
)
remake_pkg_config_generate()
