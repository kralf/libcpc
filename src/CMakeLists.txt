remake_define(CPC_LIB_VERSION ${REMAKE_PROJECT_MAJOR}.${REMAKE_PROJECT_MINOR}
  QUOTED CACHE STRING "The CPC library version.")
remake_define(CPC_LIB_COMPAT_MODE ON
  CACHE BOOL "Compile with CPC compatibility mode.")
remake_define(CAN_MAX_DEVICE 10
  CACHE STRING "The maximum number of CPC CAN devices.")

remake_add_library(cpc PREFIX OFF)
remake_add_headers(cpc.h)
