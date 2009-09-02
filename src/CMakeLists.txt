add_definitions(
  -DCPC_LIB_VERSION="${REMAKE_PROJECT_MAJOR}.${REMAKE_PROJECT_MINOR}"
)
remake_add_library(cpc PREFIX OFF)
remake_add_headers(cpc.h)
