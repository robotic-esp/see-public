find_path(Lemon_INCLUDE_DIR "lemon/maps.h" PATHS "/usr/include" "/usr/local/include" )
find_library(Lemon_LIBRARY "liblemon.a" NAMES "libemon.a" PATHS "/usr/lib" "/usr/local/lib")

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Lemon DEFAULT_MSG
    Lemon_INCLUDE_DIR
    Lemon_LIBRARY
)
