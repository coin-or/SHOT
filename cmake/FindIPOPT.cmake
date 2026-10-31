include(FindPackageHandleStandardArgs)

# Keep pkg-config's transitive dependency information on Unix. Official MSVC
# releases ship an import library and DLLs and do not require pkg-config.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND AND NOT MSVC)
    set(_IPOPT_ORIGINAL_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
    list(PREPEND CMAKE_PREFIX_PATH "${IPOPT_DIR}")
    pkg_search_module(IPOPT QUIET IMPORTED_TARGET ipopt)
    set(CMAKE_PREFIX_PATH "${_IPOPT_ORIGINAL_PREFIX_PATH}")
    unset(_IPOPT_ORIGINAL_PREFIX_PATH)
endif()

if(NOT IPOPT_FOUND)
    find_path(IPOPT_INCLUDE_DIR NAMES IpIpoptApplication.hpp
        HINTS "${IPOPT_DIR}" ENV IPOPT_DIR
        PATH_SUFFIXES include/coin-or include/coin include)
    find_library(IPOPT_LIBRARY NAMES ipopt ipopt.dll libipopt
        HINTS "${IPOPT_DIR}" ENV IPOPT_DIR PATH_SUFFIXES lib lib64)
    find_package_handle_standard_args(IPOPT REQUIRED_VARS IPOPT_INCLUDE_DIR IPOPT_LIBRARY)
    set(IPOPT_INCLUDE_DIRS "${IPOPT_INCLUDE_DIR}")
    set(IPOPT_LIBRARIES "${IPOPT_LIBRARY}")
    if(IPOPT_LIBRARY)
        get_filename_component(IPOPT_LIBRARY_DIRS "${IPOPT_LIBRARY}" DIRECTORY)
    endif()
    mark_as_advanced(IPOPT_INCLUDE_DIR IPOPT_LIBRARY)
endif()

if(IPOPT_FOUND AND NOT TARGET IPOPT::IPOPT)
    add_library(IPOPT::IPOPT INTERFACE IMPORTED)
    if(TARGET PkgConfig::IPOPT)
        target_link_libraries(IPOPT::IPOPT INTERFACE PkgConfig::IPOPT)
    else()
        set_target_properties(IPOPT::IPOPT PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${IPOPT_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${IPOPT_LIBRARIES}")
    endif()
endif()
