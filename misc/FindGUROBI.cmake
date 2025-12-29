# Try to find the Gurobi libraries. Modified from
# https://github.com/ampl/mp/blob/master/support/cmake/FindCPLEX.cmake

include(FindPackageHandleStandardArgs)

# Find the path to the main Gurobi folder. Gurobi can be installed in the
# following default locations: /opt/gurobi<version>/linux64 - Linux
# /Library/gurobi<version>/mac64 - Mac OS X C:\gurobi<version>\win64 - Windows

file(GLOB
     GUROBI_SEARCH_PATHS
     "/opt/gurobi/gurobi*/linux64"
     "/opt/gurobi*/linux64"
     "/Library/gurobi*/mac64"
     "C:\\gurobi*\\win64")
find_path(GUROBI_INSTALL_DIR
          NAMES "include/gurobi_c++.h"
          HINTS "${GUROBI_DIR}/linux64" "${GUROBI_DIR}/mac64"
                "${GUROBI_DIR}\\win64"
          PATHS ${GUROBI_SEARCH_PATHS})
message(STATUS "Found Gurobi folder: ${GUROBI_INSTALL_DIR}")

if(GUROBI_INSTALL_DIR)
  set(GUROBI_INCLUDE_DIR "${GUROBI_INSTALL_DIR}/include")
  set(GUROBI_LIB_DIR "${GUROBI_INSTALL_DIR}/lib")
  message(STATUS "Using Gurobi include folder: ${GUROBI_INCLUDE_DIR}")
  message(STATUS "Using Gurobi library folder: ${GUROBI_LIB_DIR}")

  if(UNIX)
    file(GLOB GUROBI_LIBRARY ${GUROBI_LIB_DIR}/libgurobi*.so)
    file(GLOB GUROBI_LIBRARY_DEBUG ${GUROBI_LIB_DIR}/libgurobi*.so)

    list(FILTER
         GUROBI_LIBRARY
         EXCLUDE
         REGEX
         ".*_light.so$")
    list(FILTER
         GUROBI_LIBRARY_DEBUG
         EXCLUDE
         REGEX
         ".*_light.so$")

    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_g++5.2.a)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX)
      execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                      OUTPUT_VARIABLE GCC_VERSION)
      if(GCC_VERSION VERSION_GREATER 5.2 OR GCC_VERSION VERSION_EQUAL 5.2)
        file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_g++5.2.a)
      else()
        file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_g++4.2.a)
      endif()
    endif()

    if(NOT (GUROBI_CPP_LIBRARY))
      file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_c++.a)
    endif()

    list(FILTER
         GUROBI_LIBRARY
         EXCLUDE
         REGEX
         ".*_light.so$")
    list(FILTER
         GUROBI_LIBRARY_DEBUG
         EXCLUDE
         REGEX
         ".*_light.so$")

  elseif(APPLE)
    file(GLOB GUROBI_LIBRARY ${GUROBI_LIB_DIR}/libgurobi*.so)
    file(GLOB GUROBI_LIBRARY_DEBUG ${GUROBI_LIB_DIR}/libgurobi*.so)

    if(CMAKE_COMPILER_IS_GNUCXX)
      execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                      OUTPUT_VARIABLE GCC_VERSION)
      if(GCC_VERSION VERSION_GREATER 5.2 OR GCC_VERSION VERSION_EQUAL 5.2)
        file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_g++5.2.a)
      else()
        file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_g++4.2.a)
      endif()

      if(NOT (GUROBI_CPP_LIBRARY))
        file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_c++.a)
      endif()
    endif()

  else()
    if(NOT (MSVC_VERSION LESS 1910))
      file(GLOB GUROBI_LIBRARY ${GUROBI_LIB_DIR}/gurobi_c++md2017.lib)
      file(GLOB GUROBI_LIBRARY_DEBUG ${GUROBI_LIB_DIR}/gurobi_c++mdd2017.lib)
    elseif(NOT (MSVC_VERSION LESS 1900))
      file(GLOB GUROBI_LIBRARY ${GUROBI_LIB_DIR}/gurobi_c++md2015.lib)
      file(GLOB GUROBI_LIBRARY_DEBUG ${GUROBI_LIB_DIR}/gurobi_c++mdd2015.lib)
    endif()

    file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/gurobi*.lib)
    list(FILTER
         GUROBI_CPP_LIBRARY
         EXCLUDE
         REGEX
         "gurobi_.*lib$")
  endif()

  message(STATUS "Found Gurobi library: ${GUROBI_LIBRARY}")
  message(STATUS "Found Gurobi C++ library: ${GUROBI_CPP_LIBRARY}")
endif()

# Handle the QUIETLY and REQUIRED arguments and set GUROBI_FOUND to TRUE if all
# listed variables are TRUE.
find_package_handle_standard_args(GUROBI
                                  DEFAULT_MSG
                                  GUROBI_LIBRARY
                                  GUROBI_CPP_LIBRARY)
mark_as_advanced(GUROBI_LIBRARY
                 GUROBI_CPP_LIBRARY
                 GUROBI_INCLUDE_DIR
                 GUROBI_LIB_DIR)
