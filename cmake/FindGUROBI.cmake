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
     "/Library/gurobi*/macos_universal2"
     "/Library/gurobi*/macos_arm64"
     "/usr/local"
     "C:\\gurobi*\\win64")

# If GUROBI_DIR is explicitly set, prioritize it
if(GUROBI_DIR)
    find_path(GUROBI_INSTALL_DIR
              NAMES "include/gurobi_c++.h"
              HINTS "${GUROBI_DIR}"
              NO_DEFAULT_PATH)
endif()

# If not found with GUROBI_DIR, search in standard locations
if(NOT GUROBI_INSTALL_DIR)
    find_path(GUROBI_INSTALL_DIR
              NAMES "include/gurobi_c++.h"
              HINTS "${GUROBI_DIR}/linux64" "${GUROBI_DIR}/mac64"
                    "${GUROBI_DIR}/macos_universal2" "${GUROBI_DIR}/macos_arm64"
                    "${GUROBI_DIR}\\win64"
              PATHS ${GUROBI_SEARCH_PATHS})
endif()
message(STATUS "Found Gurobi folder: ${GUROBI_INSTALL_DIR}")

if(GUROBI_INSTALL_DIR)
  set(GUROBI_INCLUDE_DIR "${GUROBI_INSTALL_DIR}/include")
  set(GUROBI_LIB_DIR "${GUROBI_INSTALL_DIR}/lib")

  # Read the SDK version before accepting its libraries, including when no compiler or license is available.
  unset(GUROBI_VERSION)

  if(EXISTS "${GUROBI_INCLUDE_DIR}/gurobi_c.h")
    file(STRINGS "${GUROBI_INCLUDE_DIR}/gurobi_c.h" GUROBI_VERSION_DEFINES
         REGEX "^#[ \t]*define[ \t]+GRB_VERSION_(MAJOR|MINOR|TECHNICAL)[ \t]+[0-9]+")
    foreach(component MAJOR MINOR TECHNICAL)
      unset(GUROBI_VERSION_${component})
      foreach(line IN LISTS GUROBI_VERSION_DEFINES)
        if(line MATCHES "GRB_VERSION_${component}[ \t]+([0-9]+)")
          set(GUROBI_VERSION_${component} "${CMAKE_MATCH_1}")
        endif()
      endforeach()
    endforeach()

    if(DEFINED GUROBI_VERSION_MAJOR AND DEFINED GUROBI_VERSION_MINOR AND DEFINED GUROBI_VERSION_TECHNICAL)
      set(GUROBI_VERSION "${GUROBI_VERSION_MAJOR}.${GUROBI_VERSION_MINOR}.${GUROBI_VERSION_TECHNICAL}")
    endif()
  endif()

  message(STATUS "Using Gurobi include folder: ${GUROBI_INCLUDE_DIR}")
  message(STATUS "Using Gurobi library folder: ${GUROBI_LIB_DIR}")

  if(APPLE)
    file(GLOB GUROBI_LIBRARY ${GUROBI_LIB_DIR}/libgurobi*.dylib)
    file(GLOB GUROBI_LIBRARY_DEBUG ${GUROBI_LIB_DIR}/libgurobi*.dylib)

    # Filter out light and JNI libraries
    list(FILTER
         GUROBI_LIBRARY
         EXCLUDE
         REGEX
         ".*_light.dylib$")
    list(FILTER
         GUROBI_LIBRARY
         EXCLUDE
         REGEX
         ".*Jni.*dylib$")
    list(FILTER
         GUROBI_LIBRARY_DEBUG
         EXCLUDE
         REGEX
         ".*_light.dylib$")
    list(FILTER
         GUROBI_LIBRARY_DEBUG
         EXCLUDE
         REGEX
         ".*Jni.*dylib$")

    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      file(GLOB GUROBI_CPP_LIBRARY ${GUROBI_LIB_DIR}/libgurobi_c++.a)
    endif()

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

  elseif(UNIX)
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

# Match the core library to the header version; a stale library from another SDK is not compatible.
set(GUROBI_VERSION_LIBRARY_MATCH FALSE)

if(GUROBI_VERSION)
  foreach(library_list GUROBI_LIBRARY GUROBI_LIBRARY_DEBUG GUROBI_CPP_LIBRARY)
    foreach(library IN LISTS ${library_list})
      get_filename_component(library_name "${library}" NAME)

      if(library_name MATCHES "^(lib)?gurobi[0-9]+\\.(dylib|so|lib)$")
        if(library_name MATCHES "^(lib)?gurobi${GUROBI_VERSION_MAJOR}${GUROBI_VERSION_MINOR}\\.(dylib|so|lib)$")
          set(GUROBI_VERSION_LIBRARY_MATCH TRUE)
        else()
          list(REMOVE_ITEM ${library_list} "${library}")
        endif()
      endif()
    endforeach()
  endforeach()
endif()
find_package_handle_standard_args(GUROBI
    REQUIRED_VARS GUROBI_INCLUDE_DIR GUROBI_LIBRARY GUROBI_CPP_LIBRARY GUROBI_VERSION GUROBI_VERSION_LIBRARY_MATCH
    VERSION_VAR GUROBI_VERSION)
mark_as_advanced(GUROBI_LIBRARY
                 GUROBI_CPP_LIBRARY
                 GUROBI_INCLUDE_DIR
                 GUROBI_LIB_DIR)
