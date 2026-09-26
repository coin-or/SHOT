# Try to find the Uno NLP solver (https://github.com/cvanaret/Uno).
#
# Uno does not install a CMake package configuration file: its CMakeLists.txt
# declares "EXPORT UnoTargets" but never calls install(EXPORT ...), so
# find_package(Uno CONFIG) cannot work. Neither does it install a pkg-config
# file, so this module locates the library and the installed C API header by
# hand, in the same spirit as misc/FindCBC.cmake and misc/FindGUROBI.cmake.
#
# Uno installs
#   ${prefix}/lib/libuno.{a,so,dylib}
#   ${prefix}/include/uno/Uno_C_API.h
#   ${prefix}/include/uno/uno_int.h
#
# Once done, this will define
#
#  UNO_FOUND       - system has Uno
#  UNO_INCLUDE_DIR - the directory containing Uno_C_API.h
#  UNO_LIBRARY     - the Uno library to link against
#  UNO_LIBRARIES   - Uno plus the dependencies a static Uno needs

include(FindPackageHandleStandardArgs)

# If UNO_DIR is explicitly set, prioritize it
if(UNO_DIR)
    find_path(UNO_INCLUDE_DIR
              NAMES "Uno_C_API.h"
              HINTS "${UNO_DIR}/include/uno" "${UNO_DIR}/include"
              NO_DEFAULT_PATH)

    find_library(UNO_LIBRARY
                 NAMES uno
                 HINTS "${UNO_DIR}/lib" "${UNO_DIR}/lib64"
                 NO_DEFAULT_PATH)
endif()

# If not found via UNO_DIR, search the standard locations
if(NOT UNO_INCLUDE_DIR)
    find_path(UNO_INCLUDE_DIR
              NAMES "Uno_C_API.h"
              PATH_SUFFIXES "uno")
endif()

if(NOT UNO_LIBRARY)
    find_library(UNO_LIBRARY NAMES uno)
endif()

set(UNO_LIBRARIES ${UNO_LIBRARY})

# A static Uno does not carry its dependencies, so they have to be repeated on
# the link line. Uno itself requires BLAS and LAPACK unconditionally, and is
# partly written in Fortran, so the Fortran runtime is needed as well. For a
# shared Uno these are already recorded in the library and adding them is
# harmless.
if(UNO_LIBRARY AND UNO_LIBRARY MATCHES "\\.a$")
    find_package(BLAS)
    find_package(LAPACK)

    if(BLAS_FOUND)
        list(APPEND UNO_LIBRARIES ${BLAS_LIBRARIES})
    endif()

    if(LAPACK_FOUND)
        list(APPEND UNO_LIBRARIES ${LAPACK_LIBRARIES})
    endif()

    # gfortran is needed for a static Uno built with the GNU Fortran compiler.
    find_library(UNO_GFORTRAN_LIBRARY NAMES gfortran)

    if(UNO_GFORTRAN_LIBRARY)
        list(APPEND UNO_LIBRARIES ${UNO_GFORTRAN_LIBRARY})
    else()
        message(STATUS "Uno: libgfortran not found; a static Uno may fail to link. "
                       "Consider building Uno with -DBUILD_SHARED_LIBS=ON.")
    endif()

    mark_as_advanced(UNO_GFORTRAN_LIBRARY)
endif()

# Handle the QUIETLY and REQUIRED arguments and set UNO_FOUND to TRUE if all
# listed variables are TRUE.
find_package_handle_standard_args(Uno
                                  DEFAULT_MSG
                                  UNO_LIBRARY
                                  UNO_INCLUDE_DIR)

mark_as_advanced(UNO_LIBRARY UNO_INCLUDE_DIR)
