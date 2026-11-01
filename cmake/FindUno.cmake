# Try to find the Uno NLP solver (https://github.com/cvanaret/Uno).
#
# Uno does not install a CMake package configuration file: its CMakeLists.txt
# declares "EXPORT UnoTargets" but never calls install(EXPORT ...), so
# find_package(Uno CONFIG) cannot work. Neither does it install a pkg-config
# file, so this module locates the library and the installed C API header by
# hand, in the same spirit as cmake/FindCBC.cmake and cmake/FindGUROBI.cmake.
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
                 NAMES uno libuno uno.dll libuno.dll
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
    find_library(UNO_LIBRARY NAMES uno libuno uno.dll libuno.dll)
endif()

# Uno's official Windows bundle is built with MinGW, but its C API can be
# called from MSVC. Generate a native import library from the DLL's C exports;
# never link the bundled MinGW static library into an MSVC target.
if(MSVC AND (NOT UNO_LIBRARY OR UNO_LIBRARY MATCHES "\\.a$"))
    set(UNO_LIBRARY UNO_LIBRARY-NOTFOUND)
    find_file(UNO_RUNTIME_DLL NAMES libuno.dll uno.dll
        HINTS "${UNO_DIR}/bin" "${UNO_DIR}/lib" "${UNO_DIR}")
    if(UNO_RUNTIME_DLL)
        get_filename_component(_uno_linker_dir "${CMAKE_LINKER}" DIRECTORY)
        find_program(UNO_DUMPBIN NAMES dumpbin HINTS "${_uno_linker_dir}")
        if(NOT UNO_DUMPBIN)
            message(FATAL_ERROR "Uno: dumpbin is required to generate an MSVC import library.")
        endif()
        execute_process(COMMAND "${UNO_DUMPBIN}" /exports "${UNO_RUNTIME_DLL}"
            OUTPUT_VARIABLE _uno_exports RESULT_VARIABLE _uno_dump_result)
        string(REGEX MATCHALL "[ \t]+[0-9]+[ \t]+[0-9A-Fa-f]+[ \t]+[0-9A-Fa-f]+[ \t]+uno_[A-Za-z0-9_]+"
            _uno_export_lines "${_uno_exports}")
        if(NOT _uno_dump_result EQUAL 0 OR NOT _uno_export_lines)
            message(FATAL_ERROR "Uno: could not read the DLL's C API exports.")
        endif()
        get_filename_component(_uno_dll_name "${UNO_RUNTIME_DLL}" NAME)
        set(_uno_def "LIBRARY ${_uno_dll_name}\nEXPORTS\n")
        foreach(_uno_export IN LISTS _uno_export_lines)
            string(REGEX MATCH "uno_[A-Za-z0-9_]+$" _uno_symbol "${_uno_export}")
            string(APPEND _uno_def "${_uno_symbol}\n")
        endforeach()
        string(SHA256 _uno_dll_id "${UNO_RUNTIME_DLL}")
        set(_uno_import_dir "${CMAKE_CURRENT_BINARY_DIR}/uno-import/${_uno_dll_id}")
        file(MAKE_DIRECTORY "${_uno_import_dir}")
        file(CONFIGURE OUTPUT "${_uno_import_dir}/uno.def" CONTENT "${_uno_def}" @ONLY)
        set(UNO_LIBRARY "${_uno_import_dir}/uno.lib")
        if(CMAKE_CXX_COMPILER_ARCHITECTURE_ID STREQUAL "ARM64")
            set(_uno_machine ARM64)
        elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(_uno_machine X64)
        else()
            set(_uno_machine X86)
        endif()
        if(NOT EXISTS "${UNO_LIBRARY}" OR "${UNO_RUNTIME_DLL}" IS_NEWER_THAN "${UNO_LIBRARY}"
                OR "${_uno_import_dir}/uno.def" IS_NEWER_THAN "${UNO_LIBRARY}")
            execute_process(COMMAND "${CMAKE_AR}" "/def:${_uno_import_dir}/uno.def"
                "/out:${UNO_LIBRARY}" "/machine:${_uno_machine}"
                RESULT_VARIABLE _uno_import_result OUTPUT_QUIET)
            if(NOT _uno_import_result EQUAL 0)
                message(FATAL_ERROR "Uno: could not generate the MSVC import library.")
            endif()
        endif()
        mark_as_advanced(UNO_RUNTIME_DLL UNO_DUMPBIN)
    endif()
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
