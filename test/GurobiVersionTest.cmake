# Version discovery must not require a compiler, a real SDK library, or a solver license.
set(CMAKE_MODULE_PATH "${SHOT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_COMPILER_ID Clang)
set(MSVC_VERSION 1930)

function(check_sdk label version library_version expected)
    set(GUROBI_DIR "${SHOT_TEST_DIR}/${label}")
    file(MAKE_DIRECTORY "${GUROBI_DIR}/include" "${GUROBI_DIR}/lib")
    file(WRITE "${GUROBI_DIR}/include/gurobi_c++.h" "")

    if(version)
        file(
            WRITE "${GUROBI_DIR}/include/gurobi_c.h"
            "#define GRB_VERSION_MAJOR ${version}\n#define GRB_VERSION_MINOR 0\n#define GRB_VERSION_TECHNICAL 3\n"
        )
    else()
        file(WRITE "${GUROBI_DIR}/include/gurobi_c.h" "")
    endif()
    foreach(
        library "libgurobi${library_version}0.dylib" "libgurobi${library_version}0.so" "gurobi${library_version}0.lib"
                libgurobi_c++.a "gurobi_c++md2017.lib" "gurobi_c++mdd2017.lib"
    )
        file(WRITE "${GUROBI_DIR}/lib/${library}" "")
    endforeach()

    if(label STREQUAL "mixed")
        foreach(library libgurobi90.dylib libgurobi90.so gurobi90.lib)
            file(WRITE "${GUROBI_DIR}/lib/${library}" "")
        endforeach()
    endif()
    unset(GUROBI_INSTALL_DIR CACHE)
    find_package(GUROBI 10.0 QUIET)

    if(expected AND NOT GUROBI_FOUND)
        message(FATAL_ERROR "${label}: supported SDK was rejected")
    elseif(NOT expected AND GUROBI_FOUND)
        message(FATAL_ERROR "${label}: unsupported SDK was accepted")
    endif()

    if(expected)
        foreach(library IN LISTS GUROBI_LIBRARY GUROBI_LIBRARY_DEBUG GUROBI_CPP_LIBRARY)
            if(library MATCHES "gurobi90\\.(dylib|so|lib)$")
                message(FATAL_ERROR "${label}: an incompatible core library was retained")
            endif()
        endforeach()
    endif()
endfunction()

check_sdk(old 9 9 FALSE)
check_sdk(supported 10 10 TRUE)
check_sdk(newer 13 13 TRUE)
check_sdk(unknown "" 10 FALSE)
check_sdk(mismatch 10 9 FALSE)
check_sdk(mixed 10 10 TRUE)
