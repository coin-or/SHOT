# Try to find the Gurobi libraries. Modified from https://github.com/ampl/mp/blob/master/support/cmake/FindCPLEX.cmake

find_path(
    GUROBI_INCLUDE_DIR
    NAMES gurobi_c++.h
    HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
    PATH_SUFFIXES include
)

find_library(
    GUROBI_LIBRARY
    NAMES gurobi gurobi81 gurobi90 gurobi95
    HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
    PATH_SUFFIXES lib
)

if(MSVC)
    # determine Visual Studio year
    if(MSVC_TOOLSET_VERSION EQUAL 142)
        set(MSVC_YEAR "2019")
    elseif(MSVC_TOOLSET_VERSION EQUAL 141)
        set(MSVC_YEAR "2017")
    elseif(MSVC_TOOLSET_VERSION EQUAL 140)
        set(MSVC_YEAR "2015")
    endif()

    if(MT)
        set(M_FLAG "mt")
    else()
        set(M_FLAG "md")
    endif()

    find_library(
        GUROBI_CXX_LIBRARY
        NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib
    )
    find_library(
        GUROBI_CXX_DEBUG_LIBRARY
        NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib
    )
else()
    find_library(
        GUROBI_CXX_LIBRARY
        NAMES gurobi_c++
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib
    )
    set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
    set(GUROBI_CPP_LIBRARY ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY GUROBI_INCLUDE_DIR GUROBI_CPP_LIBRARY)
