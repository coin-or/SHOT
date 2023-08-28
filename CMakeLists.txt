cmake_minimum_required(VERSION 3.9)
include(ExternalProject)

# For ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()


project(SHOT VERSION 1.1.0
                  DESCRIPTION "The Supporting Hyperplane Optimization Toolkit (SHOT) for Mixed-Integer Nonlinear Optimization"
                  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Location of extra CMake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/misc")

# Find std::filesystem or std::experimental::filesystem
find_package(Filesystem REQUIRED)

if(HAVE_STD_FILESYSTEM)
    add_definitions(-DHAS_STD_FILESYSTEM)
elseif(HAVE_STD_EXPERIMENTAL_FILESYSTEM)
    add_definitions(-DHAS_STD_EXPERIMENTAL_FILESYSTEM)
else()
    message(SEND_ERROR "SHOT needs support for C++17 filesystem.")
endif()

# Sets the release types, e.g. Release, Debug:
# set(CMAKE_BUILD_TYPE Debug)

# Asserts enabled in CppAD otherwise
if(NOT CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DNDEBUG)
endif()

option(COMPILE_TESTS "Should the automated tests be compiled" OFF)
option(SIMPLE_OUTPUT_CHARS "Whether to avoid using special characters in the console output (for example on MinGW)" OFF)

# Activates extra functionality, note that corresponding libraries may be needed

# AMPL
option(HAS_AMPL "Should the AMPL .nl interface be build" ON)

# GAMS
option(HAS_GAMS "Is GAMS available" OFF)
set(GAMS_DIR "/opt/gams/gams30.2" CACHE STRING "The base directory where GAMS is located (if available)")

# CPLEX
option(HAS_CPLEX "Is Cplex available" OFF)
set(CPLEX_DIR "/opt/ibm/ILOG/CPLEX_Studio1210" CACHE STRING "The base directory where CPLEX is located (if available)")

# Gurobi
option(HAS_GUROBI "Is Gurobi available" OFF)
set(GUROBI_DIR "/opt/gurobi/gurobi900" CACHE STRING "The base directory where Gurobi is located (if available)")

# Cbc
option(HAS_CBC "Is Cbc available" OFF)
set(CBC_DIR "/opt/Cbc-2.10" CACHE STRING "The base directory where Cbc is located (if available).")

# Ipopt
option(HAS_IPOPT "Is Ipopt available" OFF)
set(IPOPT_DIR "/opt/ipopt" CACHE STRING "The base directory where Ipopt is located (if available).")

# Create also the executable
option(GENERATE_EXE "Should the SHOT executable be generated (requires at least that either OS or GAMS is available)"
       ON)

# External libraries; these are included in SHOT's source tree, and should not need to be modified
set(AMPL_DIR "ThirdParty/ampl")
set(ARGH_DIR "ThirdParty/argh")
set(BOOST_DIR "ThirdParty/boost")
set(CPPAD_DIR "ThirdParty/CppAD")
set(EIGEN_DIR "ThirdParty/eigen")
set(MCPP_DIR "ThirdParty/mc++")
set(SPDLOG_DIR "ThirdParty/spdlog")
set(TINYXML2_DIR "ThirdParty/tinyxml2")

# Makes sure the provided directories are in CMake format
file(TO_CMAKE_PATH "${AMPL_DIR}" ${AMPL_DIR})
file(TO_CMAKE_PATH "${BOOST_DIR}" ${BOOST_DIR})
file(TO_CMAKE_PATH "${CBC_DIR}" ${CBC_DIR})
file(TO_CMAKE_PATH "${CPLEX_DIR}" ${CPLEX_DIR})
file(TO_CMAKE_PATH "${GAMS_DIR}" ${GAMS_DIR})
file(TO_CMAKE_PATH "${GUROBI_DIR}" ${GUROBI_DIR})
file(TO_CMAKE_PATH "${IPOPT_DIR}" ${IPOPT_DIR})

# Check if a MIP solver is defined
if(NOT (HAS_CPLEX OR HAS_GUROBI OR HAS_CBC))
    message(FATAL_ERROR "No MIP solver defined. SHOT needs at least one!")
endif()

# Try to find Git (used to get the hash shown by the console program)
find_program(GIT git)

if((GIT) AND (EXISTS ${CMAKE_CURRENT_LIST_DIR}/.git))
    execute_process(
        COMMAND
            ${GIT}
            describe
            --always
            --dirty
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE GITHASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(
        REGEX
        REPLACE
            "^.*-g"
            ""
            GITHASH
            ${GITHASH}
    )
else()
    set(GITHASH "n/a")
endif()
message(STATUS "Git hash: " ${GITHASH})

mark_as_advanced(GITHASH)

configure_file("${PROJECT_SOURCE_DIR}/src/SHOTConfig.h.in" "${PROJECT_BINARY_DIR}/SHOTConfig.h")

include_directories("${PROJECT_BINARY_DIR}")

set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)

# Adding header files
file(
    GLOB_RECURSE
    HEADERS
    "${PROJECT_SOURCE_DIR}/src/Enums.h"
    "${PROJECT_SOURCE_DIR}/src/Structs.h"
    "${PROJECT_SOURCE_DIR}/src/Environment.h"
    "${PROJECT_SOURCE_DIR}/src/EventHandler.h"
    "${PROJECT_SOURCE_DIR}/src/Model/Variables.h"
    "${PROJECT_SOURCE_DIR}/src/Model/Terms.h"
    "${PROJECT_SOURCE_DIR}/src/Model/AuxiliaryVariables.h"
    "${PROJECT_SOURCE_DIR}/src/Model/ObjectiveFunction.h"
    "${PROJECT_SOURCE_DIR}/src/Model/NonlinearExpressions.h"
    "${PROJECT_SOURCE_DIR}/src/Model/Constraints.h"
    "${PROJECT_SOURCE_DIR}/src/Model/Problem.h"
    "${PROJECT_SOURCE_DIR}/src/Model/ModelHelperFunctions.h"
    "${PROJECT_SOURCE_DIR}/src/Report.h"
    "${PROJECT_SOURCE_DIR}/src/Iteration.h"
    "${PROJECT_SOURCE_DIR}/src/Timing.h"
    "${PROJECT_SOURCE_DIR}/src/Timer.h"
    "${PROJECT_SOURCE_DIR}/src/Output.h"
    "${PROJECT_SOURCE_DIR}/src/DualSolver.h"
    "${PROJECT_SOURCE_DIR}/src/PrimalSolver.h"
    "${PROJECT_SOURCE_DIR}/src/Results.h"
    "${PROJECT_SOURCE_DIR}/src/Solver.h"
    "${PROJECT_SOURCE_DIR}/src/TaskHandler.h"
    "${PROJECT_SOURCE_DIR}/src/Utilities.h"
    "${PROJECT_SOURCE_DIR}/src/Simplifications.h"
    "${PROJECT_SOURCE_DIR}/src/ModelingSystem/IModelingSystem.h"
    "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemOSiL.h"
    "${PROJECT_SOURCE_DIR}/src/ConstraintSelectionStrategy/*.h"
    "${PROJECT_SOURCE_DIR}/src/RootsearchMethod/IRootsearchMethod.h"
    "${PROJECT_SOURCE_DIR}/src/RootsearchMethod/RootsearchMethodBoost.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/IMIPSolutionLimitStrategy.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/IMIPSolver.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/IRelaxationStrategy.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyAdaptive.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyIncrease.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyUnlimited.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverBase.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyBase.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyNone.h"
    "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyStandard.h"
    "${PROJECT_SOURCE_DIR}/src/NLPSolver/INLPSolver.h"
    "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverBase.h"
    "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverCuttingPlaneMinimax.h"
    "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverSHOT.h"
    "${PROJECT_SOURCE_DIR}/src/SolutionStrategy/*.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/*.h"
    "${PROJECT_SOURCE_DIR}/src/Settings.h"
    "${TINYXML2_DIR}/tinyxml2.h"
    #"${CPPAD_DIR}/include/cppad/cppad.hpp"
)

# Cbc

if(HAS_CBC)
    find_package(CBC)

    if(CBC_FOUND)
        link_directories(${CBC_LIBRARY_DIRS})
        include_directories(SYSTEM ${CBC_INCLUDE_DIRS})
        add_definitions(-DHAS_CBC)
    endif(CBC_FOUND)

    if(CBC_FOUND)
        set(DUAL_SOURCES "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCbc.cpp")
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp")
        set(DUAL_HEADERS "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCbc.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
    endif(CBC_FOUND)
endif(HAS_CBC)

# Cplex

if(HAS_CPLEX)
    find_package(CPLEX)
    add_definitions(-DIL_STD)
    add_definitions(-DHAS_CPLEX)

    if(CPLEX_FOUND)
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp")
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplex.cpp")

        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexSingleTree.cpp")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexSingleTree.h")

        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexSingleTreeLegacy.cpp")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplex.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexSingleTreeLegacy.h")
    endif(CPLEX_FOUND)
endif(HAS_CPLEX)

# Gurobi

if(HAS_GUROBI)
    find_package(Gurobi)
    add_definitions(-DHAS_GUROBI)

    if(GUROBI_FOUND)
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp")
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.cpp")
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiSingleTree.cpp")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiSingleTree.h")
    endif(GUROBI_FOUND)
endif(HAS_GUROBI)

# Ipopt

if(HAS_IPOPT)
    find_package(PkgConfig)
    if(PkgConfig_FOUND)
        include(FindPkgConfig)
    endif()
    set(ENV{PKG_CONFIG_PATH} "${IPOPT_DIR}/lib/pkgconfig")

    pkg_search_module(IPOPT REQUIRED ipopt)
    link_directories(${IPOPT_LIBRARY_DIRS})
    include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS})

    set(PRIMAL_SOURCES "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.cpp")
    set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.cpp")
    set(PRIMAL_HEADERS "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.h")
    set(PRIMAL_HEADERS ${PRIMAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.h")
endif(HAS_IPOPT)

# AMPL interface

if(HAS_AMPL)
    add_definitions(-DHAS_AMPL)
    set(
        MODELING_SOURCES
        ${MODELING_SOURCES}
        "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemAMPL.cpp"
        "${AMPL_DIR}/src/expr-info.cc"
        "${AMPL_DIR}/src/format.cc"
        "${AMPL_DIR}/src/gen-expr-info.cc"
        "${AMPL_DIR}/src/nl-reader.cc"
        "${AMPL_DIR}/src/posix.cc"
        "${AMPL_DIR}/src/problem.cc"
        "${AMPL_DIR}/src/os.cc"
    )
    set(MODELING_HEADERS "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemAMPL.h" "${AMPL_DIR}/include/mp/*.h")
endif(HAS_AMPL)

# GAMS interface

if(HAS_GAMS)
    add_definitions(-DHAS_GAMS)

    set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${GAMS_DIR}/apifiles/C/api/gmomcc.c")
    set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${GAMS_DIR}/apifiles/C/api/gevmcc.c")
    set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${GAMS_DIR}/apifiles/C/api/palmcc.c")

    if(EXISTS ${GAMS_DIR}/apifiles/C/api/gcmt.c)
        set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${GAMS_DIR}/apifiles/C/api/gcmt.c")
        set(PRIMAL_HEADERS ${PRIMAL_HEADERS} "${GAMS_DIR}/apifiles/C/api/gcmt.h")
    endif()

    set(MODELING_SOURCES ${MODELING_SOURCES} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemGAMS.cpp")
    set(MODELING_SOURCES ${MODELING_SOURCES} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/EntryPointsGAMS.cpp")
    set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverGAMS.cpp")
    set(MODELING_HEADERS ${MODELING_HEADERS} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemGAMS.h")
    set(PRIMAL_HEADERS ${PRIMAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverGAMS.h")
    set(MODELING_HEADERS ${MODELING_HEADERS} "${PROJECT_SOURCE_DIR}/src/GAMS/GamsNLinstr.h")

endif(HAS_GAMS)

# Finding out link directories from header files

set(INCLUDE_DIRS "")
foreach(_headerFile ${HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    file(TO_CMAKE_PATH "${_dir}" _dir)
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()

foreach(_headerFile ${DUAL_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    file(TO_CMAKE_PATH "${_dir}" _dir)
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()

foreach(_headerFile ${PRIMAL_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    file(TO_CMAKE_PATH "${_dir}" _dir)
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()

foreach(_headerFile ${MODELING_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    file(TO_CMAKE_PATH "${_dir}" _dir)
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()

list(REMOVE_DUPLICATES INCLUDE_DIRS)
include_directories(${INCLUDE_DIRS})

# Add the third party include directories
include_directories(SYSTEM "${AMPL_DIR}/include")
include_directories(SYSTEM "${ARGH_DIR}")
include_directories(SYSTEM "${BOOST_DIR}")
include_directories(SYSTEM "${CPPAD_DIR}/include/")
include_directories(SYSTEM "${EIGEN_DIR}")
include_directories(SYSTEM "${MCPP_DIR}/include")
#include_directories(SYSTEM "${MCPP_DIR}/3rdparty/cpplapack/include")
#include_directories(SYSTEM "${MCPP_DIR}/3rdparty/fadbad++")
include_directories(SYSTEM "${SPDLOG_DIR}/include")

# Make sure the source file lists are in the correct format
file(TO_CMAKE_PATH "${SOURCES}" SOURCES)
file(TO_CMAKE_PATH "${DUAL_SOURCES}" DUAL_SOURCES)
file(TO_CMAKE_PATH "${PRIMAL_SOURCES}" PRIMAL_SOURCES)
file(TO_CMAKE_PATH "${MODELING_SOURCES}" MODELING_SOURCES)

# Will use tinyxml2 as a static library
add_library(tinyxml2 STATIC ${TINYXML2_DIR}/tinyxml2.h ${TINYXML2_DIR}/tinyxml2.cpp)

# Adding spdlog using as an external project
ExternalProject_Add(spdlog
    PREFIX spdlog
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/${SPDLOG_DIR}
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX=""
    -DSPDLOG_BUILD_SHARED=OFF
)

# Adding CppAD using as an external project
ExternalProject_Add(cppad
    PREFIX cppad
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/${CPPAD_DIR}
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -Dcppad_prefix=${CMAKE_CURRENT_BINARY_DIR}/CppAD
    -Dcppad_cxx_flags="-std=c++17"
)

# Creates the helper library
add_library(
    SHOTHelper STATIC
    ${PROJECT_SOURCE_DIR}/src/Enums.h
    ${PROJECT_SOURCE_DIR}/src/Structs.h
    ${PROJECT_SOURCE_DIR}/src/Settings.h
    ${PROJECT_SOURCE_DIR}/src/Settings.cpp
    ${PROJECT_SOURCE_DIR}/src/Output.h
    ${PROJECT_SOURCE_DIR}/src/Output.cpp
    ${PROJECT_SOURCE_DIR}/src/Utilities.h
    ${PROJECT_SOURCE_DIR}/src/Utilities.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskBase.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskBase.cpp
    ${PROJECT_SOURCE_DIR}/src/TaskHandler.h
    ${PROJECT_SOURCE_DIR}/src/TaskHandler.cpp
)
target_link_libraries(SHOTHelper tinyxml2)

add_dependencies(SHOTHelper spdlog)
add_dependencies(SHOTHelper cppad)

# Creates the model library
add_library(
    SHOTModel STATIC
    ${PROJECT_SOURCE_DIR}/src/Model/Problem.h
    ${PROJECT_SOURCE_DIR}/src/Model/Problem.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/Constraints.h
    ${PROJECT_SOURCE_DIR}/src/Model/Constraints.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/ObjectiveFunction.h
    ${PROJECT_SOURCE_DIR}/src/Model/ObjectiveFunction.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/Terms.h
    ${PROJECT_SOURCE_DIR}/src/Model/Terms.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/NonlinearExpressions.h
    ${PROJECT_SOURCE_DIR}/src/Model/NonlinearExpressions.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/Variables.h
    ${PROJECT_SOURCE_DIR}/src/Model/Variables.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/AuxiliaryVariables.h
    ${PROJECT_SOURCE_DIR}/src/Model/AuxiliaryVariables.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/Simplifications.h
    ${PROJECT_SOURCE_DIR}/src/Model/Simplifications.cpp
)
target_link_libraries(SHOTModel SHOTHelper)

# Creates the results library
add_library(
    SHOTResults STATIC
    ${PROJECT_SOURCE_DIR}/src/Results.h
    ${PROJECT_SOURCE_DIR}/src/Results.cpp
    ${PROJECT_SOURCE_DIR}/src/Iteration.h
    ${PROJECT_SOURCE_DIR}/src/Iteration.cpp
    ${PROJECT_SOURCE_DIR}/src/Timing.h
)
target_link_libraries(SHOTResults SHOTModel)

# Creates the primal strategy library
add_library(
    SHOTPrimalStrategy STATIC
    ${PRIMAL_SOURCES}
    ${PROJECT_SOURCE_DIR}/src/PrimalSolver.h
    ${PROJECT_SOURCE_DIR}/src/PrimalSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverBase.cpp
    ${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverCuttingPlaneMinimax.cpp
    ${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverSHOT.cpp
)
target_link_libraries(SHOTPrimalStrategy SHOTModel)

# Creates the dual strategy library
add_library(
    SHOTDualStrategy STATIC
    ${DUAL_SOURCES}
    ${PROJECT_SOURCE_DIR}/src/DualSolver.h
    ${PROJECT_SOURCE_DIR}/src/DualSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/IMIPSolutionLimitStrategy.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyAdaptive.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyAdaptive.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyIncrease.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyIncrease.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyUnlimited.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyUnlimited.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverBase.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverBase.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyBase.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyBase.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyNone.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyNone.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyStandard.h
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyStandard.cpp
)
target_link_libraries(SHOTDualStrategy SHOTModel)

# Creates the tasks library
file(GLOB_RECURSE TASK_SOURCES "${PROJECT_SOURCE_DIR}/src/Tasks/*.cpp")
# These sources are already added in SHOTHelper
list(REMOVE_ITEM TASK_SOURCES "${PROJECT_SOURCE_DIR}/src/Tasks/TaskBase.h")
list(REMOVE_ITEM TASK_SOURCES "${PROJECT_SOURCE_DIR}/src/Tasks/TaskBase.cpp")
add_library(SHOTTasks STATIC ${TASK_SOURCES})
target_link_libraries(SHOTTasks SHOTPrimalStrategy)
target_link_libraries(SHOTTasks SHOTDualStrategy)

# Creates the solution strategies library
file(GLOB_RECURSE STRATEGIES_SOURCES "${PROJECT_SOURCE_DIR}/src/SolutionStrategy/*.cpp")
add_library(SHOTSolutionStrategies STATIC ${STRATEGIES_SOURCES})
target_link_libraries(SHOTSolutionStrategies SHOTTasks)

# Creates the modeling interfaces library
set(MODELING_SOURCES ${MODELING_SOURCES} ${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemOSiL.cpp)
add_library(SHOTModelingInterfaces STATIC ${MODELING_SOURCES})
target_link_libraries(SHOTModelingInterfaces SHOTModel)

# Cplex linking
if(HAS_CPLEX)
    if(NOT (CPLEX_FOUND) OR NOT (CPLEX_CONCERT_FOUND) OR NOT (CPLEX_ILOCPLEX_FOUND))
        message("-- CPLEX libraries could not be found!")
    else()

        message("-- CPLEX include files will be used from:")
        message("   ${CPLEX_INCLUDE_DIR}")
        message("   ${CPLEX_CONCERT_INCLUDE_DIR}")
        message("   ${CPLEX_ILOCPLEX_INCLUDE_DIR}")
        include_directories(SYSTEM "${CPLEX_INCLUDE_DIR}")
        include_directories(SYSTEM "${CPLEX_CONCERT_INCLUDE_DIR}")
        include_directories(SYSTEM "${CPLEX_ILOCPLEX_INCLUDE_DIR}")

        message("-- The following CPLEX libraries will be used:")
        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            message("   ${CPLEX_LIBRARY_DEBUG}")
            message("   ${CPLEX_ILOCPLEX_LIBRARY_DEBUG}")
            message("   ${CPLEX_CONCERT_LIBRARY_DEBUG}")

            target_link_libraries(SHOTDualStrategy ${CPLEX_CONCERT_LIBRARY_DEBUG})
            target_link_libraries(SHOTDualStrategy ${CPLEX_ILOCPLEX_LIBRARY_DEBUG})
            target_link_libraries(SHOTDualStrategy ${CPLEX_LIBRARY_DEBUG})
        else()
            message("   ${CPLEX_LIBRARY}")
            message("   ${CPLEX_CONCERT_LIBRARY}")
            message("   ${CPLEX_ILOCPLEX_LIBRARY}")

            target_link_libraries(SHOTDualStrategy ${CPLEX_CONCERT_LIBRARY})
            target_link_libraries(SHOTDualStrategy ${CPLEX_ILOCPLEX_LIBRARY})
            target_link_libraries(SHOTDualStrategy ${CPLEX_LIBRARY})
        endif()
    endif()
endif(HAS_CPLEX)

# Gurobi linking
if(HAS_GUROBI)
    if(NOT (GUROBI_FOUND))
        message("-- Gurobi libraries could not be found!")
    else()
        message("-- Gurobi include files will be used from: ${GUROBI_INCLUDE_DIR}")
        include_directories(SYSTEM "${GUROBI_INCLUDE_DIR}")

        message("-- The following Gurobi libraries will be used:")
        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            message("   ${GUROBI_LIBRARY_DEBUG}")
            message("   ${GUROBI_CPP_LIBRARY}")

            target_link_libraries(SHOTDualStrategy ${GUROBI_CPP_LIBRARY})
            target_link_libraries(SHOTDualStrategy ${GUROBI_LIBRARY_DEBUG})
        else()
            message("   ${GUROBI_LIBRARY}")
            message("   ${GUROBI_CPP_LIBRARY}")

            target_link_libraries(SHOTDualStrategy ${GUROBI_CPP_LIBRARY})
            target_link_libraries(SHOTDualStrategy ${GUROBI_LIBRARY})
        endif()
    endif()
endif(HAS_GUROBI)

# Cbc linking
if(HAS_CBC)
    if(NOT (CBC_FOUND))
        message("-- Cbc libraries could not be found!")
    else()
        message("-- Cbc include files will be used from: ${CBC_INCLUDE_DIRS}")

        target_link_libraries(SHOTDualStrategy ${CBC_LIBRARIES})
        target_compile_options(SHOTDualStrategy PUBLIC ${CBC_CFLAGS_OTHER})

        message("-- The following Cbc libraries will be used from: ${CBC_LIBRARY_DIRS}")
        message("   ${CBC_LIBRARIES}")
    endif()
endif(HAS_CBC)

# Ipopt linking
if(HAS_IPOPT)
    message("-- Ipopt include files will be used from: ${IPOPT_DIR}/include/coin")
    target_link_libraries(SHOTPrimalStrategy ${IPOPT_LIBRARIES})
    target_include_directories(SHOTPrimalStrategy PUBLIC "${IPOPT_DIR}/include/coin")
    target_compile_options(SHOTPrimalStrategy PUBLIC ${IPOPT_CFLAGS_OTHER})
    add_definitions(-DHAS_IPOPT)

    message("-- The following Ipopt libraries will be used from: ${IPOPT_LIBRARY_DIRS}")
    message("   ${IPOPT_LIBRARIES}")
endif(HAS_IPOPT)

# GAMS linking
if(HAS_GAMS)
    message("-- GAMS include files will be used from: ${GAMS_DIR}/apifiles/C/api")
    include_directories(SYSTEM "${GAMS_DIR}/apifiles/C/api")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGAMSDIR=\\\"${GAMS_DIR}\\\"")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DGC_NO_MUTEX")
endif(HAS_GAMS)

file(
    GLOB_RECURSE
    SOURCES
    "${PROJECT_SOURCE_DIR}/src/Report.cpp"
    "${PROJECT_SOURCE_DIR}/src/Solver.cpp"
    "${PROJECT_SOURCE_DIR}/src/RootsearchMethod/RootsearchMethodBoost.cpp"
)

# Creates the SHOT library that is linked to the executable
add_library(SHOTSolver SHARED ${SOURCES})

# Links the compiled libraries
target_link_libraries(SHOTSolver SHOTHelper)
target_link_libraries(SHOTSolver SHOTModel)
target_link_libraries(SHOTSolver SHOTResults)
target_link_libraries(SHOTSolver SHOTDualStrategy)
target_link_libraries(SHOTSolver SHOTPrimalStrategy)
target_link_libraries(SHOTSolver SHOTTasks)

if(MSYS OR MINGW)
    # MinGW cross compilation will not work if "-Wl,--whole-archive" is defined below
    target_link_libraries(SHOTSolver SHOTModelingInterfaces)
elseif(APPLE)
    # need these all-load flags to ensure that EntryPointGAMS symbols will be present in library
    target_link_libraries(SHOTSolver "-Wl,-all_load" SHOTModelingInterfaces)
else()
    # need these whole-archive flags to ensure that EntryPointGAMS symbols will be present in library
    target_link_libraries(SHOTSolver "-Wl,--whole-archive" SHOTModelingInterfaces "-Wl,--no-whole-archive")
endif()

target_link_libraries(SHOTSolver SHOTSolutionStrategies)

# Link the standard library required for std::filesystem (if needed)
target_link_libraries(SHOTSolver CXX::Filesystem)

# Generates the SHOT executable
if(GENERATE_EXE)
    add_executable(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/src/SHOT.cpp")
    target_link_libraries(${PROJECT_NAME} SHOTSolver)

    # Extra linking necessary for GAMS
    if(HAS_GAMS)
        if(UNIX)
            if(APPLE)
                target_link_libraries(${PROJECT_NAME} ${GAMS_DIR}/libstdc++.6.dylib)
            else(APPLE)
                target_link_libraries(${PROJECT_NAME} ${GAMS_DIR}/libstdc++.so.6)
            endif(APPLE)
        endif(UNIX)
    endif(HAS_GAMS)
endif(GENERATE_EXE)

# Extra flags for Visual Studio compilers
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
    add_definitions(-D_USE_MATH_DEFINES)
endif(MSVC)

# Extra flags for MinGW
if (MSVC OR MSYS OR MINGW)
    add_definitions(-DSIMPLE_OUTPUT_CHARS)
endif()

# Set compiler warnings levels
if(MSVC)
    target_compile_options(SHOTSolver PRIVATE /W4 /WX)
else()
    target_compile_options(
        SHOTSolver
        PRIVATE
            -Wall
            -Wextra
            -pedantic
            -Wno-unused-local-typedefs
    )
endif()

# For making the installation package
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    MyLibConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT SHOTTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Copy some additional files
install(
    FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/AUTHORS
    ${CMAKE_CURRENT_SOURCE_DIR}/INSTALL
    ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
    ${CMAKE_CURRENT_SOURCE_DIR}/README
    DESTINATION bin
)

# Need to install the library as well
install(TARGETS SHOTSolver DESTINATION lib)

install(
    EXPORT SHOTTargets
    FILE SHOTTargets.cmake
    NAMESPACE SHOT::
    DESTINATION lib/cmake/SHOT
)

# Copy all header files including directory structure
install(
    DIRECTORY "${CMAKE_SOURCE_DIR}/src/" # source directory
    DESTINATION "include" # target directory
    FILES_MATCHING # install only matched files
    PATTERN  "*.h" # select header files
)

# Setting CPack metadata
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Andreas Lundell")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Supporting Hyperplane Optimization Toolkit (SHOT)")
set(CPACK_PACKAGE_VENDOR "Åbo Akademi University")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/INSTALL")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_GENERATOR "STGZ;ZIP")
include(CPack)

# Add a target to generate API documentation with Doxygen
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation(requires Doxygen)" ${DOXYGEN_FOUND})

if(BUILD_DOCUMENTATION)
    find_package(Doxygen)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doxydoc/doxygen.conf.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(
        doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doxydoc/html DESTINATION docs)
endif()

if(COMPILE_TESTS)
    # For performing automated tests
    enable_testing()
    add_subdirectory("${PROJECT_SOURCE_DIR}/test")
endif()
