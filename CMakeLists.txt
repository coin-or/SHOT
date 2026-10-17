cmake_minimum_required(VERSION 3.20)
include(ExternalProject)

# For ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()


project(SHOT VERSION 2.0.0
                  DESCRIPTION "The Supporting Hyperplane Optimization Toolkit (SHOT) for Mixed-Integer Nonlinear Optimization"
                  LANGUAGES CXX C)

# Pre-release suffix of the version (a<n>, b<n> or rc<n>), empty for a final release.
# A release tag v<version>[-<suffix>] must match the version and suffix here.
set(SHOT_VERSION_SUFFIX "rc0")

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Location of extra CMake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/misc")

# Find std::filesystem or std::experimental::filesystem
find_package(Filesystem REQUIRED)

if(NOT HAVE_STD_FILESYSTEM AND NOT HAVE_STD_EXPERIMENTAL_FILESYSTEM)
    message(SEND_ERROR "SHOT needs support for C++17 filesystem.")
endif()

# CMake's configuration-specific compiler flags define NDEBUG for optimized
# configurations. Do not set it directory-wide: multi-config generators also
# build Debug targets from this directory.

option(SHOT_WARNINGS_AS_ERRORS "Treat warnings in SHOT C++ code as errors" OFF)
option(SHOT_EXTRA_WARNINGS "Enable additional, potentially noisy conversion and shadowing diagnostics" OFF)
option(COMPILE_TESTS "Should the automated tests be compiled" OFF)
option(SIMPLE_OUTPUT_CHARS "Whether to avoid using special characters in the console output (for example on MinGW)" OFF)

# Activates extra functionality, note that corresponding libraries may be needed

# AMPL
option(HAS_AMPL "Should the AMPL .nl interface be build" ON)

# GAMS
option(HAS_GAMS "Is GAMS available" ON)
set(GAMS_DIR "/opt/gams" CACHE STRING "The base directory where GAMS is located (if available)")

# CPLEX
option(HAS_CPLEX "Is Cplex available" ON)
set(CPLEX_DIR "/opt/cplex" CACHE STRING "The base directory where CPLEX is located (if available)")

# Gurobi
option(HAS_GUROBI "Is Gurobi available" ON)
set(GUROBI_DIR "/opt/gurobi" CACHE STRING "The base directory where Gurobi is located (if available)")

# Cbc
option(HAS_CBC "Is Cbc available" ON)
if(DEFINED ENV{CBC_DIR} AND NOT "$ENV{CBC_DIR}" STREQUAL "")
    set(CBC_DEFAULT_DIR "$ENV{CBC_DIR}")
else()
    set(CBC_DEFAULT_DIR "/opt/cbc")
endif()
set(CBC_DIR "${CBC_DEFAULT_DIR}" CACHE STRING "The base directory where Cbc is located (if available).")

# Ipopt
option(HAS_IPOPT "Is Ipopt available" ON)
if(DEFINED ENV{IPOPT_DIR} AND NOT "$ENV{IPOPT_DIR}" STREQUAL "")
    set(IPOPT_DEFAULT_DIR "$ENV{IPOPT_DIR}")
else()
    set(IPOPT_DEFAULT_DIR "/opt/ipopt")
endif()
set(IPOPT_DIR "${IPOPT_DEFAULT_DIR}" CACHE STRING "The base directory where Ipopt is located (if available).")

option(HAS_UNO "Is Uno available" OFF)
set(UNO_DIR "/opt/uno" CACHE STRING "The base directory where Uno is located (if available).")

# Python
option(HAS_PYTHON "Is Python available" OFF)

# HiGHS
option(HAS_HIGHS "Is HiGHS available" ON)
option(USE_EXTERNAL_HIGHS "Use precompiled, external HiGHS libraries" OFF)
set(HIGHS_EXTERNAL_DIR "/opt/highs" CACHE STRING "The external directory where HiGHS is located).")

# Create also the executable
option(GENERATE_EXE "Should the SHOT executable be generated (requires at least that either OS or GAMS is available)"
       ON)

# External libraries; these are included in SHOT's source tree, and should not need to be modified
set(AMPL_DIR "ThirdParty/ampl")
set(ARGH_DIR "ThirdParty/argh")
set(BOOST_DIR "ThirdParty/boost")
set(CPPAD_DIR "ThirdParty/CppAD")
set(EIGEN_DIR "ThirdParty/eigen")
set(HIGHS_DIR "ThirdParty/HiGHS")
set(MCPP_DIR "ThirdParty/mc++")
set(PYBIND11_DIR "ThirdParty/pybind11")
set(SPDLOG_DIR "ThirdParty/spdlog")
set(TINYXML2_DIR "ThirdParty/tinyxml2")

# Makes sure the provided directories are in CMake format
file(TO_CMAKE_PATH "${AMPL_DIR}" ${AMPL_DIR})
file(TO_CMAKE_PATH "${BOOST_DIR}" ${BOOST_DIR})
file(TO_CMAKE_PATH "${CBC_DIR}" ${CBC_DIR})
file(TO_CMAKE_PATH "${CPLEX_DIR}" ${CPLEX_DIR})
file(TO_CMAKE_PATH "${GAMS_DIR}" ${GAMS_DIR})
file(TO_CMAKE_PATH "${GUROBI_DIR}" ${GUROBI_DIR})
file(TO_CMAKE_PATH "${HIGHS_EXTERNAL_DIR}" ${HIGHS_EXTERNAL_DIR})
file(TO_CMAKE_PATH "${IPOPT_DIR}" ${IPOPT_DIR})

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
        RESULT_VARIABLE GIT_DESCRIBE_RESULT
        ERROR_QUIET
    )
    if(GIT_DESCRIBE_RESULT EQUAL 0 AND NOT GITHASH STREQUAL "")
        string(
            REGEX
            REPLACE
                "^.*-g"
                ""
                GITHASH
                "${GITHASH}"
        )
    else()
        set(GITHASH "n/a")
    endif()
else()
    set(GITHASH "n/a")
endif()
message(STATUS "Git hash: " ${GITHASH})

mark_as_advanced(GITHASH)

configure_file("${PROJECT_SOURCE_DIR}/src/SHOTConfig.h.in" "${PROJECT_BINARY_DIR}/SHOTConfig.h")

set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)

# Adding header files
file(
    GLOB_RECURSE
    HEADERS
    "${PROJECT_SOURCE_DIR}/src/Enums.h"
    "${PROJECT_SOURCE_DIR}/src/Structs.h"
    "${PROJECT_SOURCE_DIR}/src/Environment.h"
    "${PROJECT_SOURCE_DIR}/src/Callback.h"
    "${PROJECT_SOURCE_DIR}/src/Model/Interval.h"
    "${PROJECT_SOURCE_DIR}/src/Model/IntervalDerivatives.h"
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
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskExecuteRelaxationStrategy.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskTerminate.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskReformulateProblem.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeIteration.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckAbsoluteGap.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckDualStagnation.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeRootsearch.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesObjectiveFunction.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskFindInteriorPoint.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskPerformBoundTightening.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddPrimalReductionCut.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSimple.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskPrintIterationReport.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSolveIteration.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskRepairableBase.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskBase.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckIterationLimit.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromSolutionPool.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddIntegerCuts.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskGoto.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCreateDualProblem.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskConditional.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskException.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddHyperplanes.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckMaxNumberOfPrimalReductionCuts.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesECP.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskUpdateInteriorPoint.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskExecuteSolutionLimitStrategy.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeDualSolver.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCalculateSolutionChangeNorm.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromNLP.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckTimeLimit.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskClearFixedPrimalCandidates.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskEnumerateFixedIntegerCombinations.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesESH.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckUserTermination.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckPrimalStagnation.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromRootsearch.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckConstraintTolerance.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckRelativeGap.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSequential.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckIterationError.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskRepairInfeasibleDualProblem.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesExternal.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromExternalSource.h"
    "${PROJECT_SOURCE_DIR}/src/Tasks/TaskUpdateExternalDualBound.h"
    "${PROJECT_SOURCE_DIR}/src/Settings.h"
    "${TINYXML2_DIR}/tinyxml2.h"
)

# Cbc

if(HAS_CBC)
    find_package(PkgConfig)

    if(PkgConfig_FOUND)
        include(FindPkgConfig)
        find_package(CBC)
    endif()

    if(NOT CBC_FOUND)
        message(WARNING "Cbc not found, setting HAS_CBC to OFF")
        set(HAS_CBC OFF)
    endif()

    if(CBC_FOUND)
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCbc.cpp")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCbc.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
    endif(CBC_FOUND)
endif(HAS_CBC)

# Cplex

if(HAS_CPLEX)
    find_package(CPLEX)

    if(NOT CPLEX_FOUND)
        message(WARNING "CPLEX not found, setting HAS_CPLEX to OFF")
        set(HAS_CPLEX OFF)
    endif()

    if(CPLEX_FOUND)
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
    find_package(GUROBI)

    if(NOT GUROBI_FOUND)
        message(WARNING "Gurobi not found, setting HAS_GUROBI to OFF")
        set(HAS_GUROBI OFF)
    endif()

    if(GUROBI_FOUND)
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.cpp")
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiSingleTree.cpp")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.h")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiSingleTree.h")
    endif(GUROBI_FOUND)
endif(HAS_GUROBI)

# HiGHS

# Case 1: Use the HiGHS submodule
if(HAS_HIGHS AND NOT USE_EXTERNAL_HIGHS)
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${HIGHS_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "HiGHS submodule not found in ${HIGHS_DIR}. "
                             "Run 'git submodule update --init --recursive' to fetch it, "
                             "or configure with -DUSE_EXTERNAL_HIGHS=on to use a precompiled HiGHS instead.")
    endif()

    set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverHighs.cpp")
    set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverHighs.h")

    add_subdirectory(${HIGHS_DIR})

    # Disable HiGHS assertions even in debug builds, since some of its internal consistency checks abort on badly
    # scaled problems that release builds get through
    target_compile_definitions(highs PRIVATE NDEBUG)
    # Keep third-party header diagnostics out of SHOT's warning policy (also on CMake 3.20).
    set_property(TARGET highs APPEND PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
        "$<TARGET_PROPERTY:highs,INTERFACE_INCLUDE_DIRECTORIES>")
endif()

# Case 2: Use external HiGHS
if(HAS_HIGHS AND USE_EXTERNAL_HIGHS)
    find_package(Threads REQUIRED)
    find_package(highs PATHS ${HIGHS_EXTERNAL_DIR} NO_DEFAULT_PATH)
    
    if(NOT highs_FOUND)
        message(WARNING "External HiGHS not found, setting HAS_HIGHS to OFF")
        set(HAS_HIGHS OFF)
    endif()
    
    if(highs_FOUND)
        set(DUAL_SOURCES ${DUAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverHighs.cpp")
        set(DUAL_HEADERS ${DUAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverHighs.h")
    endif()
endif()

# Check the backends that were actually found, after optional solvers may
# have been disabled by their discovery steps.
if(NOT (HAS_CPLEX OR HAS_GUROBI OR HAS_CBC OR HAS_HIGHS))
    message(FATAL_ERROR "No MIP solver available. Enable HiGHS or install Cbc, CPLEX, or Gurobi.")
endif()

# Ipopt

if(HAS_IPOPT)
    find_package(PkgConfig REQUIRED)
    set(_IPOPT_ORIGINAL_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
    list(PREPEND CMAKE_PREFIX_PATH "${IPOPT_DIR}")
    pkg_search_module(IPOPT REQUIRED IMPORTED_TARGET ipopt)
    set(CMAKE_PREFIX_PATH "${_IPOPT_ORIGINAL_PREFIX_PATH}")
    unset(_IPOPT_ORIGINAL_PREFIX_PATH)

    set(PRIMAL_SOURCES "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.cpp")
    set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.cpp")
    set(PRIMAL_HEADERS "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.h")
    set(PRIMAL_HEADERS ${PRIMAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.h")
endif(HAS_IPOPT)

if(HAS_UNO)
    find_package(Uno)

    if(NOT UNO_FOUND)
        message(WARNING "Uno not found, setting HAS_UNO to OFF")
        set(HAS_UNO OFF)
    endif()

    if(UNO_FOUND)
        set(PRIMAL_SOURCES ${PRIMAL_SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverUno.cpp")
        set(PRIMAL_HEADERS ${PRIMAL_HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverUno.h")
    endif()
endif(HAS_UNO)

# AMPL interface

if(HAS_AMPL)
    list(APPEND MODELING_SOURCES "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemAMPL.cpp")
    set(AMPL_SOURCES
        "${AMPL_DIR}/src/expr-info.cc"
        "${AMPL_DIR}/src/format.cc"
        "${AMPL_DIR}/src/expr.cc"
        "${AMPL_DIR}/src/nl-reader.cc"
        "${AMPL_DIR}/src/posix.cc"
        "${AMPL_DIR}/src/problem.cc"
        "${AMPL_DIR}/src/os.cc"
        "${AMPL_DIR}/src/sol.cc"
    )
    set(MODELING_HEADERS "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemAMPL.h" "${AMPL_DIR}/include/mp/*.h")
endif(HAS_AMPL)

# GAMS interface

if(HAS_GAMS)
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
# Shared usage requirements for SHOT-owned targets, including tests.
add_library(SHOTBuildSettings INTERFACE)
# Apply diagnostics to all SHOT targets, not bundled dependencies or consumers.
if(MSVC)
    target_compile_options(SHOTBuildSettings INTERFACE
        "$<$<COMPILE_LANGUAGE:CXX>:/W4;/permissive-;/Zc:__cplusplus>")
    if(SHOT_WARNINGS_AS_ERRORS)
        target_compile_options(SHOTBuildSettings INTERFACE "$<$<COMPILE_LANGUAGE:CXX>:/WX>")
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    if(SHOT_EXTRA_WARNINGS)
        target_compile_options(SHOTBuildSettings INTERFACE
            "$<$<COMPILE_LANGUAGE:CXX>:-Wconversion;-Wsign-conversion;-Wshadow;-Wnon-virtual-dtor;-Woverloaded-virtual>")
    endif()
    target_compile_options(SHOTBuildSettings INTERFACE
        "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-pedantic-errors;-Wformat=2;-Wimplicit-fallthrough>")
    if(SHOT_WARNINGS_AS_ERRORS)
        target_compile_options(SHOTBuildSettings INTERFACE "$<$<COMPILE_LANGUAGE:CXX>:-Werror>")
    endif()
endif()

target_include_directories(SHOTBuildSettings INTERFACE "${PROJECT_BINARY_DIR}" ${INCLUDE_DIRS})
target_include_directories(SHOTBuildSettings SYSTEM INTERFACE
    "${AMPL_DIR}/include"
    "${ARGH_DIR}"
    "${BOOST_DIR}"
    # Consume headers installed by the same ExternalProject as cppad_lib.
    # configure.hpp must match that library's build configuration.
    "${PROJECT_BINARY_DIR}/CppAD/include"
    "${EIGEN_DIR}"
    "${MCPP_DIR}/include"
    "${PYBIND11_DIR}/include"
    "${SPDLOG_DIR}/include")

foreach(_backend CBC CPLEX GUROBI HIGHS IPOPT UNO AMPL GAMS)
    if(HAS_${_backend})
        target_compile_definitions(SHOTBuildSettings INTERFACE HAS_${_backend})
    endif()
endforeach()
if(HAVE_STD_FILESYSTEM)
    target_compile_definitions(SHOTBuildSettings INTERFACE HAS_STD_FILESYSTEM)
else()
    target_compile_definitions(SHOTBuildSettings INTERFACE HAS_STD_EXPERIMENTAL_FILESYSTEM)
endif()
if(HAS_CPLEX)
    target_compile_definitions(SHOTBuildSettings INTERFACE IL_STD)
    target_include_directories(SHOTBuildSettings SYSTEM INTERFACE
        "${CPLEX_INCLUDE_DIR}" "${CPLEX_CONCERT_INCLUDE_DIR}" "${CPLEX_ILOCPLEX_INCLUDE_DIR}")
endif()
if(HAS_GUROBI)
    target_include_directories(SHOTBuildSettings SYSTEM INTERFACE "${GUROBI_INCLUDE_DIR}")
endif()
if(HAS_CBC)
    target_include_directories(SHOTBuildSettings SYSTEM INTERFACE ${CBC_INCLUDE_DIRS})
    target_compile_options(SHOTBuildSettings INTERFACE ${CBC_CFLAGS_OTHER})
endif()
if(HAS_IPOPT)
    target_include_directories(SHOTBuildSettings SYSTEM INTERFACE ${IPOPT_INCLUDE_DIRS})
    target_compile_options(SHOTBuildSettings INTERFACE ${IPOPT_CFLAGS_OTHER})
endif()
if(HAS_UNO)
    target_include_directories(SHOTBuildSettings SYSTEM INTERFACE "${UNO_INCLUDE_DIR}")
endif()
if(HAS_GAMS)
    target_include_directories(SHOTBuildSettings SYSTEM INTERFACE "${GAMS_DIR}/apifiles/C/api")
    target_compile_definitions(SHOTBuildSettings INTERFACE "GAMSDIR=\"${GAMS_DIR}\"")
endif()
if(MSVC)
    target_compile_definitions(SHOTBuildSettings INTERFACE _USE_MATH_DEFINES)
endif()
if(MSVC OR MSYS OR MINGW)
    target_compile_definitions(SHOTBuildSettings INTERFACE SIMPLE_OUTPUT_CHARS)
endif()

# Make sure the source file lists are in the correct format
file(TO_CMAKE_PATH "${SOURCES}" SOURCES)
file(TO_CMAKE_PATH "${DUAL_SOURCES}" DUAL_SOURCES)
file(TO_CMAKE_PATH "${PRIMAL_SOURCES}" PRIMAL_SOURCES)
file(TO_CMAKE_PATH "${MODELING_SOURCES}" MODELING_SOURCES)
file(TO_CMAKE_PATH "${TASK_SOURCES}" TASK_SOURCES)

# Will use tinyxml2 as a static library
add_library(tinyxml2 STATIC ${TINYXML2_DIR}/tinyxml2.h ${TINYXML2_DIR}/tinyxml2.cpp)

# Adding spdlog using as an external project
ExternalProject_Add(spdlog
    PREFIX spdlog
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/${SPDLOG_DIR}
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX=${PROJECT_BINARY_DIR}/spdlog-install
    -DSPDLOG_BUILD_SHARED=OFF
)

# Adding CppAD using as an external project
set(CPPAD_CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    -Dcppad_prefix=${CMAKE_CURRENT_BINARY_DIR}/CppAD
    -Dcppad_cxx_flags="-std=c++17"
)

# Add macOS-specific settings for proper @rpath usage
if(APPLE)
    list(APPEND CPPAD_CMAKE_ARGS
        -DCMAKE_MACOSX_RPATH=ON
        -DCMAKE_INSTALL_NAME_DIR=@rpath
    )
endif()

# Set CppAD library path
set(CPPAD_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/CppAD/lib")
if(APPLE)
    set(CPPAD_LIBRARY "${CPPAD_LIB_DIR}/libcppad_lib.dylib")
else()
    set(CPPAD_LIBRARY "${CPPAD_LIB_DIR}/libcppad_lib.so")
endif()

# The library is declared as a byproduct so that generators such as Ninja know that it is created by the external
# project, otherwise a clean build fails since no rule creates it
ExternalProject_Add(cppad
    PREFIX cppad
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/${CPPAD_DIR}
    CMAKE_ARGS ${CPPAD_CMAKE_ARGS}
    BUILD_BYPRODUCTS ${CPPAD_LIBRARY}
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
    ${PROJECT_SOURCE_DIR}/src/Callback.h
    ${PROJECT_SOURCE_DIR}/src/Callback.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskBase.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskBase.cpp
    ${PROJECT_SOURCE_DIR}/src/TaskHandler.h
    ${PROJECT_SOURCE_DIR}/src/TaskHandler.cpp
)
target_link_libraries(SHOTHelper tinyxml2 SHOTBuildSettings)

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
    ${PROJECT_SOURCE_DIR}/src/Model/Interval.h
    ${PROJECT_SOURCE_DIR}/src/Model/IntervalDerivatives.h
    ${PROJECT_SOURCE_DIR}/src/Model/IntervalDerivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/Variables.h
    ${PROJECT_SOURCE_DIR}/src/Model/Variables.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/AuxiliaryVariables.h
    ${PROJECT_SOURCE_DIR}/src/Model/AuxiliaryVariables.cpp
    ${PROJECT_SOURCE_DIR}/src/Model/Simplifications.h
    ${PROJECT_SOURCE_DIR}/src/Model/Simplifications.cpp
)
target_link_libraries(SHOTModel SHOTHelper SHOTBuildSettings)

# Creates the results library
add_library(
    SHOTResults STATIC
    ${PROJECT_SOURCE_DIR}/src/Results.h
    ${PROJECT_SOURCE_DIR}/src/Results.cpp
    ${PROJECT_SOURCE_DIR}/src/Iteration.h
    ${PROJECT_SOURCE_DIR}/src/Iteration.cpp
    ${PROJECT_SOURCE_DIR}/src/Timing.h
)
target_link_libraries(SHOTResults SHOTModel SHOTBuildSettings)

if(HAS_PYTHON)
    # Find Python before adding pybind11 to ensure correct paths are used
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
    # Force pybind11 to use modern FindPython3 instead of deprecated FindPythonLibs
    set(PYBIND11_FINDPYTHON ON CACHE BOOL "Force pybind11 to use FindPython3" FORCE)
    add_subdirectory(${PYBIND11_DIR})
    pybind11_add_module(SHOTpy src/SHOTpy.cpp)
    add_dependencies(SHOTpy cppad)
    target_link_libraries(SHOTpy PRIVATE SHOTSolver SHOTBuildSettings ${CPPAD_LIBRARY})

    # Set rpath for finding dynamic libraries. In the build directory they are found where they were built, which the
    # type stubs generated after the build rely on, and when installed, e.g., in a wheel, next to the module, where
    # auditwheel or delocate bundles the external solvers
    if(APPLE)
        set(SHOTPY_RPATH_DIRS "${CPPAD_LIB_DIR}")
        if(HAS_GUROBI AND GUROBI_FOUND)
            list(APPEND SHOTPY_RPATH_DIRS "${GUROBI_LIB_DIR}")
        endif()
        set_target_properties(SHOTpy PROPERTIES
            BUILD_RPATH "${SHOTPY_RPATH_DIRS}"
            INSTALL_RPATH "@loader_path")
    elseif(UNIX)
        set_target_properties(SHOTpy PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()

    # The Python package, built by pip through scikit-build-core (see pyproject.toml), consists of the module, the SHOT
    # library, CppAD and the type stubs, installed in the component Python
    install(TARGETS SHOTpy
        LIBRARY DESTINATION . COMPONENT Python
        RUNTIME DESTINATION . COMPONENT Python)
    install(DIRECTORY ${CPPAD_LIB_DIR}/
        DESTINATION .
        COMPONENT Python
        FILES_MATCHING
        PATTERN "libcppad_lib*.so*"
        PATTERN "libcppad_lib*.dylib")
    
    # Ensure CppAD symbols are shared between SHOTpy and SHOTSolver
    # This prevents ODR violations with CppAD's static local variables in templates
    # These linker flags are Linux-specific (GNU ld)
    if(UNIX AND NOT APPLE)
        target_link_options(SHOTpy PRIVATE "LINKER:--no-as-needed")
        target_link_options(SHOTpy PRIVATE "LINKER:--export-dynamic")
    endif()
    
    # Copy the SHOTpy tutorial notebook to the same directory as the SHOTpy module
    add_custom_command(TARGET SHOTpy POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/docs/SHOTpy_Tutorial.ipynb
            $<TARGET_FILE_DIR:SHOTpy>/SHOTpy_Tutorial.ipynb
        COMMENT "Copying SHOTpy tutorial notebook to module directory"
    )

    # Type stubs (SHOTpy.pyi) next to the module, so that editors and type checkers know the classes and signatures
    # of the compiled module. They are generated from the built module if pybind11-stubgen is installed, and a
    # signature that cannot be written as a Python type, e.g., one with a C++ type name, fails the build
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import pybind11_stubgen"
        RESULT_VARIABLE PYBIND11_STUBGEN_MISSING
        OUTPUT_QUIET ERROR_QUIET)

    if(NOT PYBIND11_STUBGEN_MISSING)
        add_custom_command(TARGET SHOTpy POST_BUILD
            COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/misc/GenerateSHOTpyStubs.py" "$<TARGET_FILE_DIR:SHOTpy>"
            COMMENT "Generating type stubs for SHOTpy"
        )

        # SHOTpy is a module, not a package, so the stubs are found next to it without a py.typed file
        install(FILES $<TARGET_FILE_DIR:SHOTpy>/SHOTpy.pyi DESTINATION . COMPONENT Python)
    else()
        message(STATUS "pybind11-stubgen not found - type stubs for SHOTpy will not be generated")
        message(STATUS "Install it with: pip install pybind11-stubgen")
    endif()
endif(HAS_PYTHON)

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
target_link_libraries(SHOTPrimalStrategy SHOTModel SHOTBuildSettings)

# Creates the dual strategy library
add_library(
    SHOTDualStrategy STATIC
    ${DUAL_SOURCES}
    ${PROJECT_SOURCE_DIR}/src/DualSolver.h
    ${PROJECT_SOURCE_DIR}/src/DualSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp
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
target_link_libraries(SHOTDualStrategy SHOTModel SHOTBuildSettings)


# Creates the tasks library

add_library(
    SHOTTasks STATIC
    ${TASK_SOURCES}
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddHyperplanes.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddHyperplanes.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddIntegerCuts.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddIntegerCuts.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddPrimalReductionCut.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskAddPrimalReductionCut.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCalculateSolutionChangeNorm.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCalculateSolutionChangeNorm.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckAbsoluteGap.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckAbsoluteGap.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckConstraintTolerance.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckConstraintTolerance.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckDualStagnation.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckDualStagnation.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckIterationError.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckIterationError.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckIterationLimit.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckIterationLimit.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckMaxNumberOfPrimalReductionCuts.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckMaxNumberOfPrimalReductionCuts.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckPrimalStagnation.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckPrimalStagnation.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckRelativeGap.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckRelativeGap.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckTimeLimit.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckTimeLimit.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckUserTermination.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCheckUserTermination.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskClearFixedPrimalCandidates.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskClearFixedPrimalCandidates.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskConditional.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskConditional.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCreateDualProblem.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCreateDualProblem.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskEnumerateFixedIntegerCombinations.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskEnumerateFixedIntegerCombinations.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCreateMIPProblem.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskCreateMIPProblem.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskException.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskExecuteRelaxationStrategy.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskExecuteRelaxationStrategy.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskExecuteSolutionLimitStrategy.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskExecuteSolutionLimitStrategy.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskFindInteriorPoint.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskFindInteriorPoint.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskGoto.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskGoto.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeDualSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeDualSolver.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeIteration.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeIteration.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeRootsearch.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskInitializeRootsearch.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskPerformBoundTightening.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskPerformBoundTightening.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskPerformConvexBounding.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskPerformConvexBounding.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskPrintIterationReport.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskPrintIterationReport.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskReformulateProblem.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskReformulateProblem.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskRepairInfeasibleDualProblem.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskRepairInfeasibleDualProblem.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskRepairableBase.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskRepairableBase.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesECP.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesECP.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesESH.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesESH.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesExternal.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesExternal.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesObjectiveFunction.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectHyperplanesObjectiveFunction.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromNLP.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromNLP.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromExternalSource.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromExternalSource.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromRootsearch.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromRootsearch.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromSolutionPool.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalCandidatesFromSolutionPool.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalFixedNLPPointsFromSolutionPool.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSelectPrimalFixedNLPPointsFromSolutionPool.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSequential.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSequential.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSimple.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSimple.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSolveIteration.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskSolveIteration.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskTerminate.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskTerminate.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskUpdateInteriorPoint.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskUpdateInteriorPoint.h
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskUpdateExternalDualBound.cpp
    ${PROJECT_SOURCE_DIR}/src/Tasks/TaskUpdateExternalDualBound.h
)

target_link_libraries(SHOTTasks SHOTPrimalStrategy SHOTBuildSettings)
target_link_libraries(SHOTTasks SHOTDualStrategy)

# Creates the solution strategies library
file(GLOB_RECURSE STRATEGIES_SOURCES "${PROJECT_SOURCE_DIR}/src/SolutionStrategy/*.cpp")
add_library(SHOTSolutionStrategies STATIC ${STRATEGIES_SOURCES})
target_link_libraries(SHOTSolutionStrategies SHOTTasks SHOTBuildSettings)

# Creates the modeling interfaces library
set(MODELING_SOURCES ${MODELING_SOURCES} ${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemOSiL.cpp)
add_library(SHOTModelingInterfaces STATIC ${MODELING_SOURCES})
target_link_libraries(SHOTModelingInterfaces SHOTModel SHOTBuildSettings)
if(HAS_AMPL)
    # Compile bundled AMPL code separately so it does not inherit SHOT warnings.
    # Embed its objects in the same archive to preserve whole-archive linking.
    add_library(SHOTAMPL OBJECT ${AMPL_SOURCES})
    target_include_directories(SHOTAMPL SYSTEM PRIVATE "${AMPL_DIR}/include")
    target_sources(SHOTModelingInterfaces PRIVATE $<TARGET_OBJECTS:SHOTAMPL>)
endif()


# Cplex linking
if(HAS_CPLEX)
    if(NOT (CPLEX_FOUND) OR NOT (CPLEX_CONCERT_FOUND) OR NOT (CPLEX_ILOCPLEX_FOUND))
        message("-- CPLEX libraries could not be found!")
    else()

        message("-- CPLEX include files will be used from:")
        message("   ${CPLEX_INCLUDE_DIR}")
        message("   ${CPLEX_CONCERT_INCLUDE_DIR}")
        message("   ${CPLEX_ILOCPLEX_INCLUDE_DIR}")

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
        target_link_libraries(SHOTDualStrategy CBC::CBC)

        message("-- The following Cbc libraries will be used from: ${CBC_LIBRARY_DIRS}")
        message("   ${CBC_LIBRARIES}")
    endif()
endif(HAS_CBC)

# HiGHS linking
if(HAS_HIGHS)
    message("-- HiGHS include files will be used from: ${HIGHS_INCLUDE_DIRS}")
    add_dependencies(SHOTDualStrategy highs::highs)
    target_link_libraries(SHOTDualStrategy highs::highs)
    add_dependencies(SHOTPrimalStrategy highs::highs)
    target_link_libraries(SHOTPrimalStrategy highs::highs)
endif(HAS_HIGHS)

# Ipopt linking
if(HAS_IPOPT)
    message("-- Ipopt include files will be used from: ${IPOPT_INCLUDE_DIRS}")
    target_link_libraries(SHOTPrimalStrategy PkgConfig::IPOPT)

    message("-- The following Ipopt libraries will be used from: ${IPOPT_LIBRARY_DIRS}")
    message("   ${IPOPT_LIBRARIES}")
endif(HAS_IPOPT)

# Uno linking
if(HAS_UNO)
    message("-- Uno include files will be used from: ${UNO_INCLUDE_DIR}")

    # The MUMPS libraries of Ipopt, e.g. Homebrew's, leave most of their symbols to be looked up in all loaded
    # libraries at runtime. A libuno with MUMPS linked into it exports its own MUMPS symbols, and since libuno is a
    # direct dependency it is searched before the MUMPS libraries Ipopt depends on, so Ipopt's MUMPS ended up calling
    # Uno's, which crashed in the analysis of large NLPs. Linking Ipopt's MUMPS libraries before libuno makes them be
    # found first. libuno binds its own MUMPS symbols directly, so Uno is not affected by the order.
    if(APPLE AND HAS_IPOPT)
        foreach(MUMPS_LIBRARY_NAME mumps_common mpiseq pord)
            find_library(IPOPT_${MUMPS_LIBRARY_NAME}_LIBRARY NAMES ${MUMPS_LIBRARY_NAME}
                PATHS ${IPOPT_LIBRARY_DIRS} NO_DEFAULT_PATH)

            if(IPOPT_${MUMPS_LIBRARY_NAME}_LIBRARY)
                target_link_libraries(SHOTPrimalStrategy ${IPOPT_${MUMPS_LIBRARY_NAME}_LIBRARY})
                message("-- Ipopt's MUMPS library is linked before Uno: ${IPOPT_${MUMPS_LIBRARY_NAME}_LIBRARY}")
            endif()
        endforeach()
    endif()

    target_link_libraries(SHOTPrimalStrategy ${UNO_LIBRARIES})
    target_include_directories(SHOTPrimalStrategy SYSTEM PUBLIC "${UNO_INCLUDE_DIR}")

    message("-- The following Uno libraries will be used:")
    message("   ${UNO_LIBRARIES}")
endif(HAS_UNO)

# GAMS linking
if(HAS_GAMS)
    message("-- GAMS include files will be used from: ${GAMS_DIR}/apifiles/C/api")
    target_compile_definitions(SHOTPrimalStrategy PRIVATE GC_NO_MUTEX)
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
target_link_libraries(SHOTSolver PRIVATE SHOTBuildSettings)
target_compile_features(SHOTSolver PUBLIC cxx_std_17)
target_include_directories(SHOTSolver PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/SHOT/thirdparty>)
add_dependencies(SHOTSolver cppad)

if(APPLE)
    set_target_properties(SHOTSolver PROPERTIES INSTALL_RPATH "@loader_path")
elseif(UNIX)
    set_target_properties(SHOTSolver PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

if(HAS_PYTHON)
    install(TARGETS SHOTSolver
        LIBRARY DESTINATION . COMPONENT Python
        RUNTIME DESTINATION . COMPONENT Python)
endif()

# Ensures class type checking works correctly between the executable and shared library
# Without this, the program can crash when checking object types at runtime
if(APPLE)
    # macOS needs this to avoid duplicate type information that breaks type checking
    # Linux handles this correctly by default
    target_link_options(SHOTSolver PRIVATE "-Wl,-flat_namespace")
elseif(UNIX)
    # Linux already uses flat namespace, but explicitly export symbols for robustness
    target_link_options(SHOTSolver PRIVATE "-Wl,--export-dynamic")
endif()

# List static libraries before their dependencies to avoid redundant archive
# entries (and duplicate-library warnings from Apple's linker).
target_link_libraries(SHOTSolver PRIVATE
    SHOTSolutionStrategies SHOTTasks SHOTDualStrategy SHOTPrimalStrategy SHOTResults)

if(MSYS OR MINGW)
    # MinGW cross compilation will not work if "-Wl,--whole-archive" is defined below
    target_link_libraries(SHOTSolver PRIVATE SHOTModelingInterfaces)
elseif(APPLE)
    # need these all-load flags to ensure that EntryPointGAMS symbols will be present in library
    target_link_libraries(SHOTSolver PRIVATE "-Wl,-all_load" SHOTModelingInterfaces)
else()
    # need these whole-archive flags to ensure that EntryPointGAMS symbols will be present in library
    target_link_libraries(SHOTSolver PRIVATE "-Wl,--whole-archive" SHOTModelingInterfaces "-Wl,--no-whole-archive")
endif()

target_link_libraries(SHOTSolver PRIVATE SHOTModel SHOTHelper)

# Link CppAD library
target_link_libraries(SHOTSolver PRIVATE ${CPPAD_LIBRARY})

# Link stdc++fs into SHOTSolver on toolchains that need it.
target_link_libraries(SHOTSolver PRIVATE CXX::Filesystem)

# Generates the SHOT executable
if(GENERATE_EXE)
    add_executable(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/src/SHOT.cpp")
    target_link_libraries(${PROJECT_NAME} SHOTSolver SHOTBuildSettings)

    # Keep build-tree solver libraries discoverable while using the installed
    # lib directory after installation.
    if(APPLE)
        set(RPATH_DIRS "${CPPAD_LIB_DIR}")
        
        if(HAS_GUROBI AND GUROBI_FOUND)
            list(APPEND RPATH_DIRS "${GUROBI_LIB_DIR}")
        endif()
        
        set_target_properties(${PROJECT_NAME} PROPERTIES
            BUILD_RPATH "${RPATH_DIRS}"
            INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
    elseif(UNIX)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
    endif()

    # Extra linking necessary for GAMS
    if(HAS_GAMS)
        if(UNIX)
            if(APPLE)
                # Don't link libstdc++ on macOS - Clang uses libc++ and mixing them causes ABI issues
                # target_link_libraries(${PROJECT_NAME} ${GAMS_DIR}/libstdc++.6.dylib)
            else(APPLE)
                target_link_libraries(${PROJECT_NAME} ${GAMS_DIR}/libstdc++.so.6)
            endif(APPLE)
        endif(UNIX)
    endif(HAS_GAMS)
endif(GENERATE_EXE)


# Install the command-line program, shared library, and CMake package.
include(CMakePackageConfigHelpers)
set(SHOT_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/SHOT")
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/misc/SHOTConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/SHOTConfig.cmake"
    INSTALL_DESTINATION "${SHOT_CMAKE_INSTALL_DIR}")
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/SHOTConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

if(GENERATE_EXE)
    install(
        TARGETS ${PROJECT_NAME}
        EXPORT SHOTTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime
    )
endif()

install(
    FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/AUTHORS
    ${CMAKE_CURRENT_SOURCE_DIR}/INSTALL
    ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
    ${CMAKE_CURRENT_SOURCE_DIR}/README
    DESTINATION "${CMAKE_INSTALL_DOCDIR}"
    COMPONENT Runtime
)

install(TARGETS SHOTSolver EXPORT SHOTTargets
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT Runtime
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT Development)

# CppAD is built with ExternalProject, so it has no install target in this
# CMake tree. Install the versioned library and its symlinks together.
install(DIRECTORY "${CPPAD_LIB_DIR}/"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    COMPONENT Runtime
    FILES_MATCHING
    PATTERN "libcppad_lib*.so*"
    PATTERN "libcppad_lib*.dylib")

# HiGHS' FAST_BUILD target does not install its shared library itself.
if(HAS_HIGHS AND NOT USE_EXTERNAL_HIGHS)
    install(TARGETS highs
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT Runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT Development)
endif()

install(EXPORT SHOTTargets
    FILE SHOTTargets.cmake
    NAMESPACE SHOT::
    DESTINATION "${SHOT_CMAKE_INSTALL_DIR}"
    COMPONENT Development)
install(FILES
    "${PROJECT_BINARY_DIR}/SHOTConfig.cmake"
    "${PROJECT_BINARY_DIR}/SHOTConfigVersion.cmake"
    DESTINATION "${SHOT_CMAKE_INSTALL_DIR}"
    COMPONENT Development)

# Public headers refer to these bundled header-only dependencies. The CppAD
# headers come from its build output so configure.hpp matches cppad_lib.
install(DIRECTORY "${PROJECT_SOURCE_DIR}/src/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT Development
    FILES_MATCHING PATTERN "*.h")
install(FILES "${PROJECT_BINARY_DIR}/SHOTConfig.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT Development)
install(DIRECTORY
    "${PROJECT_SOURCE_DIR}/${BOOST_DIR}/boost"
    "${PROJECT_SOURCE_DIR}/${EIGEN_DIR}/Eigen"
    "${PROJECT_SOURCE_DIR}/${EIGEN_DIR}/unsupported"
    "${PROJECT_SOURCE_DIR}/${SPDLOG_DIR}/include/spdlog"
    "${PROJECT_BINARY_DIR}/CppAD/include/cppad"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/SHOT/thirdparty"
    COMPONENT Development)
install(FILES "${PROJECT_SOURCE_DIR}/${TINYXML2_DIR}/tinyxml2.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/SHOT/thirdparty"
    COMPONENT Development)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/${MCPP_DIR}/include/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/SHOT/thirdparty"
    COMPONENT Development)

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
