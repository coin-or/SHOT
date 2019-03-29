cmake_minimum_required(VERSION 3.5)

# For options depending on other options
include(CMakeDependentOption)

# For ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()

project(SHOT)
set(CMAKE_CXX_STANDARD 17 )

# Sets the release types, e.g. Release, Debug
# set(CMAKE_BUILD_TYPE Debug)

option(COMPILE_TESTS "Should the automated tests be compiled" ON)

# Activates extra functionality, note that corresponding libraries are needed

# GAMS
option(HAS_GAMS "Is GAMS available" ON)
set(GAMS_DIR "/opt/gams/gams26.1"  CACHE STRING "The base directory where GAMS is located (if available)")

# COIN-OR Optimization Services
option(HAS_OS "Is COIN-OR Optimization Services (OS) available" ON)
set(OS_DIR "/opt/OS-2.10" CACHE STRING "The base directory where COIN-OR Optimization Services is located (if available)")

# CPLEX
option(HAS_CPLEX "Is Cplex available" ON)
cmake_dependent_option(HAS_CPLEX_NEW_CALLBACK "Is Cplex version >12.8 so the new callbacks are available?" ON "HAS_CPLEX" OFF)
set(CPLEX_DIR "/opt/ibm/ILOG/CPLEX_Studio128" CACHE STRING "The base directory where CPLEX is located (if available)")

# Gurobi
option(HAS_GUROBI "Is Gurobi available" ON)
set(GUROBI_DIR "/opt/gurobi/gurobi810" CACHE STRING "The base directory where Gurobi is located (if available)")

# Cbc
option(HAS_CBC "Is Cbc available" ON)
set(CBC_DIR "/opt/Cbc" CACHE STRING "The base directory where Cbc is located (if available), if OS provides CBC use the same value as OS_DIR")

# Ipopt
option(HAS_IPOPT "Is Ipopt available" ON)
set(IPOPT_DIR "/opt/ipopt" CACHE STRING "The base directory where Ipopt is located (if available), if OS provides Ipopt use the same value as OS_DIR")

# Create also the executable
option(GENERATE_EXE "Should the SHOT executable be generated (requires at least that either OS or GAMS is available)" ON)

# External libraries; these are included in SHOT's source tree, and should not need to be modified
set(MCPP_DIR "ThirdParty/mc++")
set(SPDLOG_DIR "ThirdParty/spdlog")
set(TINYXML2_DIR "ThirdParty/tinyxml2")

# May be needed to specify the path to where Boost is installed, if not automatically detected
#set(Boost_INCLUDE_DIR /usr/include)
#set(Boost_LIBRARY_DIR /usr/lib)

# Some checks on the provided parameters

# Check if a MIP solver is defined
if(NOT(HAS_CPLEX OR HAS_GUROBI OR HAS_CBC))
  message( FATAL_ERROR "No MIP solver defined. SHOT needs at least one!")
endif()

# Check if a modeling system is defined if we want to build the executable
if(GENERATE_EXE AND NOT(HAS_GAMS OR HAS_OS))
  message( FATAL_ERROR "No modeling system defined which is required if  you want to build an executable!")
endif()

# Sets version number
set(SHOT_VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(SHOT_VERSION_MINOR   9   CACHE STRING "Project minor version number.")
set(SHOT_VERSION_PATCH   2   CACHE STRING "Project patch version number.")
mark_as_advanced(SHOT_VERSION_MAJOR SHOT_VERSION_MINOR SHOT_VERSION_PATCH)
configure_file("${PROJECT_SOURCE_DIR}/src/SHOTConfig.h.in" "${PROJECT_BINARY_DIR}/SHOTConfig.h")
include_directories("${PROJECT_BINARY_DIR}")

# Searches for PkgConfig
find_package(PkgConfig REQUIRED)

# Passes the parameters to the compiler

if(HAS_GAMS)
  add_definitions(-DHAS_GAMS)
endif(HAS_GAMS)

if(HAS_OS)
  set(ENV{PKG_CONFIG_PATH} "${OS_DIR}/lib/pkgconfig:${OS_DIR}/share/pkgconfig")
  pkg_search_module(OS REQUIRED os)
  link_directories(${OS_LIBRARY_DIRS})
  include_directories(${OS_INCLUDE_DIRS})
  add_definitions(-DHAS_OS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register") # To remove some warnings in OS
endif(HAS_OS)

# MIP solvers

if(HAS_CBC)
  set(ENV{PKG_CONFIG_PATH} "${CBC_DIR}/lib/pkgconfig")
  pkg_search_module(CBC REQUIRED cbc)
  link_directories(${CBC_LIBRARY_DIRS})
  include_directories(${CBC_INCLUDE_DIRS})
  add_definitions(-DHAS_CBC)
endif(HAS_CBC)

if(HAS_CPLEX)
  add_definitions(-DIL_STD)
  add_definitions(-LDFLAGS="-Wl,--no-as-needed")

  add_definitions(-DHAS_CPLEX)
  if(HAS_CPLEX_NEW_CALLBACK)
      add_definitions(-DHAS_CPLEX_NEW_CALLBACK)
  endif(HAS_CPLEX_NEW_CALLBACK)
endif(HAS_CPLEX)

if(HAS_GUROBI)
  # Note that you also may have to set the correct names of the Gurobi libraries below
  add_definitions(-DHAS_GUROBI)
endif(HAS_GUROBI)

# NLP solvers

if(HAS_IPOPT)
  set(ENV{PKG_CONFIG_PATH} "${IPOPT_DIR}/lib/pkgconfig") 
  pkg_search_module(IPOPT REQUIRED ipopt)
  link_directories(${IPOPT_LIBRARY_DIRS})
  include_directories(${IPOPT_INCLUDE_DIRS})
  add_definitions(-DHAS_IPOPT)
endif(HAS_IPOPT)


# Normally nothing should need to be modified below this point

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")
set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
set(Boost_USE_STATIC_LIBS OFF)

include(FindPkgConfig)

# Adding source files
file(GLOB_RECURSE SOURCES
  "${PROJECT_SOURCE_DIR}/src/Model/ObjectiveFunction.cpp"
  "${PROJECT_SOURCE_DIR}/src/Model/Constraints.cpp"
  "${PROJECT_SOURCE_DIR}/src/Model/Problem.cpp"
  "${PROJECT_SOURCE_DIR}/src/Report.cpp"
  "${PROJECT_SOURCE_DIR}/src/Iteration.cpp"
  "${PROJECT_SOURCE_DIR}/src/Output.cpp"
  "${PROJECT_SOURCE_DIR}/src/DualSolver.cpp"
  "${PROJECT_SOURCE_DIR}/src/PrimalSolver.cpp"
  "${PROJECT_SOURCE_DIR}/src/Results.cpp"
  "${PROJECT_SOURCE_DIR}/src/Solver.cpp"
  "${PROJECT_SOURCE_DIR}/src/TaskHandler.cpp"
  "${PROJECT_SOURCE_DIR}/src/Utilities.cpp"
  "${PROJECT_SOURCE_DIR}/src/Simplifications.cpp"
  "${PROJECT_SOURCE_DIR}/src/LinesearchMethod/LinesearchMethodBoost.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/IMIPSolutionLimitStrategy.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyAdaptive.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyIncrease.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyUnlimited.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverBase.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyBase.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyNone.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyStandard.cpp"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverBase.cpp"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverCuttingPlaneMinimax.cpp"
  "${PROJECT_SOURCE_DIR}/src/SolutionStrategy/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/Tasks/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/Settings.cpp"
  "${TINYXML2_DIR}/tinyxml2.cpp"
  )

# Adding header files
file(GLOB_RECURSE HEADERS
  "${PROJECT_SOURCE_DIR}/src/Shared.h"
  "${PROJECT_SOURCE_DIR}/src/Enums.h"
  "${PROJECT_SOURCE_DIR}/src/Structs.h"
  "${PROJECT_SOURCE_DIR}/src/Environment.h"
  "${PROJECT_SOURCE_DIR}/src/EventHandler.h"
  "${PROJECT_SOURCE_DIR}/src/Model/ModelShared.h"
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
  "${PROJECT_SOURCE_DIR}/src/ConstraintSelectionStrategy/*.h"
  "${PROJECT_SOURCE_DIR}/src/LinesearchMethod/ILinesearchMethod.h"
  "${PROJECT_SOURCE_DIR}/src/LinesearchMethod/LinesearchMethodBoost.h"
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
  "${PROJECT_SOURCE_DIR}/src/SolutionStrategy/*.h"
  "${PROJECT_SOURCE_DIR}/src/Tasks/*.h"
  "${PROJECT_SOURCE_DIR}/src/Settings.h"
  "${TINYXML2_DIR}/tinyxml2.h"
  )

if(HAS_CBC)
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverOsiCbc.cpp")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverOsiCbc.h")
endif(HAS_CBC)

if(HAS_CPLEX)
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplex.cpp")

  if(HAS_CPLEX_NEW_CALLBACK)
    set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazy.cpp")
    set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazy.h")
  endif(HAS_CPLEX_NEW_CALLBACK)

  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazyOriginalCallback.cpp")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplex.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazyOriginalCallback.h")
endif(HAS_CPLEX)

if(HAS_GUROBI)
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiLazy.cpp")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiLazy.h")
endif(HAS_GUROBI)

if(HAS_IPOPT)
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.cpp")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.h")
endif(HAS_IPOPT)

if(HAS_GAMS)
  set(SOURCES ${SOURCES} "${GAMS_DIR}/apifiles/C/api/gmomcc.c")
  set(SOURCES ${SOURCES} "${GAMS_DIR}/apifiles/C/api/gevmcc.c")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemGAMS.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/EntryPointsGAMS.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverGAMS.cpp")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemGAMS.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverGAMS.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/GAMS/GamsNLinstr.h")
endif(HAS_GAMS)

if(HAS_OS)
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemOS.cpp")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemOS.h")
endif(HAS_OS)

# Finding out link directories
set(INCLUDE_DIRS "")
foreach(_headerFile ${HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

include_directories(${INCLUDE_DIRS})

find_package(Boost 1.63.0 COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

include_directories("${MCPP_DIR}/include")
include_directories("${MCPP_DIR}/3rdparty/cpplapack/include")
include_directories("${MCPP_DIR}/3rdparty/fadbad++")
include_directories("${SPDLOG_DIR}")
include_directories("${TINYXML2_DIR}/")

# Creates the SHOT library that is linked to the executable
add_library(SHOTSolver SHARED ${SOURCES})

if(HAS_CPLEX)
  include_directories("${CPLEX_DIR}/cplex/include")
  include_directories("${CPLEX_DIR}/concert/include")
  target_link_libraries(SHOTSolver ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libilocplex.a)
  target_link_libraries(SHOTSolver ${CPLEX_DIR}/concert/lib/x86-64_linux/static_pic/libconcert.a)
  target_link_libraries(SHOTSolver ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libcplex.a)
endif(HAS_CPLEX)

if(HAS_GUROBI)
  include_directories("${GUROBI_DIR}/linux64/include")
  # Change the Gurobi library below to match the version on the system
  target_link_libraries(SHOTSolver ${GUROBI_DIR}/linux64/lib/libgurobi81.so)
  target_link_libraries(SHOTSolver ${GUROBI_DIR}/linux64/lib/libgurobi_g++5.2.a)
endif(HAS_GUROBI)

if(HAS_CBC)
  target_link_libraries(SHOTSolver ${CBC_LIBRARIES})
  target_include_directories(SHOTSolver PUBLIC "${CBC_DIR}/include/coin")
  target_compile_options(SHOTSolver PUBLIC ${CBC_CFLAGS_OTHER})
  include_directories(${CBC_INCLUDE_DIRS})
endif(HAS_CBC)

if(HAS_IPOPT)
  target_link_libraries(SHOTSolver ${IPOPT_LIBRARIES})
  target_include_directories(SHOTSolver PUBLIC "${IPOPT_DIR}/include/coin")
  target_compile_options(SHOTSolver PUBLIC ${IPOPT_CFLAGS_OTHER})
endif(HAS_IPOPT)

if(HAS_GAMS)
  include_directories("${GAMS_DIR}/apifiles/C/api")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGAMSDIR=\\\"${GAMS_DIR}\\\"")
endif(HAS_GAMS)

if(HAS_OS)
  target_link_libraries(SHOTSolver ${OS_LIBRARIES})
  target_include_directories(SHOTSolver PUBLIC "${OS_DIR}/include/coin")
  target_compile_options(SHOTSolver PUBLIC ${OS_CFLAGS_OTHER})
endif(HAS_OS)

# Generates the SHOT executable
if(GENERATE_EXE)
  add_executable(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/src/SHOT.cpp")
  target_link_libraries(${PROJECT_NAME} SHOTSolver)
endif(GENERATE_EXE)

if(HAS_GAMS)
  if(UNIX)
    if(APPLE)
      target_link_libraries(${PROJECT_NAME} ${GAMS_DIR}/libstdc++.6.dylib)
    else(APPLE)
      target_link_libraries(${PROJECT_NAME} ${GAMS_DIR}/libstdc++.so.6)
    endif(APPLE)
  endif(UNIX)
endif(HAS_GAMS)

target_link_libraries(SHOTSolver ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# Add a target to generate API documentation with Doxygen
find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation(requires Doxygen)" ${DOXYGEN_FOUND})

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doxydoc/doxygen.conf.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
endif()

if(COMPILE_TESTS)
    # For performing automated tests
    enable_testing()
    add_subdirectory("${PROJECT_SOURCE_DIR}/test")
endif()