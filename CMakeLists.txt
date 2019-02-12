cmake_minimum_required (VERSION 2.8)
project (SHOT)
set(CMAKE_CXX_STANDARD 17 )

# Sets the release types, e.g. Release, Debug
set(CMAKE_BUILD_TYPE Debug)

option (COMPILE_TESTS "Should the automated tests be compiled" ON)

# Sets version number.
set(SHOT_VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(SHOT_VERSION_MINOR   9   CACHE STRING "Project minor version number.")
set(SHOT_VERSION_PATCH   2   CACHE STRING "Project patch version number.")
mark_as_advanced(SHOT_VERSION_MAJOR SHOT_VERSION_MINOR SHOT_VERSION_PATCH)

# Activates extra functionality, note that corresponding libraries are needed
option (HAS_CPLEX "Is Cplex available" ON)
option (HAS_CPLEX_NEW_CALLBACK "Is Cplex version >12.8 so the new callbacks are available?" ON)
option (HAS_GUROBI "Is Gurobi available" OFF)
option (HAS_GAMS "Is GAMS available" ON)
option (HAS_OS "Is COIN-OR Optimization Services available" ON)

# Sets the paths required. Edit these to suit your system:
set(COINOS_DIR "/opt/OS-2.10" CACHE STRING "The base directory where COIN-OR Optimization Services is located (if available)")
set(GUROBI_DIR "/opt/gurobi/gurobi810" CACHE STRING "The base directory where Gurobi is located (if available)")
set(CPLEX_DIR "/opt/ibm/ILOG/CPLEX_Studio128" CACHE STRING "The base directory where CPLEX is located (if available)")
set(GAMS_DIR "/opt/gams/gams25.1.2"  CACHE STRING "The base directory where GAMS is located (if available)")

# These are included in SHOT's the source tree, and should not need to be modified
set(MCPP_DIR ThirdParty/mc++)
set(SPDLOG_DIR ThirdParty/spdlog)
set(TINYXML2_DIR ThirdParty/tinyxml2)

# May be needed to specify the path to where Boost is installed, if not automatically detected
#set(Boost_INCLUDE_DIR /usr/include)
#set(Boost_LIBRARY_DIR /usr/lib)

# Passes the parameters to the compiler
if(HAS_CPLEX)
  add_definitions(-DHAS_CPLEX)

  # To make Cplex build
  add_definitions(-DIL_STD)
  add_definitions(-LDFLAGS="-Wl,--no-as-needed")

    if(HAS_CPLEX_NEW_CALLBACK)
        add_definitions(-DHAS_CPLEX_NEW_CALLBACK)
    endif()
endif()

if(HAS_GUROBI)
  add_definitions(-DHAS_GUROBI)
endif()

if(HAS_GAMS)
  add_definitions(-DHAS_GAMS)
endif()

if(HAS_OS)
  add_definitions(-DHAS_OS)
endif()

INCLUDE(FindPkgConfig)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")

if(HAS_CPLEX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register") # To remove some warnings in OS
endif()

set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
set(Boost_USE_STATIC_LIBS OFF)

# For configuring version numbering
configure_file ("${PROJECT_SOURCE_DIR}/src/SHOTConfig.h.in" "${PROJECT_BINARY_DIR}/SHOTConfig.h")
include_directories("${PROJECT_BINARY_DIR}")

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
  "${PROJECT_SOURCE_DIR}/src/SHOTSolver.cpp"
  "${PROJECT_SOURCE_DIR}/src/TaskHandler.cpp"
  "${PROJECT_SOURCE_DIR}/src/UtilityFunctions.cpp"
  "${PROJECT_SOURCE_DIR}/src/Simplifications.cpp"
  "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemOS.cpp"
  "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemGAMS.cpp"
  "${PROJECT_SOURCE_DIR}/src/ModelingSystem/EntryPointsGAMS.cpp"
  "${PROJECT_SOURCE_DIR}/src/LinesearchMethod/LinesearchMethodBoost.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/IMIPSolutionLimitStrategy.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyAdaptive.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyIncrease.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolutionLimitStrategyUnlimited.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverBase.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverOsiCbc.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyBase.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyNone.cpp"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyStandard.cpp"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPIpoptSolver.cpp"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverBase.cpp"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverCuttingPlaneMinimax.cpp"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.cpp"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.cpp"
  "${PROJECT_SOURCE_DIR}/src/PrimalSolutionStrategy/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/SolutionStrategy/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/Tasks/*.cpp"
  "${TINYXML2_DIR}/tinyxml2.cpp"
  )

# Adding header files
file(GLOB_RECURSE HEADERS
  "${PROJECT_SOURCE_DIR}/src/Shared.h"
  "${PROJECT_SOURCE_DIR}/src/Enums.h"
  "${PROJECT_SOURCE_DIR}/src/Structs.h"
  "${PROJECT_SOURCE_DIR}/src/Environment.h"
  "${PROJECT_SOURCE_DIR}/src/Model/ModelShared.h"
  "${PROJECT_SOURCE_DIR}/src/Model/Variables.h"
  "${PROJECT_SOURCE_DIR}/src/Model/Terms.h"
  "${PROJECT_SOURCE_DIR}/src/Model/AuxilliaryVariables.h"
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
  "${PROJECT_SOURCE_DIR}/src/SHOTSolver.h"
  "${PROJECT_SOURCE_DIR}/src/TaskHandler.h"
  "${PROJECT_SOURCE_DIR}/src/UtilityFunctions.h"
  "${PROJECT_SOURCE_DIR}/src/Simplifications.h"
  "${PROJECT_SOURCE_DIR}/src/ModelingSystem/IModelingSystem.h"  
  "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemOS.h" 
  "${PROJECT_SOURCE_DIR}/src/ModelingSystem/ModelingSystemGAMS.h"
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
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverOsiCbc.h"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyBase.h"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyNone.h"
  "${PROJECT_SOURCE_DIR}/src/MIPSolver/RelaxationStrategyStandard.h"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/INLPSolver.h"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPIpoptSolver.h"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverBase.h"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverCuttingPlaneMinimax.h"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptBase.h"
  "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverIpoptRelaxed.h"
  "${PROJECT_SOURCE_DIR}/src/PrimalSolutionStrategy/*.h"
  "${PROJECT_SOURCE_DIR}/src/SolutionStrategy/*.h"
  "${PROJECT_SOURCE_DIR}/src/Tasks/*.h"
  "${TINYXML2_DIR}/tinyxml2.h"
  )

if (HAS_CPLEX)
  include_directories("${CPLEX_DIR}/cplex/include")
  include_directories("${CPLEX_DIR}/concert/include")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplex.cpp")

  if (HAS_CPLEX_NEW_CALLBACK)
    set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazy.cpp")
    set(HEADERS ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazy.h")
  endif (HAS_CPLEX_NEW_CALLBACK)

  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazyOriginalCallback.cpp")
  set(HEADERS ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
  set(HEADERS ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplex.h")
  set(HEADERS ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCplexLazyOriginalCallback.h")
endif (HAS_CPLEX)

if (HAS_GUROBI)
  include_directories("${GUROBI_DIR}/linux64/include")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.cpp")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiLazy.cpp")
  set(HEADERS ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverCallbackBase.h")
  set(HEADERS ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobi.h")
  set(HEADERS ${SOURCES} "${PROJECT_SOURCE_DIR}/src/MIPSolver/MIPSolverGurobiLazy.h")
endif (HAS_GUROBI)

if (HAS_GAMS)
  include_directories("${GAMS_DIR}/apifiles/C/api")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGAMSDIR=\\\"${GAMS_DIR}\\\"")
  set(SOURCES ${SOURCES} "${GAMS_DIR}/apifiles/C/api/gmomcc.c")
  set(SOURCES ${SOURCES} "${GAMS_DIR}/apifiles/C/api/gevmcc.c")
  set(SOURCES ${SOURCES} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverGAMS.cpp")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/NLPSolver/NLPSolverGAMS.h")
  set(HEADERS ${HEADERS} "${PROJECT_SOURCE_DIR}/src/GAMS/GamsNLinstr.h")
endif (HAS_GAMS)

# Finding out link directories
set (INCLUDE_DIRS "")
foreach (_headerFile ${HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

include_directories(${INCLUDE_DIRS})

find_package(Boost 1.63.0 COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

if(HAS_OS)
    include_directories("${COINOS_DIR}/include/coin")
    include_directories("${COINOS_DIR}/include/cppad")
    include_directories("${COINOS_DIR}/include")
endif()

include_directories("${MCPP_DIR}/include")
include_directories("${MCPP_DIR}/3rdparty/cpplapack/include")
include_directories("${MCPP_DIR}/3rdparty/fadbad++")

include_directories("${SPDLOG_DIR}")
include_directories("${TINYXML2_DIR}/")

include_directories ("${PROJECT_SOURCE_DIR}/src/SHOTSettings")
add_subdirectory ("${PROJECT_SOURCE_DIR}/src/SHOTSettings")

# got an undefined symbol _ZN4SHOT8Settings13updateSettingENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_i if not linking SHOTSettings into SHOTSolver
add_library (SHOTSolver SHARED ${SOURCES})
target_link_libraries (SHOTSolver SHOTSettings)

add_executable (${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/src/SHOT.cpp")
target_link_libraries (${PROJECT_NAME} SHOTSolver)

if (HAS_CPLEX)
  target_link_libraries (SHOTSolver ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libilocplex.a)
  target_link_libraries (SHOTSolver ${CPLEX_DIR}/concert/lib/x86-64_linux/static_pic/libconcert.a)
  target_link_libraries (SHOTSolver ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libcplex.a)
endif (HAS_CPLEX)

if (HAS_GUROBI)
  # Change the Gurobi library below to match the version on the system
  target_link_libraries (SHOTSolver ${GUROBI_DIR}/linux64/lib/libgurobi81.so)
  target_link_libraries (SHOTSolver ${GUROBI_DIR}/linux64/lib/libgurobi_g++5.2.a)
endif (HAS_GUROBI)

target_link_libraries (${PROJECT_NAME} pthread)
target_link_libraries (${PROJECT_NAME} m)
target_link_libraries (${PROJECT_NAME} bz2)
target_link_libraries (${PROJECT_NAME} z)
target_link_libraries (${PROJECT_NAME} lapack)
target_link_libraries (${PROJECT_NAME} blas)
target_link_libraries (${PROJECT_NAME} dl)
target_link_libraries (${PROJECT_NAME} gomp)

if (HAS_GAMS)
  target_link_libraries (${PROJECT_NAME} ${GAMS_DIR}/libstdc++.so.6)
endif (HAS_GAMS)

if(HAS_OS)
    target_link_libraries (SHOTSolver ${COINOS_DIR}/lib/libOS.so )
endif()

target_link_libraries(SHOTSolver ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# Add a target to generate API documentation with Doxygen
find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

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
    add_subdirectory ("${PROJECT_SOURCE_DIR}/test")
endif()