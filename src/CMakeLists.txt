cmake_minimum_required (VERSION 2.8)
project (SHOTProject)

# Sets the release types, e.g. Release, Debug
#set(CMAKE_BUILD_TYPE Release)          

# Sets version number.
set (Tutorial_VERSION_MAJOR 0)
set (Tutorial_VERSION_MINOR 9)

# Activates subsolvers, note that corresponding libraries are needed
option (HAS_CPLEX "Is Cplex available" ON) 
option (HAS_GUROBI "Is Gurobi available" ON) 
option (HAS_GAMS "Is GAMS available" OFF)

# Sets the paths required. Edit these to suit your system:
set(COINOS_DIR /opt/OS-2.10-stable)
set(GUROBI_DIR /opt/gurobi751)
set(CPLEX_DIR /opt/ibm/ILOG/CPLEX_Studio128)
set(GAMS_DIR /home/aton3/alundell/Research/COIN-OR/gams24.8)

# May be needed if Boost is not automatically detected
#set(Boost_INCLUDE_DIR /usr/include)
#set(Boost_LIBRARY_DIR /usr/lib)

# Passes the parameters to the compiler
if(HAS_CPLEX) 
  add_definitions(-DHAS_CPLEX)
endif()

if(HAS_GUROBI)
  add_definitions(-DHAS_GUROBI)
endif()

if(HAS_GAMS)
  add_definitions(-DHAS_GAMS)
endif()

INCLUDE(FindPkgConfig)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
set(Boost_USE_STATIC_LIBS OFF)

if (HAS_CPLEX)  # To make CPLEX build
  add_definitions(-DIL_STD)
  add_definitions(-LDFLAGS="-Wl,--no-as-needed")
endif (HAS_CPLEX)

# Adding source files
file(GLOB_RECURSE Foo_SOURCES
  "./Iteration.cpp"
  "./ProcessInfo.cpp"
  "./SHOT.cpp"
  "./SHOTSolver.cpp"
  "./SolutionStrategy.cpp"
  "./TaskHandler.cpp"
  "./Timer.cpp"
  "./UtilityFunctions.cpp"
  "./ConstraintSelectionStrategy/*.cpp"
  "./LinesearchMethod/*.cpp"
  "./MIPSolver/IMIPSolutionLimitStrategy.cpp"
  "./MIPSolver/IMIPSolver.cpp"
  "./MIPSolver/IRelaxationStrategy.cpp"
  "./MIPSolver/MIPSolutionLimitStrategyAdaptive.cpp"
  "./MIPSolver/MIPSolutionLimitStrategyIncrease.cpp"
  "./MIPSolver/MIPSolutionLimitStrategyUnlimited.cpp"
  "./MIPSolver/MIPSolverBase.cpp"
  "./MIPSolver/MIPSolverOsiCbc.cpp"
  "./MIPSolver/RelaxationStrategyBase.cpp"
  "./MIPSolver/RelaxationStrategyNone.cpp"
  "./MIPSolver/RelaxationStrategyStandard.cpp"
  "./NLPSolver/INLPSolver.cpp"
  "./NLPSolver/NLPIpoptSolver.cpp"
  "./NLPSolver/NLPSolverBase.cpp"
  "./NLPSolver/NLPSolverCuttingPlaneMinimax.cpp"
  "./NLPSolver/NLPSolverCuttingPlaneRelaxed.cpp"
  "./NLPSolver/NLPSolverIpoptBase.cpp"
  "./NLPSolver/NLPSolverIpoptMinimax.cpp"
  "./NLPSolver/NLPSolverIpoptRelaxed.cpp"
  "./OptProblems/*.cpp"
  "./PointSelectionStrategy/*.cpp"
  "./PrimalSolutionStrategy/*.cpp"
  "./SolutionStrategy/*.cpp"
  "./Tasks/*.cpp"
  )

# Adding header files
file(GLOB_RECURSE Foo_HEADERS
  "./Enums.h"
  "./Iteration.h"
  "./ProcessInfo.h"
  "./SHOTSolver.h"
  "./SolutionStrategy.h"
  "./TaskHandler.h"
  "./Timer.h"
  "./UtilityFunctions.h"
  "./ConstraintSelectionStrategy/*.h"
  "./LinesearchMethod/*.h"
  "./MIPSolver/IMIPSolutionLimitStrategy.h"
  "./MIPSolver/IMIPSolver.h"
  "./MIPSolver/IRelaxationStrategy.h"
  "./MIPSolver/MIPSolutionLimitStrategyAdaptive.h"
  "./MIPSolver/MIPSolutionLimitStrategyIncrease.h"
  "./MIPSolver/MIPSolutionLimitStrategyUnlimited.h"
  "./MIPSolver/MIPSolverBase.h"
  "./MIPSolver/MIPSolverOsiCbc.h"
  "./MIPSolver/RelaxationStrategyBase.h"
  "./MIPSolver/RelaxationStrategyNone.h"
  "./MIPSolver/RelaxationStrategyStandard.h"
  "./NLPSolver/INLPSolver.h"
  "./NLPSolver/NLPIpoptSolver.h"
  "./NLPSolver/NLPSolverBase.h"
  "./NLPSolver/NLPSolverCuttingPlaneMinimax.h"
  "./NLPSolver/NLPSolverCuttingPlaneRelaxed.h"
  "./NLPSolver/NLPSolverIpoptBase.h"
  "./NLPSolver/NLPSolverIpoptMinimax.h"
  "./NLPSolver/NLPSolverIpoptRelaxed.h"
  "./OptProblems/*.h"
  "./PointSelectionStrategy/*.h"
  "./PrimalSolutionStrategy/*.h"
  "./SolutionStrategy/*.h"
  "./Tasks/*.h"
  )

if (HAS_CPLEX)  
  include_directories("${CPLEX_DIR}/cplex/include")
  include_directories("${CPLEX_DIR}/concert/include")
  set(Foo_SOURCES ${Foo_SOURCES} "./MIPSolver/MIPSolverCallbackBase.cpp")
  set(Foo_SOURCES ${Foo_SOURCES} "./MIPSolver/MIPSolverCplex.cpp")
  set(Foo_SOURCES ${Foo_SOURCES} "./MIPSolver/MIPSolverCplexLazy.cpp")
  set(Foo_SOURCES ${Foo_SOURCES} "./MIPSolver/MIPSolverCplexLazyOriginalCallback.cpp")
  set(Foo_HEADERS ${Foo_SOURCES} "./MIPSolver/MIPSolverCallbackBase.h")
  set(Foo_HEADERS ${Foo_SOURCES} "./MIPSolver/MIPSolverCplex.h")
  set(Foo_HEADERS ${Foo_SOURCES} "./MIPSolver/MIPSolverCplexLazy.h")
  set(Foo_HEADERS ${Foo_SOURCES} "./MIPSolver/MIPSolverCplexLazyOriginalCallback.h")
endif (HAS_CPLEX)

if (HAS_GUROBI)  
  include_directories("${GUROBI_DIR}/linux64/include")
  set(Foo_SOURCES ${Foo_SOURCES} "./MIPSolver/MIPSolverCallbackBase.cpp")
  set(Foo_SOURCES ${Foo_SOURCES} "./MIPSolver/MIPSolverGurobi.cpp")
  set(Foo_SOURCES ${Foo_SOURCES} "./MIPSolver/MIPSolverGurobiLazy.cpp")
  set(Foo_HEADERS ${Foo_SOURCES} "./MIPSolver/MIPSolverCallbackBase.h")
  set(Foo_HEADERS ${Foo_SOURCES} "./MIPSolver/MIPSolverGurobi.h")
  set(Foo_HEADERS ${Foo_SOURCES} "./MIPSolver/MIPSolverGurobiLazy.h")
endif (HAS_GUROBI)

if (HAS_GAMS)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGAMSDIR=\\\"${GAMS_DIR}\\\"")
  set(Foo_SOURCES ${Foo_SOURCES} "$./NLPSolver/NLPSolverGAMS.cpp")
  set(Foo_HEADERS ${Foo_HEADERS} "$./NLPSolver/NLPSolverGAMS.h")
  set(Foo_SOURCES ${Foo_SOURCES} "${GAMS_DIR}/apifiles/C/api/gmomcc.c")
  set(Foo_SOURCES ${Foo_SOURCES} "./GAMS/GAMS2OS.cpp")
  set(Foo_HEADERS ${Foo_HEADERS} "./GAMS/GAMS2OS.h")
  set(Foo_HEADERS ${Foo_HEADERS} "./GAMS/GamsNLinstr.h")
  include_directories("${GAMS_DIR}/apifiles/C/api")
endif (HAS_GAMS)

# Finding out link directories
set (Foo_INCLUDE_DIRS "")
foreach (_headerFile ${Foo_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND Foo_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES Foo_INCLUDE_DIRS)

include_directories(${Foo_INCLUDE_DIRS})

find_package(Boost 1.63.0 COMPONENTS system filesystem REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
LINK_DIRECTORIES(${Boost_LIBRARY_DIR})

include_directories("${COINOS_DIR}/include/coin")
include_directories("${COINOS_DIR}/include/cppad")
include_directories("${COINOS_DIR}/include")

include_directories ("${PROJECT_SOURCE_DIR}/SHOTSettings")
add_subdirectory (SHOTSettings)

add_executable (SHOT ${Foo_SOURCES})

target_link_libraries (SHOT SHOTSettings)

if (HAS_CPLEX)  
  target_link_libraries (SHOT ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libilocplex.a)
  target_link_libraries (SHOT ${CPLEX_DIR}/concert/lib/x86-64_linux/static_pic/libconcert.a)
  target_link_libraries (SHOT ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libcplex.a)
endif (HAS_CPLEX)

if (HAS_GUROBI)  
  target_link_libraries (SHOT ${GUROBI_DIR}/linux64/lib/libgurobi75.so)
  target_link_libraries (SHOT ${GUROBI_DIR}/linux64/lib/libgurobi_g++5.2.a)
endif (HAS_GUROBI)

target_link_libraries (SHOT pthread)
target_link_libraries (SHOT m)
target_link_libraries (SHOT bz2)
target_link_libraries (SHOT z)
target_link_libraries (SHOT lapack)
target_link_libraries (SHOT blas)
target_link_libraries (SHOT dl)
target_link_libraries (SHOT gomp)

if (HAS_GAMS)  
  target_link_libraries (SHOT ${GAMS_DIR}/libstdc++.so.6)
endif (HAS_GAMS)

target_link_libraries (SHOT ${COINOS_DIR}/lib/libcoinasl.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libcoinmumps.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libcoinmetis.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libipopt.so)
target_link_libraries (SHOT ${COINOS_DIR}/lib/libipoptamplinterface.so)
target_link_libraries (SHOT ${COINOS_DIR}/lib/libCoinUtils.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libClp.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libOsi.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libOsiClp.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libCgl.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libCbc.so.3 )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libCbcSolver.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libOS.so )

target_link_libraries(SHOT ${Boost_LIBRARIES})

install(TARGETS SHOT DESTINATION bin)