
cmake_minimum_required (VERSION 2.8)
project (SHOTProject)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(Boost_USE_STATIC_LIBS OFF)
set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
#set(CMAKE_BUILD_TYPE Debug)

INCLUDE(FindPkgConfig)

# To make CPLEX build
add_definitions(-DIL_STD)
add_definitions(-LDFLAGS="-Wl,--no-as-needed")

file(GLOB_RECURSE Foo_SOURCES
  "GAMS2OS.cpp"
  "Iteration.cpp"
  "ProcessInfo.cpp"
  "SHOT.cpp"
  "SHOTSolver.cpp"
  "SolutionStrategy.cpp"
  "TaskHandler.cpp"
  "Timer.cpp"
  "UtilityFunctions.cpp"
  "./ConstraintSelectionStrategy/*.cpp"
  "./LinesearchMethod/*.cpp"
  "./MILPSolver/*.cpp"
  "./NLPSolver/*.cpp"
  "./OptProblems/*.cpp"
  "./PointSelectionStrategy/*.cpp"
  "./PrimalSolutionStrategy/*.cpp"
  "./SolutionStrategy/*.cpp"
  "./Tasks/*.cpp"
  )

#file(GLOB_RECURSE Foo_HEADERS "*.h")
file(GLOB_RECURSE Foo_HEADERS
  "Enums.h"
  "GAMS2OS.h"
  "GamsNLinstr.h"
  "Iteration.h"
  "ProcessInfo.h"
  "SHOTSolver.h"
  "SolutionStrategy.h"
  "TaskHandler.h"
  "Timer.h"
  "UtilityFunctions.h"
  "./ConstraintSelectionStrategy/*.h"
  "./LinesearchMethod/*.h"
  "./MILPSolver/*.h"
  "./NLPSolver/*.h"
  "./OptProblems/*.h"
  "./PointSelectionStrategy/*.h"
  "./PrimalSolutionStrategy/*.h"
  "./SolutionStrategy/*.h"
  "./Tasks/*.h"
  )

set (Foo_INCLUDE_DIRS "")
foreach (_headerFile ${Foo_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND Foo_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES Foo_INCLUDE_DIRS)

set(Boost_INCLUDE_DIR /usr/include)
set(Boost_LIBRARY_DIR /usr/lib)
#set(COINOS_DIR /opt/OS-2.10.1)
set(COINOS_DIR /opt/OS-2.10-andreasfix)
#set(COINOS_DIR /opt/OS-2.10.1-pre_stefan_fix)
#set(COINOS_DIR /opt/OS-2.10-stefanfix)
find_package(Boost 1.63.0 COMPONENTS system filesystem REQUIRED)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
LINK_DIRECTORIES(${Boost_LIBRARY_DIR})

include_directories(${Foo_INCLUDE_DIRS})

#include_directories("." "..")

set(GUROBI_DIR /opt/gurobi751)
set(CPLEX_DIR /opt/ibm/ILOG/CPLEX_Studio1271)

include_directories("${COINOS_DIR}/include/coin")
include_directories("${COINOS_DIR}/include/cppad")
include_directories("${COINOS_DIR}/include")

include_directories("${GUROBI_DIR}/linux64/include")
include_directories("${CPLEX_DIR}/cplex/include")
include_directories("${CPLEX_DIR}/concert/include")

set(GAMS_DIR /home/aton3/alundell/Research/COIN-OR/gams24.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGAMSDIR=\\\"${GAMS_DIR}\\\"")
set(Foo_SOURCES ${Foo_SOURCES} "${GAMS_DIR}/apifiles/C/api/gmomcc.c")
set(Foo_SOURCES ${Foo_SOURCES} "${GAMS_DIR}/apifiles/C/api/gevmcc.c")
include_directories("${GAMS_DIR}/apifiles/C/api")


include_directories ("${PROJECT_SOURCE_DIR}/SHOTSettings")
add_subdirectory (SHOTSettings)

add_executable (SHOT ${Foo_SOURCES})

#LINK_DIRECTORIES(${COINOS_DIR}/lib)

target_link_libraries (SHOT SHOTSettings)
target_link_libraries (SHOT ${GUROBI_DIR}/linux64/lib/libgurobi75.so)
target_link_libraries (SHOT ${GUROBI_DIR}/linux64/lib/libgurobi_g++5.2.a)
target_link_libraries (SHOT ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libilocplex.a)
target_link_libraries (SHOT ${CPLEX_DIR}/concert/lib/x86-64_linux/static_pic/libconcert.a)
target_link_libraries (SHOT ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libcplex.a)
target_link_libraries (SHOT pthread)
target_link_libraries (SHOT m)
target_link_libraries (SHOT bz2)
target_link_libraries (SHOT z)
target_link_libraries (SHOT lapack)
target_link_libraries (SHOT blas)
target_link_libraries (SHOT dl)
target_link_libraries (SHOT gomp)
#target_link_libraries (SHOT ${GAMS_DIR}/libstdc++.so.6)

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
target_link_libraries (SHOT ${COINOS_DIR}/lib/libCbc.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libCbcSolver.so )
target_link_libraries (SHOT ${COINOS_DIR}/lib/libOS.so )

target_link_libraries(SHOT ${Boost_LIBRARIES})

#install(TARGETS SHOT DESTINATION bin)
