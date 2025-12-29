# The name of the test executable file
set(TEST_EXE_NAME test_runner)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)

# The main groups of tests, there should be a file matching the name +"test".cpp
# in the test directory
set(cpptests Model Settings)

set(Model_parts
    1
    2
    3
    4
    5
    6
    7
    8
    9
    10) # The different parts of each test (if any)
set(Settings_parts 1 2)

if(HAS_CBC)
  set(Cbc_parts 1 2 3 4 5 6 7)
  set(cpptests ${cpptests} Cbc)
endif()

if(HAS_CPLEX)
  set(Cplex_parts 1 2 3 4 5 6 7)
  set(cpptests ${cpptests} Cplex)
endif()

if(HAS_GUROBI)
  set(Gurobi_parts 1 2 3 4 5 6 7)
  set(cpptests ${cpptests} Gurobi)
endif()

if(HAS_GAMS)
  set(GAMS_parts
      1
      2
      3
      4
      5
      6
      7
      8
      9
      10
      11)
  set(cpptests ${cpptests} GAMS)
endif()

set(Solver_parts
    1
    2
    3
    4
    5
    6)
set(cpptests ${cpptests} Solver)

if(HAS_IPOPT)
  set(cpptests ${cpptests} Ipopt)
  set(Ipopt_parts 1 2)
endif()

# Adds a "1" to tests without parts
foreach(cpptest ${cpptests})
  if(NOT DEFINED "${cpptest}_parts")
    set(${cpptest}_parts "1")
  endif()
endforeach()

# Adds the source files for the tests
foreach(cpptest ${cpptests})
  set(cpptestsrc ${cpptestsrc} ${cpptest}Test.cpp)
endforeach()

set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
set(Boost_USE_STATIC_LIBS OFF)

if(HAS_CPLEX) # To make CPLEX build
  add_definitions(-DIL_STD)
  add_definitions(-LDFLAGS="-Wl,--no-as-needed")
endif(HAS_CPLEX)

# Creates the test executable
create_test_sourcelist(srclist test_runner.cpp ${cpptestsrc})
add_executable(${TEST_EXE_NAME} ${srclist})
target_link_libraries(${TEST_EXE_NAME} ${libs})

target_link_libraries(${TEST_EXE_NAME} SHOTSolver)

if(HAS_CPLEX)
  if(NOT
     (CPLEX_FOUND)
     OR NOT (CPLEX_CONCERT_FOUND)
     OR NOT (CPLEX_ILOCPLEX_FOUND))
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

      target_link_libraries(${TEST_EXE_NAME} ${CPLEX_CONCERT_LIBRARY_DEBUG})
      target_link_libraries(${TEST_EXE_NAME} ${CPLEX_ILOCPLEX_LIBRARY_DEBUG})
      target_link_libraries(${TEST_EXE_NAME} ${CPLEX_LIBRARY_DEBUG})
    else()
      message("   ${CPLEX_LIBRARY}")
      message("   ${CPLEX_CONCERT_LIBRARY}")
      message("   ${CPLEX_ILOCPLEX_LIBRARY}")

      target_link_libraries(${TEST_EXE_NAME} ${CPLEX_CONCERT_LIBRARY})
      target_link_libraries(${TEST_EXE_NAME} ${CPLEX_ILOCPLEX_LIBRARY})
      target_link_libraries(${TEST_EXE_NAME} ${CPLEX_LIBRARY})
    endif()
  endif()
endif(HAS_CPLEX)

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

      target_link_libraries(${TEST_EXE_NAME} ${GUROBI_CPP_LIBRARY})
      target_link_libraries(${TEST_EXE_NAME} ${GUROBI_LIBRARY_DEBUG})
    else()
      message("   ${GUROBI_LIBRARY}")
      message("   ${GUROBI_CPP_LIBRARY}")

      target_link_libraries(${TEST_EXE_NAME} ${GUROBI_CPP_LIBRARY})
      target_link_libraries(${TEST_EXE_NAME} ${GUROBI_LIBRARY})
    endif()
  endif()
endif(HAS_GUROBI)

target_link_libraries(${TEST_EXE_NAME} pthread)
target_link_libraries(${TEST_EXE_NAME} m)
target_link_libraries(${TEST_EXE_NAME} z)
target_link_libraries(${TEST_EXE_NAME} lapack)
target_link_libraries(${TEST_EXE_NAME} blas)
target_link_libraries(${TEST_EXE_NAME} dl)
target_link_libraries(${TEST_EXE_NAME} gomp)

if(HAS_GAMS)
  if(UNIX)
    if(APPLE)
      target_link_libraries(${TEST_EXE_NAME} ${GAMS_DIR}/libstdc++.6.dylib)
    else(APPLE)
      target_link_libraries(${TEST_EXE_NAME} ${GAMS_DIR}/libstdc++.so.6)
    endif(APPLE)
  endif(UNIX)
endif(HAS_GAMS)

target_link_libraries(${TEST_EXE_NAME} ${Boost_LIBRARIES})

# Adds each test
foreach(cpptest ${cpptests})
  foreach(part ${${cpptest}_parts})
    add_test(${cpptest}_${part}
             ${TEST_PATH}/test_runner
             ${cpptest}test
             ${part})
    set_tests_properties(${cpptest}_${part}
                         PROPERTIES FAIL_REGULAR_EXPRESSION
                                    "ERROR;FAIL;Test failed")
  endforeach()
endforeach()

# Copies the data files to the test directory
execute_process(COMMAND ${CMAKE_COMMAND}
                        -E
                        copy_directory
                        ${CMAKE_CURRENT_SOURCE_DIR}/data
                        ${CMAKE_CURRENT_BINARY_DIR}/data)
