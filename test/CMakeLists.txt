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
    10
    11
    12
    13) # The different parts of each test (if any)
set(Settings_parts 1 2)

if(HAS_CBC)
  set(Cbc_parts 1 2 3 4 5 6 7 8)
  set(cpptests ${cpptests} Cbc)
endif()

if(HAS_CPLEX)
  set(Cplex_parts 1 2 3 4 5 6 7 8 9 10 11)
  set(cpptests ${cpptests} Cplex)
endif()

if(HAS_GUROBI)
  set(Gurobi_parts 1 2 3 4 5 6 7 8 9 10 11)
  set(cpptests ${cpptests} Gurobi)
endif()

if(HAS_HIGHS)
  set(Highs_parts 1 2)
  set(cpptests ${cpptests} Highs)
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
    6
    7
    8
    9)
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

# Set rpath for finding dynamic libraries on macOS
if(APPLE)
    set(RPATH_DIRS "${CPPAD_LIB_DIR}")
    
    if(HAS_GUROBI AND GUROBI_FOUND)
        list(APPEND RPATH_DIRS "${GUROBI_LIB_DIR}")
    endif()
    
    set_target_properties(${TEST_EXE_NAME} PROPERTIES
        BUILD_RPATH "${RPATH_DIRS}"
        INSTALL_RPATH "${RPATH_DIRS}")
endif()

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

# Platform-specific libraries
if(NOT APPLE)
  target_link_libraries(${TEST_EXE_NAME} dl)
  target_link_libraries(${TEST_EXE_NAME} gomp)
endif()

if(HAS_GAMS)
  if(UNIX)
    if(APPLE)
      # Don't link libstdc++ on macOS - Clang uses libc++ and mixing them causes ABI issues
      # target_link_libraries(${TEST_EXE_NAME} ${GAMS_DIR}/libstdc++.6.dylib)
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

# ===== Python API Tests =====
# Add Python tests if SHOTpy target exists (Python bindings are built)
if(TARGET SHOTpy)
  # Find Python interpreter
  find_package(Python3 COMPONENTS Interpreter)
  
  if(Python3_FOUND)
    # Check if pytest is available
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import pytest"
      RESULT_VARIABLE PYTEST_FOUND
      OUTPUT_QUIET
      ERROR_QUIET
    )
    
    if(PYTEST_FOUND EQUAL 0)
      message(STATUS "pytest found - Python API tests will be added")
      
      # Copy Python test files to build directory
      execute_process(COMMAND ${CMAKE_COMMAND}
                              -E
                              copy_directory
                              ${CMAKE_CURRENT_SOURCE_DIR}/python
                              ${CMAKE_CURRENT_BINARY_DIR}/python)
      
      # Add a single test that runs all Python tests via pytest
      add_test(
        NAME Python_API_Tests
        COMMAND ${Python3_EXECUTABLE} -m pytest 
                ${CMAKE_CURRENT_BINARY_DIR}/python
                -v
                --tb=short
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      )
      
      # Set environment so pytest can find SHOTpy module
      set_tests_properties(Python_API_Tests PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}"
        FAIL_REGULAR_EXPRESSION "FAILED;ERROR"
        TIMEOUT 300
      )
      
      # Individual Python test files (for more granular testing)
      set(PYTHON_TEST_FILES
        test_module_api
        test_variables_expressions
        test_constraints
        test_objectives
        test_settings
        test_problem_io
        test_create_and_solve_problems
        test_optimization_problems
        test_modeling_systems
        test_gradients_hessians
        test_callbacks
      )
      
      foreach(pytest_file ${PYTHON_TEST_FILES})
        add_test(
          NAME Python_${pytest_file}
          COMMAND ${Python3_EXECUTABLE} -m pytest 
                  ${CMAKE_CURRENT_BINARY_DIR}/python/${pytest_file}.py
                  -v
                  --tb=short
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
        set_tests_properties(Python_${pytest_file} PROPERTIES
          ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}"
          FAIL_REGULAR_EXPRESSION "FAILED;ERROR"
          TIMEOUT 300
        )
      endforeach()
      
    else()
      message(STATUS "pytest not found - Python API tests will be skipped")
      message(STATUS "Install pytest with: pip install pytest")
    endif()
  else()
    message(STATUS "Python3 not found - Python API tests will be skipped")
  endif()
endif()
