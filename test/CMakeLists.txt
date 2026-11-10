# The name of the test executable file
set(TEST_EXE_NAME test_runner)
set(CMAKE_CXX_STANDARD 17)

# The main groups of tests, there should be a file matching the name +"test".cpp
# in the test directory
set(cpptests Model Settings MIPCapabilities PrimalNLP)
set(MIPCapabilities_parts 1 2 3 4 5 6)
set(PrimalNLP_parts 1 2)

if(HAS_IPOPT)
  list(APPEND PrimalNLP_parts 3)
endif()

add_test(NAME GurobiVersion COMMAND ${CMAKE_COMMAND}
    -DSHOT_SOURCE_DIR=${PROJECT_SOURCE_DIR}
    -DSHOT_TEST_DIR=${PROJECT_BINARY_DIR}/test/gurobi-version
    -P ${CMAKE_CURRENT_SOURCE_DIR}/GurobiVersionTest.cmake)

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
    13
    14
    15
    16
    17
    18
    19
    20
    21
    22
    23
    24
    25
    26
    27
    28
    29
    30
    31
    32
    33
    34
    35
    36
    37
    38
    39
    40
    41
    42
    43
    44
    45
    46
    47
    48
    49
    50
    51
    52
    53
    54
    55
    56
    57
    58
    59
    60
    61
    62
    63
    64
    65
    66
    67
    68
    69
    70
    71
    72
    73
    74
    75
    76) # The different parts of each test (if any)

set(Settings_parts 1 2 3 4 5 6 7 8 9 10 11 12 13)

if(HAS_CBC)
  set(Cbc_parts 1 2 3 4 5 6 7 8 9 10 11 12)
  set(cpptests ${cpptests} Cbc)
endif()

if(HAS_CPLEX)
  set(Cplex_parts 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31)
  set(cpptests ${cpptests} Cplex)
endif()

if(HAS_GUROBI)
  set(Gurobi_parts 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22)
  set(cpptests ${cpptests} Gurobi)
endif()

if(HAS_HIGHS)
  set(Highs_parts 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15)
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
      11
      12
      13
      14
      15
      16
      17
      18)
  # HiGHS does not implement SHOT's SOS interface.
  if(NOT (HAS_CBC OR HAS_CPLEX OR HAS_GUROBI))
    list(REMOVE_ITEM GAMS_parts 8 9 10)
  endif()
  set(cpptests ${cpptests} GAMS)
endif()

set(DualBound_parts 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22)
set(cpptests ${cpptests} DualBound)

set(Determinism_parts 1 2 3)
set(cpptests ${cpptests} Determinism)

set(Solver_parts
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
    13
    14
    15
    16
    17
    18
    19
    20
    21
    22
    23
    24
    25
    26
    27
    28
    29
    30
    31
    32
    33
    34
    35
    36
    37
    38
    39
    40
    41
    42
    43
    44
    45
    47
    48
    49
    50
    51
    53)
if(HAS_IPOPT)
  list(APPEND Solver_parts 46)
endif()
if(HAS_GUROBI)
  list(APPEND Solver_parts 52)
endif()
set(cpptests ${cpptests} Solver)

set(StrategySelection_parts)
if(HAS_CBC)
  list(APPEND StrategySelection_parts 1)
endif()
if(HAS_HIGHS)
  list(APPEND StrategySelection_parts 2)
endif()
if(HAS_CPLEX)
  list(APPEND StrategySelection_parts 3)
endif()
if(HAS_GUROBI)
  list(APPEND StrategySelection_parts 4)
endif()
if(StrategySelection_parts)
  list(APPEND cpptests StrategySelection)
endif()

if(HAS_CONOPT)
  list(APPEND cpptests Conopt)
  set(Conopt_parts 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16)
endif()

# Equality constraints and ranges in the original and the reformulated problem
set(EqualityConstraint_parts 1 2 3 4 5)

if(HAS_HIGHS)
  list(APPEND EqualityConstraint_parts 6)
endif()

if(HAS_CBC)
  list(APPEND EqualityConstraint_parts 7)
endif()

if(HAS_GUROBI)
  list(APPEND EqualityConstraint_parts 8)
endif()

if(HAS_CPLEX)
  list(APPEND EqualityConstraint_parts 9)
endif()

if(HAS_IPOPT)
  list(APPEND EqualityConstraint_parts 10)
endif()

if(HAS_UNO)
  list(APPEND EqualityConstraint_parts 11)
endif()

set(cpptests ${cpptests} EqualityConstraint)

# The exhaustive search in the fixed-integer primal strategy, which needs an NLP solver
if(HAS_HIGHS AND HAS_IPOPT)
  set(Enumeration_parts 1 2 3 4 5 6 7)

  if(HAS_GUROBI)
    list(APPEND Enumeration_parts 8)
  endif()

  if(HAS_CPLEX)
    list(APPEND Enumeration_parts 9)
  endif()

  set(cpptests ${cpptests} Enumeration)
endif()

if(HAS_IPOPT)
  set(cpptests ${cpptests} Ipopt)
  set(Ipopt_parts 1 2 3 4 5)
endif()

if(HAS_UNO)
  set(cpptests ${cpptests} Uno)
  set(Uno_parts 1 2 3 4)
endif()

# Instance tests: one case per (MIP solver x NLP solver) combination
# Format support (.nl, .gms) is checked at runtime; guards here are MIP+NLP only
set(Instance_parts)
if(HAS_HIGHS AND HAS_CONOPT)
  list(APPEND Instance_parts 11)
endif()
if(HAS_HIGHS AND HAS_IPOPT)
  list(APPEND Instance_parts 1)
endif()
if(HAS_GUROBI AND HAS_IPOPT)
  list(APPEND Instance_parts 2)
endif()
if(HAS_CPLEX AND HAS_IPOPT)
  list(APPEND Instance_parts 3)
endif()
if(HAS_CBC AND HAS_IPOPT)
  list(APPEND Instance_parts 4)
endif()
if(HAS_HIGHS)
  list(APPEND Instance_parts 5)
endif()
if(HAS_GUROBI)
  list(APPEND Instance_parts 6)
endif()
if(HAS_CPLEX)
  list(APPEND Instance_parts 7)
endif()
if(HAS_CBC)
  list(APPEND Instance_parts 8)
endif()
if(Instance_parts)
  set(cpptests ${cpptests} Instance)
endif()

# FullInstance tests: same (MIP solver x NLP solver) matrix as Instance, but scans the larger
# opt-in benchmark folders (MINLP-convex, MINLP-nonconvex, MIQCQP-convex) instead of the small
# "core" set (see test/InstanceTestCommon.h). Registered under the same guards as Instance, but
# each registered test is marked DISABLED unless ENABLE_FULL_INSTANCE_TESTS is ON, so a bare
# `ctest` run (as used in CI) never executes them. See docs/DebuggingSHOT.md.
option(ENABLE_FULL_INSTANCE_TESTS
       "Enable the FullInstance CTest group (large opt-in MINLP benchmark instance sets). When OFF \
(default), FullInstance_* tests are registered but DISABLED, so they are skipped by a plain 'ctest' \
run; reconfigure with this ON to run them via 'ctest -R FullInstance'."
       OFF)

set(FullInstance_parts ${Instance_parts})
if(FullInstance_parts)
  set(cpptests ${cpptests} FullInstance)
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

# Creates the test executable
create_test_sourcelist(srclist test_runner.cpp ${cpptestsrc})
add_executable(${TEST_EXE_NAME} ${srclist})

if(MSVC AND HAS_GAMS)
  # Import the GAMS API function-pointer data from the solver DLL.
  file(READ "${GAMS_DIR}/apifiles/C/api/gmomcc.h" _gmo_import_header)
  string(REPLACE "extern NAME##_t NAME"
                 "extern __declspec(dllimport) NAME##_t NAME"
                 _gmo_import_header "${_gmo_import_header}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/gmomcc-import.h" "${_gmo_import_header}")
  target_include_directories(${TEST_EXE_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endif()
target_link_libraries(${TEST_EXE_NAME} ${libs} SHOTBuildSettings)

target_include_directories(${TEST_EXE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty)

target_link_libraries(${TEST_EXE_NAME} SHOTSolver)
if(WIN32)
  # Keep the runner beside the DLL, including after incremental DLL rebuilds.
  set_target_properties(${TEST_EXE_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
endif()
if(HAS_HIGHS)
  target_link_libraries(${TEST_EXE_NAME} highs::highs)
endif()
if(HAS_CBC)
  target_link_libraries(${TEST_EXE_NAME} CBC::CBC)
endif()
if(HAS_IPOPT)
  target_link_libraries(${TEST_EXE_NAME} IPOPT::IPOPT)
endif()

# Set rpath for finding dynamic libraries on macOS
if(APPLE)
    set(RPATH_DIRS "${CPPAD_LIB_DIR};${CONOPT_LIBRARY_DIR}")
    
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

if(NOT WIN32)
  target_link_libraries(${TEST_EXE_NAME} pthread m z lapack blas)
endif()

# Platform-specific libraries
if(UNIX AND NOT APPLE)
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
    add_test(NAME ${cpptest}_${part}
             COMMAND $<TARGET_FILE:${TEST_EXE_NAME}>
             ${cpptest}test
             ${part})
    set_tests_properties(${cpptest}_${part}
                         PROPERTIES FAIL_REGULAR_EXPRESSION
                                    "ERROR;FAIL;Test failed")
    if(cpptest STREQUAL "FullInstance" AND NOT ENABLE_FULL_INSTANCE_TESTS)
      set_tests_properties(${cpptest}_${part} PROPERTIES DISABLED TRUE)
    endif()
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
      
      # One test per Python test file. The files are found rather than listed, so that a new file cannot be left out;
      # as the copy above, a new file is added when CMake configures again
      file(GLOB PYTHON_TEST_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/python/test_*.py)
      list(SORT PYTHON_TEST_PATHS)

      foreach(pytest_path ${PYTHON_TEST_PATHS})
        get_filename_component(pytest_file ${pytest_path} NAME_WE)
        set(pytest_selection)

        if(pytest_file STREQUAL "test_optimization_problems")
          set(pytest_selection -k "not _gams")
        endif()

        add_test(
          NAME Python_${pytest_file}
          COMMAND ${Python3_EXECUTABLE} -m pytest 
                  ${CMAKE_CURRENT_BINARY_DIR}/python/${pytest_file}.py
                  -v
                  --tb=short
                  ${pytest_selection}
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
        # pytest returns 5 when no test was collected, e.g., when a file is skipped since it only applies to an
        # installed SHOTpy package
        set_tests_properties(Python_${pytest_file} PROPERTIES
          ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR};SHOT_TUTORIAL_NOTEBOOK=${PROJECT_SOURCE_DIR}/docs/SHOTpy_Tutorial.ipynb"
          FAIL_REGULAR_EXPRESSION "FAILED;ERROR"
          SKIP_RETURN_CODE 5
          TIMEOUT 300
        )
      endforeach()

      # GAMS starts an external process for each NLP solve. Give each MIP backend its own CTest entry and timeout,
      # rather than making the entire backend matrix share one timeout.
      if(HAS_GAMS)
        foreach(mip_solver CBC CPLEX GUROBI HIGHS)
          if(HAS_${mip_solver})
            string(TOLOWER "${mip_solver}" mip_solver_name)
            set(gams_test "Python_test_optimization_problems_GAMS_${mip_solver}")
            add_test(
              NAME ${gams_test}
              COMMAND ${Python3_EXECUTABLE} -m pytest
                      ${CMAKE_CURRENT_BINARY_DIR}/python/test_optimization_problems.py
                      -v --tb=short -k "_${mip_solver_name}_gams"
              WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
            set_tests_properties(${gams_test} PROPERTIES
              ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR};SHOT_TUTORIAL_NOTEBOOK=${PROJECT_SOURCE_DIR}/docs/SHOTpy_Tutorial.ipynb"
              FAIL_REGULAR_EXPRESSION "FAILED;ERROR"
              SKIP_RETURN_CODE 5
              TIMEOUT 900)
          endif()
        endforeach()
      endif()

      set_tests_properties(Python_test_optimization_problems PROPERTIES TIMEOUT 900)
      
    else()
      message(STATUS "pytest not found - Python API tests will be skipped")
      message(STATUS "Install pytest with: pip install pytest")
    endif()
  else()
    message(STATUS "Python3 not found - Python API tests will be skipped")
  endif()
endif()
