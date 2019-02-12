  # The name of the test executable file
set(TEST_EXE_NAME test_runner)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)

# The main groups of tests, there should be a file matching the name +"test".cpp in the test directory
set (cpptests Model Settings)

set (Model_parts 1 2 3 4 5 6)    # The different parts of each test (if any)
set (Settings_parts 1 2)   

if(HAS_CPLEX)
    set(Cplex_parts 1)
    set(cpptests ${cpptests} Cplex)
endif()

if(HAS_GUROBI)
    set(cpptests ${cpptests} Gurobi)
endif()

if(HAS_GAMS)
    set(GAMS_parts 1 2 3 4 5)
    set(cpptests ${cpptests} GAMS)
endif()

if(HAS_OS)
    set(OS_parts 1 2 3 4 5)
    set(cpptests ${cpptests} OS)
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

if (HAS_CPLEX)  # To make CPLEX build
  add_definitions(-DIL_STD)
  add_definitions(-LDFLAGS="-Wl,--no-as-needed")
endif (HAS_CPLEX)

# Creates the test executable
create_test_sourcelist(srclist test_runner.cpp ${cpptestsrc})
add_executable(${TEST_EXE_NAME}  ${srclist})
target_link_libraries(${TEST_EXE_NAME}  ${libs})

target_link_libraries (${TEST_EXE_NAME} SHOTSolver)

if (HAS_CPLEX)  
  target_link_libraries (${TEST_EXE_NAME} ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libilocplex.a)
  target_link_libraries (${TEST_EXE_NAME} ${CPLEX_DIR}/concert/lib/x86-64_linux/static_pic/libconcert.a)
  target_link_libraries (${TEST_EXE_NAME} ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic/libcplex.a)
endif (HAS_CPLEX)

if (HAS_GUROBI)  
  target_link_libraries (${TEST_EXE_NAME} ${GUROBI_DIR}/linux64/lib/libgurobi80.so)
  target_link_libraries (${TEST_EXE_NAME} ${GUROBI_DIR}/linux64/lib/libgurobi_g++5.2.a)
endif (HAS_GUROBI)

target_link_libraries (${TEST_EXE_NAME} pthread)
target_link_libraries (${TEST_EXE_NAME} m)
target_link_libraries (${TEST_EXE_NAME} bz2)
target_link_libraries (${TEST_EXE_NAME} z)
target_link_libraries (${TEST_EXE_NAME} lapack)
target_link_libraries (${TEST_EXE_NAME} blas)
target_link_libraries (${TEST_EXE_NAME} dl)
target_link_libraries (${TEST_EXE_NAME} gomp)

if (HAS_GAMS)
  target_link_libraries (${TEST_EXE_NAME} ${GAMS_DIR}/libstdc++.so.6)
endif (HAS_GAMS)

if (HAS_OS)
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libcoinasl.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libcoinmumps.so )
#target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libcoinmetis.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libipopt.so)
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libipoptamplinterface.so)
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libCoinUtils.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libClp.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libOsi.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libOsiClp.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libCgl.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libCbc.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libCbcSolver.so )
target_link_libraries (${TEST_EXE_NAME} ${COINOS_DIR}/lib/libOS.so )
endif (HAS_OS)

target_link_libraries(${TEST_EXE_NAME} ${Boost_LIBRARIES})

# Adds each test
foreach(cpptest ${cpptests})
  foreach(part ${${cpptest}_parts})
    add_test(${cpptest}_${part} ${TEST_PATH}/test_runner ${cpptest}test ${part})
    set_tests_properties(${cpptest}_${part} PROPERTIES
      FAIL_REGULAR_EXPRESSION "ERROR;FAIL;Test failed" )
  endforeach()
endforeach()

# Copies the data files to the test directory
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/data ${CMAKE_CURRENT_BINARY_DIR}/data)