find_package(PkgConfig)

set(CBC_FOUND false)

if(PkgConfig_FOUND)
  include(FindPkgConfig)

  set(ENV{PKG_CONFIG_PATH} "${CBC_DIR}/lib/pkgconfig")
  pkg_search_module(CBC cbc)

  if(CBC_FOUND)
    message("-- Cbc found using Pkg-config:")
    message("   Include directories found: ${CBC_INCLUDE_DIRS}")
    message("   Library directories found: ${CBC_LIBRARY_DIRS}")

    # Handle the QUIETLY and REQUIRED arguments and set CBC_FOUND to TRUE if all listed variables are TRUE.
    find_package_handle_standard_args(CBC
                                      DEFAULT_MSG
                                      CBC_LIBRARIES
                                      CBC_INCLUDE_DIRS
                                      CBC_LIBRARY_DIRS)
    mark_as_advanced(CBC_LIBRARIES CBC_INCLUDE_DIRS CBC_LIBRARY_DIRS)
  endif(CBC_FOUND)

endif(PkgConfig_FOUND)

if(NOT (CBC_FOUND))

  message("-- Searching for Cbc libraries and its dependencies, e.g. in ${CBC_DIR}/lib/")

  set(CBC_LIBRARIES "")

  find_library(Cbc_LIBRARY NAMES libCbc.so libCbc.lib HINTS ${CBC_DIR}/lib/)

  if(Cbc_LIBRARY)
    message("   Cbc library found at: " ${Cbc_LIBRARY})
    set(CBC_LIBRARIES ${CBC_LIBRARIES} ${Cbc_LIBRARY})
  endif()

  find_library(CbcSolver_LIBRARY NAMES libCbcSolver.so libCbcSolver.lib HINTS ${CBC_DIR}/lib/)
  if(CbcSolver_LIBRARY)
    message("   CbcSolver library found at: " ${CbcSolver_LIBRARY})
    set(CBC_LIBRARIES ${CBC_LIBRARIES} ${CbcSolver_LIBRARY})
  endif()

  find_library(OsiClp_LIBRARY NAMES libOsiClp.so libOsiClp.lib HINTS ${CBC_DIR}/lib/)
  if(OsiClp_LIBRARY)
    message("   OsiClp library found at: " ${OsiClp_LIBRARY})
    set(CBC_LIBRARIES ${CBC_LIBRARIES} ${OsiClp_LIBRARY})
  endif()

  find_library(Clp_LIBRARY NAMES libClp.so libClp.lib HINTS ${CBC_DIR}/lib/)
  if(Clp_LIBRARY)
    message("   Clp library found at: " ${Clp_LIBRARY})
    set(CBC_LIBRARIES ${CBC_LIBRARIES} ${Clp_LIBRARY})
  endif()

  find_library(Osi_LIBRARY NAMES libOsi.so libOsi.lib HINTS ${CBC_DIR}/lib/)
  if(Osi_LIBRARY)
    message("   Osi library found at: " ${Osi_LIBRARY})
    set(CBC_LIBRARIES ${CBC_LIBRARIES} ${Osi_LIBRARY})
  endif()

  find_library(CoinUtils_LIBRARY NAMES libCoinUtils.so libCoinUtils.lib HINTS ${CBC_DIR}/lib/)
  if(CoinUtils_LIBRARY)
    message("   CoinUtils library found at: " ${CoinUtils_LIBRARY})
    set(CBC_LIBRARIES ${CBC_LIBRARIES} ${CoinUtils_LIBRARY})
  endif()

  # find_library(ASL_LIBRARY NAMES libcoinasl.so libcoinasl.lib HINTS ${CBC_DIR}/lib/) if(ASL_LIBRARY) message("   ASL
  # library found at: " ${ASL_LIBRARY}) set(CBC_LIBRARIES ${CBC_LIBRARIES} ${ASL_LIBRARY}) endif()

  find_library(Cgl_LIBRARY NAMES libCgl.so libCgl.lib HINTS ${CBC_DIR}/lib/)
  if(Cgl_LIBRARY)
    message("   Cgl library found at: " ${Cgl_LIBRARY})
    set(CBC_LIBRARIES ${CBC_LIBRARIES} ${Cgl_LIBRARY})
  endif()

  find_path(CBC_INCLUDE_DIRS
            NAMES CbcConfig.h
            PATHS "${CBC_DIR}/include/coin"
                  "$ENV{CBC_DIR}/include/coin"
                  "/usr/include/coin"
                  "C:\\libs\\cbc\\include")

  # Handle the QUIETLY and REQUIRED arguments and set CBC_FOUND to TRUE if all listed variables are TRUE.
  find_package_handle_standard_args(CBC
                                    DEFAULT_MSG
                                    CBC_LIBRARIES
                                    CBC_INCLUDE_DIRS
                                    Cbc_LIBRARY
                                    CbcSolver_LIBRARY
                                    OsiClp_LIBRARY
                                    Clp_LIBRARY
                                    Osi_LIBRARY
                                    CoinUtils_LIBRARY
                                    # ASL_LIBRARY
                                    Cgl_LIBRARY)
  mark_as_advanced(CBC_LIBRARIES CBC_INCLUDE_DIR CBC_LIB_DIR)

endif()
