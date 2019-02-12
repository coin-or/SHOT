add_library(SHOTSettings SHARED SHOTSettings.cpp)

if (HAS_OS)
  # add libraries from os_addlibs_cpp.txt
  # TODO should probably use pkg-config instead
  file(READ ${COINOS_DIR}/share/coin/doc/OS/os_addlibs.txt OS_LINKFLAGS)
  string(STRIP ${OS_LINKFLAGS} OS_LINKFLAGS)
  target_link_libraries(SHOTSettings ${OS_LINKFLAGS})
  if (UNIX)
    target_link_libraries(SHOTSettings -Wl,-rpath,${COINOS_DIR}/lib)
  endif (UNIX)
endif (HAS_OS)
