# From https://github.com/vector-of-bool/pitchfork/blob/develop/cmake/FindFilesystem.cmake
include(CMakePushCheckState)
include(CheckIncludeFileCXX)
include(CheckCXXSourceCompiles)

cmake_push_check_state(RESET)
set(CMAKE_CXX_STANDARD 17)

set(have_fs FALSE)

check_include_file_cxx("filesystem" HAVE_STD_FILESYSTEM)

if(NOT HAVE_STD_FILESYSTEM)
    check_include_file_cxx("experimental/filesystem" HAVE_STD_EXPERIMENTAL_FILESYSTEM)
endif()

if(HAVE_STD_FILESYSTEM)
    set(have_fs TRUE)
    set(header filesystem)
    set(namespace std::filesystem)
elseif(HAVE_STD_EXPERIMENTAL_FILESYSTEM)
    set(have_fs TRUE)
    set(header experimental/filesystem)
    set(namespace std::experimental::filesystem)
endif()

string(CONFIGURE [[
    #include <@header@>

    int main() {
        auto cwd = @namespace@::current_path();
        return cwd.string().size();
    }
]] code @ONLY)

if(have_fs)
    check_cxx_source_compiles("${code}" CAN_COMPILE_FS_WITHOUT_LINK)

    if(NOT CAN_COMPILE_FS_WITHOUT_LINK)
        set(CMAKE_REQUIRED_LIBRARIES stdc++fs)
        check_cxx_source_compiles("${code}" CAN_COMPILE_FS_WITH_LINK)
    endif()
endif()

cmake_pop_check_state()

if(have_fs AND (CAN_COMPILE_FS_WITHOUT_LINK OR CAN_COMPILE_FS_WITH_LINK))
    add_library(CXX::Filesystem INTERFACE IMPORTED)
    if(NOT CAN_COMPILE_FS_WITHOUT_LINK)
        target_link_libraries(CXX::Filesystem INTERFACE stdc++fs)
    endif()
    set(Filesystem_FOUND TRUE CACHE BOOL "" FORCE)
else()
    set(Filesystem_FOUND FALSE CACHE BOOL "" FORCE)
    if(Filesystem_FIND_REQUIRED)
        message(FATAL_ERROR "No linkable C++17 filesystem implementation found (also tried stdc++fs).")
    endif()
endif()
