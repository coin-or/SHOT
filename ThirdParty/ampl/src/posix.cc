/*
 A C++ interface to POSIX functions.

 Copyright (c) 2012 - 2016, Victor Zverovich
 All rights reserved.

 For the license information refer to format.h.
 */

// Disable bogus MSVC warnings.
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "mp/posix.h"

#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <io.h>

#define O_CREAT _O_CREAT
#define O_TRUNC _O_TRUNC

#ifndef S_IRUSR
#define S_IRUSR _S_IREAD
#endif

#ifndef S_IWUSR
#define S_IWUSR _S_IWRITE
#endif

#ifdef __MINGW32__
#define _SH_DENYNO 0x40
#endif

#endif // _WIN32

#ifdef fileno
#undef fileno
#endif

namespace
{
#ifdef _WIN32
// Return type of read and write functions.
typedef int RWResult;

// On Windows the count argument to read and write is unsigned, so convert
// it from size_t preventing integer overflow.
inline unsigned convert_rwcount(std::size_t count)
{
    return count <= UINT_MAX ? static_cast<unsigned>(count) : UINT_MAX;
}
#else
// Return type of read and write functions.
typedef ssize_t RWResult;

inline std::size_t convert_rwcount(std::size_t count) { return count; }
#endif
}

fmtold::BufferedFile::~BufferedFile() FMTOLD_NOEXCEPT
{
    if(file_ && FMTOLD_SYSTEM(fclose(file_)) != 0)
        fmtold::report_system_error(errno, "cannot close file");
}

fmtold::BufferedFile::BufferedFile(fmtold::CStringRef filename, fmtold::CStringRef mode)
{
    FMTOLD_RETRY_VAL(file_, FMTOLD_SYSTEM(fopen(filename.c_str(), mode.c_str())), 0);
    if(!file_)
        FMTOLD_THROW(SystemError(errno, "cannot open file {}", filename));
}

void fmtold::BufferedFile::close()
{
    if(!file_)
        return;
    int result = FMTOLD_SYSTEM(fclose(file_));
    file_ = 0;
    if(result != 0)
        FMTOLD_THROW(SystemError(errno, "cannot close file"));
}

// A macro used to prevent expansion of fileno on broken versions of MinGW.
#define FMTOLD_ARGS

int fmtold::BufferedFile::fileno() const
{
    int fd = FMTOLD_POSIX_CALL(fileno FMTOLD_ARGS(file_));
    if(fd == -1)
        FMTOLD_THROW(SystemError(errno, "cannot get file descriptor"));
    return fd;
}

fmtold::File::File(fmtold::CStringRef path, int oflag)
{
    int mode = S_IRUSR | S_IWUSR;
#if defined(_WIN32) && !defined(__MINGW32__)
    fd_ = -1;
    FMTOLD_POSIX_CALL(sopen_s(&fd_, path.c_str(), oflag, _SH_DENYNO, mode));
#else
    FMTOLD_RETRY(fd_, FMTOLD_POSIX_CALL(open(path.c_str(), oflag, mode)));
#endif
    if(fd_ == -1)
        FMTOLD_THROW(SystemError(errno, "cannot open file {}", path));
}

fmtold::File::~File() FMTOLD_NOEXCEPT
{
    // Don't retry close in case of EINTR!
    // See http://linux.derkeiler.com/Mailing-Lists/Kernel/2005-09/3000.html
    if(fd_ != -1 && FMTOLD_POSIX_CALL(close(fd_)) != 0)
        fmtold::report_system_error(errno, "cannot close file");
}

void fmtold::File::close()
{
    if(fd_ == -1)
        return;
    // Don't retry close in case of EINTR!
    // See http://linux.derkeiler.com/Mailing-Lists/Kernel/2005-09/3000.html
    int result = FMTOLD_POSIX_CALL(close(fd_));
    fd_ = -1;
    if(result != 0)
        FMTOLD_THROW(SystemError(errno, "cannot close file"));
}

fmtold::LongLong fmtold::File::size() const
{
#ifdef _WIN32
    // Use GetFileSize instead of GetFileSizeEx for the case when _WIN32_WINNT
    // is less than 0x0500 as is the case with some default MinGW builds.
    // Both functions support large file sizes.
    DWORD size_upper = 0;
    HANDLE handle = reinterpret_cast<HANDLE>(_get_osfhandle(fd_));
    DWORD size_lower = FMTOLD_SYSTEM(GetFileSize(handle, &size_upper));
    if(size_lower == INVALID_FILE_SIZE)
    {
        DWORD error = GetLastError();
        if(error != NO_ERROR)
            FMTOLD_THROW(WindowsError(GetLastError(), "cannot get file size"));
    }
    fmtold::ULongLong long_size = size_upper;
    return (long_size << sizeof(DWORD) * CHAR_BIT) | size_lower;
#else
    typedef struct stat Stat;
    Stat file_stat = Stat();
    if(FMTOLD_POSIX_CALL(fstat(fd_, &file_stat)) == -1)
        FMTOLD_THROW(SystemError(errno, "cannot get file attributes"));
    FMTOLD_STATIC_ASSERT(
        sizeof(fmtold::LongLong) >= sizeof(file_stat.st_size), "return type of File::size is not large enough");
    return file_stat.st_size;
#endif
}

std::size_t fmtold::File::read(void* buffer, std::size_t count)
{
    RWResult result = 0;
    FMTOLD_RETRY(result, FMTOLD_POSIX_CALL(read(fd_, buffer, convert_rwcount(count))));
    if(result < 0)
        FMTOLD_THROW(SystemError(errno, "cannot read from file"));
    return internal::to_unsigned(result);
}

std::size_t fmtold::File::write(const void* buffer, std::size_t count)
{
    RWResult result = 0;
    FMTOLD_RETRY(result, FMTOLD_POSIX_CALL(write(fd_, buffer, convert_rwcount(count))));
    if(result < 0)
        FMTOLD_THROW(SystemError(errno, "cannot write to file"));
    return internal::to_unsigned(result);
}

fmtold::File fmtold::File::dup(int fd)
{
    // Don't retry as dup doesn't return EINTR.
    // http://pubs.opengroup.org/onlinepubs/009695399/functions/dup.html
    int new_fd = FMTOLD_POSIX_CALL(dup(fd));
    if(new_fd == -1)
        FMTOLD_THROW(SystemError(errno, "cannot duplicate file descriptor {}", fd));
    return File(new_fd);
}

void fmtold::File::dup2(int fd)
{
    int result = 0;
    FMTOLD_RETRY(result, FMTOLD_POSIX_CALL(dup2(fd_, fd)));
    if(result == -1)
    {
        FMTOLD_THROW(SystemError(errno, "cannot duplicate file descriptor {} to {}", fd_, fd));
    }
}

void fmtold::File::dup2(int fd, ErrorCode& ec) FMTOLD_NOEXCEPT
{
    int result = 0;
    FMTOLD_RETRY(result, FMTOLD_POSIX_CALL(dup2(fd_, fd)));
    if(result == -1)
        ec = ErrorCode(errno);
}

void fmtold::File::pipe(File& read_end, File& write_end)
{
    // Close the descriptors first to make sure that assignments don't throw
    // and there are no leaks.
    read_end.close();
    write_end.close();
    int fds[2] = {};
#ifdef _WIN32
    // Make the default pipe capacity same as on Linux 2.6.11+.
    enum
    {
        DEFAULT_CAPACITY = 65536
    };
    int result = FMTOLD_POSIX_CALL(pipe(fds, DEFAULT_CAPACITY, _O_BINARY));
#else
    // Don't retry as the pipe function doesn't return EINTR.
    // http://pubs.opengroup.org/onlinepubs/009696799/functions/pipe.html
    int result = FMTOLD_POSIX_CALL(pipe(fds));
#endif
    if(result != 0)
        FMTOLD_THROW(SystemError(errno, "cannot create pipe"));
    // The following assignments don't throw because read_fd and write_fd
    // are closed.
    read_end = File(fds[0]);
    write_end = File(fds[1]);
}

fmtold::BufferedFile fmtold::File::fdopen(const char* mode)
{
    // Don't retry as fdopen doesn't return EINTR.
    FILE* f = FMTOLD_POSIX_CALL(fdopen(fd_, mode));
    if(!f)
        FMTOLD_THROW(SystemError(errno, "cannot associate stream with file descriptor"));
    BufferedFile file(f);
    fd_ = -1;
    return file;
}

long fmtold::getpagesize()
{
#ifdef _WIN32
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return si.dwPageSize;
#else
    long size = FMTOLD_POSIX_CALL(sysconf(_SC_PAGESIZE));
    if(size < 0)
        FMTOLD_THROW(SystemError(errno, "cannot get memory page size"));
    return size;
#endif
}
