/*
 A C++ interface to POSIX functions.

 Copyright (c) 2012 - 2016, Victor Zverovich
 All rights reserved.

 For the license information refer to format.h.
 */

#ifndef FMTOLD_POSIX_H_
#define FMTOLD_POSIX_H_

#if defined(__MINGW32__) || defined(__CYGWIN__)
// Workaround MinGW bug https://sourceforge.net/p/mingw/bugs/2024/.
#undef __STRICT_ANSI__
#endif

#include <errno.h>
#include <fcntl.h> // for O_RDONLY
#include <locale.h> // for locale_t
#include <stdio.h>
#include <stdlib.h> // for strtod_l

#include <cstddef>

#if defined __APPLE__ || defined(__FreeBSD__)
#include <xlocale.h> // for LC_NUMERIC_MASK on OS X
#endif

#include "format.h"

#ifndef FMTOLD_POSIX
#if defined(_WIN32) && !defined(__MINGW32__)
// Fix warnings about deprecated symbols.
#define FMTOLD_POSIX(call) _##call
#else
#define FMTOLD_POSIX(call) call
#endif
#endif

// Calls to system functions are wrapped in FMTOLD_SYSTEM for testability.
#ifdef FMTOLD_SYSTEM
#define FMTOLD_POSIX_CALL(call) FMTOLD_SYSTEM(call)
#else
#define FMTOLD_SYSTEM(call) call
#ifdef _WIN32
// Fix warnings about deprecated symbols.
#define FMTOLD_POSIX_CALL(call) ::_##call
#else
#define FMTOLD_POSIX_CALL(call) ::call
#endif
#endif

#if FMTOLD_GCC_VERSION >= 407
#define FMTOLD_UNUSED __attribute__((unused))
#else
#define FMTOLD_UNUSED
#endif

#ifndef FMTOLD_USE_STATIC_ASSERT
#define FMTOLD_USE_STATIC_ASSERT 0
#endif

#if FMTOLD_USE_STATIC_ASSERT || FMTOLD_HAS_FEATURE(cxx_static_assert)                                                  \
    || (FMTOLD_GCC_VERSION >= 403 && FMTOLD_HAS_GXX_CXX11) || _MSC_VER >= 1600
#define FMTOLD_STATIC_ASSERT(cond, message) static_assert(cond, message)
#else
#define FMTOLD_CONCAT_(a, b) FMTOLD_CONCAT(a, b)
#define FMTOLD_STATIC_ASSERT(cond, message) typedef int FMTOLD_CONCAT_(Assert, __LINE__)[(cond) ? 1 : -1] FMTOLD_UNUSED
#endif

// Retries the expression while it evaluates to error_result and errno
// equals to EINTR.
#ifndef _WIN32
#define FMTOLD_RETRY_VAL(result, expression, error_result)                                                             \
    do                                                                                                                 \
    {                                                                                                                  \
        result = (expression);                                                                                         \
    } while(result == error_result && errno == EINTR)
#else
#define FMTOLD_RETRY_VAL(result, expression, error_result) result = (expression)
#endif

#define FMTOLD_RETRY(result, expression) FMTOLD_RETRY_VAL(result, expression, -1)

namespace fmtold
{

// An error code.
class ErrorCode
{
private:
    int value_;

public:
    explicit ErrorCode(int value = 0) FMTOLD_NOEXCEPT : value_(value) {}

    int get() const FMTOLD_NOEXCEPT { return value_; }
};

// A buffered file.
class BufferedFile
{
private:
    FILE* file_;

    friend class File;

    explicit BufferedFile(FILE* f) : file_(f) {}

public:
    // Constructs a BufferedFile object which doesn't represent any file.
    BufferedFile() FMTOLD_NOEXCEPT : file_(0) {}

    // Destroys the object closing the file it represents if any.
    ~BufferedFile() FMTOLD_NOEXCEPT;

#if !FMTOLD_USE_RVALUE_REFERENCES
    // Emulate a move constructor and a move assignment operator if rvalue
    // references are not supported.

private:
    // A proxy object to emulate a move constructor.
    // It is private to make it impossible call operator Proxy directly.
    struct Proxy
    {
        FILE* file;
    };

public:
    // A "move constructor" for moving from a temporary.
    BufferedFile(Proxy p) FMTOLD_NOEXCEPT : file_(p.file) {}

    // A "move constructor" for moving from an lvalue.
    BufferedFile(BufferedFile& f) FMTOLD_NOEXCEPT : file_(f.file_) { f.file_ = 0; }

    // A "move assignment operator" for moving from a temporary.
    BufferedFile& operator=(Proxy p)
    {
        close();
        file_ = p.file;
        return *this;
    }

    // A "move assignment operator" for moving from an lvalue.
    BufferedFile& operator=(BufferedFile& other)
    {
        close();
        file_ = other.file_;
        other.file_ = 0;
        return *this;
    }

    // Returns a proxy object for moving from a temporary:
    //   BufferedFile file = BufferedFile(...);
    operator Proxy() FMTOLD_NOEXCEPT
    {
        Proxy p = { file_ };
        file_ = 0;
        return p;
    }

#else
private:
    FMTOLD_DISALLOW_COPY_AND_ASSIGN(BufferedFile);

public:
    BufferedFile(BufferedFile&& other) FMTOLD_NOEXCEPT : file_(other.file_) { other.file_ = 0; }

    BufferedFile& operator=(BufferedFile&& other)
    {
        close();
        file_ = other.file_;
        other.file_ = 0;
        return *this;
    }
#endif

    // Opens a file.
    BufferedFile(CStringRef filename, CStringRef mode);

    // Closes the file.
    void close();

    // Returns the pointer to a FILE object representing this file.
    FILE* get() const FMTOLD_NOEXCEPT { return file_; }

    // We place parentheses around fileno to workaround a bug in some versions
    // of MinGW that define fileno as a macro.
    int(fileno)() const;

    void print(CStringRef format_str, const ArgList& args) { fmtold::print(file_, format_str, args); }
    FMTOLD_VARIADIC(void, print, CStringRef)
};

// A file. Closed file is represented by a File object with descriptor -1.
// Methods that are not declared with FMTOLD_NOEXCEPT may throw
// fmtold::SystemError in case of failure. Note that some errors such as
// closing the file multiple times will cause a crash on Windows rather
// than an exception. You can get standard behavior by overriding the
// invalid parameter handler with _set_invalid_parameter_handler.
class File
{
private:
    int fd_; // File descriptor.

    // Constructs a File object with a given descriptor.
    explicit File(int fd) : fd_(fd) {}

public:
    // Possible values for the oflag argument to the constructor.
    enum
    {
        RDONLY = FMTOLD_POSIX(O_RDONLY), // Open for reading only.
        WRONLY = FMTOLD_POSIX(O_WRONLY), // Open for writing only.
        RDWR = FMTOLD_POSIX(O_RDWR) // Open for reading and writing.
    };

    // Constructs a File object which doesn't represent any file.
    File() FMTOLD_NOEXCEPT : fd_(-1) {}

    // Opens a file and constructs a File object representing this file.
    File(CStringRef path, int oflag);

#if !FMTOLD_USE_RVALUE_REFERENCES
    // Emulate a move constructor and a move assignment operator if rvalue
    // references are not supported.

private:
    // A proxy object to emulate a move constructor.
    // It is private to make it impossible call operator Proxy directly.
    struct Proxy
    {
        int fd;
    };

public:
    // A "move constructor" for moving from a temporary.
    File(Proxy p) FMTOLD_NOEXCEPT : fd_(p.fd) {}

    // A "move constructor" for moving from an lvalue.
    File(File& other) FMTOLD_NOEXCEPT : fd_(other.fd_) { other.fd_ = -1; }

    // A "move assignment operator" for moving from a temporary.
    File& operator=(Proxy p)
    {
        close();
        fd_ = p.fd;
        return *this;
    }

    // A "move assignment operator" for moving from an lvalue.
    File& operator=(File& other)
    {
        close();
        fd_ = other.fd_;
        other.fd_ = -1;
        return *this;
    }

    // Returns a proxy object for moving from a temporary:
    //   File file = File(...);
    operator Proxy() FMTOLD_NOEXCEPT
    {
        Proxy p = { fd_ };
        fd_ = -1;
        return p;
    }

#else
private:
    FMTOLD_DISALLOW_COPY_AND_ASSIGN(File);

public:
    File(File&& other) FMTOLD_NOEXCEPT : fd_(other.fd_) { other.fd_ = -1; }

    File& operator=(File&& other)
    {
        close();
        fd_ = other.fd_;
        other.fd_ = -1;
        return *this;
    }
#endif

    // Destroys the object closing the file it represents if any.
    ~File() FMTOLD_NOEXCEPT;

    // Returns the file descriptor.
    int descriptor() const FMTOLD_NOEXCEPT { return fd_; }

    // Closes the file.
    void close();

    // Returns the file size. The size has signed type for consistency with
    // stat::st_size.
    LongLong size() const;

    // Attempts to read count bytes from the file into the specified buffer.
    std::size_t read(void* buffer, std::size_t count);

    // Attempts to write count bytes from the specified buffer to the file.
    std::size_t write(const void* buffer, std::size_t count);

    // Duplicates a file descriptor with the dup function and returns
    // the duplicate as a file object.
    static File dup(int fd);

    // Makes fd be the copy of this file descriptor, closing fd first if
    // necessary.
    void dup2(int fd);

    // Makes fd be the copy of this file descriptor, closing fd first if
    // necessary.
    void dup2(int fd, ErrorCode& ec) FMTOLD_NOEXCEPT;

    // Creates a pipe setting up read_end and write_end file objects for reading
    // and writing respectively.
    static void pipe(File& read_end, File& write_end);

    // Creates a BufferedFile object associated with this file and detaches
    // this File object from the file.
    BufferedFile fdopen(const char* mode);
};

// Returns the memory page size.
long getpagesize();

#if(defined(LC_NUMERIC_MASK) || defined(_MSC_VER)) && !defined(__ANDROID__) && !defined(__CYGWIN__)
#define FMTOLD_LOCALE
#endif

#ifdef FMTOLD_LOCALE
// A "C" numeric locale.
class Locale
{
private:
#ifdef _MSC_VER
    typedef _locale_t locale_t;

    enum
    {
        LC_NUMERIC_MASK = LC_NUMERIC
    };

    static locale_t newlocale(int category_mask, const char* locale, locale_t)
    {
        return _create_locale(category_mask, locale);
    }

    static void freelocale(locale_t locale) { _free_locale(locale); }

    static double strtod_l(const char* nptr, char** endptr, _locale_t locale)
    {
        return _strtod_l(nptr, endptr, locale);
    }
#endif

    locale_t locale_;

    FMTOLD_DISALLOW_COPY_AND_ASSIGN(Locale);

public:
    typedef locale_t Type;

    Locale() : locale_(newlocale(LC_NUMERIC_MASK, "C", NULL))
    {
        if(!locale_)
            FMTOLD_THROW(fmtold::SystemError(errno, "cannot create locale"));
    }
    ~Locale() { freelocale(locale_); }

    Type get() const { return locale_; }

    // Converts string to floating-point number and advances str past the end
    // of the parsed input.
    double strtod(const char*& str) const
    {
        char* end = 0;
        double result = strtod_l(str, &end, locale_);
        str = end;
        return result;
    }
};
#endif // FMTOLD_LOCALE
} // namespace fmtold

#if !FMTOLD_USE_RVALUE_REFERENCES
namespace std
{
// For compatibility with C++98.
inline fmtold::BufferedFile& move(fmtold::BufferedFile& f) { return f; }
inline fmtold::File& move(fmtold::File& f) { return f; }
}
#endif

#endif // FMTOLD_POSIX_H_
