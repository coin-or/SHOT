/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Enums.h"
#include "Structs.h"
#include <memory>

// The fmt library of spdlog, which SHOT uses to format messages
#include "spdlog/fmt/fmt.h"

// Declared instead of included, so that the files that include this header do not need to parse spdlog
namespace spdlog
{
class logger;

namespace sinks
{
    class sink;
} // namespace sinks
} // namespace spdlog

namespace SHOT
{

class DllExport Output
{
public:
    Output();
    virtual ~Output();

    void outputCritical(std::string message);
    void outputError(std::string message);
    void outputError(std::string message, std::string errormessage);
    void outputWarning(std::string message);
    void outputInfo(std::string message);
    void outputDebug(std::string message);
    void outputTrace(std::string message);

    // Whether trace output is written at all, so that the messages are not built when they are not used. Trace
    // output is compiled out in release builds.
    bool isTraceActive() const;

    void setLogLevels(E_LogLevel consoleLogLevel, E_LogLevel fileLogLevel);

    void setConsoleSink(std::shared_ptr<spdlog::sinks::sink> newSink);

    void setFileSink(std::string filename);

    void flush();

    void setPrefix(std::string prefix);

private:
    std::shared_ptr<spdlog::sinks::sink> consoleSink;
    std::shared_ptr<spdlog::sinks::sink> fileSink;

    std::shared_ptr<spdlog::logger> logger;
};

class Environment;
using EnvironmentPtr = std::shared_ptr<Environment>;

class OutputStream : public std::streambuf, public std::ostream
{
private:
    EnvironmentPtr env;
    std::stringstream ss;
    E_LogLevel logLevel;

public:
    OutputStream(EnvironmentPtr envPtr, E_LogLevel logLevel) : std::ostream(this), env(envPtr), logLevel(logLevel) { }

    int overflow(int c = std::istream::traits_type::eof());
};
}