/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Enums.h"
#include <memory>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

namespace SHOT
{
class __declspec(dllexport) Output
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

    void setLogLevels(E_LogLevel consoleLogLevel, E_LogLevel fileLogLevel);

    void setConsoleSink(std::shared_ptr<spdlog::sinks::sink> newSink);

private:
    std::shared_ptr<spdlog::sinks::sink> consoleSink;
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> fileSink;

    std::shared_ptr<spdlog::logger> logger;
};
}