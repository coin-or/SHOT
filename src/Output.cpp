/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Output.h"

namespace SHOT
{
Output::Output()
{
    osOutput = std::make_unique<OSOutput>();

    // Adds a file output
    osOutput->AddChannel("shotlogfile");
    osOutput->AddChannel("stdout");
}

Output::~Output() { outputAlways("output deleted"); }

void Output::outputAlways(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_always, message);
}

void Output::outputError(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, message);
}

void Output::outputError(std::string message, std::string errormessage)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, message);
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, " \"" + errormessage + "\"");
}

void Output::outputSummary(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_summary, message);
}

void Output::outputWarning(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_warning, message);
}

void Output::outputInfo(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info, message);
}

void Output::outputDebug(std::string message)
{
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_debug, message);
}

void Output::outputTrace(std::string message)
{
#ifndef NDEBUG
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_trace, message);
#endif
}

void Output::outputDetailedTrace(std::string message)
{
#ifndef NDEBUG
    osOutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_detailed_trace, message);
#endif
}

void Output::setLogLevels(int consoleLogLevel, int fileLogLevel)
{
    // Sets the correct log levels
    osOutput->SetPrintLevel("stdout", (ENUM_OUTPUT_LEVEL)(consoleLogLevel));
    osOutput->SetPrintLevel("shotlogfile", (ENUM_OUTPUT_LEVEL)(fileLogLevel));
}
} // namespace SHOT