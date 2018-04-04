/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Output.h"

Output::Output()
{
    osOutput = new OSOutput();

    // Adds a file output
    osOutput->AddChannel("shotlogfile");
    osOutput->AddChannel("stdout");

    iterationDetailFormat = boost::format("%|6|: %|9s|%|=11f|%||%|14s|%|11e|%|27s|%|19|%|29|");
}

Output::~Output()
{
    delete osOutput;
    osOutput = NULL;
}

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

void Output::setLogLevels()
{
    // Sets the correct log levels
    osOutput->SetPrintLevel("stdout",
                            (ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("Console.LogLevel", "Output") + 1));
    osOutput->SetPrintLevel("shotlogfile",
                            (ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("File.LogLevel", "Output") + 1));
}

void Output::outputIterationDetail(int iterationNumber,
                                   std::string iterationDesc,
                                   double totalTime,
                                   int dualCutsAdded,
                                   int dualCutsTotal,
                                   double dualObjectiveValue,
                                   double primalObjectiveValue,
                                   double absoluteObjectiveGap,
                                   double relativeObjectiveGap,
                                   double currentObjectiveValue,
                                   int maxConstraintIndex,
                                   double maxConstraintError)
{
    try
    {
        iterationDetailFormat = boost::format("%|6|: %|9s|%|=11f|%||%|14s|%|11e|%|27s|%|19|%|29|");

        std::string combDualCuts = "";

        if (dualCutsAdded > 0)
        {
            combDualCuts = (boost::format("%|4i| | %|-6i|") % dualCutsAdded % dualCutsTotal).str();
        }

        if (dualObjectiveValue != lastDualObjectiveValue)
        {
            lastDualObjectiveValue = dualObjectiveValue;
        }

        if (primalObjectiveValue != lastPrimalObjectiveValue)
        {
            lastPrimalObjectiveValue = primalObjectiveValue;
        }

        std::string combObjectiveValue = (boost::format("%|12s| | %|-12s|") % UtilityFunctions::toStringFormat(dualObjectiveValue, "%#g") % UtilityFunctions::toStringFormat(primalObjectiveValue, "%#g")).str();

        if (absoluteObjectiveGap != lastAbsoluteObjectiveGap)
        {
            lastAbsoluteObjectiveGap = absoluteObjectiveGap;
        }

        if (relativeObjectiveGap != lastRelativeObjectiveGap)
        {
            lastRelativeObjectiveGap = relativeObjectiveGap;
        }

        std::string combObjectiveGap = (boost::format("%|8s| | %|-8s|") % UtilityFunctions::toStringFormat(absoluteObjectiveGap, "%#.1e") % UtilityFunctions::toStringFormat(relativeObjectiveGap, "%#.1e")).str();

        std::string combCurrSol;

        if (UtilityFunctions::isnan(currentObjectiveValue))
        {
            combCurrSol = "            inf.";
        }
        else
        {
            combCurrSol = (boost::format("%|#12g| | %|+.1e| (%|i|)") % currentObjectiveValue % maxConstraintError % maxConstraintIndex).str();
        }

        auto tmpLine = boost::format("%|6i|: %|-10s|%|#=10.2f|%|13s|%|27s|%|19s|%|-32s|") % iterationNumber % iterationDesc % totalTime % combDualCuts % combObjectiveValue % combObjectiveGap % combCurrSol;

        Output::getInstance()
            .outputSummary(tmpLine.str());
    }
    catch (...)
    {
        Output::getInstance().Output::getInstance().outputError("ERROR, cannot write iteration solution report!");
    }
}