/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "Enums.h"
#include "SHOTSettings.h"
#include "ProcessInfo.h"

// Used for OSOutput
#include "cstdio"
#define HAVE_STDIO_H 1
#include "OSOutput.h"

#include <boost/format.hpp>

class Output
{
  public:
    virtual ~Output();

    static Output &getInstance()
    {
        static Output inst;
        return (inst);
    }

    void outputAlways(std::string message);
    void outputError(std::string message);
    void outputError(std::string message, std::string errormessage);
    void outputSummary(std::string message);
    void outputWarning(std::string message);
    void outputInfo(std::string message);
    void outputDebug(std::string message);
    void outputTrace(std::string message);
    void outputDetailedTrace(std::string message);

    void setLogLevels();

    void outputIterationDetail(int iterationNumber,
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
                               double maxConstraintError);

    void outputIterationDetailHeader();

    void outputPrimalSolutionDetailedReport();

    void outputSolverHeader();

    void outputOptionsReport();

  private:
    OSOutput *osOutput;

    boost::format iterationDetailFormat;

    double lastDualObjectiveValue = -DBL_MAX;
    double lastPrimalObjectiveValue = DBL_MAX;
    double lastAbsoluteObjectiveGap = DBL_MAX;
    double lastRelativeObjectiveGap = 1.0;

    Output();
};