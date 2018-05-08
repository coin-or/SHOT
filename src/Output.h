/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include <float.h>
#include "Enums.h"
#include "Environment.h"

// Used for OSOutput
#include "cstdio"
#define HAVE_STDIO_H 1
#include "OSOutput.h"

#include "ProcessInfo.h"

#include <boost/format.hpp>

class Output
{
  public:
    Output(EnvironmentPtr envPtr);
    virtual ~Output();

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

    void outputSolverHeader();

    void outputOptionsReport();

    void outputProblemInstanceReport();

    void outputInteriorPointPreReport();

    void outputIterationDetailHeader();

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
                               double maxConstraintError,
                               E_IterationLineType lineType);

    void outputIterationDetailHeaderMinimax();

    void outputIterationDetailMinimax(int iterationNumber,
                                      std::string iterationDesc,
                                      double totalTime,
                                      int dualCutsAdded,
                                      int dualCutsTotal,
                                      double dualObjectiveValue,
                                      double primalObjectiveValue,
                                      double absoluteObjectiveGap,
                                      double relativeObjectiveGap);

    void outputPrimalSolutionDetailedReport();

    void outputSolutionReport();

  private:
    EnvironmentPtr env;
    OSOutput *osOutput;

    double lastDualObjectiveValue = -DBL_MAX;
    double lastPrimalObjectiveValue = DBL_MAX;
    double lastAbsoluteObjectiveGap = DBL_MAX;
    double lastRelativeObjectiveGap = 1.0;
    double lastIterationOutputTimeStamp = 0.0;
    int iterationsWithoutPrintoutCounter = 0;
    int iterationPrintoutsSinceLastHeader = 0;
    bool firstIterationHeaderPrinted = false;
};