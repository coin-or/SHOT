/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Shared.h"

namespace SHOT
{
class Report
{
  public:
    Report(EnvironmentPtr envPtr);
    ~Report();

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

    double lastDualObjectiveValue = -DBL_MAX;
    double lastPrimalObjectiveValue = DBL_MAX;
    double lastAbsoluteObjectiveGap = DBL_MAX;
    double lastRelativeObjectiveGap = 1.0;
    double lastIterationOutputTimeStamp = 0.0;
    int iterationsWithoutPrintoutCounter = 0;
    int iterationPrintoutsSinceLastHeader = 0;
    bool firstIterationHeaderPrinted = false;
};
} // namespace SHOT