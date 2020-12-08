/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Environment.h"
#include "Enums.h"
#include "Structs.h"

namespace SHOT
{
class DllExport Report
{
public:
    Report(EnvironmentPtr envPtr);
    ~Report();

    void outputSolverHeader();

    void outputOptionsReport();

    void outputModelingSystemReport(ES_SourceFormat source, std::string filename = "");

    void outputProblemInstanceReport();

    void outputInteriorPointPreReport();

    void outputPreReport();

    void outputIterationDetailHeader();

    void outputIterationDetail(int iterationNumber, std::string iterationDesc, double totalTime, int dualCutsAdded,
        int dualCutsTotal, double dualObjectiveValue, double primalObjectiveValue, double absoluteObjectiveGap,
        double relativeObjectiveGap, double currentObjectiveValue, int maxConstraintIndex, double maxConstraintError,
        E_IterationLineType lineType, bool forcePrint = false);

    void outputIterationDetailHeaderMinimax();

    void outputIterationDetailMinimax(int iterationNumber, std::string iterationDesc, double totalTime,
        int dualCutsAdded, int dualCutsTotal, double dualObjectiveValue, double primalObjectiveValue,
        double absoluteObjectiveGap, double relativeObjectiveGap);

    void outputPrimalSolutionDetailedReport();

    void outputSolutionReport();

    bool firstIterationHeaderPrinted = false;

private:
    EnvironmentPtr env;

    double lastDualObjectiveValue = SHOT_DBL_MIN;
    double lastPrimalObjectiveValue = SHOT_DBL_MAX;
    double lastAbsoluteObjectiveGap = SHOT_DBL_MAX;
    double lastRelativeObjectiveGap = 1.0;
    double lastIterationOutputTimeStamp = 0.0;
    int iterationsWithoutPrintoutCounter = 0;
    int iterationPrintoutsSinceLastHeader = 0;
};
} // namespace SHOT