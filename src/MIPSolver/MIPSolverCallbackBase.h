/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Tasks/TaskUpdateNonlinearObjectiveByLinesearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromLinesearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../Tasks/TaskSelectHyperplanePointsLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsIndividualLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsSolution.h"
#include "../Tasks/TaskUpdateInteriorPoint.h"

class MIPSolverCallbackBase
{
  public:
    ~MIPSolverCallbackBase();

  private:
  protected:
    int cbCalls = 0;
    bool isMinimization = true;
    int lastNumAddedHyperplanes = 0;
    double lastUpdatedPrimal;

    int maxIntegerRelaxedHyperplanes = 0;

    int lastSummaryIter = 0;
    double lastSummaryTimeStamp = 0.0;
    int lastHeaderIter = 0;

    TaskBase *tSelectPrimNLP;
    TaskBase *taskSelectHPPts;
    TaskUpdateNonlinearObjectiveByLinesearch *taskUpdateObjectiveByLinesearch;
    TaskSelectPrimalCandidatesFromLinesearch *taskSelectPrimalSolutionFromLinesearch;
    TaskUpdateInteriorPoint *tUpdateInteriorPoint;

    bool bSelectPrimNLP = false;
    bool bSelectHPPts = false;
    bool bUpdateObjectiveByLinesearch = false;
    bool bSelectPrimalSolutionFromLinesearch = false;
    bool bUpdateInteriorPoint = false;

    bool checkFixedNLPStrategy(SolutionPoint point);

    bool checkIterationLimit();

    void addLazyConstraint(std::vector<SolutionPoint> candidatePoints);

    void printIterationReport(SolutionPoint solution, std::string threadId, std::string bestBound, std::string openNodes);
};
