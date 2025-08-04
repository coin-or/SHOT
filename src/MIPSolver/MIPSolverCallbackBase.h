/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Environment.h"

#include "../Tasks/TaskSelectHyperplanesObjectiveFunction.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromRootsearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../Tasks/TaskSelectHyperplanesESH.h"
#include "../Tasks/TaskSelectHyperplanesECP.h"
#include "../Tasks/TaskSelectHyperplanesExternal.h"
#include "../Tasks/TaskUpdateInteriorPoint.h"

#include <memory>
#include <string>
#include <vector>

namespace SHOT
{
class MIPSolverCallbackBase
{
public:
    virtual ~MIPSolverCallbackBase() = default;

private:
protected:
    bool isMinimization = true;
    int lastNumAddedHyperplanes = 0;
    double lastUpdatedPrimal;

    int lastSummaryIter = 0;
    double lastSummaryTimeStamp = 0.0;
    int lastHeaderIter = 0;

    bool warningMessageShownLargeRHS = false;

    std::shared_ptr<TaskSelectPrimalCandidatesFromNLP> taskSelectPrimNLPOriginal;
    std::shared_ptr<TaskSelectPrimalCandidatesFromNLP> taskSelectPrimNLPReformulated;
    std::shared_ptr<TaskBase> taskSelectHPPts;
    std::shared_ptr<TaskSelectHyperplanesObjectiveFunction> taskSelectHPPtsByObjectiveRootsearch;
    std::shared_ptr<TaskSelectHyperplanesExternal> taskSelectExternalHPs;
    std::shared_ptr<TaskSelectPrimalCandidatesFromRootsearch> taskSelectPrimalSolutionFromRootsearch;
    std::shared_ptr<TaskUpdateInteriorPoint> tUpdateInteriorPoint;

    bool checkFixedNLPStrategy(SolutionPoint point);

    bool checkIterationLimit();

    bool checkUserTermination();

    void addLazyConstraint(std::vector<SolutionPoint> candidatePoints);

    void printIterationReport(SolutionPoint solution, std::string threadId);

    EnvironmentPtr env;
};
} // namespace SHOT