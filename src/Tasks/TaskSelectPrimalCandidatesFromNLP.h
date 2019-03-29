/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include "../NLPSolver/NLPSolverIpoptRelaxed.h"

#ifdef HAS_GAMS
#include "../NLPSolver/NLPSolverGAMS.h"
#include "../ModelingSystem/ModelingSystemGAMS.h"
#endif

#ifdef HAS_OS
#include "../ModelingSystem/ModelingSystemOS.h"
#endif

#include "../Tasks/TaskSelectHyperplanePointsESH.h"
#include "../Tasks/TaskSelectHyperplanePointsECP.h"

namespace SHOT
{
class TaskSelectPrimalCandidatesFromNLP : public TaskBase
{
public:
    TaskSelectPrimalCandidatesFromNLP(EnvironmentPtr envPtr);
    virtual ~TaskSelectPrimalCandidatesFromNLP();
    virtual void run();
    virtual std::string getType();

private:
    virtual bool solveFixedNLP();

    std::shared_ptr<INLPSolver> NLPSolver;

    VectorInteger discreteVariableIndexes;
    std::vector<VectorDouble> testedPoints;
    VectorDouble fixPoint;

    double originalNLPTime;
    double originalNLPIter;

    VectorDouble originalLBs;
    VectorDouble originalUBs;

    VectorString variableNames;

    std::shared_ptr<TaskBase> taskSelectHPPts;

    int originalIterFrequency;
    double originalTimeFrequency;
};
} // namespace SHOT