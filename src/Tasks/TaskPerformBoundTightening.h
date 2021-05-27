/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include <memory>
#include <string>
#include <vector>

#include "../Structs.h"

namespace SHOT
{
class NLPSolverSHOT;

class TaskPerformBoundTightening : public TaskBase
{
public:
    TaskPerformBoundTightening(EnvironmentPtr envPtr, ProblemPtr source);
    ~TaskPerformBoundTightening() override;
    void run() override;
    std::string getType() override;

    std::shared_ptr<NLPSolverSHOT> POASolver;

private:
    virtual void createPOA();

    std::shared_ptr<TaskBase> taskSelectHPPts;

    ProblemPtr sourceProblem;
    ProblemPtr relaxedProblem;
};
} // namespace SHOT