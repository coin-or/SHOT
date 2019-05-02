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
#include <vector>

namespace SHOT
{

class INLPSolver;

class TaskFindInteriorPoint : public TaskBase
{
public:
    TaskFindInteriorPoint(EnvironmentPtr envPtr);
    ~TaskFindInteriorPoint() override;

    void run() override;
    std::string getType() override;

private:
    std::vector<std::unique_ptr<INLPSolver>> NLPSolvers;

    VectorString variableNames;
};
} // namespace SHOT