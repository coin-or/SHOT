/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskUpdateSingleVariableTransformationLinearizations.h"

#include <memory>

#include "../Model/Problem.h"
#include "../DualSolver.h"
#include "../Enums.h"
#include "../Environment.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

#ifdef HAS_GUROBI
#include "../MIPSolver/MIPSolverGurobi.h"
#endif

namespace SHOT
{

TaskUpdateSingleVariableTransformationLinearizations::TaskUpdateSingleVariableTransformationLinearizations(
    EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskUpdateSingleVariableTransformationLinearizations::~TaskUpdateSingleVariableTransformationLinearizations() = default;

void TaskUpdateSingleVariableTransformationLinearizations::run()
{
    if(env->dualSolver->itersWithoutAddedBreakpoints == 0)
        return;

    env->timing->startTimer("DualStrategy");

    auto solver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual"));

#ifdef HAS_GUROBI
    if(solver == ES_MIPSolver::Gurobi)
    {
        (std::dynamic_pointer_cast<MIPSolverGurobi>(env->dualSolver->MIPSolver))->removePiecewiseLinearApproximations();

        for(auto T : env->dualSolver->singleVariableTransformations)
        {
            (std::dynamic_pointer_cast<MIPSolverGurobi>(env->dualSolver->MIPSolver))
                ->addPiecewiseLinearApproximation(T);
        }
    }
#endif

    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    env->timing->stopTimer("DualStrategy");
}

std::string TaskUpdateSingleVariableTransformationLinearizations::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

} // namespace SHOT