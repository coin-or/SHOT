/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#ifdef HAS_CPLEX
#include "../MIPSolver/MIPSolverCplex.h"

#ifdef HAS_CPLEX_NEW_CALLBACK
#include "../MIPSolver/MIPSolverCplexLazy.h"
#endif

#include "../MIPSolver/MIPSolverCplexLazyOriginalCallback.h"
#endif

#ifdef HAS_GUROBI
#include "../MIPSolver/MIPSolverGurobi.h"
#include "../MIPSolver/MIPSolverGurobiLazy.h"
#endif

#ifdef HAS_CBC
#include "../MIPSolver/MIPSolverOsiCbc.h"
#endif

namespace SHOT
{
class TaskInitializeDualSolver : public TaskBase
{
public:
    TaskInitializeDualSolver(EnvironmentPtr envPtr, bool useLazyStrategy);
    virtual ~TaskInitializeDualSolver();

    void run();
    virtual std::string getType();

private:
};
} // namespace SHOT