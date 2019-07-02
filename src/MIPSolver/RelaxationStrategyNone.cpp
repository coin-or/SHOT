/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "RelaxationStrategyNone.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"

#include "../Model/Problem.h"

namespace SHOT
{

RelaxationStrategyNone::RelaxationStrategyNone(EnvironmentPtr envPtr)
{
    env = envPtr;

    setInitial();
}

RelaxationStrategyNone::~RelaxationStrategyNone() = default;

void RelaxationStrategyNone::setInitial() { setInactive(); }

void RelaxationStrategyNone::executeStrategy()
{
    env->results->getCurrentIteration()->isDualProblemDiscrete = true;
    env->results->getCurrentIteration()->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
}

void RelaxationStrategyNone::setActive() {}

void RelaxationStrategyNone::setInactive() {}
} // namespace SHOT