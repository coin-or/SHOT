/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "OptProblemOriginalQuadraticObjective.h"

using namespace SHOT;

OptProblemOriginalQuadraticObjective::OptProblemOriginalQuadraticObjective(EnvironmentPtr envPtr) : OptProblemOriginal(envPtr)
{
}

OptProblemOriginalQuadraticObjective::~OptProblemOriginalQuadraticObjective()
{
}

bool OptProblemOriginalQuadraticObjective::setProblem(OSInstance *instance)
{
    this->setObjectiveFunctionType(E_ObjectiveFunctionType::Quadratic);
    this->setProblemInstance(instance);
    this->setTypeOfObjectiveMinimize(instance->instanceData->objectives->obj[0]->maxOrMin == "min");
    this->setObjectiveFunctionNonlinear(false);
    this->setNonlinearConstraintIndexes();

    if (this->getNonlinearConstraintIndexes().size() == 0)
    {
        env->settings->updateSetting("Relaxation.IterationLimit", "Dual", 0);
        env->settings->updateSetting("MIP.SolutionLimit.Initial", "Dual", 1000);
    }

    this->setVariableBoundsTightened(std::vector<bool>(getProblemInstance()->getVariableNumber(), false));

    this->repairNonboundedVariables();

    return true;
}