/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "OptProblemNLPRelaxed.h"

OptProblemNLPRelaxed::OptProblemNLPRelaxed(EnvironmentPtr envPtr) : OptProblem(envPtr)
{
    env = envPtr;
}

OptProblemNLPRelaxed::~OptProblemNLPRelaxed()
{
    /*delete osilReader;
    osilReader = NULL;*/
}

void OptProblemNLPRelaxed::reformulate(OSInstance *originalInstance)
{
    OSInstance *newInstance = NULL;
    newInstance = new OSInstance();

    this->setObjectiveFunctionNonlinear(isConstraintNonlinear(originalInstance, -1));
    this->setTypeOfObjectiveMinimize(originalInstance->instanceData->objectives->obj[0]->maxOrMin == "min");

    this->copyVariables(originalInstance, newInstance, true);

    this->copyObjectiveFunction(originalInstance, newInstance);

    this->copyConstraints(originalInstance, newInstance);

    this->copyLinearTerms(originalInstance, newInstance);

    this->copyQuadraticTerms(originalInstance, newInstance);

    this->copyNonlinearExpressions(originalInstance, newInstance);

    this->setProblemInstance(newInstance);

    this->setNonlinearConstraintIndexes();

    if (this->isObjectiveFunctionNonlinear())
    {
        setNonlinearObjectiveConstraintIdx(-1); // Sets a virtual constraint

        setNonlinearObjectiveVariableIdx(originalInstance->getVariableNumber());
    }

    this->repairNonboundedVariables();
}

void OptProblemNLPRelaxed::copyObjectiveFunction(OSInstance *source, OSInstance *destination)
{
    int numVar = source->getVariableNumber();

    destination->setObjectiveNumber(1);

    // Use a constant zero as the objective...

    SparseVector *newobjcoeff = new SparseVector(1);

    newobjcoeff->indexes[0] = 0;
    newobjcoeff->values[0] = 1.0;

    destination->addObjective(-1, "newobj", "min", 0.0, 1.0, newobjcoeff);
    delete newobjcoeff;
}