/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverBase.h"

namespace SHOT
{

MIPSolverBase::~MIPSolverBase()
{
    lastSolutions.clear();
}

double MIPSolverBase::getObjectiveValue()
{
    double objval = getObjectiveValue(0);
    return (objval);
}

bool MIPSolverBase::getDiscreteVariableStatus()
{
    if (env->problem->properties.numberOfDiscreteVariables == 0)
    {
        return (false);
    }
    else
    {
        return (discreteVariablesActivated);
    }
}

std::vector<SolutionPoint> MIPSolverBase::getAllVariableSolutions()
{
    if (cachedSolutionHasChanged == false)
        return (lastSolutions);

    int numSol = getNumberOfSolutions();

    int numVar = env->reformulatedProblem->properties.numberOfVariables;
    std::vector<SolutionPoint> lastSolutions(numSol);

    for (int i = 0; i < numSol; i++)
    {
        SolutionPoint tmpSolPt;

        auto tmpPt = getVariableSolution(i);

        while (tmpPt.size() > env->reformulatedProblem->properties.numberOfVariables)
        {
            tmpPt.pop_back();
        }

        tmpSolPt.point = tmpPt;

        tmpSolPt.objectiveValue = getObjectiveValue(i);
        tmpSolPt.iterFound = env->process->getCurrentIteration()->iterationNumber;

        if (env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
        {
            auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(tmpPt, env->reformulatedProblem->nonlinearConstraints);
            tmpSolPt.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
        }

        lastSolutions.at(i) = tmpSolPt;
    }

    cachedSolutionHasChanged = false;

    return (lastSolutions);
}

void MIPSolverBase::createHyperplane(Hyperplane hyperplane)
{
    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

    auto optional = createHyperplaneTerms(hyperplane);

    if (!optional)
    {
        return;
    }

    auto tmpPair = optional.get();

    bool hyperplaneIsOk = true;

    for (auto E : tmpPair.first)
    {
        if (E.value != E.value) //Check for NaN
        {
            env->output->outputError(
                "     Warning: hyperplane not generated, NaN found in linear terms!");
            hyperplaneIsOk = false;
            break;
        }

        if (isinf(E.value))
        {
            env->output->outputError(
                "     Warning: hyperplane not generated, inf found in linear terms!");
            hyperplaneIsOk = false;
            break;
        }
    }

    if (hyperplaneIsOk)
    {
        GeneratedHyperplane genHyperplane;

        int constrIndex = addLinearConstraint(tmpPair.first, tmpPair.second);

        /*genHyperplane.generatedConstraintIndex = constrIndex;
        genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
        genHyperplane.generatedPoint = hyperplane.generatedPoint;
        genHyperplane.source = hyperplane.source;
        genHyperplane.generatedIter = currIter->iterationNumber;
        genHyperplane.isLazy = false;
        genHyperplane.isRemoved = false;

        generatedHyperplanes.push_back(genHyperplane);*/

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
    }
}

boost::optional<std::pair<std::vector<PairIndexValue>, double>> MIPSolverBase::createHyperplaneTerms(Hyperplane hyperplane)
{
    std::vector<PairIndexValue> elements;
    double constant = 0.0;
    SparseVariableVector gradient;
    double signFactor = 1.0; // Will be -1.0 for greater than constraints

    if (hyperplane.isObjectiveHyperplane)
    {
        constant = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)->calculateValue(hyperplane.generatedPoint);
        gradient = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)->calculateGradient(hyperplane.generatedPoint);

        PairIndexValue pair;
        pair.index = auxilliaryObjectiveVariableIndex;
        pair.value = -1.0;

        elements.push_back(pair);

        env->output->outputInfo("     HP point generated for objective function with " + std::to_string(gradient.size()) + " elements.");
    }
    else
    {
        assert(hyperplane.sourceConstraint);
        auto maxDev = hyperplane.sourceConstraint->calculateNumericValue(hyperplane.generatedPoint);

        if (maxDev.isFulfilledRHS && !maxDev.isFulfilledLHS)
        {
            signFactor = -1.0;
            constant = signFactor * maxDev.normalizedLHSValue;
        }
        else
        {
            constant = maxDev.normalizedRHSValue;
        }

        gradient = hyperplane.sourceConstraint->calculateGradient(hyperplane.generatedPoint);
        env->output->outputInfo("     HP point generated for constraint index " + std::to_string(hyperplane.sourceConstraintIndex) + " with " + std::to_string(gradient.size()) + " elements. Constraint error: " + std::to_string(constant) + ".");
    }

    for (auto const &G : gradient)
    {
        PairIndexValue pair;
        pair.index = G.first->index;
        pair.value = signFactor * G.second;

        elements.push_back(pair);

        constant += -G.second * hyperplane.generatedPoint.at(G.first->index);

        env->output->outputInfo("     Gradient for variable " + G.first->name + " in point " + std::to_string(hyperplane.generatedPoint.at(G.first->index)) + ": " + std::to_string(G.second));
    }

    boost::optional<std::pair<std::vector<PairIndexValue>, double>> optional;

    if (elements.size() > 0)
        optional = std::make_pair(elements, constant);

    elements.clear();

    return (optional);
};

void MIPSolverBase::createInteriorHyperplane(Hyperplane hyperplane)
{
    /*
    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration
    std::vector<PairIndexValue> elements;

    double constant = env->model->originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
                                                                                    hyperplane.generatedPoint);

    auto tmpArray = env->model->originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
        &hyperplane.generatedPoint.at(0), -1, true);
    int number = env->model->originalProblem->getNumberOfVariables();
    env->solutionStatistics.numberOfGradientEvaluations++;

    for (int i = 0; i < number - 1; i++)
    {
        if (tmpArray[i] != 0)
        {
            PairIndexValue pair;
            pair.index = i;
            pair.value = tmpArray[i];

            elements.push_back(pair);
            constant += -tmpArray[i] * hyperplane.generatedPoint.at(i);
        }
    }

    PairIndexValue pair;
    pair.index = env->model->originalProblem->getNonlinearObjectiveVariableIdx();
    pair.value = -1.0;

    elements.push_back(pair);
    constant += hyperplane.generatedPoint.at(pair.index);

    bool hyperplaneIsOk = true;

    for (auto E : elements)
    {
        if (E.value != E.value) //Check for NaN
        {
            env->output->outputWarning(
                "     Warning: hyperplane not generated, NaN found in linear terms!");

            hyperplaneIsOk = false;
            break;
        }
    }

    if (hyperplaneIsOk)
    {
        int constrIndex = addLinearConstraint(elements, constant, false);
        GeneratedHyperplane genHyperplane;

        genHyperplane.generatedConstraintIndex = constrIndex;
        genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
        genHyperplane.generatedPoint = hyperplane.generatedPoint;
        genHyperplane.source = hyperplane.source;
        genHyperplane.generatedIter = currIter->iterationNumber;

        generatedHyperplanes.push_back(genHyperplane);

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
    }

    currIter->totNumHyperplanes = env->process->getPreviousIteration()->totNumHyperplanes + currIter->numHyperplanesAdded;*/
}

std::vector<GeneratedHyperplane> *MIPSolverBase::getGeneratedHyperplanes()
{
    return (&generatedHyperplanes);
}

void MIPSolverBase::presolveAndUpdateBounds()
{
    /*
    auto newBounds = this->presolveAndGetNewBounds();

    for (int i = 0; i < numberOfVariables; i++)
    {
        auto currBounds = this->getCurrentVariableBounds(i);

        bool newLB = false;
        bool newUB = false;

        if (newBounds.first.at(i) > currBounds.first)
            newLB = true;
        if (newBounds.second.at(i) > currBounds.second)
            newUB = true;

        if (newLB)
        {
            env->model->originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
            env->output->outputInfo(
                "     Lower bound for variable (" + std::to_string(i) + ") updated from " + UtilityFunctions::toString(currBounds.first) + " to " + UtilityFunctions::toString(newBounds.first.at(i)));

            if (!originalProblem->hasVariableBoundsBeenTightened(i))
            {
                env->model->originalProblem->setVariableBoundsAsTightened(i);
                env->solutionStatistics.numberOfVariableBoundsTightenedInPresolve++;
            }
        }

        if (newUB)
        {
            env->model->originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
            env->output->outputInfo(
                "     Upper bound for variable (" + std::to_string(i) + ") updated from " + UtilityFunctions::toString(currBounds.second) + " to " + UtilityFunctions::toString(newBounds.second.at(i)));

            if (!originalProblem->hasVariableBoundsBeenTightened(i))
            {
                env->model->originalProblem->setVariableBoundsAsTightened(i);
                env->solutionStatistics.numberOfVariableBoundsTightenedInPresolve++;
            }
        }

        if (env->settings->getBoolSetting("MIP.Presolve.UpdateObtainedBounds", "Dual") && (newLB || newUB))
        {
            updateVariableBound(i, newBounds.first.at(i), newBounds.second.at(i));
            env->output->outputInfo("     Bounds updated also in MIP problem");
        }
    }*/
}

void MIPSolverBase::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    if (isVariablesFixed)
    {
        unfixVariables();
    }

    int size = variableIndexes.size();

    if (size == 0)
        return;

    std::vector<PairDouble> originalBounds(size);

    activateDiscreteVariables(false);

    for (int i = 0; i < size; i++)
    {
        originalBounds.at(i) = this->getCurrentVariableBounds(variableIndexes.at(i));
        this->fixVariable(variableIndexes.at(i), variableValues.at(i));
    }

    fixedVariableIndexes = variableIndexes;
    fixedVariableOriginalBounds = originalBounds;

    isVariablesFixed = true;
}

void MIPSolverBase::unfixVariables()
{
    for (int i = 0; i < fixedVariableIndexes.size(); i++)
    {
        updateVariableBound(fixedVariableIndexes.at(i), fixedVariableOriginalBounds.at(i).first,
                            fixedVariableOriginalBounds.at(i).second);
    }

    isVariablesFixed = false;
}

void MIPSolverBase::createIntegerCut(VectorInteger binaryIndexes)
{
    std::vector<PairIndexValue> elements;

    for (int i = 0; i < binaryIndexes.size(); i++)
    {
        PairIndexValue pair;
        pair.index = binaryIndexes.at(i);
        pair.value = 1.0;

        elements.push_back(pair);
    }

    this->addLinearConstraint(elements, -(binaryIndexes.size() - 1.0));
    env->solutionStatistics.numberOfIntegerCuts++;
}

int MIPSolverBase::getNumberOfOpenNodes()
{
    return (env->solutionStatistics.numberOfOpenNodes);
}
} // namespace SHOT