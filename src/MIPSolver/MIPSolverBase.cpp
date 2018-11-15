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
    if (env->model->originalProblem->getNumberOfDiscreteVariables() == 0)
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

    int numVar = env->model->originalProblem->getNumberOfVariables();
    std::vector<SolutionPoint> lastSolutions(numSol);

    for (int i = 0; i < numSol; i++)
    {
        SolutionPoint tmpSolPt;

        auto tmpPt = getVariableSolution(i);

        auto maxDev = env->model->originalProblem->getMostDeviatingConstraint(tmpPt);

        tmpSolPt.point = tmpPt;
        tmpSolPt.objectiveValue = getObjectiveValue(i);
        tmpSolPt.iterFound = env->process->getCurrentIteration()->iterationNumber;
        tmpSolPt.maxDeviation = maxDev;

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
    auto varNames = env->model->originalProblem->getVariableNames();

    std::vector<PairIndexValue> elements;

    double constant = env->model->originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex, hyperplane.generatedPoint);
    auto nablag = env->model->originalProblem->calculateConstraintFunctionGradient(hyperplane.sourceConstraintIndex, hyperplane.generatedPoint);

    env->output->outputInfo("     HP point generated for constraint index " + std::to_string(hyperplane.sourceConstraintIndex) + " with " + std::to_string(nablag->number) + " elements. Constraint error: " + std::to_string(constant) + ".");

    for (int i = 0; i < nablag->number; i++)
    {
        PairIndexValue pair;
        pair.index = nablag->indexes[i];
        pair.value = nablag->values[i];

        elements.push_back(pair);

        constant += -nablag->values[i] * hyperplane.generatedPoint.at(nablag->indexes[i]);

        env->output->outputInfo("     Gradient for variable " + varNames.at(nablag->indexes[i]) + " in point " + std::to_string(hyperplane.generatedPoint.at(nablag->indexes[i])) + ": " + std::to_string(nablag->values[i]));
    }

    boost::optional<std::pair<std::vector<PairIndexValue>, double>> optional;
    if (elements.size() > 0)
        optional = std::make_pair(elements, constant);

    delete nablag;

    elements.clear();
    varNames.clear();

    return (optional);
}

void MIPSolverBase::createInteriorHyperplane(Hyperplane hyperplane)
{
    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration
    std::vector<PairIndexValue> elements;

    auto varNames = env->model->originalProblem->getVariableNames();

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

    currIter->totNumHyperplanes = env->process->getPreviousIteration()->totNumHyperplanes + currIter->numHyperplanesAdded;
}

std::vector<GeneratedHyperplane> *MIPSolverBase::getGeneratedHyperplanes()
{
    return (&generatedHyperplanes);
}

void MIPSolverBase::presolveAndUpdateBounds()
{
    auto newBounds = this->presolveAndGetNewBounds();

    auto numVar = env->model->originalProblem->getNumberOfVariables();

    for (int i = 0; i < numVar; i++)
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
    }
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

void MIPSolverBase::updateNonlinearObjectiveFromPrimalDualBounds()
{
    if (!originalProblem->isObjectiveFunctionNonlinear())
    {
        return;
    }

    auto varIdx = env->model->originalProblem->getNonlinearObjectiveVariableIdx();

    auto newLB = env->process->getDualBound();
    auto newUB = env->process->getPrimalBound();

    auto currBounds = this->getCurrentVariableBounds(varIdx);

    if (newLB > currBounds.first || newUB < currBounds.second)
    {
        this->updateVariableBound(varIdx, newLB, newUB);
        env->output->outputInfo(
            "     Bounds for nonlinear objective function updated to " + UtilityFunctions::toString(newLB) + " and " + UtilityFunctions::toString(newUB));
    }
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