/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverBase.h"
#include "../Model/Problem.h"
#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"

namespace SHOT
{

MIPSolverBase::~MIPSolverBase() { lastSolutions.clear(); }

double MIPSolverBase::getObjectiveValue()
{
    double objval = getObjectiveValue(0);
    return (objval);
}

E_DualProblemClass MIPSolverBase::getProblemClass()
{
    bool isMIP = getDiscreteVariableStatus();

    if(hasQuadraticObjective && hasQudraticConstraint)
        return (isMIP ? E_DualProblemClass::MIQCQP : E_DualProblemClass::QCQP);
    else if(hasQuadraticObjective)
        return (isMIP ? E_DualProblemClass::MIQP : E_DualProblemClass::QP);
    else if(hasQudraticConstraint)
        return (isMIP ? E_DualProblemClass::MIQCQP : E_DualProblemClass::QCQP);
    else
        return (isMIP ? E_DualProblemClass::MIP : E_DualProblemClass::LP);
}

bool MIPSolverBase::getDiscreteVariableStatus()
{
    if(env->reformulatedProblem->properties.numberOfDiscreteVariables == 0
        && env->reformulatedProblem->properties.numberOfSemicontinuousVariables == 0
        && env->reformulatedProblem->properties.numberOfSpecialOrderedSets == 0
        && env->reformulatedProblem->properties.numberOfSemiintegerVariables == 0)
    {
        return (false);
    }
    else
    {
        return (discreteVariablesActivated);
    }
}

void MIPSolverBase::executeRelaxationStrategy()
{
    if(this->relaxationStrategy == nullptr || this->relaxationStrategy.get() == nullptr)
    {
        relaxationStrategy = std::make_unique<RelaxationStrategyStandard>(env);
    }

    MIPSolverBase::relaxationStrategy->executeStrategy();
}

std::vector<SolutionPoint> MIPSolverBase::getAllVariableSolutions()
{
    if(cachedSolutionHasChanged == false)
        return (lastSolutions);

    int numSol = getNumberOfSolutions();

    std::vector<SolutionPoint> lastSolutions(numSol);

    for(int i = 0; i < numSol; i++)
    {
        SolutionPoint tmpSolPt;

        auto tmpPt = getVariableSolution(i);

        while((int)tmpPt.size() > env->reformulatedProblem->properties.numberOfVariables)
        {
            tmpPt.pop_back();
        }

        tmpSolPt.point = tmpPt;
        tmpSolPt.hashValue = Utilities::calculateHash(tmpPt);

        tmpSolPt.objectiveValue = getObjectiveValue(i);
        tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;

        if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
        {
            auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
                tmpPt, env->reformulatedProblem->nonlinearConstraints);
            tmpSolPt.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
        }
        else
        {
            tmpSolPt.maxDeviation = PairIndexValue(-1, 0.0);
        }

        lastSolutions.at(i) = tmpSolPt;
    }

    cachedSolutionHasChanged = false;

    return (lastSolutions);
}

bool MIPSolverBase::createHyperplane(HyperplanePtr hyperplane)
{
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration
    auto optionalHyperplanes = createHyperplaneTerms(hyperplane);

    if(!optionalHyperplanes)
    {
        return (false);
    }

    std::string identifier = getConstraintIdentifier(hyperplane->source);

    if(auto constraintHyperplane = std::dynamic_pointer_cast<ConstraintHyperplane>(hyperplane))
    {
        identifier += "_" + constraintHyperplane->sourceConstraint->name;
    }
    else if(auto externalHyperplane = std::dynamic_pointer_cast<ExternalHyperplane>(hyperplane))
    {
        identifier += "_" + externalHyperplane->description;
    }

    identifier += "_" + std::to_string(constraintCounter);
    constraintCounter++;

    auto tmpPair = optionalHyperplanes.value();

    if(auto numericHyperplane = std::dynamic_pointer_cast<NumericHyperplane>(hyperplane))
    {
        for(auto& E : tmpPair.first)
        {
            if(E.second != E.second || std::isinf(E.second)) // Check for NaN or inf
            {
                env->output->outputError("        Warning: hyperplane not generated, NaN or inf "
                                         "found in linear terms for "
                    + env->reformulatedProblem->getVariable(E.first)->name + " = "
                    + std::to_string(numericHyperplane->generatedPoint.at(E.first)));

                return (false);
            }
        }
    }
    else if(auto externalHyperplane = std::dynamic_pointer_cast<ExternalHyperplane>(hyperplane))
    {
        for(auto& E : tmpPair.first)
        {
            if(E.second != E.second || std::isinf(E.second)) // Check for NaN or inf
            {
                env->output->outputError("        Warning: external hyperplane not generated, NaN or inf "
                                         "found in linear terms for "
                    + env->reformulatedProblem->getVariable(E.first)->name);

                return (false);
            }
        }
    }

    // Small fix to fix badly scaled cuts.
    // TODO: this should be made so it also takes into account small/large coefficients of the linear terms
    if(abs(tmpPair.second) > 1e15)
    {
        double scalingFactor = abs(tmpPair.second) - 1e15;

        for(auto& E : tmpPair.first)
            E.second /= scalingFactor;

        tmpPair.second /= scalingFactor;

        if(!warningMessageShownLargeRHS)
        {
            env->output->outputWarning(
                "        Large values found in RHS of cut, you might want to consider reducing the "
                "bounds of the nonlinear variables.");
            warningMessageShownLargeRHS = true;
        }
    }

    if(addLinearConstraint(tmpPair.first, tmpPair.second, identifier, false, !hyperplane->isGlobal) < 0)
        return (false);

    return (true);
}

std::optional<std::pair<std::map<int, double>, double>> MIPSolverBase::createHyperplaneTerms(HyperplanePtr hyperplane)
{
    std::map<int, double> elements;
    double constant = 0.0;
    SparseVariableVector gradient;
    double signFactor = 1.0; // Will be -1.0 for greater than constraints

    if(auto objectiveHP = std::dynamic_pointer_cast<ObjectiveHyperplane>(hyperplane))
    {
        if(env->reformulatedProblem->objectiveFunction->properties.hasNonlinearExpression)
        {
            gradient
                = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->calculateGradient(objectiveHP->generatedPoint, true);
        }
        else
        {
            gradient
                = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->calculateGradient(objectiveHP->generatedPoint, true);
        }

        elements.emplace(dualAuxiliaryObjectiveVariableIndex, -1.0);

        env->output->outputTrace("        HP point generated for objective function with "
            + std::to_string(gradient.size()) + " elements and constant " + std::to_string(constant));
    }
    else if(auto constraintHyperplane = std::dynamic_pointer_cast<ConstraintHyperplane>(hyperplane))
    {
        assert(constraintHyperplane->sourceConstraint != nullptr);
        auto maxDev
            = constraintHyperplane->sourceConstraint->calculateNumericValue(constraintHyperplane->generatedPoint);

        if(maxDev.isFulfilledRHS && !maxDev.isFulfilledLHS)
        {
            signFactor = -1.0;
            constant = maxDev.normalizedLHSValue;
        }
        else
        {
            constant = maxDev.normalizedRHSValue;
        }

        gradient = std::dynamic_pointer_cast<NonlinearConstraint>(constraintHyperplane->sourceConstraint)
                       ->calculateGradient(constraintHyperplane->generatedPoint, true);

        auto nonzeroes
            = std::count_if(gradient.begin(), gradient.end(), [](auto element) { return (element.second != 0.0); });

        if(nonzeroes == 0)
        {
            gradient = std::dynamic_pointer_cast<NonlinearConstraint>(constraintHyperplane->sourceConstraint)
                           ->calculateGradient(constraintHyperplane->generatedPoint, false);

            double eps = 0.000001;

            for(auto& G : gradient)
            {
                if(G.second == 0.0)
                    G.second = eps;
            }

            env->output->outputDebug("        All gradients nonzero, adding tolerance.");
        }

        env->output->outputTrace("        HP point generated for constraint index "
            + std::to_string(constraintHyperplane->sourceConstraint->index) + " with " + std::to_string(gradient.size())
            + " elements.");
    }
    else if(auto externalHyperplane = std::dynamic_pointer_cast<ExternalHyperplane>(hyperplane))
    {
        int addedElements = 0;

        for(size_t i = 0; i < externalHyperplane->variableIndexes.size(); i++)
        {
            auto variableIndex = externalHyperplane->variableIndexes.at(i);
            auto coefficient = externalHyperplane->variableCoefficients.at(i);

            if(coefficient != 0.0)
            {
                auto element = elements.emplace(variableIndex, coefficient);

                if(!element.second)
                {
                    // Element already exists for the variable
                    element.first->second += coefficient;
                }
                else
                {
                    addedElements++;
                }
            }
        }

        constant = -externalHyperplane->rhsValue;

        env->output->outputInfo("        HP generated for external hyperplane " + externalHyperplane->description
            + " with " + std::to_string(addedElements) + " terms.");
    }
    else
    {
        env->output->outputError("        Hyperplane type not supported!");
        return (std::nullopt);
    }

    if(auto numericHyperplane = std::dynamic_pointer_cast<NumericHyperplane>(hyperplane))
    {
        for(auto const& G : gradient)
        {
            double coefficient = signFactor * G.second;
            int variableIndex = G.first->index;

            auto element = elements.emplace(variableIndex, coefficient);

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += coefficient;
            }

            constant += signFactor * (-G.second) * numericHyperplane->generatedPoint.at(variableIndex);

            env->output->outputTrace("         Gradient for variable " + G.first->name + " in point "
                + std::to_string(numericHyperplane->generatedPoint.at(variableIndex)) + ": "
                + std::to_string(coefficient));
        }
    }

    std::optional<std::pair<std::map<int, double>, double>> optional;

    if(elements.size() > 0)
        optional = std::make_pair(elements, constant);

    elements.clear();

    return (optional);
}

bool MIPSolverBase::createInteriorHyperplane([[maybe_unused]] HyperplanePtr hyperplane)
{
    /*
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration
    std::vector<PairIndexValue> elements;

    double constant =
    env->model->originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
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

    currIter->totNumHyperplanes = env->results->getPreviousIteration()->totNumHyperplanes +
    currIter->numHyperplanesAdded;*/

    return (false);
}

void MIPSolverBase::presolveAndUpdateBounds()
{
    auto newBounds = this->presolveAndGetNewBounds();

    for(int i = 0; i < env->reformulatedProblem->properties.numberOfVariables; i++)
    {
        auto currBounds = this->getCurrentVariableBounds(i);

        bool newLB = false;
        bool newUB = false;

        if(newBounds.first.at(i) > currBounds.first)
            newLB = true;
        if(newBounds.second.at(i) > currBounds.second)
            newUB = true;

        if(newLB)
        {
            env->reformulatedProblem->getVariable(i)->lowerBound = newBounds.first.at(i);
            env->output->outputDebug("        Lower bound for variable (" + std::to_string(i) + ") updated from "
                + Utilities::toString(currBounds.first) + " to " + Utilities::toString(newBounds.first.at(i)));

            if(!env->reformulatedProblem->allVariables[i]->properties.hasLowerBoundBeenTightened)
            {
                env->reformulatedProblem->allVariables[i]->properties.hasLowerBoundBeenTightened = true;
                env->solutionStatistics.numberOfVariableBoundsTightenedInPresolve++;
            }
        }

        if(newUB)
        {
            env->reformulatedProblem->getVariable(i)->upperBound = newBounds.second.at(i);
            env->output->outputDebug("        Upper bound for variable (" + std::to_string(i) + ") updated from "
                + Utilities::toString(currBounds.second) + " to " + Utilities::toString(newBounds.second.at(i)));

            if(!env->reformulatedProblem->allVariables[i]->properties.hasUpperBoundBeenTightened)
            {
                env->reformulatedProblem->allVariables[i]->properties.hasUpperBoundBeenTightened = true;
                env->solutionStatistics.numberOfVariableBoundsTightenedInPresolve++;
            }
        }

        if(env->settings->getSetting<bool>("MIP.Presolve.UpdateObtainedBounds", "Dual") && (newLB || newUB))
        {
            updateVariableBound(i, newBounds.first.at(i), newBounds.second.at(i));
            env->output->outputDebug("        Bounds updated also in MIP problem");
        }
    }
}

void MIPSolverBase::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    if(isVariablesFixed)
    {
        unfixVariables();
    }

    auto size = variableIndexes.size();

    if(size == 0)
        return;

    std::vector<PairDouble> originalBounds(size);

    activateDiscreteVariables(false);

    for(size_t i = 0; i < size; i++)
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
    for(size_t i = 0; i < fixedVariableIndexes.size(); i++)
    {
        updateVariableBound(fixedVariableIndexes.at(i), fixedVariableOriginalBounds.at(i).first,
            fixedVariableOriginalBounds.at(i).second);
    }

    isVariablesFixed = false;
}

int MIPSolverBase::getNumberOfOpenNodes() { return (env->solutionStatistics.numberOfOpenNodes); }
} // namespace SHOT