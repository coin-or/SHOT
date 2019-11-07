/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "PrimalSolver.h"

#include "Output.h"
#include "Results.h"
#include "Settings.h"
#include "Timing.h"

#include "Model/Problem.h"
#include "Model/ObjectiveFunction.h"
#include "Model/Constraints.h"

namespace SHOT
{

void PrimalSolver::addPrimalSolutionCandidate(VectorDouble pt, E_PrimalSolutionSource source, int iter)
{
    PrimalSolution sol;

    sol.point = pt;
    sol.sourceType = source;
    sol.objValue = env->problem->objectiveFunction->calculateValue(pt);
    sol.iterFound = iter;

    if(env->problem->properties.numberOfNonlinearConstraints > 0)
    {
        auto maxDevNonlinear = env->problem->getMaxNumericConstraintValue(pt, env->problem->nonlinearConstraints);
        sol.maxDevatingConstraintNonlinear
            = PairIndexValue(maxDevNonlinear.constraint->index, maxDevNonlinear.normalizedValue);
    }

    if(env->problem->properties.numberOfLinearConstraints > 0)
    {
        auto maxDevLinear = env->problem->getMaxNumericConstraintValue(pt, env->problem->linearConstraints);
        sol.maxDevatingConstraintLinear = PairIndexValue(maxDevLinear.constraint->index, maxDevLinear.normalizedValue);
    }

    env->primalSolver->primalSolutionCandidates.push_back(sol);

    this->checkPrimalSolutionCandidates();
}

void PrimalSolver::addPrimalSolutionCandidates(std::vector<VectorDouble> pts, E_PrimalSolutionSource source, int iter)
{
    for(auto& PT : pts)
    {
        addPrimalSolutionCandidate(PT, source, iter);
    }
}

void PrimalSolver::addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source)
{
    PrimalSolution sol;

    sol.point = pt.point;
    sol.sourceType = source;
    sol.objValue = pt.objectiveValue;
    sol.iterFound = pt.iterFound;

    env->primalSolver->primalSolutionCandidates.push_back(sol);

    this->checkPrimalSolutionCandidates();
}

void PrimalSolver::addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source)
{
    for(auto& PT : pts)
    {
        addPrimalSolutionCandidate(PT, source);
    }
}

void PrimalSolver::checkPrimalSolutionCandidates()
{
    env->timing->startTimer("PrimalStrategy");

    for(auto& cand : env->primalSolver->primalSolutionCandidates)
    {
        this->checkPrimalSolutionPoint(cand);
    }

    env->primalSolver->primalSolutionCandidates.clear();

    env->timing->stopTimer("PrimalStrategy");
}

bool PrimalSolver::checkPrimalSolutionPoint(PrimalSolution primalSol)
{
    std::string sourceDesc;

    VectorDouble tmpPoint(
        primalSol.point.begin(), primalSol.point.begin() + env->problem->properties.numberOfVariables);

    double tmpObjVal = primalSol.objValue;

    bool isVariableBoundsFulfilled = true;

    switch(primalSol.sourceType)
    {
    case E_PrimalSolutionSource::Rootsearch:
        sourceDesc = "line search";
        break;
    case E_PrimalSolutionSource::RootsearchFixedIntegers:
        sourceDesc = "line search fixed";
        break;
    case E_PrimalSolutionSource::NLPFixedIntegers:
        sourceDesc = "NLP fixed";
        break;
    case E_PrimalSolutionSource::MIPSolutionPool:
        sourceDesc = "MILP sol. pool";
        break;
    case E_PrimalSolutionSource::LPFixedIntegers:
        sourceDesc = "LP fixed";
        break;
    case E_PrimalSolutionSource::LazyConstraintCallback:
        sourceDesc = "lazy constraint callback";
        break;
    case E_PrimalSolutionSource::HeuristicCallback:
        sourceDesc = "heuristic constraint callback";
        break;
    case E_PrimalSolutionSource::IncumbentCallback:
        sourceDesc = "incumbent constraint callback";
        break;
    default:
        sourceDesc = "other";
        break;
    }

    primalSol.sourceDescription = sourceDesc;

    // Recalculate if the objective to be sure it is correct
    primalSol.objValue = env->problem->objectiveFunction->calculateValue(primalSol.point);
    tmpObjVal = primalSol.objValue;

    // Check that solution fulfills bounds, project back otherwise
    bool reCalculateObjective = false;

    for(auto& V : env->problem->realVariables)
    {
        auto value = V->calculate(tmpPoint);

        if(value > V->upperBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = V->upperBound;
        }
        else if(value < V->lowerBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = V->lowerBound;
        }
    }

    for(auto& V : env->problem->semicontinuousVariables)
    {
        auto value = V->calculate(tmpPoint);

        if(value == 0.0)
        {
        }
        else if(value > V->upperBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = V->upperBound;
        }
        else if(value < V->lowerBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = V->lowerBound;
        }
    }

    for(auto& V : env->problem->integerVariables)
    {
        auto value = V->calculate(tmpPoint);

        if(value > V->upperBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = round(V->upperBound - 0.5);
        }
        else if(value < V->lowerBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = round(V->lowerBound + 0.5);
        }
    }

    for(auto& V : env->problem->binaryVariables)
    {
        auto value = V->calculate(tmpPoint);

        if(value > V->upperBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = 1.0;
        }
        else if(value < V->lowerBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = 0.0;
        }
    }

    if(!isVariableBoundsFulfilled)
    {
        reCalculateObjective = true;
        env->output->outputDebug("       Variable bounds not fulfilled. Projection to bounds performed.");
        primalSol.boundProjectionPerformed = true;
    }
    else
    {
        env->output->outputDebug("       All variable bounds fulfilled.");
        primalSol.boundProjectionPerformed = false;
    }

    // Check that it fulfills integer constraints, round otherwise
    if(env->problem->properties.numberOfDiscreteVariables > 0)
    {
        auto integerTol = env->settings->getSetting<double>("Tolerance.Integer", "Primal");

        bool isRounded = false;

        VectorDouble ptRounded(tmpPoint);

        double maxIntegerError = 0.0;

        for(auto& V : env->problem->integerVariables)
        {
            auto value = V->calculate(tmpPoint);
            int index = V->index;

            double rounded = std::round(value);
            double error = std::abs(rounded - value);

            maxIntegerError = std::max(maxIntegerError, error);

            if(error > integerTol)
            {
                ptRounded.at(index) = rounded;
                isRounded = true;
            }
        }

        for(auto& V : env->problem->binaryVariables)
        {
            auto value = V->calculate(tmpPoint);
            int index = V->index;

            double rounded = std::round(value);
            double error = std::abs(rounded - value);

            maxIntegerError = std::max(maxIntegerError, error);

            if(error > integerTol)
            {
                ptRounded.at(index) = rounded;
                isRounded = true;
            }
        }

        if(isRounded)
        {
            reCalculateObjective = true;
            tmpPoint = ptRounded;

            auto tmpLine = fmt::format(
                "       Discrete variables were not fulfilled to tolerance {}. Rounding performed...", integerTol);
            env->output->outputDebug(tmpLine);
        }
        else
        {
            auto tmpLine = fmt::format("       All discrete variables are fulfilled to tolerance {}.", integerTol);
            env->output->outputDebug(tmpLine);
        }

        primalSol.integerRoundingPerformed = isRounded;
        primalSol.maxIntegerToleranceError = maxIntegerError;
    }

    // Recalculate the objective if rounding or projection has been performed
    if(reCalculateObjective)
    {
        tmpObjVal = env->problem->objectiveFunction->calculateValue(tmpPoint);
    }

    // For example rootsearches may violate linear constraints
    bool acceptableType = (primalSol.sourceType == E_PrimalSolutionSource::MIPSolutionPool
        || primalSol.sourceType == E_PrimalSolutionSource::NLPFixedIntegers
        || primalSol.sourceType == E_PrimalSolutionSource::IncumbentCallback
        || primalSol.sourceType == E_PrimalSolutionSource::LPFixedIntegers
        || primalSol.sourceType == E_PrimalSolutionSource::LazyConstraintCallback);

    if(!primalSol.integerRoundingPerformed && acceptableType
        && env->settings->getSetting<bool>("Tolerance.TrustLinearConstraintValues", "Primal"))
    {
        env->output->outputDebug(
            "       Assuming that linear constraints are fulfilled since solution is from a subsolver.");
    }
    else
    {
        PairIndexValue mostDevLinearConstraints;

        if(env->problem->properties.numberOfLinearConstraints > 0)
        {
            auto maxLinearConstraintValue
                = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->linearConstraints);

            mostDevLinearConstraints.index = maxLinearConstraintValue.constraint->index;
            mostDevLinearConstraints.value = maxLinearConstraintValue.normalizedValue;

            auto linTol = env->settings->getSetting<double>("Tolerance.LinearConstraint", "Primal");

            if(maxLinearConstraintValue.error > linTol)
            {
                auto tmpLine = fmt::format("       Linear constraints are not fulfilled. Most deviating {}: {} > {}.",
                    maxLinearConstraintValue.constraint->name, maxLinearConstraintValue.error, linTol);
                env->output->outputDebug(tmpLine);

                return (false);
            }
            else
            {
                auto tmpLine = fmt::format("       Linear constraints are fulfilled. Most deviating {}: {} > {}.",
                    maxLinearConstraintValue.constraint->name, maxLinearConstraintValue.error, linTol);
                env->output->outputDebug(tmpLine);
            }
        }

        primalSol.maxDevatingConstraintLinear = mostDevLinearConstraints;
    }

    // Check if quadratic constraints are fulfilled
    if(env->problem->properties.numberOfQuadraticConstraints > 0)
    {
        PairIndexValue mostDevQuadraticConstraints;

        auto maxQuadraticConstraintValue
            = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->quadraticConstraints);

        mostDevQuadraticConstraints.index = maxQuadraticConstraintValue.constraint->index;
        mostDevQuadraticConstraints.value = maxQuadraticConstraintValue.normalizedValue;

        auto nonlinTol = env->settings->getSetting<double>("Tolerance.NonlinearConstraint", "Primal");

        if(mostDevQuadraticConstraints.value > nonlinTol)
        {
            auto tmpLine = fmt::format("       Quadratic constraints are not fulfilled. Most deviating {}: {} > {}.",
                maxQuadraticConstraintValue.constraint->name, maxQuadraticConstraintValue.error, nonlinTol);
            env->output->outputDebug(tmpLine);

            return (false);
        }
        else
        {
            auto tmpLine = fmt::format("       Quadratic constraints are fulfilled. Most deviating {}: {} > {}.",
                maxQuadraticConstraintValue.constraint->name, maxQuadraticConstraintValue.error, nonlinTol);
            env->output->outputDebug(tmpLine);
        }

        primalSol.maxDevatingConstraintQuadratic = mostDevQuadraticConstraints;
    }

    // Check if nonlinear constraints are fulfilled
    if(env->problem->properties.numberOfNonlinearConstraints > 0)
    {
        PairIndexValue mostDevNonlinearConstraints;

        auto maxNonlinearConstraintValue
            = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->nonlinearConstraints);

        mostDevNonlinearConstraints.index = maxNonlinearConstraintValue.constraint->index;
        mostDevNonlinearConstraints.value = maxNonlinearConstraintValue.normalizedValue;

        auto nonlinTol = env->settings->getSetting<double>("Tolerance.NonlinearConstraint", "Primal");

        if(mostDevNonlinearConstraints.value > nonlinTol)
        {
            auto tmpLine = fmt::format("       Nonlinear constraints are not fulfilled. Most deviating {}: {} > {}.",
                maxNonlinearConstraintValue.constraint->name, mostDevNonlinearConstraints.value, nonlinTol);
            env->output->outputDebug(tmpLine);

            return (false);
        }
        else
        {
            auto tmpLine = fmt::format("       Nonlinear constraints are fulfilled. Most deviating {}: {} > {}.",
                maxNonlinearConstraintValue.constraint->name, mostDevNonlinearConstraints.value, nonlinTol);
            env->output->outputDebug(tmpLine);
        }

        primalSol.maxDevatingConstraintNonlinear = mostDevNonlinearConstraints;
    }

    primalSol.objValue = tmpObjVal;

    // Make sure no extra (auxiliary) values are in the vector
    if((int)tmpPoint.size() > env->problem->properties.numberOfVariables)
        tmpPoint.resize(env->problem->properties.numberOfVariables);

    primalSol.point = tmpPoint;

    env->results->addPrimalSolution(primalSol);

    return (true);
}

void PrimalSolver::addFixedNLPCandidate(
    VectorDouble pt, E_PrimalNLPSource source, double objVal, int iter, PairIndexValue maxConstrDev)
{
    assert((int)pt.size() >= env->problem->properties.numberOfVariables);

    fixedPrimalNLPCandidates.push_back(
        PrimalFixedNLPCandidate { VectorDouble(pt.begin(), pt.begin() + env->problem->properties.numberOfVariables),
            source, objVal, iter, maxConstrDev });
}

} // namespace SHOT