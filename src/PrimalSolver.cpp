/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "PrimalSolver.h"

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

    VectorDouble tmpPoint(primalSol.point);
    double tmpObjVal = primalSol.objValue;

    bool isVariableBoundsFulfilled = true;

    switch(primalSol.sourceType)
    {
    case E_PrimalSolutionSource::Linesearch:
        sourceDesc = "line search";
        break;
    case E_PrimalSolutionSource::LinesearchFixedIntegers:
        sourceDesc = "line search fixed";
        break;
    case E_PrimalSolutionSource::NLPFixedIntegers:
        sourceDesc = "NLP fixed";
        break;
    case E_PrimalSolutionSource::NLPRelaxed:
        sourceDesc = "NLP relaxed";
        break;
    case E_PrimalSolutionSource::MIPSolutionPool:
        sourceDesc = "MILP sol. pool";
        break;
    case E_PrimalSolutionSource::ObjectiveConstraint:
        sourceDesc = "obj. constr.";
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
        auto tmpLine = boost::format("       Variable bounds not fulfilled. Projection to bounds performed.");
        env->output->outputDebug(tmpLine.str());
        primalSol.boundProjectionPerformed = true;
    }
    else
    {
        auto tmpLine = boost::format("       All variable bounds fulfilled.");
        env->output->outputDebug(tmpLine.str());
        primalSol.boundProjectionPerformed = false;
    }

    // Check that it fulfills integer constraints, round otherwise
    if(env->problem->properties.numberOfDiscreteVariables > 0)
    {
        auto integerTol = env->settings->getDoubleSetting("Tolerance.Integer", "Primal");

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

            auto tmpLine
                = boost::format("       Discrete variables were not fulfilled to tolerance %1%. Rounding performed...")
                % integerTol;
            env->output->outputDebug(tmpLine.str());
        }
        else
        {
            auto tmpLine = boost::format("       All discrete variables are fulfilled to tolerance %1%.") % integerTol;
            env->output->outputDebug(tmpLine.str());
        }

        primalSol.integerRoundingPerformed = isRounded;
        primalSol.maxIntegerToleranceError = maxIntegerError;
    }

    // Recalculate the objective if rounding or projection has been performed
    if(reCalculateObjective)
    {
        tmpObjVal = env->problem->objectiveFunction->calculateValue(tmpPoint);
    }

    // Check if primal bound is worse than current
    if((env->problem->objectiveFunction->properties.isMinimize && tmpObjVal < env->results->getPrimalBound())
        || (!env->problem->objectiveFunction->properties.isMinimize && tmpObjVal > env->results->getPrimalBound()))
    {
        auto tmpLine = boost::format("     Testing primal bound %1% found from %2%:") % tmpObjVal % sourceDesc;
        env->output->outputDebug(tmpLine.str());
    }
    else
    {
        auto tmpLine
            = boost::format("     Primal bound candidate (%1%) from %2% is not an improvement over current (%3%).")
            % tmpObjVal % sourceDesc % env->results->getPrimalBound();
        env->output->outputDebug(tmpLine.str());

        return (false);
    }

    // For example linesearches may violate linear constraints
    bool acceptableType = (primalSol.sourceType == E_PrimalSolutionSource::MIPSolutionPool
        || primalSol.sourceType == E_PrimalSolutionSource::NLPFixedIntegers
        || primalSol.sourceType == E_PrimalSolutionSource::NLPRelaxed
        || primalSol.sourceType == E_PrimalSolutionSource::IncumbentCallback
        || primalSol.sourceType == E_PrimalSolutionSource::LPFixedIntegers
        || primalSol.sourceType == E_PrimalSolutionSource::LazyConstraintCallback);

    if(acceptableType && env->settings->getBoolSetting("Tolerance.TrustLinearConstraintValues", "Primal"))
    {
        auto tmpLine = boost::format(
            "       Assuming that linear constraints are fulfilled since solution is from a subsolver.");
        env->output->outputDebug(tmpLine.str());
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

            auto linTol = env->settings->getDoubleSetting("Tolerance.LinearConstraint", "Primal");

            if(maxLinearConstraintValue.error > linTol)
            {
                auto tmpLine
                    = boost::format("       Linear constraints are not fulfilled. Most deviating %3%: %2% > %1%.")
                    % linTol % maxLinearConstraintValue.error % maxLinearConstraintValue.constraint->name;
                env->output->outputDebug(tmpLine.str());

                return (false);
            }
            else
            {
                auto tmpLine = boost::format("       Linear constraints are fulfilled. Most deviating %3%: %2% < %1%.")
                    % linTol % maxLinearConstraintValue.error % maxLinearConstraintValue.constraint->name;
                env->output->outputDebug(tmpLine.str());
            }
        }

        primalSol.maxDevatingConstraintLinear = mostDevLinearConstraints;
    }

    PairIndexValue mostDevQuadraticConstraints;

    if(env->problem->properties.numberOfQuadraticConstraints > 0)
    {
        auto maxQuadraticConstraintValue
            = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->quadraticConstraints);

        mostDevQuadraticConstraints.index = maxQuadraticConstraintValue.constraint->index;
        mostDevQuadraticConstraints.value = maxQuadraticConstraintValue.normalizedValue;

        auto nonlinTol = env->settings->getDoubleSetting("Tolerance.NonlinearConstraint", "Primal");

        if(mostDevQuadraticConstraints.value > nonlinTol)
        {
            auto tmpLine
                = boost::format("       Quadratic constraints are not fulfilled. Most deviating %3%: %2% > %1%.")
                % nonlinTol % mostDevQuadraticConstraints.value % maxQuadraticConstraintValue.constraint->name;
            env->output->outputDebug(tmpLine.str());

            return (false);
        }
        else
        {
            auto tmpLine = boost::format("       Quadratic constraints are fulfilled. Most deviating %3%: %2% < %1%.")
                % nonlinTol % mostDevQuadraticConstraints.value % maxQuadraticConstraintValue.constraint->name;
            env->output->outputDebug(tmpLine.str());
        }
    }

    primalSol.maxDevatingConstraintQuadratic = mostDevQuadraticConstraints;

    PairIndexValue mostDevNonlinearConstraints;

    if(env->problem->properties.numberOfNonlinearConstraints > 0)
    {
        auto maxNonlinearConstraintValue
            = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->nonlinearConstraints);

        mostDevNonlinearConstraints.index = maxNonlinearConstraintValue.constraint->index;
        mostDevNonlinearConstraints.value = maxNonlinearConstraintValue.normalizedValue;

        auto nonlinTol = env->settings->getDoubleSetting("Tolerance.NonlinearConstraint", "Primal");

        if(mostDevNonlinearConstraints.value > nonlinTol)
        {
            auto tmpLine
                = boost::format("       Nonlinear constraints are not fulfilled. Most deviating %3%: %2% > %1%.")
                % nonlinTol % mostDevNonlinearConstraints.value % maxNonlinearConstraintValue.constraint->name;
            env->output->outputDebug(tmpLine.str());

            return (false);
        }
        else
        {
            auto tmpLine = boost::format("       Nonlinear constraints are fulfilled. Most deviating %3%: %2% < %1%.")
                % nonlinTol % mostDevNonlinearConstraints.value % maxNonlinearConstraintValue.constraint->name;
            env->output->outputDebug(tmpLine.str());
        }
    }

    primalSol.objValue = tmpObjVal;
    primalSol.point = tmpPoint;
    primalSol.maxDevatingConstraintNonlinear = mostDevNonlinearConstraints;

    auto tmpLine = boost::format("        New primal bound %1% from %2% accepted.") % tmpObjVal % sourceDesc;
    env->output->outputCritical(tmpLine.str());

    env->results->addPrimalSolution(primalSol);

    return (true);
}

} // namespace SHOT