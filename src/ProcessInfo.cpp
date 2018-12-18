/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "ProcessInfo.h"

namespace SHOT
{

void ProcessInfo::addDualSolution(DualSolution solution)
{
    if (dualSolutions.size() == 0)
    {
        dualSolutions.push_back(solution);
    }
    else
    {
        dualSolutions.at(0) = solution;
    }
}

void ProcessInfo::addPrimalSolutionCandidate(VectorDouble pt, E_PrimalSolutionSource source, int iter)
{
    PrimalSolution sol;

    sol.point = pt;
    sol.sourceType = source;
    sol.objValue = env->problem->objectiveFunction->calculateValue(pt);
    sol.iterFound = iter;

    if (env->problem->properties.numberOfNonlinearConstraints > 0)
    {
        auto maxDevNonlinear = env->problem->getMaxNumericConstraintValue(pt, env->problem->nonlinearConstraints);
        sol.maxDevatingConstraintNonlinear = PairIndexValue(maxDevNonlinear.constraint->index, maxDevNonlinear.normalizedValue);
    }

    if (env->problem->properties.numberOfLinearConstraints > 0)
    {
        auto maxDevLinear = env->problem->getMaxNumericConstraintValue(pt, env->problem->linearConstraints);
        sol.maxDevatingConstraintLinear = PairIndexValue(maxDevLinear.constraint->index, maxDevLinear.normalizedValue);
    }

    primalSolutionCandidates.push_back(sol);

    this->checkPrimalSolutionCandidates();
}

void ProcessInfo::addPrimalSolutionCandidates(std::vector<VectorDouble> pts, E_PrimalSolutionSource source, int iter)
{
    for (auto PT : pts)
    {
        addPrimalSolutionCandidate(PT, source, iter);
    }
}

void ProcessInfo::addDualSolutionCandidate(DualSolution solution)
{
    dualSolutionCandidates.push_back(solution);

    this->checkDualSolutionCandidates();
}

void ProcessInfo::addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source)
{
    PrimalSolution sol;

    sol.point = pt.point;
    sol.sourceType = source;
    sol.objValue = pt.objectiveValue;
    sol.iterFound = pt.iterFound;

    primalSolutionCandidates.push_back(sol);

    this->checkPrimalSolutionCandidates();
}

void ProcessInfo::addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source)
{
    for (auto pt : pts)
    {
        addPrimalSolutionCandidate(pt, source);
    }
}

void ProcessInfo::addPrimalFixedNLPCandidate(VectorDouble pt, E_PrimalNLPSource source, double objVal, int iter,
                                             PairIndexValue maxConstrDev)
{
    PrimalFixedNLPCandidate cand =
        {pt, source, objVal, iter};

    primalFixedNLPCandidates.push_back(cand);
}

void ProcessInfo::checkPrimalSolutionCandidates()
{
    this->startTimer("PrimalStrategy");

    for (auto cand : this->primalSolutionCandidates)
    {
        this->checkPrimalSolutionPoint(cand);
    }

    this->primalSolutionCandidates.clear();

    this->stopTimer("PrimalStrategy");
}

void ProcessInfo::checkDualSolutionCandidates()
{
    double currDualBound = this->getDualBound();
    double currPrimalBound = this->getPrimalBound();

    double gapRelTolerance = env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination");
    double gapAbsTolerance = env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination");

    for (auto C : this->dualSolutionCandidates)
    {
        bool updateDual = false;

        if (env->problem->objectiveFunction->properties.isMinimize)
        {
            if (C.objValue < currPrimalBound * (1 + gapRelTolerance) && C.objValue > currPrimalBound)
            {
                C.objValue = currPrimalBound;
                updateDual = true;
            }
            else if (C.objValue > currDualBound && (C.objValue <= currPrimalBound))
            {
                updateDual = true;
            }
        }
        else
        {
            if (C.objValue > currPrimalBound * (1 + gapRelTolerance) && C.objValue < currPrimalBound)
            {
                C.objValue = currPrimalBound;
                updateDual = true;
            }
            else if (C.objValue < currDualBound && (C.objValue >= currPrimalBound))
            {
                updateDual = true;
            }
        }

        if (updateDual)
        {
            // New dual solution
            this->setDualBound(C.objValue);
            currDualBound = C.objValue;
            env->solutionStatistics.iterationLastDualBoundUpdate = this->getCurrentIteration()->iterationNumber;
            env->solutionStatistics.iterationLastDualBoundUpdate = this->getElapsedTime("Total");

            if (C.sourceType == E_DualSolutionSource::MIPSolutionOptimal ||
                C.sourceType == E_DualSolutionSource::LPSolution ||
                C.sourceType == E_DualSolutionSource::MIPSolverBound)
            {
                this->addDualSolution(C);
            }

            std::string sourceDesc;

            switch (C.sourceType)
            {
            case E_DualSolutionSource::LPSolution:
                sourceDesc = "LP solution";
                break;
            case E_DualSolutionSource::MIPSolutionOptimal:
                sourceDesc = "MIP solution";
                break;
            case E_DualSolutionSource::ObjectiveConstraint:
                sourceDesc = "Obj. constr. linesearch";
                break;
            case E_DualSolutionSource::MIPSolverBound:
                sourceDesc = "MIP solver bound";
                break;
            default:
                break;
            }

            auto tmpLine = boost::format("     New dual bound %1% (%2%) ") % C.objValue % sourceDesc;

            env->output->outputDebug(tmpLine.str());
        }
    }

    this->dualSolutionCandidates.clear();
}

bool ProcessInfo::isRelativeObjectiveGapToleranceMet()
{
    if (this->getRelativeObjectiveGap() <= env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination"))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

bool ProcessInfo::isAbsoluteObjectiveGapToleranceMet()
{
    if (this->getAbsoluteObjectiveGap() <= env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination"))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

bool ProcessInfo::checkPrimalSolutionPoint(PrimalSolution primalSol)
{
    std::string sourceDesc;

    VectorDouble tmpPoint(primalSol.point);
    double tmpObjVal = primalSol.objValue;

    bool isVariableBoundsFulfilled = false;

    switch (primalSol.sourceType)
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

    for (auto &V : env->problem->realVariables)
    {
        auto value = V->calculate(tmpPoint);

        if (value > V->upperBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = V->upperBound;
        }
        else if (value < V->lowerBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = V->lowerBound;
        }
    }

    for (auto &V : env->problem->integerVariables)
    {
        auto value = V->calculate(tmpPoint);

        if (value > V->upperBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = round(V->upperBound - 0.5);
        }
        else if (value < V->lowerBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = round(V->lowerBound + 0.5);
        }
    }

    for (auto &V : env->problem->binaryVariables)
    {
        auto value = V->calculate(tmpPoint);

        if (value > V->upperBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = 1.0;
        }
        else if (value < V->lowerBound)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(V->index) = 0.0;
        }
    }

    if (!isVariableBoundsFulfilled)
    {
        reCalculateObjective = true;
        auto tmpLine = boost::format("       Variable bounds not fulfilled. Projection to bounds performed.");
        env->output->outputWarning(tmpLine.str());
        primalSol.boundProjectionPerformed = true;
    }
    else
    {
        auto tmpLine = boost::format("       All variable bounds fulfilled.");
        env->output->outputWarning(tmpLine.str());
        primalSol.boundProjectionPerformed = false;
    }

    // Check that it fulfills integer constraints, round otherwise
    if (env->problem->properties.numberOfDiscreteVariables > 0)
    {
        auto integerTol = env->settings->getDoubleSetting("Tolerance.Integer", "Primal");

        bool isRounded = false;

        VectorDouble ptRounded(tmpPoint);

        double maxIntegerError = 0.0;

        for (auto &V : env->problem->integerVariables)
        {
            auto value = V->calculate(tmpPoint);
            int index = V->index;

            double rounded = UtilityFunctions::round(value);
            double error = std::abs(rounded - value);

            maxIntegerError = std::max(maxIntegerError, error);

            if (error > integerTol)
            {
                ptRounded.at(index) = rounded;
                isRounded = true;
            }
        }

        for (auto &V : env->problem->binaryVariables)
        {
            auto value = V->calculate(tmpPoint);
            int index = V->index;

            double rounded = UtilityFunctions::round(value);
            double error = std::abs(rounded - value);

            maxIntegerError = std::max(maxIntegerError, error);

            if (error > integerTol)
            {
                ptRounded.at(index) = rounded;
                isRounded = true;
            }
        }

        if (isRounded)
        {
            reCalculateObjective = true;
            tmpPoint = ptRounded;

            auto tmpLine = boost::format("       Discrete variables were not fulfilled to tolerance %1%. Rounding performed...") % integerTol;
            env->output->outputWarning(tmpLine.str());
        }
        else
        {
            auto tmpLine = boost::format("       All discrete variables are fulfilled to tolerance %1%.") % integerTol;
            env->output->outputWarning(tmpLine.str());
        }

        primalSol.integerRoundingPerformed = isRounded;
        primalSol.maxIntegerToleranceError = maxIntegerError;
    }

    // Recalculate the objective if rounding or projection has been performed
    if (reCalculateObjective)
    {
        tmpObjVal = env->problem->objectiveFunction->calculateValue(tmpPoint);
    }

    // Check if primal bound is worse than current
    if ((env->problem->objectiveFunction->properties.isMinimize && tmpObjVal < this->getPrimalBound()) ||
        (!env->problem->objectiveFunction->properties.isMinimize && tmpObjVal > this->getPrimalBound()))
    {
        auto tmpLine = boost::format("     Testing primal bound %1% found from %2%:") % tmpObjVal % sourceDesc;
        env->output->outputWarning(tmpLine.str());
    }
    else
    {
        auto tmpLine = boost::format("     Primal bound candidate (%1%) from %2% is not an improvement over current (%3%).") %
                       tmpObjVal % sourceDesc % this->getPrimalBound();
        env->output->outputWarning(tmpLine.str());

        return (false);
    }

    bool acceptableType = (primalSol.sourceType == E_PrimalSolutionSource::MIPSolutionPool ||
                           primalSol.sourceType == E_PrimalSolutionSource::NLPFixedIntegers ||
                           primalSol.sourceType == E_PrimalSolutionSource::NLPRelaxed ||
                           primalSol.sourceType == E_PrimalSolutionSource::IncumbentCallback ||
                           primalSol.sourceType == E_PrimalSolutionSource::LPFixedIntegers ||
                           primalSol.sourceType == E_PrimalSolutionSource::LazyConstraintCallback);

    if (acceptableType && env->settings->getBoolSetting("Tolerance.TrustLinearConstraintValues", "Primal"))
    {
        auto tmpLine = boost::format("       Assuming that linear constraints are fulfilled since solution is from a subsolver.");
        env->output->outputWarning(tmpLine.str());
    }
    else
    {
        PairIndexValue mostDevLinearConstraints;

        if (env->problem->properties.numberOfLinearConstraints > 0)
        {
            auto maxLinearConstraintValue = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->linearConstraints);

            mostDevLinearConstraints.index = maxLinearConstraintValue.constraint->index;
            mostDevLinearConstraints.value = maxLinearConstraintValue.normalizedValue;

            auto linTol = env->settings->getDoubleSetting("Tolerance.LinearConstraint", "Primal");

            if (maxLinearConstraintValue.error > linTol)
            {
                auto tmpLine = boost::format("       Linear constraints are not fulfilled. Most deviating %3%: %2% > %1%.") %
                               linTol % maxLinearConstraintValue.error % maxLinearConstraintValue.constraint->name;
                env->output->outputWarning(tmpLine.str());

                return (false);
            }
            else
            {
                auto tmpLine = boost::format("       Linear constraints are fulfilled. Most deviating %3%: %2% < %1%.") %
                               linTol % maxLinearConstraintValue.error % maxLinearConstraintValue.constraint->name;
                env->output->outputWarning(tmpLine.str());
            }
        }

        primalSol.maxDevatingConstraintLinear = mostDevLinearConstraints;
    }

    PairIndexValue mostDevQuadraticConstraints;

    if (env->problem->properties.numberOfQuadraticConstraints > 0)
    {
        auto maxQuadraticConstraintValue = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->quadraticConstraints);

        mostDevQuadraticConstraints.index = maxQuadraticConstraintValue.constraint->index;
        mostDevQuadraticConstraints.value = maxQuadraticConstraintValue.normalizedValue;

        auto nonlinTol = env->settings->getDoubleSetting("Tolerance.NonlinearConstraint", "Primal");

        if (mostDevQuadraticConstraints.value > nonlinTol)
        {
            auto tmpLine = boost::format(
                               "       Quadratic constraints are not fulfilled. Most deviating %3%: %2% > %1%.") %
                           nonlinTol % mostDevQuadraticConstraints.value % maxQuadraticConstraintValue.constraint->name;
            env->output->outputWarning(tmpLine.str());

            return (false);
        }
        else
        {
            auto tmpLine = boost::format(
                               "       Quadratic constraints are fulfilled. Most deviating %3%: %2% < %1%.") %
                           nonlinTol % mostDevQuadraticConstraints.value % maxQuadraticConstraintValue.constraint->name;
            env->output->outputWarning(tmpLine.str());
        }
    }

    primalSol.maxDevatingConstraintQuadratic = mostDevQuadraticConstraints;

    PairIndexValue mostDevNonlinearConstraints;

    if (env->problem->properties.numberOfNonlinearConstraints > 0)
    {
        auto maxNonlinearConstraintValue = env->problem->getMaxNumericConstraintValue(tmpPoint, env->problem->nonlinearConstraints);

        mostDevNonlinearConstraints.index = maxNonlinearConstraintValue.constraint->index;
        mostDevNonlinearConstraints.value = maxNonlinearConstraintValue.normalizedValue;

        auto nonlinTol = env->settings->getDoubleSetting("Tolerance.NonlinearConstraint", "Primal");

        if (mostDevNonlinearConstraints.value > nonlinTol)
        {
            auto tmpLine = boost::format(
                               "       Nonlinear constraints are not fulfilled. Most deviating %3%: %2% > %1%.") %
                           nonlinTol % mostDevNonlinearConstraints.value % maxNonlinearConstraintValue.constraint->name;
            env->output->outputWarning(tmpLine.str());

            return (false);
        }
        else
        {
            auto tmpLine = boost::format(
                               "       Nonlinear constraints are fulfilled. Most deviating %3%: %2% < %1%.") %
                           nonlinTol % mostDevNonlinearConstraints.value % maxNonlinearConstraintValue.constraint->name;
            env->output->outputWarning(tmpLine.str());
        }
    }

    primalSol.maxDevatingConstraintNonlinear = mostDevNonlinearConstraints;

    char HPobjadded = ' ';

    if (env->settings->getBoolSetting("HyperplaneCuts.UsePrimalObjectiveCut", "Dual") &&
        env->problem->objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic)
    {
        Hyperplane hyperplane;
        hyperplane.source = E_HyperplaneSource::PrimalSolutionSearchInteriorObjective;
        hyperplane.isObjectiveHyperplane = true;
        hyperplane.sourceConstraintIndex = -1;
        hyperplane.generatedPoint = tmpPoint;
        hyperplane.objectiveFunctionValue = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)->calculateValue(hyperplane.generatedPoint);

        this->hyperplaneWaitingList.push_back(hyperplane);

        auto tmpLine = boost::format("     Primal objective cut added.");

        env->output->outputWarning(tmpLine.str());
    }

    this->setPrimalBound(tmpObjVal);

    auto tmpLine = boost::format("     New primal bound %1% from %2% accepted.") % tmpObjVal % sourceDesc;

    env->output->outputWarning(tmpLine.str());

    primalSol.objValue = tmpObjVal;
    primalSol.point = tmpPoint;

    if (env->settings->getIntSetting("SaveNumberOfSolutions", "Output") > 1)
    {
        this->primalSolutions.insert(this->primalSolutions.begin(), primalSol);
    }
    else
    {
        if (this->primalSolutions.size() == 0)
        {
            this->primalSolutions.push_back(primalSol);
        }
        else
        {
            this->primalSolutions.at(0) = primalSol;
        }
    }

    this->primalSolution = tmpPoint;

    // Write the new primal point to a file
    if (env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        std::stringstream fileName;
        fileName << env->settings->getStringSetting("Debug.Path", "Output");
        fileName << "/primalpoint";
        fileName << this->primalSolutions.size();
        fileName << ".txt";

        UtilityFunctions::savePrimalSolutionToFile(primalSol, env->problem->allVariables, fileName.str());
    }

    return (true);
}

ProcessInfo::ProcessInfo(EnvironmentPtr envPtr) : env(envPtr)
{
}

ProcessInfo::~ProcessInfo()
{
    timers.clear();
    iterations.clear();

    primalSolution.clear();
    iterations.clear();
    primalSolutions.clear();
    dualSolutions.clear();

    primalSolutionCandidates.clear();
    primalFixedNLPCandidates.clear();
    dualSolutionCandidates.clear();
}

void ProcessInfo::createTimer(std::string name, std::string description)
{
    timers.push_back(Timer(name, description));
}

void ProcessInfo::startTimer(std::string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
        env->output->outputError("Timer with name  \"" + name + "\" not found!");
        return;
    }

    timer->start();
}

void ProcessInfo::restartTimer(std::string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
        env->output->outputError("Timer with name  \"" + name + "\" not found!");
        return;
    }

    timer->restart();
}

void ProcessInfo::stopTimer(std::string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
        env->output->outputError("Timer with name  \"" + name + "\" not found!");
        return;
    }

    timer->stop();
}

double ProcessInfo::getElapsedTime(std::string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
        env->output->outputError("Timer with name  \"" + name + "\" not found!");
        return (0.0);
    }

    return (timer->elapsed());
}

void ProcessInfo::initializeResults(int numObj, int numVar, int numConstr)
{
    osResult = std::unique_ptr<OSResult>(new OSResult());
    osResult->setObjectiveNumber(numObj);
    osResult->setVariableNumber(numVar);
    osResult->setConstraintNumber(numConstr);
}

std::string ProcessInfo::getOSrl()
{
    int numConstr = osResult->getConstraintNumber();

    int numVar = osResult->getVariableNumber();

    int numPrimalSols = primalSolutions.size();

    std::stringstream ssSolver;
    ssSolver << "Supporting Hyperplane Optimization Toolkit, version ";
    ssSolver << SHOT_VERSION_MAJOR << "." << SHOT_VERSION_MINOR << "." << SHOT_VERSION_PATCH;
    osResult->setSolverInvoked(ssSolver.str());

    osResult->setInstanceName(env->settings->getStringSetting("ProblemName", "Input"));
    osResult->setNumberOfOtherGeneralResults(1);
    osResult->setOtherGeneralResultName(0, "UsedOptions");

    osResult->setOtherGeneralResultValue(0, env->settings->getSettingsAsString());

    if (numPrimalSols == 0)
    {
        osResult->setSolutionNumber(1);
        osResult->setNumberOfObjValues(0, 1);

        std::stringstream strstrdb;
        strstrdb << std::fixed << std::setprecision(15) << getDualBound();

        osResult->setAnOtherSolutionResult(0, "DualObjectiveBound", strstrdb.str(), "Final solution",
                                           "The dual bound for the objective", 0, NULL);

        if (dualSolutions.size() > 0 && dualSolutions.back().point.size() > 0)
        {
            osResult->setObjValue(0, 0, -1, "", dualSolutions.back().objValue);

            std::stringstream strstr;
            strstr << std::fixed << std::setprecision(15)
                   << dualSolutions.back().objValue;

            osResult->setAnOtherSolutionResult(0, "MaxErrorConstrs", strstr.str(), "Final solution",
                                               "Maximal error in constraint", 0, NULL);
        }

        std::stringstream strstr2;
        strstr2 << std::fixed << std::setprecision(15) << getAbsoluteObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "AbsOptimalityGap", strstr2.str(), "Final solution",
                                           "The absolute optimality gap", 0, NULL);

        std::stringstream strstr3;
        strstr3 << std::fixed << std::setprecision(15) << getRelativeObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "RelOptimalityGap", strstr3.str(), "Final solution",
                                           "The relative optimality gap", 0, NULL);
    }
    else
    {
        int numSaveSolutions = env->settings->getIntSetting("SaveNumberOfSolutions", "Output");

        osResult->setSolutionNumber(numSaveSolutions);

        for (int i = 0; i < numSaveSolutions; i++)
        {
            if (i == 0)
            {
                std::string modelStatus;
                std::string modelStatusDescription;
                std::string modelSubStatus;
                std::string modelSubStatusDescription;

                osResult->setNumberOfSolutionSubstatuses(0, 1);

                if (this->primalSolutions.size() > 0)
                {
                    modelStatusDescription = "Feasible solution found";
                    modelStatus = "feasible";
                }
                else
                {
                    modelStatusDescription = "No feasible solutions found";
                    modelStatus = "other";
                }

                if (this->terminationReason == E_TerminationReason::AbsoluteGap)
                {
                    modelStatus = "globallyOptimal";
                    modelStatusDescription = "Solved to global optimality (assuming the problem is convex)";
                    modelSubStatus = "stoppedByBounds";
                    modelSubStatusDescription = "Objective gap fulfills absolute gap termination criterion";
                }
                else if (this->terminationReason == E_TerminationReason::RelativeGap)
                {
                    modelStatus = "globallyOptimal";
                    modelStatusDescription = "Solved to global optimality (assuming the problem is convex)";
                    modelSubStatus = "stoppedByBounds";
                    modelSubStatusDescription = "Objective gap fulfills relative gap termination criterion";
                }
                else if (this->terminationReason == E_TerminationReason::ConstraintTolerance)
                {
                    modelStatus = "locallyOptimal";
                    modelStatusDescription = "Solved to global optimality";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription = "All nonlinear constraints fulfilled to given tolerance by dual solution";
                }
                else if (this->terminationReason == E_TerminationReason::InfeasibleProblem)
                {
                    modelStatus = "infeasible";
                    modelStatusDescription = "Problem may be infeasible or specified tolerances are too strict";
                }
                else if (this->terminationReason == E_TerminationReason::UnboundedProblem)
                {
                    modelStatus = "unbounded";
                    modelStatusDescription = "Problem is unbounded";
                }
                else if (this->terminationReason == E_TerminationReason::ObjectiveGapNotReached)
                {
                    modelStatusDescription = "Feasible solution found, but could not verify optimality";
                }
                else if (this->terminationReason == E_TerminationReason::ObjectiveStagnation)
                {
                    modelStatusDescription = "Terminated due to objective stagnation";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription = "Terminated due to objective stagnation";
                }
                else if (this->terminationReason == E_TerminationReason::IterationLimit)
                {
                    modelStatusDescription = "Terminated due to iteration limit";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription = "Terminated due to iteration limit";
                }
                else if (this->terminationReason == E_TerminationReason::TimeLimit)
                {
                    modelStatusDescription = "Terminated due to time limit";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription = "Terminated due to time limit";
                }
                else if (this->terminationReason == E_TerminationReason::NumericIssues)
                {
                    modelStatusDescription = "Terminated due to numeric issues";
                }
                else if (this->terminationReason == E_TerminationReason::UserAbort)
                {
                    modelStatusDescription = "User aborted solution process";
                }
                else if (this->terminationReason == E_TerminationReason::Error)
                {
                    modelStatusDescription = "Error during solution process";
                }
                else
                {
                    modelStatus = "error";
                    modelStatusDescription = "Unknown return code obtained from solver";
                    env->output->outputError("Unknown return code obtained from solver.");
                }

                osResult->setSolutionStatusType(0, modelStatus);
                osResult->setSolutionStatusDescription(0, modelStatusDescription);
                osResult->setSolutionSubstatusType(0, 0, modelSubStatus);
                osResult->setSolutionSubstatusDescription(0, 0, modelSubStatusDescription);
            }
            else
            {
                osResult->setSolutionStatusType(i, "feasible");
                osResult->setSolutionStatusDescription(i, "Additional primal solution");
            }

            osResult->setNumberOfObjValues(i, 1);
            osResult->setNumberOfPrimalVariableValues(i, numVar);
            osResult->setObjValue(i, 0, -1, "", primalSolutions.at(i).objValue);

            osResult->setPrimalVariableValuesDense(i, &primalSolutions.at(i).point[0]);

            VectorDouble tmpConstrVals;

            osResult->setNumberOfDualValues(i, numConstr);

            for (int j = 0; j < numConstr; j++)
            {
                osResult->setDualValue(i, j, j, env->problem->numericConstraints.at(j)->name, env->problem->numericConstraints.at(j)->calculateNumericValue(primalSolutions.at(i).point).normalizedValue);
            }
        }

        std::stringstream strstrdb;
        strstrdb << std::fixed << std::setprecision(15) << getDualBound();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "DualObjectiveBound", strstrdb.str(), "Final solution",
                                           "The dual bound for the objective", 0, NULL);

        std::stringstream strstrpb;
        strstrpb << std::fixed << std::setprecision(15) << getPrimalBound();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "PrimalObjectiveBound", strstrpb.str(), "Final solution",
                                           "The primal bound for the objective", 0, NULL);

        std::stringstream strstr;
        strstr << std::fixed << std::setprecision(15) << getCurrentIteration()->maxDeviation;

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "MaxErrorConstrs", strstr.str(), "Final solution",
                                           "Maximal error in constraint", 0, NULL);

        std::stringstream strstr2;
        strstr2 << std::fixed << std::setprecision(15) << getAbsoluteObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "AbsOptimalityGap", strstr2.str(), "Final solution",
                                           "The absolute optimality gap", 0, NULL);

        std::stringstream strstr3;
        strstr3 << std::fixed << std::setprecision(15) << getRelativeObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "RelOptimalityGap", strstr3.str(), "Final solution",
                                           "The relative optimality gap", 0, NULL);
    }

    for (auto T : timers)
    {
        osResult->addTimingInformation(T.name, "SHOT", "second", T.description, T.elapsed());
    }

    numPrimalSols = std::max(1, numPrimalSols); // To make sure we also print the following even if we have no primal solution

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "LP", std::to_string(env->solutionStatistics.numberOfProblemsLP), "ProblemsSolved",
                                       "Relaxed LP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "QP", std::to_string(env->solutionStatistics.numberOfProblemsQP), "ProblemsSolved",
                                       "Relaxed QP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMILP", std::to_string(env->solutionStatistics.numberOfProblemsFeasibleMILP),
                                       "ProblemsSolved", "MILP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMILP", std::to_string(env->solutionStatistics.numberOfProblemsOptimalMILP), "ProblemsSolved",
                                       "MILP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMIQP", std::to_string(env->solutionStatistics.numberOfProblemsFeasibleMIQP),
                                       "ProblemsSolved", "MIQP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMIQP", std::to_string(env->solutionStatistics.numberOfProblemsOptimalMIQP), "ProblemsSolved",
                                       "MIQP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Total",
                                       std::to_string(env->solutionStatistics.numberOfProblemsLP + env->solutionStatistics.numberOfProblemsFeasibleMILP + env->solutionStatistics.numberOfProblemsOptimalMILP + env->solutionStatistics.numberOfProblemsQP + env->solutionStatistics.numberOfProblemsFeasibleMIQP + env->solutionStatistics.numberOfProblemsOptimalMIQP), "ProblemsSolved",
                                       "Total number of (MI)QP/(MI)LP subproblems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NLP", std::to_string(env->solutionStatistics.getNumberOfTotalNLPProblems()), "ProblemsSolved",
                                       "NLP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Functions", std::to_string(env->solutionStatistics.numberOfFunctionEvalutions), "Evaluations",
                                       "Total number of function evaluations in SHOT", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Gradients", std::to_string(env->solutionStatistics.numberOfGradientEvaluations), "Evaluations",
                                       "Total number of gradient evaluations in SHOT", 0, NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberVariables",
                                       std::to_string(env->problem->properties.numberOfVariables), "Problem",
                                       "Total number of variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberContinousVariables",
                                       std::to_string(env->problem->properties.numberOfRealVariables),
                                       "Problem", "Number of continuous variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberBinaryVariables",
                                       std::to_string(env->problem->properties.numberOfBinaryVariables), "Problem",
                                       "Number of binary variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberIntegerVariables",
                                       std::to_string(env->problem->properties.numberOfIntegerVariables), "Problem",
                                       "Number of integer variables", 0, NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberConstraints",
                                       std::to_string(env->problem->properties.numberOfNumericConstraints), "Problem",
                                       "Number of constraints", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberNonlinearConstraints",
                                       std::to_string(env->problem->properties.numberOfNonlinearConstraints), "Problem",
                                       "Number of nonlinear constraints", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberLinearConstraints",
                                       std::to_string(env->problem->properties.numberOfLinearConstraints),
                                       "Problem",
                                       "Number of linear constraints", 0, NULL);

    OSrLWriter writer;
    writer.m_bWhiteSpace = false;

    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);

    std::stringstream ssOSrL;
    ssOSrL << writer.writeOSrL(osResult.get());

    read_xml(ssOSrL, pt, boost::property_tree::xml_parser::trim_whitespace);

    std::ostringstream ossXML;
    write_xml(ossXML, pt, settings);

    return (ossXML.str());
}

std::string ProcessInfo::getTraceResult()
{
    std::stringstream ss;
    ss << env->problem->name << ",";

    switch (static_cast<E_SolutionStrategy>(env->process->usedSolutionStrategy))
    {
    case (E_SolutionStrategy::MIQP):
        ss << "MINLP";
        break;
    case (E_SolutionStrategy::MIQCQP):
        ss << "MINLP";
        break;
    case (E_SolutionStrategy::NLP):
        ss << "NLP";
        break;
    default:
        ss << "MINLP";
        break;
    }

    ss << ",";
    ss << "SHOT"
       << ",";

    switch (static_cast<ES_PrimalNLPSolver>(env->process->usedPrimalNLPSolver))
    {
    case (ES_PrimalNLPSolver::None):
        ss << "NONE";
        break;
    case (ES_PrimalNLPSolver::CuttingPlane):
        ss << "SHOT";
        break;
    case (ES_PrimalNLPSolver::GAMS):
        ss << env->settings->getStringSetting("GAMS.NLP.Solver", "Subsolver");
        break;
    case (ES_PrimalNLPSolver::Ipopt):
        ss << "Ipopt";
        break;
    default:
        ss << "NONE";
        break;
    }

    ss << ",";

    switch (static_cast<ES_MIPSolver>(env->process->usedMIPSolver))
    {
    case (ES_MIPSolver::Cplex):
        ss << "CPLEX";
        break;
    case (ES_MIPSolver::Gurobi):
        ss << "GUROBI";
        break;
    case (ES_MIPSolver::Cbc):
        ss << "CBC";
        break;
    default:
        ss << "NONE";
        break;
    }

    ss << ",";

    ss << UtilityFunctions::toStringFormat(UtilityFunctions::getJulianFractionalDate(), "%.5f", false);
    ss << ",";
    ss << (env->problem->objectiveFunction->properties.isMinimize ? "0" : "1") << ",";
    ss << env->problem->properties.numberOfNumericConstraints << ",";
    ss << env->problem->properties.numberOfVariables << ",";
    ss << env->problem->properties.numberOfDiscreteVariables << ",";

    ss << '0' << ","; // TODO: Number of nonzeroes
    ss << '0' << ","; // TODO: Number of nonlinear nonzeroes
    ss << "1"
       << ",";

    std::string solverStatus = "";
    std::string modelStatus = "";

    bool isOptimal = false;

    if (this->terminationReason == E_TerminationReason::AbsoluteGap ||
        this->terminationReason == E_TerminationReason::RelativeGap)
    {
        solverStatus = "1";
        isOptimal = true;
    }
    else if (this->terminationReason == E_TerminationReason::ObjectiveStagnation ||
             this->terminationReason == E_TerminationReason::IterationLimit)
    {
        solverStatus = "2";
    }
    else if (this->terminationReason == E_TerminationReason::TimeLimit)
    {
        solverStatus = "3";
    }
    else if (this->terminationReason == E_TerminationReason::NumericIssues)
    {
        solverStatus = "5";
    }
    else if (this->terminationReason == E_TerminationReason::UserAbort)
    {
        solverStatus = "8";
    }
    else if (this->terminationReason == E_TerminationReason::Error)
    {
        solverStatus = "10";
    }
    else if (this->terminationReason == E_TerminationReason::InfeasibleProblem ||
             this->terminationReason == E_TerminationReason::ConstraintTolerance ||
             this->terminationReason == E_TerminationReason::ObjectiveGapNotReached ||
             this->terminationReason == E_TerminationReason::UnboundedProblem)
    {
        solverStatus = "1";
    }
    else
    {
        solverStatus = "10";
        env->output->outputError("Unknown return code " + std::to_string((int)this->terminationReason) + " obtained from solver.");
    }

    auto solStatus = this->getCurrentIteration()->solutionStatus;

    if (isOptimal)
    {
        modelStatus = "1";
    }
    else if (this->primalSolutions.size() > 0)
    {
        modelStatus = "8";
    }
    else if (solStatus == E_ProblemSolutionStatus::Unbounded)
    {
        modelStatus = "3";
    }
    else if (solStatus == E_ProblemSolutionStatus::Infeasible)
    {
        modelStatus = "4";
    }
    else if (solStatus == E_ProblemSolutionStatus::Feasible ||
             solStatus == E_ProblemSolutionStatus::IterationLimit ||
             solStatus == E_ProblemSolutionStatus::TimeLimit ||
             solStatus == E_ProblemSolutionStatus::NodeLimit ||
             solStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        modelStatus = "7";
    }
    else if (solStatus == E_ProblemSolutionStatus::Error ||
             solStatus == E_ProblemSolutionStatus::Numeric ||
             solStatus == E_ProblemSolutionStatus::CutOff ||
             solStatus == E_ProblemSolutionStatus::Abort)
    {
        modelStatus = "12";
    }
    else
    {
        modelStatus = "NA";
        env->output->outputError("Unknown return code " + std::to_string((int)solStatus) + " from model solution.");
    }

    ss << modelStatus << ",";
    ss << solverStatus << ",";

    ss << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    ss << this->getPrimalBound() << ",";
    ;
    ss << this->getDualBound() << ",";
    ;
    ss << this->getElapsedTime("Total") << ",";
    ss << env->solutionStatistics.numberOfIterations << ",";
    ss << "0"
       << ",";
    ss << env->solutionStatistics.numberOfExploredNodes
       << ",";
    ss << "#";

    return (ss.str());
}

void ProcessInfo::createIteration()
{
    Iteration iter(env);

    iterations.push_back(iter);
}

Iteration *ProcessInfo::getCurrentIteration()
{
    return (&iterations.back());
}

Iteration *ProcessInfo::getPreviousIteration()
{
    if (iterations.size() > 1)
        return (&(iterations[iterations.size() - 2]));
    else
        throw ErrorClass("Only one iteration!");
}

double ProcessInfo::getPrimalBound()
{
    return (this->currentPrimalBound);
}

void ProcessInfo::setPrimalBound(double value)
{
    this->currentPrimalBound = value;
}

double ProcessInfo::getDualBound()
{
    return (this->currentDualBound);
}

void ProcessInfo::setDualBound(double value)
{
    this->currentDualBound = value;
}

double ProcessInfo::getAbsoluteObjectiveGap()
{
    double gap = abs(getDualBound() - getPrimalBound());

    return (gap);
}

double ProcessInfo::getRelativeObjectiveGap()
{
    double gap = abs(getDualBound() - getPrimalBound()) / ((1e-10) + abs(getPrimalBound()));

    return (gap);
}
} // namespace SHOT