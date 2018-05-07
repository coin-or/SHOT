/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

<<<<<<< HEAD
void ProcessInfo::addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter,
                                    IndexValuePair maxConstrDev)
{
    PrimalSolution sol =
        {pt, source, objVal, iter, maxConstrDev};
=======
#include "ProcessInfo.h"
#include "OptProblems/OptProblemOriginal.h"

void ProcessInfo::addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter, IndexValuePair maxConstrDev)
{
    PrimalSolution sol;

    sol.point = pt;
    sol.sourceType = source;
    sol.objValue = objVal;
    sol.iterFound = iter;
    sol.maxDevatingConstraintNonlinear = maxConstrDev;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    primalSolutions.push_back(sol);
}

void ProcessInfo::addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter)
{
    auto maxConstrDev = originalProblem->getMostDeviatingConstraint(pt);

<<<<<<< HEAD
    auto maxConstrDev = originalProblem->getMostDeviatingConstraint(pt);

=======
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    ProcessInfo::addPrimalSolution(pt, source, objVal, iter, maxConstrDev);
}

void ProcessInfo::addDualSolution(vector<double> pt, E_DualSolutionSource source, double objVal, int iter)
{
<<<<<<< HEAD
    DualSolution sol =
        {pt, source, objVal, iter};
=======
    DualSolution sol = {pt, source, objVal, iter, false};
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    addDualSolution(sol);
}

void ProcessInfo::addDualSolution(SolutionPoint pt, E_DualSolutionSource source)
{
<<<<<<< HEAD
    DualSolution sol =
        {pt.point, source, pt.objectiveValue, pt.iterFound};
=======
    DualSolution sol = {pt.point, source, pt.objectiveValue, pt.iterFound, false};
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    addDualSolution(sol);
}

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

void ProcessInfo::addPrimalSolutionCandidate(vector<double> pt, E_PrimalSolutionSource source, int iter)
{
<<<<<<< HEAD
    PrimalSolution sol =
        {pt, source, originalProblem->calculateOriginalObjectiveValue(pt), iter,
         originalProblem->getMostDeviatingConstraint(pt)};

    primalSolutionCandidates.push_back(sol);

=======
    PrimalSolution sol;

    sol.point = pt;
    sol.sourceType = source;
    sol.objValue = originalProblem->calculateOriginalObjectiveValue(pt);
    sol.iterFound = iter;
    sol.maxDevatingConstraintNonlinear = originalProblem->getMostDeviatingConstraint(pt);

    primalSolutionCandidates.push_back(sol);

>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    this->checkPrimalSolutionCandidates();
}

void ProcessInfo::addPrimalSolutionCandidates(vector<vector<double>> pts, E_PrimalSolutionSource source, int iter)
{
    for (auto PT : pts)
    {
        addPrimalSolutionCandidate(PT, source, iter);
    }
}

void ProcessInfo::addDualSolutionCandidate(vector<double> pt, E_DualSolutionSource source, int iter)
{
    double tmpObjVal = this->originalProblem->calculateOriginalObjectiveValue(pt);

<<<<<<< HEAD
    DualSolution sol =
        {pt, source, tmpObjVal, iter};
=======
    DualSolution sol = {pt, source, tmpObjVal, iter, false};
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    addDualSolutionCandidate(sol);
}

void ProcessInfo::addDualSolutionCandidate(SolutionPoint pt, E_DualSolutionSource source)
{
<<<<<<< HEAD
    DualSolution sol =
        {pt.point, source, pt.objectiveValue, pt.iterFound};
=======
    DualSolution sol = {pt.point, source, pt.objectiveValue, pt.iterFound, false};
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    addDualSolutionCandidate(sol);
}

void ProcessInfo::addDualSolutionCandidates(std::vector<SolutionPoint> pts, E_DualSolutionSource source)
{
    for (auto pt : pts)
    {
        addDualSolutionCandidate(pt, source);
    }
}

void ProcessInfo::addDualSolutionCandidate(DualSolution solution)
{
    dualSolutionCandidates.push_back(solution);

    this->checkDualSolutionCandidates();
}

pair<double, double> ProcessInfo::getCorrectedObjectiveBounds()
{
    pair<double, double> bounds;

    if (originalProblem->isTypeOfObjectiveMinimize())
    {
        bounds.first = currentObjectiveBounds.first;
        bounds.second = currentObjectiveBounds.second;
    }
    else
    {
        bounds.first = currentObjectiveBounds.second;
        bounds.second = currentObjectiveBounds.first;
    }

    return (bounds);
}

void ProcessInfo::addPrimalSolution(SolutionPoint pt, E_PrimalSolutionSource source)
{
<<<<<<< HEAD
    PrimalSolution sol =
        {pt.point, source, pt.objectiveValue, pt.iterFound};

    if (Settings::getInstance().getBoolSetting("SaveAllPrimalSolutions", "SHOTSolver"))
=======
    PrimalSolution sol;

    sol.point = pt.point;
    sol.sourceType = source;
    sol.objValue = pt.objectiveValue;
    sol.iterFound = pt.iterFound;

    if (Settings::getInstance().getIntSetting("SaveNumberOfSolutions", "Output") > 1)
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    {
        primalSolutions.push_back(sol);
    }
    else
    {
        if (primalSolutions.size() == 0)
            primalSolutions.push_back(sol);
        else
            primalSolutions.at(0) = sol;
    }
}

void ProcessInfo::addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source)
{
<<<<<<< HEAD
    PrimalSolution sol =
        {pt.point, source, pt.objectiveValue, pt.iterFound};
=======
    PrimalSolution sol;

    sol.point = pt.point;
    sol.sourceType = source;
    sol.objValue = pt.objectiveValue;
    sol.iterFound = pt.iterFound;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

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

void ProcessInfo::addPrimalFixedNLPCandidate(vector<double> pt, E_PrimalNLPSource source, double objVal, int iter,
                                             IndexValuePair maxConstrDev)
{
    PrimalFixedNLPCandidate cand =
        {pt, source, objVal, iter};

    primalFixedNLPCandidates.push_back(cand);
}

void ProcessInfo::setObjectiveUpdatedByLinesearch(bool updated)
{
    objectiveUpdatedByLinesearch = updated;
}

bool ProcessInfo::getObjectiveUpdatedByLinesearch()
{
    return (objectiveUpdatedByLinesearch);
<<<<<<< HEAD
}

void ProcessInfo::outputAlways(std::string message)
{
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_always, message);
}

void ProcessInfo::outputError(std::string message)
{
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, message);
}

void ProcessInfo::outputError(std::string message, std::string errormessage)
{
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, message);
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_error, "Error message: " + errormessage);
}

void ProcessInfo::outputSummary(std::string message)
{
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_summary, message);
}

void ProcessInfo::outputWarning(std::string message)
{
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_warning, message);
}

void ProcessInfo::outputInfo(std::string message)
{
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_info, message);
}

void ProcessInfo::outputDebug(std::string message)
{
    osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_debug, message);
}

void ProcessInfo::outputTrace(std::string message)
{
    //osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_detailed_trace, message);
}

void ProcessInfo::outputDetailedTrace(std::string message)
{
    //osoutput->OSPrint(ENUM_OUTPUT_AREA_main, ENUM_OUTPUT_LEVEL_detailed_trace, message);
=======
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

void ProcessInfo::checkPrimalSolutionCandidates()
{
<<<<<<< HEAD
    this->startTimer("PrimalBoundTotal");
=======
    this->startTimer("PrimalStrategy");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    for (auto cand : this->primalSolutionCandidates)
    {
        this->checkPrimalSolutionPoint(cand);
    }

    this->primalSolutionCandidates.clear();

<<<<<<< HEAD
    this->stopTimer("PrimalBoundTotal");
=======
    this->stopTimer("PrimalStrategy");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

void ProcessInfo::checkDualSolutionCandidates()
{
    bool isMinimization = this->originalProblem->isTypeOfObjectiveMinimize();

    double currDualBound = this->getDualBound();
    double currPrimalBound = this->getPrimalBound();

<<<<<<< HEAD
    for (auto C : this->dualSolutionCandidates)
    {

        if ((isMinimization && (C.objValue > currDualBound && C.objValue <= currPrimalBound + 10 ^ -6)) || (!isMinimization && (C.objValue < currDualBound && C.objValue >= currPrimalBound - 10 ^ -6)))
        {
            // New dual solution
            this->currentObjectiveBounds.first = C.objValue;
            currDualBound = C.objValue;
            this->iterLastDualBoundUpdate = this->getCurrentIteration()->iterationNumber;
            this->timeLastDualBoundUpdate = this->getElapsedTime("Total");

            // If the solution is MILP feasible we only have a bound, no variable solutions
            if (C.sourceType != E_DualSolutionSource::MILPSolutionFeasible)
=======
    double gapRelTolerance = Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination");
    double gapAbsTolerance = Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination");

    for (auto C : this->dualSolutionCandidates)
    {
        bool updateDual = false;

        if (isMinimization)
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
            this->solutionStatistics.iterationLastDualBoundUpdate = this->getCurrentIteration()->iterationNumber;
            this->solutionStatistics.iterationLastDualBoundUpdate = this->getElapsedTime("Total");

            if (C.sourceType == E_DualSolutionSource::MIPSolutionOptimal ||
                C.sourceType == E_DualSolutionSource::LPSolution ||
                C.sourceType == E_DualSolutionSource::MIPSolverBound)
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            {
                this->addDualSolution(C);
            }

            std::string sourceDesc;

            switch (C.sourceType)
            {
<<<<<<< HEAD
            /*case E_DualSolutionSource::Linesearch:
				 sourceDesc = "line search";
				 break;*/
            case E_DualSolutionSource::LPSolution:
                sourceDesc = "LP solution";
                break;
            case E_DualSolutionSource::MILPSolutionOptimal:
                sourceDesc = "MILP solution";
                break;
            case E_DualSolutionSource::MILPSolutionFeasible:
                sourceDesc = "MILP solution bound";
=======
            case E_DualSolutionSource::LPSolution:
                sourceDesc = "LP solution";
                break;
            case E_DualSolutionSource::MIPSolutionOptimal:
                sourceDesc = "MIP solution";
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
                break;
            case E_DualSolutionSource::ObjectiveConstraint:
                sourceDesc = "Obj. constr. linesearch";
                break;
<<<<<<< HEAD
=======
            case E_DualSolutionSource::MIPSolverBound:
                sourceDesc = "MIP solver bound";
                break;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            default:
                break;
            }

            auto tmpLine = boost::format("     New dual bound %1% (%2%) ") % C.objValue % sourceDesc;

<<<<<<< HEAD
            this->outputInfo(tmpLine.str());
=======
            Output::getInstance().outputInfo(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        }
    }

    this->dualSolutionCandidates.clear();
}

bool ProcessInfo::isRelativeObjectiveGapToleranceMet()
{
<<<<<<< HEAD
    if (this->getRelativeObjectiveGap() <= Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
=======
    if (this->getRelativeObjectiveGap() <= Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination"))
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
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
<<<<<<< HEAD
    if (this->getAbsoluteObjectiveGap() <= Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
=======
    if (this->getAbsoluteObjectiveGap() <= Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination"))
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
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

    std::vector<double> tmpPoint(primalSol.point);
    double tmpObjVal = primalSol.objValue;

    bool isMinimization = this->originalProblem->isTypeOfObjectiveMinimize();

    bool isLinConstrFulfilled = false;
    bool isNonLinConstrFulfilled = false;

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
<<<<<<< HEAD
    case E_PrimalSolutionSource::MILPSolutionPool:
=======
    case E_PrimalSolutionSource::MIPSolutionPool:
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
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

<<<<<<< HEAD
=======
    primalSol.sourceDescription = sourceDesc;

>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    // Recalculate if the objective is not provided
    if (UtilityFunctions::isnan(primalSol.objValue))
    {
        tmpObjVal = this->originalProblem->calculateOriginalObjectiveValue(primalSol.point);
    }

<<<<<<< HEAD
    // Check if primal bound is worse than current
    if ((isMinimization && tmpObjVal < this->currentObjectiveBounds.second) || (!isMinimization && tmpObjVal > this->currentObjectiveBounds.second))
    {
        auto tmpLine = boost::format("     Testing primal bound %1% found from %2%:") % tmpObjVal % sourceDesc;
        this->outputWarning(tmpLine.str());
    }
    else
    {
        auto tmpLine = boost::format(
                           "     Primal bound candidate (%1%) from %2% is not an improvement over current (%3%).") %
                       tmpObjVal % sourceDesc % this->currentObjectiveBounds.second;
        this->outputWarning(tmpLine.str());

        return (false);
    }

    // Check that solution fulfills bounds, project back otherwise
=======
    // Check that solution fulfills bounds, project back otherwise
    bool reCalculateObjective = false;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    auto realVarIndexes = originalProblem->getRealVariableIndices();

    for (int i = 0; i < realVarIndexes.size(); i++)
    {
        int varIdx = realVarIndexes.at(i);
        auto tmpLB = originalProblem->getVariableLowerBound(varIdx);
        auto tmpUB = originalProblem->getVariableUpperBound(varIdx);

        if (tmpPoint.at(varIdx) > tmpUB)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(varIdx) = tmpUB;
        }
        else if (tmpPoint.at(varIdx) < tmpLB)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(varIdx) = tmpLB;
        }
    }

    auto integerVarIndexes = originalProblem->getIntegerVariableIndices();

    for (int i = 0; i < integerVarIndexes.size(); i++)
    {
        int varIdx = integerVarIndexes.at(i);
        auto tmpLB = originalProblem->getVariableLowerBound(varIdx);
        auto tmpUB = originalProblem->getVariableUpperBound(varIdx);

        if (tmpPoint.at(varIdx) > tmpUB)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(varIdx) = round(tmpUB - 0.5);
        }
        else if (tmpPoint.at(varIdx) < tmpLB)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(varIdx) = round(tmpLB + 0.5);
        }
    }

    auto binaryVarIndexes = originalProblem->getBinaryVariableIndices();

    for (int i = 0; i < binaryVarIndexes.size(); i++)
    {
        int varIdx = binaryVarIndexes.at(i);
        auto tmpLB = originalProblem->getVariableLowerBound(varIdx);
        auto tmpUB = originalProblem->getVariableUpperBound(varIdx);

        if (tmpPoint.at(varIdx) > tmpUB)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(varIdx) = 1.0;
        }
        else if (tmpPoint.at(varIdx) < tmpLB)
        {
            isVariableBoundsFulfilled = false;
            tmpPoint.at(varIdx) = 0.0;
        }
    }

<<<<<<< HEAD
    if (isVariableBoundsFulfilled)
    {
        auto tmpLine = boost::format("       Variable bounds not fulfilled. Projection to bounds performed.");
        this->outputWarning(tmpLine.str());
=======
    if (!isVariableBoundsFulfilled)
    {
        reCalculateObjective = true;
        auto tmpLine = boost::format("       Variable bounds not fulfilled. Projection to bounds performed.");
        Output::getInstance().outputWarning(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    }
    else
    {
        auto tmpLine = boost::format("       All variable bounds fulfilled.");
<<<<<<< HEAD
        this->outputWarning(tmpLine.str());
    }

    // Check that it fulfills integer constraints, round otherwise
    if (originalProblem->getNumberOfBinaryVariables() > 0 || originalProblem->getNumberOfIntegerVariables() > 0)
    {
        auto integerTol = Settings::getInstance().getDoubleSetting("PrimalBoundIntegerTolerance", "PrimalBound");
=======
        Output::getInstance().outputWarning(tmpLine.str());
    }

    primalSol.boundProjectionPerformed = !isVariableBoundsFulfilled;

    // Check that it fulfills integer constraints, round otherwise
    if (originalProblem->getNumberOfBinaryVariables() > 0 || originalProblem->getNumberOfIntegerVariables() > 0)
    {
        auto integerTol = Settings::getInstance().getDoubleSetting("Tolerance.Integer", "Primal");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

        bool isRounded = false;

        auto discreteVarIndexes = this->originalProblem->getDiscreteVariableIndices();

        std::vector<double> ptRounded(tmpPoint);

<<<<<<< HEAD
=======
        double maxIntegerError = 0.0;

>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        for (int i = 0; i < discreteVarIndexes.size(); i++)
        {
            int idx = discreteVarIndexes.at(i);
            double rounded = UtilityFunctions::round(tmpPoint.at(idx));

<<<<<<< HEAD
            if (abs(rounded - tmpPoint.at(idx)) > integerTol)
=======
            double error = abs(rounded - tmpPoint.at(idx));

            maxIntegerError = max(maxIntegerError, error);

            if (error > integerTol)
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            {
                ptRounded.at(idx) = rounded;
                isRounded = true;
            }
        }

        if (isRounded)
        {
<<<<<<< HEAD
            tmpPoint = ptRounded;
            tmpObjVal = this->originalProblem->calculateOriginalObjectiveValue(ptRounded);

            if (this->originalProblem->isObjectiveFunctionNonlinear())
            {
                tmpPoint.at(this->originalProblem->getNonlinearObjectiveVariableIdx()) = tmpObjVal;
            }
=======
            reCalculateObjective = true;
            tmpPoint = ptRounded;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

            auto tmpLine = boost::format(
                               "       Discrete variables were not fulfilled to tolerance %1%. Rounding performed...") %
                           integerTol;
<<<<<<< HEAD
            this->outputWarning(tmpLine.str());
=======
            Output::getInstance().outputWarning(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        }
        else
        {
            auto tmpLine = boost::format("       All discrete variables are fulfilled to tolerance %1%.") % integerTol;
<<<<<<< HEAD
            this->outputWarning(tmpLine.str());
        }
    }

    // Assume linear constraints are valid for MIP/LP solutions

    if (primalSol.sourceType == E_PrimalSolutionSource::MILPSolutionPool || primalSol.sourceType == E_PrimalSolutionSource::NLPFixedIntegers)
    {
    }
    else
    {
        auto linTol = Settings::getInstance().getDoubleSetting("PrimalBoundLinearTolerance", "PrimalBound");

        auto nonLinearIndexes = originalProblem->getLinearConstraintIndexes();

        if (nonLinearIndexes.size() > 0)
        {
            auto mostDevLinearConstraints =
                originalProblem->getMostDeviatingConstraint(tmpPoint, nonLinearIndexes).first;
=======
            Output::getInstance().outputWarning(tmpLine.str());
        }

        primalSol.integerRoundingPerformed = isRounded;
        primalSol.maxIntegerToleranceError = maxIntegerError;
    }

    // Recalculate the objective if rounding or projection has been performed
    if (reCalculateObjective)
    {
        tmpObjVal = this->originalProblem->calculateOriginalObjectiveValue(tmpPoint);

        if (this->originalProblem->isObjectiveFunctionNonlinear())
        {
            tmpPoint.at(this->originalProblem->getNonlinearObjectiveVariableIdx()) = tmpObjVal;
        }
    }

    // Check if primal bound is worse than current
    if ((isMinimization && tmpObjVal < this->getPrimalBound()) || (!isMinimization && tmpObjVal > this->getPrimalBound()))
    {
        auto tmpLine = boost::format("     Testing primal bound %1% found from %2%:") % tmpObjVal % sourceDesc;
        Output::getInstance().outputWarning(tmpLine.str());
    }
    else
    {
        auto tmpLine = boost::format(
                           "     Primal bound candidate (%1%) from %2% is not an improvement over current (%3%).") %
                       tmpObjVal % sourceDesc % this->getPrimalBound();
        Output::getInstance().outputWarning(tmpLine.str());

        return (false);
    }

    bool acceptableType = (primalSol.sourceType == E_PrimalSolutionSource::MIPSolutionPool || primalSol.sourceType == E_PrimalSolutionSource::NLPFixedIntegers || primalSol.sourceType == E_PrimalSolutionSource::NLPRelaxed || primalSol.sourceType == E_PrimalSolutionSource::IncumbentCallback || primalSol.sourceType == E_PrimalSolutionSource::LPFixedIntegers || primalSol.sourceType == E_PrimalSolutionSource::LazyConstraintCallback);

    if (acceptableType && Settings::getInstance().getBoolSetting("Tolerance.TrustLinearConstraintValues", "Primal"))
    {
        auto tmpLine = boost::format("       Assuming that linear constraints are fulfilled since solution is from a subsolver.");
        Output::getInstance().outputInfo(tmpLine.str());
    }
    else
    {
        auto linTol = Settings::getInstance().getDoubleSetting("Tolerance.LinearConstraint", "Primal");

        auto linearConstraintIndexes = originalProblem->getLinearConstraintIndexes();

        IndexValuePair mostDevLinearConstraints;

        if (linearConstraintIndexes.size() > 0)
        {
            mostDevLinearConstraints = originalProblem->getMostDeviatingConstraint(tmpPoint, linearConstraintIndexes).first;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

            isLinConstrFulfilled = (mostDevLinearConstraints.value < linTol);

            if (isLinConstrFulfilled)
            {
                auto tmpLine = boost::format("       Linear constraints are fulfilled. Most deviating %3%: %2% < %1%.") % linTol % mostDevLinearConstraints.value % originalProblem->getConstraintNames().at(mostDevLinearConstraints.idx);
<<<<<<< HEAD
                this->outputWarning(tmpLine.str());
            }
            else
            {
                auto tmpLine = boost::format(
                                   "       Linear constraints are not fulfilled. Most deviating %3%: %2% > %1%.") %
                               linTol % mostDevLinearConstraints.value % originalProblem->getConstraintNames().at(mostDevLinearConstraints.idx);
                this->outputWarning(tmpLine.str());
=======
                Output::getInstance().outputInfo(tmpLine.str());
            }
            else
            {
                auto tmpLine = boost::format("       Linear constraints are not fulfilled. Most deviating %3%: %2% > %1%.") % linTol % mostDevLinearConstraints.value % originalProblem->getConstraintNames().at(mostDevLinearConstraints.idx);
                Output::getInstance().outputInfo(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

                return (false);
            }
        }
<<<<<<< HEAD
=======

        primalSol.maxDevatingConstraintLinear = mostDevLinearConstraints;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    }

    IndexValuePair mostDevNonlinearConstraints;

    if (originalProblem->getNumberOfNonlinearConstraints() > 0)
    {
<<<<<<< HEAD
        mostDevNonlinearConstraints = this->originalProblem->getMostDeviatingConstraint(tmpPoint,
                                                                                        originalProblem->getNonlinearConstraintIndexes())
                                          .first;

        auto nonlinTol = Settings::getInstance().getDoubleSetting("PrimalBoundNonlinearTolerance", "PrimalBound");
=======
        mostDevNonlinearConstraints = this->originalProblem->getMostDeviatingConstraint(tmpPoint, originalProblem->getNonlinearConstraintIndexes()).first;

        auto nonlinTol = Settings::getInstance().getDoubleSetting("Tolerance.NonlinearConstraint", "Primal");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        isNonLinConstrFulfilled = (mostDevNonlinearConstraints.value < nonlinTol);

        if (!isNonLinConstrFulfilled)
        {
            if (originalProblem->isObjectiveFunctionNonlinear() && (mostDevNonlinearConstraints.idx == originalProblem->getNonlinearObjectiveConstraintIdx() || mostDevNonlinearConstraints.idx == -1))
            {
                auto tmpLine =
                    boost::format(
                        "       Nonlinear constraints are not fulfilled. Most deviating is objective constraint: %2% >  %1%.") %
                    nonlinTol % mostDevNonlinearConstraints.value;
<<<<<<< HEAD
                this->outputWarning(tmpLine.str());
=======
                Output::getInstance().outputInfo(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            }
            else
            {
                auto tmpLine = boost::format(
                                   "       Nonlinear constraints are not fulfilled. Most deviating %3%: %2% > %1%.") %
                               nonlinTol % mostDevNonlinearConstraints.value % originalProblem->getConstraintNames().at(mostDevNonlinearConstraints.idx);
<<<<<<< HEAD
                this->outputWarning(tmpLine.str());
=======
                Output::getInstance().outputInfo(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            }

            return (false);
        }
        else
        {
            if (originalProblem->isObjectiveFunctionNonlinear() && (mostDevNonlinearConstraints.idx == originalProblem->getNonlinearObjectiveConstraintIdx() || mostDevNonlinearConstraints.idx == -1))
            {
                auto tmpLine =
                    boost::format(
                        "       Nonlinear constraints are fulfilled. Most deviating is objective constraint: %2% <  %1%.") %
                    nonlinTol % mostDevNonlinearConstraints.value;
<<<<<<< HEAD
                this->outputWarning(tmpLine.str());
=======
                Output::getInstance().outputInfo(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            }
            else
            {
                auto tmpLine = boost::format(
                                   "       Nonlinear constraints are fulfilled. Most deviating %3%: %2% < %1%.") %
                               nonlinTol % mostDevNonlinearConstraints.value % originalProblem->getConstraintNames().at(mostDevNonlinearConstraints.idx);
<<<<<<< HEAD
                this->outputWarning(tmpLine.str());
=======
                Output::getInstance().outputInfo(tmpLine.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            }
        }
    }

<<<<<<< HEAD
    // Checking again since rounding may have affected the outcome
    bool updatePrimal = ((isMinimization && tmpObjVal < this->currentObjectiveBounds.second) || (!isMinimization && tmpObjVal > this->currentObjectiveBounds.second));

    if (updatePrimal)
    {
        char HPobjadded = ' ';

        if (Settings::getInstance().getBoolSetting("UsePrimalObjectiveCut", "MILP") && this->originalProblem->isObjectiveFunctionNonlinear())
        {
            auto objConstrVal = this->originalProblem->calculateConstraintFunctionValue(-1, primalSol.point) - primalSol.point.back();

            if (objConstrVal < 0)
            {

                Hyperplane hyperplane;
                hyperplane.sourceConstraintIndex = -1;
                hyperplane.generatedPoint = tmpPoint;
                hyperplane.source = E_HyperplaneSource::PrimalSolutionSearchInteriorObjective;

                this->hyperplaneWaitingList.push_back(hyperplane);

                auto tmpLine = boost::format("     Primal objective cut added.");

                this->outputWarning(tmpLine.str());
            }
        }

        this->currentObjectiveBounds.second = tmpObjVal;

        if (originalProblem->getNumberOfNonlinearConstraints() > 0)
        {
            auto tmpLine = boost::format("     New primal bound %1% from %2% accepted.") % tmpObjVal % sourceDesc;

            this->outputWarning(tmpLine.str());

            primalSol.objValue = tmpObjVal;
            primalSol.point = tmpPoint;
            primalSol.maxDevatingConstraint = mostDevNonlinearConstraints;
        }
        else
        {
            auto tmpLine = boost::format("     New primal bound %1% from %2% accepted.") % tmpObjVal % sourceDesc;

            this->outputWarning(tmpLine.str());

            primalSol.objValue = tmpObjVal;
            primalSol.point = tmpPoint;
            primalSol.maxDevatingConstraint = mostDevNonlinearConstraints;
        }

        if (Settings::getInstance().getBoolSetting("SaveAllPrimalSolutions", "SHOTSolver"))
=======
    primalSol.maxDevatingConstraintNonlinear = mostDevNonlinearConstraints;

    char HPobjadded = ' ';

    if (Settings::getInstance().getBoolSetting("HyperplaneCuts.UsePrimalObjectiveCut", "Dual") && this->originalProblem->isObjectiveFunctionNonlinear())
    {
        auto objConstrVal = this->originalProblem->calculateConstraintFunctionValue(-1, tmpPoint) - tmpPoint.back();

        if (objConstrVal < 0)
        {
            Hyperplane hyperplane;
            hyperplane.sourceConstraintIndex = -1;
            hyperplane.generatedPoint = tmpPoint;
            hyperplane.source = E_HyperplaneSource::PrimalSolutionSearchInteriorObjective;

            this->hyperplaneWaitingList.push_back(hyperplane);

            auto tmpLine = boost::format("     Primal objective cut added.");

            Output::getInstance().outputWarning(tmpLine.str());
        }
    }

    this->setPrimalBound(tmpObjVal);

    auto tmpLine = boost::format("     New primal bound %1% from %2% accepted.") % tmpObjVal % sourceDesc;

    Output::getInstance().outputInfo(tmpLine.str());

    primalSol.objValue = tmpObjVal;
    primalSol.point = tmpPoint;

    if (Settings::getInstance().getIntSetting("SaveNumberOfSolutions", "Output") > 1)
    {
        this->primalSolutions.insert(this->primalSolutions.begin(), primalSol);
    }
    else
    {
        if (this->primalSolutions.size() == 0)
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        {
            this->primalSolutions.push_back(primalSol);
        }
        else
        {
<<<<<<< HEAD
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

        if (this->interiorPts.size() > 0)
        {
            // Add the new point if it is deeper within the feasible region
            if (primalSol.maxDevatingConstraint.value < this->interiorPts.at(0)->maxDevatingConstraint.value)
            {
                std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());
                tmpIP->point = tmpPoint;
                tmpIP->maxDevatingConstraint = mostDevNonlinearConstraints;

                this->outputWarning(
                    "      Interior point replaced with primal solution point due to constraint deviation.");

                this->interiorPts.back() = tmpIP;
            }
            else if (Settings::getInstance().getIntSetting("AddPrimalBoundAsInteriorPoint", "Algorithm") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth) && mostDevNonlinearConstraints.value < 0)
            {
                std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

                tmpIP->point = tmpPoint;
                tmpIP->maxDevatingConstraint = mostDevNonlinearConstraints;

                this->outputWarning("      Primal solution point used as additional interior point.");

                if (this->interiorPts.size() == this->numOriginalInteriorPoints)
                {
                    this->interiorPts.push_back(tmpIP);
                }
                else
                {
                    this->interiorPts.back() = tmpIP;
                }
            }
            else if (Settings::getInstance().getIntSetting("AddPrimalBoundAsInteriorPoint", "Algorithm") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepNew) && mostDevNonlinearConstraints.value < 0)
            {
                std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

                // Add the new point only
                tmpIP->point = tmpPoint;
                tmpIP->maxDevatingConstraint = mostDevNonlinearConstraints;

                this->outputWarning("      Interior point replaced with primal solution point.");

                this->interiorPts.back() = tmpIP;
            }
            else if (Settings::getInstance().getIntSetting("AddPrimalBoundAsInteriorPoint", "Algorithm") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::OnlyAverage) && mostDevNonlinearConstraints.value < 0)
            {
                std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

                // Find a new point in the midpoint between the original and new
                for (int i = 0; i < tmpPoint.size(); i++)
                {
                    tmpPoint.at(i) = (0.5 * tmpPoint.at(i) + 0.5 * this->interiorPts.at(0)->point.at(i));
                }

                tmpIP->point = tmpPoint;
                tmpIP->maxDevatingConstraint = this->originalProblem->getMostDeviatingConstraint(tmpPoint);

                this->outputWarning("      Interior point replaced with primal solution point.");

                this->interiorPts.back() = tmpIP;
            }
        }

        return (true);
    }

    return (false);
=======
            this->primalSolutions.at(0) = primalSol;
        }
    }

    this->primalSolution = tmpPoint;

    // Write the new primal point to a file
    if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
    {
        stringstream fileName;
        fileName << Settings::getInstance().getStringSetting("Debug.Path", "Output");
        fileName << "/primalpoint";
        fileName << this->primalSolutions.size();
        fileName << ".txt";

        UtilityFunctions::savePrimalSolutionToFile(primalSol, originalProblem->getVariableNames(), fileName.str());
    }

    /*
		 * Extra check for integers...
		 * bool isRounded = false;

		 auto discreteVarIndexes = this->originalProblem->getDiscreteVariableIndices();

		 std::vector<double> ptRounded(tmpPoint);

		 double maxIntegerError = 0.0;

		 for (int i = 0; i < discreteVarIndexes.size(); i++)
		 {
		 int idx = discreteVarIndexes.at(i);
		 double rounded = UtilityFunctions::round(this->primalSolution.at(idx));

		 double error = abs(rounded - this->primalSolution.at(idx));

		 maxIntegerError = max(maxIntegerError, error);

		 if (error > 0)
		 {
		 ptRounded.at(idx) = rounded;
		 isRounded = true;
		 std::cout << "rounded: " << tmpPoint.at(idx) << " to " << ptRounded.at(idx) << " " << sourceDesc
		 << std::endl;
		 }
		 }*/

    return (true);
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

ProcessInfo::ProcessInfo()
{
    createTimer("Total", "Total solution time");
<<<<<<< HEAD

    iterLP = 0;
    iterQP = 0;
    iterFeasMILP = 0;
    iterOptMILP = 0;
    iterFeasMIQP = 0;
    iterOptMIQP = 0;

    numNLPProbsSolved = 0;
    numPrimalFixedNLPProbsSolved = 0;

    itersWithStagnationMILP = 0;
    iterSignificantObjectiveUpdate = 0;
    itersMILPWithoutNLPCall = 0;
    solTimeLastNLPCall = 0;

    numFunctionEvals = 0;
    numGradientEvals = 0;

    iterLastPrimalBoundUpdate = 0;
    iterLastDualBoundUpdate = 0;

    numOriginalInteriorPoints = 0;

    numConstraintsRemovedInPresolve = 0;

    numIntegerCutsAdded = 0;

    currentObjectiveBounds.first = -OSDBL_MAX;
    currentObjectiveBounds.second = OSDBL_MAX;

    tasks = new TaskHandler();

    objectiveUpdatedByLinesearch = false;
=======

    currentObjectiveBounds.first = -OSDBL_MAX;
    currentObjectiveBounds.second = OSDBL_MAX;

    tasks = new TaskHandler();

    objectiveUpdatedByLinesearch = false;

    osilWriter = new OSiLWriter();
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

ProcessInfo::~ProcessInfo()
{
<<<<<<< HEAD
}

=======
    iterations.clear();

    primalSolution.clear();
    iterations.clear();
    primalSolutions.clear();
    dualSolutions.clear();

    primalSolutionCandidates.clear();
    primalFixedNLPCandidates.clear();
    dualSolutionCandidates.clear();

    delete osilWriter;
    osilReaders.clear();

    delete tasks;
}

>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
void ProcessInfo::setOriginalProblem(OptProblemOriginal *problem)
{
    this->originalProblem = problem;

    bool isMinimization = originalProblem->isTypeOfObjectiveMinimize();

    if (isMinimization)
    {
        currentObjectiveBounds.first = -OSDBL_MAX;
        currentObjectiveBounds.second = OSDBL_MAX;
    }
    else
    {
        currentObjectiveBounds.first = OSDBL_MAX;
        currentObjectiveBounds.second = -OSDBL_MAX;
    }
}

void ProcessInfo::createTimer(string name, string description)
{
    Timer tmpTimer = Timer(name, description);

    timers.push_back(tmpTimer);
}

void ProcessInfo::startTimer(string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
<<<<<<< HEAD
        outputError("Timer with name  \" " + name + "\" not found!");
=======
        Output::getInstance().outputError("Timer with name  \" " + name + "\" not found!");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        return;
    }

    timer->start();
}

void ProcessInfo::restartTimer(string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
<<<<<<< HEAD
        outputError("Timer with name  \" " + name + "\" not found!");
=======
        Output::getInstance().outputError("Timer with name  \" " + name + "\" not found!");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        return;
    }

    timer->restart();
}

void ProcessInfo::stopTimer(string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
<<<<<<< HEAD
        outputError("Timer with name  \" " + name + "\" not found!");
=======
        Output::getInstance().outputError("Timer with name  \" " + name + "\" not found!");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        return;
    }

    timer->stop();
}

double ProcessInfo::getElapsedTime(string name)
{
    auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const &T) {
        return (T.name == name);
    });

    if (timer == timers.end())
    {
<<<<<<< HEAD
        outputError("Timer with name  \" " + name + "\" not found!");
=======
        Output::getInstance().outputError("Timer with name  \" " + name + "\" not found!");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        return (0.0);
    }

    return (timer->elapsed());
}

void ProcessInfo::initializeResults(int numObj, int numVar, int numConstr)
{
<<<<<<< HEAD
    osResult = new OSResult();
=======
    osResult = std::unique_ptr<OSResult>(new OSResult());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    osResult->setObjectiveNumber(numObj);
    osResult->setVariableNumber(numVar);
    osResult->setConstraintNumber(numConstr);
}

std::string ProcessInfo::getOSrl()
{
    auto varNames = originalProblem->getVariableNames();
    auto constrNames = originalProblem->getConstraintNames();
    int numConstr = osResult->getConstraintNumber();

    int numVar = osResult->getVariableNumber();

    int numPrimalSols = primalSolutions.size();

<<<<<<< HEAD
    osResult->setNumberOfOtherGeneralResults(1);
    osResult->setOtherGeneralResultName(0, "UsedOptions");

    //std::cout << Settings::getInstance().getSettingsAsString() << std::endl;
=======
    stringstream ssSolver;
    ssSolver << "Supporting Hyperplane Optimization Toolkit, version ";
    ssSolver << SHOT_VERSION_MAJOR << "." << SHOT_VERSION_MINOR << "." << SHOT_VERSION_PATCH;
    osResult->setSolverInvoked(ssSolver.str());

    osResult->setInstanceName(Settings::getInstance().getStringSetting("ProblemName", "Input"));
    osResult->setNumberOfOtherGeneralResults(1);
    osResult->setOtherGeneralResultName(0, "UsedOptions");

>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    osResult->setOtherGeneralResultValue(0, Settings::getInstance().getSettingsAsString());

    if (numPrimalSols == 0)
    {
        osResult->setSolutionNumber(1);
<<<<<<< HEAD

=======
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        osResult->setNumberOfObjValues(0, 1);

        std::stringstream strstrdb;
        strstrdb << std::fixed << std::setprecision(15) << getDualBound();

        osResult->setAnOtherSolutionResult(0, "DualObjectiveBound", strstrdb.str(), "Final solution",
                                           "The dual bound for the objective", 0, NULL);

<<<<<<< HEAD
        if (dualSolutions.size() > 0)
=======
        if (dualSolutions.size() > 0 && dualSolutions.back().point.size() > 0)
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
        {
            osResult->setObjValue(0, 0, -1, "", dualSolutions.back().objValue);

            std::stringstream strstr;
            strstr << std::fixed << std::setprecision(15)
<<<<<<< HEAD
                   << this->originalProblem->getMostDeviatingConstraint(dualSolutions.back().point).value;
=======
                   << dualSolutions.back().objValue;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

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
<<<<<<< HEAD

        osResult->setSolutionNumber(numPrimalSols);

        for (int i = 0; i < numPrimalSols; i++)
        {
            osResult->setNumberOfVarValues(i, numVar);
=======
        int numSaveSolutions = Settings::getInstance().getIntSetting("SaveNumberOfSolutions", "Output");

        /*
        if (Settings::getInstance().getBoolSetting("SaveAllSolutions", "Output"))
        {
            numSaveSolutions = this->primalSolutions.size();
        }*/

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
                    Output::getInstance().outputError("Unknown return code obtained from solver.");
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

>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
            osResult->setNumberOfObjValues(i, 1);
            osResult->setNumberOfPrimalVariableValues(i, numVar);
            osResult->setObjValue(i, 0, -1, "", primalSolutions.at(i).objValue);

<<<<<<< HEAD
            auto primalPoint = primalSolutions.at(i).point;

            osResult->setPrimalVariableValuesDense(i, &primalPoint[0]);

            for (int j = 0; j < numVar; j++)
            {
                osResult->setVarValue(i, j, j, varNames.at(j), primalPoint.at(j));
            }
=======
            osResult->setPrimalVariableValuesDense(i, &primalSolutions.at(i).point[0]);
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

            std::vector<double> tmpConstrVals;

            osResult->setNumberOfDualValues(i, numConstr);

            for (int j = 0; j < numConstr; j++)
            {
<<<<<<< HEAD
                //tmpConstrVals.push_back(originalProblem->calculateConstraintFunctionValue(j, primalPoint));
                osResult->setDualValue(i, j, j, constrNames.at(j),
                                       originalProblem->calculateConstraintFunctionValue(j, primalPoint));
            }

            //osResult->setConstraintValuesDense(i, &tmpConstrVals[0]);
=======
                osResult->setDualValue(i, j, j, constrNames.at(j), originalProblem->calculateConstraintFunctionValue(j, primalSolutions.at(i).point));
            }
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
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

    numPrimalSols = max(1, numPrimalSols); // To make sure we also print the following even if we have no primal solution

<<<<<<< HEAD
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "LP", std::to_string(iterLP), "ProblemsSolved",
                                       "Relaxed LP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "QP", std::to_string(iterQP), "ProblemsSolved",
                                       "Relaxed QP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMILP", std::to_string(iterFeasMILP),
                                       "ProblemsSolved", "MILP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMILP", std::to_string(iterOptMILP), "ProblemsSolved",
                                       "MILP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMIQP", std::to_string(iterFeasMIQP),
                                       "ProblemsSolved", "MIQP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMIQP", std::to_string(iterOptMIQP), "ProblemsSolved",
                                       "MIQP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Total",
                                       std::to_string(iterLP + iterFeasMILP + iterOptMILP + iterQP + iterFeasMIQP + iterOptMIQP), "ProblemsSolved",
                                       "Total number of (MI)QP/(MI)LP subproblems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NLP", std::to_string(numNLPProbsSolved), "ProblemsSolved",
                                       "NLP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Functions", std::to_string(numFunctionEvals), "Evaluations",
                                       "Total number of function evaluations in SHOT", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Gradients", std::to_string(numGradientEvals), "Evaluations",
                                       "Total number of gradient evaluations in SHOT", 0, NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FileName",
                                       this->originalProblem->getProblemInstance()->getInstanceName(), "Problem", "The original filename", 0,
                                       NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberVariables",
                                       std::to_string(this->originalProblem->getProblemInstance()->getVariableNumber()), "Problem",
                                       "Total number of variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberContinousVariables",
                                       std::to_string(
                                           this->originalProblem->getProblemInstance()->getVariableNumber() - this->originalProblem->getNumberOfIntegerVariables() - this->originalProblem->getNumberOfBinaryVariables()),
                                       "Problem",
                                       "Number of continuous variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberBinaryVariables",
                                       std::to_string(this->originalProblem->getProblemInstance()->getNumberOfBinaryVariables()), "Problem",
                                       "Number of binary variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberIntegerVariables",
                                       std::to_string(this->originalProblem->getProblemInstance()->getNumberOfIntegerVariables()), "Problem",
                                       "Number of integer variables", 0, NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberConstraints",
                                       std::to_string(this->originalProblem->getProblemInstance()->getConstraintNumber()), "Problem",
                                       "Number of constraints", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberNonlinearConstraints",
                                       std::to_string(this->originalProblem->getProblemInstance()->getNumberOfNonlinearConstraints()), "Problem",
                                       "Number of nonlinear constraints", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberLinearConstraints",
                                       std::to_string(
                                           this->originalProblem->getProblemInstance()->getConstraintNumber() - this->originalProblem->getNumberOfNonlinearConstraints()),
                                       "Problem",
                                       "Number of linear constraints", 0, NULL);

    std::string modelStatus;
    std::string modelDescription;

    if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        modelStatus = "optimal";
        modelDescription = "Optimal solution found.";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Feasible)
    {
        modelStatus = "feasible";
        modelDescription = "Feasible solution found.";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Unbounded)
    {
        modelStatus = "unbounded";
        modelDescription = "Problem is unbounded.";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Error)
    {
        modelStatus = "error";
        modelDescription = "Error occured.";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        modelStatus = "infeasible";
        modelDescription = "Problem is infeasible.";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
    {
        modelStatus = "feasible";
        modelDescription = "Termination due to iteration limit.";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        modelStatus = "feasible";
        modelDescription = "";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
    {
        modelStatus = "feasible";
        modelDescription = "Termination due to time limit.";
    }
    else
    {
        modelStatus = "NA";
    }

    osResult->setSolutionStatusType(numPrimalSols - 1, modelStatus);

    OSrLWriter writer;

=======
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "LP", std::to_string(this->solutionStatistics.numberOfProblemsLP), "ProblemsSolved",
                                       "Relaxed LP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "QP", std::to_string(this->solutionStatistics.numberOfProblemsQP), "ProblemsSolved",
                                       "Relaxed QP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMILP", std::to_string(this->solutionStatistics.numberOfProblemsFeasibleMILP),
                                       "ProblemsSolved", "MILP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMILP", std::to_string(this->solutionStatistics.numberOfProblemsOptimalMILP), "ProblemsSolved",
                                       "MILP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMIQP", std::to_string(this->solutionStatistics.numberOfProblemsFeasibleMIQP),
                                       "ProblemsSolved", "MIQP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMIQP", std::to_string(this->solutionStatistics.numberOfProblemsOptimalMIQP), "ProblemsSolved",
                                       "MIQP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Total",
                                       std::to_string(this->solutionStatistics.numberOfProblemsLP + this->solutionStatistics.numberOfProblemsFeasibleMILP + this->solutionStatistics.numberOfProblemsOptimalMILP + this->solutionStatistics.numberOfProblemsQP + this->solutionStatistics.numberOfProblemsFeasibleMIQP + this->solutionStatistics.numberOfProblemsOptimalMIQP), "ProblemsSolved",
                                       "Total number of (MI)QP/(MI)LP subproblems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NLP", std::to_string(this->solutionStatistics.getNumberOfTotalNLPProblems()), "ProblemsSolved",
                                       "NLP problems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Functions", std::to_string(this->solutionStatistics.numberOfFunctionEvalutions), "Evaluations",
                                       "Total number of function evaluations in SHOT", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Gradients", std::to_string(this->solutionStatistics.numberOfGradientEvaluations), "Evaluations",
                                       "Total number of gradient evaluations in SHOT", 0, NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberVariables",
                                       std::to_string(this->problemStats.numberOfVariables), "Problem",
                                       "Total number of variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberContinousVariables",
                                       std::to_string(
                                           this->problemStats.numberOfContinousVariables),
                                       "Problem", "Number of continuous variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberBinaryVariables",
                                       std::to_string(this->problemStats.numberOfBinaryVariables), "Problem",
                                       "Number of binary variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberIntegerVariables",
                                       std::to_string(this->problemStats.numberOfIntegerVariables), "Problem",
                                       "Number of integer variables", 0, NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberConstraints",
                                       std::to_string(this->problemStats.numberOfConstraints), "Problem",
                                       "Number of constraints", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberNonlinearConstraints",
                                       std::to_string(this->problemStats.numberOfNonlinearConstraints), "Problem",
                                       "Number of nonlinear constraints", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberLinearConstraints",
                                       std::to_string(
                                           this->problemStats.numberOfLinearConstraints),
                                       "Problem",
                                       "Number of linear constraints", 0, NULL);

    OSrLWriter writer;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    writer.m_bWhiteSpace = false;

    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);

<<<<<<< HEAD
    stringstream ss;
    ss << writer.writeOSrL(osResult);

    read_xml(ss, pt, boost::property_tree::xml_parser::trim_whitespace);

    std::ostringstream oss;
    write_xml(oss, pt, settings);

    return (oss.str());
=======
    stringstream ssOSrL;
    ssOSrL << writer.writeOSrL(osResult.get());

    read_xml(ssOSrL, pt, boost::property_tree::xml_parser::trim_whitespace);

    std::ostringstream ossXML;
    write_xml(ossXML, pt, settings);

    return (ossXML.str());
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

std::string ProcessInfo::getTraceResult()
{
    std::stringstream ss;
<<<<<<< HEAD

    this->originalProblem->getProblemInstance()->initializeNonLinearStructures();

    if (this->originalProblem->getProblemInstance()->getNumberOfQuadraticTerms() > 0)
    {
        this->originalProblem->getProblemInstance()->addQTermsToExpressionTree();
    }

    this->originalProblem->getProblemInstance()->bVariablesModified = true;
    this->originalProblem->getProblemInstance()->bConstraintsModified = true;
    this->originalProblem->getProblemInstance()->bObjectivesModified = true;

    ss << this->originalProblem->getProblemInstance()->getInstanceName() << ",";
    ss << "MINLP"
       << ",";
    ss << "SHOT"
       << ",";
    ss << "IPOpt"
       << ",";
    ss << "Cplex"
       << ",";
    ss << "9999999"
       << ",";
=======
    ss << this->originalProblem->getProblemInstance()->getInstanceName() << ",";

    switch (static_cast<E_SolutionStrategy>(ProcessInfo::getInstance().usedSolutionStrategy))
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

    switch (static_cast<ES_PrimalNLPSolver>(ProcessInfo::getInstance().usedPrimalNLPSolver))
    {
    case (ES_PrimalNLPSolver::None):
        ss << "NONE";
        break;
    case (ES_PrimalNLPSolver::CuttingPlane):
        ss << "SHOT";
        break;
    case (ES_PrimalNLPSolver::GAMS):
        ss << Settings::getInstance().getStringSetting("GAMS.NLP.Solver", "Subsolver");
        break;
    case (ES_PrimalNLPSolver::Ipopt):
        ss << "Ipopt";
        break;
    default:
        ss << "NONE";
        break;
    }

    ss << ",";

    switch (static_cast<ES_MIPSolver>(ProcessInfo::getInstance().usedMIPSolver))
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
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    ss << (originalProblem->getProblemInstance()->getObjectiveMaxOrMins()[0] == "min" ? "0" : "1") << ",";
    ss << this->originalProblem->getProblemInstance()->getConstraintNumber() + 1 << ","; // +1 to comply with GAMS objective style
    ss << this->originalProblem->getProblemInstance()->getVariableNumber() + 1 << ",";   // +1 to comply with GAMS objective style
    ss << this->originalProblem->getNumberOfBinaryVariables() + this->originalProblem->getNumberOfIntegerVariables()
       << ",";

<<<<<<< HEAD
    auto nonzeroes = this->originalProblem->getProblemInstance()->getLinearConstraintCoefficientNumber() + this->originalProblem->getProblemInstance()->getAllNonlinearVariablesIndexMap().size() + 1;
    //this->originalProblem->getObjectiveCoefficientNumbers()[0] + 1;
=======
    if (this->originalProblem->getProblemInstance()->getNumberOfQuadraticTerms() > 0)
    {
        this->originalProblem->getProblemInstance()->addQTermsToExpressionTree();
    }

    auto nonzeroes = this->originalProblem->getProblemInstance()->getJacobianSparsityPattern()->valueSize +
                     this->originalProblem->getProblemInstance()->getObjectiveCoefficientNumbers()[0] +
                     this->originalProblem->getProblemInstance()->getAllNonlinearVariablesIndexMap().size() +
                     1;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

    ss << nonzeroes << ",";
    ss << this->originalProblem->getProblemInstance()->getAllNonlinearVariablesIndexMap().size() << ",";
    ss << "1"
       << ",";

    std::string solverStatus = "";
    std::string modelStatus = "";

<<<<<<< HEAD
    auto t = this->getCurrentIteration()->solutionStatus;

    if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        modelStatus = "8";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Feasible)
    {
        modelStatus = "7";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Unbounded)
    {
        modelStatus = "18";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Error)
    {
        modelStatus = "12";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        modelStatus = "4";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
    {
        modelStatus = "7";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        modelStatus = "7";
    }
    else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
    {
        modelStatus = "7";
    }
    else
    {
        modelStatus = "NA";
        outputError("Unknown return code from model solution.");
    }

    if (this->terminationReason == E_TerminationReason::TimeLimit)
    {
        solverStatus = "3";
    }
    else if (this->terminationReason == E_TerminationReason::IterationLimit)
    {
        solverStatus = "2";
    }
    else if (this->terminationReason == E_TerminationReason::ObjectiveStagnation)
    {
        solverStatus = "2";
    }
    else if (this->terminationReason == E_TerminationReason::Error)
    {
        solverStatus = "10";
    }
    else if (this->terminationReason == E_TerminationReason::InfeasibleProblem)
    {
        solverStatus = "4";
    }
    else if (this->terminationReason == E_TerminationReason::ConstraintTolerance || this->terminationReason == E_TerminationReason::AbsoluteGap || this->terminationReason == E_TerminationReason::RelativeGap)
    {
        solverStatus = "1";
    }
    else
    {
        solverStatus = "10";
        outputError("Unknown return code obtainedfrom solver.");
=======
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
        Output::getInstance().outputError("Unknown return code " + to_string((int)this->terminationReason) + " obtained from solver.");
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
        Output::getInstance().outputError("Unknown return code " + to_string((int)solStatus) + " from model solution.");
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    }

    ss << modelStatus << ",";
    ss << solverStatus << ",";
<<<<<<< HEAD
    ss << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    //ss << this->getCurrentIteration()->objectiveValue << ",";
=======

    ss << std::setprecision(std::numeric_limits<double>::digits10 + 1);
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
    ss << this->getPrimalBound() << ",";
    ;
    ss << this->getDualBound() << ",";
    ;
    ss << this->getElapsedTime("Total") << ",";
<<<<<<< HEAD
    ss << iterFeasMILP + iterOptMILP << ",";
    ss << "0"
       << ",";
    ss << "0"
=======
    ss << this->solutionStatistics.numberOfIterations << ",";
    ss << "0"
       << ",";
    ss << this->solutionStatistics.numberOfExploredNodes
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
       << ",";
    ss << "#";

    return (ss.str());
}

void ProcessInfo::createIteration()
{
<<<<<<< HEAD
    Iteration iter = Iteration();
    iter.iterationNumber = iterations.size() + 1;

    iter.numHyperplanesAdded = 0;

    if (iterations.size() == 0)
        iter.totNumHyperplanes = 0;
    else
        iter.totNumHyperplanes = iterations.at(iterations.size() - 1).totNumHyperplanes;

    iter.maxDeviation = OSDBL_MAX;
    iter.boundaryDistance = OSDBL_MAX;
    iter.MILPSolutionLimitUpdated = false;

    iter.type = relaxationStrategy->getProblemType();
    //getCurrentIteration()->type = relaxationStrategy->getProblemType();
=======
    Iteration iter;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d

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
    auto primalBound = this->currentObjectiveBounds.second;

    return (primalBound);
<<<<<<< HEAD
=======
}

void ProcessInfo::setPrimalBound(double value)
{
    this->currentObjectiveBounds.second = value;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

double ProcessInfo::getDualBound()
{
    auto dualBound = this->currentObjectiveBounds.first;

    return (dualBound);
<<<<<<< HEAD
=======
}

void ProcessInfo::setDualBound(double value)
{
    this->currentObjectiveBounds.first = value;
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

double ProcessInfo::getAbsoluteObjectiveGap()
{
    double gap = abs(getDualBound() - getPrimalBound());

    return (gap);
}

double ProcessInfo::getRelativeObjectiveGap()
{
    double gap = abs(getDualBound() - getPrimalBound()) / ((1e-10) + abs(getPrimalBound()));
<<<<<<< HEAD

    return (gap);
=======

    return (gap);
}

OSInstance *ProcessInfo::getProblemInstanceFromOSiL(std::string osil)
{
    OSiLReader *osilReader = new OSiLReader();
    OSInstance *newInstance = osilReader->readOSiL(osil);

    osilReaders.push_back(osilReader); // To be able to properly deleting them without destroying the OSInstance object
    return (newInstance);
}

std::string ProcessInfo::getOSiLFromProblemInstance(OSInstance *instance)
{
    return (osilWriter->writeOSiL(instance));
>>>>>>> 1cc513f11c0bedf6015f50366a79ea4cd93a390d
}

void ProcessInfo::setProblemStats()
{
    if (this->originalProblem == NULL)
        return;

    auto instance = this->originalProblem->getProblemInstance();

    problemStats.isMinimizationProblem = this->originalProblem->isTypeOfObjectiveMinimize();

    problemStats.objectiveFunctionType = this->originalProblem->getObjectiveFunctionType();

    problemStats.numberOfConstraints = instance->getConstraintNumber();
    problemStats.numberOfNonlinearConstraints = this->originalProblem->getNumberOfNonlinearConstraints();
    problemStats.numberOfQuadraticConstraints = this->originalProblem->getNumberOfQuadraticConstraints();

    problemStats.numberOfLinearConstraints = this->originalProblem->getNumberOfLinearConstraints();

    problemStats.numberOfQuadraticTerms = instance->getNumberOfQuadraticTerms();

    auto QPStrategy = static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"));

    if (QPStrategy == ES_QuadraticProblemStrategy::Nonlinear && problemStats.numberOfQuadraticTerms > 0)
    {
        problemStats.quadraticTermsReformulatedAsNonlinear = true;
    }
    else if (QPStrategy == ES_QuadraticProblemStrategy::QuadraticObjective && !UtilityFunctions::areAllConstraintsQuadratic(instance))
    {
        problemStats.quadraticTermsReformulatedAsNonlinear = true;
    }

    problemStats.numberOfVariables = instance->getVariableNumber();
    problemStats.numberOfIntegerVariables = instance->getNumberOfIntegerVariables();
    problemStats.numberOfBinaryVariables = instance->getNumberOfBinaryVariables();
    problemStats.numberOfSemicontinuousVariables = instance->getNumberOfSemiContinuousVariables();
    problemStats.numberOfContinousVariables = problemStats.numberOfVariables - (problemStats.numberOfIntegerVariables + problemStats.numberOfBinaryVariables + problemStats.numberOfSemicontinuousVariables);

    if (problemStats.numberOfIntegerVariables > 0 || problemStats.numberOfBinaryVariables > 0 || problemStats.numberOfSemicontinuousVariables > 0)
    {
        problemStats.isDiscreteProblem = true;
    }
    else
    {
        problemStats.isDiscreteProblem = false;
    }

    // Classify the problem
    if (!problemStats.isDiscreteProblem)
    {
        if (problemStats.numberOfNonlinearConstraints > 0 ||
            problemStats.objectiveFunctionType == E_ObjectiveFunctionType::Nonlinear ||
            problemStats.objectiveFunctionType == E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear)
        {
            problemStats.problemType = E_ProblemType::NLP;
        }
        else if (problemStats.numberOfQuadraticConstraints > 0)
        {
            problemStats.problemType = E_ProblemType::QCQP;
        }
        else if (problemStats.objectiveFunctionType == E_ObjectiveFunctionType::Quadratic)
        {
            problemStats.problemType = E_ProblemType::QP;
        }
        else
        {
            problemStats.problemType = E_ProblemType::LP;
        }
    }
    else
    {
        if (problemStats.numberOfNonlinearConstraints > 0 ||
            problemStats.objectiveFunctionType == E_ObjectiveFunctionType::Nonlinear ||
            problemStats.objectiveFunctionType == E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear)
        {
            problemStats.problemType = E_ProblemType::MINLP;
        }
        else if (problemStats.numberOfQuadraticConstraints > 0)
        {
            problemStats.problemType = E_ProblemType::MIQCQP;
        }
        else if (problemStats.objectiveFunctionType == E_ObjectiveFunctionType::Quadratic)
        {
            problemStats.problemType = E_ProblemType::MIQP;
        }
        else
        {
            problemStats.problemType = E_ProblemType::MILP;
        }
    }
}