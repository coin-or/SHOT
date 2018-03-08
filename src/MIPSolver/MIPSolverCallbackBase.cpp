/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCallbackBase.h"

bool MIPSolverCallbackBase::checkIterationLimit()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (currIter->iterationNumber >= Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual") + Settings::getInstance().getIntSetting("IterationLimit", "Termination"))
    {
        return (true);
    }

    return (false);
}

bool MIPSolverCallbackBase::checkFixedNLPStrategy(SolutionPoint point)
{
    if (!Settings::getInstance().getBoolSetting("FixedInteger.Use", "Primal"))
    {
        return (false);
    }

    ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
    ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");

    bool callNLPSolver = false;

    auto userSettingStrategy = Settings::getInstance().getIntSetting("FixedInteger.CallStrategy", "Primal");
    auto userSetting = Settings::getInstance().getIntSetting("FixedInteger.Source", "Primal");

    auto dualBound = ProcessInfo::getInstance().getDualBound();

    if (abs(point.objectiveValue - dualBound) / ((1e-10) + abs(dualBound)) < Settings::getInstance().getDoubleSetting("FixedInteger.DualPointGap.Relative", "Primal"))
    {
        callNLPSolver = true;
    }
    else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
    {
        callNLPSolver = true;
    }
    else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime) || userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
    {
        if (ProcessInfo::getInstance().MIPIterationsWithoutNLPCall >= Settings::getInstance().getIntSetting("FixedInteger.Frequency.Iteration", "Primal"))
        {
            ProcessInfo::getInstance().outputInfo(
                "     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
            callNLPSolver = true;
        }
        else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().solTimeLastNLPCall > Settings::getInstance().getDoubleSetting("FixedInteger.Frequency.Time", "Primal"))
        {
            ProcessInfo::getInstance().outputInfo(
                "     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
            callNLPSolver = true;
        }
    }

    if (!callNLPSolver)
    {
        ProcessInfo::getInstance().MIPIterationsWithoutNLPCall++;
    }

    ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
    ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");

    return (callNLPSolver);
}

void MIPSolverCallbackBase::printIterationReport(SolutionPoint solution, std::string threadId, std::string bestBound, std::string openNodes)
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (currIter->iterationNumber - lastHeaderIter > 100)
    {
        lastHeaderIter = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
        ProcessInfo::getInstance().tasks->getTask("PrintIterHeader")->run();
    }

    std::stringstream tmpType;
    if (threadId != "")
    {
        tmpType << "CB (th: " << threadId << ")";
    }
    else
    {
        tmpType << "CB";
    }

    std::string hyperplanesExpr;

    if (this->lastNumAddedHyperplanes > 0)
    {
        hyperplanesExpr = "+" + to_string(this->lastNumAddedHyperplanes) + " = " + to_string(currIter->totNumHyperplanes);
    }
    else
    {
        hyperplanesExpr = " ";
    }

    auto tmpConstrExpr = UtilityFunctions::toStringFormat(solution.maxDeviation.value, "%.5f");

    if (solution.maxDeviation.idx != -1)
    {
        tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames()[solution.maxDeviation.idx] + ": " + tmpConstrExpr;
    }
    else
    {
        tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": " + tmpConstrExpr;
    }

    std::string primalBoundExpr;

    if (ProcessInfo::getInstance().primalSolutions.size() > 0 && !ProcessInfo::getInstance().primalSolutions.at(0).displayed)
    {
        auto primalBound = ProcessInfo::getInstance().getPrimalBound();
        primalBoundExpr = UtilityFunctions::toString(primalBound);
        ProcessInfo::getInstance().primalSolutions.at(0).displayed = true;
    }
    else
    {
        primalBoundExpr = "";
    }

    std::string dualBoundExpr;

    if (ProcessInfo::getInstance().dualSolutions.size() > 0 && !ProcessInfo::getInstance().dualSolutions.at(0).displayed)
    {
        auto dualBound = ProcessInfo::getInstance().getDualBound();
        dualBoundExpr = UtilityFunctions::toString(dualBound);
        ProcessInfo::getInstance().dualSolutions.at(0).displayed = true;
    }
    else
    {
        dualBoundExpr = "";
    }

    std::string objExpr;

    objExpr = UtilityFunctions::toStringFormat(solution.objectiveValue, "%.3f", true);

    auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % to_string(currIter->iterationNumber) % tmpType.str() % hyperplanesExpr % dualBoundExpr % objExpr % primalBoundExpr % tmpConstrExpr;

    ProcessInfo::getInstance().outputSummary(tmpLine.str());

    double timeStamp = ProcessInfo::getInstance().getElapsedTime("Total");

    if (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber - lastSummaryIter > 50 || timeStamp - lastSummaryTimeStamp > 5)
    {
        lastSummaryIter = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
        lastSummaryTimeStamp = timeStamp;
        double absGap = ProcessInfo::getInstance().getAbsoluteObjectiveGap();
        double relGap = ProcessInfo::getInstance().getRelativeObjectiveGap();
        auto objBounds = ProcessInfo::getInstance().getCorrectedObjectiveBounds();
        double objLB = objBounds.first;
        double objUB = objBounds.second;

        ProcessInfo::getInstance().outputSummary(
            "                                                                                     ");

#ifdef _WIN32
        ProcessInfo::getInstance().outputSummary(
            "ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
        ProcessInfo::getInstance().outputSummary(
            "─────────────────────────────────────────────────────────────────────────────────────");
#endif

        auto tmpLineSummary = boost::format(" At %1% s the obj. bound is %|24t|[%2%, %3%] %|46t|with abs/"
                                            "rel gap %4% / %5%") %
                              ProcessInfo::getInstance().getElapsedTime("Total") % UtilityFunctions::toStringFormat(objLB, "%.3f", true) % UtilityFunctions::toStringFormat(objUB, "%.3f", true) % UtilityFunctions::toStringFormat(absGap, "%.4f", true) % UtilityFunctions::toStringFormat(relGap, "%.4f", true);

        ProcessInfo::getInstance().outputSummary(tmpLineSummary.str());

        std::stringstream tmpLine;

        tmpLine << " Number of open nodes: " << openNodes << ".";

        if (ProcessInfo::getInstance().interiorPts.size() > 1)
        {
            tmpLine << " Number of interior points: " << ProcessInfo::getInstance().interiorPts.size() << ".";
        }

        if (ProcessInfo::getInstance().numIntegerCutsAdded > 1)
        {
            tmpLine << " Number of integer cuts: " << ProcessInfo::getInstance().numIntegerCutsAdded << ".";
        }

        ProcessInfo::getInstance().outputSummary(tmpLine.str());

#ifdef _WIN32
        ProcessInfo::getInstance().outputSummary(
            "ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
        ProcessInfo::getInstance().outputSummary(
            "─────────────────────────────────────────────────────────────────────────────────────");
#endif

        ProcessInfo::getInstance().outputSummary("");
    }
}

MIPSolverCallbackBase::~MIPSolverCallbackBase()
{
}