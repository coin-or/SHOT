/** getCurrentDualBound() getCurrentDualBound() getCurrentDualBound()
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromNLP.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"
#include "../NLPSolver/INLPSolver.h"

#include "../Tasks/TaskSelectHyperplanePointsESH.h"
#include "../Tasks/TaskSelectHyperplanePointsECP.h"

#ifdef HAS_IPOPT
#include "../NLPSolver/NLPSolverIpoptRelaxed.h"
#endif

#ifdef HAS_GAMS
#include "../NLPSolver/NLPSolverGAMS.h"
#include "../ModelingSystem/ModelingSystemGAMS.h"
#endif

namespace SHOT
{

TaskSelectPrimalCandidatesFromNLP::TaskSelectPrimalCandidatesFromNLP(EnvironmentPtr envPtr, bool useReformulatedProblem)
    : TaskBase(envPtr)
{
    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    originalNLPTime = env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal");
    originalNLPIter = env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal");

    switch(static_cast<ES_PrimalNLPSolver>(env->settings->getSetting<int>("FixedInteger.Solver", "Primal")))
    {

#ifdef HAS_IPOPT
    case(ES_PrimalNLPSolver::Ipopt):
    {
        if(useReformulatedProblem)
        {
            sourceProblem = env->reformulatedProblem;
            sourceIsReformulatedProblem = true;
        }
        else
        {
            sourceProblem = env->problem;
            sourceIsReformulatedProblem = false;
        }

        env->results->usedPrimalNLPSolver = ES_PrimalNLPSolver::Ipopt;
        NLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(env, sourceProblem);
        env->results->usedPrimalNLPSolverVersion = NLPSolver->getSolverVersion();
        break;
    }
#endif

#ifdef HAS_GAMS
    case(ES_PrimalNLPSolver::GAMS):
    {
        // GAMS only has the original problem model
        sourceProblem = env->problem;
        sourceIsReformulatedProblem = false;

        env->results->usedPrimalNLPSolver = ES_PrimalNLPSolver::GAMS;
        NLPSolver = std::make_shared<NLPSolverGAMS>(env,
            (std::dynamic_pointer_cast<ModelingSystemGAMS>(env->modelingSystem))->modelingObject,
            (std::dynamic_pointer_cast<ModelingSystemGAMS>(env->modelingSystem))->auditLicensing);
        break;
    }
#endif

    default:
        // We should never get here since there is a check in Solver.cpp that makes sure that the correct solver is used
        break;
    }

    if(env->settings->getSetting<bool>("FixedInteger.CreateInfeasibilityCut", "Primal"))
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsESH>(env);
        }
        else
        {
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsECP>(env);
        }
    }

    this->originalIterFrequency = env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal");
    this->originalTimeFrequency = env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal");

    for(auto& V : sourceProblem->binaryVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }

    for(auto& V : sourceProblem->integerVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        for(auto& V : sourceProblem->allVariables)
        {
            variableNames.push_back(V->name);
        }
    }

    for(auto& V : sourceProblem->allVariables)
    {
        NLPSolver->updateVariableLowerBound(V->index, V->lowerBound);
        NLPSolver->updateVariableUpperBound(V->index, V->upperBound);
    }

    env->timing->stopTimer("PrimalBoundStrategyNLP");
    env->timing->stopTimer("PrimalStrategy");
}

TaskSelectPrimalCandidatesFromNLP::~TaskSelectPrimalCandidatesFromNLP() = default;

void TaskSelectPrimalCandidatesFromNLP::run()
{
    if(env->primalSolver->fixedPrimalNLPCandidates.size() == 0)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return;
    }

    if(env->results->getRelativeGlobalObjectiveGap() < 1e-10)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return;
    }

    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    solveFixedNLP();

    env->timing->stopTimer("PrimalBoundStrategyNLP");
    env->timing->stopTimer("PrimalStrategy");

    return;
}

std::string TaskSelectPrimalCandidatesFromNLP::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

bool TaskSelectPrimalCandidatesFromNLP::solveFixedNLP()
{
    auto currIter = env->results->getCurrentIteration();

    std::vector<PrimalFixedNLPCandidate> testPts;

    env->output->outputDebug("        Solving fixed NLP problem:");

    if(env->primalSolver->fixedPrimalNLPCandidates.size() == 0)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return (false);
    }

    int counter = 0;

    for(auto& CAND : env->primalSolver->fixedPrimalNLPCandidates)
    {
        double timeStart = env->timing->getElapsedTime("Total");
        VectorDouble fixedVariableValues(discreteVariableIndexes.size());

        int sizeOfVariableVector = sourceProblem->properties.numberOfVariables;

        // TODO: remove?
        if(env->settings->getSetting<bool>("FixedInteger.UsePresolveBounds", "Primal"))
        {
            for(auto& V : env->reformulatedProblem->allVariables)
            {
                if(V->index > sizeOfVariableVector)
                    continue;

                if(V->properties.hasUpperBoundBeenTightened)
                {
                    NLPSolver->updateVariableUpperBound(V->index, V->upperBound);
                }

                if(V->properties.hasLowerBoundBeenTightened)
                {
                    NLPSolver->updateVariableLowerBound(V->index, V->upperBound);
                }
            }
        }

        VectorInteger startingPointIndexes(sizeOfVariableVector);
        VectorDouble startingPointValues(sizeOfVariableVector);

        // Sets the fixed values for discrete variables
        for(size_t k = 0; k < discreteVariableIndexes.size(); k++)
        {
            int currVarIndex = discreteVariableIndexes.at(k);

            auto tmpSolPt = std::round(CAND.point.at(currVarIndex));

            fixedVariableValues.at(k) = tmpSolPt;

            // Sets the starting point to the fixed value
            if(env->settings->getSetting<bool>("FixedInteger.Warmstart", "Primal"))
            {
                startingPointIndexes.at(currVarIndex) = currVarIndex;
                startingPointValues.at(currVarIndex) = tmpSolPt;
            }
        }

        if(env->settings->getSetting<bool>("FixedInteger.Warmstart", "Primal"))
        {
            for(auto& V : sourceProblem->realVariables)
            {
                startingPointIndexes.at(V->index) = V->index;
                startingPointValues.at(V->index) = CAND.point.at(V->index);
            }

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                    + "/primalnlp_warmstart" + std::to_string(currIter->iterationNumber) + "_" + std::to_string(counter)
                    + ".txt";

                Utilities::saveVariablePointVectorToFile(startingPointValues, variableNames, filename);
            }

            NLPSolver->setStartingPoint(startingPointIndexes, startingPointValues);
        }

        NLPSolver->fixVariables(discreteVariableIndexes, fixedVariableValues);

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output") + "/primalnlp"
                + std::to_string(currIter->iterationNumber) + "_" + std::to_string(counter);
            NLPSolver->saveProblemToFile(filename + ".txt");
            NLPSolver->saveOptionsToFile(filename + ".osrl");
        }

        auto solvestatus = NLPSolver->solveProblem();

        NLPSolver->unfixVariables();
        env->solutionStatistics.numberOfProblemsFixedNLP++;

        double timeEnd = env->timing->getElapsedTime("Total");

        std::string source = (sourceIsReformulatedProblem) ? "R" : "O";

        std::string sourceDesc;
        switch(CAND.sourceType)
        {
        case E_PrimalNLPSource::FirstSolution:
            sourceDesc = "SOLPT-" + source;
            break;
        case E_PrimalNLPSource::FeasibleSolution:
            sourceDesc = "FEASP-" + source;
            break;
        case E_PrimalNLPSource::InfeasibleSolution:
            sourceDesc = "UNFEA-" + source;
            break;
        case E_PrimalNLPSource::SmallestDeviationSolution:
            sourceDesc = "SMDEV-" + source;
            break;
        case E_PrimalNLPSource::FirstSolutionNewDualBound:
            sourceDesc = "NEWDB-" + source;
            break;
        default:
            break;
        }

        if(solvestatus == E_NLPSolutionStatus::Feasible || solvestatus == E_NLPSolutionStatus::Optimal)
        {
            double tmpObj = NLPSolver->getObjectiveValue();
            auto variableSolution = NLPSolver->getSolution();

            if(env->settings->getSetting<bool>("FixedInteger.Frequency.Dynamic", "Primal"))
            {
                int iters = std::max(
                    std::ceil(env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal") * 0.98),
                    originalNLPIter);

                if(iters > std::max(0.1 * this->originalIterFrequency, 1.0))
                    env->settings->updateSetting("FixedInteger.Frequency.Iteration", "Primal", iters);

                double interval = std::max(
                    0.9 * env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal"), originalNLPTime);

                if(interval > 0.1 * this->originalTimeFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Time", "Primal", interval);
            }

            env->primalSolver->addPrimalSolutionCandidate(
                variableSolution, E_PrimalSolutionSource::NLPFixedIntegers, currIter->iterationNumber);

            if(sourceProblem->properties.numberOfNonlinearConstraints > 0)
            {
                auto mostDevConstr = sourceProblem->getMaxNumericConstraintValue(
                    variableSolution, sourceProblem->nonlinearConstraints);

                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(),
                    tmpObj, mostDevConstr.constraint->index, mostDevConstr.normalizedValue,
                    E_IterationLineType::PrimalNLP);
            }
            else
            {
                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(),
                    tmpObj,
                    -1, // Not shown
                    0.0, // Not shown
                    E_IterationLineType::PrimalNLP);
            }

            // Add integer cut.
            if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual")
                && sourceProblem->properties.numberOfDiscreteVariables > 0)
            {
                IntegerCut integerCut;
                integerCut.source = E_IntegerCutSource::NLPFixedInteger;
                integerCut.variableValues.reserve(discreteVariableIndexes.size());

                for(auto& I : discreteVariableIndexes)
                    integerCut.variableValues.push_back(std::abs(round(CAND.point.at(I))));

                env->dualSolver->addIntegerCut(integerCut);
            }

            if(env->settings->getSetting<bool>("FixedInteger.CreateInfeasibilityCut", "Primal"))
            {
                SolutionPoint tmpSolPt;
                tmpSolPt.point = variableSolution;
                tmpSolPt.objectiveValue = sourceProblem->objectiveFunction->calculateValue(variableSolution);
                tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;

                if(!sourceIsReformulatedProblem) // Need to calculate values for the auxiliary variables in this
                                                 // case
                {
                    for(auto& V : env->reformulatedProblem->auxiliaryVariables)
                    {
                        tmpSolPt.point.push_back(V->calculate(variableSolution));
                    }

                    if(env->reformulatedProblem->auxiliaryObjectiveVariable)
                    {
                        tmpSolPt.point.push_back(
                            env->reformulatedProblem->auxiliaryObjectiveVariable->calculate(variableSolution));
                    }
                }

                std::vector<SolutionPoint> solutionPoints(1);
                solutionPoints.at(0) = tmpSolPt;

                if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
                    == ES_HyperplaneCutStrategy::ESH)
                {
                    std::dynamic_pointer_cast<TaskSelectHyperplanePointsESH>(taskSelectHPPts)->run(solutionPoints);
                }
                else
                {
                    std::dynamic_pointer_cast<TaskSelectHyperplanePointsECP>(taskSelectHPPts)->run(solutionPoints);
                }
            }
        }
        else if(sourceProblem->properties.numberOfNonlinearConstraints > 0)
        {
            double tmpObj = NLPSolver->getObjectiveValue();

            // Utilize the solution point for adding a cutting plane / supporting hyperplane

            auto variableSolution = NLPSolver->getSolution();

            if(variableSolution.size() > 0)
            {
                auto mostDevConstr = sourceProblem->getMaxNumericConstraintValue(
                    variableSolution, sourceProblem->nonlinearConstraints);

                if(env->settings->getSetting<bool>("FixedInteger.CreateInfeasibilityCut", "Primal"))
                {
                    SolutionPoint tmpSolPt;
                    tmpSolPt.point = variableSolution;
                    tmpSolPt.objectiveValue = sourceProblem->objectiveFunction->calculateValue(variableSolution);
                    tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
                    tmpSolPt.maxDeviation
                        = PairIndexValue(mostDevConstr.constraint->index, mostDevConstr.normalizedValue);

                    if(!sourceIsReformulatedProblem) // Need to calculate values for the auxiliary variables in this
                                                     // case
                    {
                        for(auto& V : env->reformulatedProblem->auxiliaryVariables)
                        {
                            tmpSolPt.point.push_back(V->calculate(variableSolution));
                        }

                        if(env->reformulatedProblem->auxiliaryObjectiveVariable)
                        {
                            tmpSolPt.point.push_back(
                                env->reformulatedProblem->auxiliaryObjectiveVariable->calculate(variableSolution));
                        }
                    }

                    std::vector<SolutionPoint> solutionPoints(1);
                    solutionPoints.at(0) = tmpSolPt;

                    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
                        == ES_HyperplaneCutStrategy::ESH)
                    {
                        std::dynamic_pointer_cast<TaskSelectHyperplanePointsESH>(taskSelectHPPts)->run(solutionPoints);
                    }
                    else
                    {
                        std::dynamic_pointer_cast<TaskSelectHyperplanePointsECP>(taskSelectHPPts)->run(solutionPoints);
                    }
                }

                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(),
                    tmpObj, mostDevConstr.constraint->index, mostDevConstr.normalizedValue,
                    E_IterationLineType::PrimalNLP);
            }
            else
            {
                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(), NAN,
                    -1, NAN, E_IterationLineType::PrimalNLP);
            }

            if(env->settings->getSetting<bool>("FixedInteger.Frequency.Dynamic", "Primal"))
            {
                int iters
                    = std::ceil(env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal") * 1.02);

                if(iters < 10 * this->originalIterFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Iteration", "Primal", iters);

                double interval = 1.1 * env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal");

                if(interval < 10 * this->originalTimeFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Time", "Primal", interval);

                env->output->outputDebug(fmt::format("        Duration: {} s. New interval: {} s or {} iterations.",
                    timeEnd - timeStart, interval, iters));
            }

            // Add integer cut.
            if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual")
                && sourceProblem->properties.numberOfDiscreteVariables > 0)
            {
                IntegerCut integerCut;
                integerCut.source = E_IntegerCutSource::NLPFixedInteger;
                integerCut.variableValues.reserve(discreteVariableIndexes.size());

                for(auto& I : discreteVariableIndexes)
                    integerCut.variableValues.push_back(std::abs(round(CAND.point.at(I))));

                env->dualSolver->addIntegerCut(integerCut);
            }
        }
        else
        {
            env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP, ("NLP" + sourceDesc),
                env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded, currIter->totNumHyperplanes,
                env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(), NAN, -1,
                NAN, E_IterationLineType::PrimalNLP);

            // Add integer cut.
            if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual")
                && sourceProblem->properties.numberOfDiscreteVariables > 0)
            {
                IntegerCut integerCut;
                integerCut.source = E_IntegerCutSource::NLPFixedInteger;
                integerCut.variableValues.reserve(discreteVariableIndexes.size());

                for(auto& I : discreteVariableIndexes)
                    integerCut.variableValues.push_back(std::abs(round(CAND.point.at(I))));

                env->dualSolver->addIntegerCut(integerCut);
            }
        }

        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP = 0;
        env->solutionStatistics.timeLastFixedNLPCall = env->timing->getElapsedTime("Total");
        counter++;

        env->primalSolver->usedPrimalNLPCandidates.push_back(CAND);
    }

    return (true);
}

} // namespace SHOT