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

TaskSelectPrimalCandidatesFromNLP::TaskSelectPrimalCandidatesFromNLP(EnvironmentPtr envPtr) : TaskBase(envPtr)
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
        env->results->usedPrimalNLPSolver = ES_PrimalNLPSolver::Ipopt;
        NLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(env, env->problem);
        break;
    }
#endif

#ifdef HAS_GAMS
    case(ES_PrimalNLPSolver::GAMS):
    {
        env->results->usedPrimalNLPSolver = ES_PrimalNLPSolver::GAMS;
        NLPSolver = std::make_shared<NLPSolverGAMS>(
            env, (std::dynamic_pointer_cast<ModelingSystemGAMS>(env->modelingSystem))->modelingObject);
        break;
    }
#endif

    default:
        auto value = env->settings->getSetting<int>("FixedInteger.Solver", "Primal");
        throw Error("Error in solver definition for primal NLP solver. Value '" + std::to_string(value)
            + "' for setting 'Primal.FixedInteger.Solver' not valid.");
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

    for(auto& V : env->problem->binaryVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }

    for(auto& V : env->problem->integerVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        for(auto& V : env->problem->allVariables)
        {

            variableNames.push_back(V->name);
        }
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

    bool isSolved;

    std::vector<PrimalFixedNLPCandidate> testPts;

    // Check if integer value combination has been tested before
    if(testedPoints.size() > 0)
    {
        for(auto& C : env->primalSolver->fixedPrimalNLPCandidates)
        {
            for(auto& P : testedPoints)
            {
                if(Utilities::isDifferentRoundedSelectedElements(C.point, P, discreteVariableIndexes))
                {
                    testPts.push_back(C);
                    testedPoints.push_back(C.point);
                    break;
                }
            }
        }
    }
    else
    {
        testPts.push_back(env->primalSolver->fixedPrimalNLPCandidates.at(0));
        testedPoints.push_back(env->primalSolver->fixedPrimalNLPCandidates.at(0).point);
    }

    if(testPts.size() == 0)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return (false);
    }

    for(int j = 0; j < testPts.size(); j++)
    {
        auto oldPrimalBound = env->results->getPrimalBound();
        double timeStart = env->timing->getElapsedTime("Total");
        VectorDouble fixedVariableValues(discreteVariableIndexes.size());

        int sizeOfVariableVector = env->problem->properties.numberOfVariables;

        if(env->settings->getSetting<bool>("FixedInteger.UsePresolveBounds", "Primal"))
        {
            for(auto& V : env->reformulatedProblem->allVariables)
            {
                if(V->index > sizeOfVariableVector)
                    continue;

                if(V->hasUpperBoundBeenTightened)
                {
                    std::cout << "updated UB in NLP\n";
                    NLPSolver->updateVariableUpperBound(V->index, V->upperBound);
                }

                if(V->hasLowerBoundBeenTightened)
                {
                    std::cout << "updated LB in NLP\n";
                    NLPSolver->updateVariableLowerBound(V->index, V->upperBound);
                }
            }
        }

        VectorInteger startingPointIndexes(sizeOfVariableVector);
        VectorDouble startingPointValues(sizeOfVariableVector);

        // Sets the fixed values for discrete variables
        for(int k = 0; k < discreteVariableIndexes.size(); k++)
        {
            int currVarIndex = discreteVariableIndexes.at(k);

            auto tmpSolPt = std::round(testPts.at(j).point.at(currVarIndex));

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
            for(auto& V : env->problem->realVariables)
            {
                startingPointIndexes.at(V->index) = V->index;
                startingPointValues.at(V->index) = testPts.at(j).point.at(V->index);
            }

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                    + "/primalnlp_warmstart" + std::to_string(currIter->iterationNumber) + "_" + std::to_string(j)
                    + ".txt";

                Utilities::saveVariablePointVectorToFile(startingPointValues, variableNames, filename);
            }

            NLPSolver->setStartingPoint(startingPointIndexes, startingPointValues);
        }

        NLPSolver->fixVariables(discreteVariableIndexes, fixedVariableValues);

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output") + "/primalnlp"
                + std::to_string(currIter->iterationNumber) + "_" + std::to_string(j);
            NLPSolver->saveProblemToFile(filename + ".txt");
            NLPSolver->saveOptionsToFile(filename + ".osrl");
        }

        auto solvestatus = NLPSolver->solveProblem();

        NLPSolver->unfixVariables();
        env->solutionStatistics.numberOfProblemsFixedNLP++;

        double timeEnd = env->timing->getElapsedTime("Total");

        std::string sourceDesc;
        switch(testPts.at(j).sourceType)
        {
        case E_PrimalNLPSource::FirstSolution:
            sourceDesc = "SOLPT ";
            break;
        case E_PrimalNLPSource::FeasibleSolution:
            sourceDesc = "FEASPT";
            break;
        case E_PrimalNLPSource::InfeasibleSolution:
            sourceDesc = "UNFEAS";
            break;
        case E_PrimalNLPSource::SmallestDeviationSolution:
            sourceDesc = "SMADEV";
            break;
        case E_PrimalNLPSource::FirstSolutionNewDualBound:
            sourceDesc = "NEWDB";
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
                    ceil(env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal") * 0.98),
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

            if(env->problem->properties.numberOfNonlinearConstraints > 0)
            {
                auto mostDevConstr
                    = env->problem->getMaxNumericConstraintValue(variableSolution, env->problem->nonlinearConstraints);

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
                && env->problem->properties.numberOfBinaryVariables > 0
                && env->problem->properties.numberOfIntegerVariables == 0)
            {
                VectorInteger ones;
                VectorInteger zeroes;
                for(auto& V : env->problem->binaryVariables)
                {
                    if(testPts.at(j).point.at(V->index) > 0.9999)
                    {
                        ones.push_back(V->index);
                    }
                    else
                    {
                        zeroes.push_back(V->index);
                    }
                }

                env->dualSolver->MIPSolver->integerCutWaitingList.emplace_back(ones, zeroes);
            }

            if(env->settings->getSetting<bool>("FixedInteger.CreateInfeasibilityCut", "Primal"))
            {
                SolutionPoint tmpSolPt;
                tmpSolPt.point = variableSolution;
                tmpSolPt.objectiveValue = env->problem->objectiveFunction->calculateValue(variableSolution);
                tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;

                for(auto& V : env->reformulatedProblem->auxiliaryVariables)
                {
                    tmpSolPt.point.push_back(V->calculate(variableSolution));
                }

                if(env->reformulatedProblem->auxiliaryObjectiveVariable)
                {
                    tmpSolPt.point.push_back(
                        env->reformulatedProblem->auxiliaryObjectiveVariable->calculate(variableSolution));
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
        else if(env->problem->properties.numberOfNonlinearConstraints == 0)
        {
            // TODO
        }
        else if(env->problem->properties.numberOfNonlinearConstraints > 0)
        {
            double tmpObj = NLPSolver->getObjectiveValue();

            // Utilize the solution point for adding a cutting plane / supporting hyperplane
            std::vector<SolutionPoint> solutionPoints(1);

            auto variableSolution = NLPSolver->getSolution();

            if(variableSolution.size() > 0)
            {
                auto mostDevConstr
                    = env->problem->getMaxNumericConstraintValue(variableSolution, env->problem->nonlinearConstraints);

                if(env->settings->getSetting<bool>("FixedInteger.CreateInfeasibilityCut", "Primal"))
                {
                    SolutionPoint tmpSolPt;
                    tmpSolPt.point = variableSolution;
                    tmpSolPt.objectiveValue = env->problem->objectiveFunction->calculateValue(variableSolution);
                    tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
                    tmpSolPt.maxDeviation
                        = PairIndexValue(mostDevConstr.constraint->index, mostDevConstr.normalizedValue);

                    for(auto& V : env->reformulatedProblem->auxiliaryVariables)
                    {
                        tmpSolPt.point.push_back(V->calculate(variableSolution));
                    }

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
                int iters = ceil(env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal") * 1.02);

                if(iters < 10 * this->originalIterFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Iteration", "Primal", iters);

                double interval = 1.1 * env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal");

                if(interval < 10 * this->originalTimeFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Time", "Primal", interval);

                env->output->outputDebug("     Duration:  " + std::to_string(timeEnd - timeStart)
                    + " s. New interval: " + std::to_string(interval) + " s or " + std::to_string(iters) + " iters.");
            }

            // Add integer cut.
            if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual")
                && env->problem->properties.numberOfBinaryVariables > 0
                && env->problem->properties.numberOfIntegerVariables == 0)
            {
                VectorInteger ones;
                VectorInteger zeroes;
                for(auto& V : env->problem->binaryVariables)
                {
                    if(testPts.at(j).point.at(V->index) > 0.9999)
                    {
                        ones.push_back(V->index);
                    }
                    else
                    {
                        zeroes.push_back(V->index);
                    }
                }

                env->dualSolver->MIPSolver->integerCutWaitingList.emplace_back(ones, zeroes);
            }
        }

        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP = 0;
        env->solutionStatistics.timeLastFixedNLPCall = env->timing->getElapsedTime("Total");
    }

    env->primalSolver->fixedPrimalNLPCandidates.clear();

    return (true);
}

} // namespace SHOT