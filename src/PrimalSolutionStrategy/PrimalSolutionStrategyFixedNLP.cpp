/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "PrimalSolutionStrategyFixedNLP.h"

namespace SHOT
{

PrimalSolutionStrategyFixedNLP::PrimalSolutionStrategyFixedNLP(EnvironmentPtr envPtr)
{
    env = envPtr;

    originalNLPTime = env->settings->getDoubleSetting("FixedInteger.Frequency.Time", "Primal");
    originalNLPIter = env->settings->getIntSetting("FixedInteger.Frequency.Iteration", "Primal");

    switch(static_cast<ES_PrimalNLPSolver>(env->settings->getIntSetting("FixedInteger.Solver", "Primal")))
    {
    case(ES_PrimalNLPSolver::Ipopt):
    {
        env->results->usedPrimalNLPSolver = ES_PrimalNLPSolver::Ipopt;
        NLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(
            env, (std::dynamic_pointer_cast<ModelingSystemOS>(env->modelingSystem))->originalInstance);
        break;
    }
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
        env->output->outputError(
            "Error in solver definition for primal NLP solver. Check option 'Primal.FixedInteger.Solver'.");
        throw ErrorClass(
            "Error in solver definition for primal NLP solver. Check option 'Primal.FixedInteger.Solver'.");

        throw std::logic_error("Unknown PrimalNLPSolver setting.");
    }

    if(env->settings->getBoolSetting("FixedInteger.CreateInfeasibilityCut", "Primal"))
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsESH>(env);
        }
        else
        {
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsECP>(env);
        }
    }

    this->originalIterFrequency = env->settings->getIntSetting("FixedInteger.Frequency.Iteration", "Primal");
    this->originalTimeFrequency = env->settings->getDoubleSetting("FixedInteger.Frequency.Time", "Primal");

    for(auto& V : env->problem->binaryVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }

    for(auto& V : env->problem->integerVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }
}

PrimalSolutionStrategyFixedNLP::~PrimalSolutionStrategyFixedNLP()
{
    discreteVariableIndexes.clear();
    testedPoints.clear();
    fixPoint.clear();
}

bool PrimalSolutionStrategyFixedNLP::runStrategy()
{
    if(env->primalSolver->fixedPrimalNLPCandidates.size() == 0)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return (false);
    }

    auto currIter = env->results->getCurrentIteration();

    bool isSolved;

    std::vector<PrimalFixedNLPCandidate> testPts;

    // Check if integer value combination has been tested before
    if(testedPoints.size() > 0)
    {
        for(int j = 0; j < env->primalSolver->fixedPrimalNLPCandidates.size(); j++)
        {
            for(int i = 0; i < testedPoints.size(); i++)
            {
                if(UtilityFunctions::isDifferentRoundedSelectedElements(
                       env->primalSolver->fixedPrimalNLPCandidates.at(j).point, testedPoints.at(i),
                       discreteVariableIndexes))
                {
                    testPts.push_back(env->primalSolver->fixedPrimalNLPCandidates.at(j));
                    testedPoints.push_back(env->primalSolver->fixedPrimalNLPCandidates.at(j).point);
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

        if(env->settings->getBoolSetting("FixedInteger.UsePresolveBounds", "Primal"))
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

            auto tmpSolPt = UtilityFunctions::round(testPts.at(j).point.at(currVarIndex));

            fixedVariableValues.at(k) = tmpSolPt;

            // Sets the starting point to the fixed value
            if(env->settings->getBoolSetting("FixedInteger.Warmstart", "Primal"))
            {
                startingPointIndexes.at(currVarIndex) = currVarIndex;
                startingPointValues.at(currVarIndex) = tmpSolPt;
            }
        }

        if(env->settings->getBoolSetting("FixedInteger.Warmstart", "Primal"))
        {
            for(auto& V : env->problem->realVariables)
            {
                startingPointIndexes.at(V->index) = V->index;
                startingPointValues.at(V->index) = testPts.at(j).point.at(V->index);
            }
        }

        NLPSolver->setStartingPoint(startingPointIndexes, startingPointValues);

        NLPSolver->fixVariables(discreteVariableIndexes, fixedVariableValues);

        if(env->settings->getBoolSetting("Debug.Enable", "Output"))
        {
            std::string filename = env->settings->getStringSetting("Debug.Path", "Output") + "/primalnlp"
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

            if(env->settings->getBoolSetting("FixedInteger.Frequency.Dynamic", "Primal"))
            {
                int iters
                    = std::max(ceil(env->settings->getIntSetting("FixedInteger.Frequency.Iteration", "Primal") * 0.98),
                        originalNLPIter);

                if(iters > std::max(0.1 * this->originalIterFrequency, 1.0))
                    env->settings->updateSetting("FixedInteger.Frequency.Iteration", "Primal", iters);

                double interval = std::max(
                    0.9 * env->settings->getDoubleSetting("FixedInteger.Frequency.Time", "Primal"), originalNLPTime);

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
                    currIter->totNumHyperplanes, env->results->getDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteObjectiveGap(), env->results->getRelativeObjectiveGap(), tmpObj,
                    mostDevConstr.constraint->index, mostDevConstr.normalizedValue, E_IterationLineType::PrimalNLP);
            }
            else
            {
                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteObjectiveGap(), env->results->getRelativeObjectiveGap(), tmpObj,
                    -1, // Not shown
                    0.0, // Not shown
                    E_IterationLineType::PrimalNLP);
            }

            // Add integer cut.
            if(env->settings->getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual")
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

                env->dualSolver->MIPSolver->integerCutWaitingList.push_back(std::make_pair(ones, zeroes));
            }

            if(env->settings->getBoolSetting("FixedInteger.CreateInfeasibilityCut", "Primal"))
            {
                // auto mostDevConstr
                //     = env->problem->getMaxNumericConstraintValue(variableSolution,
                //     env->problem->nonlinearConstraints);

                SolutionPoint tmpSolPt;
                tmpSolPt.point = variableSolution;
                tmpSolPt.objectiveValue = env->problem->objectiveFunction->calculateValue(variableSolution);
                tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
                // tmpSolPt.maxDeviation = PairIndexValue(mostDevConstr.constraint->index,
                // mostDevConstr.normalizedValue);

                for(auto& V : env->reformulatedProblem->auxilliaryVariables)
                {
                    tmpSolPt.point.push_back(V->calculate(variableSolution));
                }

                std::vector<SolutionPoint> solutionPoints(1);
                solutionPoints.at(0) = tmpSolPt;

                if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
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

            auto mostDevConstr
                = env->problem->getMaxNumericConstraintValue(variableSolution, env->problem->nonlinearConstraints);

            if(env->settings->getBoolSetting("FixedInteger.CreateInfeasibilityCut", "Primal"))
            {
                SolutionPoint tmpSolPt;
                tmpSolPt.point = variableSolution;
                tmpSolPt.objectiveValue = env->problem->objectiveFunction->calculateValue(variableSolution);
                tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
                tmpSolPt.maxDeviation = PairIndexValue(mostDevConstr.constraint->index, mostDevConstr.normalizedValue);

                for(auto& V : env->reformulatedProblem->auxilliaryVariables)
                {
                    tmpSolPt.point.push_back(V->calculate(variableSolution));
                }

                solutionPoints.at(0) = tmpSolPt;

                if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
                    == ES_HyperplaneCutStrategy::ESH)
                {
                    std::dynamic_pointer_cast<TaskSelectHyperplanePointsESH>(taskSelectHPPts)->run(solutionPoints);
                }
                else
                {
                    std::dynamic_pointer_cast<TaskSelectHyperplanePointsECP>(taskSelectHPPts)->run(solutionPoints);
                }
            }

            if(env->settings->getBoolSetting("FixedInteger.Frequency.Dynamic", "Primal"))
            {
                int iters = ceil(env->settings->getIntSetting("FixedInteger.Frequency.Iteration", "Primal") * 1.02);

                if(iters < 10 * this->originalIterFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Iteration", "Primal", iters);

                double interval = 1.1 * env->settings->getDoubleSetting("FixedInteger.Frequency.Time", "Primal");

                if(interval < 10 * this->originalTimeFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Time", "Primal", interval);

                env->output->outputInfo("     Duration:  " + std::to_string(timeEnd - timeStart)
                    + " s. New interval: " + std::to_string(interval) + " s or " + std::to_string(iters) + " iters.");
            }

            env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP, ("NLP" + sourceDesc),
                env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded, currIter->totNumHyperplanes,
                env->results->getDualBound(), env->results->getPrimalBound(), env->results->getAbsoluteObjectiveGap(),
                env->results->getRelativeObjectiveGap(), tmpObj, mostDevConstr.constraint->index,
                mostDevConstr.normalizedValue, E_IterationLineType::PrimalNLP);

            // Add integer cut.
            if(env->settings->getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual")
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

                env->dualSolver->MIPSolver->integerCutWaitingList.push_back(std::make_pair(ones, zeroes));
            }
        }

        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP = 0;
        env->solutionStatistics.timeLastFixedNLPCall = env->timing->getElapsedTime("Total");
    }

    env->primalSolver->fixedPrimalNLPCandidates.clear();

    return (true);
}
} // namespace SHOT