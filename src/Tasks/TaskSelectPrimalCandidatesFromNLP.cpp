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
#include "../Solver.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"
#include "../NLPSolver/INLPSolver.h"

#include "../Tasks/TaskSelectHyperplanesESH.h"
#include "../Tasks/TaskSelectHyperplanesECP.h"

#ifdef HAS_IPOPT
#include "../NLPSolver/NLPSolverIpoptRelaxed.h"
#endif

#ifdef HAS_GAMS
#include "../NLPSolver/NLPSolverGAMS.h"
#include "../ModelingSystem/ModelingSystemGAMS.h"
#endif

#include "../NLPSolver/NLPSolverSHOT.h"

namespace SHOT
{

TaskSelectPrimalCandidatesFromNLP::TaskSelectPrimalCandidatesFromNLP(EnvironmentPtr envPtr, bool useReformulatedProblem)
    : TaskBase(envPtr)
{
    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    originalNLPTime = env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal");
    originalNLPIter = env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal");

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

    case(ES_PrimalNLPSolver::SHOT):
    {
        // Always use the reformulated problem with SHOT
        sourceProblem = env->reformulatedProblem;

        env->results->usedPrimalNLPSolver = ES_PrimalNLPSolver::SHOT;
        NLPSolver = std::make_shared<NLPSolverSHOT>(env, sourceProblem);
        sourceIsReformulatedProblem = true;

        break;
    }

    default:
        // We should never get here since there is a check in Solver.cpp that makes sure that the correct solver is used
        break;
    }

    env->results->usedPrimalNLPSolverDescription = NLPSolver->getSolverDescription();

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

    for(auto& V : sourceProblem->semiintegerVariables)
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
        env->output->outputDebug("         No candidate points available.");
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return (false);
    }

    int counter = 0;

    for(auto& CAND : env->primalSolver->fixedPrimalNLPCandidates)
    {
        VectorDouble fixedVariableValues(discreteVariableIndexes.size());

        int sizeOfVariableVector = sourceProblem->properties.numberOfVariables;

        // TODO: remove?
        if(env->settings->getSetting<bool>("FixedInteger.UsePresolveBounds", "Primal"))
        {
            env->output->outputDebug("         Updating variable bounds from MIP presolve.");
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
            env->output->outputDebug(
                "         Setting warm start for continuous variable to candidate solution value.");

            for(auto& V : sourceProblem->realVariables)
            {
                startingPointIndexes.at(V->index) = V->index;
                startingPointValues.at(V->index) = CAND.point.at(V->index);
            }

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                auto filename = fmt::format("{}/primalnlp{}_warmstart_{}.txt",
                    env->settings->getSetting<std::string>("Debug.Path", "Output"),
                    env->results->getCurrentIteration()->iterationNumber - 1, counter);

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

        std::string source = (sourceIsReformulatedProblem) ? "R" : "O";

        std::string sourceDesc;
        switch(CAND.sourceType)
        {
        case E_PrimalNLPSource::FirstSolution:
            env->output->outputDebug("         Source from candidate point is first MIP solution point.");
            sourceDesc = "SOLPT-" + source;
            break;
        case E_PrimalNLPSource::FeasibleSolution:
            env->output->outputDebug("         Source from candidate point is MIP solution pool.");
            sourceDesc = "FEASP-" + source;
            break;
        case E_PrimalNLPSource::InfeasibleSolution:
            env->output->outputDebug("         Source from candidate point is infeasible MIP solution.");
            sourceDesc = "UNFEA-" + source;
            break;
        case E_PrimalNLPSource::SmallestDeviationSolution:
            env->output->outputDebug(
                "         Source from candidate point is MIP solution with smallest nonlinear error.");
            sourceDesc = "SMDEV-" + source;
            break;
        case E_PrimalNLPSource::FirstSolutionNewDualBound:
            env->output->outputDebug(
                "         Source from candidate point is first MIP solution point which gave dual bound update.");
            sourceDesc = "NEWDB-" + source;
            break;
        default:
            break;
        }

        switch(solvestatus)
        {
        case E_NLPSolutionStatus::Optimal:
            env->output->outputDebug(fmt::format(
                "         Optimal solution {} found to fixed NLP problem.", NLPSolver->getObjectiveValue()));
            break;

        case E_NLPSolutionStatus::Feasible:
            env->output->outputDebug(fmt::format(
                "         Feasible solution {} found to fixed NLP problem.", NLPSolver->getObjectiveValue()));
            break;

        case E_NLPSolutionStatus::Infeasible:
            env->output->outputDebug("         Fixed NLP problem is infeasible.");
            break;

        case E_NLPSolutionStatus::Unbounded:
            env->output->outputDebug("         Fixed NLP problem is unbounded.");
            break;

        case E_NLPSolutionStatus::TimeLimit:
            env->output->outputDebug("         Time limit hit when solving fixed NLP problem.");
            break;

        case E_NLPSolutionStatus::IterationLimit:
            env->output->outputDebug("         Iteration limit hit when solving fixed NLP problem.");
            break;

        case E_NLPSolutionStatus::Error:
            env->output->outputDebug("         Error ocurred when solving fixed NLP problem.");
            break;

        default:

            break;
        }

        if(solvestatus == E_NLPSolutionStatus::Feasible || solvestatus == E_NLPSolutionStatus::Optimal)
        {
            double tmpObj = NLPSolver->getObjectiveValue();
            auto variableSolution = NLPSolver->getSolution();

            env->primalSolver->addPrimalSolutionCandidate(
                variableSolution, E_PrimalSolutionSource::NLPFixedIntegers, currIter->iterationNumber);

            if(sourceProblem->properties.numberOfNonlinearConstraints > 0
                || sourceProblem->properties.numberOfQuadraticConstraints > 0)
            {
                auto mostDevConstr = sourceProblem->getMostDeviatingNonlinearOrQuadraticConstraint(variableSolution);

                env->output->outputDebug(fmt::format("         Max error {} from nonlinear or quadratic constraint {}.",
                    mostDevConstr->normalizedValue, mostDevConstr->constraint->name));

                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(),
                    tmpObj, mostDevConstr->constraint->index, mostDevConstr->normalizedValue,
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
                createIntegerCut(CAND.point);

            if(env->settings->getSetting<bool>("FixedInteger.CreateInfeasibilityCut", "Primal"))
                createInfeasibilityCut(variableSolution);
        }
        else if(solvestatus == E_NLPSolutionStatus::Error || solvestatus == E_NLPSolutionStatus::Unbounded
            || solvestatus == E_NLPSolutionStatus::Infeasible)
        {
            env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP, ("NLP" + sourceDesc),
                env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded, currIter->totNumHyperplanes,
                env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(), NAN, -1,
                NAN, E_IterationLineType::PrimalNLP);
        }
        else if(sourceProblem->properties.numberOfNonlinearConstraints > 0
            || sourceProblem->properties.numberOfQuadraticConstraints > 0)
        {
            double tmpObj = NLPSolver->getObjectiveValue();

            auto variableSolution = NLPSolver->getSolution();

            if(variableSolution.size() > 0)
            {
                auto mostDevConstr = sourceProblem->getMostDeviatingNonlinearOrQuadraticConstraint(variableSolution);

                if(env->settings->getSetting<bool>("FixedInteger.CreateInfeasibilityCut", "Primal"))
                    createInfeasibilityCut(variableSolution);

                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(),
                    tmpObj, mostDevConstr->constraint->index, mostDevConstr->normalizedValue,
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
        }
        else
        {

            auto variableSolution = NLPSolver->getSolution();

            if(variableSolution.size() > 0)
            {
                env->report->outputIterationDetail(env->solutionStatistics.numberOfProblemsFixedNLP,
                    ("NLP" + sourceDesc), env->timing->getElapsedTime("Total"), currIter->numHyperplanesAdded,
                    currIter->totNumHyperplanes, env->results->getCurrentDualBound(), env->results->getPrimalBound(),
                    env->results->getAbsoluteGlobalObjectiveGap(), env->results->getRelativeGlobalObjectiveGap(), NAN,
                    -1, NAN, E_IterationLineType::PrimalNLP);
            }
        }

        if(env->settings->getSetting<bool>("FixedInteger.Frequency.Dynamic", "Primal"))
        {
            if(solvestatus == E_NLPSolutionStatus::Optimal || solvestatus == E_NLPSolutionStatus::Feasible)
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

                env->output->outputDebug(fmt::format(
                    "         Iteration frequency updated to {} and time frequency updated to {} ", iters, interval));
            }
            else
            {
                int iters
                    = std::ceil(env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal") * 1.02);

                if(iters < 10 * this->originalIterFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Iteration", "Primal", iters);

                double interval = 1.1 * env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal");

                if(interval < 10 * this->originalTimeFrequency)
                    env->settings->updateSetting("FixedInteger.Frequency.Time", "Primal", interval);

                env->output->outputDebug(fmt::format(
                    "         Iteration frequency updated to {} and time frequency updated to {} ", iters, interval));
            }
        }

        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP = 0;
        env->solutionStatistics.timeLastFixedNLPCall = env->timing->getElapsedTime("Total");
        counter++;

        env->primalSolver->usedPrimalNLPCandidates.push_back(CAND);
    }

    return (true);
}

void TaskSelectPrimalCandidatesFromNLP::createInfeasibilityCut(const VectorDouble variableSolution)
{
    env->output->outputDebug("         Adding infeasibility cut from fixed NLP solution.");

    SolutionPoint tmpSolPt;
    tmpSolPt.point = variableSolution;
    tmpSolPt.objectiveValue = sourceProblem->objectiveFunction->calculateValue(variableSolution);
    tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;

    if(auto mostDevConstr = sourceProblem->getMostDeviatingNonlinearOrQuadraticConstraint(variableSolution);
        mostDevConstr)
        tmpSolPt.maxDeviation = PairIndexValue(mostDevConstr->constraint->index, mostDevConstr->normalizedValue);

    if(!sourceIsReformulatedProblem) // Need to calculate values for the auxiliary variables in this
                                     // case
    {
        if((int)tmpSolPt.point.size() < env->reformulatedProblem->properties.numberOfVariables)
            env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpSolPt.point);

        assert(tmpSolPt.point.size() == env->reformulatedProblem->properties.numberOfVariables);
    }

    std::vector<SolutionPoint> solutionPoints(1);
    solutionPoints.at(0) = tmpSolPt;

    if(!taskSelectHPPts)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanesESH>(env);
        else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ECP)
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanesECP>(env);
    }

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        std::dynamic_pointer_cast<TaskSelectHyperplanesESH>(taskSelectHPPts)->run(solutionPoints);
    }
    else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ECP)
    {
        std::dynamic_pointer_cast<TaskSelectHyperplanesECP>(taskSelectHPPts)->run(solutionPoints);
    }
}

void TaskSelectPrimalCandidatesFromNLP::createIntegerCut(VectorDouble variableSolution)
{
    bool withinBounds = true;

    assert(variableSolution.size() == env->reformulatedProblem->variableLowerBounds.size());
    assert(variableSolution.size() == env->reformulatedProblem->variableUpperBounds.size());

    // Verify that solution is within bounds: if the difference is small project to the bound, otherwise do not add
    // integer cut
    for(size_t i = 0; i < variableSolution.size(); i++)
    {
        if(variableSolution[i] < env->reformulatedProblem->variableLowerBounds[i])
        {
            if(variableSolution[i] > env->reformulatedProblem->variableLowerBounds[i] - 1e-8)
                variableSolution[i] = env->reformulatedProblem->variableLowerBounds[i];
            else
            {
                withinBounds = false;
                break;
            }
        }

        if(variableSolution[i] > env->reformulatedProblem->variableUpperBounds[i])
        {
            if(variableSolution[i] < env->reformulatedProblem->variableUpperBounds[i] + 1e-8)
                variableSolution[i] = env->reformulatedProblem->variableUpperBounds[i];
            else
            {
                withinBounds = false;
                break;
            }
        }
    }

    if(!withinBounds)
    {
        env->output->outputDebug("         Can not add integer cut since solution is not within variable bounds.");
        return;
    }

    IntegerCut integerCut;
    integerCut.source = E_IntegerCutSource::NLPFixedInteger;
    integerCut.variableValues.reserve(discreteVariableIndexes.size());

    integerCut.variableIndexes = discreteVariableIndexes;

    for(auto& I : discreteVariableIndexes)
        integerCut.variableValues.push_back(round(variableSolution.at(I)));

    env->dualSolver->addIntegerCut(integerCut);
}

} // namespace SHOT