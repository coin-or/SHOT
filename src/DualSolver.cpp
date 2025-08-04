/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "DualSolver.h"
#include "Output.h"
#include "Settings.h"
#include "Results.h"
#include "Iteration.h"
#include "Utilities.h"
#include "Timing.h"
#include "Problem.h"
#include "ObjectiveFunction.h"
#include "MIPSolver/IMIPSolver.h"

namespace SHOT
{

DualSolver::DualSolver(EnvironmentPtr envPtr) { env = envPtr; }

void DualSolver::addDualSolutionCandidate(DualSolution solution)
{
    dualSolutionCandidates.push_back(solution);

    this->checkDualSolutionCandidates();
}

void DualSolver::checkDualSolutionCandidates()
{
    double currDualBound = env->results->getCurrentDualBound();
    double currPrimalBound = env->results->getPrimalBound();

    double gapRelTolerance = env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination");

    for(auto& C : this->dualSolutionCandidates)
    {
        bool updateDual = false;

        if(env->problem->objectiveFunction->properties.isMinimize)
        {
            if(C.objValue < currPrimalBound * (1 + gapRelTolerance) && C.objValue > currPrimalBound)
            {
                C.objValue = currPrimalBound;
                updateDual = true;
            }
            else if(C.objValue > currDualBound && (C.objValue <= currPrimalBound))
            {
                updateDual = true;
            }
        }
        else
        {
            if(C.objValue > currPrimalBound * (1 + gapRelTolerance) && C.objValue < currPrimalBound)
            {
                C.objValue = currPrimalBound;
                updateDual = true;
            }
            else if(C.objValue < currDualBound && (C.objValue >= currPrimalBound))
            {
                updateDual = true;
            }
        }

        if(updateDual)
        {
            // New dual solution
            env->results->setDualBound(C.objValue);
            currDualBound = C.objValue;

            if(env->results->getNumberOfIterations() > 0)
                env->solutionStatistics.iterationLastDualBoundUpdate
                    = env->results->getCurrentIteration()->iterationNumber;
            else
                env->solutionStatistics.iterationLastDualBoundUpdate = 0;

            env->solutionStatistics.iterationLastDualBoundUpdate = env->timing->getElapsedTime("Total");

            if(C.sourceType == E_DualSolutionSource::MIPSolutionOptimal
                || C.sourceType == E_DualSolutionSource::LPSolution
                || C.sourceType == E_DualSolutionSource::MIPSolverBound)
            {
                env->results->addDualSolution(C);
            }

            std::string sourceDesc;

            switch(C.sourceType)
            {
            case E_DualSolutionSource::LPSolution:
                sourceDesc = "LP solution";
                break;
            case E_DualSolutionSource::MIPSolutionOptimal:
                sourceDesc = "MIP solution";
                break;
            case E_DualSolutionSource::ObjectiveConstraint:
                sourceDesc = "Obj. constr. rootsearch";
                break;
            case E_DualSolutionSource::MIPSolverBound:
                sourceDesc = "MIP solver bound";
                break;
            default:
                break;
            }

            env->output->outputDebug(fmt::format("        New dual bound {}, source: {}", C.objValue, sourceDesc));
        }
    }

    this->dualSolutionCandidates.clear();
}

void DualSolver::addHyperplane(HyperplanePtr hyperplane)
{
    if(auto objectiveHP = std::dynamic_pointer_cast<ObjectiveHyperplane>(hyperplane))
    {
        assert((int)objectiveHP->generatedPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

        objectiveHP->pointHash = Utilities::calculateHash(objectiveHP->generatedPoint);

        if(!hasHyperplaneBeenAdded(objectiveHP->pointHash, -1))
        {
            this->hyperplaneWaitingList.push_back(hyperplane);
        }
        else
        {
            env->output->outputDebug(fmt::format(
                "        Objective hyperplane with hash {} has been added already.", objectiveHP->pointHash));
        }
    }
    else if(auto constraintHP = std::dynamic_pointer_cast<ConstraintHyperplane>(hyperplane))
    {
        assert((int)constraintHP->generatedPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

        constraintHP->pointHash = Utilities::calculateHash(constraintHP->generatedPoint);

        if(!hasHyperplaneBeenAdded(constraintHP->pointHash, constraintHP->sourceConstraint->index))
        {
            this->hyperplaneWaitingList.push_back(hyperplane);
        }
        else
        {
            env->output->outputDebug(
                fmt::format("        Hyperplane with hash {} has been added already.", constraintHP->pointHash));
        }
    }
    else if(auto externalHP = std::dynamic_pointer_cast<ExternalHyperplane>(hyperplane))
    {
        // TODO check for already added hyperplanes
        this->hyperplaneWaitingList.push_back(externalHP);
    }
}

void DualSolver::addGeneratedHyperplane(const HyperplanePtr hyperplane)
{
    std::string source = "";

    switch(hyperplane->source)
    {
    case E_HyperplaneSource::MIPOptimalRootsearch:
        source = "MIP rootsearch";
        break;
    case E_HyperplaneSource::LPRelaxedRootsearch:
        source = "LP rootsearch";
        break;
    case E_HyperplaneSource::MIPOptimalSolutionPoint:
        source = "MIP optimal solution";
        break;
    case E_HyperplaneSource::MIPSolutionPoolSolutionPoint:
        source = "MIP solution pool";
        break;
    case E_HyperplaneSource::LPRelaxedSolutionPoint:
        source = "LP solution";
        break;
    case E_HyperplaneSource::LPFixedIntegers:
        source = "LP fixed integer";
        break;
    case E_HyperplaneSource::PrimalSolutionSearch:
        source = "primal heuristic";
        break;
    case E_HyperplaneSource::PrimalSolutionSearchInteriorObjective:
        source = "primal heuristic (interior objective)";
        break;
    case E_HyperplaneSource::InteriorPointSearch:
        source = "interior point search";
        break;
    case E_HyperplaneSource::MIPCallbackRelaxed:
        source = "MIP callback relaxed";
        break;
    case E_HyperplaneSource::ObjectiveRootsearch:
        source = "objective rootsearch";
        break;
    case E_HyperplaneSource::ObjectiveCuttingPlane:
        source = "objective cutting plane";
        break;
    case E_HyperplaneSource::External:
        source = "external";
        break;
    default:
        break;
    }

    auto genHyperplane = std::make_shared<GeneratedHyperplane>();
    genHyperplane->sourceHyperplane = hyperplane;
    genHyperplane->iterationGenerated = env->results->getCurrentIteration()->iterationNumber;
    genHyperplane->isLazy = false;

    if(auto numericHP = std::dynamic_pointer_cast<NumericHyperplane>(genHyperplane->sourceHyperplane))
    {
        if(!env->settings->getSetting<bool>("HyperplaneCuts.SaveHyperplanePoints", "Dual"))
            numericHP->generatedPoint = VectorDouble();
    }

    if(!genHyperplane->sourceHyperplane->isGlobal)
    {
        if(env->results->solutionIsGlobal)
            env->output->outputDebug("        Solution is no longer global since hyperplane has been added to "
                                     "nonconvex objective or constraint.");

        env->results->solutionIsGlobal = false;
    }

    generatedHyperplanes.push_back(genHyperplane);

    auto currentIteration = env->results->getCurrentIteration();
    currentIteration->numHyperplanesAdded++;
    currentIteration->totNumHyperplanes++;
    env->solutionStatistics.iterationLastDualCutAdded = currentIteration->iterationNumber;

    env->output->outputTrace("        Hyperplane generated from: " + source);
}

bool DualSolver::hasHyperplaneBeenAdded(double hash, int constraintIndex)
{
    // Cuts added as lazy might not actually always be added (e.g. in different threads), thus we have to allow them
    // to be added again
    if(env->settings->getSetting<int>("TreeStrategy", "Dual") == static_cast<int>(ES_TreeStrategy::SingleTree))
        return false;

    for(auto& H : generatedHyperplanes)
    {
        if(auto objectiveHP = std::dynamic_pointer_cast<ObjectiveHyperplane>(H))
        {
            if(constraintIndex == -1 && Utilities::isAlmostEqual(objectiveHP->pointHash, hash, 1e-8))
            {
                return (true);
            }
        }
        else if(auto constraintHP = std::dynamic_pointer_cast<ConstraintHyperplane>(H))
        {
            if(constraintHP->sourceConstraint->index == constraintIndex
                && Utilities::isAlmostEqual(constraintHP->pointHash, hash, 1e-8))
            {
                return (true);
            }
        }
    }

    return (false);
}

void DualSolver::addIntegerCut(IntegerCut integerCut)
{
    if(env->reformulatedProblem->properties.numberOfIntegerVariables > 0
        || env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0)
    {
        integerCut.areAllVariablesBinary = false;
    }
    else
    {
        integerCut.areAllVariablesBinary = true;
    }

    integerCut.pointHash = Utilities::calculateHash(integerCut.variableValues);

    if(!hasIntegerCutBeenAdded(integerCut.pointHash))
        this->integerCutWaitingList.push_back(integerCut);
    else
        env->output->outputDebug(
            fmt::format("        Integer cut with hash {} has been added already.", integerCut.pointHash));
}

void DualSolver::addGeneratedIntegerCut(IntegerCut integerCut)
{
    std::string source = "";

    switch(integerCut.source)
    {
    case E_IntegerCutSource::NLPFixedInteger:
        source = "NLP fixed integer";
        break;

    default:
        break;
    }

    integerCut.iterationGenerated = env->results->getCurrentIteration()->iterationNumber;

    if(env->results->solutionIsGlobal && env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        env->results->solutionIsGlobal = false;
        env->output->outputInfo("        Solution is no longer global since integer cut has been added.");
    }

    env->output->outputDebug(fmt::format("        Added integer cut with hash {}", integerCut.pointHash));

    generatedIntegerCuts.push_back(integerCut);

    auto currentIteration = env->results->getCurrentIteration();
    currentIteration->numHyperplanesAdded++;
    currentIteration->totNumHyperplanes++;

    env->solutionStatistics.numberOfIntegerCuts++;

    env->output->outputDebug("        Integer cut generated from: " + source);
}

bool DualSolver::hasIntegerCutBeenAdded(double hash)
{
    for(auto& IC : generatedIntegerCuts)
    {
        if(Utilities::isAlmostEqual(IC.pointHash, hash, 1e-8))
        {
            return (true);
        }
    }

    return (false);
}

} // namespace SHOT