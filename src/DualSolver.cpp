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

void DualSolver::addHyperplane(Hyperplane& hyperplane)
{
    assert((int)hyperplane.generatedPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

    hyperplane.pointHash = Utilities::calculateHash(hyperplane.generatedPoint);

    if(((hyperplane.source == E_HyperplaneSource::ObjectiveRootsearch
            || hyperplane.source == E_HyperplaneSource::ObjectiveCuttingPlane)
           && !hasHyperplaneBeenAdded(hyperplane.pointHash, -1))
        || (hyperplane.source != E_HyperplaneSource::ObjectiveRootsearch
            && hyperplane.source != E_HyperplaneSource::ObjectiveCuttingPlane
            && (!hasHyperplaneBeenAdded(hyperplane.pointHash, hyperplane.sourceConstraint->index))))
    {
        this->hyperplaneWaitingList.push_back(hyperplane);
    }
    else
    {
        env->output->outputDebug(
            fmt::format("        Hyperplane with hash {} has been added already.", hyperplane.pointHash));
    }
}

void DualSolver::addGeneratedHyperplane(const Hyperplane& hyperplane)
{
    std::string source = "";

    switch(hyperplane.source)
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
    default:
        break;
    }

    GeneratedHyperplane genHyperplane;

    genHyperplane.source = hyperplane.source;
    if(hyperplane.sourceConstraint)
    {
        genHyperplane.sourceConstraint = hyperplane.sourceConstraint;
        genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraint->index;
    }
    else
        genHyperplane.sourceConstraintIndex = -1;

    genHyperplane.iterationGenerated = env->results->getCurrentIteration()->iterationNumber;
    genHyperplane.isLazy = false;
    genHyperplane.pointHash = hyperplane.pointHash;

    if(env->settings->getSetting<bool>("HyperplaneCuts.SaveHyperplanePoints", "Dual"))
        genHyperplane.generatedPoint = hyperplane.generatedPoint;

    genHyperplane.isSourceConvex = hyperplane.isSourceConvex;

    if(!genHyperplane.isSourceConvex)
    {
        if(env->results->solutionIsGlobal)
            env->output->outputDebug("        Solution is no longer global since hyperplane has been added to "
                                     "nonconvex objective or constraint.");

        env->results->solutionIsGlobal = false;
    }

    if(hasHyperplaneBeenAdded(genHyperplane.pointHash, genHyperplane.sourceConstraintIndex))
    {
        env->output->outputTrace(fmt::format("        Not added hyperplane with hash {} to constraint {}",
            genHyperplane.pointHash, genHyperplane.sourceConstraintIndex));
        return;
    }

    if(hyperplane.sourceConstraint)
    {
        env->output->outputTrace(fmt::format("        Added hyperplane with hash {} to constraint {}",
            genHyperplane.pointHash, genHyperplane.sourceConstraint->index));
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
    // Cuts added as lazy might not actually always be added (e.g. in different threads), thus we have to allow them to
    // be added again
    if(env->settings->getSetting<int>("TreeStrategy", "Dual") == static_cast<int>(ES_TreeStrategy::SingleTree))
        return false;

    for(auto& H : generatedHyperplanes)
    {
        if((H.source == E_HyperplaneSource::ObjectiveRootsearch
               || H.source == E_HyperplaneSource::ObjectiveCuttingPlane)
            && constraintIndex == -1 && Utilities::isAlmostEqual(H.pointHash, hash, 1e-8))
        {
            return (true);
        }
        else if(H.source != E_HyperplaneSource::ObjectiveRootsearch
            && H.source != E_HyperplaneSource::ObjectiveCuttingPlane && H.sourceConstraint->index == constraintIndex
            && Utilities::isAlmostEqual(H.pointHash, hash, 1e-8))
        {
            return (true);
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