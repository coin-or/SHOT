/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "DualSolver.h"

#include <boost/format.hpp>

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
    double currDualBound = env->results->getDualBound();
    double currPrimalBound = env->results->getPrimalBound();

    double gapRelTolerance = env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination");
    double gapAbsTolerance = env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination");

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
            env->solutionStatistics.iterationLastDualBoundUpdate = env->results->getCurrentIteration()->iterationNumber;
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

void DualSolver::addGeneratedHyperplane(const Hyperplane& hyperplane)
{
    std::string source = "";

    switch(hyperplane.source)
    {
    case E_HyperplaneSource::MIPOptimalLinesearch:
        source = "MIP linesearch";
        break;
    case E_HyperplaneSource::LPRelaxedLinesearch:
        source = "LP linesearch";
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
    case E_HyperplaneSource::ObjectiveLinesearch:
        source = "objective linesearch";
        break;
    default:
        break;
    }

    GeneratedHyperplane genHyperplane;

    genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
    genHyperplane.source = hyperplane.source;
    genHyperplane.iterationGenerated = env->results->getCurrentIteration()->iterationNumber;
    genHyperplane.isLazy = false;

    genHyperplane.pointHash = UtilityFunctions::calculateHash(hyperplane.generatedPoint);

    generatedHyperplanes.push_back(genHyperplane);

    env->output->outputTrace("     Hyperplane generated from: " + source);
}

bool DualSolver::hasHyperplaneBeenAdded(size_t hash, int constraintIndex)
{
    for(auto& H : generatedHyperplanes)
    {
        if(H.sourceConstraintIndex == constraintIndex && H.pointHash == hash)
            return (true);
    }

    return (false);
}

} // namespace SHOT