/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "DualSolver.h"

namespace SHOT
{

void DualSolver::addDualSolutionCandidate(DualSolution solution)
{
    dualSolutionCandidates.push_back(solution);

    this->checkDualSolutionCandidates();
}

void DualSolver::checkDualSolutionCandidates()
{
    double currDualBound = env->results->getDualBound();
    double currPrimalBound = env->results->getPrimalBound();

    double gapRelTolerance = env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination");
    double gapAbsTolerance = env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination");

    for(auto C : this->dualSolutionCandidates)
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
} // namespace SHOT