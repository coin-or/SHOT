/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskFindInteriorPoint.h"

#include "../DualSolver.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../MIPSolver/IMIPSolver.h"

#include "../NLPSolver/NLPSolverCuttingPlaneMinimax.h"

namespace SHOT
{

TaskFindInteriorPoint::TaskFindInteriorPoint(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        for(auto& V : env->reformulatedProblem->allVariables)
        {
            variableNames.push_back(V->name);
        }
    }
}

TaskFindInteriorPoint::~TaskFindInteriorPoint() { NLPSolvers.clear(); }

void TaskFindInteriorPoint::run()
{
    env->timing->startTimer("InteriorPointSearch");

    env->report->outputInteriorPointPreReport();

    env->output->outputDebug("Initializing NLP solver");

    auto solver
        = static_cast<ES_InteriorPointStrategy>(env->settings->getSetting<int>("ESH.InteriorPoint.Solver", "Dual"));

    if(solver == ES_InteriorPointStrategy::CuttingPlaneMiniMax)
    {
        NLPSolvers.emplace_back(std::make_unique<NLPSolverCuttingPlaneMinimax>(env, env->reformulatedProblem));

        env->output->outputDebug("Cutting plane minimax selected as NLP solver.");
    }
    else
    {
        return;
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        for(int i = 0; i < NLPSolvers.size(); i++)
        {
            std::stringstream ss;
            ss << env->settings->getSetting<std::string>("Debug.Path", "Output");
            ss << "/interiorpointnlp";
            ss << i;
            ss << ".txt";

            NLPSolvers.at(i)->saveProblemToFile(ss.str());
        }
    }

    env->output->outputDebug(" Solving NLP problem.");

    bool foundNLPPoint = false;

    for(int i = 0; i < NLPSolvers.size(); i++)
    {
        auto solutionStatus = NLPSolvers.at(i)->solveProblem();

        auto tmpIP = std::make_shared<InteriorPoint>();

        tmpIP->NLPSolver
            = static_cast<ES_InteriorPointStrategy>(env->settings->getSetting<int>("ESH.InteriorPoint.Solver", "Dual"));

        tmpIP->point = NLPSolvers.at(i)->getSolution();

        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
        tmpIP->maxDevatingConstraint = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

        if(maxDev.normalizedValue >= 0)
        {
            env->output->outputWarning("\n        Maximum deviation in interior point is too large: "
                + Utilities::toString(maxDev.normalizedValue));

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                    + "/interiorpoint_notused_" + std::to_string(i) + ".txt";
                Utilities::saveVariablePointVectorToFile(tmpIP->point, variableNames, filename);
            }
        }
        else
        {
            env->output->outputInfo("\n        Valid interior point with constraint deviation "
                + Utilities::toString(maxDev.normalizedValue) + " found.");

            env->dualSolver->MIPSolver->interiorPts.push_back(tmpIP);

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                    + "/interiorpoint_" + std::to_string(i) + ".txt";
                Utilities::saveVariablePointVectorToFile(tmpIP->point, variableNames, filename);
            }
        }

        foundNLPPoint = (foundNLPPoint || (maxDev.normalizedValue <= 0));

        if(tmpIP->NLPSolver == ES_InteriorPointStrategy::IpoptMinimax
            || tmpIP->NLPSolver == ES_InteriorPointStrategy::IpoptRelaxed
            || tmpIP->NLPSolver == ES_InteriorPointStrategy::IpoptMinimaxAndRelaxed)
        {
            env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch++;
        }
    }

    if(!foundNLPPoint)
    {
        env->output->outputError("\n        No interior point found!                            ");
        env->timing->stopTimer("InteriorPointSearch");

        return;
    }

    env->output->outputDebug("     Finished solving NLP problem.");

    env->solutionStatistics.numberOfOriginalInteriorPoints = env->dualSolver->MIPSolver->interiorPts.size();

    env->timing->stopTimer("InteriorPointSearch");
}

std::string TaskFindInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT