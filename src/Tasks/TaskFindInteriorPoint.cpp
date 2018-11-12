/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskFindInteriorPoint.h"

TaskFindInteriorPoint::TaskFindInteriorPoint(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskFindInteriorPoint::~TaskFindInteriorPoint()
{
    NLPSolvers.clear();
}

void TaskFindInteriorPoint::run()
{
    env->process->startTimer("InteriorPointSearch");

    env->report->outputInteriorPointPreReport();

    env->output->outputDebug("Initializing NLP solver");

    auto solver = static_cast<ES_InteriorPointStrategy>(env->settings->getIntSetting("ESH.InteriorPoint.Solver", "Dual"));

    if (solver == ES_InteriorPointStrategy::CuttingPlaneMiniMax)
    {
        NLPSolvers.emplace_back(new NLPSolverCuttingPlaneMinimax(env));

        NLPSolvers[0]->setProblem(env->model->originalProblem->getProblemInstance());

        env->output->outputDebug("Cutting plane minimax selected as NLP solver.");
    }
    else if (solver == ES_InteriorPointStrategy::IpoptMinimax)
    {
        NLPSolvers.emplace_back(new NLPSolverIpoptMinimax(env));

        NLPSolvers[0]->setProblem(env->model->originalProblem->getProblemInstance());

        env->output->outputDebug("Ipopt minimax selected as NLP solver.");
    }
    else if (solver == ES_InteriorPointStrategy::IpoptRelaxed)
    {
        NLPSolvers.emplace_back(new NLPSolverIpoptRelaxed(env));

        NLPSolvers[0]->setProblem(env->model->originalProblem->getProblemInstance());

        env->output->outputDebug("Ipopt relaxed selected as NLP solver.");
    }
    else if (solver == ES_InteriorPointStrategy::IpoptMinimaxAndRelaxed)
    {
        NLPSolvers.emplace_back(new NLPSolverIpoptMinimax(env));

        NLPSolvers[0]->setProblem(env->model->originalProblem->getProblemInstance());

        NLPSolvers.emplace_back(new NLPSolverIpoptRelaxed(env));

        NLPSolvers[1]->setProblem(env->model->originalProblem->getProblemInstance());

        env->output->outputDebug("Ipopt minimax and relaxed selected as NLP solver.");
    }
    else
    {
        return;
    }

    if (env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        for (int i = 0; i < NLPSolvers.size(); i++)
        {
            std::stringstream ss;
            ss << env->settings->getStringSetting("Debug.Path", "Output");
            ss << "/interiorpointnlp";
            ss << i;
            ss << ".txt";

            NLPSolvers.at(i)->saveProblemToFile(ss.str());
        }
    }

    env->output->outputDebug(" Solving NLP problem.");

    bool foundNLPPoint = false;

    for (int i = 0; i < NLPSolvers.size(); i++)
    {
        auto solutionStatus = NLPSolvers.at(i)->solveProblem();

        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

        tmpIP->NLPSolver = static_cast<ES_InteriorPointStrategy>(env->settings->getIntSetting("ESH.InteriorPoint.Solver", "Dual"));

        tmpIP->point = NLPSolvers.at(i)->getSolution();

        if (solver == ES_InteriorPointStrategy::IpoptRelaxed && tmpIP->point.size() < env->model->originalProblem->getNumberOfVariables())
        {
            tmpIP->point.push_back(
                env->model->originalProblem->calculateOriginalObjectiveValue(tmpIP->point));
        }

        while (tmpIP->point.size() > env->model->originalProblem->getNumberOfVariables())
        {
            tmpIP->point.pop_back();
        }

        auto maxDev = env->model->originalProblem->getMostDeviatingConstraint(tmpIP->point);
        tmpIP->maxDevatingConstraint = maxDev;

        if (maxDev.value > 0)
        {
            env->output->outputWarning("\n Maximum deviation in interior point is too large: " + UtilityFunctions::toString(maxDev.value));
        }
        else
        {
            env->output->outputSummary("\n Valid interior point with constraint deviation " + UtilityFunctions::toString(maxDev.value) + " found.");
            env->process->interiorPts.push_back(tmpIP);
        }

        foundNLPPoint = (foundNLPPoint || (maxDev.value <= 0));

        if (env->settings->getBoolSetting("Debug.Enable", "Output"))
        {
            auto tmpVars = env->model->originalProblem->getVariableNames();
            std::string filename = env->settings->getStringSetting("Debug.Path", "Output") + "/interiorpoint_" + std::to_string(i) + ".txt";
            UtilityFunctions::saveVariablePointVectorToFile(tmpIP->point, env->problem->allVariables, filename);
        }

        if (tmpIP->NLPSolver == ES_InteriorPointStrategy::IpoptMinimax || tmpIP->NLPSolver == ES_InteriorPointStrategy::IpoptRelaxed || tmpIP->NLPSolver == ES_InteriorPointStrategy::IpoptMinimaxAndRelaxed)
        {
            env->solutionStatistics.numberOfProblemsNLPInteriorPointSearch++;
        }
    }

    if (!foundNLPPoint)
    {
        env->output->outputError("\n No interior point found!                            ");
        env->process->stopTimer("InteriorPointSearch");

        return;
    }

    env->output->outputDebug("     Finished solving NLP problem.");

    env->solutionStatistics.numberOfOriginalInteriorPoints = env->process->interiorPts.size();

    env->process->stopTimer("InteriorPointSearch");
}

std::string TaskFindInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
