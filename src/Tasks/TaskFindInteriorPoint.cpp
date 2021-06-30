/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskFindInteriorPoint.h"

#include "../DualSolver.h"
#include "../PrimalSolver.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

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

    env->output->outputDebug(" Initializing NLP solver");

    if(env->dualSolver->interiorPointCandidates.size() > 0)
    {
        int i = 0;

        for(auto& PT : env->dualSolver->interiorPointCandidates)
        {
            auto tmpIP = std::make_shared<InteriorPoint>();

            tmpIP->point = PT->point;

            if((int)tmpIP->point.size() < env->reformulatedProblem->properties.numberOfVariables)
                env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpIP->point);

            assert(tmpIP->point.size() == env->reformulatedProblem->properties.numberOfVariables);

            auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
                tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
            tmpIP->maxDevatingConstraint = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

            if(maxDev.normalizedValue >= 0)
            {
                env->output->outputWarning(" Maximum deviation in interior point is too large: "
                    + Utilities::toString(maxDev.normalizedValue));

                if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
                {
                    std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                        + "/interiorpoint_provided_notused_" + std::to_string(i) + ".txt";
                    Utilities::saveVariablePointVectorToFile(tmpIP->point, variableNames, filename);
                }
            }
            else
            {
                env->output->outputInfo(" Valid interior point with constraint deviation "
                    + Utilities::toString(maxDev.normalizedValue) + " found.");

                env->dualSolver->interiorPts.push_back(tmpIP);

                if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
                {
                    std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                        + "/interiorpoint_provided" + std::to_string(i) + ".txt";
                    Utilities::saveVariablePointVectorToFile(tmpIP->point, variableNames, filename);
                }
            }

            i++;

            env->solutionStatistics.numberOfOriginalInteriorPoints++;
        }

        env->timing->stopTimer("InteriorPointSearch");
    }

    if(env->dualSolver->interiorPts.size() > 0)
        return;

    NLPSolvers.emplace_back(std::make_unique<NLPSolverCuttingPlaneMinimax>(env, env->reformulatedProblem));

    env->output->outputDebug(" Cutting plane minimax selected as NLP solver.");

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        for(size_t i = 0; i < NLPSolvers.size(); i++)
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

    for(size_t i = 0; i < NLPSolvers.size(); i++)
    {
        NLPSolvers.at(i)->solveProblem();

        if(NLPSolvers.at(i)->getSolution().size() == 0)
            continue;

        auto tmpIP = std::make_shared<InteriorPoint>();

        tmpIP->point = NLPSolvers.at(i)->getSolution();
        assert((int)tmpIP->point.size() == env->reformulatedProblem->properties.numberOfVariables);

        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
        tmpIP->maxDevatingConstraint = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

        if(maxDev.normalizedValue >= 0)
        {
            env->output->outputWarning("");
            env->output->outputWarning(
                " Maximum deviation in interior point is too large: " + Utilities::toString(maxDev.normalizedValue));

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                    + "/interiorpoint_notused_" + std::to_string(i) + ".txt";
                Utilities::saveVariablePointVectorToFile(tmpIP->point, variableNames, filename);
            }
        }
        else
        {
            env->output->outputInfo("");
            env->output->outputInfo(" Valid interior point with constraint deviation "
                + Utilities::toString(maxDev.normalizedValue) + " found.");

            env->dualSolver->interiorPts.push_back(tmpIP);

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                std::string filename = env->settings->getSetting<std::string>("Debug.Path", "Output")
                    + "/interiorpoint_" + std::to_string(i) + ".txt";
                Utilities::saveVariablePointVectorToFile(tmpIP->point, variableNames, filename);
            }
        }

        foundNLPPoint = (foundNLPPoint || (maxDev.normalizedValue <= 0));
    }

    if(!foundNLPPoint)
    {
        env->output->outputError("");
        env->output->outputError(" No interior point found!                            ");
        env->timing->stopTimer("InteriorPointSearch");

        return;
    }

    env->output->outputDebug(" Finished solving NLP problem.");

    env->solutionStatistics.numberOfOriginalInteriorPoints = env->dualSolver->interiorPts.size();

    for(auto IP : env->dualSolver->interiorPts)
    {
        env->primalSolver->addPrimalSolutionCandidate(IP->point, E_PrimalSolutionSource::InteriorPointSearch, 0);
    }

    env->timing->stopTimer("InteriorPointSearch");
}

std::string TaskFindInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT