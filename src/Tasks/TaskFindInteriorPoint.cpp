/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskFindInteriorPoint.h"

#include "../CallbackData.h"
#include "../DualSolver.h"
#include "../EventHandler.h"
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
    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
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

                if(env->settings->getSetting<bool>("Output.Debug.Enable"))
                {
                    std::string filename = env->settings->getSetting<std::string>("Output.Debug.Path")
                        + "/interiorpoint_provided_notused_" + std::to_string(i) + ".txt";
                    Utilities::saveVariablePointVectorToFile(tmpIP->point, variableNames, filename);
                }
            }
            else
            {
                env->output->outputInfo(" Valid interior point with constraint deviation "
                    + Utilities::toString(maxDev.normalizedValue) + " found.");

                env->dualSolver->interiorPts.push_back(tmpIP);

                if(env->settings->getSetting<bool>("Output.Debug.Enable"))
                {
                    std::string filename = env->settings->getSetting<std::string>("Output.Debug.Path")
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
    {
        env->timing->stopTimer("InteriorPointSearch");
        return;
    }

    if(static_cast<ES_ESHInteriorPointStrategy>(
           env->settings->getSetting<int>("Dual.ESH.InteriorPoint.Strategy"))
        == ES_ESHInteriorPointStrategy::OnlyExternal)
    {
        if(!env->events->hasDataProvider(E_EventType::ExternalESHRootsearchPointsSelection))
        {
            env->output->outputWarning(
                " ESH.InteriorPoint.Strategy is OnlyExternal but no callback is registered. "
                "No interior point will be available.");
            env->timing->stopTimer("InteriorPointSearch");
            return;
        }

        // Fire the callback with an empty current set
        ESHInteriorPointCallbackData callbackData(
            {}, env->problem, env->reformulatedProblem, env->solutionStatistics);

        auto callbackResult = env->events->requestData<std::vector<VectorDouble>>(
            E_EventType::ExternalESHRootsearchPointsSelection, callbackData);

        if(!callbackResult.has_value() || callbackResult->empty())
        {
            env->output->outputWarning(
                " ESH interior point callback returned no points. No interior point available.");
            env->timing->stopTimer("InteriorPointSearch");
            return;
        }

        int i = 0;
        for(auto& pt : *callbackResult)
        {
            auto tmpIP = std::make_shared<InteriorPoint>();
            tmpIP->point = pt;

            if((int)tmpIP->point.size() < env->reformulatedProblem->properties.numberOfVariables)
                env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpIP->point);

            assert(
                tmpIP->point.size() == (size_t)env->reformulatedProblem->properties.numberOfVariables);

            auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
                tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
            tmpIP->maxDevatingConstraint
                = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

            if(maxDev.normalizedValue >= 0)
            {
                env->output->outputWarning(
                    " Callback-provided interior point " + std::to_string(i)
                    + " has constraint deviation too large: "
                    + Utilities::toString(maxDev.normalizedValue) + " (discarded)");
            }
            else
            {
                env->output->outputInfo(" Valid callback-provided interior point " + std::to_string(i)
                    + " with constraint deviation " + Utilities::toString(maxDev.normalizedValue)
                    + " accepted.");
                env->dualSolver->interiorPts.push_back(tmpIP);
            }
            i++;
        }

        env->solutionStatistics.numberOfOriginalInteriorPoints = env->dualSolver->interiorPts.size();

        for(auto& IP : env->dualSolver->interiorPts)
            env->primalSolver->addPrimalSolutionCandidate(IP->point, E_PrimalSolutionSource::InteriorPointSearch, 0);

        env->timing->stopTimer("InteriorPointSearch");
        return;
    }

    NLPSolvers.emplace_back(std::make_unique<NLPSolverCuttingPlaneMinimax>(env, env->reformulatedProblem));

    env->output->outputDebug(" Cutting plane minimax selected as NLP solver.");

    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        for(size_t i = 0; i < NLPSolvers.size(); i++)
        {
            std::stringstream ss;
            ss << env->settings->getSetting<std::string>("Output.Debug.Path");
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

            if(env->settings->getSetting<bool>("Output.Debug.Enable"))
            {
                std::string filename = env->settings->getSetting<std::string>("Output.Debug.Path")
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

            if(env->settings->getSetting<bool>("Output.Debug.Enable"))
            {
                std::string filename = env->settings->getSetting<std::string>("Output.Debug.Path")
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

    // Fire ESH interior point callback after internal NLP search, allowing user to inspect,
    // filter, or augment the found interior points
    if(env->events->hasDataProvider(E_EventType::ExternalESHRootsearchPointsSelection))
    {
        std::vector<VectorDouble> currentPoints;
        for(auto& IP : env->dualSolver->interiorPts)
            currentPoints.push_back(IP->point);

        ESHInteriorPointCallbackData callbackData(
            currentPoints, env->problem, env->reformulatedProblem, env->solutionStatistics);

        auto callbackResult = env->events->requestData<std::vector<VectorDouble>>(
            E_EventType::ExternalESHRootsearchPointsSelection, callbackData);

        if(callbackResult.has_value() && !callbackResult->empty())
        {
            env->output->outputInfo(" ESH interior point callback returned replacement points.");
            env->dualSolver->interiorPts.clear();

            int i = 0;
            for(auto& pt : *callbackResult)
            {
                auto tmpIP = std::make_shared<InteriorPoint>();
                tmpIP->point = pt;

                if((int)tmpIP->point.size() < env->reformulatedProblem->properties.numberOfVariables)
                    env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpIP->point);

                assert(tmpIP->point.size() == (size_t)env->reformulatedProblem->properties.numberOfVariables);

                auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
                    tmpIP->point, env->reformulatedProblem->nonlinearConstraints);
                tmpIP->maxDevatingConstraint
                    = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);

                if(maxDev.normalizedValue >= 0)
                {
                    env->output->outputWarning(" Callback-provided interior point " + std::to_string(i)
                        + " has constraint deviation too large: " + Utilities::toString(maxDev.normalizedValue)
                        + " (discarded)");
                }
                else
                {
                    env->output->outputInfo(" Valid callback-provided interior point " + std::to_string(i)
                        + " with constraint deviation " + Utilities::toString(maxDev.normalizedValue) + " accepted.");
                    env->dualSolver->interiorPts.push_back(tmpIP);
                }
                i++;
            }

            env->solutionStatistics.numberOfOriginalInteriorPoints = env->dualSolver->interiorPts.size();
        }
    }

    env->timing->stopTimer("InteriorPointSearch");
}

std::string TaskFindInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT