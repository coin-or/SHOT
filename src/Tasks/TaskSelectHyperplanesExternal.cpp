/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/
#include "TaskSelectHyperplanesExternal.h"

#include "../DualSolver.h"
#include "../EventHandler.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include <any>

namespace SHOT
{

TaskSelectHyperplanesExternal::TaskSelectHyperplanesExternal(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("CallbackExternalHyperplaneGeneration");
    env->timing->stopTimer("CallbackExternalHyperplaneGeneration");
}

TaskSelectHyperplanesExternal::~TaskSelectHyperplanesExternal() = default;

void TaskSelectHyperplanesExternal::run() { this->run(env->results->getPreviousIteration()->solutionPoints); }

void TaskSelectHyperplanesExternal::run(std::vector<SolutionPoint> solutionPoints)
{
    env->timing->startTimer("CallbackExternalHyperplaneGeneration");

    env->output->outputDebug("        Selecting cutting planes using external callback functionality:");

    env->events->notify(E_EventType::ExternalHyperplaneSelection, solutionPoints);

    /*
    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints == 0)
        return;

    env->output->outputDebug("        Selecting cutting planes using the ECP method:");

    env->timing->startTimer("DualCutGenerationRootSearch");

    int addedHyperplanes = 0;
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    auto constraintSelectionFactor
        = env->settings->getSetting<double>("HyperplaneCuts.ConstraintSelectionFactor", "Dual");
    bool useUniqueConstraints = env->settings->getSetting<bool>("ESH.Rootsearch.UniqueConstraints", "Dual");

    int maxHyperplanesPerIter = env->settings->getSetting<int>("HyperplaneCuts.MaxPerIteration", "Dual");
    double constraintMaxSelectionFactor
        = env->settings->getSetting<double>("HyperplaneCuts.MaxConstraintFactor", "Dual");

    // Contains boolean array that indicates if a constraint has been added or not
    std::vector<bool> hyperplaneAddedToConstraint(
        env->reformulatedProblem->properties.numberOfNumericConstraints, false);

    std::vector<std::tuple<int, NumericConstraintValue>> selectedNumericValues;
    std::vector<std::tuple<int, NumericConstraintValue>> nonconvexSelectedNumericValues;

    for(size_t i = 0; i < solPoints.size(); i++)
    {
        auto numericConstraintValues = env->reformulatedProblem->getFractionOfDeviatingNonlinearConstraints(
            solPoints.at(i).point, 0.0, constraintSelectionFactor);

        for(auto& NCV : numericConstraintValues)
        {
            if(addedHyperplanes >= maxHyperplanesPerIter)
            {
                env->timing->stopTimer("DualCutGenerationRootSearch");
                break;
            }

            // Do not add hyperplane if one has been added for this constraint already
            if(useUniqueConstraints && hyperplaneAddedToConstraint.at(NCV.constraint->index))
            {
                continue;
            }

            // Do not add hyperplane if there are numerical errors
            if(std::isnan(NCV.error) || std::isnan(NCV.normalizedValue))
            {
                continue;
            }

            // Do not add hyperplane if constraint value is much less than largest
            if(NCV.error < constraintMaxSelectionFactor * numericConstraintValues.at(0).error)
            {
                continue;
            }

            double hash = Utilities::calculateHash(solPoints.at(i).point);

            if(env->dualSolver->hasHyperplaneBeenAdded(hash, NCV.constraint->index))
            {
                env->output->outputDebug("         Hyperplane already added for constraint "
                    + std::to_string(NCV.constraint->index) + " and hash " + std::to_string(hash));
                continue;
            }

            if(NCV.constraint->properties.convexity != E_Convexity::Convex)
            {
                nonconvexSelectedNumericValues.emplace_back(i, NCV);
                continue;
            }

            selectedNumericValues.emplace_back(i, NCV);
        }
    }

    for(auto& values : selectedNumericValues)
    {
        int i = std::get<0>(values);
        auto NCV = std::get<1>(values);

        auto hyperplane = std::make_shared<ConstraintHyperplane>();
        hyperplane->sourceConstraint = NCV.constraint;
        hyperplane->generatedPoint = solPoints.at(i).point;
        hyperplane->isGlobal = (NCV.constraint->properties.convexity <= E_Convexity::Convex);

        if(solPoints.at(i).isRelaxedPoint)
        {
            hyperplane->source = E_HyperplaneSource::MIPCallbackRelaxed;
        }
        else if(i == 0 && currIter->isMIP())
        {
            hyperplane->source = E_HyperplaneSource::MIPOptimalSolutionPoint;
        }
        else if(currIter->isMIP())
        {
            hyperplane->source = E_HyperplaneSource::MIPSolutionPoolSolutionPoint;
        }
        else
        {
            hyperplane->source = E_HyperplaneSource::LPRelaxedSolutionPoint;
        }

        env->dualSolver->addHyperplane(hyperplane);

        addedHyperplanes++;
        hyperplaneAddedToConstraint.at(NCV.constraint->index) = true;

        env->output->outputDebug(
            fmt::format("         Added hyperplane for constraint {} to waiting list with deviation {}",
                NCV.constraint->name, NCV.error));
    }

    std::vector<std::pair<ConstraintHyperplanePtr, double>> hyperplanesCuttingAwayPrimals;

    if(addedHyperplanes == 0)
    {
        env->output->outputDebug("         Could not add hyperplane for convex constraints, number of nonconvex: "
            + std::to_string(nonconvexSelectedNumericValues.size()));

        for(auto& values : nonconvexSelectedNumericValues)
        {
            if(addedHyperplanes > maxHyperplanesPerIter)
                break;

            int i = std::get<0>(values);
            auto NCV = std::get<1>(values);

            auto hyperplane = std::make_shared<ConstraintHyperplane>();
            hyperplane->sourceConstraint = NCV.constraint;
            hyperplane->generatedPoint = solPoints.at(i).point;
            hyperplane->isGlobal = (NCV.constraint->properties.convexity <= E_Convexity::Convex);

            if(solPoints.at(i).isRelaxedPoint)
            {
                hyperplane->source = E_HyperplaneSource::MIPCallbackRelaxed;
            }
            else if(i == 0 && currIter->isMIP())
            {
                hyperplane->source = E_HyperplaneSource::MIPOptimalSolutionPoint;
            }
            else if(currIter->isMIP())
            {
                hyperplane->source = E_HyperplaneSource::MIPSolutionPoolSolutionPoint;
            }
            else
            {
                hyperplane->source = E_HyperplaneSource::LPRelaxedSolutionPoint;
            }

            bool cutsAwayPrimalSolution = false;

            for(auto& P : env->results->primalSolutions)
            {
                if(auto terms = env->dualSolver->MIPSolver->createHyperplaneTerms(hyperplane))
                {
                    double constraintValue = terms->second;

                    for(auto& T : terms->first)
                    {
                        constraintValue += T.second * P.point[T.first];
                    }

                    if(constraintValue > 0)
                    {
                        cutsAwayPrimalSolution = true;
                        hyperplanesCuttingAwayPrimals.emplace_back(hyperplane, constraintValue);
                        break;
                    }
                }
            }

            if(!cutsAwayPrimalSolution)
            {
                env->output->outputDebug(
                    fmt::format("         Added hyperplane for constraint {} to waiting list with deviation {}",
                        NCV.constraint->name, NCV.error));

                env->dualSolver->addHyperplane(hyperplane);
                hyperplaneAddedToConstraint.at(NCV.constraint->index) = true;
                addedHyperplanes++;
            }
        }
    }

    if(addedHyperplanes == 0)
    {
        std::sort(hyperplanesCuttingAwayPrimals.begin(), hyperplanesCuttingAwayPrimals.end(),
            [](const auto& element1, const auto& element2) { return (element1.second < element2.second); });

        for(auto& HP : hyperplanesCuttingAwayPrimals)
        {
            env->dualSolver->addHyperplane(HP.first);
            hyperplaneAddedToConstraint.at(HP.first->sourceConstraint->index) = true;
            addedHyperplanes++;
            env->output->outputDebug(fmt::format("         Selected hyperplane cut for constraint {} that cuts away "
                                                 "previous primal solution with error {}",
                HP.first->sourceConstraint->index, HP.second));

            addedHyperplanes++;

            if(addedHyperplanes > maxHyperplanesPerIter)
                break;
        }
    }

    if(addedHyperplanes == 0)
    {
        env->output->outputDebug("         All nonlinear constraints fulfilled, so no constraint cuts added.");
    }

    env->timing->stopTimer("DualCutGenerationRootSearch");
    */

    env->timing->stopTimer("CallbackExternalHyperplaneGeneration");
}

std::string TaskSelectHyperplanesExternal::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT