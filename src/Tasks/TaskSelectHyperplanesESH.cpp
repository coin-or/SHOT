/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanesESH.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include "TaskSelectHyperplanesECP.h"
#include "../RootsearchMethod/IRootsearchMethod.h"

namespace SHOT
{

TaskSelectHyperplanesESH::TaskSelectHyperplanesESH(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("DualCutGenerationRootSearch");
    env->timing->stopTimer("DualCutGenerationRootSearch");
}

TaskSelectHyperplanesESH::~TaskSelectHyperplanesESH() = default;

void TaskSelectHyperplanesESH::run() { this->run(env->results->getPreviousIteration()->solutionPoints); }

void TaskSelectHyperplanesESH::run(std::vector<SolutionPoint> solPoints)
{
    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints == 0)
        return;

    env->output->outputDebug("        Selecting separating hyperplanes using the ESH method:");

    env->timing->startTimer("DualCutGenerationRootSearch");

    if(env->dualSolver->interiorPts.size() == 0)
    {
        if(!tSelectHPPts)
            tSelectHPPts = std::make_unique<TaskSelectHyperplanesECP>(env);

        env->output->outputDebug("         Adding cutting plane since no interior point is known.");
        tSelectHPPts->run(solPoints);

        env->timing->stopTimer("DualCutGenerationRootSearch");
        return;
    }
    else if(env->solutionStatistics.numberOfIterationsWithDualStagnation > 2
        && env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
    {
        if(!tSelectHPPts)
            tSelectHPPts = std::make_unique<TaskSelectHyperplanesECP>(env);

        env->output->outputDebug("         Adding cutting plane since the dual has stagnated.");
        tSelectHPPts->run(solPoints);

        env->timing->stopTimer("DualCutGenerationRootSearch");
        return;
    }

    int addedHyperplanes = 0;
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    auto constraintSelectionFactor
        = env->settings->getSetting<double>("HyperplaneCuts.ConstraintSelectionFactor", "Dual");
    bool useUniqueConstraints = env->settings->getSetting<bool>("ESH.Rootsearch.UniqueConstraints", "Dual");

    int rootMaxIter = env->settings->getSetting<int>("Rootsearch.MaxIterations", "Subsolver");
    double rootTerminationTolerance = env->settings->getSetting<double>("Rootsearch.TerminationTolerance", "Subsolver");
    double rootActiveConstraintTolerance
        = env->settings->getSetting<double>("Rootsearch.ActiveConstraintTolerance", "Subsolver");
    int maxHyperplanesPerIter = env->settings->getSetting<int>("HyperplaneCuts.MaxPerIteration", "Dual");
    double rootsearchConstraintTolerance
        = env->settings->getSetting<double>("ESH.Rootsearch.ConstraintTolerance", "Dual");
    double constraintMaxSelectionFactor
        = env->settings->getSetting<double>("HyperplaneCuts.MaxConstraintFactor", "Dual");

    // Contains boolean array that indicates if a constraint has been added or not
    std::vector<bool> hyperplaneAddedToConstraint(
        env->reformulatedProblem->properties.numberOfNumericConstraints, false);

    std::vector<std::tuple<int, int, NumericConstraintValues>> selectedNumericValues;
    std::vector<std::tuple<int, int, NumericConstraintValues>> nonconvexSelectedNumericValues;

    bool useMaxFunction = env->settings->getSetting<bool>("ESH.Rootsearch.UseMaxFunction", "Dual");

    if(useMaxFunction)
        constraintSelectionFactor = 1.0;

    // First find the interior point - solution point - constraint combination that will be used for root search
    for(size_t i = 0; i < solPoints.size(); i++)
    {
        auto numericConstraintValues = env->reformulatedProblem->getFractionOfDeviatingNonlinearConstraints(
            solPoints.at(i).point, 0.0, constraintSelectionFactor);

        if(numericConstraintValues.size() == 0)
            continue;

        if(addedHyperplanes >= maxHyperplanesPerIter)
        {
            env->output->outputDebug("        Not generating hyperplane using ESH: Max number already added.");
            env->timing->stopTimer("DualCutGenerationRootSearch");
            break;
        }

        if(useMaxFunction)
        {
            for(size_t j = 0; j < env->dualSolver->interiorPts.size(); j++)
            {
                auto numericConstraintValuesConvex = NumericConstraintValues();
                auto numericConstraintValuesAll = NumericConstraintValues();

                for(auto& NCV : numericConstraintValues)
                {
                    // Do not add hyperplane if there are numerical errors
                    if(std::isnan(NCV.error) || std::isnan(NCV.normalizedValue))
                        continue;

                    // Do not add hyperplane if less than this tolerance or negative
                    if(NCV.normalizedValue < rootsearchConstraintTolerance)
                        continue;

                    if(NCV.constraint->properties.convexity == E_Convexity::Convex)
                        numericConstraintValuesConvex.push_back(NCV);
                    else
                        numericConstraintValuesAll.push_back(NCV);

                    env->output->outputDebug("        Not generating hyperplane using ESH: Numerical error.");
                    continue;
                }

                if(numericConstraintValuesConvex.size() > 0)
                {
                    selectedNumericValues.emplace_back(i, j, numericConstraintValuesConvex);
                }

                if(numericConstraintValuesAll.size() > 0)
                    nonconvexSelectedNumericValues.emplace_back(i, j, numericConstraintValuesAll);
            }
        }
        else
        {
            for(size_t j = 0; j < env->dualSolver->interiorPts.size(); j++)
            {
                for(auto& NCV : numericConstraintValues)
                {
                    // Do not add hyperplane if one has been added for this constraint already
                    if(useUniqueConstraints && hyperplaneAddedToConstraint.at(NCV.constraint->index))
                    {
                        env->output->outputDebug("        Not generating hyperplane using ESH: Hyperplane generated "
                                                 "for constraint already.");
                        continue;
                    }

                    // Do not add hyperplane if there are numerical errors
                    if(std::isnan(NCV.error) || std::isnan(NCV.normalizedValue))
                    {
                        env->output->outputDebug(" Not generating hyperplane using ESH: Numerical error.");
                        continue;
                    }

                    // Do not add hyperplane if less than this tolerance or negative
                    if(NCV.normalizedValue < rootsearchConstraintTolerance)
                    {
                        env->output->outputDebug(
                            "        Not generating hyperplane using ESH: Rootsearch tolerance reached.");
                        continue;
                    }

                    // Do not add hyperplane if constraint value is much less than largest
                    if(NCV.error < constraintMaxSelectionFactor * numericConstraintValues.at(0).error)
                    {
                        env->output->outputDebug(
                            "        Not generating hyperplane using ESH: Constraint value much smaller than largest.");
                        continue;
                    }

                    if(NCV.constraint->properties.convexity != E_Convexity::Convex)
                    {
                        auto numericConstraintValues = NumericConstraintValues();
                        numericConstraintValues.push_back(NCV);
                        nonconvexSelectedNumericValues.emplace_back(i, j, numericConstraintValues);
                        continue;
                    }

                    auto numericConstraintValues = NumericConstraintValues();
                    numericConstraintValues.push_back(NCV);
                    selectedNumericValues.emplace_back(i, j, numericConstraintValues);
                }
            }
        }
    }

    // First try to do root search on convex constraints only
    for(auto& values : selectedNumericValues)
    {
        int solutionPtIndex = std::get<0>(values);
        int interiorPtIndex = std::get<1>(values);

        if(addedHyperplanes > maxHyperplanesPerIter)
            break;

        if(useMaxFunction)
        {
            VectorDouble externalPoint;
            VectorDouble internalPoint;

            std::vector<NumericConstraint*> currentConstraints;

            for(auto& NCV : std::get<2>(values))
                currentConstraints.push_back(NCV.constraint.get());

            try
            {
                env->timing->startTimer("DualCutGenerationRootSearch");
                auto xNewc = env->rootsearchMethod->findZero(env->dualSolver->interiorPts.at(interiorPtIndex)->point,
                    solPoints.at(solutionPtIndex).point, rootMaxIter, rootTerminationTolerance,
                    rootActiveConstraintTolerance, currentConstraints, true);

                env->timing->stopTimer("DualCutGenerationRootSearch");
                internalPoint = xNewc.first;
                externalPoint = xNewc.second;
            }
            catch(std::exception&)
            {
                env->timing->stopTimer("DualCutGenerationRootSearch");
                externalPoint = solPoints.at(solutionPtIndex).point;

                env->output->outputDebug(
                    "         Cannot find solution with rootsearch, using solution point instead.");
            }

            auto externalConstraintValue = env->reformulatedProblem->getMaxNumericConstraintValue(
                externalPoint, env->reformulatedProblem->nonlinearConstraints, 0.0);

            if(externalConstraintValue.normalizedValue >= 0)
            {
                double hash = Utilities::calculateHash(externalPoint);

                if(env->dualSolver->hasHyperplaneBeenAdded(hash, externalConstraintValue.constraint->index))
                {
                    env->output->outputDebug("         Hyperplane already added for constraint "
                        + std::to_string(externalConstraintValue.constraint->index) + " and hash "
                        + std::to_string(hash));
                    continue;
                }

                auto hyperplane = std::make_shared<ConstraintHyperplane>();
                hyperplane->sourceConstraint = externalConstraintValue.constraint;
                hyperplane->generatedPoint = externalPoint;
                hyperplane->isGlobal = true; // Only convex constraints used so far

                if(solPoints.at(solutionPtIndex).isRelaxedPoint)
                {
                    hyperplane->source = E_HyperplaneSource::MIPCallbackRelaxed;
                }
                else if(solutionPtIndex == 0 && currIter->isMIP())
                {
                    hyperplane->source = E_HyperplaneSource::MIPOptimalRootsearch;
                }
                else if(currIter->isMIP())
                {
                    hyperplane->source = E_HyperplaneSource::MIPSolutionPoolRootsearch;
                }
                else
                {
                    hyperplane->source = E_HyperplaneSource::LPRelaxedRootsearch;
                }

                env->dualSolver->addHyperplane(hyperplane);

                hyperplaneAddedToConstraint.at(externalConstraintValue.constraint->index) = true;

                env->output->outputDebug("         Added hyperplane to waiting list with deviation: "
                    + Utilities::toString(externalConstraintValue.error));

                addedHyperplanes++;
            }
            else
            {
                env->output->outputDebug("         Could not add hyperplane to waiting list since constraint value is "
                    + std::to_string(externalConstraintValue.normalizedValue));
            }
        }
        else
        {
            for(auto& NCV : std::get<2>(values))
            {
                if(NCV.error <= 0.0)
                    continue;

                VectorDouble externalPoint;
                VectorDouble internalPoint;

                std::vector<NumericConstraint*> currentConstraint;
                currentConstraint.push_back(std::dynamic_pointer_cast<NumericConstraint>(NCV.constraint).get());

                try
                {
                    env->timing->startTimer("DualCutGenerationRootSearch");
                    auto xNewc = env->rootsearchMethod->findZero(
                        env->dualSolver->interiorPts.at(interiorPtIndex)->point, solPoints.at(solutionPtIndex).point,
                        rootMaxIter, rootTerminationTolerance, rootActiveConstraintTolerance, currentConstraint, true);

                    env->timing->stopTimer("DualCutGenerationRootSearch");
                    internalPoint = xNewc.first;
                    externalPoint = xNewc.second;
                }
                catch(std::exception&)
                {
                    env->timing->stopTimer("DualCutGenerationRootSearch");
                    externalPoint = solPoints.at(solutionPtIndex).point;

                    env->output->outputDebug(
                        "         Cannot find solution with rootsearch, using solution point instead.");
                }

                auto externalConstraintValue = NCV.constraint->calculateNumericValue(externalPoint);

                if(externalConstraintValue.normalizedValue >= 0)
                {
                    double hash = Utilities::calculateHash(externalPoint);

                    if(env->dualSolver->hasHyperplaneBeenAdded(hash, externalConstraintValue.constraint->index))
                    {
                        env->output->outputDebug("         Hyperplane already added for constraint "
                            + std::to_string(externalConstraintValue.constraint->index) + " and hash "
                            + std::to_string(hash));
                        continue;
                    }

                    auto hyperplane = std::make_shared<ConstraintHyperplane>();
                    hyperplane->sourceConstraint = externalConstraintValue.constraint;
                    hyperplane->generatedPoint = externalPoint;
                    hyperplane->isGlobal
                        = (externalConstraintValue.constraint->properties.convexity <= E_Convexity::Convex);

                    if(solPoints.at(solutionPtIndex).isRelaxedPoint)
                    {
                        hyperplane->source = E_HyperplaneSource::MIPCallbackRelaxed;
                    }
                    else if(solutionPtIndex == 0 && currIter->isMIP())
                    {
                        hyperplane->source = E_HyperplaneSource::MIPOptimalRootsearch;
                    }
                    else if(currIter->isMIP())
                    {
                        hyperplane->source = E_HyperplaneSource::MIPSolutionPoolRootsearch;
                    }
                    else
                    {
                        hyperplane->source = E_HyperplaneSource::LPRelaxedRootsearch;
                    }

                    env->dualSolver->addHyperplane(hyperplane);

                    hyperplaneAddedToConstraint.at(externalConstraintValue.constraint->index) = true;

                    env->output->outputDebug("         Added hyperplane to waiting list with deviation: "
                        + Utilities::toString(externalConstraintValue.error));

                    addedHyperplanes++;
                }
                else
                {
                    env->output->outputDebug(
                        "         Could not add hyperplane to waiting list since constraint value is "
                        + std::to_string(externalConstraintValue.normalizedValue));
                }
            }
        }
    }

    std::vector<std::pair<ConstraintHyperplanePtr, double>> hyperplanesCuttingAwayPrimals;

    // If no hyperplanes added, do rootsearch also on nonconvex constraints
    if(addedHyperplanes == 0 && nonconvexSelectedNumericValues.size() > 0)
    {
        env->output->outputDebug("         Could not add hyperplane for convex constraints");

        for(auto& values : nonconvexSelectedNumericValues)
        {
            int solutionPtIndex = std::get<0>(values);
            int interiorPtIndex = std::get<1>(values);

            if(addedHyperplanes > maxHyperplanesPerIter)
                break;

            if(useMaxFunction)
            {
                VectorDouble externalPoint;
                VectorDouble internalPoint;

                std::vector<NumericConstraint*> currentConstraints;

                for(auto& NCV : std::get<2>(values))
                    currentConstraints.push_back(NCV.constraint.get());

                try
                {
                    env->timing->startTimer("DualCutGenerationRootSearch");
                    auto xNewc = env->rootsearchMethod->findZero(
                        env->dualSolver->interiorPts.at(interiorPtIndex)->point, solPoints.at(solutionPtIndex).point,
                        rootMaxIter, rootTerminationTolerance, rootActiveConstraintTolerance, currentConstraints, true);

                    env->timing->stopTimer("DualCutGenerationRootSearch");
                    internalPoint = xNewc.first;
                    externalPoint = xNewc.second;
                }
                catch(std::exception&)
                {
                    env->timing->stopTimer("DualCutGenerationRootSearch");
                    externalPoint = solPoints.at(solutionPtIndex).point;

                    env->output->outputDebug(
                        "         Cannot find solution with rootsearch, using solution point instead.");
                }

                auto externalConstraintValue = env->reformulatedProblem->getMaxNumericConstraintValue(
                    externalPoint, env->reformulatedProblem->nonlinearConstraints, 0.0);

                if(externalConstraintValue.normalizedValue >= 0)
                {
                    double hash = Utilities::calculateHash(externalPoint);

                    if(env->dualSolver->hasHyperplaneBeenAdded(hash, externalConstraintValue.constraint->index))
                    {
                        env->output->outputDebug("         Hyperplane already added for constraint "
                            + std::to_string(externalConstraintValue.constraint->index) + " and hash "
                            + std::to_string(hash));
                        continue;
                    }

                    auto hyperplane = std::make_shared<ConstraintHyperplane>();
                    hyperplane->sourceConstraint = externalConstraintValue.constraint;
                    hyperplane->generatedPoint = externalPoint;
                    hyperplane->isGlobal
                        = (externalConstraintValue.constraint->properties.convexity == E_Convexity::Convex);

                    if(solPoints.at(solutionPtIndex).isRelaxedPoint)
                    {
                        hyperplane->source = E_HyperplaneSource::MIPCallbackRelaxed;
                    }
                    else if(solutionPtIndex == 0 && currIter->isMIP())
                    {
                        hyperplane->source = E_HyperplaneSource::MIPOptimalRootsearch;
                    }
                    else if(currIter->isMIP())
                    {
                        hyperplane->source = E_HyperplaneSource::MIPSolutionPoolRootsearch;
                    }
                    else
                    {
                        hyperplane->source = E_HyperplaneSource::LPRelaxedRootsearch;
                    }

                    env->output->outputDebug("         Added hyperplane to waiting list with deviation: "
                        + Utilities::toString(externalConstraintValue.error));

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
                        env->dualSolver->addHyperplane(hyperplane);
                        hyperplaneAddedToConstraint.at(externalConstraintValue.constraint->index) = true;
                        addedHyperplanes++;
                    }

                    addedHyperplanes++;
                }
                else
                {
                    env->output->outputDebug(
                        "         Could not add hyperplane to waiting list since constraint value is "
                        + std::to_string(externalConstraintValue.normalizedValue));
                }
            }
            else
            {
                for(auto& NCV : std::get<2>(values))
                {
                    if(NCV.error <= 0.0)
                        continue;

                    VectorDouble externalPoint;
                    VectorDouble internalPoint;

                    std::vector<NumericConstraint*> currentConstraint;
                    currentConstraint.push_back(std::dynamic_pointer_cast<NumericConstraint>(NCV.constraint).get());

                    try
                    {
                        env->timing->startTimer("DualCutGenerationRootSearch");
                        auto xNewc
                            = env->rootsearchMethod->findZero(env->dualSolver->interiorPts.at(interiorPtIndex)->point,
                                solPoints.at(solutionPtIndex).point, rootMaxIter, rootTerminationTolerance,
                                rootActiveConstraintTolerance, currentConstraint, true);

                        env->timing->stopTimer("DualCutGenerationRootSearch");
                        internalPoint = xNewc.first;
                        externalPoint = xNewc.second;
                    }
                    catch(std::exception&)
                    {
                        env->timing->stopTimer("DualCutGenerationRootSearch");
                        externalPoint = solPoints.at(solutionPtIndex).point;

                        env->output->outputDebug(
                            "         Cannot find solution with rootsearch, using solution point instead.");
                    }

                    auto externalConstraintValue = NCV.constraint->calculateNumericValue(externalPoint);

                    if(externalConstraintValue.normalizedValue >= 0)
                    {
                        double hash = Utilities::calculateHash(externalPoint);

                        if(env->dualSolver->hasHyperplaneBeenAdded(hash, externalConstraintValue.constraint->index))
                        {
                            env->output->outputTrace("         Hyperplane already added for constraint "
                                + std::to_string(externalConstraintValue.constraint->index) + " and hash "
                                + std::to_string(hash));
                            continue;
                        }

                        auto hyperplane = std::make_shared<ConstraintHyperplane>();
                        hyperplane->sourceConstraint = externalConstraintValue.constraint;
                        hyperplane->generatedPoint = externalPoint;
                        hyperplane->isGlobal = (NCV.constraint->properties.convexity <= E_Convexity::Convex);

                        if(solPoints.at(solutionPtIndex).isRelaxedPoint)
                        {
                            hyperplane->source = E_HyperplaneSource::MIPCallbackRelaxed;
                        }
                        else if(solutionPtIndex == 0 && currIter->isMIP())
                        {
                            hyperplane->source = E_HyperplaneSource::MIPOptimalRootsearch;
                        }
                        else if(currIter->isMIP())
                        {
                            hyperplane->source = E_HyperplaneSource::MIPSolutionPoolRootsearch;
                        }
                        else
                        {
                            hyperplane->source = E_HyperplaneSource::LPRelaxedRootsearch;
                        }

                        env->output->outputDebug("         Added hyperplane to waiting list with deviation: "
                            + Utilities::toString(externalConstraintValue.error));

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
                            env->dualSolver->addHyperplane(hyperplane);
                            hyperplaneAddedToConstraint.at(NCV.constraint->index) = true;
                            addedHyperplanes++;
                        }
                    }
                    else
                    {
                        env->output->outputDebug(
                            "         Could not add hyperplane to waiting list since constraint value is "
                            + std::to_string(externalConstraintValue.normalizedValue));
                    }
                }
            }
        }
    }

    // Finally also add the hyperplanes cutting away primal solutions
    if(addedHyperplanes == 0 && hyperplanesCuttingAwayPrimals.size() > 0)
    {
        env->output->outputDebug("         Adding hyperplanes that cut away primal solutions");

        std::sort(hyperplanesCuttingAwayPrimals.begin(), hyperplanesCuttingAwayPrimals.end(),
            [](const auto& element1, const auto& element2) { return (element1.second < element2.second); });

        for(auto& HP : hyperplanesCuttingAwayPrimals)
        {
            double hash = Utilities::calculateHash(HP.first->generatedPoint);

            if(env->dualSolver->hasHyperplaneBeenAdded(hash, HP.first->sourceConstraint->index))
            {
                env->output->outputTrace("         Hyperplane already added for constraint "
                    + std::to_string(HP.first->sourceConstraint->index) + " and hash " + std::to_string(hash));
                continue;
            }

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
}

std::string TaskSelectHyperplanesESH::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT