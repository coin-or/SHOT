/** getCurrentDualBound() getCurrentDualBound() getCurrentDualBound()
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskPerformBoundTightening.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Solver.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"
#include "../NLPSolver/INLPSolver.h"

#include "../NLPSolver/NLPSolverSHOT.h"

namespace SHOT
{

TaskPerformBoundTightening::TaskPerformBoundTightening(EnvironmentPtr envPtr, ProblemPtr source) : TaskBase(envPtr)
{
    env->timing->startTimer("BoundTightening");

    sourceProblem = source;

    if(env->settings->getSetting<bool>("BoundTightening.InitialPOA.Use", "Model")
        && (sourceProblem->properties.numberOfNonlinearConstraints > 0
            || sourceProblem->objectiveFunction->properties.classification
                > E_ObjectiveFunctionClassification::Quadratic))
    {
        env->timing->startTimer("BoundTighteningPOA");

        relaxedProblem = sourceProblem->createCopy(env, true, true);
        POASolver = std::make_shared<NLPSolverSHOT>(env, relaxedProblem);

        POASolver->solver->updateSetting("ConstraintTolerance", "Termination",
            env->settings->getSetting<double>("BoundTightening.InitialPOA.ConstraintTolerance", "Model"));
        POASolver->solver->updateSetting("ObjectiveConstraintTolerance", "Termination",
            env->settings->getSetting<double>("BoundTightening.InitialPOA.ObjectiveConstraintTolerance", "Model"));

        POASolver->solver->updateSetting("DualStagnation.ConstraintTolerance", "Termination",
            env->settings->getSetting<double>("BoundTightening.InitialPOA.StagnationConstraintTolerance", "Model"));
        POASolver->solver->updateSetting("DualStagnation.IterationLimit", "Termination",
            env->settings->getSetting<int>("BoundTightening.InitialPOA.StagnationIterationLimit", "Model"));

        POASolver->solver->updateSetting("TimeLimit", "Termination",
            env->settings->getSetting<double>("BoundTightening.InitialPOA.TimeLimit", "Model"));

        POASolver->solver->updateSetting("IterationLimit", "Termination",
            env->settings->getSetting<int>("BoundTightening.InitialPOA.IterationLimit", "Model"));

        POASolver->solver->updateSetting("ObjectiveGap.Absolute", "Termination",
            env->settings->getSetting<double>("BoundTightening.InitialPOA.ObjectiveGapAbsolute", "Model"));
        POASolver->solver->updateSetting("ObjectiveGap.Relative", "Termination",
            env->settings->getSetting<double>("BoundTightening.InitialPOA.ObjectiveGapRelative", "Model"));

        POASolver->solver->updateSetting(
            "CutStrategy", "Dual", env->settings->getSetting<int>("BoundTightening.InitialPOA.CutStrategy", "Model"));

        POASolver->solver->updateSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual",
            static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepOriginal));

        env->timing->stopTimer("BoundTighteningPOA");
    }

    env->timing->stopTimer("BoundTightening");
}

TaskPerformBoundTightening::~TaskPerformBoundTightening() = default;

void TaskPerformBoundTightening::run()
{
    env->timing->startTimer("BoundTightening");

    if(env->settings->getSetting<bool>("BoundTightening.InitialPOA.Use", "Model") && env->reformulatedProblem
        && (sourceProblem->properties.numberOfNonlinearConstraints > 0
            || sourceProblem->objectiveFunction->properties.classification
                > E_ObjectiveFunctionClassification::Quadratic))
        createPOA();

    if(env->settings->getSetting<bool>("BoundTightening.FeasibilityBased.Use", "Model"))
    {
        bool performBoundTightening = true;

        auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
            env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"));

        // Do not do bound tightening on problems solved by MIP solver
        if(sourceProblem->properties.isLPProblem || sourceProblem->properties.isMILPProblem)
            performBoundTightening = false;
        else if(sourceProblem->properties.isMIQPProblem && quadraticStrategy != ES_QuadraticProblemStrategy::Nonlinear)
            performBoundTightening = false;
        else if(sourceProblem->properties.isMIQCQPProblem
            && quadraticStrategy != ES_QuadraticProblemStrategy::Nonlinear)
            performBoundTightening = false;

        if(performBoundTightening)
        {
            auto objectiveBoundsBefore = sourceProblem->objectiveFunction->getBounds();

            // Updating implicit variable bounds from signomials and nonlinear expressions

            auto infinteInterval = Interval(-SHOT_DBL_MAX, SHOT_DBL_MAX);

            if(sourceProblem->objectiveFunction->properties.hasSignomialTerms)
            {
                for(auto& ST : std::dynamic_pointer_cast<NonlinearObjectiveFunction>(sourceProblem->objectiveFunction)
                                   ->signomialTerms)
                {
                    for(auto& SE : ST->elements)
                        SE->tightenBounds(infinteInterval);
                }
            }

            if(sourceProblem->objectiveFunction->properties.hasNonlinearExpression)
            {
                std::dynamic_pointer_cast<NonlinearObjectiveFunction>(sourceProblem->objectiveFunction)
                    ->nonlinearExpression->tightenBounds(infinteInterval);
            }

            sourceProblem->doFBBT();

            auto objectiveBoundsAfter
                = sourceProblem->objectiveFunction->getBounds(); // To get objecive variable bounds
            env->output->outputInfo(fmt::format(
                "  - Objective bounds are: [{:g}, {:g}]", objectiveBoundsAfter.l(), objectiveBoundsAfter.u()));

            if(sourceProblem->objectiveFunction->properties.isMinimize)
            {
                DualSolution sol = { {}, E_DualSolutionSource::MIPSolverBound, objectiveBoundsAfter.l(), 0, false };
                env->dualSolver->addDualSolutionCandidate(sol);

                if(objectiveBoundsAfter.u() < env->dualSolver->cutOffToUse) // Update MIP cutoff
                {
                    env->dualSolver->cutOffToUse = objectiveBoundsAfter.u();
                    env->dualSolver->useCutOff = true;
                }
            }
            else if(sourceProblem->objectiveFunction->properties.isMaximize)
            {
                DualSolution sol = { {}, E_DualSolutionSource::MIPSolverBound, objectiveBoundsAfter.u(), 0, false };
                env->dualSolver->addDualSolutionCandidate(sol);

                if(objectiveBoundsAfter.l() > env->dualSolver->cutOffToUse) // Update MIP cutoff
                {
                    env->dualSolver->cutOffToUse = objectiveBoundsAfter.l();
                    env->dualSolver->useCutOff = true;
                }
            }
        }
    }

    env->timing->stopTimer("BoundTightening");
}

std::string TaskPerformBoundTightening::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

void TaskPerformBoundTightening::createPOA()
{
    env->timing->startTimer("BoundTighteningPOA");

    env->output->outputInfo(" Generating initial polyhedral outer approximation of nonlinear feasible set.");

    for(auto& V : sourceProblem->allVariables)
    {
        POASolver->updateVariableLowerBound(V->index, V->lowerBound);
        POASolver->updateVariableUpperBound(V->index, V->upperBound);
    }

    // TODO handle return code?
    POASolver->solveProblem();

    int hyperplaneCounter = 0;

    for(auto& HP : this->POASolver->solver->getEnvironment()->dualSolver->generatedHyperplanes)
    {
        auto newHP = std::make_shared<ConstraintHyperplane>();

        if(auto sourceHP = std::dynamic_pointer_cast<ConstraintHyperplane>(HP->sourceHyperplane))
        {
            newHP->source = sourceHP->source;
            newHP->sourceConstraint = std::dynamic_pointer_cast<NumericConstraint>(
                sourceProblem->getConstraint(sourceHP->sourceConstraint->index));
            newHP->generatedPoint = sourceHP->generatedPoint;
            newHP->isGlobal = sourceHP->isGlobal;

            auto optional
                = this->POASolver->solver->getEnvironment()->dualSolver->MIPSolver->createHyperplaneTerms(newHP);

            if(!optional)
                continue;

            auto tmpPair = optional.value();
            bool isOk = true;

            for(auto& E : tmpPair.first)
            {
                if(E.second != E.second) // Check for NaN
                {
                    env->output->outputError(
                        "        Warning: hyperplane not generated, NaN found in linear terms for variable "
                        + env->problem->getVariable(E.first)->name);

                    isOk = false;
                    break;
                }
            }

            if(!isOk)
                continue;

            // Small fix to fix badly scaled cuts.
            // TODO: this should be made so it also takes into account small/large coefficients of the linear terms
            if(abs(tmpPair.second) > 1e15)
            {
                double scalingFactor = abs(tmpPair.second) - 1e15;

                for(auto& E : tmpPair.first)
                    E.second /= scalingFactor;

                tmpPair.second /= scalingFactor;
            }

            auto linearConstraint = std::make_shared<LinearConstraint>(
                sourceProblem->properties.numberOfLinearConstraints + hyperplaneCounter,
                fmt::format("initPOA_{}_{}", newHP->sourceConstraint->name, hyperplaneCounter), SHOT_DBL_MIN,
                -tmpPair.second);

            linearConstraint->properties.classification = E_ConstraintClassification::Linear;
            linearConstraint->properties.convexity = E_Convexity::Linear;
            linearConstraint->properties.monotonicity = E_Monotonicity::Unknown;

            for(auto& E : tmpPair.first)
                linearConstraint->add(std::make_shared<LinearTerm>(E.second, sourceProblem->getVariable(E.first)));

            hyperplaneCounter++;

            sourceProblem->add(std::move(linearConstraint));
        }
    }

    for(auto& PT : this->POASolver->solver->getEnvironment()->dualSolver->interiorPts)
        env->dualSolver->interiorPointCandidates.push_back(PT);

    if(hyperplaneCounter > 0)
    {
        sourceProblem->properties.numberOfAddedLinearizations = hyperplaneCounter;
        sourceProblem->properties.numberOfNumericConstraints = sourceProblem->numericConstraints.size();
        sourceProblem->properties.numberOfLinearConstraints = sourceProblem->linearConstraints.size();
    }

    env->timing->stopTimer("BoundTighteningPOA");

    env->output->outputInfo(fmt::format("  - {} linear constraints generated in {:.2f} s.", hyperplaneCounter,
        env->timing->getElapsedTime("BoundTighteningPOA")));
}

} // namespace SHOT