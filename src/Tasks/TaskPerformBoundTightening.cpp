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

    env->timing->stopTimer("BoundTightening");
}

TaskPerformBoundTightening::~TaskPerformBoundTightening() = default;

void TaskPerformBoundTightening::run()
{
    env->timing->startTimer("BoundTightening");

    if(env->settings->getSetting<bool>("Model.BoundTightening.InitialPOA.Use") && env->reformulatedProblem
        && (sourceProblem->properties.numberOfNonlinearConstraints > 0
            || sourceProblem->objectiveFunction->properties.classification
                > E_ObjectiveFunctionClassification::Quadratic))
        createPOA();

    if(env->settings->getSetting<bool>("Model.BoundTightening.FeasibilityBased.Use"))
    {
        bool performBoundTightening = true;

        auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
            env->settings->getSetting<int>("Model.Reformulation.Quadratics.Strategy"));

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

            // sourceProblem is either env->problem or env->reformulatedProblem, and the latter's objective can
            // have a different direction than env->problem's (e.g. a maximize objective reformulated into an
            // equivalent minimize one, with its expression negated to match). objectiveBoundsAfter is then an
            // interval of the *negated* function, so translate it back to env->problem's sense (negating and
            // swapping l()/u()) before using it as a DualSolution value or cutoff, both of which are always
            // interpreted in env->problem's sense.
            bool sourceIsSignReversed
                = sourceProblem->objectiveFunction->direction != env->problem->objectiveFunction->direction;

            Interval originalSenseBounds = sourceIsSignReversed
                ? Interval(-objectiveBoundsAfter.u(), -objectiveBoundsAfter.l())
                : objectiveBoundsAfter;

            if(env->problem->objectiveFunction->properties.isMinimize)
            {
                DualSolution sol
                    = { {}, E_DualSolutionSource::MIPSolverBound, originalSenseBounds.l(), 0, false };
                env->dualSolver->addDualSolutionCandidate(sol);

                if(originalSenseBounds.u() < env->dualSolver->cutOffToUse) // Update MIP cutoff
                {
                    env->dualSolver->cutOffToUse = originalSenseBounds.u();
                    env->dualSolver->useCutOff = true;
                }
            }
            else if(env->problem->objectiveFunction->properties.isMaximize)
            {
                DualSolution sol
                    = { {}, E_DualSolutionSource::MIPSolverBound, originalSenseBounds.u(), 0, false };
                env->dualSolver->addDualSolutionCandidate(sol);

                if(originalSenseBounds.l() > env->dualSolver->cutOffToUse) // Update MIP cutoff
                {
                    env->dualSolver->cutOffToUse = originalSenseBounds.l();
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

std::shared_ptr<NLPSolverSHOT> TaskPerformBoundTightening::createPOASolver(ProblemPtr problem)
{
    auto solver = std::make_shared<NLPSolverSHOT>(env, problem);

    // The generated hyperplanes are instead added as linear constraints to the source problem in createPOA()
    solver->reuseHyperplanes = false;

    // The objective function of the relaxation is not the one of the source problem, so its cutoff does not apply
    solver->useCutOff = false;

    solver->solver->updateSetting("Termination.ConstraintTolerance",
        env->settings->getSetting<double>("Model.BoundTightening.InitialPOA.ConstraintTolerance"));
    solver->solver->updateSetting("Termination.ObjectiveConstraintTolerance",
        env->settings->getSetting<double>("Model.BoundTightening.InitialPOA.ObjectiveConstraintTolerance"));

    solver->solver->updateSetting("Termination.DualStagnation.ConstraintTolerance",
        env->settings->getSetting<double>("Model.BoundTightening.InitialPOA.StagnationConstraintTolerance"));
    solver->solver->updateSetting("Termination.DualStagnation.IterationLimit",
        env->settings->getSetting<int>("Model.BoundTightening.InitialPOA.StagnationIterationLimit"));

    solver->solver->updateSetting("Termination.IterationLimit",
        env->settings->getSetting<int>("Model.BoundTightening.InitialPOA.IterationLimit"));

    solver->solver->updateSetting("Termination.ObjectiveGap.Absolute",
        env->settings->getSetting<double>("Model.BoundTightening.InitialPOA.ObjectiveGapAbsolute"));
    solver->solver->updateSetting("Termination.ObjectiveGap.Relative",
        env->settings->getSetting<double>("Model.BoundTightening.InitialPOA.ObjectiveGapRelative"));

    solver->solver->updateSetting(
        "Dual.CutStrategy", env->settings->getSetting<int>("Model.BoundTightening.InitialPOA.CutStrategy"));

    solver->solver->updateSetting("Dual.ESH.InteriorPoint.UsePrimalSolution",
        static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepOriginal));

    return (solver);
}

void TaskPerformBoundTightening::solvePOAProblem(std::shared_ptr<NLPSolverSHOT> solver)
{
    for(auto& V : sourceProblem->allVariables)
    {
        solver->updateVariableLowerBound(V->getIndex(), V->lowerBound);
        solver->updateVariableUpperBound(V->getIndex(), V->upperBound);
    }

    // The time limit is shared by all the problems solved for the outer approximation
    solver->solver->updateSetting("Termination.TimeLimit",
        std::max(0.0,
            env->settings->getSetting<double>("Model.BoundTightening.InitialPOA.TimeLimit")
                - env->timing->getElapsedTime("BoundTighteningPOA")));

    // TODO handle return code?
    solver->solveProblem();
}

void TaskPerformBoundTightening::addPOACuts(std::shared_ptr<NLPSolverSHOT> solver,
    const std::map<std::string, NonlinearConstraintPtr>& convexConstraints, std::set<std::string>& constraintsWithCuts,
    int& hyperplaneCounter)
{
    auto solverDualSolver = solver->solver->getEnvironment()->dualSolver;

    // The solver generating the outer approximation reformulates the problem it is given, so the constraint
    // indices of its hyperplanes are not the ones of the problem the linearizations are added to: a constraint
    // index from it identifies another constraint here, or none at all. The constraints are matched by name
    // instead, and the cut is then generated from the constraint of this problem, so it is a valid linearization
    // of it even if the function it was generated for is not the same one. Only cuts for convex constraints are
    // valid everywhere, so no others are reused. Cuts for the objective function are not reused either, since the
    // objective function of the relaxation is not the one of the source problem.
    for(auto& HP : solverDualSolver->generatedHyperplanes)
    {
        auto newHP = std::make_shared<ConstraintHyperplane>();

        if(auto sourceHP = std::dynamic_pointer_cast<ConstraintHyperplane>(HP->sourceHyperplane))
        {
            if(!sourceHP->isGlobal)
                continue;

            auto match = convexConstraints.find(sourceHP->sourceConstraint->name);

            if(match == convexConstraints.end())
                continue;

            // The variables of this problem are the first ones of the reformulated problem the point comes from
            if((int)sourceHP->generatedPoint.size() < sourceProblem->properties.numberOfVariables)
                continue;

            newHP->source = sourceHP->source;
            newHP->sourceConstraint = match->second;
            newHP->generatedPoint = VectorDouble(sourceHP->generatedPoint.begin(),
                sourceHP->generatedPoint.begin() + sourceProblem->properties.numberOfVariables);
            newHP->isGlobal = sourceHP->isGlobal;

            auto optional = solverDualSolver->MIPSolver->createHyperplaneTerms(newHP);

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

            auto linearConstraint = std::make_shared<LinearConstraint>(fmt::format(
                "initPOA_{}_{}", newHP->sourceConstraint->name, hyperplaneCounter), SHOT_DBL_MIN, -tmpPair.second);

            linearConstraint->properties.classification = E_ConstraintClassification::Linear;
            linearConstraint->properties.convexity = E_Convexity::Linear;
            linearConstraint->properties.monotonicity = E_Monotonicity::Unknown;

            for(auto& E : tmpPair.first)
                linearConstraint->add(std::make_shared<LinearTerm>(E.second, sourceProblem->getVariable(E.first)));

            hyperplaneCounter++;
            constraintsWithCuts.insert(match->first);

            sourceProblem->add(std::move(linearConstraint));
        }
    }
}

void TaskPerformBoundTightening::createPOA()
{
    env->timing->startTimer("BoundTighteningPOA");

    env->output->outputInfo(" Generating initial polyhedral outer approximation of nonlinear feasible set.");

    // Only the convex constraints are part of the relaxation, and the cuts generated for them are valid everywhere
    std::map<std::string, NonlinearConstraintPtr> convexConstraints;

    for(auto& C : sourceProblem->nonlinearConstraints)
    {
        if(C->properties.convexity <= E_Convexity::Convex)
            convexConstraints.emplace(C->name, C);
    }

    std::set<std::string> constraintsWithCuts;
    int hyperplaneCounter = 0;

    // The solver is only created here, since the task is also created for the original problem, which the MIP
    // solver may not be able to handle, e.g. with quadratic constraints for Cbc
    relaxedProblem = sourceProblem->createCopy(env, true, true);

    try
    {
        POASolver = createPOASolver(relaxedProblem);
    }
    catch(Exception& e)
    {
        env->output->outputWarning(
            fmt::format("  - Initial polyhedral outer approximation not generated: {}", e.what()));
        env->timing->stopTimer("BoundTighteningPOA");
        return;
    }

    solvePOAProblem(POASolver);
    addPOACuts(POASolver, convexConstraints, constraintsWithCuts, hyperplaneCounter);

    auto& interiorPts = POASolver->solver->getEnvironment()->dualSolver->interiorPts;
    env->dualSolver->interiorPointCandidates.reserve(
        env->dualSolver->interiorPointCandidates.size() + interiorPts.size());

    for(auto& PT : interiorPts)
        env->dualSolver->interiorPointCandidates.push_back(PT);

    // The objective function only steers the cuts to where its optimum is, so the convex constraints without cuts
    // are also approximated by minimizing and maximizing the variables in their nonlinear terms
    auto variablesInConstraint = [](const NonlinearConstraintPtr& constraint)
    {
        std::set<int> indexes;

        for(auto& QT : constraint->quadraticTerms)
        {
            indexes.insert(QT->firstVariable->getIndex());
            indexes.insert(QT->secondVariable->getIndex());
        }

        for(auto& V : constraint->variablesInMonomialTerms)
            indexes.insert(V->getIndex());

        for(auto& V : constraint->variablesInSignomialTerms)
            indexes.insert(V->getIndex());

        for(auto& V : constraint->variablesInNonlinearExpression)
            indexes.insert(V->getIndex());

        return (indexes);
    };

    std::set<int> directionalVariables;

    for(auto& [name, constraint] : convexConstraints)
    {
        if(constraintsWithCuts.count(name) == 0)
        {
            auto indexes = variablesInConstraint(constraint);
            directionalVariables.insert(indexes.begin(), indexes.end());
        }
    }

    int maxDirectionalSolves = env->settings->getSetting<int>("Model.BoundTightening.InitialPOA.DirectionalSolves");
    double timeLimit = env->settings->getSetting<double>("Model.BoundTightening.InitialPOA.TimeLimit");
    int numberOfDirectionalSolves = 0;

    for(int variableIndex : directionalVariables)
    {
        for(auto direction : { E_ObjectiveFunctionDirection::Minimize, E_ObjectiveFunctionDirection::Maximize })
        {
            if(numberOfDirectionalSolves >= maxDirectionalSolves
                || env->timing->getElapsedTime("BoundTighteningPOA") >= timeLimit)
                break;

            // The variable is skipped if the constraints it is in have gotten cuts from the previous solves
            bool isInConstraintWithoutCuts = false;

            for(auto& [name, constraint] : convexConstraints)
            {
                if(constraintsWithCuts.count(name) == 0 && variablesInConstraint(constraint).count(variableIndex) > 0)
                {
                    isInConstraintWithoutCuts = true;
                    break;
                }
            }

            if(!isInConstraintWithoutCuts)
                break;

            auto directionalProblem = relaxedProblem->createCopy(env, false, false);

            auto objective = std::make_shared<LinearObjectiveFunction>(direction);
            objective->add(std::make_shared<LinearTerm>(1.0, directionalProblem->getVariable(variableIndex)));
            directionalProblem->add(std::move(objective));
            directionalProblem->updateProperties();

            std::shared_ptr<NLPSolverSHOT> directionalSolver;

            try
            {
                directionalSolver = createPOASolver(directionalProblem);
            }
            catch(Exception& e)
            {
                env->output->outputWarning(fmt::format("  - Directional solve for the initial polyhedral outer "
                                                       "approximation not performed: {}",
                    e.what()));
                continue;
            }

            solvePOAProblem(directionalSolver);
            addPOACuts(directionalSolver, convexConstraints, constraintsWithCuts, hyperplaneCounter);

            numberOfDirectionalSolves++;
        }
    }

    if(hyperplaneCounter > 0)
    {
        sourceProblem->properties.numberOfAddedLinearizations = hyperplaneCounter;
        sourceProblem->properties.numberOfNumericConstraints = sourceProblem->numericConstraints.size();
        sourceProblem->properties.numberOfLinearConstraints = sourceProblem->linearConstraints.size();
    }

    env->timing->stopTimer("BoundTighteningPOA");

    env->output->outputInfo(fmt::format("  - {} linear constraints generated in {:.2f} s ({} directional solves).",
        hyperplaneCounter, env->timing->getElapsedTime("BoundTighteningPOA"), numberOfDirectionalSolves));
}

} // namespace SHOT