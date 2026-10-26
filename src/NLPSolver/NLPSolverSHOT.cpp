/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverSHOT.h"

#include "../DualSolver.h"
#include "../Environment.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Solver.h"
#include "../Timing.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Model/ObjectiveFunction.h"
#include "../Model/Problem.h"

#include <algorithm>
#include <unordered_map>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

namespace SHOT
{

NLPSolverSHOT::NLPSolverSHOT(EnvironmentPtr envPtr, ProblemPtr source) : INLPSolver(envPtr)
{
    sourceProblem = source;
    variableLowerBounds = sourceProblem->getVariableLowerBounds();
    variableUpperBounds = sourceProblem->getVariableUpperBounds();
    initializeMIPProblem();
}

NLPSolverSHOT::~NLPSolverSHOT() = default;

// A nested solver cannot solve another problem: its task loop has terminated, and its results, bounds and cuts are
// those of the previous problem, of which the cuts for nonconvex constraints are not valid for another fixing of the
// discrete variables. A new nested solver is therefore created for each solve after the first.
void NLPSolverSHOT::initializeMIPProblem()
{
    solver = std::make_shared<Solver>();
    isNestedSolverUsed = false;

    // A SHOT NLP subsolver must not create another fixed-integer or polishing
    // NLP subsolver. With no external NLP backend, that would recurse during
    // strategy construction before any optimization can take place.
    solver->updateSetting("Primal.FixedInteger.Use", false);
    solver->updateSetting("Primal.PolishSolution.NumberOfPoints", 0);

    solver->getEnvironment()->output->setPrefix("      | ");

    if(env->settings->getSetting<bool>("Output.Console.PrimalSolver.Show"))
    {
        solver->updateSetting(
            "Output.Console.LogLevel", env->settings->getSetting<int>("Output.Console.LogLevel"));
        solver->updateSetting(
            "Output.Console.DualSolver.Show", env->settings->getSetting<bool>("Output.Console.DualSolver.Show"));
    }
    else
    {
        solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    }

    solver->updateSetting("Output.Console.Iteration.Detail", 0);

    solver->updateSetting("Output.Debug.Enable", env->settings->getSetting<bool>("Output.Debug.Enable"));
    solver->updateSetting("Dual.CutStrategy", 0);
    solver->updateSetting("Dual.TreeStrategy", 1);
    solver->updateSetting("Dual.MIP.Solver", env->settings->getSetting<int>("Dual.MIP.Solver"));
    solver->updateSetting("Dual.MIP.NumberOfThreads", env->settings->getSetting<int>("Dual.MIP.NumberOfThreads"));

    solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
    solver->updateSetting("Model.BoundTightening.FeasibilityBased.MaxIterations", 0);

    solver->updateSetting("Output.Console.Iteration.Detail", 0);

    solver->updateSetting(
        "Model.Convexity.AssumeConvex", env->settings->getSetting<bool>("Model.Convexity.AssumeConvex"));

    solver->updateSetting("Termination.ObjectiveGap.Relative", 1e-6);
    solver->updateSetting("Termination.ObjectiveGap.Absolute", 1e-6);
    solver->updateSetting(
        "Termination.ConstraintTolerance", env->settings->getSetting<double>("Termination.ConstraintTolerance"));

    solver->updateSetting(
        "Termination.TimeLimit", env->settings->getSetting<double>("Primal.FixedInteger.TimeLimit"));
    // Without an iteration limit, the nested solver uses its default Termination.IterationLimit
    if(int iterationLimit = env->settings->getSetting<int>("Primal.FixedInteger.IterationLimit"); iterationLimit > 0)
        solver->updateSetting("Termination.IterationLimit", iterationLimit);

    solver->updateSetting("Termination.DualStagnation.IterationLimit", 20);

    solver->updateSetting(
        "Model.BoundTightening.FeasibilityBased.Use", env->settings->getSetting<bool>("Subsolver.SHOT.UseFBBT"));

    // Set the debug path for the subsolver
    auto mainDebugPath = env->settings->getSetting<std::string>("Output.Debug.Path");
    fs::filesystem::path subproblemDebugPath(mainDebugPath);
    // subproblemDebugPath /= ("SHOT_fixedNLP_" + std::to_string(env->results->getCurrentIteration()->iterationNumber));
    subproblemDebugPath /= ("SHOT_fixedNLP");
    solver->updateSetting("Output.Debug.Path", subproblemDebugPath.string());

    for(auto& S : settingUpdates)
        std::visit([&](auto value) { solver->updateSetting(S.first, value); }, S.second);

    relaxedProblem = sourceProblem->createCopy(solver->getEnvironment(), true, false, false);

    for(auto& VAR : relaxedProblem->allVariables)
        relaxedProblem->setVariableBounds(
            VAR->getIndex(), variableLowerBounds[VAR->getIndex()], variableUpperBounds[VAR->getIndex()]);

    // Solver::selectStrategy() (called by setProblem()) catches its own initialization exceptions and returns
    // false rather than propagating them, so a failure here would otherwise be silently swallowed: this object
    // would be considered successfully constructed with no solution strategy set, and later crash on the
    // assert in Solver::solveProblem() instead. Throw here so the caller can catch it and disable this NLP
    // solver instead.
    if(!solver->setProblem(relaxedProblem, relaxedProblem))
        throw Exception("Could not initialize the nested SHOT solver for the fixed-integer NLP problem.");

    // No bound tightening is done when the problem is set above, since the bounds have already been tightened in the
    // main solver, but only when the discrete variables have been fixed in solveProblemInstance(). That uses the
    // settings of the nested solver, so it gets the number of passes of the main solver here.
    if(env->settings->getSetting<bool>("Subsolver.SHOT.UseFBBT"))
        solver->updateSetting("Model.BoundTightening.FeasibilityBased.MaxIterations",
            env->settings->getSetting<int>("Model.BoundTightening.FeasibilityBased.MaxIterations"));
}

void NLPSolverSHOT::setStartingPoint(VectorInteger /*variableIndexes*/, VectorDouble /*variableValues*/) { }

void NLPSolverSHOT::clearStartingPoint() { }

void NLPSolverSHOT::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    fixedVariableIndexes = variableIndexes;
    fixedVariableValues = variableValues;
}

void NLPSolverSHOT::unfixVariables()
{
    variableLowerBounds = sourceProblem->getVariableLowerBounds();
    variableUpperBounds = sourceProblem->getVariableUpperBounds();

    // The bounds are set in the nested solver for the next solve
    for(auto& VAR : sourceProblem->allVariables)
    {
        relaxedProblem->setVariableBounds(VAR->getIndex(), VAR->lowerBound, VAR->upperBound);
        VAR->properties.hasLowerBoundBeenTightened = false;
        VAR->properties.hasUpperBoundBeenTightened = false;
    }

    fixedVariableIndexes.clear();
    fixedVariableValues.clear();
}

void NLPSolverSHOT::saveOptionsToFile([[maybe_unused]] std::string fileName) { }

void NLPSolverSHOT::saveProblemToFile([[maybe_unused]] std::string fileName) { }

VectorDouble NLPSolverSHOT::getSolution()
{
    if(solver->hasPrimalSolution())
    {
        auto solution = solver->getPrimalSolution().point;

        if(sourceProblem->objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic)
            solution.push_back(solver->getPrimalSolution().objValue);

        return (solution);
    }

    VectorDouble solution;
    return (solution);
}

double NLPSolverSHOT::getSolution([[maybe_unused]] int i)
{
    throw std::logic_error("getSolution(int) not implemented");
}

double NLPSolverSHOT::getObjectiveValue()
{
    if(solver->hasPrimalSolution())
        return (solver->getPrimalSolution().objValue);

    return (sourceProblem->objectiveFunction->properties.isMinimize ? SHOT_DBL_MAX : SHOT_DBL_MIN);
}

E_NLPSolutionStatus NLPSolverSHOT::solveProblemInstance()
{
    if(isNestedSolverUsed)
    {
        try
        {
            initializeMIPProblem();
        }
        catch(const std::exception& e)
        {
            env->output->outputError(fmt::format("        Cannot solve fixed NLP problem with SHOT: {}", e.what()));
            return E_NLPSolutionStatus::Error;
        }
    }

    isNestedSolverUsed = true;

    solver->getEnvironment()->output->outputInfo("");

#ifdef SIMPLE_OUTPUT_CHARS
    solver->getEnvironment()->output->outputInfo(
        "- Solving fixed NLP problem "
        "-------------------------------------------------------------------------------------------");
#else
    solver->getEnvironment()->output->outputInfo(
        "╶ Solving fixed NLP problem "
        "──────────────────────────────────────────────────────────────────────────────────────────╴");
#endif

    solver->getEnvironment()->output->outputInfo("");

    // Set fixed discrete variables
    for(size_t i = 0; i < fixedVariableIndexes.size(); ++i)
        relaxedProblem->setVariableBounds(fixedVariableIndexes[i], fixedVariableValues[i], fixedVariableValues[i]);

    // Tighten the bounds if fixed
    if(fixedVariableIndexes.size() > 0
        && solver->getEnvironment()->settings->getSetting<bool>("Model.BoundTightening.FeasibilityBased.Use"))
        relaxedProblem->doFBBT();

    // Update the bounds to the MIP solver
    for(auto& VAR : relaxedProblem->allVariables)
        solver->getEnvironment()->dualSolver->MIPSolver->updateVariableBound(
            VAR->getIndex(), VAR->lowerBound, VAR->upperBound);

    // Setting the cutoff value from currently best known solution
    if(useCutOff && env->dualSolver->cutOffToUse != SHOT_DBL_MAX)
    {
        solver->updateSetting("Dual.MIP.CutOff.InitialValue", env->dualSolver->cutOffToUse);
        solver->updateSetting("Dual.MIP.CutOff.UseInitialValue", true);
    }

    if(timeLimit < SHOT_DBL_MAX)
    {
        solver->updateSetting("Termination.TimeLimit",
            std::min(env->settings->getSetting<double>("Primal.FixedInteger.TimeLimit"), timeLimit));
    }

    if(!problemInfoPrinted)
    {
        solver->getEnvironment()->report->outputProblemInstanceReport();
        solver->getEnvironment()->report->outputOptionsReport();
        problemInfoPrinted = true;
    }

    // The nested solver can have been created long before it is used, e.g., for the final polish, and its time limit
    // is for this solve
    solver->getEnvironment()->timing->restartTimer("Total");

    if(!solver->solveProblem())
        return E_NLPSolutionStatus::Error;

    solver->getEnvironment()->report->outputSolutionReport();

    solver->getEnvironment()->output->outputInfo("");

    int hyperplaneCounter = 0;

    if(reuseHyperplanes && env->settings->getSetting<bool>("Subsolver.SHOT.ReuseHyperplanes.Use"))
    {
        auto& subsolverHyperplanes = solver->getEnvironment()->dualSolver->generatedHyperplanes;

        int numHyperplanesToCopy = subsolverHyperplanes.size()
            * env->settings->getSetting<double>("Subsolver.SHOT.ReuseHyperplanes.Fraction");

        // The nested solver reformulates its problem again, so its constraint indices do not match the ones in the
        // main reformulated problem. Constraints are matched by name instead, and the cut is then generated from the
        // main problem's constraint, so it is a valid linearization even if the matched function is not the same.
        // Only cuts for convex constraints are valid everywhere, so the other ones are not reused.
        std::unordered_map<std::string, NonlinearConstraintPtr> mainConstraints;

        for(auto& C : env->reformulatedProblem->nonlinearConstraints)
        {
            if(C->properties.convexity <= E_Convexity::Convex)
                mainConstraints.emplace(C->name, C);
        }

        auto mainObjective = env->reformulatedProblem->objectiveFunction;

        bool isObjectiveConvex = mainObjective->properties.convexity == E_Convexity::Linear
            || (mainObjective->properties.isMinimize && mainObjective->properties.convexity == E_Convexity::Convex)
            || (mainObjective->properties.isMaximize && mainObjective->properties.convexity == E_Convexity::Concave);

        for(auto& GHP : subsolverHyperplanes)
        {
            if(hyperplaneCounter >= numHyperplanesToCopy)
                break;

            auto numericHP = std::dynamic_pointer_cast<NumericHyperplane>(GHP->sourceHyperplane);

            if(!numericHP)
                continue;

            // The nested solver's original variables are those of the main reformulated problem, and the auxiliary
            // variables of the main reformulation are recalculated from the original ones
            VectorDouble point(numericHP->generatedPoint.begin(),
                numericHP->generatedPoint.begin() + env->problem->properties.numberOfVariables);

            if((int)point.size() < env->reformulatedProblem->properties.numberOfVariables)
                env->reformulatedProblem->augmentAuxiliaryVariableValues(point);

            assert((int)point.size() == env->reformulatedProblem->properties.numberOfVariables);

            if(auto constraintHP = std::dynamic_pointer_cast<ConstraintHyperplane>(numericHP))
            {
                auto mainConstraint = mainConstraints.find(constraintHP->sourceConstraint->name);

                if(mainConstraint == mainConstraints.end())
                    continue;

                auto hyperplane = std::make_shared<ConstraintHyperplane>();
                hyperplane->generatedPoint = point;
                hyperplane->sourceConstraint = mainConstraint->second;
                hyperplane->isGlobal = true;
                hyperplane->source = E_HyperplaneSource::PrimalSolutionSearch;

                env->dualSolver->addHyperplane(hyperplane);
                hyperplaneCounter++;
            }
            else if(std::dynamic_pointer_cast<ObjectiveHyperplane>(numericHP))
            {
                // Objective cuts are expressed using the auxiliary objective variable in the main dual problem, and
                // are only valid everywhere if the objective function is convex
                if(!isObjectiveConvex || !env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
                    continue;

                auto hyperplane = std::make_shared<ObjectiveHyperplane>();
                hyperplane->generatedPoint = point;
                hyperplane->objectiveFunctionValue = mainObjective->calculateValue(point);
                hyperplane->isGlobal = true;
                hyperplane->source = E_HyperplaneSource::PrimalSolutionSearch;

                env->dualSolver->addHyperplane(hyperplane);
                hyperplaneCounter++;
            }
        }

        subsolverHyperplanes.clear();

        solver->getEnvironment()->output->outputInfo(
            fmt::format(" Added {} hyperplanes generated by SHOT primal NLP solver.", hyperplaneCounter));
    }

    E_NLPSolutionStatus status;

    auto terminationReason = solver->getEnvironment()->results->terminationReason;

    // A gap-tolerance-based termination does not by itself guarantee a primal solution exists: both the dual and
    // primal bounds default to the same "unset" sentinel value when neither has been established, which can
    // trivially satisfy an absolute/relative gap tolerance of (near) zero. Without this check, callers would be
    // told the nested solve was "Optimal" and then get an empty solution point from getSolution().
    if(solver->hasPrimalSolution()
        && (terminationReason == E_TerminationReason::AbsoluteGap
            || terminationReason == E_TerminationReason::RelativeGap))
        status = E_NLPSolutionStatus::Optimal;
    else if(solver->hasPrimalSolution())
        status = E_NLPSolutionStatus::Feasible;
    else if(terminationReason == E_TerminationReason::InfeasibleProblem)
        status = E_NLPSolutionStatus::Infeasible;
    else if(terminationReason == E_TerminationReason::UnboundedProblem)
        status = E_NLPSolutionStatus::Unbounded;
    else if(terminationReason == E_TerminationReason::ObjectiveStagnation)
        status = E_NLPSolutionStatus::IterationLimit;
    else if(terminationReason == E_TerminationReason::NoDualCutsAdded)
        status = E_NLPSolutionStatus::IterationLimit;
    else if(terminationReason == E_TerminationReason::IterationLimit)
        status = E_NLPSolutionStatus::IterationLimit;
    else if(terminationReason == E_TerminationReason::TimeLimit)
        status = E_NLPSolutionStatus::TimeLimit;
    else
        status = E_NLPSolutionStatus::Error;

    return (status);
}

VectorDouble NLPSolverSHOT::getVariableLowerBounds() { return (relaxedProblem->getVariableLowerBounds()); }

VectorDouble NLPSolverSHOT::getVariableUpperBounds() { return (relaxedProblem->getVariableUpperBounds()); }

void NLPSolverSHOT::updateVariableLowerBound(int variableIndex, double bound)
{
    variableLowerBounds[variableIndex] = bound;
    relaxedProblem->setVariableLowerBound(variableIndex, bound);
}

void NLPSolverSHOT::updateVariableUpperBound(int variableIndex, double bound)
{
    variableUpperBounds[variableIndex] = bound;
    relaxedProblem->setVariableUpperBound(variableIndex, bound);
}

void NLPSolverSHOT::updateSetting(const std::string& name, int value)
{
    settingUpdates.emplace_back(name, value);
    solver->updateSetting(name, value);
}

void NLPSolverSHOT::updateSetting(const std::string& name, double value)
{
    settingUpdates.emplace_back(name, value);
    solver->updateSetting(name, value);
}

void NLPSolverSHOT::updateSetting(const std::string& name, bool value)
{
    settingUpdates.emplace_back(name, value);
    solver->updateSetting(name, value);
}

std::string NLPSolverSHOT::getSolverDescription()
{
    std::string description = "SHOT NLP solver";

    return (description);
}

} // namespace SHOT'