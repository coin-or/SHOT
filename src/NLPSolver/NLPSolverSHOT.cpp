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
#include "../MIPSolver/IMIPSolver.h"
#include "../Model/ObjectiveFunction.h"
#include "../Model/Problem.h"

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
    initializeMIPProblem();
}

NLPSolverSHOT::~NLPSolverSHOT() = default;

void NLPSolverSHOT::initializeMIPProblem()
{
    solver = std::make_shared<Solver>();

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
    solver->updateSetting(
        "Termination.IterationLimit", env->settings->getSetting<int>("Primal.FixedInteger.IterationLimit"));

    solver->updateSetting("Termination.DualStagnation.IterationLimit", 20);

    if(env->settings->getSetting<bool>("Subsolver.SHOT.ReuseHyperplanes.Use"))
        solver->updateSetting("Dual.HyperplaneCuts.SaveHyperplanePoints", true);

    solver->updateSetting(
        "Model.BoundTightening.FeasibilityBased.Use", env->settings->getSetting<bool>("Subsolver.SHOT.UseFBBT"));

    // Put more emphasis on numerical stability in the LP solver
#ifdef HAS_CPLEX
    solver->updateSetting("Subsolver.Cplex.NumericalEmphasis", 1);
#endif

    // Put more emphasis on numerical stability in the LP solver
#ifdef HAS_GUROBI
    solver->updateSetting("Subsolver.Gurobi.NumericFocus", 3);
#endif

    // Set the debug path for the subsolver
    auto mainDebugPath = env->settings->getSetting<std::string>("Output.Debug.Path");
    fs::filesystem::path subproblemDebugPath(mainDebugPath);
    // subproblemDebugPath /= ("SHOT_fixedNLP_" + std::to_string(env->results->getCurrentIteration()->iterationNumber));
    subproblemDebugPath /= ("SHOT_fixedNLP");
    solver->updateSetting("Output.Debug.Path", subproblemDebugPath.string());

    relaxedProblem = sourceProblem->createCopy(solver->getEnvironment(), true, false, false);
    solver->setProblem(relaxedProblem, relaxedProblem);
}

void NLPSolverSHOT::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues) { }

void NLPSolverSHOT::clearStartingPoint() { }

void NLPSolverSHOT::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    fixedVariableIndexes = variableIndexes;
    fixedVariableValues = variableValues;
}

void NLPSolverSHOT::unfixVariables()
{
    for(auto& VAR : sourceProblem->allVariables)
    {
        relaxedProblem->setVariableBounds(VAR->index, VAR->lowerBound, VAR->upperBound);
        VAR->properties.hasLowerBoundBeenTightened = false;
        VAR->properties.hasUpperBoundBeenTightened = false;
    }

    for(auto& VAR : relaxedProblem->allVariables)
        solver->getEnvironment()->dualSolver->MIPSolver->updateVariableBound(
            VAR->index, VAR->lowerBound, VAR->upperBound);

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
            VAR->index, VAR->lowerBound, VAR->upperBound);

    // Setting the cutoff value from currently best known solution
    if(env->dualSolver->cutOffToUse != SHOT_DBL_MAX)
    {
        solver->updateSetting("Dual.MIP.CutOff.InitialValue", env->dualSolver->cutOffToUse);
        solver->updateSetting("Dual.MIP.CutOff.UseInitialValue", true);
    }

    if(!problemInfoPrinted)
    {
        solver->getEnvironment()->report->outputProblemInstanceReport();
        solver->getEnvironment()->report->outputOptionsReport();
        problemInfoPrinted = true;
    }

    if(!solver->solveProblem())
        return E_NLPSolutionStatus::Error;

    solver->getEnvironment()->report->outputSolutionReport();

    solver->getEnvironment()->output->outputInfo("");

    int hyperplaneCounter = 0;

    if(env->settings->getSetting<bool>("Subsolver.SHOT.ReuseHyperplanes.Use"))
    {
        int numHyperplanesToCopy = solver->getEnvironment()->dualSolver->generatedHyperplanes.size()
            * env->settings->getSetting<double>("Subsolver.SHOT.ReuseHyperplanes.Fraction");

        for(auto& HP : solver->getEnvironment()->dualSolver->generatedHyperplanes)
        {
            if(hyperplaneCounter >= numHyperplanesToCopy)
                break;

            if(auto constraintHP = std::dynamic_pointer_cast<ConstraintHyperplane>(HP))
            {
                std::vector<double> tmpSolPt(constraintHP->generatedPoint.begin(),
                    constraintHP->generatedPoint.begin() + env->problem->properties.numberOfVariables);

                if((int)tmpSolPt.size() < env->reformulatedProblem->properties.numberOfVariables)
                    env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpSolPt);

                assert(tmpSolPt.size() == env->reformulatedProblem->properties.numberOfVariables);

                auto hyperplane = std::make_shared<ConstraintHyperplane>();
                hyperplane->generatedPoint = tmpSolPt;
                hyperplane->sourceConstraint = std::dynamic_pointer_cast<NumericConstraint>(
                    env->reformulatedProblem->getConstraint(constraintHP->sourceConstraint->index));
                hyperplane->isGlobal = HP->sourceHyperplane->isGlobal;
                hyperplane->source = E_HyperplaneSource::PrimalSolutionSearch;

                env->dualSolver->addHyperplane(hyperplane);
                hyperplaneCounter++;
            }
            else if(auto objectiveHP = std::dynamic_pointer_cast<ObjectiveHyperplane>(HP))
            {
                std::vector<double> tmpSolPt(objectiveHP->generatedPoint.begin(),
                    objectiveHP->generatedPoint.begin() + env->problem->properties.numberOfVariables);

                if((int)tmpSolPt.size() < env->reformulatedProblem->properties.numberOfVariables)
                    env->reformulatedProblem->augmentAuxiliaryVariableValues(tmpSolPt);

                assert(tmpSolPt.size() == env->reformulatedProblem->properties.numberOfVariables);

                ObjectiveHyperplanePtr hyperplane;
                hyperplane->generatedPoint = tmpSolPt;
                hyperplane->objectiveFunctionValue = sourceProblem->objectiveFunction->calculateValue(tmpSolPt);
                hyperplane->isGlobal = sourceProblem->objectiveFunction->properties.convexity <= E_Convexity::Convex;
                hyperplane->source = E_HyperplaneSource::PrimalSolutionSearch;

                env->dualSolver->addHyperplane(hyperplane);
                hyperplaneCounter++;
            }
        }

        solver->getEnvironment()->dualSolver->generatedHyperplanes.clear();

        solver->getEnvironment()->output->outputInfo(
            fmt::format(" Added {} hyperplanes generated by SHOT primal NLP solver.", hyperplaneCounter));
    }

    E_NLPSolutionStatus status;

    auto terminationReason = solver->getEnvironment()->results->terminationReason;

    if(terminationReason == E_TerminationReason::AbsoluteGap || terminationReason == E_TerminationReason::RelativeGap)
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
    relaxedProblem->setVariableLowerBound(variableIndex, bound);
}

void NLPSolverSHOT::updateVariableUpperBound(int variableIndex, double bound)
{
    relaxedProblem->setVariableUpperBound(variableIndex, bound);
}

std::string NLPSolverSHOT::getSolverDescription()
{
    std::string description = "SHOT NLP solver";

    return (description);
}

} // namespace SHOT'