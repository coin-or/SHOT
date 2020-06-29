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

NLPSolverSHOT::NLPSolverSHOT(EnvironmentPtr envPtr, ProblemPtr source) : INLPSolver(envPtr) { sourceProblem = source; }

NLPSolverSHOT::~NLPSolverSHOT() = default;

void NLPSolverSHOT::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues)
{
    for(size_t i = 0; i < variableIndexes.size(); ++i)
    {
        // assert(variableIndexes[i] < gmoN(modelingObject));
        // gmoSetVarLOne(modelingObject, variableIndexes[i], variableValues[i]);
    }
}

void NLPSolverSHOT::clearStartingPoint() { }

void NLPSolverSHOT::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    fixedVariableIndexes = variableIndexes;
    fixedVariableValues = variableValues;
}

void NLPSolverSHOT::unfixVariables()
{
    /*for(int i = 0; i < relaxedProblem->properties.numberOfVariables; ++i)
    {
        auto bounds = sourceProblem->getVariableBounds();
        relaxedProblem->setVariableBounds(i, bounds.at(i).l(), bounds.at(i).u());
    }*/

    // solver->getEnvironment()->dualSolver->MIPSolver->unfixVariables();
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
    relaxedProblem = sourceProblem->createCopy(true);

    solver = std::make_unique<Solver>();

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Trace));
    solver->updateSetting("Debug.Enable", "Output", true);
    solver->updateSetting("CutStrategy", "Dual", 1);
    solver->updateSetting("Console.Iteration.Detail", "Output", 0);
    solver->updateSetting("ConstraintTolerance", "Termination", 1e-6);

    // Set the debug path for the subsolver
    auto mainDebugPath = env->settings->getSetting<std::string>("Debug.Path", "Output");
    fs::filesystem::path subproblemDebugPath(mainDebugPath);
    subproblemDebugPath /= ("SHOT_fixedNLP_" + std::to_string(env->results->getCurrentIteration()->iterationNumber));
    solver->updateSetting("Debug.Path", "Output", subproblemDebugPath.string());

    solver->setProblem(relaxedProblem);

    for(size_t i = 0; i < fixedVariableIndexes.size(); ++i)
        relaxedProblem->setVariableBounds(i, fixedVariableValues[i], fixedVariableValues[i]);

    solver->getEnvironment()->dualSolver->MIPSolver->fixVariables(fixedVariableIndexes, fixedVariableValues);

    std::cout << "---------------------------------\n";

    if(!solver->solveProblem())
        return E_NLPSolutionStatus::Error;

    std::cout << "---------------------------------\n";

    E_NLPSolutionStatus status;

    auto terminationReason = solver->getEnvironment()->results->terminationReason;

    if(terminationReason == E_TerminationReason::AbsoluteGap || terminationReason == E_TerminationReason::RelativeGap)
        status = E_NLPSolutionStatus::Optimal;
    else if(terminationReason == E_TerminationReason::ConstraintTolerance)
        status = E_NLPSolutionStatus::Feasible;
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
    /*relaxedProblem->setVariableLowerBound(variableIndex, bound);

    auto currentVariableBound
        = solver->getEnvironment()->dualSolver->MIPSolver->getCurrentVariableBounds(variableIndex);
    solver->getEnvironment()->dualSolver->MIPSolver->updateVariableBound(
        variableIndex, bound, std::get<1>(currentVariableBound));*/
}

void NLPSolverSHOT::updateVariableUpperBound(int variableIndex, double bound)
{
    /*relaxedProblem->setVariableLowerBound(variableIndex, bound);

    auto currentVariableBound
        = solver->getEnvironment()->dualSolver->MIPSolver->getCurrentVariableBounds(variableIndex);
    solver->getEnvironment()->dualSolver->MIPSolver->updateVariableBound(
        variableIndex, std::get<0>(currentVariableBound), bound);*/
}

std::string NLPSolverSHOT::getSolverDescription()
{
    std::string description = "SHOT NLP solver";

    return (description);
};
} // namespace SHOT