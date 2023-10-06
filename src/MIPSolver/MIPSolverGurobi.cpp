/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverGurobi.h"

#include "../DualSolver.h"
#include "../EventHandler.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

namespace SHOT
{

MIPSolverGurobi::MIPSolverGurobi()
{
    // Should not be called
}

MIPSolverGurobi::MIPSolverGurobi(EnvironmentPtr envPtr)
{
    env = envPtr;
    gurobiCallback = std::make_unique<GurobiCallbackMultiTree>(env);
}

MIPSolverGurobi::~MIPSolverGurobi()
{
    objectiveLinearExpression.clear();
    objectiveQuadraticExpression.clear();
    constraintLinearExpression.clear();
    constraintQuadraticExpression.clear();
}

bool MIPSolverGurobi::initializeProblem()
{
    discreteVariablesActivated = true;

    if(alreadyInitialized)
    {
        std::shared_ptr<GRBModel> gurobiModel;
    }
    else
    {

        prevSolutionLimit = 1;
        alreadyInitialized = true;
    }

    try
    {
        GRBEnv env = GRBEnv();
        gurobiModel = std::make_shared<GRBModel>(env);
    }
    catch(GRBException& e)
    {
        env->output->outputError(" Error when initializing Gurobi:", e.getMessage());
        return (false);
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();

    return (true);
}

bool MIPSolverGurobi::addVariable(
    std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound)
{
    if(lowerBound < -getUnboundedVariableBoundValue())
        lowerBound = -getUnboundedVariableBoundValue();

    if(upperBound > getUnboundedVariableBoundValue())
        upperBound = getUnboundedVariableBoundValue();

    try
    {
        switch(type)
        {
        case E_VariableType::Real:
            gurobiModel->addVar(lowerBound, upperBound, 0.0, GRB_CONTINUOUS, name);
            break;

        case E_VariableType::Integer:
            isProblemDiscrete = true;
            gurobiModel->addVar(lowerBound, upperBound, 0.0, GRB_INTEGER, name);
            break;

        case E_VariableType::Binary:
            isProblemDiscrete = true;
            gurobiModel->addVar(lowerBound, upperBound, 0.0, GRB_BINARY, name);
            break;

        case E_VariableType::Semicontinuous:
            isProblemDiscrete = true;
            if(semiBound < 0.0)
                upperBound = semiBound;
            else
                lowerBound = semiBound;
            gurobiModel->addVar(lowerBound, upperBound, 0.0, GRB_SEMICONT, name);
            break;

        case E_VariableType::Semiinteger:
            isProblemDiscrete = true;
            if(semiBound < 0.0)
                upperBound = semiBound;
            else
                lowerBound = semiBound;
            gurobiModel->addVar(lowerBound, upperBound, 0.0, GRB_SEMIINT, name);
            break;

        default:
            break;
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi exception caught when adding variable to model: ", e.getMessage());
        return (false);
    }

    variableTypes.push_back(type);
    variableNames.push_back(name);
    variableLowerBounds.push_back(lowerBound);
    variableUpperBounds.push_back(upperBound);
    numberOfVariables++;
    return (true);
}

bool MIPSolverGurobi::initializeObjective()
{
    try
    {
        gurobiModel->update(); // Needed to make sure variables are available
        objectiveQuadraticExpression = GRBQuadExpr(0);
        objectiveLinearExpression = GRBLinExpr(0);
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Gurobi exception caught when initializing objective function: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverGurobi::addLinearTermToObjective(double coefficient, int variableIndex)
{
    try
    {
        objectiveLinearExpression += coefficient * gurobiModel->getVar(variableIndex);
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Gurobi exception caught when adding linear term to objective: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverGurobi::addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
    try
    {
        objectiveQuadraticExpression
            += coefficient * gurobiModel->getVar(firstVariableIndex) * gurobiModel->getVar(secondVariableIndex);
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Gurobi exception caught when adding quadratic term to objective: ", e.getMessage());
        return (false);
    }

    hasQuadraticObjective = true;

    return (true);
}

bool MIPSolverGurobi::finalizeObjective(bool isMinimize, double constant)
{
    try
    {
        if(constant != 0.0)
            objectiveLinearExpression += constant;

        if(isMinimize)
        {
            gurobiModel->setObjective(objectiveLinearExpression + objectiveQuadraticExpression, GRB_MINIMIZE);
            isMinimizationProblem = true;
        }
        else
        {
            gurobiModel->setObjective(objectiveLinearExpression + objectiveQuadraticExpression, GRB_MAXIMIZE);
            isMinimizationProblem = false;
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Gurobi exception caught when adding objective function to model: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverGurobi::initializeConstraint()
{
    try
    {
        constraintQuadraticExpression = GRBQuadExpr(0);
        constraintLinearExpression = GRBLinExpr(0);
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi exception caught when initializing constraint: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverGurobi::addLinearTermToConstraint(double coefficient, int variableIndex)
{
    try
    {
        constraintLinearExpression += coefficient * gurobiModel->getVar(variableIndex);
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Gurobi exception caught when adding linear term to constraint: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverGurobi::addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
    try
    {
        constraintQuadraticExpression
            += coefficient * gurobiModel->getVar(firstVariableIndex) * gurobiModel->getVar(secondVariableIndex);
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Gurobi exception caught when adding quadratic term to constraint: ", e.getMessage());
        return (false);
    }

    hasQudraticConstraint = true;

    return (true);
}

bool MIPSolverGurobi::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
{
    try
    {
        if(constant != 0.0)
            constraintLinearExpression += constant;

        if(constraintQuadraticExpression.size() == 0)
        {
            if(valueLHS == valueRHS)
            {
                gurobiModel->addConstr(constraintLinearExpression == valueRHS, name);
            }
            else if(valueLHS < valueRHS)
            {
                if(valueLHS > SHOT_DBL_MIN)
                    gurobiModel->addConstr(valueLHS <= constraintLinearExpression, name + "_a");

                if(valueRHS < SHOT_DBL_MAX)
                    gurobiModel->addConstr(constraintLinearExpression <= valueRHS, name + "_b");
            }
            else
            {
                if(valueLHS < SHOT_DBL_MAX)
                    gurobiModel->addConstr(valueLHS >= constraintLinearExpression, name + "_a");

                if(valueRHS > SHOT_DBL_MIN)
                    gurobiModel->addConstr(constraintLinearExpression >= valueRHS, name + "_b");
            }
        }
        else
        {
            if(valueLHS == valueRHS)
            {
                gurobiModel->addQConstr(constraintLinearExpression + constraintQuadraticExpression == valueRHS, name);
            }
            else if(valueLHS < valueRHS)
            {
                if(valueLHS > SHOT_DBL_MIN)
                    gurobiModel->addQConstr(
                        valueLHS <= constraintLinearExpression + constraintQuadraticExpression, name + "_a");

                if(valueRHS < SHOT_DBL_MAX)
                    gurobiModel->addQConstr(
                        constraintLinearExpression + constraintQuadraticExpression <= valueRHS, name + "_b");
            }
            else
            {

                if(valueLHS < SHOT_DBL_MAX)
                    gurobiModel->addQConstr(
                        valueLHS >= constraintLinearExpression + constraintQuadraticExpression, name + "_a");

                if(valueRHS > SHOT_DBL_MIN)
                    gurobiModel->addQConstr(
                        constraintLinearExpression + constraintQuadraticExpression >= valueRHS, name + "_b");
            }
        }

        if(constraintQuadraticExpression.size() == 0)
            allowRepairOfConstraint.push_back(false);
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi exception caught when adding constraint to model: ", e.getMessage());
        return (false);
    }

    numberOfConstraints++;
    return (true);
}

bool MIPSolverGurobi::finalizeProblem()
{
    try
    {
        if(env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
        {
            int setSolLimit;
            bool discreteVariablesActivated = getDiscreteVariableStatus();

            if(env->results->getNumberOfIterations() > 0)
            {
                setSolLimit = env->results->getCurrentIteration()->usedMIPSolutionLimit;
                discreteVariablesActivated = env->results->getCurrentIteration()->isMIP();
            }
            else
            {
                setSolLimit = env->settings->getSetting<int>("MIP.SolutionLimit.Initial", "Dual");
            }

            setSolutionLimit(setSolLimit);

            if(!discreteVariablesActivated)
            {
                activateDiscreteVariables(false);
            }
        }

        modelUpdated = true;
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi exception caught when finalizing model", e.getMessage());
        return (false);
    }

    return (true);
}

void MIPSolverGurobi::initializeSolverSettings()
{
    try
    {
        // Console output controlled in callback, but need to disable it so messages won't appear twice
        gurobiModel->set(GRB_IntParam_LogToConsole, 0);

        // Set termination tolerances
        gurobiModel->set(
            GRB_DoubleParam_MIPGap, env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") / 1.0);
        gurobiModel->set(
            GRB_DoubleParam_MIPGapAbs, env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);
        gurobiModel->set(
            GRB_DoubleParam_FeasibilityTol, env->settings->getSetting<double>("Tolerance.LinearConstraint", "Primal"));
        gurobiModel->set(GRB_DoubleParam_IntFeasTol, env->settings->getSetting<double>("Tolerance.Integer", "Primal"));
        gurobiModel->set(
            GRB_DoubleParam_OptimalityTol, env->settings->getSetting<double>("MIP.OptimalityTolerance", "Dual"));

        // Add a user-provided node limit
        if(auto nodeLimit = env->settings->getSetting<double>("MIP.NodeLimit", "Dual"); nodeLimit > 0)
            gurobiModel->set(GRB_DoubleParam_NodeLimit, nodeLimit);

        // Set solution pool settings
        gurobiModel->set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
        gurobiModel->set(
            GRB_IntParam_SolutionNumber, env->settings->getSetting<int>("MIP.SolutionPool.Capacity", "Dual") + 1);
        gurobiModel->set(
            GRB_IntParam_PoolSearchMode, env->settings->getSetting<int>("Gurobi.PoolSearchMode", "Subsolver"));
        gurobiModel->set(
            GRB_IntParam_PoolSolutions, env->settings->getSetting<int>("Gurobi.PoolSolutions", "Subsolver"));

        // Set solver emphasis
        gurobiModel->set(GRB_IntParam_NumericFocus, env->settings->getSetting<int>("Gurobi.NumericFocus", "Subsolver"));

        // Set parameters for quadratics
        gurobiModel->set(GRB_DoubleParam_PSDTol,
            env->settings->getSetting<double>("Convexity.Quadratics.EigenValueTolerance", "Model"));

#if GRB_VERSION_MAJOR >= 9
        // Supports nonconvex MIQCQP
        if(static_cast<ES_QuadraticProblemStrategy>(
               env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"))
            == ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained)
        {
            gurobiModel->set(GRB_IntParam_NonConvex, 2);
        }
#endif

        // Set various solver specific MIP settings
        gurobiModel->set(GRB_IntParam_ScaleFlag, env->settings->getSetting<int>("Gurobi.ScaleFlag", "Subsolver"));
        gurobiModel->set(GRB_IntParam_MIPFocus, env->settings->getSetting<int>("Gurobi.MIPFocus", "Subsolver"));
        gurobiModel->set(
            GRB_DoubleParam_Heuristics, env->settings->getSetting<double>("Gurobi.Heuristics", "Subsolver"));

        // Set number of threads
        gurobiModel->set(GRB_IntParam_Threads, env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual"));
    }
    catch(GRBException& e)
    {
        env->output->outputError(" Error when initializing Gurobi parameters: ", e.getMessage());
    }
}

int MIPSolverGurobi::addLinearConstraint(
    const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
{
    try
    {
        int numConstraintsBefore = gurobiModel->get(GRB_IntAttr_NumConstrs);

        auto expr = std::make_unique<GRBLinExpr>(0.0);

        for(auto E : elements)
        {
            auto variable = gurobiModel->getVar(E.first);

            if(std::abs(E.second) > 1e-13) // Gurobi might crash otherwise
                *expr.get() = *expr.get() + E.second * variable;
        }

        if(isGreaterThan)
        {
            gurobiModel->addConstr(-constant >= *expr, name);
        }
        else
        {
            gurobiModel->addConstr(*expr <= -constant, name);
        }

        gurobiModel->update();

        if(gurobiModel->get(GRB_IntAttr_NumConstrs) > numConstraintsBefore)
        {
            allowRepairOfConstraint.push_back(allowRepair);
        }
        else
        {
            env->output->outputInfo("        Hyperplane not added by Gurobi");
            return (-1);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when adding linear constraint", e.getMessage());
        return (-1);
    }

    return (gurobiModel->get(GRB_IntAttr_NumConstrs) - 1);
}

bool MIPSolverGurobi::addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights)
{
    try
    {
        std::vector<GRBVar> variables;

        for(auto I : variableIndexes)
            variables.push_back(gurobiModel->getVar(I));

        if(variableWeights.size() == 0)
        {
            variableWeights.resize(variableIndexes.size());

            for(size_t i = 0; i < variableIndexes.size(); i++)
                variableWeights[i] = i;
        }

        assert(variableWeights.size() == variableIndexes.size());

        gurobiModel->addSOS(&variables[0], &variableWeights[0], variables.size(),
            (type == E_SOSType::One) ? GRB_SOS_TYPE1 : GRB_SOS_TYPE2);
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when adding special ordered set constraint", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverGurobi::createIntegerCut(IntegerCut& integerCut)
{
    bool allowIntegerCutRepair = env->settings->getSetting<bool>("MIP.InfeasibilityRepair.IntegerCuts", "Dual");

    try
    {
        int numConstraintsBefore = gurobiModel->get(GRB_IntAttr_NumConstrs);
        GRBLinExpr expr = 0;
        size_t index = 0;

        // Verify that no integer values are outside of variable bounds
        for(size_t i = 0; i < integerCut.variableIndexes.size(); i++)
        {
            auto VAR = env->reformulatedProblem->getVariable(integerCut.variableIndexes[i]);
            int variableValue = integerCut.variableValues[i];

            if(variableValue < VAR->lowerBound || variableValue > VAR->upperBound)
                return (false);
        }

        for(auto& I : integerCut.variableIndexes)
        {
            auto VAR = env->reformulatedProblem->getVariable(I);
            int variableValue = integerCut.variableValues[index];
            auto variable = gurobiModel->getVar(I);

            assert(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                || VAR->properties.type == E_VariableType::Semiinteger);

            if(variableValue == VAR->upperBound)
            {
                expr += (variableValue - variable);
            }
            else if(variableValue == VAR->lowerBound)
            {
                expr += variable;
            }
            else
            {
                numberOfVariables += 2;

                auto w = gurobiModel->addVar(0, getUnboundedVariableBoundValue(), 0.0, GRB_CONTINUOUS,
                    fmt::format("wIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));
                auto v = gurobiModel->addVar(
                    0, 1, 0.0, GRB_BINARY, fmt::format("vIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));
                gurobiModel->update();

                expr += 1.0 * w;

                double M1 = 2 * (variableValue - VAR->lowerBound);
                double M2 = 2 * (VAR->upperBound - variableValue);

                int tmpNumConstraints = gurobiModel->get(GRB_IntAttr_NumConstrs);
                gurobiModel->addConstr(-w <= variable - variableValue,
                    fmt::format("IC{}_{}_1a", env->solutionStatistics.numberOfIntegerCuts, index));
                gurobiModel->update();

                if(gurobiModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
                {
                    integerCuts.push_back(numConstraintsBefore + index);
                    allowRepairOfConstraint.push_back(false);
                }

                tmpNumConstraints = gurobiModel->get(GRB_IntAttr_NumConstrs);
                gurobiModel->addConstr(variable - variableValue <= w,
                    fmt::format("IC{}_{}_1b", env->solutionStatistics.numberOfIntegerCuts, index));
                gurobiModel->update();

                if(gurobiModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
                {
                    integerCuts.push_back(numConstraintsBefore + index);
                    allowRepairOfConstraint.push_back(false);
                }

                tmpNumConstraints = gurobiModel->get(GRB_IntAttr_NumConstrs);
                gurobiModel->addConstr(w <= variable - variableValue + M1 * (1 - v),
                    fmt::format("IC{}_{}_2", env->solutionStatistics.numberOfIntegerCuts, index));
                gurobiModel->update();

                if(gurobiModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
                {
                    integerCuts.push_back(numConstraintsBefore + index);
                    allowRepairOfConstraint.push_back(false);
                }

                tmpNumConstraints = gurobiModel->get(GRB_IntAttr_NumConstrs);
                gurobiModel->addConstr(w <= variableValue - variable + M2 * v,
                    fmt::format("IC{}_{}_3", env->solutionStatistics.numberOfIntegerCuts, index));
                gurobiModel->update();

                if(gurobiModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
                {
                    integerCuts.push_back(numConstraintsBefore + index);
                    allowRepairOfConstraint.push_back(false);
                }
            }

            index++;
        }

        int tmpNumConstraints = gurobiModel->get(GRB_IntAttr_NumConstrs);
        gurobiModel->addConstr(expr >= 1, fmt::format("IC{}_4", env->solutionStatistics.numberOfIntegerCuts));
        gurobiModel->update();

        if(gurobiModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
        {
            integerCuts.push_back(numConstraintsBefore + index);
            allowRepairOfConstraint.push_back(allowIntegerCutRepair);
        }

        gurobiModel->update();

        auto addedConstraints = gurobiModel->get(GRB_IntAttr_NumConstrs) - numConstraintsBefore;

        if(addedConstraints == 0)
        {
            env->output->outputInfo("        Integer cut not added by Gurobi");
            return (false);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when adding integer cut", e.getMessage());
        return (false);
    }

    return (true);
}

VectorDouble MIPSolverGurobi::getVariableSolution(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
    VectorDouble solution(numVar);

    try
    {
        if(isMIP && solIdx > 0)
        {
            gurobiModel->set(GRB_IntParam_SolutionNumber, solIdx);

            for(int i = 0; i < numVar; i++)
            {
                GRBVar tmpVar = gurobiModel->getVar(i);
                solution.at(i) = tmpVar.get(GRB_DoubleAttr_Xn);
            }
        }
        else
        {
            for(int i = 0; i < numVar; i++)
            {
                GRBVar tmpVar = gurobiModel->getVar(i);
                solution.at(i) = tmpVar.get(GRB_DoubleAttr_X);
            }
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Error when reading solution with index " + std::to_string(solIdx), e.getMessage());
    }

    return (solution);
}

int MIPSolverGurobi::getNumberOfSolutions()
{
    int numSols = 0;

    numSols = gurobiModel->get(GRB_IntAttr_SolCount);

    return (numSols);
}

void MIPSolverGurobi::activateDiscreteVariables(bool activate)
{

    if(env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0
        || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
        return;

    if(activate)
    {
        env->output->outputDebug("        Activating MIP strategy.");

        for(int i = 0; i < numberOfVariables; i++)
        {
            assert(variableTypes.at(i) != E_VariableType::Semicontinuous
                && variableTypes.at(i) != E_VariableType::Semiinteger);

            if(variableTypes.at(i) == E_VariableType::Integer)
            {
                GRBVar tmpVar = gurobiModel->getVar(i);

                tmpVar.set(GRB_CharAttr_VType, 'I');
            }
            else if(variableTypes.at(i) == E_VariableType::Binary)
            {
                GRBVar tmpVar = gurobiModel->getVar(i);
                tmpVar.set(GRB_CharAttr_VType, 'B');
            }
        }

        discreteVariablesActivated = true;
    }
    else
    {
        env->output->outputDebug("        Activating LP strategy.");

        for(int i = 0; i < numberOfVariables; i++)
        {
            assert(variableTypes.at(i) != E_VariableType::Semicontinuous
                && variableTypes.at(i) != E_VariableType::Semiinteger);

            if(variableTypes.at(i) == E_VariableType::Integer || variableTypes.at(i) == E_VariableType::Binary)
            {
                GRBVar tmpVar = gurobiModel->getVar(i);

                tmpVar.set(GRB_CharAttr_VType, 'C');
            }
        }

        discreteVariablesActivated = false;
    }

    modelUpdated = true;
}

E_ProblemSolutionStatus MIPSolverGurobi::getSolutionStatus()
{
    E_ProblemSolutionStatus MIPSolutionStatus;

    int status = gurobiModel->get(GRB_IntAttr_Status);

    if(status == GRB_OPTIMAL)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
    }
    else if(status == GRB_INFEASIBLE)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if(status == GRB_INF_OR_UNBD)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(status == GRB_UNBOUNDED)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(status == GRB_ITERATION_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(status == GRB_NODE_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(status == GRB_TIME_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
    }
    else if(status == GRB_SOLUTION_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
    }
    else if(status == GRB_INTERRUPTED)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
    }
    else if(status == GRB_NUMERIC)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Numeric;
    }
    else if(status == GRB_CUTOFF)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if(status == GRB_SUBOPTIMAL)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
    }
    else if(status == GRB_LOADED)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else
    {
        env->output->outputError("        MIP solver return status " + std::to_string(status));
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverGurobi::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        if(modelUpdated)
        {
            gurobiModel->update();
            modelUpdated = false;
        }
        gurobiModel->setCallback(gurobiCallback.get());
        gurobiModel->optimize();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    // To find a feasible point for an unbounded dual problem  and not when solving the minimax-problem
    if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded && env->results->getNumberOfIterations() > 0)
    {
        std::vector<PairIndexValue> originalObjectiveCoefficients;
        bool problemUpdated = false;

        if((env->reformulatedProblem->objectiveFunction->properties.classification
                   == E_ObjectiveFunctionClassification::Linear
               && std::dynamic_pointer_cast<LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->isDualUnbounded())
            || (env->reformulatedProblem->objectiveFunction->properties.classification
                    == E_ObjectiveFunctionClassification::Quadratic
                && std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                       ->isDualUnbounded()))
        {
            for(auto& V : env->reformulatedProblem->allVariables)
            {
                if(!V->properties.inObjectiveFunction)
                    continue;

                if(V->isDualUnbounded())
                {
                    // Temporarily remove unbounded terms from objective
                    originalObjectiveCoefficients.emplace_back(
                        V->index, gurobiModel->getVar(V->index).get(GRB_DoubleAttr_Obj));

                    gurobiModel->getVar(V->index).set(GRB_DoubleAttr_Obj, 0.0);
                    problemUpdated = true;
                }
            }
        }
        else if(env->reformulatedProblem->objectiveFunction->properties.classification
                >= E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear
            && hasDualAuxiliaryObjectiveVariable())
        {
            // The auxiliary variable in the dual problem is unbounded
            updateVariableBound(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue() / 1.1,
                getUnboundedVariableBoundValue() / 1.1);
            problemUpdated = true;
        }

        if(problemUpdated)
        {
            gurobiModel->update();
            gurobiModel->setCallback(gurobiCallback.get());
            gurobiModel->optimize();

            MIPSolutionStatus = getSolutionStatus();

            for(auto& P : originalObjectiveCoefficients)
                gurobiModel->getVar(P.index).set(GRB_DoubleAttr_Obj, P.value);

            gurobiModel->update();

            if(env->results->iterations.size() > 0) // Might not have iterations if we are using the minimax solver
                env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }
    }

    return (MIPSolutionStatus);
}

bool MIPSolverGurobi::repairInfeasibility()
{
    if(env->dualSolver->generatedHyperplanes.size() == 0)
        return (false);

    try
    {
        gurobiModel->update();
        auto feasModel = GRBModel(*gurobiModel);

        // Gurobi copies over the cutoff from the original model
        if(isMinimizationProblem)
            feasModel.set(GRB_DoubleParam_Cutoff, SHOT_DBL_MAX);
        else
            feasModel.set(GRB_DoubleParam_Cutoff, SHOT_DBL_MIN);

        int numOrigConstraints = env->reformulatedProblem->properties.numberOfLinearConstraints;
        int numOrigVariables = gurobiModel->get(GRB_IntAttr_NumVars);
        int numCurrConstraints = feasModel.get(GRB_IntAttr_NumConstrs);

        std::vector<GRBConstr> repairConstraints;
        std::vector<GRBConstr> originalConstraints;
        VectorDouble relaxParameters;
        int numConstraintsToRepair = 0;

        for(int i = numOrigConstraints; i < numCurrConstraints; i++)
        {
            if(allowRepairOfConstraint[i])
            {
                repairConstraints.push_back(feasModel.getConstr(i));
                originalConstraints.push_back(gurobiModel->getConstr(i));
                relaxParameters.push_back(1 / (((double)i - numOrigConstraints) + 1.0));
                numConstraintsToRepair++;
            }
        }

        // Saves the relaxation weights to a file
        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            VectorString constraints(relaxParameters.size());

            for(size_t i = 0; i < relaxParameters.size(); i++)
            {
                std::ostringstream expression;
                constraints[i] = repairConstraints[i].get(GRB_StringAttr_ConstrName);
            }

            auto filename = fmt::format("{}/dualiter{}_infeasrelaxweights.txt",
                env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            Utilities::saveVariablePointVectorToFile(relaxParameters, constraints, filename);
        }

        // Gurobi modifies the value when running feasModel.optimize()
        int numConstraintsToRepairOrig = numConstraintsToRepair;

        if(feasModel.feasRelax(GRB_FEASRELAX_LINEAR, false, 0, nullptr, nullptr, nullptr, numConstraintsToRepair,
               &repairConstraints[0], &relaxParameters[0])
            < 0)
        {
            env->output->outputDebug("        Could not repair the infeasible dual problem.");
            return (false);
        }

        feasModel.optimize();

        // Saves the relaxation model to file
        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            auto filename = fmt::format("{}/dualiter{}_infeasrelax.lp",
                env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            try
            {
                feasModel.write(filename);
            }
            catch(GRBException& e)
            {
                env->output->outputError("        Error when saving model to file", e.getMessage());
            }
        }

        int status = feasModel.get(GRB_IntAttr_Status);

        if(status != GRB_OPTIMAL)
        {
            env->output->outputDebug("        Could not repair the infeasible dual problem.");
            return (false);
        }

        int numRepairs = 0;

        for(int i = 0; i < numConstraintsToRepairOrig; i++)
        {
            auto variable = feasModel.getVar(numOrigVariables + i);
            double slackValue = variable.get(GRB_DoubleAttr_X);

            if(slackValue == 0.0)
                continue;

            auto constraint = originalConstraints.at(i);
            double oldRHS = constraint.get(GRB_DoubleAttr_RHS);
            constraint.set(GRB_DoubleAttr_RHS, oldRHS + 1.5 * slackValue);

            numRepairs++;

            env->output->outputDebug("        Constraint: " + constraint.get(GRB_StringAttr_ConstrName)
                + " repaired with infeasibility = " + std::to_string(1.5 * slackValue));
        }

        env->results->getCurrentIteration()->numberOfInfeasibilityRepairedConstraints = numRepairs;

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            auto filename = fmt::format("{}/dualiter{}_infeasrelax.lp",
                env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            writeProblemToFile(filename);
        }

        if(numRepairs == 0)
        {
            env->output->outputDebug("        Could not repair the infeasible dual problem.");
            return (false);
        }

        env->output->outputDebug("        Number of constraints modified: " + std::to_string(numRepairs));

        return (true);
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when trying to repair infeasibility",
            e.getMessage() + " (" + std::to_string(e.getErrorCode()) + ")");
    }

    return (false);
}

int MIPSolverGurobi::increaseSolutionLimit(int increment)
{
    gurobiModel->set(GRB_IntParam_SolutionLimit, gurobiModel->get(GRB_IntParam_SolutionLimit) + increment);

    return (gurobiModel->get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobi::setSolutionLimit(long limit)
{
    if(limit > GRB_MAXINT)
        gurobiModel->set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
    else
        gurobiModel->set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobi::getSolutionLimit() { return (gurobiModel->get(GRB_IntParam_SolutionLimit)); }

void MIPSolverGurobi::setTimeLimit(double seconds)
{
    try
    {
        if(seconds > 0)
        {
            gurobiModel->set(GRB_DoubleParam_TimeLimit, seconds);
        }
        else
        {
            gurobiModel->set(GRB_DoubleParam_TimeLimit, 0.00001);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when setting time limit", e.getMessage());
    }
}

void MIPSolverGurobi::setCutOff(double cutOff)
{
    if(std::abs(cutOff) > 1e20)
        return;

    try
    {
        // Gurobi has problems if not an epsilon value is added to the cutoff...

        double cutOffTol = env->settings->getSetting<double>("MIP.CutOff.Tolerance", "Dual");

        if(isMinimizationProblem)
        {
            gurobiModel->set(GRB_DoubleParam_Cutoff, cutOff + cutOffTol);
            env->output->outputDebug(
                fmt::format("        Setting cutoff value to  {} for maximization.", cutOff + cutOffTol));
        }
        else
        {
            gurobiModel->set(GRB_DoubleParam_Cutoff, cutOff - cutOffTol);
            env->output->outputDebug(
                fmt::format("        Setting cutoff value to  {} for maximization.", cutOff - cutOffTol));
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when setting cut off value", e.getMessage());
    }
}

void MIPSolverGurobi::setCutOffAsConstraint(double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    try
    {
        if(!cutOffConstraintDefined)
        {
            if(env->reformulatedProblem->objectiveFunction->properties.isMaximize)
            {
                gurobiModel->addConstr(-objectiveLinearExpression <= -cutOff, "CUTOFF_C");

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for maximization.");
            }
            else
            {
                gurobiModel->addConstr(objectiveLinearExpression <= cutOff, "CUTOFF_C");

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
            }

            allowRepairOfConstraint.push_back(false);
            gurobiModel->update();
            modelUpdated = false;

            cutOffConstraintDefined = true;
            cutOffConstraintIndex = gurobiModel->get(GRB_IntAttr_NumConstrs) - 1;
        }
        else
        {
            auto constraint = gurobiModel->getConstr(cutOffConstraintIndex);

            if(env->reformulatedProblem->objectiveFunction->properties.isMaximize)
            {

                if(hasDualAuxiliaryObjectiveVariable())
                    constraint.set(GRB_DoubleAttr_RHS, -cutOff);
                else
                    constraint.set(
                        GRB_DoubleAttr_RHS, -(cutOff - env->reformulatedProblem->objectiveFunction->constant));

                env->output->outputDebug(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
            }
            else
            {

                if(hasDualAuxiliaryObjectiveVariable())
                    constraint.set(GRB_DoubleAttr_RHS, cutOff);
                else
                    constraint.set(GRB_DoubleAttr_RHS, cutOff - env->reformulatedProblem->objectiveFunction->constant);

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
            }

            modelUpdated = true;
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when setting cut off value through constraint", e.getMessage());
    }
}

void MIPSolverGurobi::addMIPStart(VectorDouble point)
{
    try
    {
        VectorDouble startVal;

        if((int)point.size() < env->reformulatedProblem->properties.numberOfVariables)
            env->reformulatedProblem->augmentAuxiliaryVariableValues(point);

        assert(env->reformulatedProblem->properties.numberOfVariables == point.size());

        if(this->hasDualAuxiliaryObjectiveVariable())
            point.push_back(env->reformulatedProblem->objectiveFunction->calculateValue(point));

        assert(variableNames.size() == point.size());

        for(double P : point)
            startVal.push_back(P);

        for(size_t i = 0; i < startVal.size(); i++)
        {
            GRBVar tmpVar = gurobiModel->getVar(i);
            tmpVar.set(GRB_DoubleAttr_Start, startVal.at(i));
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when adding MIP starting point", e.getMessage());
    }

    env->output->outputDebug("        Added MIP starting point.");
}

void MIPSolverGurobi::writeProblemToFile(std::string filename)
{
    try
    {
        gurobiModel->write(filename);
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when saving model to file", e.getMessage());
    }
}

double MIPSolverGurobi::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    double objVal = NAN;

    /* Does not seem to be true for nonconvex MIQCQP
    if(!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError(
            "Cannot obtain solution with index " + std::to_string(solIdx) + " since the problem is LP/QP!");

        return (objVal);
    }*/

    try
    {
        if((isMIP && solIdx == 0) || !isMIP)
        {
            objVal = gurobiModel->get(GRB_DoubleAttr_ObjVal);
        }
        else // Gurobi has no functionality to access the objective value of a specific solution
        {
            gurobiModel->set(GRB_IntParam_SolutionNumber, solIdx);

            auto objective = gurobiModel->getObjective();
            objVal = objective.getLinExpr().getConstant();

            for(size_t i = 0; i < objective.size(); i++)
            {
                objVal += objective.getCoeff(i) * objective.getVar1(i).get(GRB_DoubleAttr_Xn)
                    * objective.getVar2(i).get(GRB_DoubleAttr_Xn);
            }

            auto linexpr = objective.getLinExpr();

            for(size_t i = 0; i < linexpr.size(); i++)
            {
                objVal += linexpr.getCoeff(i) * linexpr.getVar(i).get(GRB_DoubleAttr_Xn);
            }
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Error when obtaining objective value for solution index " + std::to_string(solIdx),
            e.getMessage());
    }

    return (objVal);
}

void MIPSolverGurobi::deleteMIPStarts()
{
    int numVar = gurobiModel->get(GRB_IntAttr_NumVars);

    try
    {
        for(int i = 0; i < numVar; i++)
        {
            GRBVar tmpVar = gurobiModel->getVar(i);
            tmpVar.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when deleting MIP starting points", e.getMessage());
    }

    env->output->outputDebug("        Deleted MIP starting points.");
}

void MIPSolverGurobi::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverGurobi::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound && currentVariableBounds.second == upperBound)
        return;

    try
    {
        GRBVar tmpVar = gurobiModel->getVar(varIndex);

        tmpVar.set(GRB_DoubleAttr_LB, lowerBound);
        tmpVar.set(GRB_DoubleAttr_UB, upperBound);

        modelUpdated = true;
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }
}

void MIPSolverGurobi::updateVariableLowerBound(int varIndex, double lowerBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound)
        return;

    try
    {
        GRBVar tmpVar = gurobiModel->getVar(varIndex);

        tmpVar.set(GRB_DoubleAttr_LB, lowerBound);

        modelUpdated = true;
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }
}

void MIPSolverGurobi::updateVariableUpperBound(int varIndex, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.second == upperBound)
        return;

    try
    {
        GRBVar tmpVar = gurobiModel->getVar(varIndex);

        tmpVar.set(GRB_DoubleAttr_UB, upperBound);

        modelUpdated = true;
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }
}

PairDouble MIPSolverGurobi::getCurrentVariableBounds(int varIndex)
{
    PairDouble tmpBounds;

    try
    {
        GRBVar tmpVar = gurobiModel->getVar(varIndex);

        tmpBounds.first = tmpVar.get(GRB_DoubleAttr_LB);
        tmpBounds.second = tmpVar.get(GRB_DoubleAttr_UB);
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "        Error when obtaining variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }

    return (tmpBounds);
}

bool MIPSolverGurobi::supportsQuadraticObjective() { return (true); }
bool MIPSolverGurobi::supportsQuadraticConstraints() { return (true); }

double MIPSolverGurobi::getUnboundedVariableBoundValue() { return (1e+20); }

double MIPSolverGurobi::getDualObjectiveValue()
{
    bool isMIP = getDiscreteVariableStatus();
    double objVal = (isMinimizationProblem ? SHOT_DBL_MIN : SHOT_DBL_MAX);

    try
    {
        if(isMIP)
        {
            objVal = gurobiModel->get(GRB_DoubleAttr_ObjBound);
        }
        else
        {
            objVal = gurobiModel->get(GRB_DoubleAttr_ObjVal);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when obtaining dual objective value",
            e.getMessage() + " (" + std::to_string(e.getErrorCode()) + ")");
    }

    return (objVal);
}

void MIPSolverGurobi::writePresolvedToFile([[maybe_unused]] std::string filename) { }

void MIPSolverGurobi::checkParameters() { }

std::pair<VectorDouble, VectorDouble> MIPSolverGurobi::presolveAndGetNewBounds()
{
    // TODO
    // auto m = gurobiModel->presolve();

    return (std::make_pair(variableLowerBounds, variableUpperBounds));
}

int MIPSolverGurobi::getNumberOfExploredNodes()
{
    try
    {
        return ((int)gurobiModel->get(GRB_DoubleAttr_NodeCount));
    }
    catch(GRBException&)
    {
        env->output->outputDebug("        Error when getting number of nodes.");
        return 0;
    }
}

std::string MIPSolverGurobi::getSolverVersion()
{
    return (fmt::format("{}.{}", std::to_string(GRB_VERSION_MAJOR), std::to_string(GRB_VERSION_MINOR)));
}

GurobiCallbackMultiTree::GurobiCallbackMultiTree(EnvironmentPtr envPtr)
{
    env = envPtr;
    showOutput = env->settings->getSetting<bool>("Console.DualSolver.Show", "Output");
}

void GurobiCallbackMultiTree::callback()
{
    try
    {
        if(where == GRB_CB_MESSAGE && showOutput) // Show output on console and log
        {
            auto message = getStringInfo(GRB_CB_MSG_STRING);
            message.erase(std::remove(message.begin(), message.end(), '\n'), message.end());
            env->output->outputInfo(fmt::format("      | {} ", message));
        }
        else if(where == GRB_CB_MIP)
        // Used to get the number of open nodes
        {
            auto currIter = env->results->getCurrentIteration();
            currIter->numberOfExploredNodes = (int)getDoubleInfo(GRB_CB_MIP_NODCNT);
            currIter->numberOfOpenNodes = (int)getDoubleInfo(GRB_CB_MIP_NODLFT);
        }

        if(checkUserTermination())
            this->abort();
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when running main callback method", e.getMessage());
    }
    catch(...)
    {
        env->output->outputError("        Gurobi error when running main callback method");
    }
}
} // namespace SHOT
