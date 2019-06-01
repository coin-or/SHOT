/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverGurobi.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Results.h"
#include "../Settings.h"
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

    initializeProblem();
}

MIPSolverGurobi::~MIPSolverGurobi() = default;

bool MIPSolverGurobi::initializeProblem()
{
    discreteVariablesActivated = true;

    if(alreadyInitialized)
    {
        std::shared_ptr<GRBEnv> gurobiEnv;
        std::shared_ptr<GRBModel> gurobiModel;
    }
    else
    {

        prevSolutionLimit = 1;
        alreadyInitialized = true;
    }

    try
    {
        gurobiEnv = std::make_shared<GRBEnv>();
        gurobiModel = std::make_shared<GRBModel>(*gurobiEnv.get());
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when initializing problem:", e.getMessage());
        return (false);
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();

    return (true);
}

bool MIPSolverGurobi::addVariable(std::string name, E_VariableType type, double lowerBound, double upperBound)
{
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
            gurobiModel->addVar(lowerBound, upperBound, 0.0, GRB_SEMICONT, name);
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
                env->output->outputError("        Gurobi does not support quadratic equality constraints.");
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
        if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            gurobiModel->getEnv().set(GRB_IntParam_OutputFlag, 0);
        }

        gurobiModel->getEnv().set(
            GRB_DoubleParam_MIPGap, env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") / 1.0);
        gurobiModel->getEnv().set(
            GRB_DoubleParam_MIPGapAbs, env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);

        // Default 0 to fix som problems with some problems
        gurobiModel->getEnv().set(
            GRB_IntParam_ScaleFlag, env->settings->getSetting<int>("Gurobi.ScaleFlag", "Subsolver"));
        gurobiModel->getEnv().set(
            GRB_IntParam_NumericFocus, env->settings->getSetting<int>("Gurobi.NumericFocus", "Subsolver"));
        gurobiModel->getEnv().set(
            GRB_IntParam_MIPFocus, env->settings->getSetting<int>("Gurobi.MIPFocus", "Subsolver"));
        gurobiModel->getEnv().set(GRB_IntParam_Threads, env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual"));
        // gurobiModel->getEnv().set(GRB_DoubleParam_FeasibilityTol, 1e-6);
        // gurobiModel->getEnv().set(GRB_DoubleParam_IntFeasTol, 1e-6);
        // gurobiModel->getEnv().set(GRB_DoubleParam_OptimalityTol, 1e-6);
        // gurobiModel->getEnv().set(GRB_DoubleParam_MarkowitzTol, 1e-4);
        // gurobiModel->getEnv().set(GRB_DoubleParam_NodeLimit, 1e15);
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
        gurobiModel->getEnv().set(
            GRB_IntParam_SolutionNumber, env->settings->getSetting<int>("MIP.SolutionPool.Capacity", "Dual") + 1);
    }
    catch(GRBException& e)
    {
        {
            env->output->outputError("        Error when initializing parameters for linear solver", e.getMessage());
        }
    }
}

int MIPSolverGurobi::addLinearConstraint(
    const std::vector<PairIndexValue>& elements, double constant, std::string name, bool isGreaterThan)
{
    try
    {
        auto expr = std::make_unique<GRBLinExpr>(0.0);

        for(auto E : elements)
        {
            auto variable = gurobiModel->getVar(E.index);
            *expr.get() = *expr.get() + E.value * variable;
        }

        if(isGreaterThan)
        {
            gurobiModel->addConstr(-constant >= *expr);
        }
        else
        {
            gurobiModel->addConstr(*expr <= -constant);
        }

        modelUpdated = true;
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when adding linear constraint", e.getMessage());

        return (-1);
    }

    return (gurobiModel->get(GRB_IntAttr_NumConstrs) - 1);
}

void MIPSolverGurobi::createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes)
{
    try
    {
        GRBLinExpr expr = 0;

        for(int I : binaryIndexesOnes)
        {
            auto variable = gurobiModel->getVar(I);
            expr += 1.0 * variable;
        }

        for(int I : binaryIndexesZeroes)
        {
            auto variable = gurobiModel->getVar(I);
            expr += (1.0 - 1.0 * variable);
        }

        gurobiModel->addConstr(expr <= binaryIndexesOnes.size() + binaryIndexesZeroes.size() - 1.0);

        modelUpdated = true;

        env->solutionStatistics.numberOfIntegerCuts++;
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when adding lazy integer cut", e.getMessage());
    }
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
            gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber, solIdx);

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
    if(activate)
    {
        env->output->outputDebug("        Activating MIP strategy.");

        for(int i = 0; i < numberOfVariables; i++)
        {
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

        GurobiInfoCallback gurobiCallback = GurobiInfoCallback(env);
        gurobiModel->setCallback(&gurobiCallback);
        gurobiModel->optimize();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

bool MIPSolverGurobi::repairInfeasibility()
{
    try
    {
        gurobiModel->update();
        auto feasModel = GRBModel(*gurobiModel);

        int numOrigConstraints = env->reformulatedProblem->properties.numberOfLinearConstraints;
        int numOrigVariables = gurobiModel->get(GRB_IntAttr_NumVars);
        int numCurrConstraints = feasModel.get(GRB_IntAttr_NumConstrs);

        std::vector<GRBConstr> repairConstraints;
        std::vector<GRBConstr> originalConstraints;
        VectorDouble relaxParameters;
        int numConstraintsToRepair = 0;

        int cutoffOffset = 0;

        for(int i = numOrigConstraints; i < numCurrConstraints; i++)
        {
            if(i == cutOffConstraintIndex)
            {
                cutoffOffset = 1;
                continue;
            }
            else if(env->dualSolver->generatedHyperplanes.at(i - numOrigConstraints - cutoffOffset).isSourceConvex)
            {
                continue;
            }
            else if(std::find(integerCuts.begin(), integerCuts.end(), i) != integerCuts.end())
            {
                continue;
            }
            else
            {
                repairConstraints.push_back(feasModel.getConstr(i));
                originalConstraints.push_back(gurobiModel->getConstr(i));
                relaxParameters.push_back(1 / (((double)i) + 1.0));
                numConstraintsToRepair++;
            }
        }

        // Gurobi modifies the value when running feasModel.optimize()
        int numConstraintsToRepairOrig = numConstraintsToRepair;

        if(feasModel.feasRelax(GRB_FEASRELAX_LINEAR, false, 0, nullptr, nullptr, nullptr, numConstraintsToRepair,
               &repairConstraints[0], &relaxParameters[0])
            < 0)
        {
            env->output->outputCritical("        Could not repair the infeasible dual problem.");
            return (false);
        }

        feasModel.optimize();

        int status = feasModel.get(GRB_IntAttr_Status);

        if(status != GRB_OPTIMAL)
        {
            env->output->outputCritical("        Could not repair the infeasible dual problem.");
            return (false);
        }

        int numRepairs = 0;

        auto vars = feasModel.getVars();

        for(int i = 0; i < numConstraintsToRepairOrig; i++)
        {
            auto variable = feasModel.getVar(numOrigVariables + i);
            double slackValue = variable.get(GRB_DoubleAttr_X);
            auto constraint = originalConstraints.at(i);
            double oldRHS = constraint.get(GRB_DoubleAttr_RHS);
            constraint.set(GRB_DoubleAttr_RHS, oldRHS + 1.5 * slackValue);

            numRepairs++;

            env->output->outputDebug("        Constraint: " + constraint.get(GRB_StringAttr_ConstrName)
                + " repaired with infeasibility = " + std::to_string(1.5 * slackValue));
        }

        env->output->outputCritical("        Number of constraints modified: " + std::to_string(numRepairs));

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            std::stringstream ss;
            ss << env->settings->getSetting<std::string>("Debug.Path", "Output");
            ss << "/lp";
            ss << env->results->getCurrentIteration()->iterationNumber - 1;
            ss << "repaired.lp";
            env->dualSolver->MIPSolver->writeProblemToFile(ss.str());
        }

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
    gurobiModel->getEnv().set(
        GRB_IntParam_SolutionLimit, gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

    return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobi::setSolutionLimit(long limit)
{
    if(limit > GRB_MAXINT)
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
    else
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobi::getSolutionLimit() { return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit)); }

void MIPSolverGurobi::setTimeLimit(double seconds)
{
    try
    {
        if(seconds > 0)
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_TimeLimit, seconds);
        }
        else
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_TimeLimit, 0.00001);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when setting time limit", e.getMessage());
    }
}

void MIPSolverGurobi::setCutOff(double cutOff)
{
    try
    {
        // Gurobi has problems if not an epsilon value is added to the cutoff...

        double cutOffTol = env->settings->getSetting<double>("MIP.CutOffTolerance", "Dual");

        if(isMinimizationProblem)
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_Cutoff, cutOff + cutOffTol);

            env->output->outputDebug(
                "        Setting cutoff value to " + std::to_string(cutOff) + " for minimization.");
        }
        else
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_Cutoff, cutOff - cutOffTol);
            env->output->outputDebug(
                "        Setting cutoff value to " + std::to_string(cutOff) + " for maximization.");
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
                constraint.set(GRB_DoubleAttr_RHS, -cutOff);
                env->output->outputCritical(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
            }
            else
            {
                constraint.set(GRB_DoubleAttr_RHS, cutOff);
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

        for(double P : point)
        {
            startVal.push_back(P);
        }

        for(auto& V : env->reformulatedProblem->auxiliaryVariables)
        {
            startVal.push_back(V->calculateAuxiliaryValue(point));
        }

        if(env->reformulatedProblem->auxiliaryObjectiveVariable)
            startVal.push_back(env->reformulatedProblem->auxiliaryObjectiveVariable->calculateAuxiliaryValue(point));

        for(int i = 0; i < startVal.size(); i++)
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

    if(!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError(
            "Cannot obtain solution with index " + std::to_string(solIdx) + " since the problem is LP/QP!");

        return (objVal);
    }

    try
    {
        if((isMIP && solIdx == 0) || !isMIP)
        {
            objVal = gurobiModel->get(GRB_DoubleAttr_ObjVal);
        }
        else // Gurobi has no functionality to access the objective value of a specific solution
        {
            gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber, solIdx);

            int numvars = gurobiModel->get(GRB_IntAttr_NumVars);

            auto objective = gurobiModel->getObjective();
            objVal = objective.getLinExpr().getConstant();

            for(int i = 0; i < objective.size(); i++)
            {
                objVal += objective.getCoeff(i) * objective.getVar1(i).get(GRB_DoubleAttr_Xn)
                    * objective.getVar2(i).get(GRB_DoubleAttr_Xn);
            }

            auto linexpr = objective.getLinExpr();

            for(int i = 0; i < linexpr.size(); i++)
            {
                objVal += linexpr.getCoeff(i) * linexpr.getVar(i).get(GRB_DoubleAttr_Xn);
            }
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError(
            "Error when obtaining objective value for solution index " + std::to_string(solIdx), e.getMessage());
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
            "Error when updating variable bounds for variable index" + std::to_string(varIndex), e.getMessage());
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
            "Error when updating variable bounds for variable index" + std::to_string(varIndex), e.getMessage());
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
            "Error when updating variable bounds for variable index" + std::to_string(varIndex), e.getMessage());
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
            "Error when obtaining variable bounds for variable index" + std::to_string(varIndex), e.getMessage());
    }

    return (tmpBounds);
}

bool MIPSolverGurobi::supportsQuadraticObjective() { return (true); }
bool MIPSolverGurobi::supportsQuadraticConstraints() { return (true); }

double MIPSolverGurobi::getUnboundedVariableBoundValue() { return (1e+20); }

double MIPSolverGurobi::getDualObjectiveValue()
{

    bool isMIP = getDiscreteVariableStatus();
    double objVal = NAN;

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

void MIPSolverGurobi::writePresolvedToFile(std::string filename) {}

void MIPSolverGurobi::checkParameters() {}

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
    catch(GRBException& e)
    {
        env->output->outputError("        Error when getting number of nodes", e.getMessage());
        return 0;
    }
}

GurobiInfoCallback::GurobiInfoCallback(EnvironmentPtr envPtr) : env(envPtr) {}

// Used to get the number of open nodes
void GurobiInfoCallback::callback()
{
    try
    {
        if(where == GRB_CB_MIP)
        {
            auto currIter = env->results->getCurrentIteration();
            currIter->numberOfExploredNodes = (int)getDoubleInfo(GRB_CB_MIP_NODCNT);
            currIter->numberOfOpenNodes = (int)getDoubleInfo(GRB_CB_MIP_NODLFT);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when running main callback method", e.getMessage());
    }
}
} // namespace SHOT