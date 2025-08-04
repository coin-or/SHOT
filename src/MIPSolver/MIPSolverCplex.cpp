/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplex.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

namespace SHOT
{

UserTerminationCallbackI::UserTerminationCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv)
    : IloCplex::MIPInfoCallbackI(iloEnv)
{
    env = envPtr;
}

IloCplex::CallbackI* UserTerminationCallbackI::duplicateCallback() const
{
    return (new(getEnv()) UserTerminationCallbackI(*this));
}

void UserTerminationCallbackI::main() // Called at each node...
{
    if(checkUserTermination())
    {
        env->output->outputDebug("        Terminated by user.");

        this->abort();
        return;
    }

    return;
}

MIPSolverCplex::MIPSolverCplex()
{
    // Should not be called
}

MIPSolverCplex::MIPSolverCplex(EnvironmentPtr envPtr)
{
    env = envPtr;
    cplexModel = IloModel(cplexEnv);

    cplexVars = IloNumVarArray(cplexEnv);
    cplexConstrs = IloRangeArray(cplexEnv);
}

MIPSolverCplex::~MIPSolverCplex()
{

    cplexVarConvers.clear();
    cplexModel.end();
    cplexVars.end();
    cplexConstrs.end();
    cplexInstance.end();
    cplexEnv.end();

    if(callbacksInitialized)
    {
        cplexInstance.remove(infoCallback);
        delete infoCallback;
        callbacksInitialized = false;
    }
}
bool MIPSolverCplex::initializeProblem()
{
    discreteVariablesActivated = true;

    if(alreadyInitialized)
    {
        cplexVarConvers.clear();
        cplexModel.end();
        cplexVars.end();
        cplexConstrs.end();
        cplexInstance.end();
    }
    else
    {
        prevSolutionLimit = 1;
        alreadyInitialized = true;
    }

    try
    {
        cplexModel = IloModel(cplexEnv);

        cplexVars = IloNumVarArray(cplexEnv);
        cplexConstrs = IloRangeArray(cplexEnv);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Cplex exception caught when initializing model", e.getMessage());
        return (false);
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;
    modelUpdated = false;

    checkParameters();

    return (true);
}

bool MIPSolverCplex::addVariable(
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
        {
            auto cplexVar = IloNumVar(cplexEnv, lowerBound, upperBound, ILOFLOAT, name.c_str());
            cplexVars.add(cplexVar);
            cplexModel.add(cplexVar);
            break;
        }
        case E_VariableType::Integer:
        case E_VariableType::Binary:
        {
            isProblemDiscrete = true;
            auto cplexVar = IloNumVar(cplexEnv, lowerBound, upperBound, ILOINT, name.c_str());
            cplexVars.add(cplexVar);
            cplexModel.add(cplexVar);
            break;
        }
        case E_VariableType::Semicontinuous:
        case E_VariableType::Semiinteger:
        {
            isProblemDiscrete = true;
            if(semiBound < 0.0)
                upperBound = semiBound;
            else
                lowerBound = semiBound;
            auto cplexVar = IloSemiContVar(cplexEnv, lowerBound, upperBound,
                (type == E_VariableType::Semicontinuous) ? ILOFLOAT : ILOINT, name.c_str());
            cplexVars.add(cplexVar);
            cplexModel.add(cplexVar);
            break;
        }
        default:
        {
            break;
        }
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Cplex exception caught when adding variable to model: ", e.getMessage());
        return (false);
    }

    variableTypes.push_back(type);
    variableNames.push_back(name);
    variableLowerBounds.push_back(lowerBound);
    variableUpperBounds.push_back(upperBound);
    numberOfVariables++;
    return (true);
}

bool MIPSolverCplex::initializeObjective()
{
    try
    {
        objExpression = IloExpr(cplexEnv);
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Cplex exception caught when initializing objective function: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverCplex::addLinearTermToObjective(double coefficient, int variableIndex)
{
    try
    {
        objExpression += coefficient * cplexVars[variableIndex];
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Cplex exception caught when adding linear term to objective: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverCplex::addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
    try
    {
        objExpression += coefficient * cplexVars[firstVariableIndex] * cplexVars[secondVariableIndex];
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Cplex exception caught when adding quadratic term to objective: ", e.getMessage());
        return (false);
    }

    hasQuadraticObjective = true;

    return (true);
}

bool MIPSolverCplex::finalizeObjective(bool isMinimize, double constant)
{
    try
    {
        if(constant != 0.0 && !this->hasDualAuxiliaryObjectiveVariable())
            objExpression += constant;

        if(isMinimize)
        {
            cplexObjectiveExpression = objExpression;
            cplexModel.add(IloMinimize(cplexEnv, cplexObjectiveExpression));
            isMinimizationProblem = true;
        }
        else
        {
            cplexObjectiveExpression = objExpression;
            cplexModel.add(IloMaximize(cplexEnv, cplexObjectiveExpression));
            isMinimizationProblem = false;
        }
    }
    catch(IloException& e)
    {
        objExpression.end();
        env->output->outputError(
            "        Cplex exception caught when adding objective function to model: ", e.getMessage());
        return (false);
    }

    objExpression.end();
    return (true);
}

bool MIPSolverCplex::initializeConstraint()
{
    try
    {
        constrExpression = IloExpr(cplexEnv);
    }
    catch(IloException& e)
    {
        objExpression.end();
        env->output->outputError("        Cplex exception caught when initializing constraint: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverCplex::addLinearTermToConstraint(double coefficient, int variableIndex)
{
    try
    {
        constrExpression += coefficient * cplexVars[variableIndex];
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Cplex exception caught when adding linear term to constraint: ", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverCplex::addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
    try
    {
        constrExpression += coefficient * cplexVars[firstVariableIndex] * cplexVars[secondVariableIndex];
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Cplex exception caught when adding quadratic term to constraint: ", e.getMessage());
        return (false);
    }

    hasQudraticConstraint = true;

    return (true);
}

bool MIPSolverCplex::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
{
    try
    {
        if(constant != 0.0)
            constrExpression += constant;

        if(valueLHS <= valueRHS)
        {
            IloRange tmpRange = IloRange(cplexEnv, valueLHS, constrExpression, valueRHS, name.c_str());
            cplexModel.add(tmpRange);
            cplexConstrs.add(tmpRange);
            allowRepairOfConstraint.push_back(false);
        }
        else
        {
            IloRange tmpRange = IloRange(cplexEnv, valueRHS, constrExpression, valueLHS, name.c_str());
            cplexModel.add(tmpRange);
            cplexConstrs.add(tmpRange);
            allowRepairOfConstraint.push_back(false);
        }
    }
    catch(IloException& e)
    {
        constrExpression.end();
        env->output->outputError("        Cplex exception caught when adding constraint to model: ", e.getMessage());
        return (false);
    }

    constrExpression.end();

    numberOfConstraints++;
    return (true);
}

bool MIPSolverCplex::finalizeProblem()
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

            cplexInstance = IloCplex(cplexModel);
            setSolutionLimit(setSolLimit);

            if(!discreteVariablesActivated)
            {
                activateDiscreteVariables(false);
            }
        }
        else
        {
            cplexInstance = IloCplex(cplexModel);
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Cplex exception caught when finalizing model", e.getMessage());
        return (false);
    }

    return (true);
}

void MIPSolverCplex::initializeSolverSettings()
{
    try
    {
        // Control solver output
        if(env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            cplexOutputStream = std::make_unique<OutputStream>(env, E_LogLevel::Info);
            cplexInstance.setOut(*cplexOutputStream);
            cplexInstance.setWarning(cplexEnv.getNullStream());
        }
        else
        {
            cplexInstance.setOut(cplexEnv.getNullStream());

            if(env->settings->getSetting<int>("Console.LogLevel", "Output") <= static_cast<int>(E_LogLevel::Debug))
            {
                cplexOutputStream = std::make_unique<OutputStream>(env, E_LogLevel::Info);
                cplexInstance.setWarning(*cplexOutputStream);
            }
            else
                cplexInstance.setWarning(cplexEnv.getNullStream());
        }

        // Set termination tolerances
        cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::MIPGap,
            env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") / 1.0);
        cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap,
            env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);
        cplexInstance.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility,
            env->settings->getSetting<double>("Tolerance.LinearConstraint", "Primal"));
        cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::Integrality,
            env->settings->getSetting<double>("Tolerance.Integer", "Primal"));
        cplexInstance.setParam(IloCplex::Param::Simplex::Tolerances::Optimality,
            env->settings->getSetting<double>("MIP.OptimalityTolerance", "Dual"));

        // Adds a user-provided node limit
        if(auto nodeLimit = env->settings->getSetting<double>("MIP.NodeLimit", "Dual"); nodeLimit > 0)
        {
            if(nodeLimit > CPX_BIGINT)
                cplexInstance.setParam(IloCplex::Param::MIP::Limits::Nodes, CPX_BIGINT);
        }

        // Set solution pool settings
        cplexInstance.setParam(IloCplex::Param::MIP::Pool::Intensity,
            env->settings->getSetting<int>("Cplex.SolutionPoolIntensity", "Subsolver")); // Don't use 3 with heuristics
        cplexInstance.setParam(IloCplex::Param::MIP::Pool::Replace,
            env->settings->getSetting<int>("Cplex.SolutionPoolReplace", "Subsolver"));
        cplexInstance.setParam(IloCplex::Param::MIP::Pool::RelGap,
            env->settings->getSetting<double>("Cplex.SolutionPoolGap", "Subsolver"));
        cplexInstance.setParam(
            IloCplex::Param::MIP::Pool::Capacity, env->settings->getSetting<int>("MIP.SolutionPool.Capacity", "Dual"));

        if(env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
        {
            if(env->results->getNumberOfIterations() == 0)
                cplexInstance.setParam(IloCplex::Param::MIP::Limits::Solutions,
                    env->settings->getSetting<int>("MIP.SolutionLimit.Initial", "Dual"));
        }
        else
        {
            cplexInstance.setParam(IloCplex::Param::MIP::Limits::Solutions, 21000000);
        }

        // Set solver emphasis
        cplexInstance.setParam(IloCplex::Param::Emphasis::Numerical,
            env->settings->getSetting<int>("Cplex.NumericalEmphasis", "Subsolver"));
        cplexInstance.setParam(
            IloCplex::Param::Emphasis::Memory, env->settings->getSetting<int>("Cplex.MemoryEmphasis", "Subsolver"));

        // Set parameters for quadratics
        cplexInstance.setParam(
            IloCplex::Param::OptimalityTarget, env->settings->getSetting<int>("Cplex.OptimalityTarget", "Subsolver"));

        // Set various solver specific MIP settings
        cplexInstance.setParam(
            IloCplex::Param::MIP::Strategy::Probe, env->settings->getSetting<int>("Cplex.Probe", "Subsolver"));
        cplexInstance.setParam(
            IloCplex::Param::Emphasis::MIP, env->settings->getSetting<int>("Cplex.MIPEmphasis", "Subsolver"));
        cplexInstance.setParam(
            IloCplex::Param::Parallel, env->settings->getSetting<int>("Cplex.ParallelMode", "Subsolver"));

        // Set number of threads
        cplexInstance.setParam(IloCplex::Param::Threads, env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual"));

        // Options for using swap file
        if(auto workdir = env->settings->getSetting<std::string>("Cplex.WorkDirectory", "Subsolver"); workdir != "")
            cplexInstance.setParam(IloCplex::Param::WorkDir, workdir.c_str());

        if(auto workmem = env->settings->getSetting<double>("Cplex.WorkMemory", "Subsolver"); workmem > 0)
            cplexInstance.setParam(IloCplex::Param::WorkMem, workmem);

        cplexInstance.setParam(
            IloCplex::Param::MIP::Strategy::File, env->settings->getSetting<int>("Cplex.NodeFile", "Subsolver"));

        // Set feasibility relaxation parameters
        cplexInstance.setParam(
            IloCplex::Param::Feasopt::Mode, env->settings->getSetting<int>("Cplex.FeasOptMode", "Subsolver"));
    }
    catch(IloException& e)
    {
        env->output->outputError(" Error when initializing Cplex parameters: ", e.getMessage());
    }
}

int MIPSolverCplex::addLinearConstraint(
    const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
{
    try
    {
        int numConstraintsBefore = cplexInstance.getNrows();

        IloExpr expr(cplexEnv);

        for(auto E : elements)
        {
            expr += E.second * cplexVars[E.first];
        }

        if(isGreaterThan)
        {
            IloRange tmpRange(cplexEnv, -constant, expr, IloInfinity);
            tmpRange.setName(name.c_str());

            cplexModel.add(tmpRange);
            cplexInstance.extract(cplexModel);

            // Make sure that Cplex actually has added the constraint
            if(cplexInstance.getNrows() > numConstraintsBefore)
            {
                cplexConstrs.add(tmpRange);
                allowRepairOfConstraint.push_back(allowRepair);
            }
            else
            {
                env->output->outputDebug("        Hyperplane not added by Cplex");
                tmpRange.end();
                return (-1);
            }
        }
        else
        {
            IloRange tmpRange(cplexEnv, -IloInfinity, expr, -constant);
            tmpRange.setName(name.c_str());

            cplexModel.add(tmpRange);
            cplexInstance.extract(cplexModel);

            // Make sure that Cplex actually has added the constraint
            if(cplexInstance.getNrows() > numConstraintsBefore)
            {
                cplexConstrs.add(tmpRange);
                allowRepairOfConstraint.push_back(allowRepair);
            }
            else
            {
                env->output->outputDebug("        Hyperplane not added by Cplex");
                tmpRange.end();
                return (-1);
            }
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when adding linear constraint", e.getMessage());
        return (-1);
    }

    return (cplexInstance.getNrows() - 1);
}

bool MIPSolverCplex::addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights)
{
    try
    {
        IloNumVarArray variables(cplexEnv, variableIndexes.size());

        for(size_t i = 0; i < variableIndexes.size(); i++)
            variables[i] = cplexVars[variableIndexes[i]];

        if(variableWeights.size() > 0)
        {
            assert(variableWeights.size() == variableIndexes.size());

            IloNumArray weights(cplexEnv, variableWeights.size());

            for(size_t i = 0; i < variableWeights.size(); i++)
                weights[i] = variableWeights[i];

            if(type == E_SOSType::One)
                cplexModel.add(IloSOS1(cplexEnv, variables, weights));
            else if(type == E_SOSType::Two)
                cplexModel.add(IloSOS2(cplexEnv, variables, weights));
            else
            {
                env->output->outputError(
                    "        Error when adding special ordered set constraint: type not specified!");
                return (false);
            }
        }
        else
        {
            if(type == E_SOSType::One)
                cplexModel.add(IloSOS1(cplexEnv, variables));
            else if(type == E_SOSType::Two)
                cplexModel.add(IloSOS2(cplexEnv, variables));
            else
            {
                env->output->outputError(
                    "        Error when adding special ordered set constraint: type not specified!");
                return (false);
            }
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when adding special ordered set constraint", e.getMessage());
        return (false);
    }

    return (true);
}

void MIPSolverCplex::activateDiscreteVariables(bool activate)
{
    if(env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0
        || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
        return;

    try
    {
        for(auto& C : cplexVarConvers)
        {
            C.end();
        }

        cplexVarConvers.clear();

        if(activate)
        {
            env->output->outputDebug("        Activating MIP strategy.");

            for(int i = 0; i < numberOfVariables; i++)
            {
                if(variableTypes.at(i) == E_VariableType::Integer)
                {
                    auto tmpVar = cplexVars[i];
                    auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOINT);
                    cplexModel.add(tmpConv);
                    cplexVarConvers.push_back(tmpConv);
                }
                else if(variableTypes.at(i) == E_VariableType::Binary)
                {
                    auto tmpVar = cplexVars[i];
                    auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOBOOL);
                    cplexModel.add(tmpConv);
                    cplexVarConvers.push_back(tmpConv);
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
                    auto tmpVar = cplexVars[i];
                    auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOFLOAT);
                    cplexModel.add(tmpConv);
                    cplexVarConvers.push_back(tmpConv);
                }
            }

            discreteVariablesActivated = false;
        }

        modelUpdated = true;
    }
    catch(IloException& e)
    {
        if(activate)
            env->output->outputError("        Error when activating discrete variables", e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplex::getSolutionStatus()
{
    E_ProblemSolutionStatus MIPSolutionStatus;

    try
    {
        auto status = cplexInstance.getCplexStatus();

        if(status == IloCplex::CplexStatus::Optimal || status == IloCplex::CplexStatus::OptimalTol)
        {
            auto statusInstance = cplexInstance.getStatus();

            if(statusInstance == IloAlgorithm::Status::Optimal)
            {
                MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
            }
            else
            {
                MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
            }
        }
        else if(status == IloCplex::CplexStatus::OptimalInfeas)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
        }
        else if(status == IloCplex::CplexStatus::Feasible)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
        }
        else if(status == IloCplex::CplexStatus::OptimalRelaxedQuad)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
        }
        else if(status == IloCplex::CplexStatus::Infeasible)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
        }
        else if(status == IloCplex::CplexStatus::InfOrUnbd)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
        }
        else if(status == IloCplex::CplexStatus::Unbounded)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
        }
        else if(status == IloCplex::CplexStatus::NodeLimFeas)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::NodeLimit;
        }
        else if(status == IloCplex::CplexStatus::NumBest)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
        }
        else if(status == IloCplex::CplexStatus::AbortTimeLim || status == IloCplex::CplexStatus::AbortDetTimeLim)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
        }
        else if(status == IloCplex::CplexStatus::SolLim)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
        }
        else if(status == IloCplex::CplexStatus::AbortUser)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
        }
        else
        {
            env->output->outputError("        MIP solver return status " + std::to_string(status));
            MIPSolutionStatus = E_ProblemSolutionStatus::Error;
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when obtaining solution status", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverCplex::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        // If we in previous iteration solved a feasibility problem since the objective was unbounded, the original
        // objective needs to be restored
        if(objectiveFunctionReplacedWithZero)
        {
            cplexModel.remove(cplexInstance.getObjective());

            if(isMinimizationProblem)
                cplexModel.add(IloMinimize(cplexEnv, cplexObjectiveExpression));
            else
                cplexModel.add(IloMaximize(cplexEnv, cplexObjectiveExpression));

            modelUpdated = true;
        }

        if(modelUpdated)
        {
            cplexInstance.extract(cplexModel);
            modelUpdated = false;
        }

        if(objectiveFunctionReplacedWithZero || !getDiscreteVariableStatus())
        {
            objectiveFunctionReplacedWithZero = false;

            // Fixes a deadlock bug in Cplex 12.7 and 12.8
            cplexEnv.setNormalizer(false);

            cplexInstance.solve();
            MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();
        }
        else
        {
            infoCallback = new(cplexEnv) UserTerminationCallbackI(env, cplexEnv);
            callbacksInitialized = true;

            cplexInstance.use(infoCallback);

            // Fixes a deadlock bug in Cplex 12.7 and 12.8
            cplexEnv.setNormalizer(false);

            cplexInstance.solve();
            MIPSolutionStatus = getSolutionStatus();
        }

        // Try to solve a feasibility problem to get a valid solution point if unbounded and not when solving the
        // minimax-problem
        if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded && env->results->getNumberOfIterations() > 0)
        {
            cplexModel.remove(cplexInstance.getObjective());

            if(isMinimizationProblem)
                cplexModel.add(IloMinimize(cplexEnv, 0.0));
            else
                cplexModel.add(IloMaximize(cplexEnv, 0.0));

            cplexInstance.extract(cplexModel);
            cplexInstance.solve();
            MIPSolutionStatus = getSolutionStatus();

            if(MIPSolutionStatus == E_ProblemSolutionStatus::Optimal)
                MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;

            objectiveFunctionReplacedWithZero = true;
            modelUpdated = true;
            env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }

        // If the previous repair failed, we can try this
        if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded && env->results->getNumberOfIterations() > 0)
        {
            repairInfeasibility();
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
            env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }

        if(callbacksInitialized)
        {
            cplexInstance.remove(infoCallback);
            delete infoCallback;
            callbacksInitialized = false;
        }
    }

    catch(IloException& e)
    {
        std::string errorString = e.getMessage();

        if(errorString.rfind("CPLEX Error  5002", 0) == 0)
        {
            cplexInstance.setParam(IloCplex::Param::OptimalityTarget, CPX_OPTIMALITYTARGET_OPTIMALGLOBAL);
            return (solveProblem());
        }

        env->output->outputError("        Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

bool MIPSolverCplex::repairInfeasibility()
{
    if(env->dualSolver->generatedHyperplanes.size() == 0)
        return (false);

    try
    {
        if(modelUpdated)
        {
            cplexInstance.extract(cplexModel);
            modelUpdated = false;
        }

        IloNumArray relax(cplexEnv);

        int numCurrConstraints = cplexConstrs.getSize();
        int numOrigConstraints = env->reformulatedProblem->properties.numberOfLinearConstraints
            + env->reformulatedProblem->properties.numberOfConvexQuadraticConstraints;

        relax.add(numOrigConstraints, 0.0);

        for(int i = numOrigConstraints; i < numCurrConstraints; i++)
        {
            if(allowRepairOfConstraint[i])
            {
                relax.add(1 / (((double)i) - numOrigConstraints + 1.0));
            }
            else
            {
                relax.add(0.0);
            }
        }

        // Saves the relaxation weights to a file
        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            VectorDouble weights(relax.getSize());

            for(int i = 0; i < relax.getSize(); i++)
                weights[i] = relax[i];

            VectorString constraints(cplexConstrs.getSize());

            for(int i = 0; i < cplexConstrs.getSize(); i++)
            {
                std::ostringstream expression;
                expression << cplexConstrs[i];
                constraints[i] = expression.str();
            }

            auto filename = fmt::format("{}/dualiter{}_infeasrelaxweights.txt",
                env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            Utilities::saveVariablePointVectorToFile(weights, constraints, filename);
        }

        if(cplexInstance.feasOpt(cplexConstrs, relax))
        {
            IloNumArray infeas(cplexEnv);
            IloNumArray vals(cplexEnv);

            cplexInstance.getInfeasibilities(infeas, cplexConstrs);

            int numRepairs = 0;

            for(int i = numOrigConstraints; i < numCurrConstraints; i++)
            {
                if(infeas[i] > 1e-10) // Cplex does not accept too small values, so zero cannot be used
                {
                    numRepairs++;
                    env->output->outputDebug("        Constraint: " + std::to_string(i)
                        + " repaired with infeasibility = " + std::to_string(infeas[i]));
                    double newRHS = cplexConstrs[i].getUB() + 1.5 * infeas[i];

                    cplexConstrs[i].setUB(newRHS);
                }
                else if(infeas[i] < -1e-10) // Should not happen for generated cuts
                {
                    numRepairs++;
                    env->output->outputDebug("        Constraint: " + std::to_string(i)
                        + " repaired with infeasibility = " + std::to_string(infeas[i]));
                    double newLHS = cplexConstrs[i].getLB() + 1.5 * infeas[i];
                    cplexConstrs[i].setLB(newLHS);
                }
            }

            cplexInstance.extract(cplexModel);

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

        env->output->outputDebug("        Could not repair the infeasible dual problem.");
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when trying to repair infeasibility", e.getMessage());
    }

    return (false);
}

int MIPSolverCplex::increaseSolutionLimit(int increment)
{
    int sollim = 0;

    try
    {
        cplexInstance.setParam(IloCplex::Param::MIP::Limits::Solutions,
            cplexInstance.getParam(IloCplex::Param::MIP::Limits::Solutions) + increment);
        sollim = cplexInstance.getParam(IloCplex::Param::MIP::Limits::Solutions);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when increasing solution limit", e.getMessage());
    }

    return (sollim);
}

void MIPSolverCplex::setSolutionLimit(long limit)
{
    if(limit <= 1)
        limit = 1;

    try
    {
        cplexInstance.setParam(IloCplex::Param::MIP::Limits::Solutions, limit);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when setting solution limit", e.getMessage());
    }
}

int MIPSolverCplex::getSolutionLimit()
{
    int solLim = 0;

    try
    {
        solLim = cplexInstance.getParam(IloCplex::Param::MIP::Limits::Solutions);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when obtaining solution limit", e.getMessage());
    }

    return (solLim);
}

VectorDouble MIPSolverCplex::getVariableSolution(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus() && isProblemDiscrete;
    int numVar = cplexVars.getSize();
    VectorDouble solution(numVar);

    IloNumArray tmpSolsCplex(cplexEnv);

    try
    {
        if(isMIP)
        {
            cplexInstance.getValues(cplexVars, tmpSolsCplex, solIdx);
        }
        else
        {
            cplexInstance.getValues(cplexVars, tmpSolsCplex);
        }

        for(int i = 0; i < numVar; i++)
        {
            solution.at(i) = tmpSolsCplex[i];
        }
    }
    catch(IloException& e)
    {
        if(isMIP)
            env->output->outputError(
                "        Error when reading solution with index for MIP " + std::to_string(solIdx), e.getMessage());
        else
            env->output->outputError("        Error when reading solution for LP", e.getMessage());
    }

    tmpSolsCplex.end();
    return (solution);
}

int MIPSolverCplex::getNumberOfSolutions()
{
    int numSols = 0;
    bool isMIP = getDiscreteVariableStatus();

    try
    {
        if(isProblemDiscrete && isMIP)
        {
            numSols = cplexInstance.getSolnPoolNsolns();
        }
        else
        {
            // LP problem
            auto cplexStatus = cplexInstance.getStatus();

            if(cplexStatus == IloAlgorithm::Status::Optimal || cplexStatus == IloAlgorithm::Status::Feasible)
                numSols = 1;
            else
                numSols = 0;
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when obtaining number of solutions", e.getMessage());
    }

    return (numSols);
}

double MIPSolverCplex::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus() && isProblemDiscrete;

    double objVal = NAN;

    if(!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError(
            "        Cannot obtain solution with index " + std::to_string(solIdx) + " since the problem is LP/QP!");
        return (objVal);
    }

    try
    {
        if(isMIP)
        {
            objVal = cplexInstance.getObjValue(solIdx);
        }
        else
        {
            objVal = cplexInstance.getObjValue();
        }
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Error when obtaining objective value for solution index " + std::to_string(solIdx),
            e.getMessage());
    }

    return (objVal);
}

void MIPSolverCplex::setTimeLimit(double seconds)
{
    try
    {
        if(seconds > 1e+75) { }
        else if(seconds > 0)
        {
            cplexInstance.setParam(IloCplex::Param::TimeLimit, seconds);
        }
        else
        {
            cplexInstance.setParam(IloCplex::Param::TimeLimit, 0.01);
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when setting time limit", e.getMessage());
    }
}

void MIPSolverCplex::setCutOff(double cutOff)
{
    try
    {
        if(isMinimizationProblem)
        {
            cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, cutOff);
            env->output->outputDebug(fmt::format("        Setting cutoff value to  {} for minimization.", cutOff));
        }
        else
        {
            cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::LowerCutoff, cutOff);
            env->output->outputDebug(fmt::format("        Setting cutoff value to  {} for maximization.", cutOff));
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when setting cut off value", e.getMessage());
    }
}

void MIPSolverCplex::setCutOffAsConstraint(double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    try
    {
        if(!cutOffConstraintDefined)
        {
            if(env->reformulatedProblem->objectiveFunction->properties.isMaximize)
            {
                IloRange tmpRange(cplexEnv, cutOff, cplexObjectiveExpression);
                tmpRange.setName("CUTOFF_C");
                cplexModel.add(tmpRange);
                cplexConstrs.add(tmpRange);
                allowRepairOfConstraint.push_back(false);

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for maximization.");
            }
            else
            {
                IloRange tmpRange(cplexEnv, -IloInfinity, cplexObjectiveExpression, cutOff);
                tmpRange.setName("CUTOFF_C");
                cplexModel.add(tmpRange);
                cplexConstrs.add(tmpRange);
                allowRepairOfConstraint.push_back(false);

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
            }

            cutOffConstraintIndex = cplexConstrs.getSize() - 1;

            modelUpdated = true;

            cutOffConstraintDefined = true;
        }
        else
        {
            if(env->reformulatedProblem->objectiveFunction->properties.isMaximize)
            {
                if(hasDualAuxiliaryObjectiveVariable())
                    cplexConstrs[cutOffConstraintIndex].setLB(cutOff);
                else
                    cplexConstrs[cutOffConstraintIndex].setLB(
                        cutOff - env->reformulatedProblem->objectiveFunction->constant);

                env->output->outputDebug(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
            }
            else
            {
                if(hasDualAuxiliaryObjectiveVariable())
                    cplexConstrs[cutOffConstraintIndex].setUB(cutOff);
                else
                    cplexConstrs[cutOffConstraintIndex].setUB(
                        cutOff - env->reformulatedProblem->objectiveFunction->constant);

                cplexConstrs[cutOffConstraintIndex].setUB(cutOff);
                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
            }

            modelUpdated = true;
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when setting cut off value through constraint", e.getMessage());
    }
}

void MIPSolverCplex::addMIPStart(VectorDouble point)
{
    IloNumArray startVal(cplexEnv);

    if((int)point.size() < env->reformulatedProblem->properties.numberOfVariables)
        env->reformulatedProblem->augmentAuxiliaryVariableValues(point);

    assert(env->reformulatedProblem->properties.numberOfVariables == point.size());

    for(double P : point)
        startVal.add(P);

    auto startVars = IloNumVarArray(cplexEnv);

    if(this->hasDualAuxiliaryObjectiveVariable())
        startVal.add(env->reformulatedProblem->objectiveFunction->calculateValue(point));

    // Not all variables in cplexVars need to be in the MIP start (e.g. variables from integer cuts)
    for(int i = 0; i < startVal.getSize(); i++)
        startVars.add(cplexVars[i]);

    assert(variableNames.size() == startVal.getSize());
    assert(startVars.getSize() == startVal.getSize());

    try
    {
        cplexInstance.addMIPStart(startVars, startVal, IloCplex::MIPStartAuto);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when adding MIP starting point", e.getMessage());
    }

    startVal.end();
    startVars.end();

    env->output->outputDebug("        Added MIP starting point.");
}

void MIPSolverCplex::deleteMIPStarts()
{
    int numStarts = cplexInstance.getNMIPStarts();

    if(numStarts > 0)
    {
        try
        {
            cplexInstance.deleteMIPStarts(0, numStarts);
        }
        catch(IloException& e)
        {
            env->output->outputError("        Error when deleting MIP starting points", e.getMessage());
        }

        env->output->outputDebug("        Deleted " + std::to_string(numStarts) + " MIP starting points.");
    }
}

void MIPSolverCplex::writeProblemToFile(std::string filename)
{
    try
    {
        if(objectiveFunctionReplacedWithZero)
        {
            cplexModel.remove(cplexInstance.getObjective());

            if(isMinimizationProblem)
                cplexModel.add(IloMinimize(cplexEnv, cplexObjectiveExpression));
            else
                cplexModel.add(IloMaximize(cplexEnv, cplexObjectiveExpression));

            modelUpdated = true;
            objectiveFunctionReplacedWithZero = false;
        }

        if(modelUpdated)
        {
            // Extract the model if we have updated the constraints
            cplexInstance.extract(cplexModel);
            modelUpdated = false;
        }

        cplexInstance.exportModel(filename.c_str());
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when saving model to file", e.getMessage());
    }
}

void MIPSolverCplex::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverCplex::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound && currentVariableBounds.second == upperBound)
        return;

    try
    {
        cplexVars[varIndex].setBounds(lowerBound, upperBound);
        modelUpdated = true;
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }
}

void MIPSolverCplex::updateVariableLowerBound(int varIndex, double lowerBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound)
        return;

    try
    {
        cplexVars[varIndex].setLB(lowerBound);
        modelUpdated = true;
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }
}

void MIPSolverCplex::updateVariableUpperBound(int varIndex, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.second == upperBound)
        return;

    try
    {
        cplexVars[varIndex].setUB(upperBound);
        modelUpdated = true;
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }
}

PairDouble MIPSolverCplex::getCurrentVariableBounds(int varIndex)
{
    PairDouble tmpBounds;

    try
    {
        tmpBounds.first = cplexVars[varIndex].getLB();
        tmpBounds.second = cplexVars[varIndex].getUB();
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Error when obtaining variable bounds for variable index" + std::to_string(varIndex),
            e.getMessage());
    }
    return (tmpBounds);
}

bool MIPSolverCplex::supportsQuadraticObjective() { return (true); }
bool MIPSolverCplex::supportsQuadraticConstraints() { return (true); }

double MIPSolverCplex::getUnboundedVariableBoundValue() { return (1e+20); }

double MIPSolverCplex::getDualObjectiveValue()
{
    bool isMIP = getDiscreteVariableStatus();
    double objVal = (isMinimizationProblem ? SHOT_DBL_MIN : SHOT_DBL_MAX);

    try
    {
        if(isMIP)
        {
            objVal = cplexInstance.getBestObjValue();
        }
        else if(getSolutionStatus() == E_ProblemSolutionStatus::Optimal)
        {
            objVal = cplexInstance.getObjValue();
        }
    }
    catch(IloException&)
    {
        // Happens for infeasible LP
    }

    return (objVal);
}

std::pair<VectorDouble, VectorDouble> MIPSolverCplex::presolveAndGetNewBounds()
{
    IloNumArray redubs(cplexEnv, numberOfVariables);
    IloNumArray redlbs(cplexEnv, numberOfVariables);

    IloBoolArray redund(cplexEnv);

    try
    {
        bool isUpdated = false;

        cplexInstance.basicPresolve(cplexVars, redlbs, redubs, cplexConstrs, redund);

        VectorDouble newLBs;
        VectorDouble newUBs;

        newLBs.reserve(numberOfVariables);
        newUBs.reserve(numberOfVariables);

        for(int i = 0; i < numberOfVariables; i++)
        {
            newLBs.push_back(redlbs[i]);
            newUBs.push_back(redubs[i]);
        }

        if(env->settings->getSetting<bool>("MIP.Presolve.RemoveRedundantConstraints", "Dual"))
        {
            int numconstr = 0;

            for(int j = 0; j < cplexConstrs.getSize(); j++)
            {
                if(redund[j] == 0)
                {
                    cplexModel.remove(cplexConstrs[j]);
                    cplexConstrs[j].asConstraint().removeFromAll();

                    numconstr++;
                    isUpdated = true;
                }
            }

            if(isUpdated)
            {
                cplexInstance.extract(cplexModel);
                env->output->outputDebug(
                    "        Removed " + std::to_string(numconstr) + " redundant constraints from MIP model.");
                env->solutionStatistics.numberOfConstraintsRemovedInPresolve = numconstr;
            }
        }

        redlbs.end();
        redubs.end();
        redund.end();

        return (std::make_pair(newLBs, newUBs));
    }
    catch(IloException& e)
    {
        redlbs.end();
        redubs.end();
        redund.end();

        env->output->outputError("        Error during presolve", e.getMessage());

        return (std::make_pair(variableLowerBounds, variableUpperBounds));
    }
}

void MIPSolverCplex::writePresolvedToFile([[maybe_unused]] std::string filename)
{
    try
    {
        // Not implemented
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when saving presolved model to file", e.getMessage());
    }
}

void MIPSolverCplex::checkParameters() { }

bool MIPSolverCplex::createHyperplane(
    HyperplanePtr hyperplane, std::function<IloConstraint(IloRange)> addConstraintFunction)
{
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    auto optional = createHyperplaneTerms(hyperplane);

    if(!optional)
    {
        return (false);
    }

    auto tmpPair = optional.value();

    for(auto& E : tmpPair.first)
    {
        if(E.second != E.second) // Check for NaN
        {
            env->output->outputError("     Warning: hyperplane not generated, NaN found in linear terms for variable "
                + env->problem->getVariable(E.first)->name);
            return (false);
        }
    }

    try
    {
        IloExpr expr(cplexEnv);

        for(auto& P : tmpPair.first)
        {
            expr += P.second * cplexVars[P.first];
        }

        IloRange tmpRange(cplexEnv, -IloInfinity, expr, -tmpPair.second);

        addConstraintFunction(tmpRange);

        tmpRange.end();
        expr.end();
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when creating hyperplane in Cplex", e.getMessage());
        return (false);
    }

    return (true);
}

bool MIPSolverCplex::createIntegerCut(IntegerCut& integerCut)
{
    bool allowIntegerCutRepair = env->settings->getSetting<bool>("MIP.InfeasibilityRepair.IntegerCuts", "Dual");

    // Verify that no integer values are outside of variable bounds
    for(size_t i = 0; i < integerCut.variableIndexes.size(); i++)
    {
        auto VAR = env->reformulatedProblem->getVariable(integerCut.variableIndexes[i]);
        int variableValue = integerCut.variableValues[i];

        if(variableValue < VAR->lowerBound || variableValue > VAR->upperBound)
            return (false);
    }

    try
    {
        int numConstraintsBefore = cplexInstance.getNrows();
        IloExpr expr(cplexEnv);
        size_t index = 0;

        for(auto& I : integerCut.variableIndexes)
        {
            auto VAR = env->reformulatedProblem->getVariable(I);
            int variableValue = integerCut.variableValues[index];
            auto variable = cplexVars[VAR->index];

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

                auto w = IloNumVar(cplexEnv, 0, getUnboundedVariableBoundValue(), ILOFLOAT,
                    fmt::format("wIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index).c_str());
                cplexVars.add(w);
                cplexModel.add(w);

                auto v = IloNumVar(cplexEnv, 0, 1, ILOBOOL,
                    fmt::format("vIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index).c_str());
                cplexVars.add(v);
                cplexModel.add(v);

                expr += 1.0 * w;

                double M1 = 2 * (variableValue - VAR->lowerBound);
                double M2 = 2 * (VAR->upperBound - variableValue);

                int tmpNumConstraints = cplexInstance.getNrows();
                IloRange cut1(cplexEnv, -IloInfinity, -w - variable, -variableValue,
                    fmt::format("IC{}_{}_1a", env->solutionStatistics.numberOfIntegerCuts, index).c_str());
                cplexModel.add(cut1);

                if(cplexInstance.getNrows() > tmpNumConstraints)
                {
                    cplexInstance.extract(cplexModel);
                    integerCuts.push_back(numConstraintsBefore + index);
                    cplexConstrs.add(cut1);
                    allowRepairOfConstraint.push_back(false);
                }

                tmpNumConstraints = cplexInstance.getNrows();
                IloRange cut2(cplexEnv, -variableValue, w - variable, IloInfinity,
                    fmt::format("IC{}_{}_1b", env->solutionStatistics.numberOfIntegerCuts, index).c_str());
                cplexModel.add(cut2);

                if(cplexInstance.getNrows() > tmpNumConstraints)
                {
                    cplexInstance.extract(cplexModel);
                    integerCuts.push_back(numConstraintsBefore + index);
                    cplexConstrs.add(cut2);
                    allowRepairOfConstraint.push_back(false);
                }

                tmpNumConstraints = cplexInstance.getNrows();
                IloRange cut3(cplexEnv, -IloInfinity, w - variable + M1 * v, -variableValue + M1,
                    fmt::format("IC{}_{}_2", env->solutionStatistics.numberOfIntegerCuts, index).c_str());
                cplexModel.add(cut3);

                if(cplexInstance.getNrows() > tmpNumConstraints)
                {
                    cplexInstance.extract(cplexModel);
                    integerCuts.push_back(numConstraintsBefore + index);
                    cplexConstrs.add(cut3);
                    allowRepairOfConstraint.push_back(false);
                }

                tmpNumConstraints = cplexInstance.getNrows();
                IloRange cut4(cplexEnv, -IloInfinity, w + variable - M2 * v, variableValue,
                    fmt::format("IC{}_{}_3", env->solutionStatistics.numberOfIntegerCuts, index).c_str());
                cplexModel.add(cut4);

                if(cplexInstance.getNrows() > tmpNumConstraints)
                {
                    cplexInstance.extract(cplexModel);
                    integerCuts.push_back(numConstraintsBefore + index);
                    cplexConstrs.add(cut4);
                    allowRepairOfConstraint.push_back(false);
                }
            }

            index++;
        }

        int tmpNumConstraints = cplexInstance.getNrows();
        IloRange cut5(
            cplexEnv, 1, expr, IloInfinity, fmt::format("IC{}_4", env->solutionStatistics.numberOfIntegerCuts).c_str());
        cplexModel.add(cut5);

        if(cplexInstance.getNrows() > tmpNumConstraints)
        {
            cplexInstance.extract(cplexModel);
            integerCuts.push_back(numConstraintsBefore + index);
            cplexConstrs.add(cut5);
            allowRepairOfConstraint.push_back(allowIntegerCutRepair);
        }

        auto addedConstraints = cplexInstance.getNrows() - numConstraintsBefore;

        if(addedConstraints == 0)
        {
            env->output->outputDebug("        Integer cut not added by Cplex");
            return (false);
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when creating integer cut in Cplex", e.getMessage());
        return (false);
    }

    return (true);
}

int MIPSolverCplex::getNumberOfExploredNodes()
{
    try
    {
        return (cplexInstance.getNnodes());
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when getting number of explored nodes", e.getMessage());
        return 0;
    }
}

int MIPSolverCplex::getNumberOfOpenNodes()
{
    try
    {
        int nodesLeft = cplexInstance.getNnodesLeft();

        env->solutionStatistics.numberOfOpenNodes = nodesLeft;

        return (cplexInstance.getNnodesLeft());
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when getting number of open nodes", e.getMessage());
        return 0;
    }
}

std::string MIPSolverCplex::getSolverVersion()
{
    std::string version = std::to_string(cplexInstance.getVersionNumber());
    std::string major = version.substr(0, 2);
    std::string minor = version.substr(2, 2);

    if(minor.substr(0, 1) == "0")
        minor = minor.substr(1, 1);

    return (fmt::format("{}.{}", major, minor));
}
} // namespace SHOT