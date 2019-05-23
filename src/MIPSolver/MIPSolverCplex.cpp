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

MIPSolverCplex::MIPSolverCplex()
{
    // Should not be called
}

MIPSolverCplex::MIPSolverCplex(EnvironmentPtr envPtr)
{
    env = envPtr;

    initializeProblem();
}

MIPSolverCplex::~MIPSolverCplex()
{
    cplexVarConvers.clear();
    cplexModel.end();
    cplexVars.end();
    cplexConstrs.end();
    cplexInstance.end();
    cplexEnv.end();
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

    checkParameters();
    modelUpdated = false;

    return (true);
}

bool MIPSolverCplex::addVariable(std::string name, E_VariableType type, double lowerBound, double upperBound)
{
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
        {
            isProblemDiscrete = true;
            auto cplexVar = IloSemiContVar(cplexEnv, lowerBound, upperBound, ILOFLOAT, name.c_str());
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

    return (true);
}

bool MIPSolverCplex::finalizeObjective(bool isMinimize, double constant)
{
    try
    {
        if(constant != 0.0)
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
            cplexConstrs.add(tmpRange);
            cplexModel.add(tmpRange);
        }
        else
        {
            IloRange tmpRange = IloRange(cplexEnv, valueRHS, constrExpression, valueLHS, name.c_str());
            cplexConstrs.add(tmpRange);
            cplexModel.add(tmpRange);
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

            if(env->results->iterations.size() > 0)
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
        // Disable Cplex output
        if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            cplexInstance.setOut(cplexEnv.getNullStream());

            if(env->settings->getSetting<int>("Console.LogLevel", "Output") >= static_cast<int>(E_LogLevel::Debug))
            {
                cplexInstance.setWarning(cplexEnv.getNullStream());
            }
        }

        cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::MIPGap,
            env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") / 1.0);
        cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap,
            env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);

        if(env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
        {
            if(env->results->iterations.size() == 0)
                cplexInstance.setParam(
                    IloCplex::IntSolLim, env->settings->getSetting<int>("MIP.SolutionLimit.Initial", "Dual"));
        }
        else
        {
            cplexInstance.setParam(IloCplex::IntSolLim, 21000000);
        }

        cplexInstance.setParam(IloCplex::SolnPoolIntensity,
            env->settings->getSetting<int>("Cplex.SolnPoolIntensity", "Subsolver")); // Don't use 3 with heuristics
        cplexInstance.setParam(
            IloCplex::SolnPoolReplace, env->settings->getSetting<int>("Cplex.SolnPoolReplace", "Subsolver"));
        cplexInstance.setParam(
            IloCplex::SolnPoolGap, env->settings->getSetting<double>("Cplex.SolnPoolGap", "Subsolver"));
        cplexInstance.setParam(
            IloCplex::SolnPoolCapacity, env->settings->getSetting<int>("MIP.SolutionPool.Capacity", "Dual"));

        cplexInstance.setParam(IloCplex::Probe, env->settings->getSetting<int>("Cplex.Probe", "Subsolver"));
        cplexInstance.setParam(IloCplex::MIPEmphasis, env->settings->getSetting<int>("Cplex.MIPEmphasis", "Subsolver"));

        cplexInstance.setParam(
            IloCplex::ParallelMode, env->settings->getSetting<int>("Cplex.ParallelMode", "Subsolver"));
        cplexInstance.setParam(IloCplex::Threads, env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual"));

        cplexInstance.setParam(
            IloCplex::NumericalEmphasis, env->settings->getSetting<int>("Cplex.NumericalEmphasis", "Subsolver"));
        cplexInstance.setParam(
            IloCplex::MemoryEmphasis, env->settings->getSetting<int>("Cplex.MemoryEmphasis", "Subsolver"));

        // Sets whether CPLEX allows nonconvex quadratic functions
        cplexInstance.setParam(
            IloCplex::Param::OptimalityTarget, env->settings->getSetting<int>("Cplex.OptimalityTarget", "Subsolver"));

        // Options for using swap file
        cplexInstance.setParam(
            IloCplex::WorkDir, env->settings->getSetting<std::string>("Cplex.WorkDir", "Subsolver").c_str());
        cplexInstance.setParam(IloCplex::WorkMem, env->settings->getSetting<double>("Cplex.WorkMem", "Subsolver"));
        cplexInstance.setParam(IloCplex::NodeFileInd, env->settings->getSetting<int>("Cplex.NodeFileInd", "Subsolver"));

        cplexInstance.setParam(IloCplex::FeasOptMode, 2);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Cplex error when initializing parameters for linear solver", e.getMessage());
    }
}

int MIPSolverCplex::addLinearConstraint(
    const std::vector<PairIndexValue>& elements, double constant, std::string name, bool isGreaterThan)
{
    try
    {
        IloExpr expr(cplexEnv);

        for(auto E : elements)
        {
            expr += E.value * cplexVars[E.index];
        }

        if(isGreaterThan)
        {
            IloRange tmpRange(cplexEnv, -constant, expr, IloInfinity);
            tmpRange.setName(name.c_str());
            cplexConstrs.add(tmpRange);
            cplexModel.add(tmpRange);
        }
        else
        {
            IloRange tmpRange(cplexEnv, -IloInfinity, expr, -constant);
            tmpRange.setName(name.c_str());
            cplexConstrs.add(tmpRange);
            cplexModel.add(tmpRange);
        }

        modelUpdated = true;

        expr.end();
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when adding linear constraint", e.getMessage());

        return (-1);
    }

    return (cplexInstance.getNrows() - 1);
}

void MIPSolverCplex::activateDiscreteVariables(bool activate)
{
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
        else if(status == IloCplex::CplexStatus::Infeasible)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
        }
        else if(status == IloCplex::CplexStatus::InfOrUnbd)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
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
        if(modelUpdated)
        {
            cplexInstance.extract(cplexModel);
            modelUpdated = false;
        }

        cplexInstance.solve();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

bool MIPSolverCplex::repairInfeasibility()
{
    try
    {
        if(modelUpdated)
        {
            cplexInstance.extract(cplexModel);

            modelUpdated = false;
        }

        IloNumArray relax(cplexEnv);

        cplexInstance.extract(cplexModel);

        int numCurrConstraints = cplexConstrs.getSize();
        int numOrigConstraints = env->reformulatedProblem->properties.numberOfLinearConstraints;

        relax.add(numOrigConstraints, 0.0);

        int cutoffOffset = 0;

        for(int i = numOrigConstraints; i < numCurrConstraints; i++)
        {
            if(i == cutOffConstraintIndex)
            {
                relax.add(0.0);
                cutoffOffset = 1;
            }
            else if(env->dualSolver->generatedHyperplanes.at(i - numOrigConstraints - cutoffOffset).isSourceConvex)
            {
                relax.add(0);
            }
            else if(std::find(integerCuts.begin(), integerCuts.end(), i) != integerCuts.end())
            {
                relax.add(0);
            }
            else
            {
                relax.add(1 / (((double)i) + 1.0));
            }
        }

        if(cplexInstance.feasOpt(cplexConstrs, relax))
        {
            IloNumArray infeas(cplexEnv);
            IloNumArray vals(cplexEnv);

            cplexInstance.getInfeasibilities(infeas, cplexConstrs);

            int numRepairs = 0;

            for(int i = numOrigConstraints; i < infeas.getSize(); i++)
            {
                if(infeas[i] > 0)
                {
                    numRepairs++;
                    env->output->outputDebug("        Constraint: " + std::to_string(i)
                        + " repaired with infeasibility = " + std::to_string(infeas[i]));
                    cplexConstrs[i].setUB(cplexConstrs[i].getUB() + 1.5 * infeas[i]);
                }
                else if(infeas[i] < 0) // Should not happen for generated cuts
                {
                    numRepairs++;
                    env->output->outputDebug("        Constraint: " + std::to_string(i)
                        + " repaired with infeasibility = " + std::to_string(infeas[i]));
                    cplexConstrs[i].setLB(cplexConstrs[i].getLB() + 1.5 * infeas[i]);
                }
            }

            if(numRepairs == 0)
            {
                env->output->outputCritical("        Could not repair the infeasible dual problem.");
                return (false);
            }

            env->output->outputCritical("        Number of constraints modified: " + std::to_string(numRepairs));

            cplexInstance.extract(cplexModel);

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

        env->output->outputCritical("        Could not repair the infeasible dual problem.");
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when trying to repair infeasibility", e.getMessage());
    }

    return (false);
}

int MIPSolverCplex::increaseSolutionLimit(int increment)
{
    int sollim;

    try
    {
        cplexInstance.setParam(IloCplex::IntSolLim, cplexInstance.getParam(cplexInstance.IntSolLim) + increment);
        sollim = cplexInstance.getParam(cplexInstance.IntSolLim);
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
        cplexInstance.setParam(IloCplex::IntSolLim, limit);
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
        solLim = cplexInstance.getParam(cplexInstance.IntSolLim);
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
            auto cplexStatus = cplexInstance.getCplexStatus();

            if(cplexStatus == IloCplex::CplexStatus::Optimal || cplexStatus == IloCplex::CplexStatus::OptimalTol)
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
        if(seconds > 0)
        {
            cplexInstance.setParam(IloCplex::TiLim, seconds);
        }
        else
        {
            cplexInstance.setParam(IloCplex::TiLim, 0.01);
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
            cplexInstance.setParam(IloCplex::CutUp, cutOff);
            env->output->outputDebug(
                "        Setting cutoff value to " + Utilities::toString(cutOff) + " for minimization.");
        }
        else
        {
            cplexInstance.setParam(IloCplex::CutLo, cutOff);
            env->output->outputDebug(
                "        Setting cutoff value to " + Utilities::toString(cutOff) + " for maximization.");
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
                cplexConstrs.add(tmpRange);
                cplexModel.add(tmpRange);

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for maximization.");
            }
            else
            {
                IloRange tmpRange(cplexEnv, -IloInfinity, cplexObjectiveExpression, cutOff);
                tmpRange.setName("CUTOFF_C");
                cplexConstrs.add(tmpRange);
                cplexModel.add(tmpRange);

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
                cplexConstrs[cutOffConstraintIndex].setLB(cutOff);
                env->output->outputDebug(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
            }
            else
            {
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

    for(double P : point)
    {
        startVal.add(P);
    }

    for(auto& V : env->reformulatedProblem->auxiliaryVariables)
    {
        startVal.add(V->calculateAuxiliaryValue(point));
    }

    if(env->reformulatedProblem->auxiliaryObjectiveVariable)
        startVal.add(env->reformulatedProblem->auxiliaryObjectiveVariable->calculateAuxiliaryValue(point));

    try
    {
        cplexInstance.addMIPStart(cplexVars, startVal);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when adding MIP starting point", e.getMessage());
    }

    startVal.end();

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

double MIPSolverCplex::getDualObjectiveValue()
{
    double objVal = NAN;

    bool isMIP = getDiscreteVariableStatus() && isProblemDiscrete;

    try
    {
        if(isMIP)
        {
            objVal = cplexInstance.getBestObjValue();
        }
        else
        {
            objVal = cplexInstance.getObjValue();
        }
    }
    catch(IloException& e)
    {
        // Happens for infeasible LP
        if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
            return (SHOT_DBL_MIN);
        else
            return (SHOT_DBL_MAX);
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
                if(redund[j] == true)
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

void MIPSolverCplex::writePresolvedToFile(std::string filename)
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

void MIPSolverCplex::checkParameters() {}

void MIPSolverCplex::createHyperplane(
    Hyperplane hyperplane, std::function<IloConstraint(IloRange)> addConstraintFunction)
{
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    auto optional = createHyperplaneTerms(hyperplane);

    if(!optional)
    {
        return;
    }

    auto tmpPair = optional.value();

    // auto tmpPair = createHyperplaneTerms(hyperplane);
    bool hyperplaneIsOk = true;

    for(auto& E : tmpPair.first)
    {
        if(E.value != E.value) // Check for NaN
        {
            env->output->outputError("     Warning: hyperplane not generated, NaN found in linear terms for variable "
                + env->problem->getVariable(E.index)->name);
            hyperplaneIsOk = false;
            break;
        }
    }

    if(hyperplaneIsOk)
    {
        IloExpr expr(cplexEnv);

        for(auto& P : tmpPair.first)
        {
            expr += P.value * cplexVars[P.index];
        }

        IloRange tmpRange(cplexEnv, -IloInfinity, expr, -tmpPair.second);

        auto addedConstr = addConstraintFunction(tmpRange);

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
        expr.end();
    }
}

void MIPSolverCplex::createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes)
{
    IloExpr expr(cplexEnv);

    for(int I : binaryIndexesOnes)
    {
        expr += 1.0 * cplexVars[I];
    }

    for(int I : binaryIndexesZeroes)
    {
        expr += (1 - 1.0 * cplexVars[I]);
    }

    IloRange tmpRange(cplexEnv, -IloInfinity, expr, binaryIndexesOnes.size() + binaryIndexesZeroes.size() - 1.0);
    tmpRange.setName("IC");

    integerCuts.push_back(cplexConstrs.getSize());

    env->solutionStatistics.numberOfIntegerCuts++;

    cplexConstrs.add(tmpRange);
    cplexModel.add(tmpRange);

    expr.end();
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
} // namespace SHOT