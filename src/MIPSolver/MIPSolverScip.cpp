/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverScip.h"

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
#include "scip/scip.h"
#include "scip/scipdefplugins.h"

#include <string>

#define SCIP_CALL_ENV(x) \
    do                                                                                                 \
    {                                                                                                  \
	SCIP_RETCODE _restat_;                                                                         \
	if( (_restat_ = (x)) != SCIP_OKAY )                                                            \
	{                                                                                              \
	    env->output->outputError(" Error " + std::to_string(_restat_) + " in SCIP function call"); abort(); \
	    return (false);                                                                            \
	}                                                                                              \
    }                                                                                                  \
    while( FALSE )


namespace SHOT
{

MIPSolverScip::MIPSolverScip()
{
    // Should not be called
}

MIPSolverScip::MIPSolverScip(EnvironmentPtr envPtr)
{
    env = envPtr;
    scipCallback = std::make_unique<ScipCallbackMultiTree>(env);
}

MIPSolverScip::~MIPSolverScip()
{
    if( scip != nullptr )
    {
        SCIP_RETCODE retcode;

        for( SCIP_VAR* var : vars )
        {
            retcode = SCIPreleaseVar(scip, &var);
            if( retcode != SCIP_OKAY )
            {
                env->output->outputError(" Error " + std::to_string(retcode) + " in SCIPreleaseVar");
            }
        }

        retcode = SCIPfree(&scip);
        if( retcode != SCIP_OKAY )
        {
            env->output->outputError(" Error " + std::to_string(retcode) + " in SCIPfree");
        }
    }
//    objectiveLinearExpression.clear();
//    objectiveQuadraticExpression.clear();
//    constraintLinearExpression.clear();
//    constraintQuadraticExpression.clear();
}

bool MIPSolverScip::initializeProblem()
{
    discreteVariablesActivated = true;

    if(alreadyInitialized)
    {
        std::shared_ptr<SCIP> scip;
    }
    else
    {
        prevSolutionLimit = 1;
        alreadyInitialized = true;
    }

    SCIP_CALL_ENV( SCIPcreate(&scip) );
    SCIP_CALL_ENV( SCIPincludeDefaultPlugins(scip) );
    SCIP_CALL_ENV( SCIPcreateProbBasic(scip, "SHOT") );

//    SCIP_CALL_ENV( SCIPsetBoolParam(scip, "misc/transsolsorig", FALSE) );
    SCIP_CALL_ENV( SCIPsetIntParam(scip, "display/verblevel", SCIP_VERBLEVEL_NONE) );


    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();

    return (true);
}

bool MIPSolverScip::addVariable(
    std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound)
{
    if(lowerBound < -getUnboundedVariableBoundValue())
        lowerBound = -getUnboundedVariableBoundValue();

    if(upperBound > getUnboundedVariableBoundValue())
        upperBound = getUnboundedVariableBoundValue();

    SCIP_VARTYPE vartype;
    switch(type)
    {
        case E_VariableType::Real:
            vartype = SCIP_VARTYPE_CONTINUOUS;
            break;

        case E_VariableType::Integer:
            isProblemDiscrete = true;
            vartype = SCIP_VARTYPE_INTEGER;
            break;

        case E_VariableType::Binary:
            isProblemDiscrete = true;
            vartype = SCIP_VARTYPE_BINARY;
            break;

        case E_VariableType::Semicontinuous:
            isProblemDiscrete = true;
            vartype = SCIP_VARTYPE_CONTINUOUS;
//            if(semiBound < 0.0)
//                upperBound = semiBound;
//            else
//                lowerBound = semiBound;
//            ScipModel->addVar(lowerBound, upperBound, 0.0, GRB_SEMICONT, name);
            break;

        case E_VariableType::Semiinteger:
            isProblemDiscrete = true;
            vartype = SCIP_VARTYPE_INTEGER;
//            if(semiBound < 0.0)
//                upperBound = semiBound;
//            else
//                lowerBound = semiBound;
//            ScipModel->addVar(lowerBound, upperBound, 0.0, GRB_SEMIINT, name);
            break;

        default:
            break;
    }

    SCIP_VAR* var;
    SCIP_CALL_ENV( SCIPcreateVarBasic(scip, &var, name.c_str(), lowerBound, upperBound, 0.0, vartype) );
    SCIP_CALL_ENV( SCIPaddVar(scip, var) );

    vars.push_back(var);
    variableTypes.push_back(type);
    variableNames.push_back(name);
    variableLowerBounds.push_back(lowerBound);
    variableUpperBounds.push_back(upperBound);
    numberOfVariables++;
    return (true);
}

bool MIPSolverScip::initializeObjective()
{
//    try
//    {
//        objectiveQuadraticExpression = GRBQuadExpr(0);
//        objectiveLinearExpression = GRBLinExpr(0);
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError(
//            "        Scip exception caught when initializing objective function: ", e.getMessage());
//        return (false);
//    }
    // TODO reset all obj coefs to 0?

    return (true);
}

bool MIPSolverScip::addLinearTermToObjective(double coefficient, int variableIndex)
{
    assert(variableIndex < vars.size());

    SCIP_CALL_ENV( SCIPchgVarObj(scip, vars[variableIndex], coefficient) );

    return (true);
}

bool MIPSolverScip::addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
    //TODO
//    try
//    {
//        objectiveQuadraticExpression
//            += coefficient * ScipModel->getVar(firstVariableIndex) * ScipModel->getVar(secondVariableIndex);
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError(
//            "        Scip exception caught when adding quadratic term to objective: ", e.getMessage());
//        return (false);
//    }

    hasQuadraticObjective = true;

    return (true);
}

bool MIPSolverScip::finalizeObjective(bool isMinimize, double constant)
{
    if( constant != 0.0 )
    {
        SCIP_CALL_ENV( SCIPaddObjoffset(scip, constant) );
    }

    SCIP_CALL_ENV( SCIPsetObjsense(scip, isMinimize ? SCIP_OBJSENSE_MINIMIZE : SCIP_OBJSENSE_MAXIMIZE) );
    isMinimizationProblem = (isMinimize == SCIP_OBJSENSE_MINIMIZE);

    return (true);
}

bool MIPSolverScip::initializeConstraint()
{
    assert(lincons == nullptr);

    SCIP_CALL_ENV( SCIPcreateConsBasicLinear(scip, &lincons, "lincons", 0, nullptr, nullptr, -SCIPinfinity(scip), SCIPinfinity(scip)) );

    return (true);
}

bool MIPSolverScip::addLinearTermToConstraint(double coefficient, int variableIndex)
{
    SCIP_CALL_ENV( SCIPaddCoefLinear(scip, lincons, vars.at(variableIndex), coefficient) );

    return (true);
}

bool MIPSolverScip::addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
//    try
//    {
//        constraintQuadraticExpression
//            += coefficient * ScipModel->getVar(firstVariableIndex) * ScipModel->getVar(secondVariableIndex);
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError(
//            "        Scip exception caught when adding quadratic term to constraint: ", e.getMessage());
//        return (false);
//    }

    hasQudraticConstraint = true;

    return (true);
}

bool MIPSolverScip::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
{
    assert(lincons != nullptr);

    if( valueLHS > SHOT_DBL_MIN )
        valueLHS -= constant;
    else
        valueLHS = -SCIPinfinity(scip);

    if( valueRHS < SHOT_DBL_MAX )
        valueRHS -= constant;
    else
        valueRHS = SCIPinfinity(scip);

    //SCIPconsSetName(lincons, name.c_str());

    SCIP_CALL_ENV( SCIPchgLhsLinear(scip, lincons, valueLHS) );
    SCIP_CALL_ENV( SCIPchgRhsLinear(scip, lincons, valueRHS) );

//        if(constraintQuadraticExpression.size() == 0)
//            allowRepairOfConstraint.push_back(false);

    if( SCIPgetStage(scip) >= SCIP_STAGE_INITPRESOLVE )
    {
        SCIP_CALL_ENV( SCIPfreeTransform(scip));
    }

    SCIP_CALL_ENV( SCIPaddCons(scip, lincons) );
    SCIP_CALL_ENV( SCIPreleaseCons(scip, &lincons) );

    numberOfConstraints++;
    return (true);
}

bool MIPSolverScip::finalizeProblem()
{
    if(env->settings->getSetting<bool>("Dual.TreeStrategy.Multi.Reinitialize"))
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
            setSolLimit = env->settings->getSetting<int>("Dual.MIP.SolutionLimit.Initial");
        }

        setSolutionLimit(setSolLimit);

        if(!discreteVariablesActivated)
        {
            activateDiscreteVariables(false);
        }
    }

    modelUpdated = true;

    return (true);
}

void MIPSolverScip::initializeSolverSettings()
{
    // Console output controlled in callback, but need to disable it so messages won't appear twice
    //ScipModel->set(GRB_IntParam_LogToConsole, 0);

    // Set termination tolerances
    SCIP_CALL_ABORT( SCIPsetRealParam(scip, "limits/gap", env->settings->getSetting<double>("Termination.ObjectiveGap.Relative")) );
    SCIP_CALL_ABORT( SCIPsetRealParam(scip, "limits/absgap", env->settings->getSetting<double>("Termination.ObjectiveGap.Absolute")) );
    SCIP_Real feastol = std::min(env->settings->getSetting<double>("Primal.Tolerance.LinearConstraint"), env->settings->getSetting<double>("Primal.Tolerance.Integer"));
    SCIP_CALL_ABORT( SCIPsetRealParam(scip, "numerics/feastol", feastol) );
    SCIP_CALL_ABORT( SCIPsetRealParam(scip, "numerics/dualfeastol", env->settings->getSetting<double>("Dual.MIP.OptimalityTolerance")) );

    // Add a user-provided node limit
    if(auto nodeLimit = env->settings->getSetting<double>("Dual.MIP.NodeLimit"); nodeLimit > 0)
    {
//        SCIP_CALL_ABORT( SCIPsetLongintParam(scip, "limits/totalnodes", static_cast<SCIP_Longint>(nodeLimit)) );
    }
    // Set solution pool settings
    SCIP_CALL_ABORT( SCIPsetIntParam(scip, "limits/solutions", -1) );
    SCIP_CALL_ABORT( SCIPsetIntParam(scip, "limits/maxsol", env->settings->getSetting<int>("Dual.MIP.SolutionPool.Capacity") + 1) );

    // Set solver emphasis
//    if( env->settings->getSetting<int>("Subsolver.Scip.NumericFocus") )
//    {
//        SCIP_CALL_ABORT( SCIPsetEmphasis(scip, SCIP_PARAMEMPHASIS_NUMERICS, FALSE) );
//        //TODO we cannot undo this setting easily
//    }

    // Set parameters for quadratics
    //        ScipModel->set(GRB_DoubleParam_PSDTol,
    //            env->settings->getSetting<double>("Model.Convexity.Quadratics.EigenValueTolerance"));

    // Set various solver specific MIP settings
    //ScipModel->set(GRB_IntParam_MIPFocus, env->settings->getSetting<int>("Subsolver.Scip.MIPFocus"));
    //ScipModel->set(
    //    GRB_DoubleParam_Heuristics, env->settings->getSetting<double>("Subsolver.Scip.Heuristics"));

    // Set number of threads
    //ScipModel->set(GRB_IntParam_Threads, env->settings->getSetting<int>("Dual.MIP.NumberOfThreads"));
}

int MIPSolverScip::addLinearConstraint(
    const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
{
    SCIP_RETCODE retcode;
    SCIP_VAR** consvars = nullptr;
    SCIP_Real* conscoefs = nullptr;
    int ncoefs;
    int i = 0;

    ncoefs = elements.size();

    if( SCIPgetStage(scip) >= SCIP_STAGE_INITPRESOLVE )
    {
        SCIP_CALL_TERMINATE(retcode, SCIPfreeTransform(scip), TERMINATE);
    }

    SCIP_CALL_TERMINATE( retcode, SCIPallocBufferArray(scip, &consvars, ncoefs), TERMINATE );
    SCIP_CALL_TERMINATE( retcode, SCIPallocBufferArray(scip, &conscoefs, ncoefs), TERMINATE );

    for( const auto& e : elements )
    {
        assert(i < ncoefs);
        consvars[i] = vars.at(e.first);
        conscoefs[i] = e.second;
        ++i;
    }

    //int numConstraintsBefore = ScipModel->get(GRB_IntAttr_NumConstrs);

    assert(lincons == nullptr);
    SCIP_CALL_TERMINATE(retcode, SCIPcreateConsBasicLinear(scip, &lincons, name.c_str(), ncoefs, consvars, conscoefs,
                                                           isGreaterThan ? -constant : -SCIPinfinity(scip), isGreaterThan ? SCIPinfinity(scip) : -constant), TERMINATE);
//    SCIPprintCons(scip, lincons, NULL);
//    SCIPinfoMessage(scip, NULL, "\n");
    SCIP_CALL_TERMINATE(retcode, SCIPaddCons(scip, lincons), TERMINATE);
    SCIP_CALL_TERMINATE(retcode, SCIPreleaseCons(scip, &lincons), TERMINATE);


//        if(ScipModel->get(GRB_IntAttr_NumConstrs) > numConstraintsBefore)
//        {
//            allowRepairOfConstraint.push_back(allowRepair);
//        }
//        else
//        {
//            env->output->outputInfo("        Hyperplane not added by Scip");
//            return (-1);
//        }

    TERMINATE:

    SCIPfreeBufferArray(scip, &conscoefs);
    SCIPfreeBufferArray(scip, &consvars);

    if( retcode != SCIP_OKAY )
    {
        env->output->outputError("        Error " + std::to_string(retcode) + " when adding linear constraint");
        return (-1);
    }

    //return (ScipModel->get(GRB_IntAttr_NumConstrs) - 1);
    return SCIPgetNConss(scip) - 1;
}

bool MIPSolverScip::addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights)
{
//    try
//    {
//        std::vector<GRBVar> variables;
//
//        for(auto I : variableIndexes)
//            variables.push_back(ScipModel->getVar(I));
//
//        if(variableWeights.size() == 0)
//        {
//            variableWeights.resize(variableIndexes.size());
//
//            for(size_t i = 0; i < variableIndexes.size(); i++)
//                variableWeights[i] = i;
//        }
//
//        assert(variableWeights.size() == variableIndexes.size());
//
//        ScipModel->addSOS(&variables[0], &variableWeights[0], variables.size(),
//            (type == E_SOSType::One) ? GRB_SOS_TYPE1 : GRB_SOS_TYPE2);
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError("        Error when adding special ordered set constraint", e.getMessage());
//        return (false);
//    }
//
//    return (true);
    return (false);
}

bool MIPSolverScip::createIntegerCut(IntegerCut& integerCut)
{
    bool allowIntegerCutRepair = env->settings->getSetting<bool>("Dual.MIP.InfeasibilityRepair.IntegerCuts");

//    try
//    {
//        int numConstraintsBefore = ScipModel->get(GRB_IntAttr_NumConstrs);
//        GRBLinExpr expr = 0;
//        size_t index = 0;
//
//        // Verify that no integer values are outside of variable bounds
//        for(size_t i = 0; i < integerCut.variableIndexes.size(); i++)
//        {
//            auto VAR = env->reformulatedProblem->getVariable(integerCut.variableIndexes[i]);
//            int variableValue = integerCut.variableValues[i];
//
//            if(variableValue < VAR->lowerBound || variableValue > VAR->upperBound)
//                return (false);
//        }
//
//        for(auto& I : integerCut.variableIndexes)
//        {
//            auto VAR = env->reformulatedProblem->getVariable(I);
//            int variableValue = integerCut.variableValues[index];
//            auto variable = ScipModel->getVar(I);
//
//            assert(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
//                || VAR->properties.type == E_VariableType::Semiinteger);
//
//            if(variableValue == VAR->upperBound)
//            {
//                expr += (variableValue - variable);
//            }
//            else if(variableValue == VAR->lowerBound)
//            {
//                expr += variable;
//            }
//            else
//            {
//                numberOfVariables += 2;
//
//                auto w = ScipModel->addVar(0, getUnboundedVariableBoundValue(), 0.0, GRB_CONTINUOUS,
//                    fmt::format("wIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));
//                auto v = ScipModel->addVar(
//                    0, 1, 0.0, GRB_BINARY, fmt::format("vIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));
//                ScipModel->update();
//
//                expr += 1.0 * w;
//
//                double M1 = 2 * (variableValue - VAR->lowerBound);
//                double M2 = 2 * (VAR->upperBound - variableValue);
//
//                int tmpNumConstraints = ScipModel->get(GRB_IntAttr_NumConstrs);
//                ScipModel->addConstr(-w <= variable - variableValue,
//                    fmt::format("IC{}_{}_1a", env->solutionStatistics.numberOfIntegerCuts, index));
//                ScipModel->update();
//
//                if(ScipModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
//                {
//                    integerCuts.push_back(numConstraintsBefore + index);
//                    allowRepairOfConstraint.push_back(false);
//                }
//
//                tmpNumConstraints = ScipModel->get(GRB_IntAttr_NumConstrs);
//                ScipModel->addConstr(variable - variableValue <= w,
//                    fmt::format("IC{}_{}_1b", env->solutionStatistics.numberOfIntegerCuts, index));
//                ScipModel->update();
//
//                if(ScipModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
//                {
//                    integerCuts.push_back(numConstraintsBefore + index);
//                    allowRepairOfConstraint.push_back(false);
//                }
//
//                tmpNumConstraints = ScipModel->get(GRB_IntAttr_NumConstrs);
//                ScipModel->addConstr(w <= variable - variableValue + M1 * (1 - v),
//                    fmt::format("IC{}_{}_2", env->solutionStatistics.numberOfIntegerCuts, index));
//                ScipModel->update();
//
//                if(ScipModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
//                {
//                    integerCuts.push_back(numConstraintsBefore + index);
//                    allowRepairOfConstraint.push_back(false);
//                }
//
//                tmpNumConstraints = ScipModel->get(GRB_IntAttr_NumConstrs);
//                ScipModel->addConstr(w <= variableValue - variable + M2 * v,
//                    fmt::format("IC{}_{}_3", env->solutionStatistics.numberOfIntegerCuts, index));
//                ScipModel->update();
//
//                if(ScipModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
//                {
//                    integerCuts.push_back(numConstraintsBefore + index);
//                    allowRepairOfConstraint.push_back(false);
//                }
//            }
//
//            index++;
//        }
//
//        int tmpNumConstraints = ScipModel->get(GRB_IntAttr_NumConstrs);
//        ScipModel->addConstr(expr >= 1, fmt::format("IC{}_4", env->solutionStatistics.numberOfIntegerCuts));
//        ScipModel->update();
//
//        if(ScipModel->get(GRB_IntAttr_NumConstrs) > tmpNumConstraints)
//        {
//            integerCuts.push_back(numConstraintsBefore + index);
//            allowRepairOfConstraint.push_back(allowIntegerCutRepair);
//        }
//
//        ScipModel->update();
//
//        auto addedConstraints = ScipModel->get(GRB_IntAttr_NumConstrs) - numConstraintsBefore;
//
//        if(addedConstraints == 0)
//        {
//            env->output->outputInfo("        Integer cut not added by Scip");
//            return (false);
//        }
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError("        Scip error when adding integer cut", e.getMessage());
//        return (false);
//    }
//
//    return (true);
    return (false);
}

VectorDouble MIPSolverScip::getVariableSolution(int solIdx)
{
    SCIP_RETCODE retcode;

//    int numVar = ScipModel->get(GRB_IntAttr_NumVars);
    VectorDouble solution(vars.size());

    assert(solIdx < SCIPgetNSols(scip));

    SCIP_CALL_TERMINATE(retcode, SCIPgetSolVals(scip, SCIPgetSols(scip)[solIdx], vars.size(), vars.data(), solution.data()), TERMINATE);

    TERMINATE:
    if( retcode != SCIP_OKAY )
    {
        env->output->outputError(
            "        Error when reading solution with index " + std::to_string(solIdx));
    }

    return (solution);
}

int MIPSolverScip::getNumberOfSolutions()
{
    return SCIPgetNSols(scip);
}

void MIPSolverScip::activateDiscreteVariables(bool activate)
{
    SCIP_Bool infeas;

    if(env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0
        || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
        return;

    if( SCIPgetStage(scip) >= SCIP_STAGE_INITPRESOLVE )
    {
        SCIP_CALL_ABORT( SCIPfreeTransform(scip) );
    }

    if(activate)
    {
        env->output->outputDebug("        Activating MIP strategy.");

        for(int i = 0; i < numberOfVariables; i++)
        {
            assert(variableTypes.at(i) != E_VariableType::Semicontinuous
                && variableTypes.at(i) != E_VariableType::Semiinteger);

            switch(variableTypes.at(i))
            {
                case E_VariableType::Semiinteger:
                case E_VariableType::Integer:
                    SCIP_CALL_ABORT( SCIPchgVarType(scip, vars.at(i), SCIP_VARTYPE_INTEGER, &infeas) );
                    assert(!infeas);
                    break;

                case E_VariableType::Binary:
                    SCIP_CALL_ABORT( SCIPchgVarType(scip, vars.at(i), SCIP_VARTYPE_BINARY, &infeas) );
                    assert(!infeas);
                    break;

                default:
                    break;
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

            if( SCIPvarGetType(vars.at(i)) != SCIP_VARTYPE_CONTINUOUS )
            {
                SCIP_CALL_ABORT( SCIPchgVarType(scip, vars.at(i), SCIP_VARTYPE_CONTINUOUS, &infeas) );
                assert(!infeas);
            }
        }

        discreteVariablesActivated = false;
    }

    modelUpdated = true;
}

E_ProblemSolutionStatus MIPSolverScip::getSolutionStatus()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    SCIP_STATUS status = SCIPgetStatus(scip);

    switch(status)
    {
        case SCIP_STATUS_UNKNOWN:
            MIPSolutionStatus = E_ProblemSolutionStatus::Error;
            break;

        case SCIP_STATUS_OPTIMAL:
            MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
            break;

        case SCIP_STATUS_INFEASIBLE:
            MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
            break;

        case SCIP_STATUS_UNBOUNDED:
        case SCIP_STATUS_INFORUNBD:
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
            break;

        case SCIP_STATUS_USERINTERRUPT:
        case SCIP_STATUS_TERMINATE:
            MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
            break;

        case SCIP_STATUS_NODELIMIT:
        case SCIP_STATUS_TOTALNODELIMIT:
        case SCIP_STATUS_STALLNODELIMIT:
            MIPSolutionStatus = E_ProblemSolutionStatus::NodeLimit;
            break;

        case SCIP_STATUS_TIMELIMIT:
            MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
            break;

        case SCIP_STATUS_MEMLIMIT:
            MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
            break;

        case SCIP_STATUS_GAPLIMIT:
            MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
            break;

        case SCIP_STATUS_PRIMALLIMIT:
            MIPSolutionStatus = E_ProblemSolutionStatus::CutOff;
            break;

        case SCIP_STATUS_DUALLIMIT:
            // status should not come up since no dual limit is set
            MIPSolutionStatus = E_ProblemSolutionStatus::Error;
            break;

        case SCIP_STATUS_SOLLIMIT:
        case SCIP_STATUS_BESTSOLLIMIT:
            MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
            break;

        case SCIP_STATUS_RESTARTLIMIT:
            // status should not come up since no restart limit is set
            MIPSolutionStatus = E_ProblemSolutionStatus::Error;
            break;

        default:
            env->output->outputError("        MIP solver return status " + std::to_string(status));
            MIPSolutionStatus = E_ProblemSolutionStatus::Error;
            break;
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverScip::solveProblem()
{
    SCIP_RETCODE retcode;
    E_ProblemSolutionStatus MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    cachedSolutionHasChanged = true;

    if( SCIPgetStage(scip) >= SCIP_STAGE_INITPRESOLVE )
    {
        SCIP_CALL_TERMINATE(retcode, SCIPfreeTransform(scip), TERMINATE);
    }

    // ScipModel->setCallback(ScipCallback.get());
    SCIP_CALL_TERMINATE(retcode, SCIPsolve(scip), TERMINATE);

    MIPSolutionStatus = getSolutionStatus();

    // To find a feasible point for an unbounded dual problem and not when solving the minimax-problem
//    if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded && env->results->getNumberOfIterations() > 0)
//    {
//        std::vector<PairIndexValue> originalObjectiveCoefficients;
//        bool problemUpdated = false;
//
//        if((env->reformulatedProblem->objectiveFunction->properties.classification
//                   == E_ObjectiveFunctionClassification::Linear
//               && std::dynamic_pointer_cast<LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
//                      ->isDualUnbounded())
//            || (env->reformulatedProblem->objectiveFunction->properties.classification
//                    == E_ObjectiveFunctionClassification::Quadratic
//                && std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
//                       ->isDualUnbounded()))
//        {
//            for(auto& V : env->reformulatedProblem->allVariables)
//            {
//                if(!V->properties.inObjectiveFunction)
//                    continue;
//
//                if(V->isDualUnbounded())
//                {
//                    // Temporarily remove unbounded terms from objective
//                    originalObjectiveCoefficients.emplace_back(
//                        V->index, ScipModel->getVar(V->index).get(GRB_DoubleAttr_Obj));
//
//                    ScipModel->getVar(V->index).set(GRB_DoubleAttr_Obj, 0.0);
//                    problemUpdated = true;
//                }
//            }
//        }
//        else if(env->reformulatedProblem->objectiveFunction->properties.classification
//                >= E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear
//            && hasDualAuxiliaryObjectiveVariable())
//        {
//            // The auxiliary variable in the dual problem is unbounded
//            updateVariableBound(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue() / 1.1,
//                getUnboundedVariableBoundValue() / 1.1);
//            problemUpdated = true;
//        }
//
//        if(problemUpdated)
//        {
//            ScipModel->update();
//            ScipModel->setCallback(ScipCallback.get());
//            ScipModel->optimize();
//
//            MIPSolutionStatus = getSolutionStatus();
//
//            for(auto& P : originalObjectiveCoefficients)
//                ScipModel->getVar(P.index).set(GRB_DoubleAttr_Obj, P.value);
//
//            ScipModel->update();
//
//            if(env->results->iterations.size() > 0) // Might not have iterations if we are using the minimax solver
//                env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
//        }
//    }

    TERMINATE:

    if( retcode != SCIP_OKAY )
        env->output->outputError("        Error " + std::to_string(retcode) + " when solving MIP/LP problem");

    return (MIPSolutionStatus);
}

bool MIPSolverScip::repairInfeasibility()
{
//    if(env->dualSolver->generatedHyperplanes.size() == 0)
//        return (false);
//
//    try
//    {
//        ScipModel->update();
//        auto feasModel = GRBModel(*ScipModel);
//
//        // Scip copies over the cutoff from the original model
//        if(isMinimizationProblem)
//            feasModel.set(GRB_DoubleParam_Cutoff, SHOT_DBL_MAX);
//        else
//            feasModel.set(GRB_DoubleParam_Cutoff, SHOT_DBL_MIN);
//
//        int numOrigConstraints = env->reformulatedProblem->properties.numberOfLinearConstraints;
//        int numOrigVariables = ScipModel->get(GRB_IntAttr_NumVars);
//        int numCurrConstraints = feasModel.get(GRB_IntAttr_NumConstrs);
//
//        std::vector<GRBConstr> repairConstraints;
//        std::vector<GRBConstr> originalConstraints;
//        VectorDouble relaxParameters;
//        int numConstraintsToRepair = 0;
//
//        for(int i = numOrigConstraints; i < numCurrConstraints; i++)
//        {
//            if(allowRepairOfConstraint[i])
//            {
//                repairConstraints.push_back(feasModel.getConstr(i));
//                originalConstraints.push_back(ScipModel->getConstr(i));
//                relaxParameters.push_back(1 / (((double)i - numOrigConstraints) + 1.0));
//                numConstraintsToRepair++;
//            }
//        }
//
//        // Saves the relaxation weights to a file
//        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
//        {
//            VectorString constraints(relaxParameters.size());
//
//            for(size_t i = 0; i < relaxParameters.size(); i++)
//            {
//                std::ostringstream expression;
//                constraints[i] = repairConstraints[i].get(GRB_StringAttr_ConstrName);
//            }
//
//            auto filename = fmt::format("{}/dualiter{}_infeasrelaxweights.txt",
//                env->settings->getSetting<std::string>("Output.Debug.Path"),
//                env->results->getCurrentIteration()->iterationNumber - 1);
//
//            Utilities::saveVariablePointVectorToFile(relaxParameters, constraints, filename);
//        }
//
//        // Scip modifies the value when running feasModel.optimize()
//        int numConstraintsToRepairOrig = numConstraintsToRepair;
//
//        if(feasModel.feasRelax(GRB_FEASRELAX_LINEAR, false, 0, nullptr, nullptr, nullptr, numConstraintsToRepair,
//               &repairConstraints[0], &relaxParameters[0])
//            < 0)
//        {
//            env->output->outputDebug("        Could not repair the infeasible dual problem.");
//            return (false);
//        }
//
//        feasModel.optimize();
//
//        // Saves the relaxation model to file
//        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
//        {
//            auto filename = fmt::format("{}/dualiter{}_infeasrelax.lp",
//                env->settings->getSetting<std::string>("Output.Debug.Path"),
//                env->results->getCurrentIteration()->iterationNumber - 1);
//
//            try
//            {
//                feasModel.write(filename);
//            }
//            catch(GRBException& e)
//            {
//                env->output->outputError("        Error when saving model to file", e.getMessage());
//            }
//        }
//
//        int status = feasModel.get(GRB_IntAttr_Status);
//
//        if(status != GRB_OPTIMAL)
//        {
//            env->output->outputDebug("        Could not repair the infeasible dual problem.");
//            return (false);
//        }
//
//        int numRepairs = 0;
//
//        for(int i = 0; i < numConstraintsToRepairOrig; i++)
//        {
//            auto variable = feasModel.getVar(numOrigVariables + i);
//            double slackValue = variable.get(GRB_DoubleAttr_X);
//
//            if(slackValue == 0.0)
//                continue;
//
//            auto constraint = originalConstraints.at(i);
//            double oldRHS = constraint.get(GRB_DoubleAttr_RHS);
//            constraint.set(GRB_DoubleAttr_RHS, oldRHS + 1.5 * slackValue);
//
//            numRepairs++;
//
//            env->output->outputDebug("        Constraint: " + constraint.get(GRB_StringAttr_ConstrName)
//                + " repaired with infeasibility = " + std::to_string(1.5 * slackValue));
//        }
//
//        env->results->getCurrentIteration()->numberOfInfeasibilityRepairedConstraints = numRepairs;
//
//        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
//        {
//            auto filename = fmt::format("{}/dualiter{}_infeasrelax.lp",
//                env->settings->getSetting<std::string>("Output.Debug.Path"),
//                env->results->getCurrentIteration()->iterationNumber - 1);
//
//            writeProblemToFile(filename);
//        }
//
//        if(numRepairs == 0)
//        {
//            env->output->outputDebug("        Could not repair the infeasible dual problem.");
//            return (false);
//        }
//
//        env->output->outputDebug("        Number of constraints modified: " + std::to_string(numRepairs));
//
//        return (true);
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError("        Error when trying to repair infeasibility",
//            e.getMessage() + " (" + std::to_string(e.getErrorCode()) + ")");
//    }

    return (false);
}

int MIPSolverScip::increaseSolutionLimit(int increment)
{
    int sollimit = -1;
    SCIP_CALL_ABORT( SCIPgetIntParam(scip, "limits/solutions", &sollimit) );
    if( sollimit >= 0 )
    {
        SCIP_CALL_ABORT( SCIPsetIntParam(scip, "limits/solutions", sollimit + increment) );
        return sollimit + increment;
    }
    else
        return INT_MAX;
}

void MIPSolverScip::setSolutionLimit(long limit)
{
    SCIP_CALL_ABORT( SCIPsetIntParam(scip, "limits/solutions", limit > INT_MAX ? -1 : limit) );
}

int MIPSolverScip::getSolutionLimit() {
    int sollimit = -1;
    SCIP_CALL_ABORT( SCIPgetIntParam(scip, "limits/solutions", &sollimit) );
    return sollimit;
}

void MIPSolverScip::setTimeLimit(double seconds)
{
    if( SCIPisInfinity(scip, seconds) )
    {
        SCIP_CALL_ABORT( SCIPsetRealParam(scip, "limits/time", SCIPinfinity(scip)) );
    }
    else
    {
        SCIP_CALL_ABORT( SCIPsetRealParam(scip, "limits/time", seconds > 0.0 ? seconds : 0.00001) );
    }
}

void MIPSolverScip::setCutOff(double cutOff)
{
    if( SCIPisInfinity(scip, std::abs(cutOff)) )
        return;

    double cutOffTol = env->settings->getSetting<double>("Dual.MIP.CutOff.Tolerance");
    if(isMinimizationProblem)
        cutOff += cutOffTol;
    else
        cutOff -= cutOffTol;

    env->output->outputDebug(fmt::format("        Setting primal bound limit to  {}.", cutOff));

    SCIP_CALL_ABORT( SCIPsetRealParam(scip, "limits/primal", cutOff) );
}

void MIPSolverScip::setCutOffAsConstraint(double cutOff)
{
//    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
//        return;
//
//    try
//    {
//        if(!cutOffConstraintDefined)
//        {
//            if(env->reformulatedProblem->objectiveFunction->properties.isMaximize)
//            {
//                ScipModel->addConstr(-objectiveLinearExpression <= -cutOff, "CUTOFF_C");
//
//                env->output->outputDebug(
//                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for maximization.");
//            }
//            else
//            {
//                ScipModel->addConstr(objectiveLinearExpression <= cutOff, "CUTOFF_C");
//
//                env->output->outputDebug(
//                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
//            }
//
//            allowRepairOfConstraint.push_back(false);
//            ScipModel->update();
//            modelUpdated = false;
//
//            cutOffConstraintDefined = true;
//            cutOffConstraintIndex = ScipModel->get(GRB_IntAttr_NumConstrs) - 1;
//        }
//        else
//        {
//            auto constraint = ScipModel->getConstr(cutOffConstraintIndex);
//
//            if(env->reformulatedProblem->objectiveFunction->properties.isMaximize)
//            {
//
//                if(hasDualAuxiliaryObjectiveVariable())
//                    constraint.set(GRB_DoubleAttr_RHS, -cutOff);
//                else
//                    constraint.set(
//                        GRB_DoubleAttr_RHS, -(cutOff - env->reformulatedProblem->objectiveFunction->constant));
//
//                env->output->outputDebug(
//                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
//            }
//            else
//            {
//
//                if(hasDualAuxiliaryObjectiveVariable())
//                    constraint.set(GRB_DoubleAttr_RHS, cutOff);
//                else
//                    constraint.set(GRB_DoubleAttr_RHS, cutOff - env->reformulatedProblem->objectiveFunction->constant);
//
//                env->output->outputDebug(
//                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
//            }
//
//            modelUpdated = true;
//        }
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError("        Error when setting cut off value through constraint", e.getMessage());
//    }
}

void MIPSolverScip::addMIPStart(VectorDouble point)
{
    assert(point.size() == env->dualSolver->MIPSolver->getNumberOfVariables());
    assert(variableNames.size() == point.size());

//    try
//    {
//        VectorDouble startVal;
//
//        for(double P : point)
//            startVal.push_back(P);
//
//        for(size_t i = 0; i < startVal.size(); i++)
//        {
//            GRBVar tmpVar = ScipModel->getVar(i);
//            tmpVar.set(GRB_DoubleAttr_Start, startVal.at(i));
//        }
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError("        Error when adding MIP starting point", e.getMessage());
//    }
//
//    env->output->outputDebug("        Added MIP starting point.");
}

void MIPSolverScip::writeProblemToFile(std::string filename)
{
    SCIP_CALL_ABORT( SCIPwriteOrigProblem(scip, filename.c_str(), NULL, FALSE) );
}

double MIPSolverScip::getObjectiveValue(int solIdx)
{
    assert(solIdx < SCIPgetNSols(scip));
    return SCIPsolGetOrigObj(SCIPgetSols(scip)[solIdx]);
}

void MIPSolverScip::deleteMIPStarts()
{
//    int numVar = ScipModel->get(GRB_IntAttr_NumVars);
//
//    try
//    {
//        for(int i = 0; i < numVar; i++)
//        {
//            GRBVar tmpVar = ScipModel->getVar(i);
//            tmpVar.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
//        }
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError("        Error when deleting MIP starting points", e.getMessage());
//    }
//
//    env->output->outputDebug("        Deleted MIP starting points.");
}

void MIPSolverScip::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverScip::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if( currentVariableBounds.first != lowerBound )
    {
        SCIP_CALL_ABORT( SCIPchgVarLb(scip, vars[varIndex], lowerBound) );
    }
    if( currentVariableBounds.second != upperBound )
    {
        SCIP_CALL_ABORT( SCIPchgVarUb(scip, vars[varIndex], upperBound) );
    }
}

void MIPSolverScip::updateVariableLowerBound(int varIndex, double lowerBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first != lowerBound)
    {
        SCIP_CALL_ABORT( SCIPchgVarLb(scip, vars[varIndex], lowerBound) );
    }
}

void MIPSolverScip::updateVariableUpperBound(int varIndex, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.second != upperBound)
    {
        SCIP_CALL_ABORT( SCIPchgVarUb(scip, vars[varIndex], upperBound) );
    }
}

PairDouble MIPSolverScip::getCurrentVariableBounds(int varIndex)
{
    PairDouble tmpBounds;

    tmpBounds.first = SCIPvarGetLbOriginal(vars[varIndex]);
    tmpBounds.second = SCIPvarGetUbOriginal(vars[varIndex]);

    return (tmpBounds);
}

bool MIPSolverScip::supportsQuadraticObjective() { return (false); }
bool MIPSolverScip::supportsQuadraticConstraints() { return (false); }

double MIPSolverScip::getUnboundedVariableBoundValue() { return SCIPinfinity(scip); }

double MIPSolverScip::getDualObjectiveValue()
{
    return SCIPgetDualbound(scip);
}

void MIPSolverScip::writePresolvedToFile(std::string filename) {
    SCIP_CALL_ABORT( SCIPwriteTransProblem(scip, filename.c_str(), NULL, FALSE) );
}

void MIPSolverScip::checkParameters() { }

std::pair<VectorDouble, VectorDouble> MIPSolverScip::presolveAndGetNewBounds()
{
    // TODO
    // auto m = ScipModel->presolve();

    return (std::make_pair(variableLowerBounds, variableUpperBounds));
}

int MIPSolverScip::getNumberOfExploredNodes()
{
    SCIP_Longint totalnodes = SCIPgetNTotalNodes(scip);
    if( totalnodes > INT_MAX )
        return INT_MAX;
    else
        return (int)totalnodes;
}

std::string MIPSolverScip::getSolverVersion()
{
    return (fmt::format("{}.{}.{}", std::to_string(SCIP_VERSION_MAJOR), std::to_string(SCIP_VERSION_MINOR), std::to_string(SCIP_VERSION_PATCH)));
}

ScipCallbackMultiTree::ScipCallbackMultiTree(EnvironmentPtr envPtr)
{
    env = envPtr;
    showOutput = env->settings->getSetting<bool>("Output.Console.DualSolver.Show");
}

void ScipCallbackMultiTree::callback()
{
//    try
//    {
//        if(where == GRB_CB_MESSAGE && showOutput) // Show output on console and log
//        {
//            auto message = getStringInfo(GRB_CB_MSG_STRING);
//            message.erase(std::remove(message.begin(), message.end(), '\n'), message.end());
//            env->output->outputInfo(fmt::format("      | {} ", message));
//        }
//        else if(where == GRB_CB_MIP)
//        // Used to get the number of open nodes
//        {
//            auto currIter = env->results->getCurrentIteration();
//            currIter->numberOfExploredNodes = (int)getDoubleInfo(GRB_CB_MIP_NODCNT);
//            currIter->numberOfOpenNodes = (int)getDoubleInfo(GRB_CB_MIP_NODLFT);
//        }
//
//        if(checkUserTermination())
//            this->abort();
//    }
//    catch(GRBException& e)
//    {
//        env->output->outputError("        Scip error when running main callback method", e.getMessage());
//    }
//    catch(...)
//    {
//        env->output->outputError("        Scip error when running main callback method");
//    }
}
} // namespace SHOT
