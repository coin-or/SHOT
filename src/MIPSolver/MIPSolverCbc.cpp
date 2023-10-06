/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCbc.h"
#include "MIPSolverCallbackBase.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

#include "CoinBuild.hpp"
#include "CoinModel.hpp"
#include "CoinPragma.hpp"
#include "CbcModel.hpp"
#include "CbcSolver.hpp"
#include "CbcBranchLotsize.hpp"
#include "OsiClpSolverInterface.hpp"

namespace SHOT
{

class TerminationEventHandler : public CbcEventHandler, public MIPSolverCallbackBase
{

public:
    TerminationEventHandler(EnvironmentPtr envPtr) { env = envPtr; };

    virtual ~TerminationEventHandler() {};

    TerminationEventHandler(const TerminationEventHandler& rhs) : CbcEventHandler(rhs) {};

    TerminationEventHandler& operator=(const TerminationEventHandler& rhs)
    {
        if(this != &rhs)
        {
            CbcEventHandler::operator=(rhs);
        }
        return *this;
    }

    virtual TerminationEventHandler* clone() const { return new TerminationEventHandler(env); };

    virtual CbcAction event(CbcEvent whichEvent)
    {
        if(whichEvent == CbcEventHandler::CbcEvent::node && checkUserTermination())
        {
            env->output->outputDebug("        Terminated by user.");

            return (CbcEventHandler::CbcAction::stop);
        }

        return (CbcEventHandler::CbcAction::noAction);
    }
};

static int dummyCallback(CbcModel* /*model*/, int /*whereFrom*/) { return 0; }

MIPSolverCbc::MIPSolverCbc(EnvironmentPtr envPtr) { env = envPtr; }

MIPSolverCbc::~MIPSolverCbc() = default;

bool MIPSolverCbc::initializeProblem()
{
    discreteVariablesActivated = true;

    this->cutOff = 1e100;

    osiInterface = std::make_unique<OsiClpSolverInterface>();
    coinModel = std::make_unique<CoinModel>();

    messageHandler = std::make_unique<CbcMessageHandler>(env);

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();

    return (true);
}

bool MIPSolverCbc::addVariable(
    std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound)
{
    int index = numberOfVariables;

    if(lowerBound < -getUnboundedVariableBoundValue())
        lowerBound = -getUnboundedVariableBoundValue();

    if(upperBound > getUnboundedVariableBoundValue())
        upperBound = getUnboundedVariableBoundValue();

    try
    {
        coinModel->setColumnBounds(index, lowerBound, upperBound);
        coinModel->setColName(index, name.c_str());

        switch(type)
        {
        case E_VariableType::Real:
            break;

        case E_VariableType::Integer:
            isProblemDiscrete = true;
            coinModel->setInteger(index);
            break;

        case E_VariableType::Binary:
            isProblemDiscrete = true;
            coinModel->setInteger(index);
            break;

        case E_VariableType::Semiinteger:
            coinModel->setInteger(index);
        case E_VariableType::Semicontinuous:
        {
            // variable is either {0} or in [semiBound,upperBound]
            std::array<double, 4> points = { 0.0, 0.0, semiBound, upperBound };
            if(semiBound < 0.0)
            {
                // variable is actually either {0} or in [lowerBound,semiBound]
                points[2] = lowerBound;
                points[3] = semiBound;
            }
            lotsizes.push_back(std::make_pair(index, points));

            isProblemDiscrete = true;
            break;
        }

        default:
            break;
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Cbc exception caught when adding variable to model: ", e.what());
        return (false);
    }

    variableTypes.push_back(type);
    variableNames.push_back(name);
    variableLowerBounds.push_back(lowerBound);
    variableUpperBounds.push_back(upperBound);
    numberOfVariables++;
    return (true);
}

bool MIPSolverCbc::initializeObjective() { return (true); }

bool MIPSolverCbc::addLinearTermToObjective(double coefficient, int variableIndex)
{
    try
    {
        // In case there is already a linear term  for the variable present in the objective we need to take this into
        // consideration
        double currentValue = coinModel->getColObjective(variableIndex);

        coinModel->setColObjective(variableIndex, coefficient + currentValue);
    }
    catch(CoinError& e)
    {
        env->output->outputError("        Cbc exception caught when adding linear term to objective: ", e.message());
        return (false);
    }

    return (true);
}

bool MIPSolverCbc::addQuadraticTermToObjective([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverCbc::finalizeObjective(bool isMinimize, double constant)
{
    try
    {
        objectiveLinearExpression.clear();

        for(int i = 0; i < coinModel->numberColumns(); i++)
        {
            if(auto coefficient = coinModel->getColObjective(i); coefficient != 0.0)
            {
                if(!isMinimize) // if maximize, we need to change the sign
                    coefficient *= -1;

                objectiveLinearExpression.insert(i, coefficient);
                coinModel->setColObjective(i, coefficient);
            }
        }

        if(!isMinimize)
            isMinimizationProblem = false;
        else
            isMinimizationProblem = true;

        this->objectiveConstant = constant;

        coinModel->setOptimizationDirection(1.0);
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Cbc exception caught when adding objective function to model: ", e.what());
        return (false);
    }

    return (true);
}

bool MIPSolverCbc::initializeConstraint() { return (true); }

bool MIPSolverCbc::addLinearTermToConstraint(double coefficient, int variableIndex)
{
    try
    {
        // In case there is already a linear term  for the variable present in the constraint we need to take this into
        // consideration
        double currentValue = coinModel->getElement(numberOfConstraints, variableIndex);

        coinModel->setElement(numberOfConstraints, variableIndex, coefficient + currentValue);
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Cbc exception caught when adding linear term to constraint: ", e.what());
        return (false);
    }

    return (true);
}

bool MIPSolverCbc::addQuadraticTermToConstraint([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverCbc::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
{
    int index = numberOfConstraints;
    try
    {
        if(valueLHS <= valueRHS)
        {
            coinModel->setRowBounds(index, valueLHS - constant, valueRHS - constant);
        }
        else
        {
            coinModel->setRowBounds(index, valueRHS - constant, valueLHS - constant);
        }

        coinModel->setRowName(index, name.c_str());
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Cbc exception caught when adding constraint to model: ", e.what());
        return (false);
    }

    allowRepairOfConstraint.push_back(false);
    numberOfConstraints++;
    return (true);
}

bool MIPSolverCbc::finalizeProblem()
{
    try
    {
        osiInterface->loadFromCoinModel(*coinModel);
        cbcModel = std::make_unique<CbcModel>(*osiInterface);

        CbcSolverUsefulData solverData;
        CbcMain0(*cbcModel, solverData);

        if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            cbcModel->setLogLevel(0);
            osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
        }

        osiInterface->setDblParam(OsiObjOffset, this->objectiveConstant);

        setSolutionLimit(1);
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Cbc exception caught when finalizing model", e.what());
        return (false);
    }

    return (true);
}

void MIPSolverCbc::initializeSolverSettings()
{
    // Set termination tolerances
    cbcModel->setAllowableGap(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);
    cbcModel->setAllowableFractionGap(env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination") / 1.0);
    osiInterface->setDblParam(
        OsiPrimalTolerance, env->settings->getSetting<double>("Tolerance.LinearConstraint", "Primal"));
    cbcModel->setIntegerTolerance(env->settings->getSetting<double>("Tolerance.Integer", "Primal"));
    osiInterface->setDblParam(OsiDualTolerance, env->settings->getSetting<double>("MIP.OptimalityTolerance", "Dual"));

    // Adds a user-provided node limit
    if(auto nodeLimit = env->settings->getSetting<double>("MIP.NodeLimit", "Dual"); nodeLimit > 0)
    {
        if(nodeLimit > SHOT_INT_MAX)
            nodeLimit = SHOT_INT_MAX;

        cbcModel->setMaximumNodes(nodeLimit);
    }

    // Set solution pool settings
    cbcModel->setMaximumSolutions(solLimit);
    cbcModel->setMaximumSavedSolutions(env->settings->getSetting<int>("MIP.SolutionPool.Capacity", "Dual"));

    // Set number of threads
    if(cbcModel->haveMultiThreadSupport())
    {
        // Cbc runs deterministcally if 100 is added to the number of threads
        if(env->settings->getSetting<bool>("Cbc.DeterministicParallelMode", "Subsolver"))
            numberOfThreads = env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual") + 100;
        else
            numberOfThreads = env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual");
    }
    else
        numberOfThreads = 1;

    cbcModel->passInMessageHandler(messageHandler.get());
}

int MIPSolverCbc::addLinearConstraint(
    const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
{
    try
    {
        int numConstraintsBefore = osiInterface->getNumRows();
        CoinPackedVector cut;

        for(auto E : elements)
        {
            cut.insert(E.first, E.second);
        }

        // Adds the cutting plane
        if(isGreaterThan)
        {
            osiInterface->addRow(cut, -constant, osiInterface->getInfinity(), name);
            assert(osiInterface->getRowLower()[osiInterface->getNumRows() - 1] == -constant);
        }
        else
        {
            osiInterface->addRow(cut, -osiInterface->getInfinity(), -constant, name);
            assert(osiInterface->getRowUpper()[osiInterface->getNumRows() - 1] == -constant);
        }

        if(osiInterface->getNumRows() > numConstraintsBefore)
        {
            allowRepairOfConstraint.push_back(allowRepair);
        }
        else
        {
            env->output->outputDebug("        Linear constraint  not added by Cbc");
            return (-1);
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when adding term to linear constraint in Cbc: ", e.what());
        return (-1);
    }
    catch(CoinError& e)
    {
        env->output->outputError("        Error when adding term to linear constraint in Cbc: ", e.message());
        return (-1);
    }

    return (osiInterface->getNumRows() - 1);
}

bool MIPSolverCbc::addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights)
{
    try
    {
        if(variableWeights.size() == 0)
        {
            variableWeights.resize(variableIndexes.size());

            for(size_t i = 0; i < variableIndexes.size(); i++)
                variableWeights[i] = i;
        }

        assert(variableWeights.size() == variableIndexes.size());

        OsiObject* object = new OsiSOS(osiInterface.get(), variableIndexes.size(), &variableIndexes[0],
            &variableWeights[0], (type == E_SOSType::One) ? 1 : 2);

        osiInterface->addObjects(1, &object);

        delete object;
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when adding special ordered set constraint in Cbc:", e.what());
        return (false);
    }
    catch(CoinError& e)
    {
        env->output->outputError("        Error when adding special ordered set constraint in Cbc:", e.message());
        return (false);
    }

    return (true);
}

void MIPSolverCbc::activateDiscreteVariables(bool activate)
{
    if(env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0
        || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
        return;

    if(activate)
    {
        env->output->outputDebug("        Activating MIP strategy");

        for(int i = 0; i < numberOfVariables; i++)
        {
            assert(variableTypes.at(i) != E_VariableType::Semicontinuous
                && variableTypes.at(i) != E_VariableType::Semiinteger);

            if(variableTypes.at(i) == E_VariableType::Integer || variableTypes.at(i) == E_VariableType::Binary)
            {
                osiInterface->setInteger(i);
                assert(osiInterface->isInteger(i));
            }
        }

        discreteVariablesActivated = true;
    }
    else
    {
        env->output->outputDebug("        Activating LP strategy");
        for(int i = 0; i < numberOfVariables; i++)
        {
            assert(variableTypes.at(i) != E_VariableType::Semicontinuous
                && variableTypes.at(i) != E_VariableType::Semiinteger);

            if(variableTypes.at(i) == E_VariableType::Integer || variableTypes.at(i) == E_VariableType::Binary)
            {
                osiInterface->setContinuous(i);
                assert(osiInterface->isContinuous(i));
            }
        }

        discreteVariablesActivated = false;
    }
}

E_ProblemSolutionStatus MIPSolverCbc::getSolutionStatus()
{
    E_ProblemSolutionStatus MIPSolutionStatus;

    if(cbcModel->isProvenOptimal() && cbcModel->numberSavedSolutions() > 0)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
    }
    else if(cbcModel->isProvenInfeasible())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if(cbcModel->isProvenDualInfeasible())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(cbcModel->isSolutionLimitReached() && cbcModel->numberSavedSolutions() > 0)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
    }
    else if(cbcModel->isSecondsLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
    }
    else if(cbcModel->isNodeLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::NodeLimit;
    }
    else if(cbcModel->isAbandoned())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
    }
    else if(cbcModel->isContinuousUnbounded())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(cbcModel->status() == 5)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
    }
    else
    {
        auto status = cbcModel->status();
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
        env->output->outputError(
            fmt::format("        MIP solver return status unknown (Cbc returned status {}).", status));
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverCbc::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    const int numArguments = 17;
    char* argv[numArguments];
    std::string arg;

    argv[0] = strdup("");
    argv[1] = strdup("-autoscale");
    if(env->settings->getSetting<bool>("Cbc.AutoScale", "Subsolver"))
        argv[2] = strdup("on");
    else
        argv[2] = strdup("off");

    argv[3] = strdup("-nodestrategy");

    switch(env->settings->getSetting<int>("Cbc.NodeStrategy", "Subsolver"))
    {
    case 0:
        arg = "depth";
        break;

    case 1:
        arg = "downdepth";
        break;

    case 2:
        arg = "downfewest";
        break;

    case 3:
        arg = "fewest";
        break;

    case 4:
        arg = "hybrid";
        break;

    case 5:
        arg = "updepth";
        break;

    case 6:
        arg = "upfewest";
        break;

    default:
        arg = "hybrid";
        break;
    }

    argv[4] = strdup(arg.c_str());

    argv[5] = strdup("-scaling");

    switch(env->settings->getSetting<int>("Cbc.Scaling", "Subsolver"))
    {
    case 0:
        arg = "automatic";
        break;

    case 1:
        arg = "dynamic";
        break;

    case 2:
        arg = "equilibrium";
        break;

    case 3:
        arg = "geometric";
        break;

    case 4:
        arg = "off";
        break;

    case 5:
        arg = "rowsonly";
        break;

    default:
        arg = "automatic";
        break;
    }

    argv[6] = strdup(arg.c_str());

    argv[7] = strdup("-strategy");
    arg = std::to_string(env->settings->getSetting<int>("Cbc.Strategy", "Subsolver"));
    argv[8] = strdup(arg.c_str());

    /*
        TODO: Adding cutoffs seems to have stability-issues (status changes from unbounded -> infeasible in some
        cases, cf. https://github.com/coin-or/SHOT/issues/133). As the cutoff is added as a constraint, this can be
        deactivated here.

        argv[9] = strdup("-cutoff");

        if(this->cutOff > 1e100)
            arg = "1e100";
        else if(this->cutOff < -1e100)
            arg = "-1e100";
        else
            arg = fmt::format("{}", this->cutOff);

        argv[10] = strdup(arg.c_str());*/

    argv[9] = strdup("-sec");
    arg = fmt::format("{}", this->timeLimit);
    argv[10] = strdup(arg.c_str());

    // pass threads option if not running single-threaded (101 = 1 thread + deterministic multithreading)
    if(numberOfThreads != 1 && numberOfThreads != 101)
    {
        argv[11] = strdup("-threads");
        arg = std::to_string(numberOfThreads);
        argv[12] = strdup(arg.c_str());

        argv[13] = strdup("-solve");
        argv[14] = strdup("-quit");
        argv[15] = strdup("");
        argv[16] = strdup("");
    }
    else
    {
        argv[11] = strdup("-solve");
        argv[12] = strdup("-quit");
        argv[13] = strdup("");
        argv[14] = strdup("");
        argv[15] = strdup("");
        argv[16] = strdup("");
    }

    try
    {
        cbcModel = std::make_unique<CbcModel>(*osiInterface);

        initializeSolverSettings();

        // Adding the MIP start provided, so far only if there are no special variable types included, since the MIP
        // start functionality in Cbc version 2 is unstable
        if(MIPStart.size() > 0
            && (env->reformulatedProblem->properties.numberOfSemiintegerVariables
                    + env->reformulatedProblem->properties.numberOfSemicontinuousVariables
                    + env->reformulatedProblem->properties.numberOfSpecialOrderedSets
                == 0))
            cbcModel->setMIPStart(MIPStart);

        // Create and add lotsize objects
        if(!lotsizes.empty())
        {
            std::vector<CbcObject*> cbcobjects;
            cbcobjects.reserve(lotsizes.size());

            for(const auto& l : lotsizes)
            {
                if(l.second[2] == l.second[3]) // special case where second interval is singleton, too
                    cbcobjects.push_back(new CbcLotsize(cbcModel.get(), l.first, 2, l.second.data() + 1, false));
                else
                    cbcobjects.push_back(new CbcLotsize(cbcModel.get(), l.first, 2, l.second.data(), true));
            }

            cbcModel->addObjects(cbcobjects.size(), cbcobjects.data());

            for(CbcObject* o : cbcobjects)
                delete o;
        }

        CbcSolverUsefulData solverData;
        CbcMain0(*cbcModel, solverData);

        if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            cbcModel->setLogLevel(0);
            osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
        }

        osiInterface->setDblParam(OsiObjOffset, this->objectiveConstant);

        TerminationEventHandler eventHandler(env);
        cbcModel->passInEventHandler(&eventHandler);

        CbcMain1(numArguments, const_cast<const char**>(argv), *cbcModel, dummyCallback, solverData);

        MIPSolutionStatus = getSolutionStatus();
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when solving subproblem with Cbc", e.what());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    if(MIPSolutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        if(env->reformulatedProblem->objectiveFunction->properties.classification
                == E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear
            && hasDualAuxiliaryObjectiveVariable())
        {
            osiInterface->setColBounds(getDualAuxiliaryObjectiveVariableIndex(), -1000000000.0, 1000000000.0);

            cbcModel = std::make_unique<CbcModel>(*osiInterface);

            initializeSolverSettings();

            CbcSolverUsefulData solverData;
            CbcMain0(*cbcModel, solverData);

            if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
            {
                cbcModel->setLogLevel(0);
                osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
            }

            osiInterface->setDblParam(OsiObjOffset, this->objectiveConstant);

            CbcMain1(numArguments, const_cast<const char**>(argv), *cbcModel, dummyCallback, solverData);

            MIPSolutionStatus = getSolutionStatus();

            if(MIPSolutionStatus == E_ProblemSolutionStatus::Optimal)
                MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;

            osiInterface->setColBounds(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue(),
                getUnboundedVariableBoundValue());
        }
    }

    // To find a feasible point for an unbounded dual problem and not when solving the minimax-problem
    if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded && env->results->getNumberOfIterations() > 0)
    {
        VectorInteger variablesWithChangedBounds;
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
                if(V->isDualUnbounded())
                {
                    // Temporarily introduce bounds [-1e20,1e20] for unbounded variables in objective
                    updateVariableBound(V->index, -1e20, 1e20);
                    variablesWithChangedBounds.push_back(V->index);
                    problemUpdated = true;
                }
            }
        }
        else if(env->reformulatedProblem->objectiveFunction->properties.classification
                >= E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear
            && hasDualAuxiliaryObjectiveVariable())
        {
            // The auxiliary variable in the dual problem is unbounded
            updateVariableBound(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue() / 10e40,
                getUnboundedVariableBoundValue() / 10e40);
            problemUpdated = true;
        }

        if(problemUpdated)
        {
            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {

                auto filename = fmt::format("{}/dualiter{}_unbounded.lp",
                    env->settings->getSetting<std::string>("Debug.Path", "Output"),
                    env->results->getCurrentIteration()->iterationNumber - 1);

                try
                {
                    osiInterface->writeLp(filename.c_str(), "", 1e-7, 10, 10, 0.0, true);
                }
                catch(std::exception& e)
                {
                    env->output->outputError(
                        "        Error when saving relaxed infesibility model to file in Cbc", e.what());
                }
            }

            cbcModel = std::make_unique<CbcModel>(*osiInterface);

            initializeSolverSettings();

            CbcSolverUsefulData solverData;
            CbcMain0(*cbcModel, solverData);

            if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
            {
                cbcModel->setLogLevel(0);
                osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
            }

            osiInterface->setDblParam(OsiObjOffset, this->objectiveConstant);

            CbcMain1(numArguments, const_cast<const char**>(argv), *cbcModel, dummyCallback, solverData);

            MIPSolutionStatus = getSolutionStatus();

            if(MIPSolutionStatus == E_ProblemSolutionStatus::Optimal)
                MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;

            for(auto& I : variablesWithChangedBounds)
            {
                updateVariableBound(I, env->reformulatedProblem->getVariableLowerBound(I),
                    env->reformulatedProblem->getVariableUpperBound(I));
            }

            env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }
    }

    for(int i = numArguments - 1; i >= 0; --i)
        free(argv[i]);

    return (MIPSolutionStatus);
}

bool MIPSolverCbc::repairInfeasibility()
{
    if(env->dualSolver->generatedHyperplanes.size() == 0)
        return (false);

    try
    {
        // auto repairedInterface = std::make_unique<OsiClpSolverInterface>(*osiInterface->clone());
        auto repairedInterface = osiInterface->clone();

        int numOrigConstraints = env->reformulatedProblem->properties.numberOfLinearConstraints;
        int numOrigVariables = osiInterface->getNumCols();
        int numCurrConstraints = osiInterface->getNumRows();

        VectorInteger repairConstraints;
        VectorDouble relaxParameters;

        int numConstraintsToRepair = 0;

        auto rowSense = osiInterface->getRowSense();

        for(int i = numOrigConstraints; i < numCurrConstraints; i++)
        {
            if(allowRepairOfConstraint[i])
            {
                repairConstraints.push_back(i);
                relaxParameters.push_back(1 / (((double)i) - numOrigConstraints + 1.0));
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
                constraints[i] = osiInterface->getRowName(repairConstraints[i]);
            }

            auto filename = fmt::format("{}/dualiter{}_infeasrelaxweights.txt",
                env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            Utilities::saveVariablePointVectorToFile(relaxParameters, constraints, filename);
        }

        for(int i = 0; i < numConstraintsToRepair; i++)
        {
            if(rowSense[repairConstraints.at(i)] == 'L')
            {
                int tmpConstraint[1] = { repairConstraints.at(i) };
                double tmpCoefficient[1] = { -1.0 };
                repairedInterface->addCol(
                    1, tmpConstraint, tmpCoefficient, 0.0, osiInterface->getInfinity(), relaxParameters.at(i));
            }
            else if(rowSense[repairConstraints.at(i)] == 'G')
            {
                int tmpConstraint[1] = { repairConstraints.at(i) };
                double tmpCoefficient[1] = { 1.0 };
                repairedInterface->addCol(
                    1, tmpConstraint, tmpCoefficient, 0.0, osiInterface->getInfinity(), relaxParameters.at(i));
            }
            else
            {
                std::cout << " constraint not supported\n";
            }
        }

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            auto filename = fmt::format("{}/dualiter{}_infeasrelax.lp",
                env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            try
            {
                repairedInterface->writeLp(filename.c_str(), "", 1e-7, 10, 10, 0.0, true);
            }
            catch(std::exception& e)
            {
                env->output->outputError(
                    "        Error when saving relaxed infesibility model to file in Cbc", e.what());
            }
        }

        cbcModel = std::make_unique<CbcModel>(*repairedInterface);

        initializeSolverSettings();

        CbcSolverUsefulData solverData;
        CbcMain0(*cbcModel, solverData);

        if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            cbcModel->setLogLevel(0);
            osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
        }

        osiInterface->setDblParam(OsiObjOffset, this->objectiveConstant);

        cachedSolutionHasChanged = true;

        const int numArguments = 17;
        char* argv[numArguments];
        std::string arg;

        argv[0] = strdup("");
        argv[1] = strdup("-autoscale");
        if(env->settings->getSetting<bool>("Cbc.AutoScale", "Subsolver"))
            argv[2] = strdup("on");
        else
            argv[2] = strdup("off");

        argv[3] = strdup("-nodestrategy");

        switch(env->settings->getSetting<int>("Cbc.NodeStrategy", "Subsolver"))
        {
        case 0:
            arg = "depth";
            break;

        case 1:
            arg = "downdepth";
            break;

        case 2:
            arg = "downfewest";
            break;

        case 3:
            arg = "fewest";
            break;

        case 4:
            arg = "hybrid";
            break;

        case 5:
            arg = "updepth";
            break;

        case 6:
            arg = "upfewest";
            break;

        default:
            arg = "hybrid";
            break;
        }

        argv[4] = strdup(arg.c_str());

        argv[5] = strdup("-scaling");

        switch(env->settings->getSetting<int>("Cbc.Scaling", "Subsolver"))
        {
        case 0:
            arg = "automatic";
            break;

        case 1:
            arg = "dynamic";
            break;

        case 2:
            arg = "equilibrium";
            break;

        case 3:
            arg = "geometric";
            break;

        case 4:
            arg = "off";
            break;

        case 5:
            arg = "rowsonly";
            break;

        default:
            arg = "automatic";
            break;
        }

        argv[6] = strdup(arg.c_str());

        argv[7] = strdup("-strategy");
        arg = std::to_string(env->settings->getSetting<int>("Cbc.Strategy", "Subsolver"));
        argv[8] = strdup(arg.c_str());

        /*
        argv[9] = strdup("-cutoff");

        if(this->cutOff > 1e100)
            arg = "1e100";
        else if(this->cutOff < -1e100)
            arg = "-1e100";
        else
            arg = fmt::format("{}", this->cutOff);

        argv[10] = strdup(arg.c_str());*/

        argv[9] = strdup("-sec");
        arg = fmt::format("{}", this->timeLimit);
        argv[10] = strdup(arg.c_str());

        // pass threads option if not running single-threaded (101 = 1 thread + deterministic multithreading)
        if(numberOfThreads != 1 && numberOfThreads != 101)
        {
            argv[11] = strdup("-threads");
            arg = std::to_string(numberOfThreads);
            argv[12] = strdup(arg.c_str());

            argv[13] = strdup("-solve");
            argv[14] = strdup("-quit");
            argv[15] = strdup("");
            argv[16] = strdup("");
        }
        else
        {
            argv[11] = strdup("-solve");
            argv[12] = strdup("-quit");
            argv[13] = strdup("");
            argv[14] = strdup("");
            argv[15] = strdup("");
            argv[16] = strdup("");
        }

        CbcMain1(numArguments, const_cast<const char**>(argv), *cbcModel, dummyCallback, solverData);

        for(int i = numArguments - 1; i >= 0; --i)
            free(argv[i]);

        auto MIPSolutionStatus = getSolutionStatus();

        if(MIPSolutionStatus != E_ProblemSolutionStatus::Optimal)
        {
            env->output->outputDebug("        Could not repair the infeasible dual problem.");

            delete repairedInterface;
            return (false);
        }

        auto solution = getVariableSolution(0);
        int numRepairs = 0;

        for(int i = 0; i < numConstraintsToRepair; i++)
        {
            double slackValue = solution[numOrigVariables + i];

            if(slackValue == 0.0)
                continue;

            double oldRHS = osiInterface->getRowUpper()[repairConstraints[i]];

            if(rowSense[repairConstraints.at(i)] == 'L')
            {
                osiInterface->setRowUpper(repairConstraints[i], oldRHS + 1.5 * slackValue);
                env->output->outputDebug("        Constraint: " + osiInterface->getRowName(repairConstraints[i])
                    + " repaired with infeasibility = " + std::to_string(1.5 * slackValue));
            }
            else if(rowSense[repairConstraints.at(i)] == 'G')
            {
                env->output->outputDebug("        Constraint: " + osiInterface->getRowName(repairConstraints[i])
                    + " repaired with infeasibility = " + std::to_string(-1.5 * slackValue));
                osiInterface->setRowUpper(repairConstraints[i], oldRHS - 1.5 * slackValue);
            }

            numRepairs++;
        }

        env->results->getCurrentIteration()->numberOfInfeasibilityRepairedConstraints = numRepairs;

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            auto filename = fmt::format("{}/dualiter{}_infeasrelax.lp",
                env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            writeProblemToFile(filename);
        }

        delete repairedInterface;

        if(numRepairs == 0)
        {
            env->output->outputDebug("        Could not repair the infeasible dual problem.");
            return (false);
        }

        env->output->outputDebug("        Number of constraints modified: " + std::to_string(numRepairs));

        cbcModel = std::make_unique<CbcModel>(*osiInterface);

        return (true);
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when trying to repair infeasibility", e.what());
    }

    return (false);
}

int MIPSolverCbc::increaseSolutionLimit(int increment)
{
    this->solLimit += increment;

    this->setSolutionLimit(this->solLimit);

    return (this->solLimit);
}

void MIPSolverCbc::setSolutionLimit(long int limit) { this->solLimit = limit; }

int MIPSolverCbc::getSolutionLimit() { return (this->solLimit); }

void MIPSolverCbc::setTimeLimit(double seconds)
{
    if(seconds > 1e100)
        timeLimit = 1e100;
    else if(seconds < 0)
        timeLimit = 0.00001;
    else
        timeLimit = seconds;
}

void MIPSolverCbc::setCutOff(double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    double cutOffTol = env->settings->getSetting<double>("MIP.CutOff.Tolerance", "Dual");

    if(isMinimizationProblem)
    {
        this->cutOff = cutOff + cutOffTol;

        env->output->outputDebug(fmt::format("        Setting cutoff value to {} for minimization.", this->cutOff));
    }
    else
    {
        this->cutOff = -1 * (cutOff + cutOffTol);

        env->output->outputDebug(fmt::format("        Setting cutoff value to {} for maximization.", this->cutOff));
    }
}

void MIPSolverCbc::setCutOffAsConstraint([[maybe_unused]] double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    try
    {
        if(!cutOffConstraintDefined)
        {
            if(isMinimizationProblem)
            {
                osiInterface->addRow(objectiveLinearExpression, -osiInterface->getInfinity(),
                    (cutOff - this->objectiveConstant), "CUTOFF_C");

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
            }
            else
            {
                osiInterface->addRow(objectiveLinearExpression, -osiInterface->getInfinity(),
                    -1.0 * (cutOff - this->objectiveConstant), "CUTOFF_C");

                env->output->outputDebug(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
            }

            allowRepairOfConstraint.push_back(false);

            cutOffConstraintDefined = true;
            cutOffConstraintIndex = osiInterface->getNumRows() - 1;

            modelUpdated = true;
        }
        else
        {
            if(isMinimizationProblem)
            {
                osiInterface->setRowUpper(cutOffConstraintIndex, cutOff - this->objectiveConstant);

                env->output->outputDebug(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for minimization.");
            }
            else
            {
                osiInterface->setRowUpper(cutOffConstraintIndex, -(cutOff - this->objectiveConstant));

                env->output->outputDebug(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
            }

            modelUpdated = true;
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when setting cut off constraint value", e.what());
    }
}

void MIPSolverCbc::addMIPStart(VectorDouble point)
{
    MIPStart.clear();

    if((int)point.size() < env->reformulatedProblem->properties.numberOfVariables)
        env->reformulatedProblem->augmentAuxiliaryVariableValues(point);

    if(this->hasDualAuxiliaryObjectiveVariable())
        point.push_back(env->reformulatedProblem->objectiveFunction->calculateValue(point));

    assert(variableNames.size() == point.size());

    for(size_t i = 0; i < point.size(); i++)
        MIPStart.emplace_back(variableNames.at(i).c_str(), point.at(i));
}

void MIPSolverCbc::writeProblemToFile(std::string filename)
{
    try
    {
        osiInterface->writeLp(filename.c_str(), "", 1e-7, 10, 10, 0.0, true);
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when saving model to file in Cbc", e.what());
    }
}

double MIPSolverCbc::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    if(!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError("        Cannot obtain solution with index " + std::to_string(solIdx)
            + " in Cbc since the problem is LP/QP!");

        return (NAN);
    }

    double objectiveValue = NAN;

    // Cannot trust Cbc to give the correct sign of the objective back se we recalculate it
    try
    {
        auto variableSolution = getVariableSolution(solIdx);
        double factor = (isMinimizationProblem) ? 1.0 : -1.0;

        objectiveValue = factor * coinModel->objectiveOffset();

        for(int i = 0; i < objectiveLinearExpression.getNumElements(); i++)
        {
            objectiveValue += factor * objectiveLinearExpression.getElements()[i]
                * variableSolution[objectiveLinearExpression.getIndices()[i]];
        }

        objectiveValue += this->objectiveConstant;
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "        Error when obtaining objective value for solution index " + std::to_string(solIdx) + " in Cbc",
            e.what());
    }

    return (objectiveValue);
}

void MIPSolverCbc::deleteMIPStarts() { MIPStart.clear(); }

bool MIPSolverCbc::createIntegerCut(IntegerCut& integerCut)
{
    assert(integerCut.variableValues.size() == (size_t)env->reformulatedProblem->properties.numberOfDiscreteVariables);
    bool allowIntegerCutRepair = env->settings->getSetting<bool>("MIP.InfeasibilityRepair.IntegerCuts", "Dual");

    int numConstraintsBefore = osiInterface->getNumRows();
    int constraintCounter = osiInterface->getNumRows();

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
        if(integerCut.areAllVariablesBinary) // Integer cut for problem with binary variables only
        {
            size_t index = 0;
            CoinPackedVector cut;

            for(auto& VAR : env->reformulatedProblem->allVariables)
            {
                if(!(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                       || VAR->properties.type == E_VariableType::Semiinteger))
                    continue;

                int variableValue = integerCut.variableValues[index];

                if(variableValue == 1.0)
                    cut.insert(VAR->index, 1.0);
                else if(variableValue == 0.0)
                    cut.insert(VAR->index, -1.0);
                else
                {
                    env->output->outputDebug("        Integer cut not added by Cbc ");
                    return (false);
                }

                index++;
            }

            int tmpNumConstraints = osiInterface->getNumRows();

            osiInterface->addRow(cut, -osiInterface->getInfinity(), integerCut.variableValues.size() - 1.0,
                fmt::format("IC_{}", env->solutionStatistics.numberOfIntegerCuts));

            if(osiInterface->getNumRows() > tmpNumConstraints)
            {
                allowRepairOfConstraint.push_back(allowIntegerCutRepair);
                integerCuts.push_back(constraintCounter);
                constraintCounter++;
            }
        }
        else // Integer cut for problem with general integers
        {
            size_t index = 0;
            CoinPackedVector cut;
            double sumLB = 0.0;
            double sumUB = 0.0;

            for(auto& I : integerCut.variableIndexes)
            {
                auto VAR = env->reformulatedProblem->getVariable(I);
                int variableValue = integerCut.variableValues[index];

                assert(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                    || VAR->properties.type == E_VariableType::Semiinteger);

                if(variableValue == VAR->upperBound)
                {
                    sumUB += VAR->upperBound;
                    cut.insert(VAR->index, -1.0);
                }
                else if(variableValue == VAR->lowerBound)
                {
                    sumLB -= VAR->lowerBound;
                    cut.insert(VAR->index, 1.0);
                }
                else
                {
                    int wIndex = numberOfVariables;
                    int vIndex = numberOfVariables + 1;
                    numberOfVariables += 2;

                    double M1 = 2 * (variableValue - VAR->lowerBound);
                    double M2 = 2 * (VAR->upperBound - variableValue);

                    double tmpCoefficient[1] = { 0.0 };
                    int tmpConstraint[1] = { 0 };
                    osiInterface->addCol(1, tmpConstraint, tmpCoefficient, 0.0, osiInterface->getInfinity(), 0.0,
                        fmt::format("wIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));
                    osiInterface->addCol(1, tmpConstraint, tmpCoefficient, 0.0, osiInterface->getInfinity(), 0.0,
                        fmt::format("vIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));

                    cut.insert(wIndex, 1.0);

                    CoinPackedVector cut1a, cut1b, cut2, cut3;

                    int tmpNumConstraints = osiInterface->getNumRows();
                    cut1a.insert(VAR->index, 1.0);
                    cut1a.insert(wIndex, 1.0);
                    osiInterface->addRow(cut1a, variableValue, osiInterface->getInfinity(),
                        fmt::format("IC{}_{}_1a", env->solutionStatistics.numberOfIntegerCuts, index));

                    if(osiInterface->getNumRows() > tmpNumConstraints)
                    {
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }

                    tmpNumConstraints = osiInterface->getNumRows();
                    cut1b.insert(VAR->index, 1.0);
                    cut1b.insert(wIndex, -1.0);

                    osiInterface->addRow(cut1b, -osiInterface->getInfinity(), variableValue,
                        fmt::format("IC{}_{}_1b", env->solutionStatistics.numberOfIntegerCuts, index));

                    if(osiInterface->getNumRows() > tmpNumConstraints)
                    {
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }

                    tmpNumConstraints = osiInterface->getNumRows();
                    cut2.insert(wIndex, 1.0);
                    cut2.insert(VAR->index, -1.0);
                    cut2.insert(vIndex, M1);
                    osiInterface->addRow(cut2, -osiInterface->getInfinity(), -variableValue + M1,
                        fmt::format("IC{}_{}_2", env->solutionStatistics.numberOfIntegerCuts, index));

                    if(osiInterface->getNumRows() > tmpNumConstraints)
                    {
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }

                    tmpNumConstraints = osiInterface->getNumRows();
                    cut3.insert(wIndex, 1.0);
                    cut3.insert(VAR->index, 1.0);
                    cut3.insert(vIndex, -M2);
                    osiInterface->addRow(cut3, -osiInterface->getInfinity(), variableValue,
                        fmt::format("IC{}_{}_3", env->solutionStatistics.numberOfIntegerCuts, index));

                    if(osiInterface->getNumRows() > tmpNumConstraints)
                    {
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }

                    osiInterface->setContinuous(wIndex);
                    osiInterface->setInteger(vIndex);
                    osiInterface->setColLower(wIndex, 0);
                    osiInterface->setColLower(vIndex, 0);
                    osiInterface->setColUpper(vIndex, 1);

                    index++;
                }
            }

            int tmpNumConstraints = osiInterface->getNumRows();

            osiInterface->addRow(cut, 1 - sumLB - sumUB, osiInterface->getInfinity(),
                fmt::format("IC{}_4", env->solutionStatistics.numberOfIntegerCuts));

            if(osiInterface->getNumRows() > tmpNumConstraints)
            {
                allowRepairOfConstraint.push_back(allowIntegerCutRepair);
                integerCuts.push_back(constraintCounter);
                constraintCounter++;
            }
        }

        if(constraintCounter == numConstraintsBefore)
        {
            env->output->outputDebug("        Integer cut not added by Cbc");
            return (false);
        }
    }
    catch(CoinError& e)
    {
        env->output->outputError("        Error when adding term to integer cut in Cbc: ", e.message());
        return (false);
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when adding term to integer cut in Cbc: ", e.what());
        return (false);
    }

    return (true);
}

VectorDouble MIPSolverCbc::getVariableSolution(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();
    int numVar = cbcModel->getNumCols();
    VectorDouble solution(numVar);

    try
    {
        if(isMIP)
        {
            auto tmpSol = cbcModel->savedSolution(solIdx);
            for(int i = 0; i < numVar; i++)
            {
                solution.at(i) = tmpSol[i];
            }
        }
        else
        {
            auto tmpSol = cbcModel->bestSolution();

            for(int i = 0; i < numVar; i++)
            {
                solution.at(i) = tmpSol[i];
            }
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "        Error when reading solution with index " + std::to_string(solIdx) + " in Cbc", e.what());
    }
    return (solution);
}

int MIPSolverCbc::getNumberOfSolutions()
{
    int numSols = 0;

    try
    {
        numSols = cbcModel->numberSavedSolutions();
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when obtaining number of solutions in Cbc", e.what());
    }

    return (numSols);
}

void MIPSolverCbc::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverCbc::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound && currentVariableBounds.second == upperBound)
        return;

    try
    {
        osiInterface->setColBounds(varIndex, lowerBound, upperBound);
        // TODO: activate these again, cf. issue #77
        // assert(osiInterface->getColLower()[varIndex] == lowerBound);
        // assert(osiInterface->getColUpper()[varIndex] == upperBound);
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex) + " in Cbc",
            e.what());
    }
}

void MIPSolverCbc::updateVariableLowerBound(int varIndex, double lowerBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound)
        return;

    try
    {
        osiInterface->setColLower(varIndex, lowerBound);
        // TODO: activate these again, cf. issue #77
        // assert(osiInterface->getColLower()[varIndex] == lowerBound);
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex) + " in Cbc",
            e.what());
    }
}

void MIPSolverCbc::updateVariableUpperBound(int varIndex, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.second == upperBound)
        return;

    try
    {
        osiInterface->setColUpper(varIndex, upperBound);
        // TODO: activate these again, cf. issue #77
        // assert(osiInterface->getColUpper()[varIndex] == upperBound);
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "        Error when updating variable bounds for variable index" + std::to_string(varIndex) + " in Cbc",
            e.what());
    }
}

PairDouble MIPSolverCbc::getCurrentVariableBounds(int varIndex)
{
    PairDouble tmpBounds;

    try
    {
        tmpBounds.first = osiInterface->getColLower()[varIndex];
        tmpBounds.second = osiInterface->getColUpper()[varIndex];
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "        Error when obtaining variable bounds for variable index" + std::to_string(varIndex) + " in Cbc",
            e.what());
    }

    return (tmpBounds);
}

bool MIPSolverCbc::supportsQuadraticObjective() { return (false); }

bool MIPSolverCbc::supportsQuadraticConstraints() { return (false); }

double MIPSolverCbc::getUnboundedVariableBoundValue() { return (1e+50); }

double MIPSolverCbc::getDualObjectiveValue()
{
    bool isMIP = getDiscreteVariableStatus();
    double objVal = (isMinimizationProblem ? SHOT_DBL_MIN : SHOT_DBL_MAX);

    try
    {
        if(isMIP)
        {
            objVal = cbcModel->getBestPossibleObjValue();
            if(!isMinimizationProblem)
                objVal *= -1.0;
        }
        else if(getSolutionStatus() == E_ProblemSolutionStatus::Optimal)
        {
            objVal = getObjectiveValue();
        }
        else
        {
            objVal = cbcModel->getBestPossibleObjValue();
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when obtaining dual objective value in Cbc", e.what());
    }

    return (objVal);
}

std::pair<VectorDouble, VectorDouble> MIPSolverCbc::presolveAndGetNewBounds()
{
    return (std::make_pair(variableLowerBounds, variableUpperBounds));
}

void MIPSolverCbc::writePresolvedToFile([[maybe_unused]] std::string filename)
{
    // Not implemented
}

void MIPSolverCbc::checkParameters()
{
    // For stability
    env->settings->updateSetting("Tolerance.TrustLinearConstraintValues", "Primal", false);
}

int MIPSolverCbc::getNumberOfExploredNodes()
{
    try
    {
        return (cbcModel->getNodeCount());
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when getting number of explored nodes", e.what());
        return 0;
    }
}

std::string MIPSolverCbc::getSolverVersion() { return (CBC_VERSION); }

int CbcMessageHandler::print()
{
    if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        return 0;

    std::string message(CoinMessageHandler::messageBuffer());

    auto lines = Utilities::splitStringByCharacter(CoinMessageHandler::messageBuffer(), '\n');

    for(auto const& line : lines)
        env->output->outputInfo(fmt::format("      | {} ", line));

    return 0;
}
} // namespace SHOT
