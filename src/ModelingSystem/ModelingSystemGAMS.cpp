/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ModelingSystemGAMS.h"

#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"
#include "../Enums.h"

#include "../Model/Simplifications.h"

#include "GamsNLinstr.h"
#ifdef GAMS_BUILD
extern "C" void HSLInit();
#endif

#include <filesystem>

namespace SHOT
{

ModelingSystemGAMS::ModelingSystemGAMS(EnvironmentPtr envPtr)
    : IModelingSystem(envPtr)
    , modelingObject(nullptr)
    , modelingEnvironment(nullptr)
    , createdtmpdir(false)
    , createdgmo(false)
{
}

void ModelingSystemGAMS::setModelingObject(gmoHandle_t gmo)
{
    modelingObject = gmo;
    modelingEnvironment = (gevHandle_t)gmoEnvironment(gmo);
}

ModelingSystemGAMS::~ModelingSystemGAMS()
{
    clearGAMSObjects();

    if(createdgmo)
    {
        gmoLibraryUnload();
        gevLibraryUnload();
    }
}

void ModelingSystemGAMS::augmentSettings([[maybe_unused]] SettingsPtr settings) {}

void ModelingSystemGAMS::updateSettings(SettingsPtr settings, [[maybe_unused]] palHandle_t pal)
{
    assert(modelingEnvironment != nullptr);
    assert(modelingObject != nullptr);

#ifdef GAMS_BUILD
    assert(pal != nullptr);
    if(palLicenseCheckSubSys(pal, const_cast<char*>("IP")) == 0)
    {
        /* IPOPTH is licensed: use HSL MA27 and make it available */
        env->settings->updateSetting("Ipopt.LinearSolver", "Subsolver", static_cast<int>(ES_IpoptSolver::ma27));
        HSLInit();
    }
    else
        env->settings->updateSetting("Ipopt.LinearSolver", "Subsolver", static_cast<int>(ES_IpoptSolver::mumps));
#endif

    // Process GAMS options.
    // We do not want to use GAMS defaults if called on a gms file, in which case we would have created our own GMO.
    if(!createdgmo)
    {
        env->settings->updateSetting("TimeLimit", "Termination", gevGetDblOpt(modelingEnvironment, gevResLim));
        if(gevGetIntOpt(modelingEnvironment, gevIterLim) != ITERLIM_INFINITY)
            env->settings->updateSetting(
                "IterationLimit", "Termination", gevGetIntOpt(modelingEnvironment, gevIterLim));
        else
            env->settings->updateSetting("IterationLimit", "Termination", SHOT_INT_MAX);
        env->settings->updateSetting(
            "ObjectiveGap.Absolute", "Termination", gevGetDblOpt(modelingEnvironment, gevOptCA));
        env->settings->updateSetting(
            "ObjectiveGap.Relative", "Termination", gevGetDblOpt(modelingEnvironment, gevOptCR));

        env->settings->updateSetting("MIP.NumberOfThreads", "Dual", gevThreads(modelingEnvironment));

        // TODO? gevDomLim: stop if so many evaluation errors in nonlinear functions
        // TODO gevNodeLim: should be node limit for single-tree strategy, if > 0
        // TODO? gevCutOff, gevUseCutOff: stop if dual bound is above this value (if I remember right)
        // TODO gevCheat, gevUseCheat -> MIP.CutOffTolerance ?
        // TODO?? gevTryInt: handling of fractional values in initial solution for repair heuristics

        env->output->outputDebug("Time limit set to "
            + Utilities::toString(env->settings->getSetting<double>("TimeLimit", "Termination")) + " by GAMS");
        env->output->outputDebug("Iteration limit set to "
            + Utilities::toString(env->settings->getSetting<int>("IterationLimit", "Termination")) + " by GAMS");
        env->output->outputDebug("Absolute termination tolerance set to "
            + Utilities::toString(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination"))
            + " by GAMS");
        env->output->outputDebug("Relative termination tolerance set to "
            + Utilities::toString(env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination"))
            + " by GAMS");
        env->output->outputDebug("MIP number of threads set to "
            + Utilities::toString(env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual")) + " by GAMS");
    }

    if(gmoOptFile(modelingObject) > 0) // GAMS provides an option file
    {
        gmoNameOptFile(modelingObject, buffer);
        if(std::filesystem::exists(buffer))
        {
            env->output->outputInfo(" Reading options from " + std::string(buffer));
            try
            {
                std::string fileContents = Utilities::getFileAsString(buffer);
                settings->readSettingsFromString(fileContents);
            }
            catch(std::exception& e)
            {
                env->output->outputError("Error when reading GAMS options file " + std::string(buffer), e.what());
                throw std::logic_error("Cannot read GAMS options file.");
            }
        }
        else  /* in GAMS, solvers don't stop if the options file is not present */
            env->output->outputError(" Error: Options file " + std::string(buffer) + " not found.");
    }

    env->output->setLogLevels(static_cast<E_LogLevel>(settings->getSetting<int>("Console.LogLevel", "Output")),
        static_cast<E_LogLevel>(settings->getSetting<int>("File.LogLevel", "Output")));

#ifdef GAMS_BUILD
    /* if CPLEX is set, then check whether GAMS/CPLEX license is present */
    if(env->settings->getSetting<int>("MIP.Solver", "Dual") == (int)ES_MIPSolver::Cplex)
    {
        /* sometimes we would also allow a solver if demo-sized problem, but we don't know how large the MIPs will be */
        if(palLicenseCheckSubSys(pal, const_cast<char*>("OCCPCL")) != 0)
        {
            env->output->outputInfo(" CPLEX chosen as MIP solver, but no GAMS/CPLEX license available. Changing to CBC.");
            env->settings->updateSetting("MIP.Solver", "Dual", (int)ES_MIPSolver::Cbc);
        }
    }
#endif
}

E_ProblemCreationStatus ModelingSystemGAMS::createProblem(
    ProblemPtr& problem, const std::string& filename, const E_GAMSInputSource& inputSource)
{
    if(!std::filesystem::exists(filename))
    {
        env->output->outputError("File \"" + filename + "\" does not exist.");

        return (E_ProblemCreationStatus::FileDoesNotExist);
    }

    try
    {
        if(inputSource == E_GAMSInputSource::ProblemFile)
        {
            createModelFromProblemFile(filename);

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
        }
        else if(inputSource == E_GAMSInputSource::GAMSModel)
        {
            createModelFromGAMSModel(filename);

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
        }
    }
    catch(const Error& eclass)
    {
        env->output->outputError("Error when reading GAMS model from \"" + filename + "\"", eclass.message);

        return (E_ProblemCreationStatus::Error);
    }

    return createProblem(problem);
}

E_ProblemCreationStatus ModelingSystemGAMS::createProblem(ProblemPtr& problem)
{
    assert(modelingObject != nullptr);
    assert(modelingEnvironment != nullptr);

    /* reformulate objective variable out of model, if possible */
    gmoObjReformSet(modelingObject, 1);
    gmoObjStyleSet(modelingObject, gmoObjType_Fun);
    gmoMinfSet(modelingObject, SHOT_DBL_MIN);
    gmoPinfSet(modelingObject, SHOT_DBL_MAX);
    gmoIndexBaseSet(modelingObject, 0);
    gmoUseQSet(modelingObject, 1);

    try
    {
        gmoNameInput(modelingObject, buffer);
        problem->name = buffer;

        /* copyVariables and copyConstraints only return false if there was an unsupported variable or equation type or
         * there were no variables or no equations all cases are SHOT capability problems
         */

        if(!copyVariables(problem))
            return (E_ProblemCreationStatus::CapabilityProblem);

        if(!copyObjectiveFunction(problem))
            return (E_ProblemCreationStatus::ErrorInObjective);

        if(!copyConstraints(problem))
            return (E_ProblemCreationStatus::CapabilityProblem);

        if(!copyLinearTerms(problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if(!copyQuadraticTerms(problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if(!copyNonlinearExpressions(problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        problem->updateProperties();

        bool extractMonomialTerms = env->settings->getSetting<bool>("Reformulation.Monomials.Extract", "Model");
        bool extractSignomialTerms = env->settings->getSetting<bool>("Reformulation.Signomials.Extract", "Model");
        bool extractQuadraticTerms = env->settings->getSetting<bool>("Reformulation.Quadratics.Extract", "Model");

        simplifyNonlinearExpressions(problem, extractMonomialTerms, extractSignomialTerms, extractQuadraticTerms);

        problem->finalize();
    }
    catch(const OperationNotImplementedException& e)
    {
        env->output->outputError("Capability problem when creating problem from GAMS object: ");
        env->output->outputError(e.what());
        return (E_ProblemCreationStatus::CapabilityProblem);
    }
    catch(const std::exception& e)
    {
        env->output->outputError("Error when creating problem from GAMS object.", e.what());

        return (E_ProblemCreationStatus::Error);
    }

    return (E_ProblemCreationStatus::NormalCompletion);
}

void ModelingSystemGAMS::createModelFromProblemFile(const std::string& filename)
{
    char gamscall[1024];
    char buffer[GMS_SSSIZE];
    int rc;
    FILE* convertdopt;

    assert(modelingObject == nullptr);
    assert(modelingEnvironment == nullptr);

    /* create temporary directory */
    std::filesystem::create_directory("loadgms.tmp");
    std::filesystem::permissions("loadgms.tmp", std::filesystem::perms::all);

    createdtmpdir = true;

    /* create empty convertd options file */
    convertdopt = fopen("loadgms.tmp/convertd.opt", "w");
    if(convertdopt == nullptr)
    {
        throw std::logic_error("Could not create convertd options file.");
    }
    fputs(" ", convertdopt);
    fclose(convertdopt);

    /* call GAMS with convertd solver to get compiled model instance in temporary directory
     * we set lo=3 so that we get lo=3 into the gams control file, which is useful for showing the log of GAMS (NLP)
     * solvers later but since we don't want to see the stdout output from gams here, we redirect stdout to /dev/null
     * for this gams call
     */
    snprintf(gamscall, sizeof(gamscall),
#ifdef GAMSDIR
        GAMSDIR "/gams %s SOLVER=CONVERTD SCRDIR=loadgms.tmp output=loadgms.tmp/listing optdir=loadgms.tmp optfile=1 "
                "pf4=0 solprint=0 limcol=0 limrow=0 pc=2 lo=3 > loadgms.tmp/gamsconvert.log",
#else
                "gams %s SOLVER=CONVERTD SCRDIR=loadgms.tmp output=loadgms.tmp/listing optdir=loadgms.tmp optfile=1 "
                "pf4=0 solprint=0 limcol=0 limrow=0 pc=2 lo=3 > loadgms.tmp/gamsconvert.log",
#endif
        filename.c_str());

    /* printf(gamscall); fflush(stdout); */
    rc = system(gamscall);
    if(rc != 0)
    {
        snprintf(buffer, sizeof(buffer), "GAMS call returned with code %d", rc);
        throw std::logic_error(buffer);
    }

    createModelFromGAMSModel("loadgms.tmp/gamscntr.dat");

    /* since we ran convert with options file, GMO now stores convertd.opt as options file, which we don't want to use
     * as a SHOT options file */
    gmoOptFileSet(modelingObject, 0);

    /* do not have GEV catch SIGINT, as with this setup we would not pass this signal on to SHOT */
    gevTerminateUninstall(modelingEnvironment);
}

void ModelingSystemGAMS::createModelFromGAMSModel(const std::string& filename)
{
    char buffer[GMS_SSSIZE];

    /* initialize GMO and GEV libraries */
#ifdef GAMSDIR
    if(!gmoCreateDD(&modelingObject, GAMSDIR, buffer, sizeof(buffer))
        || !gevCreateDD(&modelingEnvironment, GAMSDIR, buffer, sizeof(buffer)))
#else
    if(!gmoCreate(&modelingObject, buffer, sizeof(buffer))
        || !gevCreate(&modelingEnvironment, buffer, sizeof(buffer)))
#endif
        throw std::logic_error(buffer);

    createdgmo = true;

    /* load control file */
    if(gevInitEnvironmentLegacy(modelingEnvironment, filename.c_str()))
    {
        gmoFree(&modelingObject);
        gevFree(&modelingEnvironment);
        throw std::logic_error("Could not load control file loadgms.tmp/gamscntr.dat.");
    }

    if(gmoRegisterEnvironment(modelingObject, modelingEnvironment, buffer))
    {
        gmoFree(&modelingObject);
        gevFree(&modelingEnvironment);
        throw std::logic_error(std::string("Error registering GAMS Environment: ") + buffer);
    }

    if(gmoLoadDataLegacy(modelingObject, buffer))
    {
        gmoFree(&modelingObject);
        gevFree(&modelingEnvironment);
        throw std::logic_error("Could not load model data.");
    }
}

void ModelingSystemGAMS::finalizeSolution()
{
    ResultsPtr r = env->results;
    assert(r != nullptr);
#ifndef NDEBUG
    bool haveSolution = false;
#endif

    // set primal solution and model status
    if(r->hasPrimalSolution())
    {
        gmoSetSolutionPrimal(modelingObject, &r->primalSolution[0]);
#ifndef NDEBUG
        haveSolution = true;
#endif
    }
    else
    {
        gmoModelStatSet(modelingObject, gmoModelStat_NoSolutionReturned);
    }

    // set model status
    switch(r->getModelReturnStatus())
    {
    /*case E_ModelReturnStatus::OptimalLocal:
        gmoModelStatSet(
            modelingObject, gmoNDisc(modelingObject) > 0 ? gmoModelStat_Integer : gmoModelStat_OptimalLocal);
        break; */
    case E_ModelReturnStatus::OptimalGlobal:
        gmoModelStatSet(modelingObject, gmoModelStat_OptimalGlobal);
        break;
    case E_ModelReturnStatus::FeasibleSolution:
        if(env->problem->properties.isDiscrete)
            gmoModelStatSet(modelingObject, gmoModelStat_Integer);
        else
            gmoModelStatSet(modelingObject, gmoModelStat_Feasible);
        break;
    case E_ModelReturnStatus::InfeasibleLocal:
        assert(!haveSolution);
        gmoModelStatSet(modelingObject, gmoModelStat_InfeasibleLocal);
        break;
    case E_ModelReturnStatus::InfeasibleGlobal:
        assert(!haveSolution);
        gmoModelStatSet(modelingObject, gmoModelStat_InfeasibleGlobal);
        break;
    case E_ModelReturnStatus::Unbounded:
        gmoModelStatSet(modelingObject, gmoModelStat_Unbounded);
        break;
    case E_ModelReturnStatus::UnboundedNoSolution:
        gmoModelStatSet(modelingObject, gmoModelStat_UnboundedNoSolution);
        break;
    case E_ModelReturnStatus::NoSolutionReturned:
        gmoModelStatSet(modelingObject, gmoModelStat_NoSolutionReturned);
        break;
    case E_ModelReturnStatus::ErrorUnknown:
        gmoModelStatSet(modelingObject, gmoModelStat_ErrorUnknown);
        break;
    case E_ModelReturnStatus::None:
    case E_ModelReturnStatus::ErrorNoSolution:
        gmoModelStatSet(modelingObject, gmoModelStat_ErrorNoSolution);
    };

    // set solve status and possibly change model status
    switch(r->terminationReason)
    {
    case E_TerminationReason::IterationLimit:
        gmoSolveStatSet(modelingObject, gmoSolveStat_Iteration);
        break;
    case E_TerminationReason::TimeLimit:
        gmoSolveStatSet(modelingObject, gmoSolveStat_Resource);
        break;
    case E_TerminationReason::UserAbort:
        gmoSolveStatSet(modelingObject, gmoSolveStat_User);
        break;
    case E_TerminationReason::InfeasibleProblem:
    case E_TerminationReason::UnboundedProblem:
    case E_TerminationReason::ConstraintTolerance:
    case E_TerminationReason::AbsoluteGap:
    case E_TerminationReason::RelativeGap:
        gmoSolveStatSet(modelingObject, gmoSolveStat_Normal);
        break;
    case E_TerminationReason::ObjectiveStagnation:
    case E_TerminationReason::NoDualCutsAdded:
        gmoSolveStatSet(modelingObject, gmoSolveStat_Solver);
        break;
    case E_TerminationReason::Error:
    case E_TerminationReason::NumericIssues:
        gmoSolveStatSet(modelingObject, gmoSolveStat_SolverErr);
        break;
    case E_TerminationReason::None:
        gmoSolveStatSet(modelingObject, gmoSolveStat_SystemErr);
        break;
    }

    gmoCompleteSolution(modelingObject);

    // set some more statistics, etc
    gmoSetHeadnTail(modelingObject, gmoTmipbest,
        r->currentDualBound); // TODO how do we know that a dual bound has actually been computed
    gmoSetHeadnTail(modelingObject, gmoHiterused, r->getCurrentIteration()->iterationNumber);
    gmoSetHeadnTail(modelingObject, gmoHresused, env->timing->getElapsedTime("Total"));
    // TODO gmoSetHeadnTail(modelingObject, gmoTmipnod,   );
    // TODO? gmoHdomused

    // if we created the GMO object due to starting from a .gms or .dat file, then we should write the solution into a
    // GAMS solution file (though it's probably of no interest if started from .gms and starting from .dat has been
    // removed here)
    if(createdgmo)
        gmoUnloadSolutionLegacy(modelingObject);
}

void ModelingSystemGAMS::clearGAMSObjects()
{
    if(createdgmo && modelingObject != nullptr)
    {
        gmoFree(&modelingObject);
        modelingObject = nullptr;

        assert(modelingEnvironment != nullptr);
        gevFree(&modelingEnvironment);
        modelingEnvironment = nullptr;
    }

    /* remove temporary directory content (should have only files) and directory itself) */
    if(createdtmpdir)
    {
        system("rm loadgms.tmp/* && rmdir loadgms.tmp");
        createdtmpdir = false;
    }
}

bool ModelingSystemGAMS::copyVariables(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy variables between GAMS modeling and SHOT problem objects.");

    int numVariables = gmoN(modelingObject);

    if(numVariables > 0)
    {
        double minLBCont = env->settings->getSetting<double>("ContinuousVariable.MinimumLowerBound", "Model");
        double maxUBCont = env->settings->getSetting<double>("ContinuousVariable.MaximumUpperBound", "Model");
        double minLBInt = env->settings->getSetting<double>("IntegerVariable.MinimumLowerBound", "Model");
        double maxUBInt = env->settings->getSetting<double>("IntegerVariable.MaximumUpperBound", "Model");

        double* variableLBs = new double[numVariables];
        double* variableUBs = new double[numVariables];
        gmoGetVarLower(modelingObject, variableLBs);
        gmoGetVarUpper(modelingObject, variableUBs);

        for(int i = 0; i < numVariables; i++)
        {
            if(gmoDict(modelingObject))
                gmoGetVarNameOne(modelingObject, i, buffer);
            else
                sprintf(buffer, "x%08d", i);

            std::string variableName = buffer;

            E_VariableType variableType;

            switch(gmoGetVarTypeOne(modelingObject, i))
            {
            case gmovar_X:
                variableType = E_VariableType::Real;

                if(variableLBs[i] < minLBCont)
                {
                    variableLBs[i] = minLBCont;
                }

                if(variableUBs[i] > maxUBCont)
                {
                    variableUBs[i] = maxUBCont;
                }

                break;

            case gmovar_B:
                variableType = E_VariableType::Binary;

                if(variableLBs[i] < 0.0)
                {
                    variableLBs[i] = 0.0;
                }

                if(variableUBs[i] > 1.0)
                {
                    variableUBs[i] = 1.0;
                }

                break;

            case gmovar_I:
                variableType = E_VariableType::Integer;

                if(variableLBs[i] < minLBInt)
                {
                    variableLBs[i] = minLBInt;
                }

                if(variableUBs[i] > maxUBInt)
                {
                    variableUBs[i] = maxUBInt;
                }

                break;

            case gmovar_SC:
                variableType = E_VariableType::Semicontinuous;

                if(variableLBs[i] < 0.0)
                {
                    variableLBs[i] = 0.0;
                }

                if(variableUBs[i] > maxUBCont)
                {
                    variableUBs[i] = maxUBCont;
                }

                break;
            case gmovar_SI:
                env->output->outputError("Unsupported variable type.");

                delete[] variableLBs;
                delete[] variableUBs;

                return (false);
                break;
            case gmovar_S1:
                env->output->outputError("Unsupported variable type.");

                delete[] variableLBs;
                delete[] variableUBs;

                return (false);
                break;
            case gmovar_S2:
                env->output->outputError("Unsupported variable type.");

                delete[] variableLBs;
                delete[] variableUBs;

                return (false);
                break;
            default:
                env->output->outputError("Unsupported variable type.");

                delete[] variableLBs;
                delete[] variableUBs;

                return (false);
                break;
            }

            auto variable
                = std::make_shared<SHOT::Variable>(variableName, i, variableType, variableLBs[i], variableUBs[i]);
            destination->add(std::move(variable));
        }
        delete[] variableLBs;
        delete[] variableUBs;
    }
    else
    {
        env->output->outputError("Problem has no variables.");

        return (false);
    }

    env->output->outputDebug("Finished copying variables between GAMS modeling and SHOT problem objects.");

    return (true);
}

bool ModelingSystemGAMS::copyObjectiveFunction(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy objective function between GAMS modeling and SHOT problem objects.");

    // Check if we have an objective at all
    if(gmoModelType(modelingObject) == gmoProc_cns)
    {
        // no objective in constraint satisfaction models
        env->output->outputError("Problem has no objective function.");
        return (false);
    }

    ObjectiveFunctionPtr objectiveFunction;

    switch(gmoGetObjOrder(modelingObject))
    {
    case gmoorder_L:
        objectiveFunction = std::make_shared<LinearObjectiveFunction>();
        break;

    case gmoorder_Q:
        if(env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model")
            >= static_cast<int>(ES_QuadraticProblemStrategy::QuadraticObjective))
            objectiveFunction = std::make_shared<QuadraticObjectiveFunction>();
        else
            objectiveFunction = std::make_shared<NonlinearObjectiveFunction>();
        break;

    case gmoorder_NL:
        objectiveFunction = std::make_shared<NonlinearObjectiveFunction>();
        break;

    default:
        env->output->outputError("Objective function of unknown type.");
        return (false);
        break;
    }

    if(gmoSense(modelingObject) == gmoObj_Min)
        objectiveFunction->direction = E_ObjectiveFunctionDirection::Minimize;
    else
        objectiveFunction->direction = E_ObjectiveFunctionDirection::Maximize;

    objectiveFunction->constant = gmoObjConst(modelingObject);

    // Now copying the linear terms (if any)
    if(gmoN(modelingObject) > 0)
    {
        int* variableIndexes = new int[gmoObjNZ(modelingObject)];
        double* coefficients = new double[gmoObjNZ(modelingObject)];
        int* nonlinearFlags = new int[gmoObjNZ(modelingObject)];
        int numberOfNonzeros;
        int numberOfNonlinearNonzeros;

        gmoGetObjSparse(modelingObject, variableIndexes, coefficients, nonlinearFlags, &numberOfNonzeros,
            &numberOfNonlinearNonzeros);

        int numberLinearTerms = numberOfNonzeros - numberOfNonlinearNonzeros;

        for(int i = 0; i < numberLinearTerms; ++i)
        {
            try
            {
                VariablePtr variable = destination->getVariable(variableIndexes[i]);
                (std::static_pointer_cast<LinearObjectiveFunction>(objectiveFunction))
                    ->add(std::make_shared<LinearTerm>(coefficients[i], variable));
            }
            catch(const VariableNotFoundException& e)
            {
                delete[] variableIndexes;
                delete[] coefficients;
                delete[] nonlinearFlags;

                return (false);
            }
        }

        delete[] variableIndexes;
        delete[] coefficients;
        delete[] nonlinearFlags;
    }

    destination->add(objectiveFunction);

    env->output->outputDebug("Finished copying objective function between GAMS modeling and SHOT problem objects.");

    return (true);
}

bool ModelingSystemGAMS::copyConstraints(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy constraints between GAMS modeling and SHOT problem objects.");

    int numberOfConstraints = gmoM(modelingObject);

    if(numberOfConstraints > 0)
    {
        for(int i = 0; i < numberOfConstraints; i++)
        {
            double lb;
            double ub;

            switch(gmoGetEquTypeOne(modelingObject, i))
            {
            case gmoequ_E:
                lb = ub = gmoGetRhsOne(modelingObject, i);
                break;

            case gmoequ_L:
                lb = SHOT_DBL_MIN;
                ub = gmoGetRhsOne(modelingObject, i);
                break;

            case gmoequ_G:
                lb = gmoGetRhsOne(modelingObject, i);
                ub = SHOT_DBL_MAX;
                break;

            case gmoequ_N:
                lb = SHOT_DBL_MIN;
                ub = SHOT_DBL_MAX;
                break;

            default:
                env->output->outputError("Constraint index" + std::to_string(i) + "is of unknown type.");
                return (false);
            }

            if(gmoDict(modelingObject))
                gmoGetEquNameOne(modelingObject, i, buffer);
            else
                sprintf(buffer, "e%08d", i);

            switch(gmoGetEquOrderOne(modelingObject, i))
            {
            case(gmoorder_L):
            {
                LinearConstraintPtr constraint = std::make_shared<LinearConstraint>(i, buffer, lb, ub);
                destination->add(std::move(constraint));
                break;
            }
            case(gmoorder_Q):
            {
                QuadraticConstraintPtr constraint = std::make_shared<QuadraticConstraint>(i, buffer, lb, ub);
                destination->add(std::move(constraint));
                break;
            }
            case(gmoorder_NL):
            {
                NonlinearConstraintPtr constraint = std::make_shared<NonlinearConstraint>(i, buffer, lb, ub);
                destination->add(std::move(constraint));
                break;
            }
            default:
                env->output->outputError("Constraint index" + std::to_string(i) + "is of unknown type.");
                return (false);
            }
        }
    }
    else
    {
        env->output->outputDebug("GAMS modeling object does not have any constraints.");
    }

    env->output->outputDebug("Finished copying constraints between GAMS modeling and SHOT problem objects.");

    return (true);
}

bool ModelingSystemGAMS::copyLinearTerms(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy linear terms between GAMS modeling and SHOT problem objects.");

    double* linearCoefficients = new double[gmoNZ(modelingObject) + gmoN(modelingObject)];
    int* variableIndexes = new int[gmoNZ(modelingObject) + gmoN(modelingObject)];
    int* nonlinearFlags = new int[gmoN(modelingObject)];

    int numConstraints = gmoM(modelingObject);
    int nz = 0;
    for(int row = 0; row < numConstraints; ++row)
    {
        int rownz;
        int nlnz;

        gmoGetRowSparse(
            modelingObject, row, &variableIndexes[nz], &linearCoefficients[nz], nonlinearFlags, &rownz, &nlnz);

        try
        {
            LinearConstraintPtr constraint
                = std::static_pointer_cast<LinearConstraint>(destination->getConstraint(row));

            for(int j = 0; j < rownz; j++)
            {
                constraint->add(
                    std::make_shared<LinearTerm>(linearCoefficients[j], destination->getVariable(variableIndexes[j])));
            }
        }
        catch(const VariableNotFoundException& e)
        {
            delete[] linearCoefficients;
            delete[] variableIndexes;
            delete[] nonlinearFlags;
            return (false);
        }
        catch(const ConstraintNotFoundException& e)
        {
            delete[] linearCoefficients;
            delete[] variableIndexes;
            delete[] nonlinearFlags;
            return (false);
        }
    }

    delete[] linearCoefficients;
    delete[] variableIndexes;
    delete[] nonlinearFlags;

    env->output->outputDebug("Finished copying linear terms between GAMS modeling and SHOT problem objects.");

    return (true);
}

bool ModelingSystemGAMS::copyQuadraticTerms(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy quadratic terms between GAMS modeling and SHOT problem objects.");

    if(gmoGetObjOrder(modelingObject) == gmoorder_Q)
    {
        int numQuadraticTerms = gmoObjQNZ(modelingObject);

        int* variableOneIndexes = new int[numQuadraticTerms];
        int* variableTwoIndexes = new int[numQuadraticTerms];
        double* quadraticCoefficients = new double[numQuadraticTerms];

        gmoGetObjQ(modelingObject, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);

        for(int j = 0; j < numQuadraticTerms; ++j)
        {
            if(variableOneIndexes[j] == variableTwoIndexes[j])
                quadraticCoefficients[j]
                    /= 2.0; /* for some strange reason, the coefficients on the diagonal are multiplied by 2 in GMO */

            try
            {
                VariablePtr firstVariable = destination->getVariable(variableOneIndexes[j]);
                VariablePtr secondVariable = destination->getVariable(variableTwoIndexes[j]);

                (std::static_pointer_cast<QuadraticObjectiveFunction>(destination->objectiveFunction))
                    ->add(std::make_shared<QuadraticTerm>(quadraticCoefficients[j], firstVariable, secondVariable));
            }
            catch(const VariableNotFoundException& e)
            {
                delete[] variableOneIndexes;
                delete[] variableTwoIndexes;
                delete[] quadraticCoefficients;

                return (false);
            }
        }

        delete[] variableOneIndexes;
        delete[] variableTwoIndexes;
        delete[] quadraticCoefficients;
    }

    int numberOfConstraints = gmoM(modelingObject);

    for(int i = 0; i < numberOfConstraints; ++i)
    {
        if(gmoGetEquOrderOne(modelingObject, i) == gmoorder_Q)
        {
            // handle quadratic equation

            int numQuadraticTerms = gmoGetRowQNZOne(modelingObject, i);

            int* variableOneIndexes = new int[numQuadraticTerms];
            int* variableTwoIndexes = new int[numQuadraticTerms];
            double* quadraticCoefficients = new double[numQuadraticTerms];

            gmoGetRowQ(modelingObject, i, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);

            for(int j = 0; j < numQuadraticTerms; ++j)
            {
                if(variableOneIndexes[j] == variableTwoIndexes[j])
                    quadraticCoefficients[j] /= 2.0; /* for some strange reason, the coefficients on the diagonal are
                                                        multiplied by 2 in GMO */

                try
                {
                    VariablePtr firstVariable = destination->getVariable(variableOneIndexes[j]);
                    VariablePtr secondVariable = destination->getVariable(variableTwoIndexes[j]);

                    auto constraint = std::static_pointer_cast<QuadraticConstraint>(destination->getConstraint(i));
                    constraint->add(
                        std::make_shared<QuadraticTerm>(quadraticCoefficients[j], firstVariable, secondVariable));
                }
                catch(const VariableNotFoundException& e)
                {
                    delete[] variableOneIndexes;
                    delete[] variableTwoIndexes;
                    delete[] quadraticCoefficients;

                    return (false);
                }
                catch(const ConstraintNotFoundException& e)
                {
                    delete[] variableOneIndexes;
                    delete[] variableTwoIndexes;
                    delete[] quadraticCoefficients;

                    return (false);
                }
            }
        }
    }

    env->output->outputDebug("Finished copying quadratic terms between GAMS modeling and SHOT problem objects.");

    return (true);
}

bool ModelingSystemGAMS::copyNonlinearExpressions(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy nonlinear expressions between GAMS modeling and SHOT problem objects.");

    int* opcodes = new int[gmoNLCodeSizeMaxRow(modelingObject) + 1];
    int* fields = new int[gmoNLCodeSizeMaxRow(modelingObject) + 1];
    int constantlen = gmoNLConst(modelingObject);
    double* constants = (double*)gmoPPool(modelingObject);
    int codelen;

    if(gmoObjNLNZ(modelingObject) > 0 && gmoGetObjOrder(modelingObject) == gmoorder_NL)
    {
        // handle nonlinear objective

        gmoDirtyGetObjFNLInstr(modelingObject, &codelen, opcodes, fields);

        try
        {
            auto destinationExpression
                = parseGamsInstructions(codelen, opcodes, fields, constantlen, constants, destination);

            if(codelen > 0)
            {
                double objjacval = gmoObjJacVal(modelingObject);
                if(objjacval == 1.0)
                {
                    // scale by -1/objjacval = negate
                    destinationExpression = std::make_shared<ExpressionNegate>(destinationExpression);
                }
                else if(objjacval != -1.0)
                {
                    // scale by -1/objjacval
                    destinationExpression = std::make_shared<ExpressionProduct>(
                        std::make_shared<ExpressionConstant>(-1 / objjacval), destinationExpression);
                }

                auto objective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination->objectiveFunction);
                objective->add(std::move(destinationExpression));
            }
        }
        catch(const ConstraintNotFoundException& e)
        {
            return (false);
        }
    }

    for(int i = 0; i < gmoM(modelingObject); ++i)
    {
        if(gmoGetEquOrderOne(modelingObject, i) == gmoorder_NL)
        {
            gmoDirtyGetRowFNLInstr(modelingObject, i, &codelen, opcodes, fields);
            if(codelen == 0)
                continue;

            try
            {
                auto destinationExpression
                    = parseGamsInstructions(codelen, opcodes, fields, constantlen, constants, destination);

                auto constraint = std::dynamic_pointer_cast<NonlinearConstraint>(destination->getConstraint(i));
                constraint->add(std::move(destinationExpression));
            }
            catch(const ConstraintNotFoundException& e)
            {
                return (false);
            }
        }
    }

    env->output->outputDebug("Finished copying nonlinear expressions between GAMS modeling and SHOT problem objects.");

    return (true);
}

NonlinearExpressionPtr ModelingSystemGAMS::parseGamsInstructions(int codelen, /**< length of GAMS instructions */
    int* opcodes, /**< opcodes of GAMS instructions */
    int* fields, /**< fields of GAMS instructions */
    [[maybe_unused]] int constantlen, /**< length of GAMS constants pool */
    double* constants, /**< GAMS constants pool */
    const ProblemPtr& destination)
{
    bool debugoutput = gevGetIntOpt(modelingEnvironment, gevInteger1) & 0x4;
#define debugout                                                                                                       \
    if(debugoutput)                                                                                                    \
    std::clog

    std::vector<NonlinearExpressionPtr> stack;
    stack.reserve(20);

    for(int i = 0; i < codelen; ++i)
    {
        auto opcode = (GamsOpCode)opcodes[i];
        int address = fields[i] - 1;

        debugout << '\t' << GamsOpCodeName[opcode] << ": ";

        switch(opcode)
        {
        case nlNoOp: // no operation
        case nlStore: // store row
        case nlHeader: // header
        {
            // debugout << "ignored" << std::endl;
            break;
        }

        case nlPushV: // push variable
        {
            address = gmoGetjSolver(modelingObject, address);
            stack.push_back(std::make_shared<ExpressionVariable>(destination->getVariable(address)));
            break;
        }

        case nlPushI: // push constant
        {
            stack.push_back(std::make_shared<ExpressionConstant>(constants[address]));
            break;
        }

        case nlPushZero: // push zero
        {
            stack.push_back(std::make_shared<ExpressionConstant>(0.0));
            break;
        }

        case nlAdd: // add
        {
            auto expression = std::make_shared<ExpressionSum>(stack.rbegin()[1], stack.rbegin()[0]);
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlAddV: // add variable
        {
            address = gmoGetjSolver(modelingObject, address);
            auto expression = std::make_shared<ExpressionSum>(
                std::make_shared<ExpressionVariable>(destination->getVariable(address)), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlAddI: // add immediate
        {
            auto expression = std::make_shared<ExpressionSum>(
                std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlSub: // minus
        {
            auto expression = std::make_shared<ExpressionSum>(
                stack.rbegin()[1], std::make_shared<ExpressionNegate>(stack.rbegin()[0]));
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlSubV: // subtract variable
        {
            address = gmoGetjSolver(modelingObject, address);
            auto expression = std::make_shared<ExpressionSum>(stack.rbegin()[0],
                std::make_shared<ExpressionNegate>(
                    std::make_shared<ExpressionVariable>(destination->getVariable(address))));
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlSubI: // subtract immediate
        {
            auto expression = std::make_shared<ExpressionSum>(stack.rbegin()[0],
                std::make_shared<ExpressionNegate>(std::make_shared<ExpressionConstant>(constants[address])));
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMul: // multiply
        {
            auto expression = std::make_shared<ExpressionProduct>((stack.rbegin()[1]), (stack.rbegin()[0]));
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMulV: // multiply variable
        {
            address = gmoGetjSolver(modelingObject, address);
            auto expression = std::make_shared<ExpressionProduct>(
                std::make_shared<ExpressionVariable>(destination->getVariable(address)), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMulI: // multiply immediate
        {
            auto expression = std::make_shared<ExpressionProduct>(
                std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMulIAdd: // multiply immediate and add
        {
            auto expressionProduct = std::make_shared<ExpressionProduct>(
                std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expressionProduct);
            auto expressionSum = std::make_shared<ExpressionSum>(stack.rbegin()[1], stack.rbegin()[0]);
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expressionSum);
            break;
        }

        case nlDiv: // divide
        {
            auto expression = std::make_shared<ExpressionDivide>(stack.rbegin()[1], stack.rbegin()[0]);
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlDivV: // divide variable
        {
            address = gmoGetjSolver(modelingObject, address);
            auto expression = std::make_shared<ExpressionDivide>(
                std::make_shared<ExpressionVariable>(destination->getVariable(address)), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlDivI: // divide immediate
        {
            auto expression = std::make_shared<ExpressionDivide>(
                std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlUMin: // unary minus
        {
            auto expression = std::make_shared<ExpressionNegate>(stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlUMinV: // unary minus variable
        {
            address = gmoGetjSolver(modelingObject, address);
            stack.push_back(std::make_shared<ExpressionNegate>(
                std::make_shared<ExpressionVariable>(destination->getVariable(address))));
            break;
        }

        case nlFuncArgN: // number of function arguments
        {
            break;
        }

        case nlCallArg1:
        case nlCallArg2:
        case nlCallArgN:
        {
            debugout << "call function ";

            switch(GamsFuncCode(address + 1))
            // undo shift by 1
            {

            case fnsqr:
            {
                auto expression = std::make_shared<ExpressionSquare>(stack.rbegin()[0]);
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnexp:
            {
                auto expression = std::make_shared<ExpressionExp>(stack.rbegin()[0]);
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnlog:
            {
                auto expression = std::make_shared<ExpressionLog>(stack.rbegin()[0]);
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnlog10:
            {
                auto expression
                    = std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(1.0 / log(10.0)),
                        std::make_shared<ExpressionLog>(stack.rbegin()[0]));

                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnlog2:
            {
                auto expression
                    = std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(1.0 / log(2.0)),
                        std::make_shared<ExpressionLog>(stack.rbegin()[0]));
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnsqrt:
            {
                auto expression = std::make_shared<ExpressionSquareRoot>(stack.rbegin()[0]);
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnabs:
            {
                auto expression = std::make_shared<ExpressionAbs>(stack.rbegin()[0]);
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fncos:
            {
                auto expression = std::make_shared<ExpressionCos>(stack.rbegin()[0]);
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnsin:
            {
                auto expression = std::make_shared<ExpressionSin>(stack.rbegin()[0]);
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnpower:
            case fnrpower: // x ^ y
            case fncvpower: // constant ^ x
            case fnvcpower: // x ^ constant
            {
                auto expression = std::make_shared<ExpressionPower>(stack.rbegin()[1], stack.rbegin()[0]);
                stack.pop_back();
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnpi:
            {
                stack.push_back(std::make_shared<ExpressionConstant>(3.14159265));
                break;
            }

            case fndiv:
            {
                auto expression = std::make_shared<ExpressionDivide>(stack.rbegin()[1], stack.rbegin()[0]);
                stack.pop_back();
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            // TODO some more we could handle
            case fnpoly:
            case fnmin:
            case fnmax:
            case fnerrf:
            case fnceil:
            case fnfloor:
            case fnround:
            case fnmod:
            case fntrunc:
            case fnsign:
            case fnarctan:
            case fndunfm:
            case fndnorm:
            case fnerror:
            case fnfrac:
            case fnerrorl:
            case fnfact /* factorial */:
            case fnunfmi /* uniform random number */:
            case fnncpf /* fischer: sqrt(x1^2+x2^2+2*x3) */:
            case fnncpcm /* chen-mangasarian: x1-x3*ln(1+exp((x1-x2)/x3))*/:
            case fnentropy /* x*ln(x) */:
            case fnsigmoid /* 1/(1+exp(-x)) */:
            case fnboolnot:
            case fnbooland:
            case fnboolor:
            case fnboolxor:
            case fnboolimp:
            case fnbooleqv:
            case fnrelopeq:
            case fnrelopgt:
            case fnrelopge:
            case fnreloplt:
            case fnrelople:
            case fnrelopne:
            case fnifthen:
            case fnedist /* euclidian distance */:
            case fncentropy /* x*ln((x+d)/(y+d))*/:
            case fngamma:
            case fnloggamma:
            case fnbeta:
            case fnlogbeta:
            case fngammareg:
            case fnbetareg:
            case fnsinh:
            case fncosh:
            case fntanh:
            case fnsignpower /* sign(x)*abs(x)^c */:
            case fnncpvusin /* veelken-ulbrich */:
            case fnncpvupow /* veelken-ulbrich */:
            case fnbinomial:
            case fntan:
            case fnarccos:
            case fnarcsin:
            case fnarctan2 /* arctan(x2/x1) */:
            default:
            {
                debugout << "nr. " << address + 1 << " - unsuppored. Error." << std::endl;
                char buffer[256];
                sprintf(buffer, "Error: Unsupported GAMS function %s.\n", GamsFuncCodeName[address + 1]);
                gevLogStatPChar(modelingEnvironment, buffer);
                throw OperationNotImplementedException("Error: Unsupported GAMS function " + std::string(buffer));
            }
            }
            break;
        }

        default:
        {
            debugout << "opcode " << opcode << " - unsuppored. Error." << std::endl;
            char buffer[256];
            sprintf(buffer, "Error: Unsupported GAMS opcode %s.\n", GamsOpCodeName[opcode]);
            gevLogStatPChar(modelingEnvironment, buffer);
            throw OperationNotImplementedException("Error: Unsupported GAMS opcode " + std::string(buffer));
        }
        }
    }

    assert(stack.size() == 1);
    return stack[0];
#undef debugout
}

} // Namespace SHOT