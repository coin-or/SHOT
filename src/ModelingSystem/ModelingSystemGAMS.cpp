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
#include "GamsLicensing.h"
#include "GamsHSLInit.h"
#endif

#if defined(_WIN32)
#define WEXITSTATUS(x) (x)
#endif

#include <cstdio> // for tmpnam()
#include <cstdlib> // for mkdtemp()
#include <climits>
#include <fstream>

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

ModelingSystemGAMS::ModelingSystemGAMS(EnvironmentPtr envPtr)
    : IModelingSystem(envPtr)
    , modelingObject(nullptr)
    , modelingEnvironment(nullptr)
    , auditLicensing(nullptr)
    , createdtmpdir(false)
    , createdgmo(false)
{
}

void ModelingSystemGAMS::setModelingObject(gmoHandle_t gmo)
{
    modelingObject = gmo;
    modelingEnvironment = (gevHandle_t)gmoEnvironment(gmo);

    createAuditLicensing();
}

ModelingSystemGAMS::~ModelingSystemGAMS()
{
    clearGAMSObjects();

    if(createdgmo)
    {
        gmoLibraryUnload();
        gevLibraryUnload();
        palLibraryUnload();
    }
}

void ModelingSystemGAMS::augmentSettings([[maybe_unused]] SettingsPtr settings)
{
    // Subsolver settings: GAMS NLP

    env->settings->createSettingGroup("Subsolver", "GAMS", "GAMS", "Settings for the GAMS NLP solvers.");

    std::string optfile = "";
    env->settings->createSetting(
        "GAMS.NLP.OptionsFilename", "Subsolver", optfile, "Options file for the NLP solver in GAMS");

    std::string solver = "auto";
    env->settings->createSetting(
        "GAMS.NLP.Solver", "Subsolver", solver, "NLP solver to use in GAMS (auto: SHOT chooses)");

#if GMOAPIVERSION >= 21
    env->settings->createSettingGroup(
        "ModelingSystem", "GAMS", "GAMS interface", "These settings control functionality used in the GAMS interface.");

    VectorString enumQExtractAlg;
    enumQExtractAlg.push_back("automatic");
    enumQExtractAlg.push_back("threepass");
    enumQExtractAlg.push_back("doubleforward");
    env->settings->createSetting("GAMS.QExtractAlg", "ModelingSystem", 0,
        "Extraction algorithm for quadratic equations in GAMS interface", enumQExtractAlg);
#endif
}

void ModelingSystemGAMS::updateSettings(SettingsPtr settings)
{
    assert(modelingEnvironment != nullptr);
    assert(modelingObject != nullptr);

#ifdef GAMS_BUILD
    assert(auditLicensing != nullptr);

    /* if IPOPTH is licensed, use MA27, otherwise Mumps */
    if(GAMScheckIpoptLicense(auditLicensing, false))
    {
        GamsHSLInit();
        env->settings->updateSetting("Ipopt.LinearSolver", "Subsolver", static_cast<int>(ES_IpoptSolver::ma27));
    }
    else
        env->settings->updateSetting("Ipopt.LinearSolver", "Subsolver", static_cast<int>(ES_IpoptSolver::mumps));
#endif

    // Process GAMS options.
    // We do not want to use GAMS defaults if called on a gms file, in which case we would have created our own GMO.
    if(!createdgmo)
    {
        // Sets time limit
        env->settings->updateSetting("TimeLimit", "Termination", gevGetDblOpt(modelingEnvironment, gevResLim));
        env->output->outputDebug(
            fmt::format("Time limit set to {} by GAMS", env->settings->getSetting<double>("TimeLimit", "Termination")));

        // Sets iteration limit, if different than SHOT default
        if(gevGetIntOpt(modelingEnvironment, gevIterLim) < INT_MAX)
        {
            env->settings->updateSetting(
                "IterationLimit", "Termination", gevGetIntOpt(modelingEnvironment, gevIterLim));
            env->output->outputDebug(fmt::format(
                "Iteration limit set to {} by GAMS", env->settings->getSetting<int>("IterationLimit", "Termination")));
        }
        else
        {
            env->settings->updateSetting("IterationLimit", "Termination", SHOT_INT_MAX);
        }

        // Sets absolute objective gap tolerance
        env->settings->updateSetting(
            "ObjectiveGap.Absolute", "Termination", gevGetDblOpt(modelingEnvironment, gevOptCA));
        env->output->outputDebug(fmt::format("Absolute termination tolerance set to {} by GAMS",
            env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination")));

        // Sets relative objective gap tolerance
        env->settings->updateSetting(
            "ObjectiveGap.Relative", "Termination", gevGetDblOpt(modelingEnvironment, gevOptCR));
        env->output->outputDebug(fmt::format("Relative termination tolerance set to {} by GAMS",
            env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination")));

        // Sets cutoff value for dual solver
        if(gevGetIntOpt(modelingEnvironment, gevUseCutOff) == 1)
        {
            env->settings->updateSetting("MIP.CutOff.UseInitialValue", "Dual", true);
            env->settings->updateSetting(
                "MIP.CutOff.InitialValue", "Dual", gevGetDblOpt(modelingEnvironment, gevCutOff));
        }

        // Sets node limit for dual solver
        if(gevGetIntOpt(modelingEnvironment, gevNodeLim) > 0)
        {
            env->settings->updateSetting(
                "MIP.NodeLimit", "Dual", (double)gevGetIntOpt(modelingEnvironment, gevNodeLim));
        }

        // Sets the number of threads
        env->settings->updateSetting("MIP.NumberOfThreads", "Dual", gevThreads(modelingEnvironment));
        env->output->outputDebug(fmt::format(
            "MIP number of threads set to {} by GAMS", env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual")));

        // Uses NLP solver in GAMS by default, Ipopt can be used directly if value set by user in options file (read
        // below)
        env->settings->updateSetting("FixedInteger.Solver", "Primal", static_cast<int>(ES_PrimalNLPSolver::GAMS));
    }

    if(gmoOptFile(modelingObject) > 0) // GAMS provides an option file
    {
        gmoNameOptFile(modelingObject, buffer);
        if(fs::filesystem::exists(buffer))
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
        else /* in GAMS, solvers don't stop if the options file is not present */
            env->output->outputError(" Error: Options file " + std::string(buffer) + " not found.");
    }

    env->output->setLogLevels(static_cast<E_LogLevel>(settings->getSetting<int>("Console.LogLevel", "Output")),
        static_cast<E_LogLevel>(settings->getSetting<int>("File.LogLevel", "Output")));

#ifdef GAMS_BUILD
    /* if CPLEX is set, then check whether GAMS/CPLEX license is present */
    if(env->settings->getSetting<int>("MIP.Solver", "Dual") == (int)ES_MIPSolver::Cplex)
    {
        /* sometimes we would also allow a solver if demo-sized problem, but we don't know how large the MIPs will be */
        if(!GAMScheckCPLEXLicense(auditLicensing, true))
        {
            env->output->outputInfo(
                " CPLEX chosen as MIP solver, but no GAMS/CPLEX license available. Changing to CBC.");
            env->settings->updateSetting("MIP.Solver", "Dual", (int)ES_MIPSolver::Cbc);
        }
    }
#endif
}
E_ProblemCreationStatus ModelingSystemGAMS::createProblem(
    ProblemPtr& problem, const std::string& filename, const E_GAMSInputSource& inputSource)
{
    if(!fs::filesystem::exists(filename))
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
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format(" Error when reading GAMS model from \"{}\".\n {}", filename, e.what()));

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

#if GMOAPIVERSION >= 21
    gmoQExtractAlgSet(modelingObject, env->settings->getSetting<int>("GAMS.QExtractAlg", "ModelingSystem"));
    gevTimeDiff(modelingEnvironment);
#endif
    gmoUseQSet(modelingObject, 1);
#if GMOAPIVERSION >= 21
    env->output->outputInfo(
        std::string(" Time to extract information on quadratics: ") + std::to_string(gevTimeDiff(modelingEnvironment)));
#endif

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
        bool extractQuadraticTerms
            = (env->settings->getSetting<int>("Reformulation.Quadratics.ExtractStrategy", "Model")
                >= static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame));

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
    char buffer[GMS_SSSIZE];
    int rc;

    assert(modelingObject == nullptr);
    assert(modelingEnvironment == nullptr);

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        tmpdirname = Utilities::createTemporaryDirectory(
            "SHOT_GAMS_", env->settings->getSetting<std::string>("Debug.Path", "Output"));
    else
        tmpdirname = Utilities::createTemporaryDirectory("SHOT_GAMS_");

    if(tmpdirname == "")
        throw std::logic_error("Could not create temporary directory.");

    createdtmpdir = true;

    /* create empty convert options file */
    std::ofstream convertopt((fs::filesystem::path(tmpdirname) / "convert.opt").string(), std::ios::out);

    if(!convertopt.good())
    {
        throw std::logic_error("Could not create convert options file.");
    }

    convertopt << " " << std::endl;
    convertopt.close();

    /* call GAMS with convert solver to get compiled model instance in temporary directory
     * we set lo=3 so that we get lo=3 into the gams control file, which is useful for showing the log of GAMS (NLP)
     * solvers later but since we don't want to see the stdout output from gams here, we redirect stdout to /dev/null
     * for this gams call
     */
    std::string gamscall;
#ifdef GAMSDIR
    gamscall = (fs::filesystem::path(GAMSDIR) / "gams").string();
#else
    gamscall = "gams";
#endif
    gamscall += " \"" + filename + "\"";
    gamscall += " SOLVER=CONVERT PF4=0 SOLPRINT=0 LIMCOL=0 LIMROW=0 PC=2";
    gamscall += " SCRDIR=" + tmpdirname;
    gamscall += " OUTPUT=" + (fs::filesystem::path(tmpdirname) / "listing").string();
    gamscall += " OPTFILE=1 OPTDIR=" + tmpdirname;
    gamscall += " LO=3 > " + (fs::filesystem::path(tmpdirname) / "gamsconvert.log").string();
    // printf(gamscall.c_str()); fflush(stdout);

    rc = system(gamscall.c_str());
    switch(WEXITSTATUS(rc))
    {
    case 0: /* Normal return */
        break;

    case 2: /* Compilation error */
    case 3: /* Execution error */
    {
        std::string msg;

        if(WEXITSTATUS(rc) == 2)
            msg = "GAMS call returned with compilation error:\n";
        else
            msg = "GAMS call returned with execution error:\n";

        std::ifstream lst((fs::filesystem::path(tmpdirname) / "listing").string());
        std::string line;
        while(lst.good() && !lst.eof())
        {
            getline(lst, line);
            if(line.find("****") == 0 && line != "**** FILE SUMMARY")
            {
                msg.append(1, '\n');
                msg.append(1, ' ');
                msg += line;
            }
        }

        msg.append(1, '\n');

        throw std::logic_error(msg);
        break;
    }

    default:
    {
        std::string msg;

        switch(WEXITSTATUS(rc))
        {
        case 4: /* System limits reached */
            msg = "GAMS call returned with system limits reached.";
            break;
        case 5: /* File error */
            msg = "GAMS call returned with file error.";
            break;
        case 6: /* Parameter */
            msg = "GAMS call returned with parameter error.";
            break;
        case 7: /* Licensing error */
            msg = "GAMS call returned with licensing error.";
            break;
        case 8: /* System error */
            msg = "GAMS call returned with system error.";
            break;
        case 9: /* GAMS could not be started */
            msg = "GAMS could not be started.";
            break;
        case 10: /* out of memory */
            msg = "GAMS ran out of memory.";
            break;
        case 11: /* out of disk */
            msg = "GAMS ran out of disk space.";
            break;
        default: /* other errors, I don't want to handle each of them here... */
            snprintf(buffer, sizeof(buffer),
                "GAMS call returned with exit code %d (see also "
                "https://www.gams.com/latest/docs/UG_GAMSReturnCodes.html#UG_GAMSReturnCodes_ListOfErrorCodes).",
                WEXITSTATUS(rc));
            msg = buffer;
            break;
        }

        std::ifstream log((fs::filesystem::path(tmpdirname) / "gamsconvert.log").string());
        std::string line;
        msg += " GAMS log:\n";
        while(log.good() && !log.eof())
        {
            getline(log, line);
            msg.append(1, '\n');
            msg.append(1, ' ');
            msg += line;
        }

        throw std::logic_error(msg);
    }
    }

    createModelFromGAMSModel((fs::filesystem::path(tmpdirname) / "gamscntr.dat").string());

    /* since we ran convert with options file, GMO now stores convert.opt as options file, which we don't want to use
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
    if(!gmoCreate(&modelingObject, buffer, sizeof(buffer)) || !gevCreate(&modelingEnvironment, buffer, sizeof(buffer)))
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

    createAuditLicensing();
}

void ModelingSystemGAMS::createAuditLicensing()
{
    assert(auditLicensing == nullptr);

    char msg[GMS_SSSIZE];
#ifdef GAMSDIR
    if(!palCreateD(&auditLicensing, GAMSDIR, msg, sizeof(msg)))
#else
    if(!palCreate(&auditLicensing, msg, sizeof(msg)))
#endif
        throw std::logic_error(msg);

    char buf[80];
    palLicenseRegisterGAMS(auditLicensing, 1, gevGetStrOpt(modelingEnvironment, "License1", buf));
    palLicenseRegisterGAMS(auditLicensing, 2, gevGetStrOpt(modelingEnvironment, "License2", buf));
    palLicenseRegisterGAMS(auditLicensing, 3, gevGetStrOpt(modelingEnvironment, "License3", buf));
    palLicenseRegisterGAMS(auditLicensing, 4, gevGetStrOpt(modelingEnvironment, "License4", buf));
    palLicenseRegisterGAMS(auditLicensing, 5, gevGetStrOpt(modelingEnvironment, "License5", buf));
    palLicenseRegisterGAMS(auditLicensing, 6, gevGetStrOpt(modelingEnvironment, "License6", buf));
    palLicenseRegisterGAMSDone(auditLicensing);

    palLicenseCheck(auditLicensing, gmoM(modelingObject), gmoN(modelingObject), gmoNZ(modelingObject),
        gmoNLNZ(modelingObject), gmoNDisc(modelingObject));
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
        gmoSolveStatSet(modelingObject, gmoSolveStat_Normal);
        break;
    }

    gmoCompleteSolution(modelingObject);

    // set some more statistics, etc
    gmoSetHeadnTail(modelingObject, gmoTmipbest, r->getGlobalDualBound());
    gmoSetHeadnTail(modelingObject, gmoHiterused, r->getCurrentIteration()->iterationNumber);
    // TODO this seems to be 0: gmoSetHeadnTail(modelingObject, gmoHiterused,
    // env->solutionStatistics.numberOfIterations);
    gmoSetHeadnTail(modelingObject, gmoHresused, env->timing->getElapsedTime("Total"));
    gmoSetHeadnTail(modelingObject, gmoTmipnod, env->solutionStatistics.numberOfExploredNodes);

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

    if(auditLicensing != nullptr)
    {
        palFree(&auditLicensing);
        auditLicensing = nullptr;
    }

    /* remove temporary directory contents if not in debug mode (should have only files) and directory itself) */
    if(createdtmpdir && !env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        fs::filesystem::remove_all(tmpdirname);
        createdtmpdir = false;
    }
}

bool ModelingSystemGAMS::copyVariables(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy variables between GAMS modeling and SHOT problem objects.");

    int numVariables = gmoN(modelingObject);

    if(numVariables > 0)
    {
        double minLBCont = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
        double maxUBCont = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");
        double minLBInt = env->settings->getSetting<double>("Variables.Integer.MinimumLowerBound", "Model");
        double maxUBInt = env->settings->getSetting<double>("Variables.Integer.MaximumUpperBound", "Model");

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
        objectiveFunction = std::make_shared<QuadraticObjectiveFunction>();
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
            catch(const VariableNotFoundException&)
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
        catch(const VariableNotFoundException&)
        {
            delete[] linearCoefficients;
            delete[] variableIndexes;
            delete[] nonlinearFlags;
            return (false);
        }
        catch(const ConstraintNotFoundException&)
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
#if GMOAPIVERSION <= 19
        int numQuadraticTerms = gmoObjQNZ(modelingObject);
#else
        int numQuadraticTerms = gmoObjQMatNZ(modelingObject);
#endif

        int* variableOneIndexes = new int[numQuadraticTerms];
        int* variableTwoIndexes = new int[numQuadraticTerms];
        double* quadraticCoefficients = new double[numQuadraticTerms];

#if GMOAPIVERSION <= 19
        gmoGetObjQ(modelingObject, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);
#else
        gmoGetObjQMat(modelingObject, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);
#endif

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
            catch(const VariableNotFoundException&)
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

#if GMOAPIVERSION <= 19
            gmoGetRowQ(modelingObject, i, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);
#else
            gmoGetRowQMat(modelingObject, i, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);
#endif

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
                catch(const VariableNotFoundException&)
                {
                    delete[] variableOneIndexes;
                    delete[] variableTwoIndexes;
                    delete[] quadraticCoefficients;

                    return (false);
                }
                catch(const ConstraintNotFoundException&)
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
        catch(const ConstraintNotFoundException&)
        {
            delete[] opcodes;
            delete[] fields;
            delete[] constants;
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
            catch(const ConstraintNotFoundException&)
            {
                delete[] opcodes;
                delete[] fields;
                delete[] constants;
                return (false);
            }
        }
    }

    delete[] opcodes;
    delete[] fields;

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
                stack.rbegin()[0], std::make_shared<ExpressionVariable>(destination->getVariable(address)));
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlDivI: // divide immediate
        {
            auto expression = std::make_shared<ExpressionDivide>(
                stack.rbegin()[0], std::make_shared<ExpressionConstant>(constants[address]));
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
                throw OperationNotImplementedException(fmt::format("Error: Unsupported GAMS function {}", buffer));
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
            throw OperationNotImplementedException(fmt::format("Error: Unsupported GAMS opcode {}", buffer));
        }
        }
    }

    assert(stack.size() == 1);
    return stack[0];
#undef debugout
}

} // Namespace SHOT