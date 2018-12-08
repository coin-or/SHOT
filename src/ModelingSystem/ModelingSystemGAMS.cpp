/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "ModelingSystemGAMS.h"

namespace SHOT
{

ModelingSystemGAMS::ModelingSystemGAMS(EnvironmentPtr envPtr) : IModelingSystem(envPtr), modelingObject(NULL), modelingEnvironment(NULL), createdtmpdir(false)
{
}

ModelingSystemGAMS::~ModelingSystemGAMS()
{
    clearGAMSObjects();

    gmoLibraryUnload();
    gevLibraryUnload();
}

void ModelingSystemGAMS::augmentSettings(SettingsPtr settings)
{
}

void ModelingSystemGAMS::updateSettings(SettingsPtr settings)
{
    if (gmoOptFile(modelingObject) > 0) // GAMS provides an option file
    {
        gmoNameOptFile(modelingObject, buffer);
        gevLogPChar(modelingEnvironment, "Reading options from ");
        gevLog(modelingEnvironment, buffer);

        if (!boost::filesystem::exists(buffer))
            throw std::logic_error("Options file not found.");

        try
        {
            std::string fileContents = UtilityFunctions::getFileAsString(buffer);

            settings->readSettingsFromGAMSOptFormat(fileContents);
        }
        catch (std::exception &e)
        {
            env->output->outputError("Error when reading GAMS options file" + std::string(buffer));
            throw std::logic_error("Cannot read GAMS options file from.");
        }
    }
    else // get default settings from GAMS
    {
        // Removed this functionality, since otherwise we cannot control the time limit from an options file when SHOT is called on a gms file
        /*env->settings->updateSetting("TimeLimit", "Termination", gevGetDblOpt(gev, gevResLim));
		 env->settings->updateSetting("ObjectiveGap.Absolute", "Termination", gevGetDblOpt(gev, gevOptCA));
		 env->settings->updateSetting("ObjectiveGap.Relative", "Termination", gevGetDblOpt(gev, gevOptCR));


		 env->output->outputInfo(
		 "Time limit set to "
		 + UtilityFunctions::toString(env->settings->getDoubleSetting("TimeLimit", "Termination"))
		 + " by GAMS");
		 env->output->outputInfo(
		 "Absolute termination tolerance set to "
		 + UtilityFunctions::toString(
		 env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination"))
		 + " by GAMS");
		 env->output->outputInfo(
		 "Relative termination tolerance set to "
		 + UtilityFunctions::toString(
		 env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination"))
		 + " by GAMS");

		 */
    }

    // want to solve the NLP problems with GAMS
    settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::GAMS);
}

E_ProblemCreationStatus ModelingSystemGAMS::createProblem(ProblemPtr &problem, const std::string &filename, const E_GAMSInputSource &inputSource)
{
    if (!boost::filesystem::exists(filename))
    {
        env->output->outputError("File \"" + filename + "\" does not exist.");

        return (E_ProblemCreationStatus::FileDoesNotExist);
    }

    try
    {
        if (inputSource == E_GAMSInputSource::ProblemFile)
        {
            createModelFromProblemFile(filename);

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
        }
        else if (inputSource == E_GAMSInputSource::GAMSModel)
        {
            createModelFromGAMSModel(filename);

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
        }
    } // namespace SHOT
    catch (const ErrorClass &eclass)
    {
        env->output->outputError("Error when reading GAMS model from \"" + filename + "\"", eclass.errormsg);

        return (E_ProblemCreationStatus::Error);
    }

    /* reformulate objective variable out of model, if possible */
    gmoObjReformSet(modelingObject, 1);
    gmoObjStyleSet(modelingObject, gmoObjType_Fun);
    gmoMinfSet(modelingObject, -OSDBL_MAX);
    gmoPinfSet(modelingObject, OSDBL_MAX);
    gmoIndexBaseSet(modelingObject, 0);
    gmoUseQSet(modelingObject, 1);

    //env->process->GAMSModelingObject = gmo;

    try
    {
        gmoNameInput(modelingObject, buffer);
        problem->name = buffer;

        if (!copyVariables(problem))
            return (E_ProblemCreationStatus::ErrorInVariables);

        if (!copyObjectiveFunction(problem))
            return (E_ProblemCreationStatus::ErrorInObjective);

        if (!copyConstraints(problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if (!copyLinearTerms(problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if (!copyQuadraticTerms(problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if (!copyNonlinearExpressions(problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        problem->finalize();
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when creating problem from GAMS object.");

        return (E_ProblemCreationStatus::Error);
    }

    return (E_ProblemCreationStatus::NormalCompletion);
}

void ModelingSystemGAMS::createModelFromProblemFile(const std::string &filename)
{
    char gamscall[1024];
    char buffer[GMS_SSSIZE];
    int rc;
    FILE *convertdopt;

    assert(modelingObject == NULL);
    assert(modelingEnvironment == NULL);

    /* create temporary directory */
    mkdir("loadgms.tmp", S_IRWXU);
    createdtmpdir = true;

    /* create empty convertd options file */
    convertdopt = fopen("loadgms.tmp/convertd.opt", "w");
    if (convertdopt == NULL)
    {
        throw std::logic_error("Could not create convertd options file.");
    }
    fputs(" ", convertdopt);
    fclose(convertdopt);

    /* call GAMS with convertd solver to get compiled model instance in temporary directory
	 * we set lo=3 so that we get lo=3 into the gams control file, which is useful for showing the log of GAMS (NLP) solvers later
	 * but since we don't want to see the stdout output from gams here, we redirect stdout to /dev/null for this gams call
	 */
    snprintf(gamscall,
             sizeof(gamscall),
             GAMSDIR "/gams %s SOLVER=CONVERTD SCRDIR=loadgms.tmp output=loadgms.tmp/listing optdir=loadgms.tmp optfile=1 pf4=0 solprint=0 limcol=0 limrow=0 pc=2 lo=3 > /dev/null",
             filename.c_str());

    /* printf(gamscall); fflush(stdout); */
    rc = system(gamscall);
    if (rc != 0)
    {
        snprintf(buffer, sizeof(buffer), "GAMS call returned with code %d", rc);
        throw std::logic_error(buffer);
    }

    createModelFromGAMSModel("loadgms.tmp/gamscntr.dat");

    /* since we ran convert with options file, GMO now stores convertd.opt as options file, which we don't want to use as a SHOT options file */
    gmoOptFileSet(modelingObject, 0);
}

void ModelingSystemGAMS::createModelFromGAMSModel(const std::string &filename)
{
    char buffer[GMS_SSSIZE];

    /* initialize GMO and GEV libraries */
    if (!gmoCreateDD(&modelingObject, GAMSDIR, buffer, sizeof(buffer)) || !gevCreateDD(&modelingEnvironment, GAMSDIR, buffer, sizeof(buffer)))
        throw std::logic_error(
            buffer);

    /* load control file */
    if (gevInitEnvironmentLegacy(modelingEnvironment, filename.c_str()))
    {
        gmoFree(&modelingObject);
        gevFree(&modelingEnvironment);
        throw std::logic_error("Could not load control file loadgms.tmp/gamscntr.dat.");
    }

    if (gmoRegisterEnvironment(modelingObject, modelingEnvironment, buffer))
    {
        gmoFree(&modelingObject);
        gevFree(&modelingEnvironment);
        snprintf(buffer, sizeof(buffer), "Error registering GAMS Environment: %s", buffer);
        throw std::logic_error(buffer);
    }

    if (gmoLoadDataLegacy(modelingObject, buffer))
    {
        gmoFree(&modelingObject);
        gevFree(&modelingEnvironment);
        throw std::logic_error("Could not load model data.");
    }

    gevTerminateUninstall(modelingEnvironment);
}

void ModelingSystemGAMS::finalizeSolution()
{
}

void ModelingSystemGAMS::clearGAMSObjects()
{
    if (modelingObject == NULL)
        return;

    gmoUnloadSolutionLegacy(modelingObject);

    gmoFree(&modelingObject);
    modelingObject = NULL;

    assert(modelingEnvironment != NULL);
    gevFree(&modelingEnvironment);
    modelingEnvironment = NULL;

    /* remove temporary directory content (should have only files) and directory itself) */
    if (createdtmpdir)
    {
        system("rm loadgms.tmp/* && rmdir loadgms.tmp");
        createdtmpdir = false;
    }
}

bool ModelingSystemGAMS::copyVariables(ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy variables between GAMS modeling and SHOT problem objects.");

    int numVariables = gmoN(modelingObject);

    if (numVariables > 0)
    {
        double minLBCont = env->settings->getDoubleSetting("ContinuousVariable.EmptyLowerBound", "Model");
        double maxUBCont = env->settings->getDoubleSetting("ContinuousVariable.EmptyUpperBound", "Model");
        double minLBInt = env->settings->getDoubleSetting("IntegerVariable.EmptyLowerBound", "Model");
        double maxUBInt = env->settings->getDoubleSetting("IntegerVariable.EmptyUpperBound", "Model");

        double *variableLBs = new double[numVariables];
        double *variableUBs = new double[numVariables];
        gmoGetVarLower(modelingObject, variableLBs);
        gmoGetVarUpper(modelingObject, variableUBs);

        for (int i = 0; i < numVariables; i++)
        {
            if (gmoDict(modelingObject))
                gmoGetVarNameOne(modelingObject, i, buffer);
            else
                sprintf(buffer, "x%08d", i);

            std::string variableName = buffer;

            E_VariableType variableType;

            switch (gmoGetVarTypeOne(modelingObject, i))
            {
            case gmovar_X:
                variableType = E_VariableType::Real;

                if (variableLBs[i] < minLBCont)
                {
                    variableLBs[i] = minLBCont;
                }

                if (variableUBs[i] > maxUBCont)
                {
                    variableUBs[i] = maxUBCont;
                }

                break;

            case gmovar_B:
                variableType = E_VariableType::Binary;

                if (variableLBs[i] < 0.0)
                {
                    variableLBs[i] = 0.0;
                }

                if (variableUBs[i] > 1.0)
                {
                    variableUBs[i] = 1.0;
                }

                break;

            case gmovar_I:
                variableType = E_VariableType::Integer;

                if (variableLBs[i] < minLBInt)
                {
                    variableLBs[i] = minLBInt;
                }

                if (variableUBs[i] > maxUBInt)
                {
                    variableUBs[i] = maxUBInt;
                }

                break;

            case gmovar_SC:
                variableType = E_VariableType::Semicontinuous;

                if (variableLBs[i] < 0.0)
                {
                    variableLBs[i] = 0.0;
                }

                if (variableUBs[i] > maxUBCont)
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

            auto variable = std::make_shared<SHOT::Variable>(variableName, i, variableType, variableLBs[i], variableUBs[i]);
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
    if (gmoModelType(modelingObject) == gmoProc_cns)
    {
        // no objective in constraint satisfaction models
        env->output->outputError("Problem has no objective function.");
        return (false);
    }

    ObjectiveFunctionPtr objectiveFunction;

    switch (gmoGetObjOrder(modelingObject))
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

    if (gmoSense(modelingObject) == gmoObj_Min)
        objectiveFunction->direction = E_ObjectiveFunctionDirection::Minimize;
    else
        objectiveFunction->direction = E_ObjectiveFunctionDirection::Maximize;

    objectiveFunction->constant = gmoObjConst(modelingObject);

    // Now copying the linear terms (if any)
    if (gmoN(modelingObject) > 0)
    {
        int *variableIndexes = new int[gmoObjNZ(modelingObject)];
        double *coefficients = new double[gmoObjNZ(modelingObject)];
        int *nonlinearFlags = new int[gmoObjNZ(modelingObject)];
        int numberOfNonzeros;
        int numberOfNonlinearNonzeros;

        gmoGetObjSparse(modelingObject, variableIndexes, coefficients, nonlinearFlags, &numberOfNonzeros, &numberOfNonlinearNonzeros);

        int numberLinearTerms = numberOfNonzeros - numberOfNonlinearNonzeros;

        for (int i = 0, j = 0; i < numberLinearTerms; ++i)
        {
            try
            {
                VariablePtr variable = destination->getVariable(variableIndexes[i]);
                (std::static_pointer_cast<LinearObjectiveFunction>(objectiveFunction))->add(std::move(std::make_shared<LinearTerm>(coefficients[i], variable)));
            }
            catch (const VariableNotFoundException &e)
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

    if (numberOfConstraints > 0)
    {
        for (int i = 0; i < numberOfConstraints; i++)
        {
            double lb;
            double ub;

            switch (gmoGetEquTypeOne(modelingObject, i))
            {
            case gmoequ_E:
                lb = ub = gmoGetRhsOne(modelingObject, i);
                break;

            case gmoequ_L:
                lb = -std::numeric_limits<double>::infinity();
                ub = gmoGetRhsOne(modelingObject, i);
                break;

            case gmoequ_G:
                lb = gmoGetRhsOne(modelingObject, i);
                ub = std::numeric_limits<double>::infinity();
                break;

            case gmoequ_N:
                lb = -std::numeric_limits<double>::infinity();
                ub = std::numeric_limits<double>::infinity();
                break;

            default:
                env->output->outputDebug("Constraint index" + std::to_string(i) + "is of unknown type.");
                return (false);
            }

            if (gmoDict(modelingObject))
                gmoGetEquNameOne(modelingObject, i, buffer);
            else
                sprintf(buffer, "e%08d", i);

            switch (gmoGetEquOrderOne(modelingObject, i))
            {
            case (gmoorder_L):
            {
                LinearConstraintPtr constraint = std::make_shared<LinearConstraint>(i, buffer, lb, ub);
                destination->add(std::move(constraint));
                break;
            }
            case (gmoorder_Q):
            {
                QuadraticConstraintPtr constraint = std::make_shared<QuadraticConstraint>(i, buffer, lb, ub);
                destination->add(std::move(constraint));
                break;
            }
            case (gmoorder_NL):
            {
                NonlinearConstraintPtr constraint = std::make_shared<NonlinearConstraint>(i, buffer, lb, ub);
                destination->add(std::move(constraint));
                break;
            }
            default:
                env->output->outputDebug("Constraint index" + std::to_string(i) + "is of unknown type.");
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

    double *linearCoefficients = new double[gmoNZ(modelingObject) + gmoN(modelingObject)];
    int *variableIndexes = new int[gmoNZ(modelingObject) + gmoN(modelingObject)];
    int *nonlinearFlags = new int[gmoN(modelingObject)];

    int numConstraints = gmoM(modelingObject);
    int nz = 0;
    for (int row = 0; row < numConstraints; ++row)
    {
        int rownz;
        int nlnz;

        gmoGetRowSparse(modelingObject, row, &variableIndexes[nz], &linearCoefficients[nz], nonlinearFlags, &rownz, &nlnz);

        try
        {
            LinearConstraintPtr constraint = std::static_pointer_cast<LinearConstraint>(destination->getConstraint(row));

            for (int j = 0; j < rownz; j++)
            {
                constraint->add(std::make_shared<LinearTerm>(linearCoefficients[j], destination->getVariable(variableIndexes[j])));
            }
        }
        catch (const VariableNotFoundException &e)
        {
            delete[] linearCoefficients;
            delete[] variableIndexes;
            delete[] nonlinearFlags;
            return (false);
        }
        catch (const ConstraintNotFoundException &e)
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

    if (gmoGetObjOrder(modelingObject) == gmoorder_Q)
    {
        int numQuadraticTerms = gmoObjQNZ(modelingObject);

        int *variableOneIndexes = new int[numQuadraticTerms];
        int *variableTwoIndexes = new int[numQuadraticTerms];
        double *quadraticCoefficients = new double[numQuadraticTerms];

        gmoGetObjQ(modelingObject, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);

        for (int j = 0; j < numQuadraticTerms; ++j)
        {
            if (variableOneIndexes[j] == variableTwoIndexes[j])
                quadraticCoefficients[j] /= 2.0; /* for some strange reason, the coefficients on the diagonal are multiplied by 2 in GMO */

            try
            {
                VariablePtr firstVariable = destination->getVariable(variableOneIndexes[j]);
                VariablePtr secondVariable = destination->getVariable(variableTwoIndexes[j]);

                (std::static_pointer_cast<QuadraticObjectiveFunction>(destination->objectiveFunction))->add(std::make_shared<QuadraticTerm>(quadraticCoefficients[j], firstVariable, secondVariable));
            }
            catch (const VariableNotFoundException &e)
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

    for (int i = 0; i < numberOfConstraints; ++i)
    {
        if (gmoGetEquOrderOne(modelingObject, i) == gmoorder_Q)
        {
            // handle quadratic equation

            int numQuadraticTerms = gmoGetRowQNZOne(modelingObject, i);

            int *variableOneIndexes = new int[numQuadraticTerms];
            int *variableTwoIndexes = new int[numQuadraticTerms];
            double *quadraticCoefficients = new double[numQuadraticTerms];

            gmoGetRowQ(modelingObject, i, variableOneIndexes, variableTwoIndexes, quadraticCoefficients);

            for (int j = 0; j < numQuadraticTerms; ++j)
            {
                if (variableOneIndexes[j] == variableTwoIndexes[j])
                    quadraticCoefficients[j] /= 2.0; /* for some strange reason, the coefficients on the diagonal are multiplied by 2 in GMO */

                try
                {
                    VariablePtr firstVariable = destination->getVariable(variableOneIndexes[j]);
                    VariablePtr secondVariable = destination->getVariable(variableTwoIndexes[j]);

                    auto constraint = std::static_pointer_cast<QuadraticConstraint>(destination->getConstraint(i));
                    constraint->add(std::make_shared<QuadraticTerm>(quadraticCoefficients[j], firstVariable, secondVariable));
                }
                catch (const VariableNotFoundException &e)
                {
                    delete[] variableOneIndexes;
                    delete[] variableTwoIndexes;
                    delete[] quadraticCoefficients;

                    return (false);
                }
                catch (const ConstraintNotFoundException &e)
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

    int *opcodes = new int[gmoNLCodeSizeMaxRow(modelingObject) + 1];
    int *fields = new int[gmoNLCodeSizeMaxRow(modelingObject) + 1];
    int constantlen = gmoNLConst(modelingObject);
    double *constants = (double *)gmoPPool(modelingObject);
    int codelen;

    if (gmoObjNLNZ(modelingObject) > 0 && gmoGetObjOrder(modelingObject) == gmoorder_NL)
    {
        // handle nonlinear objective

        gmoDirtyGetObjFNLInstr(modelingObject, &codelen, opcodes, fields);

        try
        {
            auto destinationExpression = parseGamsInstructions(codelen, opcodes, fields, constantlen, constants, destination);

            if (codelen > 0)
            {
                double objjacval = gmoObjJacVal(modelingObject);
                if (objjacval == 1.0)
                {
                    // scale by -1/objjacval = negate
                    destinationExpression = std::make_shared<ExpressionNegate>(destinationExpression);
                }
                else if (objjacval != -1.0)
                {
                    // scale by -1/objjacval
                    destinationExpression = std::make_shared<ExpressionTimes>(
                        std::make_shared<ExpressionConstant>(-1 / objjacval),
                        destinationExpression);
                }

                auto objective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination->objectiveFunction);
                objective->add(std::move(destinationExpression));
            }
        }
        catch (const ConstraintNotFoundException &e)
        {
            return (false);
        }
        catch (const OperationNotImplementedException &e)
        {
            return (false);
        }
    }

    for (int i = 0; i < gmoM(modelingObject); ++i)
    {
        if (gmoGetEquOrderOne(modelingObject, i) == gmoorder_NL)
        {
            gmoDirtyGetRowFNLInstr(modelingObject, i, &codelen, opcodes, fields);
            if (codelen == 0)
                continue;

            try
            {
                auto destinationExpression = parseGamsInstructions(codelen, opcodes, fields, constantlen, constants, destination);

                auto constraint = std::dynamic_pointer_cast<NonlinearConstraint>(destination->getConstraint(i));
                constraint->add(std::move(destinationExpression));
            }
            catch (const ConstraintNotFoundException &e)
            {
                return (false);
            }
            catch (const OperationNotImplementedException &e)
            {
                return (false);
            }
        }
    }

    env->output->outputDebug("Finished copying nonlinear expressions between GAMS modeling and SHOT problem objects.");

    return (true);
}

NonlinearExpressionPtr ModelingSystemGAMS::parseGamsInstructions(int codelen,       /**< length of GAMS instructions */
                                                                 int *opcodes,      /**< opcodes of GAMS instructions */
                                                                 int *fields,       /**< fields of GAMS instructions */
                                                                 int constantlen,   /**< length of GAMS constants pool */
                                                                 double *constants, /**< GAMS constants pool */
                                                                 const ProblemPtr &destination)
{
    bool debugoutput = gevGetIntOpt(modelingEnvironment, gevInteger1) & 0x4;
#define debugout     \
    if (debugoutput) \
    std::clog

    std::vector<NonlinearExpressionPtr> stack;
    stack.reserve(20);

    for (int i = 0; i < codelen; ++i)
    {
        GamsOpCode opcode = (GamsOpCode)opcodes[i];
        int address = fields[i] - 1;

        debugout << '\t' << GamsOpCodeName[opcode] << ": ";

        switch (opcode)
        {
        case nlNoOp:   // no operation
        case nlStore:  // store row
        case nlHeader: // header
        {
            //debugout << "ignored" << std::endl;
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
            auto expression = std::make_shared<ExpressionPlus>(stack.rbegin()[1], stack.rbegin()[0]);
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlAddV: // add variable
        {
            address = gmoGetjSolver(modelingObject, address);
            auto expression = std::make_shared<ExpressionPlus>(std::make_shared<ExpressionVariable>(destination->getVariable(address)), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlAddI: // add immediate
        {
            auto expression = std::make_shared<ExpressionPlus>(std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlSub: // minus
        {
            auto expression = std::make_shared<ExpressionMinus>(stack.rbegin()[1], stack.rbegin()[0]);
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlSubV: // subtract variable
        {
            address = gmoGetjSolver(modelingObject, address);
            auto expression = std::make_shared<ExpressionMinus>(stack.rbegin()[0], std::make_shared<ExpressionVariable>(destination->getVariable(address)));
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlSubI: // subtract immediate
        {
            auto expression = std::make_shared<ExpressionMinus>(stack.rbegin()[0], std::make_shared<ExpressionConstant>(constants[address]));
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMul: // multiply
        {
            auto expression = std::make_shared<ExpressionTimes>((stack.rbegin()[1]), (stack.rbegin()[0]));
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMulV: // multiply variable
        {
            address = gmoGetjSolver(modelingObject, address);
            auto expression = std::make_shared<ExpressionTimes>(std::make_shared<ExpressionVariable>(destination->getVariable(address)),
                                                                stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMulI: // multiply immediate
        {
            auto expression = std::make_shared<ExpressionTimes>(std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlMulIAdd: // multiply immediate and add
        {
            auto expressionTimes = std::make_shared<ExpressionTimes>(std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expressionTimes);
            auto expressionPlus = std::make_shared<ExpressionPlus>(stack.rbegin()[1], stack.rbegin()[0]);
            stack.pop_back();
            stack.pop_back();
            stack.push_back(expressionPlus);
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
            auto expression = std::make_shared<ExpressionDivide>(std::make_shared<ExpressionVariable>(destination->getVariable(address)), stack.rbegin()[0]);
            stack.pop_back();
            stack.push_back(expression);
            break;
        }

        case nlDivI: // divide immediate
        {
            auto expression = std::make_shared<ExpressionDivide>(std::make_shared<ExpressionConstant>(constants[address]), stack.rbegin()[0]);
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

            switch (GamsFuncCode(address + 1))
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
            case fnslexp:
            case fnsqexp:
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
            case fnsllog10:
            case fnsqlog10:
            {
                auto expression = std::make_shared<ExpressionTimes>(
                    std::make_shared<ExpressionConstant>(1.0 / log(10.0)),
                    std::make_shared<ExpressionLog>(stack.rbegin()[0]));
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnlog2:
            {
                auto expression = std::make_shared<ExpressionTimes>(
                    std::make_shared<ExpressionConstant>(1.0 / log(2.0)),
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
            case fnrpower:  // x ^ y
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
            case fndiv0:
            {
                auto expression = std::make_shared<ExpressionDivide>(stack.rbegin()[1], stack.rbegin()[0]);
                stack.pop_back();
                stack.pop_back();
                stack.push_back(expression);
                break;
            }

            case fnslrec: // 1/x
            case fnsqrec: // 1/x
            {
                stack.push_back(std::make_shared<ExpressionConstant>(1.0));
                auto expression = std::make_shared<ExpressionDivide>(stack.rbegin()[0], std::make_shared<ExpressionConstant>(1.0));
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
                throw new OperationNotImplementedException("Error: Unsupported GAMS function " + std::string(buffer));
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
            throw new OperationNotImplementedException("Error: Unsupported GAMS opcode " + std::string(buffer));
        }
        }
    }

    assert(stack.size() == 1);
    return stack[0];
#undef debugout
}
} // Namespace SHOT