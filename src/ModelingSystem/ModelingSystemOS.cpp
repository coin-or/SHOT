/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "ModelingSystemOS.h"

namespace SHOT
{

ModelingSystemOS::ModelingSystemOS(EnvironmentPtr envPtr) : env(envPtr)
{
    osilWriter = new OSiLWriter();
}

virtual ~ModelingSystemOS::ModelingSystemOS()
{
    osilReaders.clear();
}

virtual void ModelingSystemOS::augmentSettings(SettingsPtr settings)
{
}

virtual void ModelingSystemOS::updateSettings(SettingsPtr settings)
{
}

virtual E_ProblemCreationStatus ModelingSystemOS::createProblem(ProblemPtr &problem, const std::string &filename, E_OSInputFileFormat type)
{
    if (!boost::filesystem::exists(filename))
    {
        env->output->outputError("Problem file \"" + filename + "\" does not exist.");

        return (E_ProblemCreationStatus::FileDoesNotExist);
    }

    boost::filesystem::path problemFile(filename);
    //boost::filesystem::path problemExtension = problemFile.extension();
    boost::filesystem::path problemPath = problemFile.parent_path();

    OSInstancePtr instance;

    try
    {
        if (type == E_OSInputFileFormat::OSiL)
        {
            instance = readInstanceFromOSiLFile(filename);

            //TODO: needed??
            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::OSiL));

            /*
                TODO: this should be moved elsewhere
                if (static_cast<ES_PrimalNLPSolver>(env->settings->getIntSetting("FixedInteger.Solver", "Primal")) == ES_PrimalNLPSolver::GAMS)
                {
                    env->output->outputError("Cannot use GAMS NLP solvers in combination with OSiL-files. Switching to Ipopt");
                    env->settings->updateSetting("FixedInteger.Solver", "Primal", (int)ES_PrimalNLPSolver::Ipopt);
                }*/
        }
        else if (type == E_OSInputFileFormat::Ampl)
        {
            instance = readInstanceFromFile(filename);

            //TODO: needed??
            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::NL));
        }
    }
    catch (const ErrorClass &eclass)
    {
        env->output->outputError("Error when reading problem from \"" + filename + "\"", eclass.errormsg);

        return (E_ProblemCreationStatus::Error);
    }

    E_ProblemCreationStatus status = createProblem(problem, instance);

    return (E_ProblemCreationStatus::NormalCompletion);
}

virtual E_ProblemCreationStatus ModelingSystemOS::createProblem(ProblemPtr &problem, const OSInstancePtr &instance)
{
    try
    {
        if (!copyVariables(instance, problem))
            return (E_ProblemCreationStatus::ErrorInVariables);

        if (!copyObjectiveFunction(instance, problem))
            return (E_ProblemCreationStatus::ErrorInObjective);

        if (!copyConstraints(instance, problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        problem->finalize();
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when creating problem from OSInstance object.");

        return (E_ProblemCreationStatus::Error);
    }

    return (E_ProblemCreationStatus::NormalCompletion);
}

virtual void ModelingSystemOS::finalizeSolution()
{
}

virtual OSInstancePtr readInstanceFromOSiL(const std::string &text)
{
    osilReader = std::unique_ptr<OSiLReader>(new OSiLReader());
    OSInstancePtr instance = std::make_shared<OSInstance>(osilReader->readOSiL(text));

    osilReaders.push_back(osilReader); // To be able to properly deleting them without destroying the OSInstance object

    return instance;
}

virtual OSInstancePtr readInstanceFromOSiLFile(const std::string &filename)
{
    std::string fileContents = UtilityFunctions::getFileAsString(fileName);

    return readInstanceFromOSiL(fileContents);
}

virtual OSInstancePtr readInstanceFromAmplFile(const std::string &filename)
{
    nl2os = std::unique_ptr<OSnl2OS>(new OSnl2OS());
    nl2os->readNl(filename);
    nl2os->createOSObjects();
    OSInstancePtr instance = std::make_shared<OSInstance>(nl2os->osinstance);
}

virtual bool ModelingSystemOS::copyVariables(OSInstancePtr source, ProblemPtr destination)
{
    env->output->outputError("Starting to copy variables between OSInstance and SHOT problem classes.");

    if (source->instanceData->variables != NULL && source->instanceData->variables->numberOfVariables > 0)
    {
        double minLBCont = env->settings->getDoubleSetting("ContinuousVariable.EmptyLowerBound", "Model");
        double maxUBCont = env->settings->getDoubleSetting("ContinuousVariable.EmptyUpperBound", "Model");
        double minLBInt = env->settings->getDoubleSetting("IntegerVariable.EmptyLowerBound", "Model");
        double maxUBInt = env->settings->getDoubleSetting("IntegerVariable.EmptyUpperBound", "Model");

        int numVariables = source->getVariableNumber();

        std::string *variableNames = source->getVariableNames();
        char *variableTypes = source->getVariableTypes();
        double *variableLBs = source->getVariableLowerBounds();
        double *variableUBs = source->getVariableUpperBounds();

        for (int i = 0; i < numVariables; i++)
        {
            E_VariableType variableType;

            switch (variableTypes[i])
            {
            case 'C':
                variableType = E_VariableType::Real;

                if (variableLBs[i] < minLBCont)
                {
                    env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " + std::to_string(variableLBs[i]) + " to " + std::to_string(minLBCont));
                    variableLBs[i] = minLBInt;
                }

                if (variableUBs[i] > maxUBCont)
                {
                    env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " + std::to_string(variableUBs[i]) + " to " + std::to_string(maxUBCont));
                    variableUBs[i] = maxUBCont;
                }

                break;

            case 'B':
                variableType = E_VariableType::Binary;

                if (variableLBs[i] < 0.0)
                {
                    env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " + std::to_string(variableLBs[i]) + " to " + std::to_string(0.0));
                    variableLBs[i] = 0.0;
                }

                if (variableUBs[i] > 1.0)
                {
                    env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " + std::to_string(variableUBs[i]) + " to " + std::to_string(1.0));
                    variableUBs[i] = 1.0;
                }

                break;

            case 'I':
                variableType = E_VariableType::Integer;

                if (variableLBs[i] < minLBInt)
                {
                    env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " + std::to_string(variableLBs[i]) + " to " + std::to_string(minLBInt));
                    variableLBs[i] = minLBInt;
                }

                if (variableUBs[i] > maxUBInt)
                {
                    env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " + std::to_string(variableUBs[i]) + " to " + std::to_string(maxUBInt));
                    variableUBs[i] = maxUBInt;
                }

                break;

            case 'D':
                variableType = E_VariableType::Semicontinuous;

                if (variableLBs[i] < 0.0)
                {
                    env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " + std::to_string(variableLBs[i]) + " to " + std::to_string(0.0));
                    variableLBs[i] = 0.0;
                }

                if (variableUBs[i] > maxUBCont)
                {
                    env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " + std::to_string(variableUBs[i]) + " to " + std::to_string(maxUBCont));
                    variableUBs[i] = maxUBCont;
                }

                break;

            default:
                return (false);
                break;
            }

            auto variable = std::make_shared<SHOT::Variable>(variableNames[i], i, variableType, variableLBs[i], variableUBs[i]);
            destination->add(variable);
        }
    }

    env->output->outputError("Finished copying variables between OSInstance and SHOT problem classes.");

    return (true);
}

virtual bool ModelingSystemOS::copyObjectiveFunction(OSInstancePtr source, ProblemPtr destination)
{
    return (true);
}

virtual bool ModelingSystemOS::copyConstraints(OSInstancePtr source, ProblemPtr destination)
{
    return (true);
}
} // namespace SHOT