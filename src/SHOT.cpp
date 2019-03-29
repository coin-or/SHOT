/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Shared.h"
#include "Solver.h"

using namespace SHOT;

int main(int argc, char* argv[])
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    if(argc == 1)
    {
        env->report->outputSolverHeader();
        std::cout << " Usage: filename.[osil|xml|gms|nl] options.[opt|xml|osol] results.osrl results.trc" << std::endl;

        return (0);
    }

    bool defaultOptionsGenerated = false;

    solver->getEnvironment()->timing->startTimer("Total");

    boost::filesystem::path resultFile, optionsFile, traceFile;

    if(argc == 2) // No options file specified, use or create defaults
    {
        bool GAMSOptFileExists = boost::filesystem::exists(boost::filesystem::current_path() / "options.opt");
        bool OSoLFileExists = boost::filesystem::exists(boost::filesystem::current_path() / "options.xml");

        if(GAMSOptFileExists)
        {
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.opt");
        }
        else if(OSoLFileExists)
        {
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.xml");
        }
        else
        {
            // Create OSoL-file
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.xml");

            if(!UtilityFunctions::writeStringToFile(optionsFile.string(), solver->getOptionsOSoL()))
            {
                env->output->outputError(" Error when writing OSoL file: " + optionsFile.string());
            }

            // Create GAMS option file
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.opt");

            if(!UtilityFunctions::writeStringToFile(optionsFile.string(), solver->getOptions()))
            {
                env->output->outputError(" Error when writing options file: " + optionsFile.string());
            }

            defaultOptionsGenerated = true;
        }
    }
    else if(argc == 3)
    {
        if(!boost::filesystem::exists(argv[2]))
        {
            env->report->outputSolverHeader();

            return (0);
        }

        optionsFile = boost::filesystem::path(argv[2]);
    }
    else if(argc == 4)
    {
        if(!boost::filesystem::exists(argv[2]))
        {
            env->report->outputSolverHeader();
            std::cout << " Options file " << argv[2] << " not found!" << std::endl;

            return (0);
        }

        optionsFile = boost::filesystem::path(argv[2]);
        resultFile = boost::filesystem::path(argv[3]);
    }
    else
    {
        if(!boost::filesystem::exists(argv[2]))
        {
            env->report->outputSolverHeader();
            std::cout << " Options file " << argv[2] << " not found!" << std::endl;

            return (0);
        }

        optionsFile = boost::filesystem::path(argv[2]);
        resultFile = boost::filesystem::path(argv[3]);
        traceFile = boost::filesystem::path(argv[4]);
    }

    try
    {
        if(!boost::filesystem::exists(argv[1]))
        {
            env->report->outputSolverHeader();
            std::cout << " Problem file " << argv[1] << " not found!" << std::endl;

            return (0);
        }

        std::string fileName = argv[1];

        if(!defaultOptionsGenerated)
        {
            if(!optionsFile.empty() && !solver->setOptions(optionsFile.string()))
            {
                env->report->outputSolverHeader();
                std::cout << " Cannot set options!" << std::endl;
                return (0);
            }
        }

        env->output->setLogLevels(static_cast<E_LogLevel>(env->settings->getSetting<int>("Console.LogLevel", "Output")),
            static_cast<E_LogLevel>(env->settings->getSetting<int>("File.LogLevel", "Output")));
        env->report->outputSolverHeader();

        if(!solver->setProblem(fileName))
        {
            env->output->outputError(" Error when reading problem file.");

            return (0);
        }

        env->report->outputOptionsReport();
        env->report->outputProblemInstanceReport();

        if(!solver->solveProblem()) // solve problem
        {
            env->output->outputError(" Error when solving problem.");

            return (0);
        }

        env->report->outputSolutionReport();

        env->output->outputInfo("╶──────────────────────────────────────────────────────────────────────────────────"
                                "───────────────────────────────────╴\r\n");
    }
    catch(const Error& eclass)
    {
        env->output->outputError(eclass.message);

        return (0);
    }

    std::string osrl = solver->getResultsOSrL();

    if(resultFile.empty())
    {
        boost::filesystem::path resultPath(env->settings->getSetting<std::string>("ResultPath", "Output"));
        resultPath /= env->problem->name;
        resultPath = resultPath.replace_extension(".osrl");
        env->output->outputInfo(" Results written to: " + resultPath.string());

        if(!UtilityFunctions::writeStringToFile(resultPath.string(), osrl))
        {
            env->output->outputError(" Error when writing OSrL file: " + resultPath.string());
        }
    }
    else
    {
        env->output->outputInfo(" Results written to: " + resultFile.string());

        if(!UtilityFunctions::writeStringToFile(resultFile.string(), osrl))
        {
            env->output->outputError(" Error when writing OSrL file: " + resultFile.string());
        }
    }

    std::string trace = solver->getResultsTrace();

    if(traceFile.empty())
    {
        boost::filesystem::path tracePath(env->settings->getSetting<std::string>("ResultPath", "Output"));
        tracePath /= env->problem->name;
        tracePath = tracePath.replace_extension(".trc");
        env->output->outputInfo("                     " + tracePath.string());

        if(!UtilityFunctions::writeStringToFile(tracePath.string(), trace))
        {
            env->output->outputError(" Error when writing trace file: " + tracePath.string());
        }
    }
    else
    {
        if(!UtilityFunctions::writeStringToFile(traceFile.string(), trace))
        {
            env->output->outputError(" Error when writing trace file: " + traceFile.string());
        }
    }

    return (0);
}
