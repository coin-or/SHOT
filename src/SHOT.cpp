/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSolver.h"

int main(int argc, char *argv[])
{
    if (argc == 1)
    {
        Output::getInstance().outputSolverHeader();
        std::cout << " Usage: filename.[osil|xml|gms] options.[opt|xml|osol] results.osrl results.trc" << std::endl;

        return (0);
    }

    unique_ptr<SHOTSolver> solver(new SHOTSolver());
    bool defaultOptionsGenerated = false;

    ProcessInfo::getInstance().startTimer("Total");

    boost::filesystem::path resultFile, optionsFile, traceFile;

    if (strlen(argv[1]) > 4 && strcmp(argv[1] + (strlen(argv[1]) - 4), ".dat") == 0)
    {
        // special handling when run on gams control file (.dat): don't read options file, don't write results or trace file
        // TODO it would probably be better to have a specialized SHOT executable for running under GAMS than hijacking this main()
    }
    else if (argc == 2) // No options file specified, use or create defaults
    {
        bool GAMSOptFileExists = boost::filesystem::exists(boost::filesystem::current_path() / "options.opt");
        bool OSoLFileExists = boost::filesystem::exists(boost::filesystem::current_path() / "options.xml");

        if (GAMSOptFileExists)
        {
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.opt");
        }
        else if (OSoLFileExists)
        {
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.xml");
        }
        else
        {
            // Create OSoL-file
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.xml");

            if (!UtilityFunctions::writeStringToFile(optionsFile.string(), solver->getOSoL()))
            {
                Output::getInstance().outputError(" Error when writing OSoL file: " + optionsFile.string());
            }

            // Create GAMS option file
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.opt");

            if (!UtilityFunctions::writeStringToFile(optionsFile.string(), solver->getGAMSOptFile()))
            {
                Output::getInstance().outputError(" Error when writing options file: " + optionsFile.string());
            }

            defaultOptionsGenerated = true;
        }
    }
    else if (argc == 3)
    {
        if (!boost::filesystem::exists(argv[2]))
        {
            Output::getInstance().outputSolverHeader();

            return (0);
        }

        optionsFile = boost::filesystem::path(argv[2]);
    }
    else if (argc == 4)
    {
        if (!boost::filesystem::exists(argv[2]))
        {
            Output::getInstance().outputSolverHeader();
            std::cout << " Options file " << argv[2] << " not found!" << std::endl;

            return (0);
        }

        optionsFile = boost::filesystem::path(argv[2]);
        resultFile = boost::filesystem::path(argv[3]);
    }
    else
    {
        if (!boost::filesystem::exists(argv[2]))
        {
            Output::getInstance().outputSolverHeader();
            std::cout << " Options file " << argv[2] << " not found!" << std::endl;

            return (0);
        }

        optionsFile = boost::filesystem::path(argv[2]);
        resultFile = boost::filesystem::path(argv[3]);
        traceFile = boost::filesystem::path(argv[4]);
    }

    try
    {
        if (!boost::filesystem::exists(argv[1]))
        {
            Output::getInstance().outputSolverHeader();
            std::cout << " Problem file " << argv[1] << " not found!" << std::endl;

            return (0);
        }

        std::string fileName = argv[1];

        if (!defaultOptionsGenerated)
        {
            if (!optionsFile.empty() && !solver->setOptions(optionsFile.string()))
            {
                Output::getInstance().outputSolverHeader();
                std::cout << " Cannot set options!" << std::endl;
                return (0);
            }
        }

        Output::getInstance().setLogLevels();
        Output::getInstance().outputSolverHeader();

        // Prints out the welcome message to the logging facility

        if (!solver->setProblem(fileName))
        {
            Output::getInstance().outputError(" Error when reading problem file.");

            return (0);
        }

        Output::getInstance().outputOptionsReport();
        Output::getInstance().outputProblemInstanceReport();

        if (!solver->solveProblem()) // solve problem
        {
            Output::getInstance().outputError(" Error when solving problem.");

            return (0);
        }

        Output::getInstance().outputSolutionReport();

        Output::getInstance().outputSummary("╶─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────╴\r\n");
    }
    catch (const ErrorClass &eclass)
    {
        Output::getInstance().outputError(eclass.errormsg);

        return (0);
    }

    std::string osrl = solver->getOSrL();

    if (resultFile.empty())
    {
        boost::filesystem::path resultPath(Settings::getInstance().getStringSetting("ResultPath", "Output"));
        resultPath /= ProcessInfo::getInstance().originalProblem->getProblemInstance()->getInstanceName();
        resultPath = resultPath.replace_extension(".osrl");
        Output::getInstance().outputSummary(" Results written to: " + resultPath.string());

        if (!UtilityFunctions::writeStringToFile(resultPath.string(), osrl))
        {
            Output::getInstance().outputError(" Error when writing OSrL file: " + resultPath.string());
        }
    }
    else
    {
        Output::getInstance().outputSummary(" Results written to: " + resultFile.string());

        if (!UtilityFunctions::writeStringToFile(resultFile.string(), osrl))
        {
            Output::getInstance().outputError(" Error when writing OSrL file: " + resultFile.string());
        }
    }

    std::string trace = solver->getTraceResult();

    if (traceFile.empty())
    {
        boost::filesystem::path tracePath(Settings::getInstance().getStringSetting("ResultPath", "Output"));
        tracePath /= ProcessInfo::getInstance().originalProblem->getProblemInstance()->getInstanceName();
        tracePath = tracePath.replace_extension(".trc");
        Output::getInstance().outputSummary("                     " + tracePath.string());

        if (!UtilityFunctions::writeStringToFile(tracePath.string(), trace))
        {
            Output::getInstance().outputError(" Error when writing trace file: " + tracePath.string());
        }
    }
    else
    {
        if (!UtilityFunctions::writeStringToFile(traceFile.string(), trace))
        {
            Output::getInstance().outputError(" Error when writing trace file: " + traceFile.string());
        }
    }

    return (0);
}
