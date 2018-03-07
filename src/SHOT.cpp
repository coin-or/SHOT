/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSolver.h"

std::string startmessage;

int main(int argc, char *argv[])
{
// Visual Studio does not play nice with unicode:
#ifdef _WIN32
    startmessage = ""
                   "ÚÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¿\n"
                   "³          SHOT - Supporting Hyperplane Optimization Toolkit          ³\n"
                   "ÃÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ´\n"
                   "³ - Implementation by Andreas Lundell (andreas.lundell@abo.fi)        ³\n"
                   "³ - Based on the Extended Supporting Hyperplane (ESH) algorithm       ³\n"
                   "³   by Jan Kronqvist, Andreas Lundell and Tapio Westerlund            ³\n"
                   "³   bo Akademi University, Turku, Finland                            ³\n"
                   "ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÙ\n";
#else
    startmessage = ""
                   "┌─────────────────────────────────────────────────────────────────────┐\n"
                   "│          SHOT - Supporting Hyperplane Optimization Toolkit          │\n"
                   "├─────────────────────────────────────────────────────────────────────┤\n"
                   "│ - Implementation by Andreas Lundell (andreas.lundell@abo.fi)        │\n"
                   "│ - Based on the Extended Supporting Hyperplane (ESH) algorithm       │\n"
                   "│   by Jan Kronqvist, Andreas Lundell and Tapio Westerlund            │\n"
                   "│   Åbo Akademi University, Turku, Finland                            │\n"
                   "└─────────────────────────────────────────────────────────────────────┘\n";

#endif

    if (argc == 1)
    {
        std::cout << startmessage << std::endl;
        std::cout << "Usage: filename.[osil|xml|gms] options.[opt|xml|osol] results.osrl results.trc" << std::endl;

        return (0);
    }

    unique_ptr<SHOTSolver> solver(new SHOTSolver());
    bool defaultOptionsGenerated = false;

    ProcessInfo::getInstance().startTimer("Total");

    // Adds a file output
    osoutput->AddChannel("shotlogfile");

    boost::filesystem::path resultFile, optionsFile, traceFile;

    if (strlen(argv[1]) > 4 && strcmp(argv[1] + (strlen(argv[1]) - 4), ".dat") == 0)
    {
        // special handling when run on gams control file (.dat): don't read options file, don't write results or trace file
        // TODO it would probably be better to have a specialized SHOT executable for running under GAMS than hijacking this main()

        osoutput->SetPrintLevel("stdout", ENUM_OUTPUT_LEVEL_summary);
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
                ProcessInfo::getInstance().outputError("Error when writing OsoL file: " + optionsFile.string());
            }

            // Create GAMS option file
            optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.opt");

            if (!UtilityFunctions::writeStringToFile(optionsFile.string(), solver->getGAMSOptFile()))
            {
                ProcessInfo::getInstance().outputError("Error when writing options file: " + optionsFile.string());
            }

            defaultOptionsGenerated = true;
        }
    }
    else if (argc == 3)
    {
        if (!boost::filesystem::exists(argv[2]))
        {
            std::cout << startmessage << std::endl;

            return (0);
        }

        optionsFile = boost::filesystem::path(argv[2]);
    }
    else if (argc == 4)
    {
        if (!boost::filesystem::exists(argv[2]))
        {
            std::cout << startmessage << std::endl;
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
            std::cout << startmessage << std::endl;
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
            std::cout << startmessage << std::endl;
            std::cout << " Problem file " << argv[1] << " not found!" << std::endl;

            return (0);
        }

        std::string fileName = argv[1];

        if (defaultOptionsGenerated)
        {
            osoutput->SetPrintLevel("stdout",
                                    (ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("Console.LogLevel", "Output") + 1));
            osoutput->SetPrintLevel("shotlogfile",
                                    (ENUM_OUTPUT_LEVEL)(Settings::getInstance().getIntSetting("File.LogLevel", "Output") + 1));
        }
        else
        {
            if (!optionsFile.empty() && !solver->setOptions(optionsFile.string()))
            {
                std::cout << startmessage << std::endl;
                std::cout << " Cannot set options!" << std::endl;
                return (0);
            }
        }

        // Prints out the welcome message to the logging facility
        ProcessInfo::getInstance().outputSummary(startmessage);

        if (!solver->setProblem(fileName))
        {
            ProcessInfo::getInstance().outputError(" Error when reading problem file.");

            return (0);
        }

        if (!solver->solveProblem()) // solve problem
        {
            ProcessInfo::getInstance().outputError(" Error when solving problem.");

            return (0);
        }
    }
    catch (const ErrorClass &eclass)
    {
        ProcessInfo::getInstance().outputError(eclass.errormsg);

        return (0);
    }

    ProcessInfo::getInstance().stopTimer("Total");

    std::string osrl = solver->getOSrL();

    if (resultFile.empty())
    {
        boost::filesystem::path resultPath(Settings::getInstance().getStringSetting("ResultPath", "Output"));
        resultPath /= ProcessInfo::getInstance().originalProblem->getProblemInstance()->getInstanceName();
        resultPath = resultPath.replace_extension(".osrl");
        ProcessInfo::getInstance().outputSummary("\n Results written to: " + resultPath.string());

        if (!UtilityFunctions::writeStringToFile(resultPath.string(), osrl))
        {
            ProcessInfo::getInstance().outputError("Error when writing OSrL file: " + resultPath.string());
        }
    }
    else
    {
        ProcessInfo::getInstance().outputSummary("\n Results written to: " + resultFile.string());

        if (!UtilityFunctions::writeStringToFile(resultFile.string(), osrl))
        {
            ProcessInfo::getInstance().outputError("Error when writing OSrL file: " + resultFile.string());
        }
    }

    std::string trace = solver->getTraceResult();

    if (traceFile.empty())
    {
        boost::filesystem::path tracePath(Settings::getInstance().getStringSetting("ResultPath", "Output"));
        tracePath /= ProcessInfo::getInstance().originalProblem->getProblemInstance()->getInstanceName();
        tracePath = tracePath.replace_extension(".trc");
        ProcessInfo::getInstance().outputSummary("                     " + tracePath.string());

        if (!UtilityFunctions::writeStringToFile(tracePath.string(), trace))
        {
            ProcessInfo::getInstance().outputError("Error when writing trace file: " + tracePath.string());
        }
    }
    else
    {
        if (!UtilityFunctions::writeStringToFile(traceFile.string(), trace))
        {
            ProcessInfo::getInstance().outputError("Error when writing trace file: " + traceFile.string());
        }
    }

#ifdef _WIN32
    ProcessInfo::getInstance().outputSummary("\n"
                                             "ÚÄÄÄ Solution time ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¿");

    for (auto T : ProcessInfo::getInstance().timers)
    {
        auto elapsed = T.elapsed();

        if (elapsed > 0)
        {
            auto tmpLine = boost::format("%1%: %|54t|%2%") % T.description % elapsed;

            ProcessInfo::getInstance().outputSummary("³ " + tmpLine.str());
        }
    }

    ProcessInfo::getInstance().outputSummary("ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÙ");
#else
    ProcessInfo::getInstance().outputSummary("\n"
                                             "┌─── Solution time ──────────────────────────────────────────────────────────────┐");

    for (auto T : ProcessInfo::getInstance().timers)
    {
        auto elapsed = T.elapsed();

        if (elapsed > 0)
        {
            auto tmpLine = boost::format("%1%: %|54t|%2%") % T.description % elapsed;

            ProcessInfo::getInstance().outputSummary("│ " + tmpLine.str());
        }
    }

    ProcessInfo::getInstance().outputSummary(
        "└────────────────────────────────────────────────────────────────────────────────┘");
#endif

    return (0);
}
