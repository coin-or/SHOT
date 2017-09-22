#include "SHOTSolver.h"

SHOTSolver *solver = NULL;

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
		std::cout << "Usage: filename.osil options.[opt|xml|osol] results.osrl trace.trc" << std::endl;

		return (0);
	}

	solver = new SHOTSolver();

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
			FileUtil *fileUtil;
			fileUtil = new FileUtil();

			// Create OSoL-file
			optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.xml");
			fileUtil->writeFileFromString(optionsFile.string(), solver->getOSoL());

			// Create GAMS option file
			optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.opt");
			fileUtil->writeFileFromString(optionsFile.string(), solver->getGAMSOptFile());
		}

		// Define names for result files
		resultFile = boost::filesystem::path(boost::filesystem::current_path() / "results.osrl");
		traceFile = boost::filesystem::path(boost::filesystem::current_path() / "trace.trc");
	}
	else if (argc == 3)
	{
		if (!boost::filesystem::exists(argv[2]))
		{
			std::cout << startmessage << std::endl;
			std::cout << "Options file " << argv[2] << " not found!" << std::endl;

			delete solver;

			return (0);
		}

		optionsFile = boost::filesystem::path(argv[2]);
		resultFile = boost::filesystem::path(boost::filesystem::current_path() / "results.osrl");
		traceFile = boost::filesystem::path(boost::filesystem::current_path() / "trace.trc");
	}
	else if (argc == 4)
	{
		if (!boost::filesystem::exists(argv[2]))
		{
			std::cout << startmessage << std::endl;
			std::cout << "Options file " << argv[2] << " not found!" << std::endl;

			delete solver;

			return (0);
		}

		optionsFile = boost::filesystem::path(argv[2]);
		resultFile = boost::filesystem::path(argv[3]);
		traceFile = boost::filesystem::path(boost::filesystem::current_path() / "trace.trc");
	}
	else
	{
		if (!boost::filesystem::exists(argv[2]))
		{
			std::cout << startmessage << std::endl;
			std::cout << "Options file " << argv[2] << " not found!" << std::endl;

			delete solver;

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
			std::cout << "Problem file " << argv[1] << " not found!" << std::endl;

			delete solver;

			return (0);
		}

		std::string osilFileName = argv[1];

		if (!optionsFile.empty() && !solver->setOptions(optionsFile.string()))
		{
			delete solver;

			std::cout << startmessage << std::endl;
			std::cout << "Cannot set options!" << std::endl;
			return (0);
		}

		// Prints out the welcome message to the logging facility
		ProcessInfo::getInstance().outputSummary(startmessage);

		std::cout << osilFileName << std::endl;
		if (!solver->setProblem(osilFileName))
		{
			ProcessInfo::getInstance().outputError("Error when reading problem file.");

			delete solver;

			return (0);
		}

		if (!solver->solveProblem()) // solve problem
		{
			ProcessInfo::getInstance().outputError("Error when solving problem.");

			delete solver;

			return (0);
		}
	}
	catch (const ErrorClass& eclass)
	{
		ProcessInfo::getInstance().outputError(eclass.errormsg);

		delete solver;

		return (0);
	}

	ProcessInfo::getInstance().stopTimer("Total");

	FileUtil *fileUtil;
	fileUtil = new FileUtil();

	if (!resultFile.empty())
	{
		std::string osrl = solver->getOSrL();

		fileUtil->writeFileFromString(resultFile.string(), osrl);
	}

	if (!traceFile.empty())
	{
		std::string trace = solver->getTraceResult();
		fileUtil->writeFileFromString(traceFile.string(), trace);
	}

	delete fileUtil;

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
	delete solver;
	return (0);
}
