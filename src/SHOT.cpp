#include "SHOTSolver.h"

SHOTSolver *solver = NULL;
FileUtil *fileUtil = NULL;

std::string startmessage;

int main(int argc, char *argv[])
{
	solver = new SHOTSolver();

	ProcessInfo::getInstance().startTimer("Total");

	// Adds a file output
	osoutput->AddChannel("shotlogfile");

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
		std::cout << "Usage: filename.osil options.osol results.osrl trace.trc" << std::endl;

		return (0);
	}

	boost::filesystem::path resultFile, optionsFile, traceFile;

	fileUtil = new FileUtil();

	if (argc == 2)
	{
		optionsFile = boost::filesystem::path(boost::filesystem::current_path() / "options.xml");

		if (!boost::filesystem::exists(optionsFile))
		{
			fileUtil->writeFileFromString(optionsFile.string(), solver->getOSol());
		}

		resultFile = boost::filesystem::path(boost::filesystem::current_path() / "results.osrl");
		traceFile = boost::filesystem::path(boost::filesystem::current_path() / "trace.trc");
	}
	else if (argc == 3)
	{
		if (!boost::filesystem::exists(argv[2]))
		{
			std::cout << startmessage << std::endl;
			std::cout << "Options file not found!" << std::endl;

			delete fileUtil;
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
			std::cout << "Options file not found!" << std::endl;

			delete fileUtil;
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
			std::cout << "Options file not found!" << std::endl;

			delete fileUtil;
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
			std::cout << "Problem file not found!" << std::endl;

			delete fileUtil;
			delete solver;

			return (0);
		}

		std::string osilFileName = argv[1];

		if (!solver->setOptions(optionsFile.string()))
		{
			delete fileUtil;
			delete solver;

			std::cout << startmessage << std::endl;
			std::cout << "Cannot set options!" << std::endl;
			return (0);
		}

		// Prints out the welcome message to the logging facility
		ProcessInfo::getInstance().outputSummary(startmessage);

		if (!solver->setProblem(osilFileName))
		{
			ProcessInfo::getInstance().outputError("Error when reading problem file.");

			delete fileUtil;
			delete solver;

			return (0);
		}

		if (!solver->solveProblem()) // solve problem
		{
			ProcessInfo::getInstance().outputError("Error when solving problem.");

			delete fileUtil;
			delete solver;

			return (0);
		}
	}
	catch (const ErrorClass& eclass)
	{
		ProcessInfo::getInstance().outputError(eclass.errormsg);
		delete fileUtil;
		delete solver;

		return (0);
	}

	ProcessInfo::getInstance().stopTimer("Total");

	std::string osrl = solver->getOSrl();

	fileUtil->writeFileFromString(resultFile.string(), osrl);

	std::string trace = solver->getTraceResult();
	fileUtil->writeFileFromString(traceFile.string(), trace);

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

	delete fileUtil;
	//delete processInfo;
	delete solver;
	return (0);
}
