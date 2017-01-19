#include "SHOTSolver.h"

SHOTSolver *solver = NULL;
FileUtil *fileUtil = NULL;

int main(int argc, char *argv[])
{
	ProcessInfo *processInfo;
	processInfo = ProcessInfo::getInstance();
	processInfo->startTimer("Total");

	processInfo->logger.message(1)
			<< "==================================================================================" << CoinMessageEol;
	processInfo->logger.message(1)
			<< "=                SHOT - Supporting Hyperplane Optimization Toolkit               =" << CoinMessageEol;
	processInfo->logger.message(1)
			<< "=================================================================================="
			<< CoinMessageNewline << CoinMessageEol;

	if (argc == 1)
	{
		std::cout << "Usage: filename.osil options.osol results.osrl trace.trc" << std::endl;

		delete processInfo;

		return (0);
	}

	boost::filesystem::path resultFile, optionsFile, traceFile;

	fileUtil = new FileUtil();
	solver = new SHOTSolver();

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
			std::cout << "Options file not found!" << std::endl;

			delete fileUtil, solver, processInfo;

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
			std::cout << "Options file not found!" << std::endl;

			delete fileUtil, solver, processInfo;

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
			std::cout << "Options file not found!" << std::endl;

			delete fileUtil, solver, processInfo;

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
			std::cout << "Problem file not found!" << std::endl;

			delete fileUtil, solver, processInfo;

			return (0);
		}

		std::string osilFileName = argv[1];

		if (!solver->setOptions(optionsFile.string()))
		{
			delete fileUtil, solver, processInfo;

			return (0);
		}

		if (!solver->setProblem(osilFileName))
		{
			processInfo->logger.message(0) << "Error when reading problem file" << CoinMessageEol;

			delete fileUtil, solver, processInfo;

			return (0);
		}

		if (!solver->solveProblem()) // solve problem
		{
			processInfo->logger.message(0) << "Error when solving problem." << CoinMessageEol;

			delete fileUtil, solver, processInfo;

			return (0);
		}
	}
	catch (const ErrorClass& eclass)
	{
		processInfo->logger.message(0) << eclass.errormsg << CoinMessageEol;

		std::cout << eclass.errormsg << CoinMessageEol;
		delete fileUtil, solver, processInfo;

		return (0);
	}

	processInfo->stopTimer("Total");

	std::string osrl = solver->getOSrl();

	fileUtil->writeFileFromString(resultFile.string(), osrl);

	std::string trace = solver->getTraceResult();
	fileUtil->writeFileFromString(traceFile.string(), trace);

	processInfo->logger.message(2) << CoinMessageNewline;

	for (auto T : processInfo->timers)
	{
		auto elapsed = T.elapsed();

		if (elapsed > 0)
		{
			auto tmpLine = boost::format("%1%: %|54t|%2%") % T.description % elapsed;
			processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;
		}

	}

	delete fileUtil, solver, processInfo;
	return (0);
}
