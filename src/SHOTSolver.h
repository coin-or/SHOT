#pragma once

#include "Enums.h"
#include "SHOTSettings.h"
#include "SolutionStrategy/ISolutionStrategy.h"
#include "SolutionStrategy/SolutionStrategySHOT.h"
#include "ProcessInfo.h"
#include "OSnl2OS.h"
#include "boost/filesystem.hpp"
#include "TaskHandler.h"
#include "GAMS2OS.h"

class SHOTSolver
{
	private:
		ISolutionStrategy *solutionStrategy;

		GAMS2OS* gms2os;

		void initializeSettings();
		void initializeDebugMode();

		bool isProblemInitialized;

	public:

		SHOTSolver();
		~SHOTSolver();

		bool setOptions(std::string fileName);
		bool setOptions(OSOption *osOptions);

		bool setProblem(std::string fileName);
		bool setProblem(OSInstance *osInstance);

		bool solveProblem();

		std::string getOSoL();
		std::string getGAMSOptFile();

		std::string getOSrL();
		std::string getTraceResult();
};
