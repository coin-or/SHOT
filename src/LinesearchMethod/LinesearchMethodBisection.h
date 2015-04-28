#pragma once
#include "ILinesearchMethod.h"
#include "SHOTSettings.h"

//#include "..\OptProblems\OptProblemReformulated.h"
class LinesearchMethodBisection :
	public ILinesearchMethod
{
public:
	LinesearchMethodBisection();
	virtual ~LinesearchMethodBisection();

	virtual std::vector<double> findZero(std::vector<double> ptA, std::vector<double> ptB, int Nmax, double delta);

private:
	ProcessInfo* processInfo;
	SHOTSettings::Settings *settings;
};
