#pragma once
#include "ILinesearchMethod.h"
#include "SHOTSettings.h"

#include <boost/math/tools/roots.hpp>

//#include "..\OptProblems\OptProblemReformulated.h"
class LinesearchMethodBoost :
	public ILinesearchMethod
{
public:
	LinesearchMethodBoost();
	virtual ~LinesearchMethodBoost();

	virtual std::vector<double> findZero(std::vector<double> ptA, std::vector<double> ptB, int Nmax, double delta);

private:
	ProcessInfo* processInfo;
	SHOTSettings::Settings *settings;
};
