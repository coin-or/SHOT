#pragma once
#include <vector>
//#include "../OptProblems/OptProblemReformulated.h"
#include "../OptProblems/OptProblemOriginal.h"

class ILinesearchMethod
{
public:
	virtual ~ILinesearchMethod() {};

	virtual std::vector<double> findZero(std::vector<double> ptA, std::vector<double> ptB, int Nmax, double delta) = 0;

};
