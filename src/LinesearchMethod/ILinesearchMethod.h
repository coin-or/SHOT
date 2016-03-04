#pragma once
#include <vector>
//#include "../OptProblems/OptProblemReformulated.h"
#include "../OptProblems/OptProblemOriginal.h"

class ILinesearchMethod
{
	public:
		virtual ~ILinesearchMethod()
		{
		}
		;

		virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
				std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol) = 0;

		virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
				std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol, std::vector<int> constrIdxs) = 0;
};
