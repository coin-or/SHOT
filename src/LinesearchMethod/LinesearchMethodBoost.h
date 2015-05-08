#pragma once
#include "ILinesearchMethod.h"
#include "SHOTSettings.h"

#include <boost/math/tools/roots.hpp>

class Test
{
	private:

		//std::vector<char> varTypes;

	public:
		std::vector<double> firstPt;
		std::vector<double> secondPt;
		OptProblemOriginal *originalProblem;

		Test()
		{
			/*firstPt = ptA;
			 secondPt = ptB;
			 originalProblem = prob;
			 */
			//auto varTypes = originalProblem->getVariableTypes();
			/*
			 for (int i = 0; i < originalProblem->getNumberOfVariables();i++)
			 {
			 if (varTypes.at(i) == 'B' || varTypes.at(i) == 'I')
			 {
			 std::cout << "changing point from " << secondPt.at(i) " to " <<firstPt.at(i) << std::endl;
			 secondPt.at(i) = firstPt.at(i);
			 }
			 }*/
		}

		double operator()(const double x)
		{
			int length = firstPt.size();
			std::vector<double> ptNew(length);

			for (int i = 0; i < length; i++)
			{
				ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
			}

			auto validNewPt = originalProblem->getMostDeviatingConstraint(ptNew).value;

			return validNewPt;
		}

};

class TerminationCondition
{
	private:
		double tol;

	public:
		TerminationCondition(double tolerance)
		{
			tol = tolerance;
		}

		bool operator()(double min, double max)
		{
			return (abs(min - max) <= tol);
		}
};

//#include "..\OptProblems\OptProblemReformulated.h"
class LinesearchMethodBoost: public ILinesearchMethod
{
	public:
		LinesearchMethodBoost();
		virtual ~LinesearchMethodBoost();

		virtual std::vector<double> findZero(std::vector<double> ptA, std::vector<double> ptB, int Nmax, double delta);

	private:
		ProcessInfo* processInfo;
		SHOTSettings::Settings *settings;
		Test *test;
};
