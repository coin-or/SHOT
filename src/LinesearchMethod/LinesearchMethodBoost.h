#pragma once
#include "ILinesearchMethod.h"
#include "SHOTSettings.h"

#include "../OptProblems/OptProblemOriginal.h"
#include "boost/math/tools/roots.hpp"

class Test
{
	private:

		//std::vector<char> varTypes;
		//std::vector<int> activeConstraints;

	public:
		std::vector<double> firstPt;
		std::vector<double> secondPt;

		double valFirstPt;
		double valSecondPt;

		OptProblemOriginal *originalProblem;

		Test();
		void determineActiveConstraints(double constrTol);
		void setActiveConstraints(std::vector<int> constrIdxs);
		std::vector<int> getActiveConstraints();
		void clearActiveConstraints();
		void addActiveConstraint(int constrIdx);

		double operator()(const double x);

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

class LinesearchMethodBoost: public ILinesearchMethod
{
	public:
		LinesearchMethodBoost();
		virtual ~LinesearchMethodBoost();

		virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
				std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol);

		virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
				std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol, std::vector<int> constrIdxs);
	private:
		ProcessInfo* processInfo;
		SHOTSettings::Settings *settings;
		Test *test;
};
