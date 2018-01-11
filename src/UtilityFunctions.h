#pragma once
#include <vector>
#include <ostream>
#include "OSFileUtil.h"
#include "OSInstance.h"
#include "boost/format.hpp"
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <boost/math/special_functions/fpclassify.hpp> // isnan
#include "Structs.h"

// Fix for missing NAN i Visual Studio
#ifdef WIN32
#ifndef NAN
static const unsigned long __nan[2] =
{	0xffffffff, 0x7fffffff};
#define NAN (*(const float *) __nan)
#endif
#endif

// Fix for Visual Studio c++ compiler
namespace UtilityFunctions
{
	int round(double d);

	bool isnan(double val);

	void saveVariablePointVectorToFile(std::vector<double> point, std::vector<std::string> variables,
			std::string fileName);

	void savePrimalSolutionToFile(PrimalSolution solution, std::vector<std::string> variables, std::string fileName);

	bool isObjectiveGenerallyNonlinear(OSInstance *instance);
	bool isObjectiveQuadratic(OSInstance *instance);

	void displayVector(std::vector<double> point);
	void displayVector(std::vector<double> point1, std::vector<double> point2);
	void displayVector(std::vector<int> point);
	void displayVector(std::vector<std::string> point);
	void displayVector(std::vector<int> point1, std::vector<int> point2);
	void displayVector(std::vector<int> point1, std::vector<double> point2);

	void displayVector(std::vector<std::vector<double>> points);
	void displayVector(std::vector<std::vector<int>> points);
	void displayVector(std::vector<std::vector<std::string>> points);

	void displayDifferencesInVector(std::vector<double> point1, std::vector<double> point2, double tol);

	double L2Norm(std::vector<double> ptA, std::vector<double> ptB);
	std::vector<double> L2Norms(std::vector<std::vector<double>> ptsA, std::vector<double> ptB);
	std::vector<double> calculateCenterPoint(std::vector<std::vector<double>> pts);

	int numDifferentRoundedSelectedElements(std::vector<double> firstPt, std::vector<double> secondPt,
			std::vector<int> indexes);
	bool isDifferentRoundedSelectedElements(std::vector<double> firstPt, std::vector<double> secondPt,
			std::vector<int> indexes);

	bool isDifferentSelectedElements(std::vector<double> firstPt, std::vector<double> secondPt,
			std::vector<int> indexes);

	std::string toStringFormat(double value, std::string format, bool useInfinitySymbol);
	std::string toStringFormat(double value, std::string format);
	std::string toString(double value);
}
