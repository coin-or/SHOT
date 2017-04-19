#pragma once
#include <vector>
#include <iostream>
#include "OSFileUtil.h"
#include "OSInstance.h"
#include <boost/format.hpp>

namespace UtilityFunctions
{
	void saveVariablePointVectorToFile(std::vector<double> point, std::vector<std::string> variables,
			std::string fileName);
	bool isObjectiveGenerallyNonlinear(OSInstance *instance);
	bool isObjectiveQuadratic(OSInstance *instance);

	void displayVector(std::vector<double> point);
	void displayVector(std::vector<double> point1, std::vector<double> point2);
	void displayVector(std::vector<int> point);
	void displayVector(std::vector<std::string> point);

	void displayVector(std::vector<std::vector<double>> points);
	void displayVector(std::vector<std::vector<int>> points);
	void displayVector(std::vector<std::vector<std::string>> points);

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
