#pragma once
#include <vector>
#include <iostream>
#include "OSFileUtil.h"
#include "OSInstance.h"

namespace UtilityFunctions
{
	void saveVariablePointVectorToFile(std::vector<double> point, std::vector<std::string> variables,
			std::string fileName);
	bool isObjectiveGenerallyNonlinear(OSInstance *instance);
	bool isObjectiveQuadratic(OSInstance *instance);

	void displayVector(std::vector<double> point);
	void displayVector(std::vector<int> point);
	void displayVector(std::vector<std::string> point);

	double L2Norm(std::vector<double> ptA, std::vector<double> ptB);
	std::vector<double> L2Norms(std::vector<std::vector<double>> ptsA, std::vector<double> ptB);
	std::vector<double> calculateCenterPoint(std::vector<std::vector<double>> pts);

}
