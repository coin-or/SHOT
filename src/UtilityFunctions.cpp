#include "UtilityFunctions.h"

void UtilityFunctions::saveVariablePointVectorToFile(std::vector<double> point, std::vector<std::string> variables,
		std::string fileName)
{
	FileUtil *fileUtil = new FileUtil();

	std::stringstream str;

	for (int i = 0; i < point.size(); i++)
	{
		str << variables.at(i);
		str << "\t";
		str << point.at(i) << std::endl;
	}

	fileUtil->writeFileFromString(fileName, str.str());

	delete fileUtil;
}

void UtilityFunctions::displayVector(std::vector<double> point)
{

	std::stringstream str;

	for (int i = 0; i < point.size(); i++)
	{
		str << i;
		str << "\t";
		str << point.at(i) << std::endl;
	}

	std::cout << str.str() << std::endl;
}

void UtilityFunctions::displayVector(std::vector<int> point)
{

	std::stringstream str;

	for (int i = 0; i < point.size(); i++)
	{
		str << i;
		str << "\t";
		str << point.at(i) << std::endl;
	}

	std::cout << str.str() << std::endl;
}

void UtilityFunctions::displayVector(std::vector<std::string> point)
{

	std::stringstream str;

	for (int i = 0; i < point.size(); i++)
	{
		str << i;
		str << "\t";
		str << point.at(i) << std::endl;
	}

	std::cout << str.str() << std::endl;
}

bool UtilityFunctions::isObjectiveGenerallyNonlinear(OSInstance *instance)
{
	for (int i = 0; i < instance->getNumberOfNonlinearExpressions(); i++)
	{
		int tmpIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;
		if (tmpIndex == -1) return true;
	}
	return false;
}

bool UtilityFunctions::isObjectiveQuadratic(OSInstance *instance)
{
	for (int i = 0; i < instance->getNumberOfQuadraticTerms(); i++)
	{
		int tmpIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

		if (tmpIndex == -1) return true;
	}

	return false;
}

double UtilityFunctions::L2Norm(std::vector<double> ptA, std::vector<double> ptB)
{
	double norm = 0.0;

	if (ptA.size() != ptB.size())
	{
		return -1.0;
	}

	for (int i = 0; i < ptA.size(); i++)
	{
		norm = norm + pow(ptA.at(i) - ptB.at(i), 2.0);
	}

	norm = sqrt(norm);

	return norm;
}

std::vector<double> UtilityFunctions::L2Norms(std::vector<std::vector<double>> ptsA, std::vector<double> ptB)
{
	std::vector<double> norms(ptsA.size());

	for (int i = 0; i < ptsA.size(); i++)
	{
		norms.at(i) = L2Norm(ptsA.at(i), ptB);
	}

	return norms;
}

std::vector<double> UtilityFunctions::calculateCenterPoint(std::vector<std::vector<double>> pts)
{
	int ptSize = pts.at(0).size();
	int numPts = pts.size();

	std::vector<double> newPt(ptSize, 0.0);

	/*for (int j=0; j < numPts; j++)
	 {
	 displayVector(pts.at(j));
	 }*/

	for (int i = 0; i < ptSize; i++)
	{

		for (int j = 0; j < numPts; j++)
		{
			newPt.at(i) += pts.at(j).at(i);
		}

		newPt.at(i) = newPt.at(i) / numPts;
	}

	//displayVector(newPt);

	return newPt;
}
