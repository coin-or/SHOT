#include "UtilityFunctions.h"

void UtilityFunctions::saveVariablePointVectorToFile(std::vector<double> point, std::vector<std::string> variables,
		std::string fileName)
{
	if (point.size() != variables.size())
	{
		std::cout << "Error when saving variable point to file. Sizes mismatch!" << point.size() << "!="
				<< variables.size() << std::endl;
		return;
	}

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

void UtilityFunctions::displayVector(std::vector<double> point1, std::vector<double> point2)
{
	std::stringstream str;

	if (point1.size() != point2.size()) return;

	for (int i = 0; i < point1.size(); i++)
	{
		str << i;
		str << "\t";
		str << point1.at(i);

		str << "\t";
		str << point2.at(i);
		str << std::endl;
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

void UtilityFunctions::displayVector(std::vector<std::vector<double> > points)
{
	std::stringstream str;

	for (int i = 0; i < points.at(0).size(); i++)
	{
		str << i;

		for (int j = 0; j < points.size(); j++)
		{
			str << "\t";
			str << points.at(j).at(i);
		}

		str << std::endl;
	}

	std::cout << str.str() << std::endl;
}

void UtilityFunctions::displayVector(std::vector<std::vector<int> > points)
{
	std::stringstream str;

	for (int i = 0; i < points.at(0).size(); i++)
	{
		str << i;

		for (int j = 0; j < points.size(); j++)
		{
			str << "\t";
			str << points.at(j).at(i);
		}

		str << std::endl;
	}

	std::cout << str.str() << std::endl;
}

void UtilityFunctions::displayVector(std::vector<std::vector<std::string> > points)
{
	std::stringstream str;

	for (int i = 0; i < points.at(0).size(); i++)
	{
		str << i;

		for (int j = 0; j < points.size(); j++)
		{
			str << "\t";
			str << points.at(j).at(i);
		}

		str << std::endl;
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

	for (int i = 0; i < ptSize; i++)
	{

		for (int j = 0; j < numPts; j++)
		{
			newPt.at(i) += pts.at(j).at(i);
		}

		newPt.at(i) = newPt.at(i) / numPts;
	}

	return (newPt);
}

int UtilityFunctions::numDifferentRoundedSelectedElements(std::vector<double> firstPt, std::vector<double> secondPt,
		std::vector<int> indexes)
{
	int numDiff = 0;

	for (int i = 0; i < indexes.size(); i++)
	{
		if (round(firstPt.at(indexes.at(i))) != round(secondPt.at(indexes.at(i))))
		{
			numDiff++;
			/*std::cout << "Different at element: " << indexes.at(i) << std::endl;
			 std::cout << " values are " << firstPt.at(indexes.at(i)) << " and " << secondPt.at(indexes.at(i))
			 << std::endl;*/
		}
	}
	return (numDiff);
}

bool UtilityFunctions::isDifferentRoundedSelectedElements(std::vector<double> firstPt, std::vector<double> secondPt,
		std::vector<int> indexes)
{
	for (int i = 0; i < indexes.size(); i++)
	{
		if (round(firstPt.at(indexes.at(i))) != round(secondPt.at(indexes.at(i)))) return (true);
	}

	return (false);

}

bool UtilityFunctions::isDifferentSelectedElements(std::vector<double> firstPt, std::vector<double> secondPt,
		std::vector<int> indexes)
{
	for (int i = 0; i < indexes.size(); i++)
	{
		if (firstPt.at(indexes.at(i)) != secondPt.at(indexes.at(i))) return (true);
	}

	return (false);

}

std::string UtilityFunctions::toStringFormat(double value, std::string format)
{
	return (UtilityFunctions::toStringFormat(value, format, true));
}

std::string UtilityFunctions::toStringFormat(double value, std::string format, bool useInfinitySymbol)
{
	std::string str;

	if (useInfinitySymbol && value < -1.e100)
	{
		str = "-∞";
	}
	else if (useInfinitySymbol && value > 1.e100)
	{
		str = "∞";
	}
	else
	{
		str = ((boost::format(format) % value).str());
	}

	return (str);
}

std::string UtilityFunctions::toString(double value)
{
	return (UtilityFunctions::toStringFormat(value, "%.3f", true));
}
