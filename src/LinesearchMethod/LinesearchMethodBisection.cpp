#include "LinesearchMethodBisection.h"


LinesearchMethodBisection::LinesearchMethodBisection()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

LinesearchMethodBisection::~LinesearchMethodBisection()
{
}

std::vector<double> LinesearchMethodBisection::findZero(std::vector<double> ptA, std::vector<double> ptB, int Nmax, double delta)
{
	try
	{
		int length = ptA.size();
		bool validNewPt;
		bool validB = processInfo->originalProblem->isConstraintsFulfilledInPoint(ptB);

		std::vector<double> ptNew(length);

		double a = 0;
		double b = 1;
		double c;
		int n = 1;

		while (n <= Nmax)
		{
			c = (a + b) / 2.0;
			for (int i = 0; i < length; i++)
			{
				ptNew.at(i) = c * ptA.at(i) + (1 - c)*ptB.at(i);
			}

			validNewPt = processInfo->originalProblem->isConstraintsFulfilledInPoint(ptNew);

			if ((b - a) / 2 < delta)
			{
				break;
			}

			n++;

			if ((validNewPt && validB) || (!validNewPt && !validB))
			{
				a = c;
				validB = validNewPt;
			}
			else
			{
				b = c;
			}
		}

		//std::cout << "Obtained linesearch point with lambda =" << c << " : " << std::endl;
		/*for (int i = 0; i < length; i++)
		{
		std::cout << "   " << i << ": " << ptNew[i] << std::endl;
		}*/

		//std::cout << "Error term linesearch: " << problemInstance->getMostDeviatingConstraint(ptNew).value << std::endl;

		processInfo->logger.message(3) << " Linesearch completed in " << n << " iterations." << CoinMessageEol;

		return ptNew;
	}
	catch (...)
	{
		processInfo->logger.message(0) << "Error while doing linesearch." << CoinMessageEol;
		processInfo->logger.message(0) << "Returning solution point instead. " << CoinMessageEol;
		return ptB;
	}
}