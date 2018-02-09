#pragma once
#include "Enums.h"
#include "vector"
#include "SHOTSettings.h"

/*
 enum E_IterationTerminationType {
 Optimal,
 TimeLimit,
 IterationLimit,
 Infeasible,
 Error
 };*/

struct SolutionPoint
{
		vector<double> point;
		double objectiveValue;
		int iterFound;
		IndexValuePair maxDeviation;
};

class Iteration
{
	public:
		Iteration();
		~Iteration();

		E_IterationProblemType type;
		E_ProblemSolutionStatus solutionStatus;

		std::vector<SolutionPoint> solutionPoints;

		double objectiveValue;
		std::pair<double, double> currentObjectiveBounds;

		std::vector<double> constraintDeviations;
		double maxDeviation;
		int maxDeviationConstraint;

		double usedConstraintTolerance;

		int usedMIPSolutionLimit;
		bool MIPSolutionLimitUpdated;

		int iterationNumber;

		int numHyperplanesAdded;
		int totNumHyperplanes;

		double boundaryDistance;

		bool isMIP();

		double solutionTime;

		std::vector<std::vector<double>> hyperplanePoints;

		SolutionPoint getSolutionPointWithSmallestDeviation();
		int getSolutionPointWithSmallestDeviationIndex();
};

