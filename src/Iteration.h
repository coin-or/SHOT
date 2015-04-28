#pragma once
#include "Enums.h"
#include <vector>
#include "SHOTSettings.h"

/*
 enum E_IterationTerminationType {
 Optimal,
 TimeLimit,
 IterationLimit,
 Infeasible,
 Error
 };*/

class Iteration
{
	public:
		Iteration();
		~Iteration();

		E_IterationProblemType type;
		E_ProblemSolutionStatus solutionStatus;

		std::vector<std::vector<double>> variableSolutions;
		double objectiveValue;
		std::pair<double, double> currentObjectiveBounds;

		std::vector<double> constraintDeviations;
		double maxDeviation;
		int maxDeviationConstraint;

		double usedConstraintTolerance;

		int usedMILPSolutionLimit;
		bool MILPSolutionLimitUpdated;

		int iterationNumber;

		int numHyperplanesAdded;
		int totNumHyperplanes;

		double boundaryDistance;

		bool isMILP();

		double solutionTime;

		std::vector<std::vector<double>> hyperplanePoints;

};

