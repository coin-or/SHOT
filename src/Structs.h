#pragma once
#include <vector>
#include "Enums.h"
#include "OSGeneral.h"

struct InteriorPoint
{
		std::vector<double> point;
		ES_NLPSolver NLPSolver;
		IndexValuePair maxDevatingConstraint;
};

struct PrimalSolution
{
		std::vector<double> point;
		E_PrimalSolutionSource sourceType;
		std::string sourceDescription;
		double objValue;
		int iterFound;
		IndexValuePair maxDevatingConstraintNonlinear;
		IndexValuePair maxDevatingConstraintLinear;
		double maxIntegerToleranceError; // The maximum integer error before rounding
		bool boundProjectionPerformed = false; // Has the variable bounds been corrected to either upper or lower bounds?
		bool integerRoundingPerformed = false; // Has the integers been rounded?
		//bool assumeLinearConstraintsValid = false; // In some cases the linear constraints are assumed to be valid
		bool displayed = false; // Has the primal solution been displayed on console?
};

struct PrimalFixedNLPCandidate
{
		std::vector<double> point;
		E_PrimalNLPSource sourceType;
		double objValue;
		int iterFound;
		IndexValuePair maxDevatingConstraint;
};

struct DualSolution
{
		std::vector<double> point;
		E_DualSolutionSource sourceType;
		double objValue;
		int iterFound;
		bool displayed; // Has the dual solution been displayed on console?
};

struct Hyperplane
{
		int sourceConstraintIndex;
		std::vector<double> generatedPoint;
		E_HyperplaneSource source;
};
