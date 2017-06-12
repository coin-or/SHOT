#pragma once
#include "IRelaxationStrategy.h"
#include "RelaxationStrategyBase.h"
#include "algorithm"

class RelaxationStrategyAdaptive: public IRelaxationStrategy, RelaxationStrategyBase
{
	public:
		RelaxationStrategyAdaptive(IMILPSolver *MILPSolver);
		~RelaxationStrategyAdaptive();

		virtual void executeStrategy();

		virtual void setActive();

		virtual void setInactive();

		virtual void setInitial();

		virtual E_IterationProblemType getProblemType();

	private:
		bool isRelaxationDistanceSmall();
		bool isCurrentToleranceReached();
		bool isIterationLimitReached();
		void updateCurrentDistanceLevel();

		double currentDistanceLevel;
		//double initialDistanceLevel;
		//int numLPMeans;

		bool maxLPToleranceReached;

		int iterLastMILP;

		std::vector<double> distanceLevels;

		IMILPSolver *MILPSolver;
};
