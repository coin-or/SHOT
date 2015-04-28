#pragma once
#include "IRelaxationStrategy.h"
#include "RelaxationStrategyBase.h"

class RelaxationStrategyStandard: public IRelaxationStrategy, RelaxationStrategyBase
{
	public:
		RelaxationStrategyStandard();
		~RelaxationStrategyStandard();

		virtual void executeStrategy();

		virtual void setActive();

		virtual void setInactive();

		virtual void setInitial();

		virtual E_IterationProblemType getProblemType();

	private:
		bool isIterationLimitReached();
		bool isLPStepFinished();
		bool isObjectiveStagnant();

		bool LPFinished;
};
