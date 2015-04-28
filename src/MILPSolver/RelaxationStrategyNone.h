#pragma once
#include "IRelaxationStrategy.h"
#include "RelaxationStrategyBase.h"

class RelaxationStrategyNone: public IRelaxationStrategy, RelaxationStrategyBase
{
	public:
		RelaxationStrategyNone();
		~RelaxationStrategyNone();

		virtual void executeStrategy();

		virtual void setActive();

		virtual void setInactive();

		virtual void setInitial();

		virtual E_IterationProblemType getProblemType();

	private:
};
