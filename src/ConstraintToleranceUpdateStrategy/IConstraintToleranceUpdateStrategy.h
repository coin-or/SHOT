#pragma once
class IConstraintToleranceUpdateStrategy
{
public:
	//virtual IConstraintToleranceUpdateStrategy();
	virtual ~IConstraintToleranceUpdateStrategy() {};

	virtual void calculateNewTolerance() = 0;
	//virtual double getInitialTolerance() = 0;
};

