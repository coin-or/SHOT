#pragma once
#include "../Enums.h"
#include "SHOTSettings.h"
class IRelaxationStrategy
{
public:
	//IRelaxationStrategy();
	virtual ~IRelaxationStrategy() {};

	virtual void executeStrategy() = 0;

	virtual void setActive() = 0;
	
	virtual void setInactive() = 0;

	virtual void setInitial() = 0;

	virtual E_IterationProblemType getProblemType() = 0;
};
