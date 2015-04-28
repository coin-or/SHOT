#pragma once
#include "SHOTSettings.h"
#include "../Iteration.h"
#include "../ProcessInfo.h"

class ConstraintToleranceUpdateStrategyBase
{
protected:
	SHOTSettings::Settings *settings;
	ProcessInfo *processInfo;
public:
	ConstraintToleranceUpdateStrategyBase();
	~ConstraintToleranceUpdateStrategyBase();
};

