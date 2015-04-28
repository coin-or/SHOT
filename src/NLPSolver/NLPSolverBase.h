#pragma once
#include "SHOTSettings.h"
#include "../ProcessInfo.h"

class NLPSolverBase
{
protected:
	SHOTSettings::Settings *settings;
	ProcessInfo *processInfo;

public:
	NLPSolverBase();
	~NLPSolverBase();
};
