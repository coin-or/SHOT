#include "NLPSolverBase.h"


NLPSolverBase::NLPSolverBase()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

NLPSolverBase::~NLPSolverBase()
{
}

