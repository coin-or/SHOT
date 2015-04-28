#include "Iteration.h"

Iteration::Iteration()
{
}

Iteration::~Iteration()
{
}

bool Iteration::isMILP()
{

	return (this->type == E_IterationProblemType::MIP);
}
