#include "ProcessInfo.h"
#include "OptProblems/OptProblemOriginal.h"

bool ProcessInfo::instanceFlag = false;
ProcessInfo* ProcessInfo::single = NULL;
CoinMessageHandler logger;
OSResult *osResult = NULL;

void ProcessInfo::addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter)
{
	PrimalSolution sol =
	{ pt, source, objVal, iter };

	primalSolutions.push_back(sol);
}

void ProcessInfo::addDualSolution(vector<double> pt, E_DualSolutionSource source, double objVal, int iter)
{
	DualSolution sol =
	{ pt, source, objVal, iter };

	dualSolutions.push_back(sol);
}

void ProcessInfo::addPrimalSolutionCandidate(vector<double> pt, E_PrimalSolutionSource source, int iter)
{
	PrimalSolution sol =
	{ pt, source, NAN, iter };

	primalSolutionCandidates.push_back(sol);
}

void ProcessInfo::addPrimalSolutionCandidates(vector<vector<double> > pts, E_PrimalSolutionSource source, int iter)
{
	for (auto PT : pts)
	{
		addPrimalSolutionCandidate(PT, source, iter);
	}
}

void ProcessInfo::addDualSolutionCandidate(vector<double> pt, E_DualSolutionSource source, int iter)
{
	DualSolution sol =
	{ pt, source, NAN, iter };

	dualSolutionCandidates.push_back(sol);
}

pair<double, double> ProcessInfo::getCorrectedObjectiveBounds()
{
	pair<double, double> bounds;

	if (originalProblem->isTypeOfObjectiveMinimize())
	{
		bounds.first = currentObjectiveBounds.first;
		bounds.second = currentObjectiveBounds.second;
	}
	else
	{
		bounds.first = currentObjectiveBounds.second;
		bounds.second = currentObjectiveBounds.first;
	}

	return (bounds);
}

void ProcessInfo::addPrimalSolution(SolutionPoint pt, E_PrimalSolutionSource source)
{
	PrimalSolution sol =
	{ pt.point, source, pt.objectiveValue, pt.iterFound };

	primalSolutions.push_back(sol);
}

void ProcessInfo::addDualSolution(SolutionPoint pt, E_DualSolutionSource source)
{
	DualSolution sol =
	{ pt.point, source, pt.objectiveValue, pt.iterFound };

	dualSolutions.push_back(sol);
}

void ProcessInfo::addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source)
{
	PrimalSolution sol =
	{ pt.point, source, pt.objectiveValue, pt.iterFound };

	primalSolutionCandidates.push_back(sol);
}

void ProcessInfo::addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source)
{
	for (auto pt : pts)
	{
		addPrimalSolutionCandidate(pt, source);
	}
}

void ProcessInfo::addDualSolutionCandidate(SolutionPoint pt, E_DualSolutionSource source)
{
	DualSolution sol =
	{ pt.point, source, pt.objectiveValue, pt.iterFound };

	dualSolutionCandidates.push_back(sol);
}

void ProcessInfo::addDualSolutionCandidates(std::vector<SolutionPoint> pts, E_DualSolutionSource source)
{
	for (auto pt : pts)
	{
		addDualSolutionCandidate(pt, source);
	}

}

ProcessInfo::ProcessInfo()
{
	createTimer("Total", "Total solution time");

	iterLP = 0;
	iterFeasMILP = 0;
	iterOptMILP = 0;
	itersWithStagnationMILP = 0;
	iterSignificantObjectiveUpdate = 0;
	itersMILPWithoutNLPCall = 0;
	solTimeLastNLPCall = 0;

	iterLastPrimalBoundUpdate = 0;
	iterLastDualBoundUpdate = 0;

	numOriginalInteriorPoints = 0;

	currentObjectiveBounds.first = -DBL_MAX;
	currentObjectiveBounds.second = DBL_MAX;

	settings = SHOTSettings::Settings::getInstance();

	logger.setLogLevel(2);

	tasks = new TaskHandler();
}

ProcessInfo* ProcessInfo::getInstance()
{
	if (!instanceFlag)
	{
		single = new ProcessInfo();
		instanceFlag = true;
		return (single);
	}
	else
	{
		return (single);
	}
}

void ProcessInfo::setOriginalProblem(OptProblemOriginal *problem)
{
	this->originalProblem = problem;

	bool isMinimization = originalProblem->isTypeOfObjectiveMinimize();

	if (isMinimization)
	{
		currentObjectiveBounds.first = -DBL_MAX;
		currentObjectiveBounds.second = DBL_MAX;
	}
	else
	{
		currentObjectiveBounds.first = DBL_MAX;
		currentObjectiveBounds.second = -DBL_MAX;
	}

}

void ProcessInfo::createTimer(string name, string description)
{
	Timer tmpTimer = Timer(name, description);

	timers.push_back(tmpTimer);
}

void ProcessInfo::startTimer(string name)
{
	auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const & T)
	{
		return (T.name == name);
	});

	if (timer == timers.end())
	{
		logger.message(0) << "Error: timer with name  \" " << name << "\" not found!";
		return;
	}

	timer->start();
}

void ProcessInfo::restartTimer(string name)
{
	auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const & T)
	{
		return (T.name == name);
	});

	if (timer == timers.end())
	{
		logger.message(0) << "Error: timer with name  \" " << name << "\" not found!";
		return;
	}

	timer->restart();
}

void ProcessInfo::stopTimer(string name)
{
	auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const & T)
	{
		return (T.name == name);
	});

	if (timer == timers.end())
	{
		logger.message(0) << "Error: timer with name  \" " << name << "\" not found!";
		return;
	}

	timer->stop();
}

double ProcessInfo::getElapsedTime(string name)
{
	auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const & T)
	{
		return (T.name == name);
	});

	if (timer == timers.end())
	{
		logger.message(0) << "Error: timer with name  \" " << name << "\" not found!";
		return (0.0);
	}

	return (timer->elapsed());
}

void ProcessInfo::initializeResults(int numObj, int numVar, int numConstr)
{
	osResult = new OSResult();
	osResult->setObjectiveNumber(numObj);
	osResult->setVariableNumber(numVar);
	osResult->setConstraintNumber(numConstr);

	osResult->setSolutionNumber(1);
}

std::string ProcessInfo::getOSrl()
{

	/*if (dualSolutions.size() > 0)
	 {
	 osResult->setDualVariableValuesDense(0, &(dualSolutions.at(dualSolutions.size() - 1).point)[0]);
	 }
	 */
	if (primalSolutions.size() > 0)
	{
		osResult->setPrimalVariableValuesDense(0, &(primalSolutions.at(primalSolutions.size() - 1).point)[0]);
	}

	//osResult->setObjValue(0, 0, -1, originalProblem->getProblemInstance()->getObjectiveNames()[0], getDualBound());

	double tmpObjval[1] =
	{ getDualBound() };

	osResult->setObjectiveValuesDense(0, tmpObjval);

	osResult->setAnOtherSolutionResult(0, "MaxErrorConstrs", std::to_string(getCurrentIteration()->maxDeviation),
			"Term_tolerance", "Maximal error in constraint", 0, NULL);

	for (auto T : timers)
	{
		osResult->addTimingInformation(T.name, "SHOT", "second", T.description, T.elapsed());
	}

	/*
	 osResult->addTimingInformation("ElapsedTotal", "total", "second", "Total solution time",
	 getElapsedTime(E_TimerTypes::Total));
	 osResult->addTimingInformation("ElapsedReformulating", "input", "second", "Total preprocessing time",
	 getElapsedTime(E_TimerTypes::ReformulateProblem));
	 osResult->addTimingInformation("ElapsedNLP", "preprocessing", "second", "Total solution time for NLP solver",
	 getElapsedTime(E_TimerTypes::ReformulateSolveNLP));
	 osResult->addTimingInformation("ElapsedLPMILP", "total", "second", "The total LP/MILP solution time",
	 getElapsedTime(E_TimerTypes::TotalLPMILP));
	 osResult->addTimingInformation("ElapsedLP1", "total", "second", "The total LP1 solution time",
	 getElapsedTime(E_TimerTypes::TotalLP1));
	 osResult->addTimingInformation("ElapsedLP2", "total", "second", "The total LP2 solution time",
	 getElapsedTime(E_TimerTypes::TotalLP2));
	 osResult->addTimingInformation("ElapsedMILP", "total", "second", "The total MILP solution time",
	 getElapsedTime(E_TimerTypes::TotalMILP));
	 osResult->addTimingInformation("ElapsedLinesearch", "total", "second", "The total time spent on linesearch",
	 getElapsedTime(E_TimerTypes::Linesearch));
	 */

	osResult->setAnOtherSolutionResult(0, "IterationsLP", std::to_string(iterLP), "Iterations", "LP iterations", 0,
			NULL);
	osResult->setAnOtherSolutionResult(0, "IterationsFeasibleMILP", std::to_string(iterFeasMILP), "Iterations",
			"MILP iterations solved to feasibility", 0, NULL);
	osResult->setAnOtherSolutionResult(0, "IterationsOptimalMILP", std::to_string(iterOptMILP), "Iterations",
			"MILP iterations solved to optimality", 0, NULL);
	osResult->setAnOtherSolutionResult(0, "IterationsTotal", std::to_string(iterLP + iterFeasMILP + iterOptMILP),
			"Iterations", "Total number of iterations", 0, NULL);

	osResult->setAnOtherSolutionResult(0, "FileName", this->originalProblem->getProblemInstance()->getInstanceName(),
			"Problem", "The original filename", 0, NULL);

	osResult->setAnOtherSolutionResult(0, "NumberVariables",
			std::to_string(this->originalProblem->getProblemInstance()->getVariableNumber()), "Problem",
			"Total number of variables", 0, NULL);
	osResult->setAnOtherSolutionResult(0, "NumberContinousVariables",
			std::to_string(
					this->originalProblem->getProblemInstance()->getVariableNumber()
							- this->originalProblem->getNumberOfIntegerVariables()
							- this->originalProblem->getNumberOfBinaryVariables()), "Problem",
			"Number of continuous variables", 0, NULL);
	osResult->setAnOtherSolutionResult(0, "NumberBinaryVariables",
			std::to_string(this->originalProblem->getProblemInstance()->getNumberOfBinaryVariables()), "Problem",
			"Number of binary variables", 0, NULL);
	osResult->setAnOtherSolutionResult(0, "NumberIntegerVariables",
			std::to_string(this->originalProblem->getProblemInstance()->getNumberOfIntegerVariables()), "Problem",
			"Number of integer variables", 0, NULL);

	osResult->setAnOtherSolutionResult(0, "NumberConstraints",
			std::to_string(this->originalProblem->getProblemInstance()->getConstraintNumber()), "Problem",
			"Number of constraints", 0, NULL);
	osResult->setAnOtherSolutionResult(0, "NumberNonlinearConstraints",
			std::to_string(this->originalProblem->getProblemInstance()->getNumberOfNonlinearConstraints()), "Problem",
			"Number of nonlinear constraints", 0, NULL);
	osResult->setAnOtherSolutionResult(0, "NumberLinearConstraints",
			std::to_string(
					this->originalProblem->getProblemInstance()->getConstraintNumber()
							- this->originalProblem->getNumberOfNonlinearConstraints()), "Problem",
			"Number of linear constraints", 0, NULL);

	OSrLWriter writer;

	std::string osrl = writer.writeOSrL(osResult);

	return (osrl);
}

std::string ProcessInfo::getTraceResult()
{
	std::stringstream ss;

	this->originalProblem->getProblemInstance()->initializeNonLinearStructures();

	if (this->originalProblem->getProblemInstance()->getNumberOfQuadraticTerms() > 0)
	{
		this->originalProblem->getProblemInstance()->addQTermsToExpressionTree();
	}

	this->originalProblem->getProblemInstance()->bVariablesModified = true;
	this->originalProblem->getProblemInstance()->bConstraintsModified = true;
	this->originalProblem->getProblemInstance()->bObjectivesModified = true;

	ss << this->originalProblem->getProblemInstance()->getInstanceName() << ",";
	ss << "MINLP" << ",";
	ss << "SHOT" << ",";
	ss << "IPOpt" << ",";
	ss << "Cplex" << ",";
	ss << "9999999" << ",";
	ss << (originalProblem->getProblemInstance()->getObjectiveMaxOrMins()[0] == "min" ? "0" : "1") << ",";
	ss << this->originalProblem->getProblemInstance()->getConstraintNumber() + 1 << ","; // +1 to comply with GAMS objective style
	ss << this->originalProblem->getProblemInstance()->getVariableNumber() + 1 << ","; // +1 to comply with GAMS objective style
	ss << this->originalProblem->getNumberOfBinaryVariables() + this->originalProblem->getNumberOfIntegerVariables()
			<< ",";

	auto nonzeroes = this->originalProblem->getProblemInstance()->getLinearConstraintCoefficientNumber()
			+ this->originalProblem->getProblemInstance()->getAllNonlinearVariablesIndexMap().size() + 1;
//this->originalProblem->getObjectiveCoefficientNumbers()[0] + 1;

	ss << nonzeroes << ",";
	ss << this->originalProblem->getProblemInstance()->getAllNonlinearVariablesIndexMap().size() << ",";
	ss << "1" << ",";

	std::string solverStatus = "";
	std::string modelStatus = "";

	auto t = this->getCurrentIteration()->solutionStatus;

	if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		modelStatus = "8";
	}
	else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Feasible)
	{
		modelStatus = "7";
	}
	else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Unbounded)
	{
		modelStatus = "18";
	}
	else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Error)
	{
		modelStatus = "12";
	}
	else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Infeasible)
	{
		modelStatus = "4";
	}
	else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
	{
		modelStatus = "7";
	}
	else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
	{
		modelStatus = "7";
	}
	else if (this->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
	{
		modelStatus = "7";
	}
	else
	{
		modelStatus = "NA";
		this->logger.message(0) << "ERROR: Unknown return code from model solution" << CoinMessageEol;
	}

	if (this->terminationReason == E_TerminationReason::TimeLimit)
	{
		solverStatus = "3";
	}
	else if (this->terminationReason == E_TerminationReason::IterationLimit)
	{
		solverStatus = "2";
	}
	else if (this->terminationReason == E_TerminationReason::ObjectiveStagnation)
	{
		solverStatus = "2";
	}
	else if (this->terminationReason == E_TerminationReason::Error)
	{
		solverStatus = "10";
	}
	else if (this->terminationReason == E_TerminationReason::InfeasibleProblem)
	{
		solverStatus = "4";
	}
	else if (this->terminationReason == E_TerminationReason::ConstraintTolerance
			|| this->terminationReason == E_TerminationReason::AbsoluteGap
			|| this->terminationReason == E_TerminationReason::RelativeGap)
	{
		solverStatus = "1";
	}
	else
	{
		solverStatus = "10";
		this->logger.message(0) << "ERROR: Unknown return code from solver" << CoinMessageEol;
	}

	ss << modelStatus << ",";
	ss << solverStatus << ",";
	ss << std::setprecision(std::numeric_limits<double>::digits10 + 1);
//ss << this->getCurrentIteration()->objectiveValue << ",";
	ss << this->getPrimalBound() << ",";
	;
	ss << this->getDualBound() << ",";
	;
	ss << this->getElapsedTime("Total") << ",";
	ss << iterFeasMILP + iterOptMILP << ",";
	ss << "0" << ",";
	ss << "0" << ",";
	ss << "#";

	return (ss.str());
}

void ProcessInfo::createIteration()
{
	Iteration iter = Iteration();
	iter.iterationNumber = iterations.size() + 1;

	iter.numHyperplanesAdded = 0;

	if (iterations.size() == 0) iter.totNumHyperplanes = 0;
	else iter.totNumHyperplanes = iterations.at(iterations.size() - 1).totNumHyperplanes;

	iter.maxDeviation = DBL_MAX;
	iter.boundaryDistance = DBL_MAX;
	iter.MILPSolutionLimitUpdated = false;

	iter.type = relaxationStrategy->getProblemType();
	//getCurrentIteration()->type = relaxationStrategy->getProblemType();

	iterations.push_back(iter);

}

Iteration *ProcessInfo::getCurrentIteration()
{
	return (&iterations.back());
}

Iteration *ProcessInfo::getPreviousIteration()
{
	if (iterations.size() > 1) return (&(iterations[iterations.size() - 2]));
	else throw ErrorClass("Only one iteration!");
}

double ProcessInfo::getPrimalBound()
{
	auto primalBound = this->currentObjectiveBounds.second;

	return (primalBound);
}

double ProcessInfo::getDualBound()
{
	auto dualBound = this->currentObjectiveBounds.first;

	return (dualBound);
}

double ProcessInfo::getAbsoluteObjectiveGap()
{
	double gap = abs(getDualBound() - getPrimalBound());

	return (gap);
}

double ProcessInfo::getRelativeObjectiveGap()
{
	double gap = abs(getDualBound() - getPrimalBound()) / ((1e-10) + abs(getPrimalBound()));

	return (gap);
}
