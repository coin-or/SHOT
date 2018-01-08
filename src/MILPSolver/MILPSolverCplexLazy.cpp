#include <MILPSolverCplexLazy.h>
#include "IMILPSolver.h"
//#include "ilcplex/cplex.h"
ILOSTLBEGIN

std::mutex cplexMutex2;
std::mutex addMutex2;

HCallbackI2::HCallbackI2(IloEnv env, IloNumVarArray xx2) :
		IloCplex::HeuristicCallbackI(env), cplexVars(xx2)
{
}

IloCplex::CallbackI* HCallbackI2::duplicateCallback() const
{
	return (new (getEnv()) HCallbackI2(*this));
}

IloCplex::Callback HCallback2(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) HCallbackI2(env, cplexVars)));
}

// This callback injects the best known primal solution into all threads.
void HCallbackI2::main() // Called at each node...
{
	cplexMutex2.lock();

	bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

	if ((ProcessInfo::getInstance().primalSolutions.size() > 0)
			&& ((isMinimization && this->getIncumbentObjValue() > ProcessInfo::getInstance().getPrimalBound())
					|| (!isMinimization && this->getIncumbentObjValue() < ProcessInfo::getInstance().getPrimalBound())))
	{
		auto primalSol = ProcessInfo::getInstance().primalSolution;

		IloNumArray tmpVals(this->getEnv());

		std::vector<double> solution(tmpVals.getSize());

		for (int i = 0; i < primalSol.size(); i++)
		{
			tmpVals.add(primalSol.at(i));
		}

		setSolution(cplexVars, tmpVals);
	}

	cplexMutex2.unlock();
	return;
}

InfoCallbackI2::InfoCallbackI2(IloEnv env, IloNumVarArray xx2) :
		IloCplex::MIPInfoCallbackI(env), cplexVars(xx2)
{
}

IloCplex::CallbackI* InfoCallbackI2::duplicateCallback() const
{
	return (new (getEnv()) InfoCallbackI2(*this));
}

IloCplex::Callback InfoCallback2(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) InfoCallbackI2(env, cplexVars)));
}

// This callback injects the best known primal solution into all threads.
void InfoCallbackI2::main() // Called at each node...
{
	cplexMutex2.lock();

	bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

	auto absObjGap = ProcessInfo::getInstance().getAbsoluteObjectiveGap();
	auto relObjGap = ProcessInfo::getInstance().getRelativeObjectiveGap();

	auto relMIPGap = this->getMIPRelativeGap();

	if (abs(relMIPGap) < Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		ProcessInfo::getInstance().outputAlways(
				"     Terminated by relative MIP gap tolerance in info callback: "
						+ UtilityFunctions::toString(relMIPGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));

		cplexMutex2.unlock();
		this->abort();
	}
	else if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
				"     Terminated by relative objective gap tolerance in info callback: "
						+ UtilityFunctions::toString(relObjGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));

		cplexMutex2.unlock();
		this->abort();
	}
	else if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
				"     Terminated by absolute objective gap tolerance in info callback: "
						+ UtilityFunctions::toString(absObjGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));

		cplexMutex2.unlock();
		this->abort();
	}

	cplexMutex2.unlock();
	return;
}

IncCallbackI2::IncCallbackI2(IloEnv env, IloNumVarArray xx2) :
		IloCplex::IncumbentCallbackI(env), cplexVars(xx2)
{
}

IloCplex::CallbackI* IncCallbackI2::duplicateCallback() const
{
	return (new (getEnv()) IncCallbackI2(*this));
}

IloCplex::Callback IncCallback2(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) IncCallbackI2(env, cplexVars)));
}

// This callback is called whenever the lazy callback does not cut away the solution
void IncCallbackI2::main()
{
	cplexMutex2.lock();

	IloNumArray tmpVals(this->getEnv());

	if (!this->hasIncumbent())
	{
		cplexMutex2.unlock();
		return;
	}

	this->getValues(tmpVals, cplexVars);

	std::vector<double> solution(tmpVals.getSize());

	for (int i = 0; i < tmpVals.getSize(); i++)
	{
		solution.at(i) = tmpVals[i];
	}

	auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

	auto relMIPGap = this->getMIPRelativeGap();

	if (abs(relMIPGap) < Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		cplexMutex2.unlock();
		return;
	}

	if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		cplexMutex2.unlock();
		return;
	}

	if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		cplexMutex2.unlock();
		return;
	}

	if (mostDevConstr.value <= Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		cplexMutex2.unlock();
		return;
	}

	reject();
	cplexMutex2.unlock();
}

CtCallbackI2::CtCallbackI2(IloEnv env, IloNumVarArray xx2, MILPSolverCplexLazy *solver) :
		IloCplex::LazyConstraintCallbackI(env), cplexVars(xx2), cplexSolver(solver)
{
	std::lock_guard < std::mutex > lock(cplexMutex2);

	ProcessInfo::getInstance().lastLazyAddedIter = 0;
	isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();
	cbCalls = 0;

	auto taskInitLinesearch = new TaskInitializeLinesearch();

	if (static_cast<ES_HyperplanePointStrategy>(Settings::getInstance().getIntSetting("HyperplanePointStrategy",
			"Algorithm")) == ES_HyperplanePointStrategy::ESH)
	{
		if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
				"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
		{
			taskSelectHPPts = new TaskSelectHyperplanePointsLinesearch();
		}
		else
		{
			taskSelectHPPts = new TaskSelectHyperplanePointsIndividualLinesearch();
		}
	}
	else
	{
		taskSelectHPPts = new TaskSelectHyperplanePointsSolution();
	}

	tSelectPrimNLP = new TaskSelectPrimalCandidatesFromNLP();

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear()
			&& Settings::getInstance().getBoolSetting("UseObjectiveLinesearch", "PrimalBound"))
	{
		taskUpdateObjectiveByLinesearch = new TaskUpdateNonlinearObjectiveByLinesearch();
	}

	cplexMutex2.unlock();
}

IloCplex::CallbackI * CtCallbackI2::duplicateCallback() const
{
	return (new (getEnv()) CtCallbackI2(*this));
}

IloCplex::Callback CtCallback2(IloEnv env, IloNumVarArray cplexVars, MILPSolverCplexLazy *cplexSolver)
{
	return (IloCplex::Callback(new (env) CtCallbackI2(env, cplexVars, cplexSolver)));
}

bool CtCallbackI2::checkAbsoluteObjectiveGapToleranceMet(SolutionPoint point)
{
	if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
				"     Absolute objective gap tolerance met: "
						+ UtilityFunctions::toString(ProcessInfo::getInstance().getAbsoluteObjectiveGap()) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));
		return (true);
	}
	return (false);
}

bool CtCallbackI2::checkRelativeObjectiveGapToleranceMet(SolutionPoint point)
{
	if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
				"     Relative objective gap tolerance met: "
						+ UtilityFunctions::toString(ProcessInfo::getInstance().getRelativeObjectiveGap()) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));
		return (true);
	}
	return (false);
}

bool CtCallbackI2::checkRelativeMIPGapToleranceMet(SolutionPoint point)
{
	auto relMIPGap = this->getMIPRelativeGap();

	if (abs(relMIPGap) < Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		ProcessInfo::getInstance().outputAlways(
				"     Relative MIP gap tolerance met: " + UtilityFunctions::toString(relMIPGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));
		return (true);
	}
	return (false);
}

void CtCallbackI2::main()
{
	cplexMutex2.lock();

	lastNumAddedHyperplanes = 0;
	this->cbCalls++;
	(static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->currIter++;
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	IloNumArray tmpVals(this->getEnv());

	this->getValues(tmpVals, cplexVars);

	std::vector<double> solution(tmpVals.getSize());

	for (int i = 0; i < tmpVals.getSize(); i++)
	{
		solution.at(i) = tmpVals[i];
	}

	auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

	double tmpDualObjBound = this->getBestObjValue();

	SolutionPoint tmpSolPt;

	tmpSolPt.point = solution;
	tmpSolPt.objectiveValue = getObjValue();
	tmpSolPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
	tmpSolPt.maxDeviation = mostDevConstr;

	// Check if better dual bound
	if ((isMinimization && tmpDualObjBound > ProcessInfo::getInstance().getDualBound())
			|| (!isMinimization && tmpDualObjBound < ProcessInfo::getInstance().getDualBound()))
	{
		DualSolution sol =
		{ solution, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound, currIter->iterationNumber };
		ProcessInfo::getInstance().addDualSolutionCandidate(sol);
	}

	if (this->hasIncumbent())
	{
		double tmpPrimalObjBound = this->getIncumbentObjValue();

		if ((isMinimization && tmpPrimalObjBound < ProcessInfo::getInstance().getPrimalBound())
				|| (!isMinimization && tmpPrimalObjBound > ProcessInfo::getInstance().getPrimalBound()))
		{

			IloNumArray tmpPrimalVals(this->getEnv());

			this->getIncumbentValues(tmpPrimalVals, cplexVars);

			std::vector<double> primalSolution(tmpPrimalVals.getSize());

			for (int i = 0; i < tmpPrimalVals.getSize(); i++)
			{
				primalSolution.at(i) = tmpPrimalVals[i];
			}

			SolutionPoint tmpPt;
			tmpPt.iterFound = currIter->iterationNumber;
			tmpPt.maxDeviation = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(primalSolution);
			tmpPt.objectiveValue = this->getIncumbentObjValue();
			tmpPt.point = primalSolution;

			ProcessInfo::getInstance().addPrimalSolutionCandidate(tmpPt,
					E_PrimalSolutionSource::LazyConstraintCallback);
		}
	}

	if (this->checkRelativeMIPGapToleranceMet(tmpSolPt))
	{
		printIterationReport(tmpSolPt);
		cplexMutex2.unlock();
		return;
	}

	if (this->checkRelativeObjectiveGapToleranceMet(tmpSolPt))
	{
		printIterationReport(tmpSolPt);
		cplexMutex2.unlock();
		return;
	}

	if (this->checkAbsoluteObjectiveGapToleranceMet(tmpSolPt))
	{
		printIterationReport(tmpSolPt);
		cplexMutex2.unlock();
		return;
	}

	std::vector < SolutionPoint > solutionPoints(1);

	solutionPoints.at(0) = tmpSolPt;

	if (static_cast<ES_HyperplanePointStrategy>(Settings::getInstance().getIntSetting("HyperplanePointStrategy",
			"Algorithm")) == ES_HyperplanePointStrategy::ESH)
	{
		if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
				"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
		{
			static_cast<TaskSelectHyperplanePointsLinesearch*>(taskSelectHPPts)->run(solutionPoints);
		}
		else
		{
			static_cast<TaskSelectHyperplanePointsIndividualLinesearch*>(taskSelectHPPts)->run(solutionPoints);
		}
	}
	else
	{
		static_cast<TaskSelectHyperplanePointsSolution*>(taskSelectHPPts)->run(solutionPoints);
	}

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear()
			&& Settings::getInstance().getBoolSetting("UseObjectiveLinesearch", "PrimalBound"))
	{
		taskUpdateObjectiveByLinesearch->updateObjectiveInPoint(solutionPoints.at(0));
	}

	std::function<IloConstraint(IloRange range)> callback2 = [&](IloRange range) -> IloConstraint
	{
		auto tmp = add(range, IloCplex::CutManagement::UseCutForce);
		return (tmp);
	};

	for (auto hp : ProcessInfo::getInstance().hyperplaneWaitingList)
	{

		this->createHyperplane(hp);

		this->lastNumAddedHyperplanes++;
	}

	ProcessInfo::getInstance().hyperplaneWaitingList.clear();

	/*
	 if (Settings::getInstance().getBoolSetting("AddIntegerCuts", "Algorithm"))
	 {
	 bool addedIntegerCut = false;

	 for (auto ic : ProcessInfo::getInstance().integerCutWaitingList)
	 {
	 cplexSolver->createIntegerCut(ic, callback2);
	 addedIntegerCut = true;
	 }

	 if (addedIntegerCut)
	 {
	 ProcessInfo::getInstance().outputAlways(
	 "     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size())
	 + " integer cut(s).                                        ");
	 }

	 ProcessInfo::getInstance().integerCutWaitingList.clear();
	 }*/
	printIterationReport(tmpSolPt);
	cplexMutex2.unlock();
}

void CtCallbackI2::createHyperplane(Hyperplane hyperplane)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration
	auto optional = ProcessInfo::getInstance().MILPSolver->createHyperplaneTerms(hyperplane);

	if (!optional)
	{
		return;
	}

	auto tmpPair = optional.get();

	//auto tmpPair = createHyperplaneTerms(hyperplane);
	bool hyperplaneIsOk = true;

	for (auto E : tmpPair.first)
	{
		if (E.value != E.value) //Check for NaN
		{
			ProcessInfo::getInstance().outputWarning(
					"     Warning: hyperplane not generated, NaN found in linear terms!");
			hyperplaneIsOk = false;
			break;
		}
	}

	if (hyperplaneIsOk)
	{
		GeneratedHyperplane genHyperplane;

		IloExpr expr(this->getEnv());

		for (int i = 0; i < tmpPair.first.size(); i++)
		{
			expr += tmpPair.first.at(i).value * cplexVars[tmpPair.first.at(i).idx];
		}

		IloRange tmpRange(this->getEnv(), -IloInfinity, expr, -tmpPair.second);

		auto addedConstr = add(tmpRange);

		int constrIndex = 0;
		genHyperplane.generatedConstraintIndex = constrIndex;
		genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
		genHyperplane.generatedPoint = hyperplane.generatedPoint;
		genHyperplane.source = hyperplane.source;
		genHyperplane.generatedIter = currIter->iterationNumber;
		genHyperplane.isLazy = false;
		genHyperplane.isRemoved = false;

		//ProcessInfo::getInstance().MILPSolver->generatedHyperplanes.push_back(genHyperplane);

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;
		expr.end();
	}
}

bool CtCallbackI2::checkFixedNLPStrategy(SolutionPoint point)
{
	if (!Settings::getInstance().getBoolSetting("PrimalStrategyFixedNLP", "PrimalBound"))
	{
		return (false);
	}

	ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
	ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");

	bool callNLPSolver = false;

	auto userSettingStrategy = Settings::getInstance().getIntSetting("NLPFixedStrategy", "PrimalBound");
	auto userSetting = Settings::getInstance().getIntSetting("NLPFixedSource", "PrimalBound");

	if (abs(point.objectiveValue - ProcessInfo::getInstance().getDualBound()) < 0.1)
	{
		callNLPSolver = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
	{
		callNLPSolver = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime)
			|| userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		if (ProcessInfo::getInstance().itersMILPWithoutNLPCall
				>= Settings::getInstance().getIntSetting("NLPFixedMaxIters", "PrimalBound"))
		{
			ProcessInfo::getInstance().outputInfo(
					"     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
			callNLPSolver = true;
		}
		else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().solTimeLastNLPCall
				> Settings::getInstance().getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
		{
			ProcessInfo::getInstance().outputInfo(
					"     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
			callNLPSolver = true;
		}
	}

	if (!callNLPSolver)
	{
		ProcessInfo::getInstance().itersMILPWithoutNLPCall++;
	}

	ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
	ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");

	return (callNLPSolver);
}

MILPSolverCplexLazy::MILPSolverCplexLazy()
{

	discreteVariablesActivated = true;

	cplexModel = IloModel(cplexEnv);

	cplexVars = IloNumVarArray(cplexEnv);
	cplexConstrs = IloRangeArray(cplexEnv);
	cplexLazyConstrs = IloRangeArray(cplexEnv);

//itersSinceNLPCall = 0;

	cachedSolutionHasChanged = true;
	isVariablesFixed = false;

	checkParameters();
	modelUpdated = false;
}

MILPSolverCplexLazy::~MILPSolverCplexLazy()
{
	cplexEnv.end();
}

void MILPSolverCplexLazy::initializeSolverSettings()
{
	try
	{
		MILPSolverCplex::initializeSolverSettings();
		cplexInstance.use(IncCallback2(cplexEnv, cplexVars));
		cplexInstance.use(CtCallback2(cplexEnv, cplexVars, this));
		cplexInstance.use(HCallback2(cplexEnv, cplexVars));
		cplexInstance.use(InfoCallback2(cplexEnv, cplexVars));
	}
	catch (IloException& e)
	{
		ProcessInfo::getInstance().outputError("CPLEX error when initializing parameters for linear solver",
				e.getMessage());
	}
}

E_ProblemSolutionStatus MILPSolverCplexLazy::solveProblem()
{
	MILPSolverCplex::startTimer();

	E_ProblemSolutionStatus MILPSolutionStatus;
	MILPSolverCplex::cachedSolutionHasChanged = true;

	try
	{
		if (modelUpdated)
		{
			//Extract the model if we have updated the constraints
			cplexInstance.extract(cplexModel);
		}

		double timeStart = ProcessInfo::getInstance().getElapsedTime("Total");

		cplexInstance.solve();
		double timeEnd = ProcessInfo::getInstance().getElapsedTime("Total");

		iterDurations.push_back(timeEnd - timeStart);
		MILPSolutionStatus = MILPSolverCplex::getSolutionStatus();

	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when solving MILP/LP problem", e.getMessage());
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	MILPSolverBase::stopTimer();

	return (MILPSolutionStatus);
}

int MILPSolverCplexLazy::increaseSolutionLimit(int increment)
{
	int sollim;

	try
	{
		cplexInstance.setParam(IloCplex::IntSolLim, cplexInstance.getParam(cplexInstance.IntSolLim) + increment);
		sollim = cplexInstance.getParam(cplexInstance.IntSolLim);
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when increasing solution limit", e.getMessage());
	}

	return (sollim);

}

void MILPSolverCplexLazy::setSolutionLimit(long limit)
{
	if (MILPSolverBase::originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
	{
		limit = Settings::getInstance().getIntSetting("MILPSolLimitInitial", "MILP");
	}

	try
	{
		cplexInstance.setParam(IloCplex::IntSolLim, limit);
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when setting solution limit", e.getMessage());
	}
}

int MILPSolverCplexLazy::getSolutionLimit()
{
	int solLim = 0;

	try
	{
		solLim = cplexInstance.getParam(cplexInstance.IntSolLim);
	}
	catch (IloException &e)
	{

		ProcessInfo::getInstance().outputError("Error when obtaining solution limit", e.getMessage());

	}

	return (solLim);
}

void MILPSolverCplexLazy::checkParameters()
{

}

void CtCallbackI2::printIterationReport(SolutionPoint solution)
{
	auto MILPSolver = static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver);
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (MILPSolver->currIter - MILPSolver->lastHeaderIter > 100)
	{
		MILPSolver->lastHeaderIter = MILPSolver->currIter;
		ProcessInfo::getInstance().tasks->getTask("PrintIterHeader")->run();
	}

	std::stringstream tmpType;
	tmpType << "LazyCB (" << this->getMyThreadNum() << ")";

	std::string hyperplanesExpr;

	if (this->lastNumAddedHyperplanes > 0)
	{
		hyperplanesExpr = "+" + to_string(this->lastNumAddedHyperplanes) + " = "
				+ to_string(currIter->totNumHyperplanes);
	}
	else
	{
		hyperplanesExpr = " ";
	}

	auto tmpConstrExpr = UtilityFunctions::toStringFormat(solution.maxDeviation.value, "%.5f");

	if (solution.maxDeviation.idx != -1)
	{
		tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames()[solution.maxDeviation.idx]
				+ ": " + tmpConstrExpr;
	}
	else
	{
		tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": " + tmpConstrExpr;
	}

	std::string primalBoundExpr;
	std::string dualBoundExpr;
	std::string objExpr;

	if (this->hasIncumbent())
	{
		primalBoundExpr = UtilityFunctions::toString(this->getIncumbentObjValue());
	}

	objExpr = UtilityFunctions::toString(solution.objectiveValue);
	dualBoundExpr = UtilityFunctions::toString(this->getBestObjValue());

	auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % to_string(cbCalls)
			% tmpType.str() % hyperplanesExpr % dualBoundExpr % objExpr % primalBoundExpr % tmpConstrExpr;

	ProcessInfo::getInstance().outputSummary(tmpLine.str());

	double timeStamp = ProcessInfo::getInstance().getElapsedTime("Total");

	if (MILPSolver->currIter - MILPSolver->lastSummaryIter > 50 || timeStamp - MILPSolver->lastSummaryTimeStamp > 5)
	{
		MILPSolver->lastSummaryIter = MILPSolver->currIter;
		MILPSolver->lastSummaryTimeStamp = timeStamp;
		double absGap = ProcessInfo::getInstance().getAbsoluteObjectiveGap();
		double relGap = ProcessInfo::getInstance().getRelativeObjectiveGap();
		auto objBounds = ProcessInfo::getInstance().getCorrectedObjectiveBounds();
		double objLB = objBounds.first;
		double objUB = objBounds.second;

		ProcessInfo::getInstance().outputSummary(
				"                                                                                     ");

#ifdef _WIN32
		ProcessInfo::getInstance().outputSummary(
				"ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
		ProcessInfo::getInstance().outputSummary(
				"─────────────────────────────────────────────────────────────────────────────────────");
#endif

		auto tmpLineSummary = boost::format(" At %1% s the obj. bound is %|24t|[%2%, %3%] %|46t|with abs/"
				"rel gap %4% / %5%") % ProcessInfo::getInstance().getElapsedTime("Total") % objLB % objUB % absGap
				% relGap;

		ProcessInfo::getInstance().outputSummary(tmpLineSummary.str());

		std::stringstream tmpLine;

		tmpLine << " Number of open nodes: " << this->getNremainingNodes() << ".";

		if (ProcessInfo::getInstance().interiorPts.size() > 1)
		{
			tmpLine << " Number of interior points: " << ProcessInfo::getInstance().interiorPts.size() << ".";
		}

		if (ProcessInfo::getInstance().numIntegerCutsAdded > 1)
		{
			tmpLine << " Number of integer cuts: " << ProcessInfo::getInstance().numIntegerCutsAdded << ".";
		}

		ProcessInfo::getInstance().outputSummary(tmpLine.str());

#ifdef _WIN32
		ProcessInfo::getInstance().outputSummary(
				"ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ");
#else
		ProcessInfo::getInstance().outputSummary(
				"─────────────────────────────────────────────────────────────────────────────────────");
#endif

		ProcessInfo::getInstance().outputSummary("");
	}
}

