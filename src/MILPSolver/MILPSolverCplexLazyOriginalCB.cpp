#include <MILPSolverCplexLazyOriginalCB.h>
#include "IMILPSolver.h"
//#include "ilcplex/cplex.h"
ILOSTLBEGIN

HCallbackI::HCallbackI(IloEnv env, IloNumVarArray xx2) :
		IloCplex::HeuristicCallbackI(env), cplexVars(xx2)
{
		std::lock_guard < std::mutex> lock((static_cast<MILPSolverCplexLazyOriginalCB*>(ProcessInfo::getInstance().MILPSolver))->callbackMutex2);

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
}

IloCplex::CallbackI* HCallbackI::duplicateCallback() const
{
	return (new (getEnv()) HCallbackI(*this));
}

IloCplex::Callback HCallback(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) HCallbackI(env, cplexVars)));
}

// This callback injects the best known primal solution into all threads.
void HCallbackI::main() // Called at each node...
{
	std::lock_guard < std::mutex> lock((static_cast<MILPSolverCplexLazyOriginalCB*>(ProcessInfo::getInstance().MILPSolver))->callbackMutex2);

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

		tmpVals.end();
	}

	if (Settings::getInstance().getBoolSetting("AddHyperplanesForRelaxedLazySolutions", "Algorithm"))
	{
		std::vector < SolutionPoint > solutionPoints(1);

		IloNumArray tmpVals(getEnv());

		getValues(tmpVals, cplexVars);

		std::vector<double> solution(tmpVals.getSize());

		for (int i = 0; i < tmpVals.getSize(); i++)
		{
			solution.at(i) = tmpVals[i];
		}

		tmpVals.end();

		auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

		SolutionPoint tmpSolPt;

		tmpSolPt.point = solution;
		tmpSolPt.objectiveValue = getObjValue();
		tmpSolPt.iterFound = 0;
		tmpSolPt.maxDeviation = mostDevConstr;

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
	}

	return;
}

InfoCallbackI::InfoCallbackI(IloEnv env, IloNumVarArray xx2) :
		IloCplex::MIPInfoCallbackI(env), cplexVars(xx2)
{
}

IloCplex::CallbackI* InfoCallbackI::duplicateCallback() const
{
	return (new (getEnv()) InfoCallbackI(*this));
}

IloCplex::Callback InfoCallback(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) InfoCallbackI(env, cplexVars)));
}

void InfoCallbackI::main() // Called at each node...
{
	std::lock_guard < std::mutex> lock((static_cast<MILPSolverCplexLazyOriginalCB*>(ProcessInfo::getInstance().MILPSolver))->callbackMutex2);

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

		this->abort();
		return;
	}
	else if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
				"     Terminated by relative objective gap tolerance in info callback: "
						+ UtilityFunctions::toString(relObjGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));

		this->abort();
		return;
	}
	else if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
				"     Terminated by absolute objective gap tolerance in info callback: "
						+ UtilityFunctions::toString(absObjGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));

		this->abort();
		return;
	}
	else if (checkIterationLimit())
	{
		ProcessInfo::getInstance().outputAlways("     Terminated since iteration limit reached in info callback.");

		this->abort();
		return;
	}

	return;
}

IncCallbackI::IncCallbackI(IloEnv env, IloNumVarArray xx2) :
		IloCplex::IncumbentCallbackI(env), cplexVars(xx2)
{
}

IloCplex::CallbackI* IncCallbackI::duplicateCallback() const
{
	return (new (getEnv()) IncCallbackI(*this));
}

IloCplex::Callback IncCallback(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) IncCallbackI(env, cplexVars)));
}

// This callback is called whenever the lazy callback does not cut away the solution
void IncCallbackI::main()
{
	std::lock_guard < std::mutex> lock((static_cast<MILPSolverCplexLazyOriginalCB*>(ProcessInfo::getInstance().MILPSolver))->callbackMutex2);

	IloNumArray tmpVals(this->getEnv());

	if (!this->hasIncumbent())
	{
		return;
	}

	this->getValues(tmpVals, cplexVars);

	std::vector<double> solution(tmpVals.getSize());

	for (int i = 0; i < tmpVals.getSize(); i++)
	{
		solution.at(i) = tmpVals[i];
	}

	tmpVals.end();

	auto relMIPGap = this->getMIPRelativeGap();

	if (abs(relMIPGap) < Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		return;
	}

	if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet() || ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		return;
	}

	auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

	if (mostDevConstr.value <= Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return;
	}
	
	reject();
}

CtCallbackI::CtCallbackI(IloEnv env, IloNumVarArray xx2, MILPSolverCplexLazyOriginalCB *solver) :
		IloCplex::LazyConstraintCallbackI(env), cplexVars(xx2), cplexSolver(solver)
{
	std::lock_guard < std::mutex> lock((static_cast<MILPSolverCplexLazyOriginalCB*>(ProcessInfo::getInstance().MILPSolver))->callbackMutex2);

	ProcessInfo::getInstance().lastLazyAddedIter = 0;
	isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();
	cbCalls = 0;

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

	if (Settings::getInstance().getBoolSetting("PrimalStrategyLinesearch", "PrimalBound"))
	{
		taskSelectPrimalSolutionFromLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
	}
}

IloCplex::CallbackI * CtCallbackI::duplicateCallback() const
{
	return (new (getEnv()) CtCallbackI(*this));
}

IloCplex::Callback CtCallback(IloEnv env, IloNumVarArray cplexVars, MILPSolverCplexLazyOriginalCB *cplexSolver)
{
	return (IloCplex::Callback(new (env) CtCallbackI(env, cplexVars, cplexSolver)));
}

void CtCallbackI::main()
{
	std::lock_guard < std::mutex> lock((static_cast<MILPSolverCplexLazyOriginalCB*>(ProcessInfo::getInstance().MILPSolver))->callbackMutex2);

	lastNumAddedHyperplanes = 0;
	this->cbCalls++;
	
	IloNumArray tmpVals(this->getEnv());

	this->getValues(tmpVals, cplexVars);

	std::vector<double> solution(tmpVals.getSize());

	for (int i = 0; i < tmpVals.getSize(); i++)
	{
		solution.at(i) = tmpVals[i];
	}

	tmpVals.end();

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
		{ solution, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound, ProcessInfo::getInstance().getCurrentIteration()->iterationNumber};
		ProcessInfo::getInstance().addDualSolutionCandidate(sol);
	}

	// Check if better primal solution
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
			tmpPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
			tmpPt.maxDeviation = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(primalSolution);
			tmpPt.objectiveValue = this->getIncumbentObjValue();
			tmpPt.point = primalSolution;

			ProcessInfo::getInstance().addPrimalSolutionCandidate(tmpPt,
					E_PrimalSolutionSource::LazyConstraintCallback);
		}
	}

	if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet() || ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet() || checkIterationLimit())
	{
		abort();
		return;
	}
	
	ProcessInfo::getInstance().createIteration();
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	std::vector < SolutionPoint > candidatePoints(1);

	candidatePoints.at(0) = tmpSolPt;

	if (Settings::getInstance().getBoolSetting("PrimalStrategyLinesearch", "PrimalBound"))
	{
		taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
	}

	if (checkFixedNLPStrategy(tmpSolPt))
	{
		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution,
				this->getObjValue(), ProcessInfo::getInstance().getCurrentIteration()->iterationNumber,
				tmpSolPt.maxDeviation);

		tSelectPrimNLP->run();

		ProcessInfo::getInstance().checkPrimalSolutionCandidates();

		if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet() || ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
		{
			abort();
			return;
		}
	}

	if (static_cast<ES_HyperplanePointStrategy>(Settings::getInstance().getIntSetting("HyperplanePointStrategy",
			"Algorithm")) == ES_HyperplanePointStrategy::ESH)
	{
		if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
				"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
		{
			static_cast<TaskSelectHyperplanePointsLinesearch*>(taskSelectHPPts)->run(candidatePoints);
		}
		else
		{
			static_cast<TaskSelectHyperplanePointsIndividualLinesearch*>(taskSelectHPPts)->run(candidatePoints);
		}
	}
	else
	{
		static_cast<TaskSelectHyperplanePointsSolution*>(taskSelectHPPts)->run(candidatePoints);
	}

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear()
			&& Settings::getInstance().getBoolSetting("UseObjectiveLinesearch", "PrimalBound"))
	{
		taskUpdateObjectiveByLinesearch->updateObjectiveInPoint(candidatePoints.at(0));
	}

	for (auto hp : ProcessInfo::getInstance().hyperplaneWaitingList)
	{
		this->createHyperplane(hp);

		this->lastNumAddedHyperplanes++;
	}

	ProcessInfo::getInstance().hyperplaneWaitingList.clear();

	if (Settings::getInstance().getBoolSetting("AddIntegerCuts", "Algorithm"))
	{
		bool addedIntegerCut = false;

		for (auto ic : ProcessInfo::getInstance().integerCutWaitingList)
		{
			this->createIntegerCut(ic);
			addedIntegerCut = true;
		}

		if (addedIntegerCut)
		{
			ProcessInfo::getInstance().outputAlways(
					"     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size())
							+ " integer cut(s).                                        ");
		}

		ProcessInfo::getInstance().integerCutWaitingList.clear();
	}

	auto bestBound = UtilityFunctions::toStringFormat(this->getBestObjValue(), "%.3f", true);
	auto threadId = to_string(this->getMyThreadNum());
	auto openNodes = to_string(this->getNremainingNodes());
		
	printIterationReport(candidatePoints.at(0), threadId, bestBound, openNodes);
}

void CtCallbackI::createHyperplane(Hyperplane hyperplane)
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

void CtCallbackI::createIntegerCut(std::vector<int> binaryIndexes)
{
	IloExpr expr(this->getEnv());

	for (int i = 0; i < binaryIndexes.size(); i++)
	{
		expr += 1.0 * cplexVars[binaryIndexes.at(i)];
	}

	IloRange tmpRange(this->getEnv(), -IloInfinity, expr, binaryIndexes.size() - 1.0);

	add(tmpRange);
	ProcessInfo::getInstance().numIntegerCutsAdded++;

	expr.end();
}

MILPSolverCplexLazyOriginalCB::MILPSolverCplexLazyOriginalCB()
{

	discreteVariablesActivated = true;

	cplexModel = IloModel(cplexEnv);

	cplexVars = IloNumVarArray(cplexEnv);
	cplexConstrs = IloRangeArray(cplexEnv);
	cplexLazyConstrs = IloRangeArray(cplexEnv);

	cachedSolutionHasChanged = true;
	isVariablesFixed = false;

	checkParameters();
	modelUpdated = false;
}

MILPSolverCplexLazyOriginalCB::~MILPSolverCplexLazyOriginalCB()
{
	cplexEnv.end();
}

void MILPSolverCplexLazyOriginalCB::initializeSolverSettings()
{
	try
	{
		MILPSolverCplex::initializeSolverSettings();
		cplexInstance.use(IncCallback(cplexEnv, cplexVars));
		cplexInstance.use(CtCallback(cplexEnv, cplexVars, this));
		cplexInstance.use(HCallback(cplexEnv, cplexVars));
		cplexInstance.use(InfoCallback(cplexEnv, cplexVars));
	}
	catch (IloException& e)
	{
		ProcessInfo::getInstance().outputError("CPLEX error when initializing parameters for linear solver",
				e.getMessage());
	}
}

E_ProblemSolutionStatus MILPSolverCplexLazyOriginalCB::solveProblem()
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

int MILPSolverCplexLazyOriginalCB::increaseSolutionLimit(int increment)
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

void MILPSolverCplexLazyOriginalCB::setSolutionLimit(long limit)
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

int MILPSolverCplexLazyOriginalCB::getSolutionLimit()
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

void MILPSolverCplexLazyOriginalCB::checkParameters()
{

}
