#include <MILPSolverCplexLazy.h>
#include "IMILPSolver.h"
//#include "ilcplex/cplex.h"
ILOSTLBEGIN

CplexCallback::CplexCallback(const IloNumVarArray &vars)
{
	std::lock_guard<std::mutex> lock((static_cast<MILPSolverCplexLazy *>(ProcessInfo::getInstance().MILPSolver))->cplexMutex);

	cplexVars = vars;

	isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

	ProcessInfo::getInstance().lastLazyAddedIter = 0;

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

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear() && Settings::getInstance().getBoolSetting("UseObjectiveLinesearch", "PrimalBound"))
	{
		taskUpdateObjectiveByLinesearch = new TaskUpdateNonlinearObjectiveByLinesearch();
	}

	if (Settings::getInstance().getBoolSetting("PrimalStrategyLinesearch", "PrimalBound"))
	{
		taskSelectPrimalSolutionFromLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
	}

	lastUpdatedPrimal = ProcessInfo::getInstance().getPrimalBound();
}

void CplexCallback::invoke(const IloCplex::Callback::Context &context)
{
	std::lock_guard<std::mutex> lock((static_cast<MILPSolverCplexLazy *>(ProcessInfo::getInstance().MILPSolver))->cplexMutex);

	int iterationNumber = (static_cast<MILPSolverCplexLazy *>(ProcessInfo::getInstance().MILPSolver))->currIter;

	try
	{
		// Check if better dual bound
		double tmpDualObjBound = context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);

		if ((isMinimization && tmpDualObjBound > ProcessInfo::getInstance().getDualBound()) || (!isMinimization && tmpDualObjBound < ProcessInfo::getInstance().getDualBound()))
		{
			std::vector<double> doubleSolution; // Empty since we have no point

			DualSolution sol =
				{doubleSolution, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound, iterationNumber};
			ProcessInfo::getInstance().addDualSolutionCandidate(sol);
		}

		// Check for new primal solution
		double tmpPrimalObjBound = context.getIncumbentObjective();

		if ((tmpPrimalObjBound < 1e74) && ((isMinimization && tmpPrimalObjBound < ProcessInfo::getInstance().getPrimalBound()) || (!isMinimization && tmpPrimalObjBound > ProcessInfo::getInstance().getPrimalBound())))
		{
			IloNumArray tmpPrimalVals(context.getEnv());

			context.getIncumbent(cplexVars, tmpPrimalVals);

			std::vector<double> primalSolution(tmpPrimalVals.getSize());

			for (int i = 0; i < tmpPrimalVals.getSize(); i++)
			{
				primalSolution.at(i) = tmpPrimalVals[i];
			}

			SolutionPoint tmpPt;
			tmpPt.iterFound = iterationNumber;
			tmpPt.maxDeviation = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(primalSolution);
			tmpPt.objectiveValue = ProcessInfo::getInstance().originalProblem->calculateOriginalObjectiveValue(
				primalSolution);
			tmpPt.point = primalSolution;

			ProcessInfo::getInstance().addPrimalSolutionCandidate(tmpPt,
																  E_PrimalSolutionSource::LazyConstraintCallback);

			tmpPrimalVals.end();
		}

		if (checkRelativeObjectiveGapToleranceMet(context) || checkAbsoluteObjectiveGapToleranceMet(context))
		{
			abort();
			return;
		}

		if (context.inRelaxation())
		{
			if (Settings::getInstance().getBoolSetting("AddHyperplanesForRelaxedLazySolutions", "Algorithm"))
			{
				std::vector<SolutionPoint> solutionPoints(1);

				IloNumArray tmpVals(context.getEnv());

				context.getRelaxationPoint(cplexVars, tmpVals);

				std::vector<double> solution(tmpVals.getSize());

				for (int i = 0; i < tmpVals.getSize(); i++)
				{
					solution.at(i) = tmpVals[i];
				}

				tmpVals.end();

				auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

				SolutionPoint tmpSolPt;

				tmpSolPt.point = solution;
				tmpSolPt.objectiveValue = context.getRelaxationObjective();
				tmpSolPt.iterFound = iterationNumber;
				tmpSolPt.maxDeviation = mostDevConstr;

				solutionPoints.at(0) = tmpSolPt;

				if (static_cast<ES_HyperplanePointStrategy>(Settings::getInstance().getIntSetting(
						"HyperplanePointStrategy", "Algorithm")) == ES_HyperplanePointStrategy::ESH)
				{
					if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
							"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
					{
						static_cast<TaskSelectHyperplanePointsLinesearch *>(taskSelectHPPts)->run(solutionPoints);
					}
					else
					{
						static_cast<TaskSelectHyperplanePointsIndividualLinesearch *>(taskSelectHPPts)->run(solutionPoints);
					}
				}
				else
				{
					static_cast<TaskSelectHyperplanePointsSolution *>(taskSelectHPPts)->run(solutionPoints);
				}
			}
		}

		if (context.inCandidate())
		{
			ProcessInfo::getInstance().createIteration();

			auto currIter = ProcessInfo::getInstance().getCurrentIteration();

			IloNumArray tmpVals(context.getEnv());

			context.getCandidatePoint(cplexVars, tmpVals);

			std::vector<double> solution(tmpVals.getSize());

			for (int i = 0; i < tmpVals.getSize(); i++)
			{
				solution.at(i) = tmpVals[i];
			}

			tmpVals.end();

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);
			SolutionPoint solutionCandidate;

			solutionCandidate.point = solution;
			solutionCandidate.objectiveValue = context.getCandidateObjective();
			solutionCandidate.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
			solutionCandidate.maxDeviation = mostDevConstr;

			std::vector<SolutionPoint> candidatePoints(1);
			candidatePoints.at(0) = solutionCandidate;

			addLazyConstraint(candidatePoints, context);

			currIter->maxDeviation = mostDevConstr.value;
			currIter->maxDeviationConstraint = mostDevConstr.idx;

			currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;

			currIter->objectiveValue = context.getCandidateObjective();
			auto bounds = std::make_pair(ProcessInfo::getInstance().getDualBound(), ProcessInfo::getInstance().getPrimalBound());
			currIter->currentObjectiveBounds = bounds;

			if (Settings::getInstance().getBoolSetting("PrimalStrategyLinesearch", "PrimalBound"))
			{
				taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
			}

			if (checkFixedNLPStrategy(candidatePoints.at(0)))
			{
				ProcessInfo::getInstance().addPrimalFixedNLPCandidate(candidatePoints.at(0).point,
																	  E_PrimalNLPSource::FirstSolution, context.getCandidateObjective(), iterationNumber,
																	  candidatePoints.at(0).maxDeviation);

				tSelectPrimNLP->run();

				ProcessInfo::getInstance().checkPrimalSolutionCandidates();
			}
	
			if (Settings::getInstance().getBoolSetting("AddIntegerCuts", "Algorithm"))
			{
				bool addedIntegerCut = false;

				for (auto ic : ProcessInfo::getInstance().integerCutWaitingList)
				{
					this->createIntegerCut(ic, context);
					addedIntegerCut = true;
				}

				if (addedIntegerCut)
				{
					ProcessInfo::getInstance().outputInfo(
						"     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size()) + " integer cut(s).                                        ");
				}

				ProcessInfo::getInstance().integerCutWaitingList.clear();
			}

			printIterationReport(candidatePoints.at(0), context);

			if (checkRelativeObjectiveGapToleranceMet(context) || checkAbsoluteObjectiveGapToleranceMet(context))
			{
				abort();
				return;
			}
		}

		// Add current primal bound as new incumbent candidate
		auto primalBound = ProcessInfo::getInstance().getPrimalBound();

		if (((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && primalBound > primalBound)))
		{
			auto primalSol = ProcessInfo::getInstance().primalSolution;

			IloNumArray tmpVals(context.getEnv());

			std::vector<double> solution(primalSol.size());

			for (int i = 0; i < primalSol.size(); i++)
			{
				tmpVals.add(primalSol.at(i));
			}

			context.postHeuristicSolution(cplexVars, tmpVals, primalBound,
										  IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);

			tmpVals.end();

			lastUpdatedPrimal = primalBound;
		}

		// Adds cutoff
		if (isMinimization)
		{
			(static_cast<MILPSolverCplexLazy *>(ProcessInfo::getInstance().MILPSolver))->cplexInstance.setParam(IloCplex::CutUp, primalBound);

			ProcessInfo::getInstance().outputInfo(
				"     Setting cutoff value to " + to_string(primalBound) + " for minimization.");
		}
		else
		{
			(static_cast<MILPSolverCplexLazy *>(ProcessInfo::getInstance().MILPSolver))->cplexInstance.setParam(IloCplex::CutLo, primalBound);

			ProcessInfo::getInstance().outputInfo(
				"     Setting cutoff value to " + to_string(primalBound) + " for maximization.");
		}
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("CPLEX error when invoking general callback", e.getMessage());
	}
}

/// Destructor
CplexCallback::~CplexCallback()
{
	//cuts.endElements();
}

void CplexCallback::createHyperplane(Hyperplane hyperplane, const IloCplex::Callback::Context &context)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration
	auto optionalHyperplanes = ProcessInfo::getInstance().MILPSolver->createHyperplaneTerms(hyperplane);

	if (!optionalHyperplanes)
	{
		return;
	}

	auto tmpPair = optionalHyperplanes.get();

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

		IloExpr expr(context.getEnv());

		for (int i = 0; i < tmpPair.first.size(); i++)
		{
			expr += tmpPair.first.at(i).value * cplexVars[tmpPair.first.at(i).idx];
		}

		IloRange tmpRange(context.getEnv(), -IloInfinity, expr, -tmpPair.second);

		auto addedConstr = context.rejectCandidate(tmpRange);

		int constrIndex = 0;
		genHyperplane.generatedConstraintIndex = constrIndex;
		genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
		genHyperplane.generatedPoint = hyperplane.generatedPoint;
		genHyperplane.source = hyperplane.source;
		genHyperplane.generatedIter = currIter->iterationNumber;
		genHyperplane.isLazy = true;
		genHyperplane.isRemoved = false;

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;
		expr.end();
	}
}

void CplexCallback::createIntegerCut(std::vector<int> binaryIndexes, const IloCplex::Callback::Context &context)
{
	IloExpr expr(context.getEnv());

	for (int i = 0; i < binaryIndexes.size(); i++)
	{
		expr += 1.0 * cplexVars[binaryIndexes.at(i)];
	}

	IloRange tmpRange(context.getEnv(), -IloInfinity, expr, binaryIndexes.size() - 1.0);

	context.rejectCandidate(tmpRange);
	ProcessInfo::getInstance().numIntegerCutsAdded++;

	expr.end();
}

bool CplexCallback::checkFixedNLPStrategy(SolutionPoint point)
{
	if (!Settings::getInstance().getBoolSetting("PrimalStrategyFixedNLP", "PrimalBound"))
	{
		ProcessInfo::getInstance().itersMILPWithoutNLPCall++;
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
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime) || userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		if (ProcessInfo::getInstance().itersMILPWithoutNLPCall >= Settings::getInstance().getIntSetting("NLPFixedMaxIters", "PrimalBound"))
		{
			ProcessInfo::getInstance().outputInfo(
				"     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
			callNLPSolver = true;
		}
		else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().solTimeLastNLPCall > Settings::getInstance().getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
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

void CplexCallback::printIterationReport(SolutionPoint solution, const IloCplex::Callback::Context &context)
{
	auto MILPSolver = static_cast<MILPSolverCplexLazy *>(ProcessInfo::getInstance().MILPSolver);
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->iterationNumber - MILPSolver->lastHeaderIter > 100)
	{
		MILPSolver->lastHeaderIter = MILPSolver->currIter;
		ProcessInfo::getInstance().tasks->getTask("PrintIterHeader")->run();
	}

	std::stringstream tmpType;
	tmpType << "LazyCB (" << context.getIntInfo(IloCplex::Callback::Context::Info::ThreadId) << ")";

	std::string hyperplanesExpr;

	if (this->lastNumAddedHyperplanes > 0)
	{
		hyperplanesExpr = "+" + to_string(this->lastNumAddedHyperplanes) + " = " + to_string(currIter->totNumHyperplanes);
	}
	else
	{
		hyperplanesExpr = " ";
	}

	auto tmpConstrExpr = UtilityFunctions::toStringFormat(solution.maxDeviation.value, "%.5f");

	if (solution.maxDeviation.idx != -1)
	{
		tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames()[solution.maxDeviation.idx] + ": " + tmpConstrExpr;
	}
	else
	{
		tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": " + tmpConstrExpr;
	}

	std::string primalBoundExpr;
	std::string dualBoundExpr;
	std::string objExpr;

	primalBoundExpr = UtilityFunctions::toStringFormat(
		ProcessInfo::getInstance().getPrimalBound(), "%.3f", true);

	objExpr = UtilityFunctions::toStringFormat(solution.objectiveValue, "%.3f", true);
	dualBoundExpr = UtilityFunctions::toStringFormat(
		context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound), "%.3f", true);

	auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % to_string(currIter->iterationNumber) % tmpType.str() % hyperplanesExpr % dualBoundExpr % objExpr % primalBoundExpr % tmpConstrExpr;

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
											"rel gap %4% / %5%") %
							  ProcessInfo::getInstance().getElapsedTime("Total") % UtilityFunctions::toStringFormat(objLB, "%.3f", true) % UtilityFunctions::toStringFormat(objUB, "%.3f", true) % UtilityFunctions::toStringFormat(absGap, "%.4f", true) % UtilityFunctions::toStringFormat(relGap, "%.4f", true);

		ProcessInfo::getInstance().outputSummary(tmpLineSummary.str());

		std::stringstream tmpLine;

		tmpLine << " Number of open nodes: " << context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount) << ".";

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

bool CplexCallback::checkAbsoluteObjectiveGapToleranceMet(const IloCplex::Callback::Context &context)
{
	if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
			"     Absolute objective gap tolerance met: " + UtilityFunctions::toString(ProcessInfo::getInstance().getAbsoluteObjectiveGap()) + " < " + UtilityFunctions::toString(Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));
		return (true);
	}
	return (false);
}

bool CplexCallback::checkRelativeObjectiveGapToleranceMet(const IloCplex::Callback::Context &context)
{
	if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputAlways(
			"     Relative objective gap tolerance met: " + UtilityFunctions::toString(ProcessInfo::getInstance().getRelativeObjectiveGap()) + " < " + UtilityFunctions::toString(Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));
		return (true);
	}
	return (false);
}

void CplexCallback::addLazyConstraint(std::vector<SolutionPoint> candidatePoints,
									  const IloCplex::Callback::Context &context)
{
	try
	{
		lastNumAddedHyperplanes = 0;
		ProcessInfo::getInstance().getCurrentIteration()->numHyperplanesAdded++;
		(static_cast<MILPSolverCplexLazy *>(ProcessInfo::getInstance().MILPSolver))->currIter++;

		if (static_cast<ES_HyperplanePointStrategy>(Settings::getInstance().getIntSetting("HyperplanePointStrategy",
																						  "Algorithm")) == ES_HyperplanePointStrategy::ESH)
		{
			if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
					"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
			{
				static_cast<TaskSelectHyperplanePointsLinesearch *>(taskSelectHPPts)->run(candidatePoints);
			}
			else
			{
				static_cast<TaskSelectHyperplanePointsIndividualLinesearch *>(taskSelectHPPts)->run(candidatePoints);
			}
		}
		else
		{
			static_cast<TaskSelectHyperplanePointsSolution *>(taskSelectHPPts)->run(candidatePoints);
		}

		for (auto hp : ProcessInfo::getInstance().hyperplaneWaitingList)
		{
			this->createHyperplane(hp, context);
			this->lastNumAddedHyperplanes++;
		}

		ProcessInfo::getInstance().hyperplaneWaitingList.clear();
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("CPLEX error when invoking general lazy callback", e.getMessage());
	}
}

MILPSolverCplexLazy::MILPSolverCplexLazy()
{

	discreteVariablesActivated = true;

	cplexModel = IloModel(cplexEnv);

	cplexVars = IloNumVarArray(cplexEnv);
	cplexConstrs = IloRangeArray(cplexEnv);
	//cplexLazyConstrs = IloRangeArray(cplexEnv);

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

		cplexInstance.setParam(IloCplex::NumericalEmphasis, 1);
	}
	catch (IloException &e)
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

		CplexCallback cCallback(cplexVars);
		CPXLONG contextMask = 0;

		contextMask |= IloCplex::Callback::Context::Id::Candidate;
		contextMask |= IloCplex::Callback::Context::Id::Relaxation;

		// If contextMask is not zero we add the callback.
		if (contextMask != 0)
			cplexInstance.use(&cCallback, contextMask);

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
