#include <MILPSolverCplexLazy.h>
#include "IMILPSolver.h"
//#include "ilcplex/cplex.h"
ILOSTLBEGIN

CplexCallback::CplexCallback(const IloNumVarArray& vars)
{
	std::lock_guard < std::mutex
			> lock((static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->cplexMutex);

	cplexVars = vars;

	isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

	ProcessInfo::getInstance().lastLazyAddedIter = 0;

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

	lastUpdatedPrimal = ProcessInfo::getInstance().getPrimalBound();
}

/*
 void CplexCallback::heuristicCallback(const IloCplex::Callback::Context& context)
 {
 return;
 if (ProcessInfo::getInstance().primalSolutions.size() == 0) return;

 auto cplexPrimalBound = context.getIncumbentObjective();

 auto primalBound = ProcessInfo::getInstance().getPrimalBound();

 //std::cout << "cplex: " << cplexPrimalBound << " shot: " << primalBound << std::endl;

 if (((isMinimization && primalBound < cplexPrimalBound) || (!isMinimization && primalBound > cplexPrimalBound)))
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

 //std::cout << "incumbent: " << context.getIncumbentObjective() << " pb: " << primalBound << std::endl;

 tmpVals.end();

 if (isMinimization)
 {

 (static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->cplexInstance.setParam(
 IloCplex::CutUp, primalBound);

 ProcessInfo::getInstance().outputInfo(
 "     Setting cutoff value to " + to_string(primalBound) + " for minimization.");
 }
 else
 {
 (static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->cplexInstance.setParam(
 IloCplex::CutLo, primalBound);

 ProcessInfo::getInstance().outputInfo(
 "     Setting cutoff value to " + to_string(primalBound) + " for maximization.");
 }

 }

 else
 {
 //std::cout << "no incumbent: " << context.getIncumbentObjective() << " pb: " << primalBound << std::endl;

 }
 }*/

void CplexCallback::lazyConstraintCallback(const IloCplex::Callback::Context& context)
{
	if (!context.isCandidatePoint()) throw IloCplex::Exception(-1, "Unbounded solution");

	try
	{

		lastNumAddedHyperplanes = 0;
		this->cbCalls++;
		(static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->currIter++;
//auto currIter = ProcessInfo::getInstance().getCurrentIteration();

		IloNumArray tmpVals(context.getEnv());

		context.getCandidatePoint(cplexVars, tmpVals);

		std::vector<double> solution(tmpVals.getSize());

		for (int i = 0; i < tmpVals.getSize(); i++)
		{
			solution.at(i) = tmpVals[i];
		}

		auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);
		double tmpDualObjBound = context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);
		SolutionPoint tmpSolPt;

		tmpSolPt.point = solution;
		tmpSolPt.objectiveValue = context.getCandidateObjective();
		tmpSolPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
		tmpSolPt.maxDeviation = mostDevConstr;

// Check if better dual bound
		if ((isMinimization && tmpDualObjBound > ProcessInfo::getInstance().getDualBound())
				|| (!isMinimization && tmpDualObjBound < ProcessInfo::getInstance().getDualBound()))
		{
			DualSolution sol =
			{ solution, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound, 0 };
			ProcessInfo::getInstance().addDualSolutionCandidate(sol);
		}

		double tmpPrimalObjBound = context.getIncumbentObjective();

		if ((tmpPrimalObjBound < 10e74)
				&& ((isMinimization && tmpPrimalObjBound < ProcessInfo::getInstance().getPrimalBound())
						|| (!isMinimization && tmpPrimalObjBound > ProcessInfo::getInstance().getPrimalBound())))
		{
			IloNumArray tmpPrimalVals(context.getEnv());

			context.getIncumbent(cplexVars, tmpPrimalVals);

			std::vector<double> primalSolution(tmpPrimalVals.getSize());

			for (int i = 0; i < tmpPrimalVals.getSize(); i++)
			{
				primalSolution.at(i) = tmpPrimalVals[i];
			}

			SolutionPoint tmpPt;
			tmpPt.iterFound = 0;
			tmpPt.maxDeviation = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(primalSolution);
			tmpPt.objectiveValue = ProcessInfo::getInstance().originalProblem->calculateOriginalObjectiveValue(
					primalSolution);
			tmpPt.point = primalSolution;

			ProcessInfo::getInstance().addPrimalSolutionCandidate(tmpPt,
					E_PrimalSolutionSource::LazyConstraintCallback);
		}

		/*if (this->checkRelativeMIPGapToleranceMet(tmpSolPt, context))
		 {
		 printIterationReport(tmpSolPt, context);
		 cplexMutex2.unlock();
		 abort();
		 return;
		 }*/

		if (this->checkRelativeObjectiveGapToleranceMet(tmpSolPt, context))
		{
			printIterationReport(tmpSolPt, context);
			context.abort();
			return;
		}

		if (this->checkAbsoluteObjectiveGapToleranceMet(tmpSolPt, context))
		{
			printIterationReport(tmpSolPt, context);
			context.abort();
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

		if (checkFixedNLPStrategy(tmpSolPt, context))

		{
			ProcessInfo::getInstance().addPrimalFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution,
					context.getCandidateObjective(), 0, tmpSolPt.maxDeviation);

			tSelectPrimNLP->run();

			ProcessInfo::getInstance().checkPrimalSolutionCandidates();

			if (checkRelativeObjectiveGapToleranceMet(tmpSolPt, context))
			{
				printIterationReport(tmpSolPt, context);
				return;
			}

			if (checkAbsoluteObjectiveGapToleranceMet(tmpSolPt, context))
			{
				printIterationReport(tmpSolPt, context);
				return;
			}
		}

		/*
		 std::function<IloConstraint(IloRange range)> callback2 = [&](IloRange range) -> IloConstraint
		 {
		 auto tmp = context.rejectCandidate(range);
		 return (tmp);
		 };*/

		for (auto hp : ProcessInfo::getInstance().hyperplaneWaitingList)
		{
			this->createHyperplane(hp, context);

			this->lastNumAddedHyperplanes++;
		}

		ProcessInfo::getInstance().hyperplaneWaitingList.clear();

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
						"     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size())
								+ " integer cut(s).                                        ");
			}

			ProcessInfo::getInstance().integerCutWaitingList.clear();
		}

		printIterationReport(tmpSolPt, context);

	}
	catch (IloException& e)
	{
		ProcessInfo::getInstance().outputError("CPLEX error when invoking general lazy callback", e.getMessage());
	}
}

void CplexCallback::invoke(const IloCplex::Callback::Context &context)
{
	std::lock_guard < std::mutex
			> lock((static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->cplexMutex);

	try
	{
		auto absObjGap = ProcessInfo::getInstance().getAbsoluteObjectiveGap();
		auto relObjGap = ProcessInfo::getInstance().getRelativeObjectiveGap();

		/*
		 auto relMIPGap = this->getMIPRelativeGap();

		 if (abs(relMIPGap) < Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
		 {
		 ProcessInfo::getInstance().outputAlways(
		 "     Terminated by relative MIP gap tolerance in info callback: "
		 + UtilityFunctions::toString(relMIPGap) + " < "
		 + UtilityFunctions::toString(
		 Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));

		 cplexMutex.unlock();
		 this->abort();
		 }
		 else
		 */
		if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
		{
			ProcessInfo::getInstance().outputAlways(
					"     Terminated by relative objective gap tolerance in info callback: "
							+ UtilityFunctions::toString(relObjGap) + " < "
							+ UtilityFunctions::toString(
									Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));

			context.abort();
			return;
		}
		else if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
		{
			ProcessInfo::getInstance().outputAlways(
					"     Terminated by absolute objective gap tolerance in info callback: "
							+ UtilityFunctions::toString(absObjGap) + " < "
							+ UtilityFunctions::toString(
									Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));

			context.abort();
			return;
		}

		/*if (context.inRelaxation())
		 {
		 heuristicCallback(context);
		 }*/

		auto primalBound = ProcessInfo::getInstance().getPrimalBound();

		if (((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && primalBound > primalBound)))
		{
			auto cplexPrimalBound = context.getIncumbentObjective();

			auto primalSol = ProcessInfo::getInstance().primalSolution;

			IloNumArray tmpVals(context.getEnv());

			std::vector<double> solution(primalSol.size());

			for (int i = 0; i < primalSol.size(); i++)
			{
				tmpVals.add(primalSol.at(i));
			}

			context.postHeuristicSolution(cplexVars, tmpVals, primalBound,
					IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);

			//std::cout << "incumbent: " << context.getIncumbentObjective() << " pb: " << primalBound << std::endl;

			tmpVals.end();

			lastUpdatedPrimal = primalBound;
		}

		if (isMinimization)
		{

			(static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->cplexInstance.setParam(
					IloCplex::CutUp, primalBound);

			ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + to_string(primalBound) + " for minimization.");
		}
		else
		{
			(static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver))->cplexInstance.setParam(
					IloCplex::CutLo, primalBound);

			ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + to_string(primalBound) + " for maximization.");
		}

		//std::cout << "cplex: " << cplexPrimalBound << " shot: " << primalBound << std::endl;
		/*
		 if (((isMinimization && primalBound < cplexPrimalBound) || (!isMinimization && primalBound > cplexPrimalBound)))
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

		 //std::cout << "incumbent: " << context.getIncumbentObjective() << " pb: " << primalBound << std::endl;

		 tmpVals.end();

		 }
		 else
		 {
		 //std::cout << "no incumbent: " << context.getIncumbentObjective() << " pb: " << primalBound << std::endl;

		 }*/

		if (Settings::getInstance().getBoolSetting("AddHyperplanesForRelaxedLazySolutions", "Algorithm")
				&& context.inRelaxation())
		{
			//

			std::vector < SolutionPoint > solutionPoints(1);

			IloNumArray tmpVals(context.getEnv());

			context.getRelaxationPoint(cplexVars, tmpVals);

			std::vector<double> solution(tmpVals.getSize());

			for (int i = 0; i < tmpVals.getSize(); i++)
			{
				solution.at(i) = tmpVals[i];
			}

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

			SolutionPoint tmpSolPt;

			tmpSolPt.point = solution;
			tmpSolPt.objectiveValue = context.getRelaxationObjective();
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
		else if (context.inCandidate())
		{
			lazyConstraintCallback(context);
		}
	}
	catch (IloException& e)
	{
		ProcessInfo::getInstance().outputError("CPLEX error when invoking general callback", e.getMessage());
	}
}

/// Destructor
CplexCallback::~CplexCallback()
{
//cuts.endElements();
}

void CplexCallback::createHyperplane(Hyperplane hyperplane, const IloCplex::Callback::Context& context)
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
		genHyperplane.isLazy = false;
		genHyperplane.isRemoved = false;

		//ProcessInfo::getInstance().MILPSolver->generatedHyperplanes.push_back(genHyperplane);

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;
		expr.end();
	}
}

void CplexCallback::createIntegerCut(std::vector<int> binaryIndexes, const IloCplex::Callback::Context& context)
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

bool CplexCallback::checkFixedNLPStrategy(SolutionPoint point, const IloCplex::Callback::Context& context)
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

void CplexCallback::printIterationReport(SolutionPoint solution, const IloCplex::Callback::Context& context)
{
	auto MILPSolver = static_cast<MILPSolverCplexLazy*>(ProcessInfo::getInstance().MILPSolver);
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (MILPSolver->currIter - MILPSolver->lastHeaderIter > 100)
	{
		MILPSolver->lastHeaderIter = MILPSolver->currIter;
		ProcessInfo::getInstance().tasks->getTask("PrintIterHeader")->run();
	}

	std::stringstream tmpType;
	tmpType << "LazyCB (" << context.getIntInfo(IloCplex::Callback::Context::Info::ThreadId) << ")";

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

//if (context.isCandidatePoint())
//{
	primalBoundExpr = UtilityFunctions::toStringFormat(
			context.getDoubleInfo(IloCplex::Callback::Context::Info::BestSolution), "%.3f", true);
//}

	objExpr = UtilityFunctions::toStringFormat(solution.objectiveValue, "%.3f", true);
	dualBoundExpr = UtilityFunctions::toStringFormat(
			context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound), "%.3f", true);

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

bool CplexCallback::checkAbsoluteObjectiveGapToleranceMet(SolutionPoint point,
		const IloCplex::Callback::Context& context)
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

bool CplexCallback::checkRelativeObjectiveGapToleranceMet(SolutionPoint point,
		const IloCplex::Callback::Context& context)
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

		CplexCallback cCallback(cplexVars);
		CPXLONG contextMask = 0;

		contextMask |= IloCplex::Callback::Context::Id::Candidate;
		contextMask |= IloCplex::Callback::Context::Id::Relaxation;

		// If contextMask is not zero we add the callback.
		if (contextMask != 0) cplexInstance.use(&cCallback, contextMask);

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
