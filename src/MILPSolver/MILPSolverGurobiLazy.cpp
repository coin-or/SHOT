#include "MILPSolverGurobiLazy.h"

MILPSolverGurobiLazy::MILPSolverGurobiLazy()
{
	discreteVariablesActivated = true;

	gurobiEnv = new GRBEnv();
	gurobiModel = new GRBModel(*gurobiEnv);

	cachedSolutionHasChanged = true;
	isVariablesFixed = false;

	checkParameters();

}

MILPSolverGurobiLazy::~MILPSolverGurobiLazy()
{
	delete gurobiEnv;
	delete gurobiModel;
}

void MILPSolverGurobiLazy::initializeSolverSettings()
{
	MILPSolverGurobi::initializeSolverSettings();

	try
	{
		gurobiModel->set(GRB_IntParam_LazyConstraints, 1);
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when initializing parameters for linear solver", e.getMessage());
	}
}

int MILPSolverGurobiLazy::increaseSolutionLimit(int increment)
{
	gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit,
			gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MILPSolverGurobiLazy::setSolutionLimit(long limit)
{
	if (limit > GRB_MAXINT) gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
	else gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MILPSolverGurobiLazy::getSolutionLimit()
{
	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MILPSolverGurobiLazy::checkParameters()
{

}

E_ProblemSolutionStatus MILPSolverGurobiLazy::solveProblem()
{
	E_ProblemSolutionStatus MILPSolutionStatus;
	cachedSolutionHasChanged = true;

	try
	{
		GurobiCallback gurobiCallback = GurobiCallback(gurobiModel->getVars());
		gurobiModel->setCallback(&gurobiCallback);
		gurobiModel->optimize();

		MILPSolutionStatus = getSolutionStatus();
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when solving MILP/LP problem", e.getMessage());
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	return (MILPSolutionStatus);
}

void GurobiCallback::callback()
{
	if (where == GRB_CB_POLLING || where == GRB_CB_PRESOLVE || where == GRB_CB_SIMPLEX || where == GRB_CB_MESSAGE
			|| where == GRB_CB_BARRIER) return;

	try
	{
		int iterationNumber = (static_cast<MILPSolverGurobiLazy*>(ProcessInfo::getInstance().MILPSolver))->currIter;
		// Check if better dual bound
		double tmpDualObjBound;

		if (where == GRB_CB_MIP || where == GRB_CB_MIPSOL || where == GRB_CB_MIPNODE)
		{
			switch (where)
			{
				case GRB_CB_MIP:
					tmpDualObjBound = getDoubleInfo(GRB_CB_MIP_OBJBND);
					break;
				case GRB_CB_MIPSOL:
					tmpDualObjBound = getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
					break;
				case GRB_CB_MIPNODE:
					tmpDualObjBound = getDoubleInfo(GRB_CB_MIPNODE_OBJBND);
					break;
				default:
					break;
			}

			if ((isMinimization && tmpDualObjBound > ProcessInfo::getInstance().getDualBound())
					|| (!isMinimization && tmpDualObjBound < ProcessInfo::getInstance().getDualBound()))
			{
				std::vector<double> doubleSolution; // Empty since we have no point

				DualSolution sol =
				{ doubleSolution, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound, iterationNumber };
				ProcessInfo::getInstance().addDualSolutionCandidate(sol);
			}
		}

		if (where == GRB_CB_MIPSOL)
		{
			// Check for new primal solution
			double tmpPrimalObjBound = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

			if ((tmpPrimalObjBound < 1e100)
					&& ((isMinimization && tmpPrimalObjBound < ProcessInfo::getInstance().getPrimalBound())
							|| (!isMinimization && tmpPrimalObjBound > ProcessInfo::getInstance().getPrimalBound())))
			{
				std::vector<double> primalSolution(numVar);

				for (int i = 0; i < numVar; i++)
				{
					primalSolution.at(i) = getSolution(vars[i]);
				}

				SolutionPoint tmpPt;
				tmpPt.iterFound = iterationNumber;
				tmpPt.maxDeviation = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
						primalSolution);
				tmpPt.objectiveValue = ProcessInfo::getInstance().originalProblem->calculateOriginalObjectiveValue(
						primalSolution);
				tmpPt.point = primalSolution;

				ProcessInfo::getInstance().addPrimalSolutionCandidate(tmpPt,
						E_PrimalSolutionSource::LazyConstraintCallback);
			}
		}

		if (checkRelativeObjectiveGapToleranceMet() || checkAbsoluteObjectiveGapToleranceMet())
		{
			abort();
			return;
		}

		if (where == GRB_CB_MIPNODE && getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL)
		{
			if (Settings::getInstance().getBoolSetting("AddHyperplanesForRelaxedLazySolutions", "Algorithm"))
			{
				std::vector < SolutionPoint > solutionPoints(1);

				std::vector<double> solution(numVar);

				for (int i = 0; i < numVar; i++)
				{
					solution.at(i) = getNodeRel(vars[i]);
				}

				auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

				SolutionPoint tmpSolPt;

				tmpSolPt.point = solution;
				tmpSolPt.objectiveValue = ProcessInfo::getInstance().originalProblem->calculateOriginalObjectiveValue(
						solution);
				tmpSolPt.iterFound = iterationNumber;
				tmpSolPt.maxDeviation = mostDevConstr;

				solutionPoints.at(0) = tmpSolPt;

				if (static_cast<ES_HyperplanePointStrategy>(Settings::getInstance().getIntSetting(
						"HyperplanePointStrategy", "Algorithm")) == ES_HyperplanePointStrategy::ESH)
				{
					if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
							"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
					{
						static_cast<TaskSelectHyperplanePointsLinesearch*>(taskSelectHPPts)->run(solutionPoints);
					}
					else
					{
						static_cast<TaskSelectHyperplanePointsIndividualLinesearch*>(taskSelectHPPts)->run(
								solutionPoints);
					}
				}
				else
				{
					static_cast<TaskSelectHyperplanePointsSolution*>(taskSelectHPPts)->run(solutionPoints);
				}
			}
		}

		if (where == GRB_CB_MIPSOL)
		{
			std::vector<double> solution(numVar);

			for (int i = 0; i < numVar; i++)
			{
				solution.at(i) = getSolution(vars[i]);
			}

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);
			SolutionPoint solutionCandidate;

			solutionCandidate.point = solution;
			solutionCandidate.objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
			solutionCandidate.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
			solutionCandidate.maxDeviation = mostDevConstr;

			std::vector < SolutionPoint > candidatePoints(1);
			candidatePoints.at(0) = solutionCandidate;

			addLazyConstraint (candidatePoints);

			if (Settings::getInstance().getBoolSetting("PrimalStrategyLinesearch", "PrimalBound"))
			{
				taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
			}

			if (checkFixedNLPStrategy(candidatePoints.at(0)))
			{
				ProcessInfo::getInstance().addPrimalFixedNLPCandidate(candidatePoints.at(0).point,
						E_PrimalNLPSource::FirstSolution, getDoubleInfo(GRB_CB_MIPSOL_OBJ), iterationNumber,
						candidatePoints.at(0).maxDeviation);

				tSelectPrimNLP->run();

				ProcessInfo::getInstance().checkPrimalSolutionCandidates();
			}

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
					ProcessInfo::getInstance().outputInfo(
							"     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size())
									+ " integer cut(s).                                        ");
				}

				ProcessInfo::getInstance().integerCutWaitingList.clear();
			}

			printIterationReport(candidatePoints.at(0));

			if (checkRelativeObjectiveGapToleranceMet() || checkAbsoluteObjectiveGapToleranceMet())
			{
				abort();
				return;
			}
		}

		if (where == GRB_CB_MIPSOL)
		{
			// Add current primal bound as new incumbent candidate
			auto primalBound = ProcessInfo::getInstance().getPrimalBound();

			if (((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && primalBound > primalBound)))
			{

				auto primalSol = ProcessInfo::getInstance().primalSolution;

				std::vector<double> primalSolution(numVar);

				for (int i = 0; i < numVar; i++)
				{
					setSolution(vars[i], primalSol.at(i));
				}

				lastUpdatedPrimal = primalBound;
			}

			// Adds cutoff
			if (isMinimization)
			{
				static_cast<MILPSolverGurobiLazy*>(ProcessInfo::getInstance().MILPSolver)->gurobiModel->set(
						GRB_DoubleParam_Cutoff, primalBound /*+ 0.0000001*/);

				ProcessInfo::getInstance().outputInfo(
						"     Setting cutoff value to " + UtilityFunctions::toString(primalBound /*+ 0.0000001*/)
								+ " for minimization.");
			}
			else
			{
				static_cast<MILPSolverGurobiLazy*>(ProcessInfo::getInstance().MILPSolver)->gurobiModel->set(
						GRB_DoubleParam_Cutoff, primalBound /*- 0.0000001*/);

				ProcessInfo::getInstance().outputInfo(
						"     Setting cutoff value to " + UtilityFunctions::toString(primalBound /*- 0.0000001*/)
								+ " for minimization.");
			}
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Gurobi error when running main callback method", e.getMessage());
	}
}

bool GurobiCallback::checkFixedNLPStrategy(SolutionPoint point)
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

void GurobiCallback::printIterationReport(SolutionPoint solution)
{
	try
	{
		auto MILPSolver = static_cast<MILPSolverGurobiLazy*>(ProcessInfo::getInstance().MILPSolver);
		auto currIter = ProcessInfo::getInstance().getCurrentIteration();

		if (MILPSolver->currIter - MILPSolver->lastHeaderIter > 100)
		{
			MILPSolver->lastHeaderIter = MILPSolver->currIter;
			ProcessInfo::getInstance().tasks->getTask("PrintIterHeader")->run();
		}

		std::stringstream tmpType;
		tmpType << "LazyCB"/* << context.getIntInfo(IloCplex::Callback::Context::Info::ThreadId) */<< "";

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
			tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": "
					+ tmpConstrExpr;
		}

		std::string primalBoundExpr;
		std::string dualBoundExpr;
		std::string objExpr;

//if (context.isCandidatePoint())
//{
		primalBoundExpr = UtilityFunctions::toStringFormat(ProcessInfo::getInstance().getPrimalBound(), "%.3f", true);
//}

		objExpr = UtilityFunctions::toStringFormat(solution.objectiveValue, "%.3f", true);
		dualBoundExpr = UtilityFunctions::toStringFormat(getDoubleInfo(GRB_CB_MIPSOL_OBJBND), "%.3f", true);

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
					"rel gap %4% / %5%") % ProcessInfo::getInstance().getElapsedTime("Total")
					% UtilityFunctions::toStringFormat(objLB, "%.3f", true)
					% UtilityFunctions::toStringFormat(objUB, "%.3f", true)
					% UtilityFunctions::toStringFormat(absGap, "%.4f", true)
					% UtilityFunctions::toStringFormat(relGap, "%.4f", true);

			ProcessInfo::getInstance().outputSummary(tmpLineSummary.str());

			std::stringstream tmpLine;

			tmpLine << " Number of open nodes: ";
			//tmpLine << 	(static_cast<MILPSolverGurobiLazy*>(ProcessInfo::getInstance().MILPSolver))->gurobiModel->get(
			//		GRB_Int_NOD);
			//<< getIntInfo(GRB_CB_MIP_NODLFT) << ".";

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
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Gurobi error when printing iteration report", e.getMessage());
	}
}

bool GurobiCallback::checkAbsoluteObjectiveGapToleranceMet()
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

bool GurobiCallback::checkRelativeObjectiveGapToleranceMet()
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

void GurobiCallback::createHyperplane(Hyperplane hyperplane)
{
	try
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

			GRBLinExpr expr = 0;

			for (int i = 0; i < tmpPair.first.size(); i++)
			{
				expr += +(tmpPair.first.at(i).value) * (vars[tmpPair.first.at(i).idx]);
			}

			addLazy(expr <= -tmpPair.second);

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
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Gurobi error when creating lazy hyperplane", e.getMessage());
	}
}

GurobiCallback::GurobiCallback(GRBVar * xvars)
{
	vars = xvars;

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

	if (Settings::getInstance().getBoolSetting("PrimalStrategyLinesearch", "PrimalBound"))
	{
		taskSelectPrimalSolutionFromLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
	}

	lastUpdatedPrimal = ProcessInfo::getInstance().getPrimalBound();

	numVar = (static_cast<MILPSolverGurobiLazy*>(ProcessInfo::getInstance().MILPSolver))->gurobiModel->get(
			GRB_IntAttr_NumVars);

}

void GurobiCallback::createIntegerCut(std::vector<int> binaryIndexes)
{
	try
	{
		GRBLinExpr expr = 0;

		for (int i = 0; i < binaryIndexes.size(); i++)
		{
			expr += vars[binaryIndexes.at(i)];
		}

		addLazy(expr <= binaryIndexes.size() - 1.0);

		ProcessInfo::getInstance().numIntegerCutsAdded++;
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Gurobi error when adding lazy integer cut", e.getMessage());
	}
}

void GurobiCallback::addLazyConstraint(std::vector<SolutionPoint> candidatePoints)
{
	try
	{
		lastNumAddedHyperplanes = 0;
		this->cbCalls++;
		(static_cast<MILPSolverGurobiLazy*>(ProcessInfo::getInstance().MILPSolver))->currIter++;

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

		for (auto hp : ProcessInfo::getInstance().hyperplaneWaitingList)
		{
			this->createHyperplane(hp);
			this->lastNumAddedHyperplanes++;
		}

		ProcessInfo::getInstance().hyperplaneWaitingList.clear();
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Gurobi error when invoking adding lazy constraint", e.getMessage());
	}
}
