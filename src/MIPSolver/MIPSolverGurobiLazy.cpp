#include "MIPSolverGurobiLazy.h"

MIPSolverGurobiLazy::MIPSolverGurobiLazy()
{
	discreteVariablesActivated = true;

	gurobiEnv = new GRBEnv();
	gurobiModel = new GRBModel(*gurobiEnv);

	cachedSolutionHasChanged = true;
	isVariablesFixed = false;

	checkParameters();
}

MIPSolverGurobiLazy::~MIPSolverGurobiLazy()
{
	delete gurobiEnv;
	delete gurobiModel;
}

void MIPSolverGurobiLazy::initializeSolverSettings()
{
	MIPSolverGurobi::initializeSolverSettings();

	try
	{
		gurobiModel->set(GRB_IntParam_LazyConstraints, 1);
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when initializing parameters for linear solver", e.getMessage());
	}
}

int MIPSolverGurobiLazy::increaseSolutionLimit(int increment)
{
	gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit,
							  gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobiLazy::setSolutionLimit(long limit)
{
	if (limit > GRB_MAXINT)
		gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
	else
		gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobiLazy::getSolutionLimit()
{
	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobiLazy::checkParameters()
{
}

E_ProblemSolutionStatus MIPSolverGurobiLazy::solveProblem()
{
	E_ProblemSolutionStatus MIPSolutionStatus;
	cachedSolutionHasChanged = true;

	try
	{
		GurobiCallback gurobiCallback = GurobiCallback(gurobiModel->getVars());
		gurobiModel->setCallback(&gurobiCallback);
		gurobiModel->optimize();

		MIPSolutionStatus = getSolutionStatus();
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when solving MIP/LP problem", e.getMessage());
		MIPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	return (MIPSolutionStatus);
}

void GurobiCallback::callback()
{
	if (where == GRB_CB_POLLING || where == GRB_CB_PRESOLVE || where == GRB_CB_SIMPLEX || where == GRB_CB_MESSAGE || where == GRB_CB_BARRIER)
		return;

	try
	{
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

			if ((isMinimization && tmpDualObjBound > ProcessInfo::getInstance().getDualBound()) || (!isMinimization && tmpDualObjBound < ProcessInfo::getInstance().getDualBound()))
			{
				std::vector<double> doubleSolution; // Empty since we have no point

				DualSolution sol =
					{doubleSolution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound, ProcessInfo::getInstance().getCurrentIteration()->iterationNumber};
				ProcessInfo::getInstance().addDualSolutionCandidate(sol);
			}
		}

		if (where == GRB_CB_MIPSOL)
		{
			// Check for new primal solution
			double tmpPrimalObjBound = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

			if ((tmpPrimalObjBound < 1e100) && ((isMinimization && tmpPrimalObjBound < ProcessInfo::getInstance().getPrimalBound()) || (!isMinimization && tmpPrimalObjBound > ProcessInfo::getInstance().getPrimalBound())))
			{
				std::vector<double> primalSolution(numVar);

				for (int i = 0; i < numVar; i++)
				{
					primalSolution.at(i) = getSolution(vars[i]);
				}

				SolutionPoint tmpPt;
				tmpPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
				tmpPt.maxDeviation = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
					primalSolution);
				tmpPt.objectiveValue = ProcessInfo::getInstance().originalProblem->calculateOriginalObjectiveValue(
					primalSolution);
				tmpPt.point = primalSolution;

				ProcessInfo::getInstance().addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);
			}
		}

		if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet() || ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet() || checkIterationLimit())
		{
			abort();
			return;
		}

		if (where == GRB_CB_MIPNODE && getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL)
		{
			if (maxIntegerRelaxedHyperplanes < Settings::getInstance().getIntSetting("Relaxation.MaxLazyConstraints", "Dual"))
			{
				int waitingListSize = ProcessInfo::getInstance().hyperplaneWaitingList.size();
				std::vector<SolutionPoint> solutionPoints(1);

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
				tmpSolPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
				tmpSolPt.maxDeviation = mostDevConstr;

				solutionPoints.at(0) = tmpSolPt;

				if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting(
						"CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
				{
					if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting(
							"ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
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

				maxIntegerRelaxedHyperplanes += (ProcessInfo::getInstance().hyperplaneWaitingList.size() - waitingListSize);
			}
		}

		if (where == GRB_CB_MIPSOL)
		{
			ProcessInfo::getInstance().createIteration();
			auto currIter = ProcessInfo::getInstance().getCurrentIteration();

			std::vector<double> solution(numVar);

			for (int i = 0; i < numVar; i++)
			{
				solution.at(i) = getSolution(vars[i]);
			}

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

			//Remove??
			/*
			if (mostDevConstr.value <= Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
			{
				return;
			}*/

			SolutionPoint solutionCandidate;

			solutionCandidate.point = solution;
			solutionCandidate.objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
			solutionCandidate.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
			solutionCandidate.maxDeviation = mostDevConstr;

			std::vector<SolutionPoint> candidatePoints(1);
			candidatePoints.at(0) = solutionCandidate;

			addLazyConstraint(candidatePoints);

			currIter->maxDeviation = mostDevConstr.value;
			currIter->maxDeviationConstraint = mostDevConstr.idx;
			currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;
			currIter->objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

			auto bounds = std::make_pair(ProcessInfo::getInstance().getDualBound(), ProcessInfo::getInstance().getPrimalBound());
			currIter->currentObjectiveBounds = bounds;

			if (Settings::getInstance().getBoolSetting("Linesearch.Use", "Primal"))
			{
				taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
			}

			if (checkFixedNLPStrategy(candidatePoints.at(0)))
			{
				ProcessInfo::getInstance().addPrimalFixedNLPCandidate(candidatePoints.at(0).point,
																	  E_PrimalNLPSource::FirstSolution, getDoubleInfo(GRB_CB_MIPSOL_OBJ), ProcessInfo::getInstance().getCurrentIteration()->iterationNumber,
																	  candidatePoints.at(0).maxDeviation);

				tSelectPrimNLP->run();

				ProcessInfo::getInstance().checkPrimalSolutionCandidates();
			}

			if (Settings::getInstance().getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual"))
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
						"     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size()) + " integer cut(s).                                        ");
				}

				ProcessInfo::getInstance().integerCutWaitingList.clear();
			}

			auto bestBound = UtilityFunctions::toStringFormat(getDoubleInfo(GRB_CB_MIPSOL_OBJBND), "%.3f", true);
			auto threadId = "";
			auto openNodes = "?";

			printIterationReport(candidatePoints.at(0), threadId, bestBound, openNodes);

			if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet() || ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
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

			double cutOffTol = Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual");

			if (isMinimization)
			{
				static_cast<MIPSolverGurobiLazy *>(ProcessInfo::getInstance().MIPSolver)->gurobiModel->set(GRB_DoubleParam_Cutoff, primalBound + cutOffTol);

				ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + UtilityFunctions::toString(primalBound + cutOffTol) + " for minimization.");
			}
			else
			{
				static_cast<MIPSolverGurobiLazy *>(ProcessInfo::getInstance().MIPSolver)->gurobiModel->set(GRB_DoubleParam_Cutoff, -primalBound - cutOffTol);

				ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + UtilityFunctions::toString(-primalBound - cutOffTol) + " for minimization.");
			}
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Gurobi error when running main callback method", e.getMessage());
	}
}

void GurobiCallback::createHyperplane(Hyperplane hyperplane)
{
	try
	{
		auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration
		auto optional = ProcessInfo::getInstance().MIPSolver->createHyperplaneTerms(hyperplane);

		if (!optional)
		{
			return;
		}

		auto tmpPair = optional.get();

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

			//ProcessInfo::getInstance().MIPSolver->generatedHyperplanes.push_back(genHyperplane);

			currIter->numHyperplanesAdded++;
			currIter->totNumHyperplanes++;
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Gurobi error when creating lazy hyperplane", e.getMessage());
	}
}

GurobiCallback::GurobiCallback(GRBVar *xvars)
{
	vars = xvars;

	isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

	ProcessInfo::getInstance().lastLazyAddedIter = 0;

	cbCalls = 0;

	if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
	{
		tUpdateInteriorPoint = new TaskUpdateInteriorPoint();
		bUpdateInteriorPoint = true;

		if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting(
				"ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
		{
			taskSelectHPPts = new TaskSelectHyperplanePointsLinesearch();
			bSelectHPPts = true;
		}
		else
		{
			taskSelectHPPts = new TaskSelectHyperplanePointsIndividualLinesearch();
			bSelectHPPts = true;
		}
	}
	else
	{
		taskSelectHPPts = new TaskSelectHyperplanePointsSolution();
		bSelectHPPts = true;
	}

	tSelectPrimNLP = new TaskSelectPrimalCandidatesFromNLP();
	bSelectPrimNLP = true;

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear() && Settings::getInstance().getBoolSetting("ObjectiveLinesearch.Use", "Dual"))
	{
		taskUpdateObjectiveByLinesearch = new TaskUpdateNonlinearObjectiveByLinesearch();
		bUpdateObjectiveByLinesearch = true;
	}

	if (Settings::getInstance().getBoolSetting("Linesearch.Use", "Primal"))
	{
		taskSelectPrimalSolutionFromLinesearch = new TaskSelectPrimalCandidatesFromLinesearch();
		bSelectPrimalSolutionFromLinesearch = true;
	}

	lastUpdatedPrimal = ProcessInfo::getInstance().getPrimalBound();

	numVar = (static_cast<MIPSolverGurobiLazy *>(ProcessInfo::getInstance().MIPSolver))->gurobiModel->get(GRB_IntAttr_NumVars);
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

		if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
		{
			tUpdateInteriorPoint->run();

			if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting("ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
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
