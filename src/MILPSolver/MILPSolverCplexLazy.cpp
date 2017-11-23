#include <MILPSolverCplexLazy.h>
#include "IMILPSolver.h"
//#include "ilcplex/cplex.h"
ILOSTLBEGIN

//

/*void SolutionFilterCallbackI::main()
 {
 //create filter
 //std::cout << "calling incumbent callback" << std::endl;
 IloModel model = getModel();
 //cout << m << " " << n << endl;
 IloEnv env = getEnv();

 IloNumArray val(env, xVar.getSize());
 getValues(val, xVar);

 int numVar = originalProblem->getNumberOfVariables();

 std::vector<double> pt(numVar);

 for (int i = 0; i < numVar; i++)
 {
 pt.at(i) = val[i];
 }

 auto tmpVal = originalProblem->getMostDeviatingConstraint(pt).value;
 std::cout << "Callback value " << tmpVal << std::endl;
 if (ProcessInfo::getInstance().iterations.size() > 10 && tmpVal > 0)
 {
 std::cout << "Rejected! " << tmpVal << std::endl;
 reject();

 }
 //cout<<val<<endl;
 }

 //export callback
 IloCplex::Callback SolutionFilterCallback(IloEnv env, IloNumVarArray x, ProcessInfo *pInfo)
 {
 return (IloCplex::Callback(new (env) SolutionFilterCallbackI(env, x, pInfo)));
 }*/

std::vector<IloRange> cplexLazyConstrs;

int iterSinceLastIncumbent;
int itersSinceNLPCall;

double timestampLastNLPCall;
double lastIncumbentObjVal;

TaskBase *tSelectPrimNLP;

class HCallbackI: public IloCplex::HeuristicCallbackI
{
		IloNumVarArray cplexVars;

	private:

	public:
		IloCplex::CallbackI* duplicateCallback() const
		{
			return (new (getEnv()) HCallbackI(*this));
		}
		HCallbackI(IloEnv env, IloNumVarArray xx2) :
				IloCplex::HeuristicCallbackI(env), cplexVars(xx2)
		{
			//this->processInfo = pInfo;

		}
		void main();	// the call back function
};

IloCplex::Callback HCallback(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) HCallbackI(env, cplexVars)));
}

void HCallbackI::main() // Called at each node...
{

	if ((ProcessInfo::getInstance().primalSolutions.size() > 0)
			&& (this->getIncumbentObjValue() > ProcessInfo::getInstance().getPrimalBound()))
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

	return;
	/*
	 //std::cout << "HCallbackI" << std::endl;
	 if (this->hasIncumbent())
	 {
	 std::cout << "Has incumbent" << std::endl;
	 auto primalBound = ProcessInfo::getInstance().getPrimalBound();

	 bool isMinimization = originalProblem->isTypeOfObjectiveMinimize();

	 bool betterPrimalThanIncumbent = ((isMinimization && primalBound < this->getIncumbentObjValue())
	 || (!isMinimization && primalBound > this->getIncumbentObjValue()));

	 if (betterPrimalThanIncumbent)
	 {
	 auto primalSol = ProcessInfo::getInstance().primalSolution;

	 IloNumArray tmpVals(this->getEnv());

	 std::vector<double> solution(tmpVals.getSize());

	 for (int i = 0; i < primalSol.size(); i++)
	 {
	 tmpVals.add(primalSol.at(i));
	 }

	 std::cout << "Solution suggested: " << this->getIncumbentObjValue() << " -> " << primalBound << std::endl;

	 setSolution(cplexVars, tmpVals);

	 }
	 else
	 {
	 int numVar = cplexVars.getSize();
	 std::vector<double> solution(numVar);

	 IloNumArray tmpSolsCplex(this->getEnv());

	 try
	 {

	 this->getIncumbentValues(tmpSolsCplex, cplexVars);

	 for (int i = 0; i < numVar; i++)
	 {
	 solution.at(i) = tmpSolsCplex[i];
	 }

	 ProcessInfo::getInstance().addPrimalSolutionCandidate(solution, E_PrimalSolutionSource::LazyConstraintCallback,
	 ProcessInfo::getInstance().getCurrentIteration()->iterationNumber);

	 }
	 catch (IloException &e)
	 {
	 std::cout << "ERROR" << std::endl;
	 }

	 }
	 }
	 else //Added august 2016
	 {
	 this->std::cout << "Has no incumbent" << std::endl;
	 if (ProcessInfo::getInstance().primalSolution.size() == 0) return;

	 auto primalSol = ProcessInfo::getInstance().primalSolution;

	 IloNumArray tmpVals(this->getEnv());

	 std::vector<double> solution(tmpVals.getSize());

	 for (int i = 0; i < primalSol.size(); i++)
	 {
	 tmpVals.add(primalSol.at(i));
	 }

	 setSolution(cplexVars, tmpVals);
	 std::cout << "Solution suggested: " << this->getIncumbentObjValue() << std::endl;
	 }*/

}

class IncCallbackI: public IloCplex::IncumbentCallbackI
{
		IloNumVarArray cplexVars;

		//int numLazyConstrs;
		//int lastIterNum;
		//int numLazyAdded;
		//bool isBusy;

	private:
		//

	public:
		IloCplex::CallbackI* duplicateCallback() const
		{
			return (new (getEnv()) IncCallbackI(*this));
		}
		IncCallbackI(IloEnv env, IloNumVarArray xx2) :
				IloCplex::IncumbentCallbackI(env), cplexVars(xx2)
		{
			//this->processInfo = pInfo;

		}
		void main();	// the call back function
};

IloCplex::Callback IncCallback(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) IncCallbackI(env, cplexVars)));
}

void IncCallbackI::main() // Called at each node...
{
	IloNumArray tmpVals(this->getEnv());

	this->getValues(tmpVals, cplexVars);

	std::vector<double> solution(tmpVals.getSize());

	for (int i = 0; i < tmpVals.getSize(); i++)
	{
		solution.at(i) = tmpVals[i];
	}

	auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (ProcessInfo::getInstance().getPrimalBound() > this->getIncumbentObjValue())
	{
		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution,
				this->getObjValue(), ProcessInfo::getInstance().getCurrentIteration()->iterationNumber, mostDevConstr);
		tSelectPrimNLP->run();

		itersSinceNLPCall = 0;
		timestampLastNLPCall = ProcessInfo::getInstance().getElapsedTime("Total");
	}

	if (mostDevConstr.value <= Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		ProcessInfo::getInstance().addPrimalSolutionCandidate(solution, E_PrimalSolutionSource::IncumbentCallback, 0);
		//std::cout << "Most dev: " << mostDevConstr.value << std::endl;
		return;
	}
	else
	{
		ProcessInfo::getInstance().outputSummary("Not rejected: " + to_string(mostDevConstr.value));
		//reject();
	}
}

class CtCallbackI: public IloCplex::LazyConstraintCallbackI
{
		IloNumVarArray cplexVars;

		//bool isBusy;
		bool isMinimization;
		int cbCalls;

		TaskBase *taskSelectHPPts;

	public:
		IloCplex::CallbackI* duplicateCallback() const
		{
			return (new (getEnv()) CtCallbackI(*this));
		}
		CtCallbackI(IloEnv env, IloNumVarArray xx2) :
				IloCplex::LazyConstraintCallbackI(env), cplexVars(xx2)
		{
			//this->processInfo = pInfo;
			ProcessInfo::getInstance().lastLazyAddedIter = 0;
			//isBusy = false;
			isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();
			//lazyiters = 0;
			itersSinceNLPCall = 0;
			cbCalls = 0;

			auto taskInitLinesearch = new TaskInitializeLinesearch();

			if (static_cast<ES_SolutionStrategy>(Settings::getInstance().getIntSetting("SolutionStrategy", "Algorithm"))
					== ES_SolutionStrategy::ESH)
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

		}
		void main();	// the call back function
		bool createHyperplane(Hyperplane hyperplane);
};

IloCplex::Callback CtCallback(IloEnv env, IloNumVarArray cplexVars)
{
	return (IloCplex::Callback(new (env) CtCallbackI(env, cplexVars)));
}

bool CtCallbackI::createHyperplane(Hyperplane hyperplane)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration
	//auto originalProblem = originalProblem;
	std::vector < IndexValuePair > elements;

	auto varNames = ProcessInfo::getInstance().originalProblem->getVariableNames();

	double constant = ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionValue(
			hyperplane.sourceConstraintIndex, hyperplane.generatedPoint);

	if (hyperplane.sourceConstraintIndex == -1
			|| hyperplane.sourceConstraintIndex
					== ProcessInfo::getInstance().originalProblem->getNonlinearObjectiveConstraintIdx())
	{
		ProcessInfo::getInstance().outputDebug("     HP point generated for auxiliary objective function constraint");

		auto tmpArray =
				ProcessInfo::getInstance().originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
						&hyperplane.generatedPoint.at(0), -1, true);
		int number = ProcessInfo::getInstance().originalProblem->getNumberOfVariables();
		ProcessInfo::getInstance().numGradientEvals++;

		for (int i = 0; i < number - 1; i++)
		{
			if (tmpArray[i] != 0)
			{
				IndexValuePair pair;
				pair.idx = i;
				pair.value = tmpArray[i];

				elements.push_back(pair);
				constant += -tmpArray[i] * hyperplane.generatedPoint.at(i);

				ProcessInfo::getInstance().outputDebug(
						"     Gradient for variable " + varNames.at(i) + ": " + to_string(tmpArray[i]));
			}
		}

		ProcessInfo::getInstance().outputDebug("     Gradient for obj.var.: -1");

		IndexValuePair pair;
		pair.idx = ProcessInfo::getInstance().originalProblem->getNonlinearObjectiveVariableIdx();
		pair.value = -1.0;

		elements.push_back(pair);
		constant += /*-(-1) **/hyperplane.generatedPoint.at(pair.idx);
	}
	else
	{
		ProcessInfo::getInstance().outputDebug(
				"     HP point generated for constraint index " + to_string(hyperplane.sourceConstraintIndex));

		auto nablag = ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionGradient(
				hyperplane.sourceConstraintIndex, hyperplane.generatedPoint);

		for (int i = 0; i < nablag->number; i++)
		{
			IndexValuePair pair;
			pair.idx = nablag->indexes[i];
			pair.value = nablag->values[i];

			elements.push_back(pair);
			constant += -nablag->values[i] * hyperplane.generatedPoint.at(nablag->indexes[i]);

			ProcessInfo::getInstance().outputDebug(
					"     Gradient for variable" + varNames.at(nablag->indexes[i]) + ": "
							+ to_string(nablag->values[i]));
		}
	}

	bool hyperplaneIsOk = true;

	for (auto E : elements)
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
		IloExpr expr(this->getEnv());

		for (int i = 0; i < elements.size(); i++)
		{
			expr += elements.at(i).value * cplexVars[elements.at(i).idx];
		}

		IloRange tmpRange(this->getEnv(), -IloInfinity, expr, -constant);

		this->add(tmpRange);
		//this->add(tmpRange, IloCplex::CutManagement::UseCutPurge);
		//this->addLocal(tmpRange);

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;

		return (true);
	}

	return (false);
}

void CtCallbackI::main()
{
	this->cbCalls++;
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();
	bool returnAfter = false;

	double primalBegin = ProcessInfo::getInstance().getPrimalBound();

	if (itersSinceNLPCall == 0)
	{
		timestampLastNLPCall = ProcessInfo::getInstance().getElapsedTime("Total");
	}

	IloNumArray tmpVals(this->getEnv());

	this->getValues(tmpVals, cplexVars);

	std::vector<double> solution(tmpVals.getSize());

	for (int i = 0; i < tmpVals.getSize(); i++)
	{
		solution.at(i) = tmpVals[i];
	}

	auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

	auto absObjGap = ProcessInfo::getInstance().getAbsoluteObjectiveGap();
	auto relObjGap = ProcessInfo::getInstance().getRelativeObjectiveGap();

	double tmpDualObjBound = this->getBestObjValue();

// Check if better dual bound
	if ((isMinimization && tmpDualObjBound > ProcessInfo::getInstance().getDualBound())
			|| (!isMinimization && tmpDualObjBound < ProcessInfo::getInstance().getDualBound()))
	{
		DualSolution sol =
		{ solution, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound, currIter->iterationNumber };
		ProcessInfo::getInstance().addDualSolutionCandidate(sol);
	}

	auto relMIPGap = this->getMIPRelativeGap();

	if (abs(relMIPGap) < Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		ProcessInfo::getInstance().outputInfo(
				"Terminated by relative MIP gap tolerance: " + UtilityFunctions::toString(relMIPGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));
		return;
	}

	if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputInfo(
				"Terminated by relative objective gap tolerance: " + UtilityFunctions::toString(relObjGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolRelative", "Algorithm")));
		return;
	}

	if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().outputInfo(
				"Terminated by absolute objective gap tolerance: " + UtilityFunctions::toString(absObjGap) + " < "
						+ UtilityFunctions::toString(
								Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm")));
		return;
	}

	double elapsedTime = ProcessInfo::getInstance().getElapsedTime("Total") - timestampLastNLPCall;
	if ((elapsedTime > 5)
			|| ((itersSinceNLPCall > 50)
					&& mostDevConstr.value
							< (1000 * Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm")))
			|| (itersSinceNLPCall > 200))
	{
		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution,
				this->getObjValue(), ProcessInfo::getInstance().getCurrentIteration()->iterationNumber, mostDevConstr);
		tSelectPrimNLP->run();

		ProcessInfo::getInstance().checkPrimalSolutionCandidates();

		itersSinceNLPCall = 0;
		timestampLastNLPCall = ProcessInfo::getInstance().getElapsedTime("Total");
	}
	else
	{
		itersSinceNLPCall++;
	}

	std::vector < SolutionPoint > solutionPoints(1);

	SolutionPoint tmpSolPt;

	tmpSolPt.point = solution;
	tmpSolPt.objectiveValue = getObjValue();
	tmpSolPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
	tmpSolPt.maxDeviation = mostDevConstr;

	solutionPoints.at(0) = tmpSolPt;

	if (static_cast<ES_SolutionStrategy>(Settings::getInstance().getIntSetting("SolutionStrategy", "Algorithm"))
			== ES_SolutionStrategy::ESH)
	{
		if (static_cast<ES_LinesearchConstraintStrategy>(Settings::getInstance().getIntSetting(
				"LinesearchConstraintStrategy", "ESH")) == ES_LinesearchConstraintStrategy::AllAsMaxFunct)
		{
			dynamic_cast<TaskSelectHyperplanePointsLinesearch*>(taskSelectHPPts)->run(solutionPoints);
		}
		else
		{
			dynamic_cast<TaskSelectHyperplanePointsIndividualLinesearch*>(taskSelectHPPts)->run(solutionPoints);
		}
	}
	else
	{
		dynamic_cast<TaskSelectHyperplanePointsSolution*>(taskSelectHPPts)->run(solutionPoints);
	}

	int numHPsAdded = 0;

	for (auto hp : ProcessInfo::getInstance().hyperplaneWaitingList)
	{
		if (this->createHyperplane(hp)) numHPsAdded++;
	}

	ProcessInfo::getInstance().hyperplaneWaitingList.clear();

	std::stringstream tmpType;
	tmpType << "LazyCB";

	std::string hyperplanesExpr;

	if (numHPsAdded > 0)
	{
		hyperplanesExpr = "+" + to_string(numHPsAdded) + " = " + to_string(currIter->totNumHyperplanes);
	}
	else
	{
		hyperplanesExpr = " ";
	}

	auto tmpConstrExpr = UtilityFunctions::toStringFormat(mostDevConstr.value, "%.5f");

	if (mostDevConstr.idx != -1)
	{
		tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames()[mostDevConstr.idx] + ": "
				+ tmpConstrExpr;
	}
	else tmpConstrExpr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": " + tmpConstrExpr;
	{
	}

	std::string primalBoundExpr;
	std::string dualBoundExpr;
	std::string objExpr;

	if (this->hasIncumbent())
	{
		primalBoundExpr = UtilityFunctions::toString(this->getIncumbentObjValue());
	}

	objExpr = UtilityFunctions::toString(this->getObjValue());
	dualBoundExpr = UtilityFunctions::toString(this->getBestObjValue());

	auto tmpLine = boost::format("%|5| %|-11s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|")
			% (to_string(this->getMyThreadNum()) + "-" + to_string(cbCalls)) % tmpType.str() % hyperplanesExpr
			% dualBoundExpr % objExpr % primalBoundExpr % tmpConstrExpr;

	/*stringstream str;

	 str << ProcessInfo::getInstance().hyperplaneWaitingList.size() << "   LAZY #: " << currIter->totNumHyperplanes << "\t DB: "
	 << this->getBestObjValue() << "\t OBJ: " << this->getObjValue() << "\t GAP: " << absObjGap << "\t PB: "
	 << this->getIncumbentObjValue() << "\t DEV: " << mostDevConstr.value;
	 */
	ProcessInfo::getInstance().outputSummary(tmpLine.str());

//std::cout << "Before: " << primalBegin << " after " << ProcessInfo::getInstance().getPrimalBound() << std::endl;
	/*if (returnAfter || abs(primalBegin - ProcessInfo::getInstance().getPrimalBound()) > 0)
	 {
	 isBusy = false;
	 std::cout << "Term return after" << std::endl;
	 return;
	 }*/

	//isBusy = false;
}

MILPSolverCplexLazy::MILPSolverCplexLazy()
{

	discreteVariablesActivated = true;

	cplexModel = IloModel(cplexEnv);

	cplexVars = IloNumVarArray(cplexEnv);
	cplexConstrs = IloRangeArray(cplexEnv);
	cplexLazyConstrs = IloRangeArray(cplexEnv);

	itersSinceNLPCall = 0;

	cachedSolutionHasChanged = true;
	isVariablesFixed = false;

	checkParameters();
	addedHyperplanes = 0;
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
// Disable CPLEX output
		cplexInstance.setOut(cplexEnv.getNullStream());
		cplexInstance.setWarning(cplexEnv.getNullStream());

		cplexInstance.setParam(IloCplex::SolnPoolIntensity,
				Settings::getInstance().getIntSetting("SolnPoolIntensity", "CPLEX")); // Don't use 3 with heuristics
		cplexInstance.setParam(IloCplex::SolnPoolReplace,
				Settings::getInstance().getIntSetting("SolnPoolReplace", "CPLEX"));

		cplexInstance.setParam(IloCplex::RepairTries, 5);
//cplexInstance.setParam(IloCplex::HeurFreq,2);
//cplexInstance.setParam(IloCplex::AdvInd,2);

		cplexInstance.setParam(IloCplex::SolnPoolGap, Settings::getInstance().getDoubleSetting("SolnPoolGap", "CPLEX"));
		cplexInstance.setParam(IloCplex::SolnPoolCapacity,
				Settings::getInstance().getIntSetting("SolutionPoolSize", "MILP"));

		cplexInstance.setParam(IloCplex::Probe, Settings::getInstance().getIntSetting("Probe", "CPLEX"));
		cplexInstance.setParam(IloCplex::MIPEmphasis, Settings::getInstance().getIntSetting("MIPEmphasis", "CPLEX"));
		//cplexInstance.setParam(IloCplex::NodeSel, CPX_NODESEL_BESTEST);

//cplexInstance.setParam(IloCplex::ParallelMode, 1);
		cplexInstance.setParam(IloCplex::Threads, 8);

//	cplexInstance.setParam(IloCplex::PopulateLim, 10);

//cplexInstance.setParam(IloCplex::SolnPoolGap, 0);

//cplexInstance.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.1);
		cplexInstance.setParam(IloCplex::WorkMem, 30000);

		cplexInstance.setParam(IloCplex::NodeFileInd, 2);

//cplexInstance.setParam(IloCplex::Param::Tune::Measure, CPX_TUNE_AVERAGE);
//cplexInstance.setParam(IloCplex::Param::Tune::TimeLimit, 10);

//cplexInstance.setParam(IloCplex::NumericalEmphasis, 1);
//cplexInstance.setParam(IloCplex::MemoryEmphasis, 1);
//cplexInstance.setParam(IloCplex::EpGap, 10 ^ (-14));
//cplexInstance.setParam(IloCplex::EpInt, 10 ^ (-6));
//cplexInstance.setParam(IloCplex::EpOpt, 1 ^ (-9));
//cplexInstance.setParam(IloCplex::EpAGap, 10 ^ (-14));

		cplexInstance.setParam(IloCplex::WorkDir, "/data/stuff/tmp/");

		cplexInstance.setParam(IloCplex::IntSolLim, 2100000000);
//cplexInstance.setParam(IloCplex::EpMrk, 0.9);

		cplexInstance.use(CtCallback(cplexEnv, cplexVars));
		cplexInstance.use(IncCallback(cplexEnv, cplexVars));
		cplexInstance.use(HCallback(cplexEnv, cplexVars));

	}
	catch (IloException& e)
	{
		ProcessInfo::getInstance().outputError("CPLEX error when initializing parameters for linear solver",
				e.getMessage());
	}
}

int MILPSolverCplexLazy::addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan)
{
	try
	{
		IloExpr expr(cplexEnv);

		for (int i = 0; i < elements.size(); i++)
		{
			expr += elements.at(i).value * cplexVars[elements.at(i).idx];
		}

		if (isGreaterThan)
		{
			IloRange tmpRange(cplexEnv, -constant, expr);
			cplexConstrs.add(tmpRange);
			cplexModel.add(tmpRange);
		}
		else
		{
			IloRange tmpRange(cplexEnv, -IloInfinity, expr, -constant);
			cplexConstrs.add(tmpRange);
			cplexModel.add(tmpRange);
		}

		modelUpdated = true;

		expr.end();
	}
	catch (IloException& e)
	{
		ProcessInfo::getInstance().outputError("Error when adding linear constraint:", e.getMessage());

		return (-1);
	}

	return (cplexInstance.getNrows() - 1);
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
			// Must add the lazy constraints again if we have extracted the model

			if (cplexLazyConstrs.getSize() > 0)
			{
				ProcessInfo::getInstance().startTimer("LazyChange");
				cplexInstance.addLazyConstraints(cplexLazyConstrs);
				ProcessInfo::getInstance().stopTimer("LazyChange");
			}
			modelUpdated = false;
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

void MILPSolverCplexLazy::setSolutionLimit(int limit)
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
