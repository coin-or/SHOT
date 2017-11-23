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

bool MILPSolverCplexLazy::createLinearProblem(OptProblem * origProblem)
{
	originalProblem = origProblem;

	auto numVar = origProblem->getNumberOfVariables();
	auto tmpLBs = origProblem->getVariableLowerBounds();
	auto tmpUBs = origProblem->getVariableUpperBounds();
	auto tmpNames = origProblem->getVariableNames();
	auto tmpTypes = origProblem->getVariableTypes();

	int numCon = origProblem->getNumberOfConstraints();
	if (origProblem->isObjectiveFunctionNonlinear()) numCon--; // Only want the number of original constraints and not the objective function

// Now creating the variables
	for (int i = 0; i < numVar; i++)
	{
		if (tmpTypes.at(i) == 'C')
		{
			cplexVars.add(IloNumVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOFLOAT, tmpNames.at(i).c_str()));
		}
		else if (tmpTypes.at(i) == 'I')
		{
			cplexVars.add(IloNumVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOINT, tmpNames.at(i).c_str()));
		}
		else if (tmpTypes.at(i) == 'B')
		{
			cplexVars.add(IloNumVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOBOOL, tmpNames.at(i).c_str()));
		}
		else if (tmpTypes.at(i) == 'D')
		{
			cplexVars.add(IloSemiContVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOFLOAT, tmpNames.at(i).c_str()));
		}
		else
		{
			ProcessInfo::getInstance().outputWarning(
					"Error variable type " + to_string(tmpTypes.at(i)) + " for " + tmpNames.at(i));
		}
	}

	cplexModel.add(cplexVars);

// Now creating the objective function

	IloExpr objExpr(cplexEnv);

	auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

	for (int i = 0; i < tmpObjPairs.size(); i++)
	{
		objExpr += tmpObjPairs.at(i).second * cplexVars[tmpObjPairs.at(i).first];
	}

// Add quadratic terms in the objective if they exist (and the strategy is to solve QPs)
	if (origProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
	{
		auto quadTerms = origProblem->getQuadraticTermsInConstraint(-1);

		for (auto T : quadTerms)
		{
			objExpr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
		}
	}

	double objConstant = origProblem->getObjectiveConstant();
	if (objConstant != 0.0) objExpr += objConstant;

	if (origProblem->isTypeOfObjectiveMinimize())
	{
		cplexModel.add(IloMinimize(cplexEnv, objExpr));
	}
	else
	{
		cplexModel.add(IloMaximize(cplexEnv, objExpr));
	}

	objExpr.end();

// Now creating the constraints

	int row_nonz = 0;
	int obj_nonz = 0;
	int varIdx = 0;

	SparseMatrix *m_linearConstraintCoefficientsInRowMajor =
			origProblem->getProblemInstance()->getLinearConstraintCoefficientsInRowMajor();

	auto constrTypes = origProblem->getProblemInstance()->getConstraintTypes();
	auto constrNames = origProblem->getProblemInstance()->getConstraintNames();
	auto constrLBs = origProblem->getProblemInstance()->getConstraintLowerBounds();
	auto constrUBs = origProblem->getProblemInstance()->getConstraintUpperBounds();

//try
//{
	for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
	{
// Only use constraints that don't contain a nonlinear part (may include a quadratic part)
		if (!origProblem->isConstraintNonlinear(rowIdx))
		{
			IloExpr expr(cplexEnv);

			if (origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients != NULL
					&& origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients->numberOfValues
							> 0)
			{
				row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1]
						- m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

				for (int j = 0; j < row_nonz; j++)
				{
					double val =
							m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
									+ j];
					varIdx =
							m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
									+ j];

					expr += val * cplexVars[varIdx];
				}
			}

			// Add quadratic terms if they exist and have been defined as quadratic and not nonlinear
			auto quadTerms = origProblem->getQuadraticTermsInConstraint(rowIdx);

			for (auto T : quadTerms)
			{
				expr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
			}

			expr += origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

			// Add the constraint
			if (constrTypes[rowIdx] == 'L')
			{
				IloRange tmpRange = IloRange(cplexEnv, -IloInfinity, expr, constrUBs[rowIdx],
						constrNames[rowIdx].c_str());
				cplexConstrs.add(tmpRange);
			}
			else if (constrTypes[rowIdx] == 'G')
			{
				IloRange tmpRange = IloRange(cplexEnv, constrLBs[rowIdx], expr, IloInfinity,
						constrNames[rowIdx].c_str());
				cplexConstrs.add(tmpRange);
			}
			else if (constrTypes[rowIdx] == 'E')
			{
				IloRange tmpRange = IloRange(cplexEnv, constrLBs[rowIdx], expr, constrUBs[rowIdx],
						constrNames[rowIdx].c_str());
				cplexConstrs.add(tmpRange);
			}
			else
			{
			}

			expr.end();
		}
	}

	cplexModel.add(cplexConstrs);

	try
	{
//cplexInstance.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
		cplexInstance = IloCplex(cplexModel);

		IloExprArray lhs;
		IloNumArray rhs;

// Do not use the callback if quadratic function, not needed anymore...
		cplexInstance.use(CtCallback(cplexEnv, cplexVars));
		cplexInstance.use(IncCallback(cplexEnv, cplexVars));
		cplexInstance.use(HCallback(cplexEnv, cplexVars));

	}
	catch (IloException& e)
	{
		ProcessInfo::getInstance().outputError("CPLEX exception caught when creating model", e.getMessage());
		return (false);
	}

	return (true);
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

void MILPSolverCplexLazy::activateDiscreteVariables(bool activate)
{
	auto variableTypes = originalProblem->getVariableTypes();

	try
	{
		for (int i = 0; i < cplexVarConvers.size(); i++)
		{
			cplexVarConvers.at(i).end();
		}

		cplexVarConvers.clear();

		if (activate)
		{
			ProcessInfo::getInstance().outputDebug("     Activating MILP strategy");

			for (int i = 0; i < originalProblem->getNumberOfVariables(); i++)
			{
				if (variableTypes.at(i) == 'I')
				{
					auto tmpVar = cplexVars[i];
					auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOINT);
					cplexModel.add(tmpConv);
					cplexVarConvers.push_back(tmpConv);
				}
				else if (variableTypes.at(i) == 'B')
				{
					auto tmpVar = cplexVars[i];
					auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOBOOL);
					cplexModel.add(tmpConv);
					cplexVarConvers.push_back(tmpConv);
				}
			}

			discreteVariablesActivated = true;
		}
		else
		{
			ProcessInfo::getInstance().outputDebug("     Activating LP strategy");
			for (int i = 0; i < originalProblem->getNumberOfVariables(); i++)
			{
				if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
				{
					auto tmpVar = cplexVars[i];
					auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOFLOAT);
					cplexModel.add(tmpConv);
					cplexVarConvers.push_back(tmpConv);
				}
			}

			discreteVariablesActivated = false;
		}
	}
	catch (IloException& e)
	{
		if (activate) ProcessInfo::getInstance().outputError("Error when activating discrete variables",
				e.getMessage());
		else ProcessInfo::getInstance().outputError("Error when deactivating discrete variables", e.getMessage());
	}
}

E_ProblemSolutionStatus MILPSolverCplexLazy::getSolutionStatus()
{
	E_ProblemSolutionStatus MILPSolutionStatus;

	try
	{
		auto status = cplexInstance.getStatus();

		if (status == IloAlgorithm::Status::Optimal)
		{
			MILPSolutionStatus = E_ProblemSolutionStatus::Optimal;
		}
		else if (status == IloAlgorithm::Status::Infeasible)
		{
			MILPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
		}
		else if (status == IloAlgorithm::Status::InfeasibleOrUnbounded)
		{
			MILPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
		}
		else if (status == IloAlgorithm::Status::Unbounded)
		{
			MILPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
		}
		else if (status == IloAlgorithm::Status::Infeasible)
		{
			MILPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
		}
		else if (status == IloAlgorithm::Status::Feasible)
		{
			if (this->getDiscreteVariableStatus())
			{
				MILPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
			}
			else
			{
				MILPSolutionStatus = E_ProblemSolutionStatus::Optimal;
			}

		}
		else if (status == IloAlgorithm::Status::Error)
		{
			MILPSolutionStatus = E_ProblemSolutionStatus::Error;
		}
		else if (status == IloAlgorithm::Status::Unknown)
		{
			MILPSolutionStatus = E_ProblemSolutionStatus::Error;
		}
		else
		{
			ProcessInfo::getInstance().outputError("MILP solver return status unknown = " + status);
			MILPSolutionStatus = E_ProblemSolutionStatus::Error;
		}
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when obtaining solution status:", e.getMessage());

	}

	return (MILPSolutionStatus);
}

E_ProblemSolutionStatus MILPSolverCplexLazy::solveProblem()
{
	startTimer();

	E_ProblemSolutionStatus MILPSolutionStatus;
	cachedSolutionHasChanged = true;

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
		MILPSolutionStatus = getSolutionStatus();

	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when solving MILP/LP problem", e.getMessage());
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	stopTimer();

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
	if (originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
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

std::vector<double> MILPSolverCplexLazy::getVariableSolution(int solIdx)
{
	bool isMILP = getDiscreteVariableStatus();
	int numVar = cplexVars.getSize();
	std::vector<double> solution(numVar);

	IloNumArray tmpSolsCplex(cplexEnv);

	try
	{
		if (isMILP)
		{
			cplexInstance.getValues(cplexVars, tmpSolsCplex, solIdx);
		}
		else
		{
			cplexInstance.getValues(cplexVars, tmpSolsCplex);
		}

		for (int i = 0; i < numVar; i++)
		{
			solution.at(i) = tmpSolsCplex[i];
		}
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when reading solution with index", e.getMessage());
	}

	return (solution);
}

int MILPSolverCplexLazy::getNumberOfSolutions()
{
	int numSols = 0;
	bool isMILP = getDiscreteVariableStatus();

	try
	{
		if (isMILP) numSols = cplexInstance.getSolnPoolNsolns();
		else numSols = 1;
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when obtaining number of solutions", e.getMessage());
	}

	return (numSols);
}

double MILPSolverCplexLazy::getObjectiveValue(int solIdx)
{
	bool isMILP = getDiscreteVariableStatus();

	double objVal = NAN;

	if (!isMILP && solIdx > 0) // LP problems only have one solution!
	{

		ProcessInfo::getInstance().outputError(
				"Cannot obtain solution with index " + to_string(solIdx) + " since the problem is LP/QP!");

		return (objVal);
	}

	try
	{
		if (isMILP)
		{
			objVal = cplexInstance.getObjValue(solIdx);
		}
		else
		{
			objVal = cplexInstance.getObjValue();
		}

	}
	catch (IloException &e)
	{

		ProcessInfo::getInstance().outputError(
				"Error when obtaining objective value for solution index " + to_string(solIdx), e.getMessage());

	}

	return (objVal);

}

void MILPSolverCplexLazy::populateSolutionPool()
{
	ProcessInfo::getInstance().startTimer("PopulateSolutionPool");
	double initialPopulateTimeLimit = 0.5;
	double timeLimitIncreaseFactor = 2.0;

	try
	{
		int poolSizeBefore = cplexInstance.getSolnPoolNsolns();

		if (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber == 0)
		{
			setTimeLimit(initialPopulateTimeLimit);
		}
		else
		{
			// Note that the vector elements are rearranged each iteration
			std::nth_element(iterDurations.begin(), iterDurations.begin() + iterDurations.size() / 2,
					iterDurations.end());

			double newTimeLimit = timeLimitIncreaseFactor * iterDurations[iterDurations.size() / 2];
			setTimeLimit(newTimeLimit);
		}

		double newSolnPoolGap = min(1.0e+75, ProcessInfo::getInstance().getAbsoluteObjectiveGap());
		cplexInstance.setParam(IloCplex::SolnPoolGap, newSolnPoolGap);

		cplexInstance.populate();

		int poolSizeAfter = cplexInstance.getSolnPoolNsolns();

		if (poolSizeAfter > poolSizeBefore)
		{

			ProcessInfo::getInstance().outputDebug(
					"     Solution pool populated from: " + to_string(poolSizeBefore) + " to "
							+ to_string(poolSizeAfter));
		}
	}
	catch (IloException &e)
	{

		ProcessInfo::getInstance().outputError("Error when populating solution pool", e.getMessage());

		ProcessInfo::getInstance().stopTimer("PopulateSolutionPool");

	}

	ProcessInfo::getInstance().stopTimer("PopulateSolutionPool");
}

void MILPSolverCplexLazy::setTimeLimit(double seconds)
{
	try
	{
		if (seconds > 0)
		{
			cplexInstance.setParam(IloCplex::TiLim, seconds);
		}
		else
		{
			cplexInstance.setParam(IloCplex::TiLim, 0.001);
		}
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when setting time limit", e.getMessage());
	}
}

void MILPSolverCplexLazy::setCutOff(double cutOff)
{
	try
	{
		if (originalProblem->isTypeOfObjectiveMinimize())
		{
			cplexInstance.setParam(IloCplex::CutUp, cutOff);
			ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + to_string(cutOff) + " for minimization.");

		}
		else
		{
			cplexInstance.setParam(IloCplex::CutLo, cutOff);
			ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + to_string(cutOff) + " for maximization.");

		}
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when setting cut off value", e.getMessage());
	}
}

void MILPSolverCplexLazy::addMIPStart(std::vector<double> point)
{
	IloNumArray startVal(cplexEnv);

	int numVar = cplexVars.getSize();

	for (int i = 0; i < numVar; i++)
	{
		startVal.add(point.at(i));
	}

	try
	{
		cplexInstance.addMIPStart(cplexVars, startVal);

	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when adding MIP starting point", e.getMessage());
	}

	ProcessInfo::getInstance().outputInfo("     Added MIP starting point.");
}

void MILPSolverCplexLazy::deleteMIPStarts()
{
	int numStarts = cplexInstance.getNMIPStarts();

	if (numStarts > 0)
	{
		try
		{
			cplexInstance.deleteMIPStarts(0, numStarts);

		}
		catch (IloException &e)
		{
			ProcessInfo::getInstance().outputError("Error when deleting MIP starting points", e.getMessage());
		}

		ProcessInfo::getInstance().outputInfo("    Deleted " + to_string(numStarts) + " MIP starting points.");
	}
}

void MILPSolverCplexLazy::writeProblemToFile(std::string filename)
{
	try
	{
		if (modelUpdated)
		{
			//Extract the model if we have updated the constraints
			cplexInstance.extract(cplexModel);
			// Must add the lazy constraints again if we have extracted the model
			if (cplexLazyConstrs.getSize() > 0) cplexInstance.addLazyConstraints(cplexLazyConstrs);
			modelUpdated = false;
		}

		cplexInstance.exportModel(filename.c_str());
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when saving model to file", e.getMessage());
	}
}

void MILPSolverCplexLazy::fixVariable(int varIndex, double value)
{
	updateVariableBound(varIndex, value, value);
}

void MILPSolverCplexLazy::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
	try
	{
		cplexVars[varIndex].setBounds(lowerBound, upperBound);
		cplexInstance.extract(cplexModel);
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError(
				"Error when updating variable bounds for variable index" + to_string(varIndex), e.getMessage());
	}
}

pair<double, double> MILPSolverCplexLazy::getCurrentVariableBounds(int varIndex)
{
	pair<double, double> tmpBounds;

	try
	{
		tmpBounds.first = cplexVars[varIndex].getLB();
		tmpBounds.second = cplexVars[varIndex].getUB();
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError(
				"Error when obtaining variable bounds for variable index" + to_string(varIndex), e.getMessage());
	}
	return (tmpBounds);
}

bool MILPSolverCplexLazy::supportsQuadraticObjective()
{
	return (true);
}
bool MILPSolverCplexLazy::supportsQuadraticConstraints()
{
	return (true);
}

double MILPSolverCplexLazy::getDualObjectiveValue()
{

	bool isMILP = getDiscreteVariableStatus();
	double objVal = NAN;

	try
	{
		objVal = cplexInstance.getBestObjValue();

	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when obtaining dual objective value", e.getMessage());
	}

	return (objVal);
}

void MILPSolverCplexLazy::checkParameters()
{

}

std::pair<std::vector<double>, std::vector<double> > MILPSolverCplexLazy::presolveAndGetNewBounds()
{
	return (std::make_pair(originalProblem->getVariableLowerBounds(), originalProblem->getVariableLowerBounds()));
}

void MILPSolverCplexLazy::writePresolvedToFile(std::string filename)
{
	try
	{
//Not implemented
	}
	catch (IloException &e)
	{
		ProcessInfo::getInstance().outputError("Error when saving presolved model to file", e.getMessage());
	}
}
