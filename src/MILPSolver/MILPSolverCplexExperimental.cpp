#include "IMILPSolver.h"
#include "MILPSolverCplexExperimental.h"
//#include <ilcplex/cplex.h>
ILOSTLBEGIN

//ProcessInfo* processInfo;

/*void SolutionFilterCallbackI::main()
 {
 //create filter
 //std::cout << "calling incumbent callback" << std::endl;
 IloModel model = getModel();
 //cout << m << " " << n << endl;
 IloEnv env = getEnv();

 IloNumArray val(env, xVar.getSize());
 getValues(val, xVar);

 int numVar = processInfo->originalProblem->getNumberOfVariables();

 std::vector<double> pt(numVar);

 for (int i = 0; i < numVar; i++)
 {
 pt.at(i) = val[i];
 }

 auto tmpVal = processInfo->originalProblem->getMostDeviatingConstraint(pt).value;
 std::cout << "Callback value " << tmpVal << std::endl;
 if (processInfo->iterations.size() > 10 && tmpVal > 0)
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
int lazyiters;
int itersSinceNLPCall;
//double minConstrDev;
//bool useMinConstrDev;
int lazyBreakCtr;
double timestampLastNLPCall;
double lastIncumbentObjVal;

TaskBase *tCheckPrimCands;
TaskBase *tCheckDualCands;
TaskBase *tSelectPrimNLP;

class HCallbackI: public IloCplex::HeuristicCallbackI
{
		IloNumVarArray cplexVars;

		ProcessInfo *processInfo;
		SHOTSettings::Settings *settings;

		//int numLazyConstrs;
		//int lastIterNum;
		//int numLazyAdded;
		//bool isBusy;

	private:
		//ProcessInfo *processInfo;

	public:
		IloCplex::CallbackI* duplicateCallback() const
		{
			return (new (getEnv()) HCallbackI(*this));
		}
		HCallbackI(IloEnv env, IloNumVarArray xx2, ProcessInfo *pInfo) :
				IloCplex::HeuristicCallbackI(env), cplexVars(xx2)
		{
			this->processInfo = pInfo;

			settings = SHOTSettings::Settings::getInstance();
		}
		void main();	// the call back function
};

IloCplex::Callback HCallback(IloEnv env, IloNumVarArray cplexVars, ProcessInfo *pInfo)
{
	return (IloCplex::Callback(new (env) HCallbackI(env, cplexVars, pInfo)));
}

void HCallbackI::main()
{
	if (this->hasIncumbent())
	{
		auto primalBound = processInfo->getPrimalBound();

		bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

		bool betterPrimalThanIncumbent = ((isMinimization && primalBound < this->getIncumbentObjValue())
				|| (!isMinimization && primalBound > this->getIncumbentObjValue()));

		if (betterPrimalThanIncumbent)
		{
			auto primalSol = processInfo->primalSolution;

			IloNumArray tmpVals(this->getEnv());

			std::vector<double> solution(tmpVals.getSize());

			for (int i = 0; i < primalSol.size(); i++)
			{
				tmpVals.add(primalSol.at(i));
			}

			//std::cout << "Solution suggested: " << this->getIncumbentObjValue() << " -> " << primalBound << std::endl;

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

				processInfo->addPrimalSolutionCandidate(solution, E_PrimalSolutionSource::LazyConstraintCallback,
						processInfo->getCurrentIteration()->iterationNumber);

			}
			catch (IloException &e)
			{
				std::cout << "ERROR" << std::endl;
			}

		}
	}
	else //Added august 2016
	{
		if (processInfo->primalSolution.size() == 0) return;

		auto primalSol = processInfo->primalSolution;

		IloNumArray tmpVals(this->getEnv());

		std::vector<double> solution(tmpVals.getSize());

		for (int i = 0; i < primalSol.size(); i++)
		{
			tmpVals.add(primalSol.at(i));
		}

		//std::cout << "Solution suggested: " << this->getIncumbentObjValue() << " -> " << primalBound << std::endl;

		setSolution(cplexVars, tmpVals);
	}
}
class CtCallbackI: public IloCplex::LazyConstraintCallbackI
{
		IloNumVarArray cplexVars;

		ProcessInfo *processInfo;
		SHOTSettings::Settings *settings;

		//int numLazyConstrs;
		//int lastIterNum;
		//int numLazyAdded;
		bool isBusy;

		ILinesearchMethod *linesearchMethod;
	private:
		//ProcessInfo *processInfo;

	public:
		IloCplex::CallbackI* duplicateCallback() const
		{
			return (new (getEnv()) CtCallbackI(*this));
		}
		CtCallbackI(IloEnv env, IloNumVarArray xx2, ProcessInfo *pInfo) :
				IloCplex::LazyConstraintCallbackI(env), cplexVars(xx2)
		{
			this->processInfo = pInfo;
			//cplexLazyConstrs = IloRangeArray(env);
			//cplexLazyConstrs.reserve(1000);
			processInfo->lastLazyAddedIter = 0;
			//numLazyAdded = 0;
			isBusy = false;
			lazyiters = 0;
			itersSinceNLPCall = 0;

			//minConstrDev = 1000;
			//useMinConstrDev = false;
			lazyBreakCtr = 1000;
			settings = SHOTSettings::Settings::getInstance();

			if (settings->getIntSetting("LinesearchMethod", "Linesearch")
					== static_cast<int>(ES_LinesearchMethod::Boost))
			{
				linesearchMethod = new LinesearchMethodBoost();
				processInfo->outputDebug("Boost linesearch implementation selected.");
			}
			else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
					== static_cast<int>(ES_LinesearchMethod::Bisection))
			{
				linesearchMethod = new LinesearchMethodBisection();
				processInfo->outputDebug("Bisection linesearch implementation selected.");
			}

			tCheckPrimCands = new TaskCheckPrimalSolutionCandidates();
			tCheckDualCands = new TaskCheckDualSolutionCandidates();
			tSelectPrimNLP = new TaskSelectPrimalCandidatesFromNLP();

		}
		void main();	// the call back function
};

IloCplex::Callback CtCallback(IloEnv env, IloNumVarArray cplexVars, ProcessInfo *pInfo)
{
	return (IloCplex::Callback(new (env) CtCallbackI(env, cplexVars, pInfo)));
}

void CtCallbackI::main()
{
	int currIterNum = processInfo->getCurrentIteration()->iterationNumber;
	bool returnAfter = false;

	/*while (isBusy)
	 {
	 sleep(0.000001);
	 }
	 */
	isBusy = true;

	double primalBegin = processInfo->getPrimalBound();

	if (itersSinceNLPCall == 0)
	{
		timestampLastNLPCall = processInfo->getElapsedTime("Total");
	}

	/*
	 if (lazyiters == 50)
	 {

	 if (minConstrDev > settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	 {
	 useMinConstrDev = true;

	 }
	 }*/
	IloNumArray tmpVals(this->getEnv());

	this->getValues(tmpVals, cplexVars);

	std::vector<double> solution(tmpVals.getSize());

	for (int i = 0; i < tmpVals.getSize(); i++)
	{
		solution.at(i) = tmpVals[i];
	}

	auto currIter = processInfo->getCurrentIteration();
	bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

	auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(solution);

	auto gap = processInfo->getAbsoluteObjectiveGap();
	auto relgap = processInfo->getRelativeObjectiveGap();

	if (lazyiters > lazyBreakCtr && this->hasIncumbent())
	{
		//lazyBreakCtr = lazyBreakCtr * 1.5;
		//minConstrDev = 1000;
		std::cout << "     OBJI: " << this->getObjValue() << "\t BEST: " << this->getBestObjValue() << "\t INT: "
				<< this->getIncumbentObjValue() << "\t\t DEV: " << mostDevConstr.value << " < " << std::endl;
		std::cout << "     Lazyiters: " << lazyiters << " >= " << lazyBreakCtr << std::endl;
		lazyiters = 0;
		itersSinceNLPCall = 0;
		//timestampLastNLPCall = processInfo->getElapsedTime("Total");
		returnAfter = true;
		//isBusy = false;
		//return;
	}
	else
	{
		lazyiters++;
	}

	if (mostDevConstr.value <= settings->getDoubleSetting("PrimalBoundNonlinearTolerance", "PrimalBound"))
	{
		auto tmpPrimal = processInfo->getPrimalBound();

		processInfo->addPrimalSolutionCandidate(solution, E_PrimalSolutionSource::LazyConstraintCallback, currIterNum);

		tCheckPrimCands->run();

		if (abs(tmpPrimal - processInfo->getPrimalBound()) > 0)
		{
			isBusy = false;
			return;
		}
	}

	double tmpDualObjBound = this->getBestObjValue();

	if (abs(tmpDualObjBound - processInfo->getDualBound() > 0.000001))
	{
		DualSolution sol =
		{ solution, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound, currIter->iterationNumber };
		processInfo->addDualSolutionCandidate(sol);

		tCheckDualCands->run();
	}

	if (mostDevConstr.value <= settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		returnAfter = true;
		isBusy = false;
		return;
	}

	//auto dualitygap = abs(this->getBestObjValue() - this->getObjValue());
	auto mipgap = this->getMIPRelativeGap();
	//std::cout << "Mip gap: " << mipgap << std::endl;
	//if (dualitygap < settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
	if (abs(mipgap) < settings->getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		//std::cout << "Mip gap ok: " << dualitygap << "(" << this->getBestObjValue() << ", " << this->getObjValue()
		//<< std::endl;
		std::cout << "Mip gap ok: " << mipgap << std::endl;

		returnAfter = true;
		isBusy = false;
		return;
	}
	else
	{
		//std::cout << "Duality gap: " << dualitygap << std::endl;
	}

//auto gap = abs(processInfo->getPrimalBound() - this->getBestObjValue());

//double relgap = gap / ((1e-10) + abs(processInfo->getPrimalBound()));

//std::cout << "O:" << this->getIncumbentObjValue() << std::endl;

//auto gap = processInfo->getAbsoluteObjectiveGap();
//auto relgap = processInfo->getRelativeObjectiveGap();

	/*if (abs(this->getIncumbentObjValue() - this->getBestObjValue())
	 < settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
	 {
	 std::cout << "     OBJ: " << this->getObjValue() << "\t BEST: " << this->getBestObjValue() << "\t INT: "
	 << this->getIncumbentObjValue() << "\t\t DEV: " << mostDevConstr.value << " < " << std::endl;
	 std::cout << "     Relative gap: " << relgap << std::endl;
	 returnAfter = true;
	 isBusy = false;
	 return;
	 }
	 else*/
	if (relgap < settings->getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{
		std::cout << "     OBJR: " << this->getObjValue() << "\t BEST: " << this->getBestObjValue() << "\t INT: "
				<< this->getIncumbentObjValue() << "\t\t DEV: " << mostDevConstr.value << " < " << std::endl;
		std::cout << "     Relative gap: " << relgap << std::endl;
		returnAfter = true;
		isBusy = false;
		return;
	}
	else if (gap < settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
	{
		std::cout << "     OBJA: " << this->getObjValue() << "\t BEST: " << this->getBestObjValue() << "\t INT: "
				<< this->getIncumbentObjValue() << "\t\t DEV: " << mostDevConstr.value << " < " << std::endl;
		std::cout << "     Absolute gap: " << gap << std::endl;
		returnAfter = true;
		isBusy = false;
		return;
	}

	double elapsedTime = processInfo->getElapsedTime("Total") - timestampLastNLPCall;
//std::cout << "Elapsed" << elapsedTime << std::endl;
	/*if ((elapsedTime > 5.0)
	 || ((itersSinceNLPCall > 50)
	 && mostDevConstr.value < (1000 * settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm")))
	 || (itersSinceNLPCall > 200))
	 {
	 /*dynamic_cast<TaskSelectPrimalCandidatesFromNLP*>(tSelectPrimNLP)->setFixedPoint(solution);
	 tSelectPrimNLP->run();

	 tCheckPrimCands->run();
	 itersSinceNLPCall = 0;
	 timestampLastNLPCall = processInfo->getElapsedTime("Total");
	 }
	 else
	 {
	 itersSinceNLPCall++;
	 }*/

	std::vector < Hyperplane > hyperplanes;
	Hyperplane hyperplane;

	for (int j = 0; j < processInfo->interiorPts.size(); j++)
	{
		//processInfo->startTimer(E_TimerTypes::Linesearch);
		auto xNLP = processInfo->interiorPts.at(j).point;

		processInfo->startTimer("HyperplaneLinesearch");

		auto intervalPts = linesearchMethod->findZero(xNLP, solution,
				settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
				settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
				settings->getDoubleSetting("LinesearchConstrEps", "Linesearch"));

		auto xNewc = intervalPts.second;

		processInfo->stopTimer("HyperplaneLinesearch");
		//processInfo->stopTimer(E_TimerTypes::Linesearch);

		if (xNewc.size() == 0) break;	// TODO remove?

		auto tmpMostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(xNewc);

		if (tmpMostDevConstr.value < 0)
		{
			//processInfo->addPrimalSolutionCandidate(xNewc, E_PrimalSolutionSource::LazyConstraintCallback, currIterNum);

			auto tmpPrimal = processInfo->getPrimalBound();

			processInfo->addPrimalSolutionCandidate(xNewc, E_PrimalSolutionSource::LazyConstraintCallback, currIterNum);

			tCheckPrimCands->run();

			if (abs(tmpPrimal - processInfo->getPrimalBound()) > 0)
			{
				// Tillsatt 10.4.2016
				returnAfter = true;
			}
		}

		if (tmpMostDevConstr.value < 0)
		{
			isBusy = false;
			processInfo->outputWarning("     Hyperplane point is on the interior.");

			return;
		}
		else
		{
			processInfo->outputDebug("     Hyperplane point is on the exterior.");

			hyperplane.sourceConstraintIndex = tmpMostDevConstr.idx;
			hyperplane.generatedPoint = xNewc;

			hyperplanes.push_back(hyperplane);

		}
	}

// Tries to remove the incumbent solution. Does not seem to work
	if (false && abs(this->getIncumbentObjValue() - lastIncumbentObjVal) > 0.1)
	{
		lastIncumbentObjVal = this->getIncumbentObjValue();

		Hyperplane hyperplane2;

		if (this->hasIncumbent())
		{
			this->getIncumbentValues(tmpVals, cplexVars);
			std::cout << "     INCUMBENT: " << this->getIncumbentObjValue() << std::endl;

			std::vector<double> solution(tmpVals.getSize());

			for (int i = 0; i < tmpVals.getSize(); i++)
			{
				solution.at(i) = tmpVals[i];
			}

			auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(solution);
			std::cout << "Incumbent max dev" << mostDevConstr.value << std::endl;

			for (int j = 0; j < processInfo->interiorPts.size(); j++)
			{
				//processInfo->startTimer(E_TimerTypes::Linesearch);
				auto xNLP = processInfo->interiorPts.at(j).point;

				processInfo->startTimer("HyperplaneLinesearch");

				auto intervalPts = linesearchMethod->findZero(xNLP, solution,
						settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
						settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
						settings->getDoubleSetting("LinesearchConstrEps", "Linesearch"));

				auto xNewc = intervalPts.second;

				processInfo->stopTimer("HyperplaneLinesearch");
				//processInfo->stopTimer(E_TimerTypes::Linesearch);

				if (xNewc.size() == 0) break;	// TODO remove?

				auto tmpMostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(xNewc);

				if (tmpMostDevConstr.value < 0)
				{
					processInfo->addPrimalSolutionCandidate(xNewc, E_PrimalSolutionSource::LazyConstraintCallback,
							currIterNum);
				}

				if (tmpMostDevConstr.value < 0)
				{
					isBusy = false;

					processInfo->outputWarning("     Hyperplane point is on the interior.");

					return;
				}
				else
				{
					processInfo->outputDebug("     Hyperplane point is on the exterior.");

					hyperplane2.sourceConstraintIndex = tmpMostDevConstr.idx;
					hyperplane2.generatedPoint = xNewc;

				}
			}
		}
	}

	auto varNames = processInfo->originalProblem->getVariableNames();

	for (auto hp : hyperplanes)
	{
		std::vector < IndexValuePair > elements;
		double constant = processInfo->originalProblem->calculateConstraintFunctionValue(hp.sourceConstraintIndex,
				hp.generatedPoint);

		if (hp.sourceConstraintIndex == -1
				|| hp.sourceConstraintIndex == processInfo->originalProblem->getNonlinearObjectiveConstraintIdx())
		{

			processInfo->outputDebug("     HP point generated for auxiliary objective function constraint");

			auto tmpArray = processInfo->originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
					&hp.generatedPoint.at(0), -1, true);
			int number = processInfo->originalProblem->getNumberOfVariables();

			for (int i = 0; i < number - 1; i++)
			{
				if (tmpArray[i] != 0)
				{
					IndexValuePair pair;
					pair.idx = i;
					pair.value = tmpArray[i];

					elements.push_back(pair);
					constant += -tmpArray[i] * hp.generatedPoint.at(i);
				}
			}

			IndexValuePair pair;
			pair.idx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();
			pair.value = -1.0;

			elements.push_back(pair);
			constant += /*-(-1) **/hp.generatedPoint.at(pair.idx);
		}
		else
		{

			auto nablag = processInfo->originalProblem->calculateConstraintFunctionGradient(hp.sourceConstraintIndex,
					hp.generatedPoint);
			processInfo->numGradientEvals++;

			for (int i = 0; i < nablag->number; i++)
			{
				IndexValuePair pair;
				pair.idx = nablag->indexes[i];
				pair.value = nablag->values[i];

				elements.push_back(pair);
				constant += -nablag->values[i] * hp.generatedPoint.at(nablag->indexes[i]);
			}
		}

		bool hyperplaneIsOk = true;

		for (auto E : elements)
		{
			if (E.value != E.value) //Check for NaN
			{
				processInfo->outputWarning("     Warning: hyperplane not generated, NaN found in linear terms!");

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

			//IloRange tmpRange2(this->getEnv(), -IloInfinity, expr, -constant);

			cplexLazyConstrs.push_back(tmpRange);
			//numLazyAdded++;

			std::cout << "     LAZY #: " << cplexLazyConstrs.size() << "\t DB: " << this->getBestObjValue()
					<< "\t OBJ: " << this->getObjValue() << "\t GAP: " << gap << "\t PB: "
					<< this->getIncumbentObjValue() << "\t DEV: " << mostDevConstr.value << std::endl;
			//this->add(tmpRange, IloCplex::CutManagement::UseCutPurge);
			this->add(tmpRange);
			//this->addLocal(tmpRange);
			//std::cout << "HEJ" << std::endl;
			//tmpRange2.end();

			//numLazyConstrs++;
			//cplexConstrs.add(tmpRange);
			//cplexModel.add(tmpRange);

			//int constrIndex = processInfo->MILPSolver->addLinearConstraint(elements, constant);
			/*addedHyperplanes++;
			 GeneratedHyperplane genHyperplane;

			 genHyperplane.generatedConstraintIndex = constrIndex;
			 genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
			 genHyperplane.generatedPoint = hyperplane.generatedPoint;
			 genHyperplane.source = hyperplane.source;
			 genHyperplane.generatedIter = currIter->iterationNumber;

			 generatedHyperplanes.push_back(genHyperplane);

			 currIter->numHyperplanesAdded++;
			 currIter->totNumHyperplanes++;*/
		}
	}
//}

//std::cout << "Before: " << primalBegin << " after " << processInfo->getPrimalBound() << std::endl;
	if (returnAfter || abs(primalBegin - processInfo->getPrimalBound()) > 0)
	{
		isBusy = false;
		return;
	}

	isBusy = false;

}

MILPSolverCplexExperimental::MILPSolverCplexExperimental()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	discreteVariablesActivated = true;

	cplexModel = IloModel(cplexEnv);

	cplexVars = IloNumVarArray(cplexEnv);
	cplexConstrs = IloRangeArray(cplexEnv);
	cplexLazyConstrs = IloRangeArray(cplexEnv);

	iterLastLazyConvert = 0;

	cachedSolutionHasChanged = true;

	checkParameters();
}

MILPSolverCplexExperimental::~MILPSolverCplexExperimental()
{
	cplexEnv.end();
	delete tCheckDualCands;
	delete tCheckPrimCands;
}

bool MILPSolverCplexExperimental::createLinearProblem(OptProblem * origProblem)
{
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
			processInfo->outputWarning("Error variable type " + to_string(tmpTypes.at(i)) + " for " + tmpNames.at(i));
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
		cplexInstance.use(CtCallback(cplexEnv, cplexVars, this->processInfo));
		//cplexInstance.use(HCallback(cplexEnv, cplexVars, this->processInfo));

	}
	catch (IloException& e)
	{
		std::cout << "CPLEX exception caught when creating model: " << e << std::endl;
		return (false);
	}

	return (true);
}

void MILPSolverCplexExperimental::initializeSolverSettings()
{
	firstNonLazyHyperplane = processInfo->originalProblem->getNumberOfLinearConstraints();

	try
	{
		// Disable CPLEX output
		cplexInstance.setOut(cplexEnv.getNullStream());
		cplexInstance.setWarning(cplexEnv.getNullStream());

		cplexInstance.setParam(IloCplex::SolnPoolIntensity, settings->getIntSetting("SolnPoolIntensity", "CPLEX")); // Don't use 3 with heuristics
		cplexInstance.setParam(IloCplex::SolnPoolReplace, settings->getIntSetting("SolnPoolReplace", "CPLEX"));

		cplexInstance.setParam(IloCplex::RepairTries, 5);
		//cplexInstance.setParam(IloCplex::HeurFreq,2);
		//cplexInstance.setParam(IloCplex::AdvInd,2);

		cplexInstance.setParam(IloCplex::SolnPoolGap, settings->getDoubleSetting("SolnPoolGap", "CPLEX"));
		cplexInstance.setParam(IloCplex::SolnPoolCapacity, settings->getIntSetting("SolutionPoolSize", "MILP"));

		cplexInstance.setParam(IloCplex::Probe, settings->getIntSetting("Probe", "CPLEX"));
		cplexInstance.setParam(IloCplex::MIPEmphasis, settings->getIntSetting("MIPEmphasis", "CPLEX"));

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
		std::cout << "Error when initializing parameters for linear solver: " << e << std::endl;
	}
}

int MILPSolverCplexExperimental::addLinearConstraint(std::vector<IndexValuePair> elements, double constant,
		bool isGreaterThan)
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

		/*if (settings->getBoolSetting("UseLazyConstraints", "MILP"))
		 {
		 if (discreteVariablesActivated)
		 {
		 cplexInstance.addLazyConstraint(tmpRange);
		 }
		 else
		 {
		 cplexModel.add(tmpRange);
		 cplexInstance.extract(cplexModel);
		 }
		 }
		 else
		 {
		 */

		modelUpdated = true;

		//}

		expr.end();
	}
	catch (IloException& e)
	{
		processInfo->outputError("Error when adding linear constraint:", e.getMessage());

		return (-1);
	}

	return (cplexInstance.getNrows() - 1);

	if (processInfo->getCurrentIteration()->iterationNumber - iterLastLazyConvert > 10)
	{
		std::vector<int> idxs;

		int percentage = max(20.0, ceil(processInfo->getCurrentIteration()->numHyperplanesAdded * 0.05));

		for (int i = this->firstNonLazyHyperplane; i < cplexConstrs.getSize() - percentage; i++)
		{
			idxs.push_back(i);
		}

		if (idxs.size() > 0)
		{
			this->firstNonLazyHyperplane = idxs.at(idxs.size() - 1) + 1;
			iterLastLazyConvert = processInfo->getCurrentIteration()->iterationNumber;
			lastLazyUpdateConstrSlacks = lastSolutionConstrSlacks;
		}
	}

	if (cplexLazyConstrs.getSize() > 0) cplexInstance.addLazyConstraints(cplexLazyConstrs);

	return (true);
}

void MILPSolverCplexExperimental::activateDiscreteVariables(bool activate)
{
	auto variableTypes = processInfo->originalProblem->getVariableTypes();

	try
	{
		for (int i = 0; i < cplexVarConvers.size(); i++)
		{
			cplexVarConvers.at(i).end();
		}

		cplexVarConvers.clear();

		if (activate)
		{
			processInfo->outputDebug("     Activating MILP strategy");

			for (int i = 0; i < processInfo->originalProblem->getNumberOfVariables(); i++)
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
			processInfo->outputDebug("     Activating LP strategy");
			for (int i = 0; i < processInfo->originalProblem->getNumberOfVariables(); i++)
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
		if (activate) processInfo->outputError("Error when activating discrete variables", e.getMessage());
		else processInfo->outputError("Error when deactivating discrete variables", e.getMessage());
	}
}

E_ProblemSolutionStatus MILPSolverCplexExperimental::getSolutionStatus()
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
			processInfo->outputError("MILP solver return status unknown = " + status);
			MILPSolutionStatus = E_ProblemSolutionStatus::Error;
		}
	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when obtaining solution status:", e.getMessage());

	}

	return (MILPSolutionStatus);
}

E_ProblemSolutionStatus MILPSolverCplexExperimental::solveProblem()
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
				processInfo->startTimer("LazyChange");
				cplexInstance.addLazyConstraints(cplexLazyConstrs);
				processInfo->stopTimer("LazyChange");
			}
			modelUpdated = false;
		}

		double timeStart = processInfo->getElapsedTime("Total");

		cplexInstance.solve();
		double timeEnd = processInfo->getElapsedTime("Total");

		iterDurations.push_back(timeEnd - timeStart);
		MILPSolutionStatus = getSolutionStatus();

	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when solving MILP/LP problem", e.getMessage());
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	stopTimer();

	return (MILPSolutionStatus);
}

int MILPSolverCplexExperimental::increaseSolutionLimit(int increment)
{
	int sollim;

	try
	{
		cplexInstance.setParam(IloCplex::IntSolLim, cplexInstance.getParam(cplexInstance.IntSolLim) + increment);
		sollim = cplexInstance.getParam(cplexInstance.IntSolLim);
	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when increasing solution limit", e.getMessage());
	}

	return (sollim);

}

void MILPSolverCplexExperimental::setSolutionLimit(int limit)
{

	if (processInfo->originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
	{
		limit = settings->getIntSetting("MILPSolLimitInitial", "MILP");
	}

	try
	{
		cplexInstance.setParam(IloCplex::IntSolLim, limit);
	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when setting solution limit", e.getMessage());
	}
}

int MILPSolverCplexExperimental::getSolutionLimit()
{
	int solLim = 0;

	try
	{
		solLim = cplexInstance.getParam(cplexInstance.IntSolLim);
	}
	catch (IloException &e)
	{

		processInfo->outputError("Error when obtaining solution limit", e.getMessage());

	}

	return (solLim);
}

std::vector<double> MILPSolverCplexExperimental::getVariableSolution(int solIdx)
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
		processInfo->outputError("Error when reading solution with index", e.getMessage());
	}

	return (solution);
}

int MILPSolverCplexExperimental::getNumberOfSolutions()
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
		processInfo->outputError("Error when obtaining number of solutions", e.getMessage());
	}

	return (numSols);
}

double MILPSolverCplexExperimental::getObjectiveValue(int solIdx)
{
	bool isMILP = getDiscreteVariableStatus();

	double objVal = NAN;

	if (!isMILP && solIdx > 0) // LP problems only have one solution!
	{

		processInfo->outputError(
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

		processInfo->outputError("Error when obtaining objective value for solution index " + to_string(solIdx),
				e.getMessage());

	}

	return (objVal);

}

void MILPSolverCplexExperimental::populateSolutionPool()
{
	processInfo->startTimer("PopulateSolutionPool");
	double initialPopulateTimeLimit = 0.5;
	double timeLimitIncreaseFactor = 2.0;

	try
	{
		int poolSizeBefore = cplexInstance.getSolnPoolNsolns();

		if (processInfo->getCurrentIteration()->iterationNumber == 0)
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

		double newSolnPoolGap = min(1.0e+75, processInfo->getAbsoluteObjectiveGap());
		cplexInstance.setParam(IloCplex::SolnPoolGap, newSolnPoolGap);

		cplexInstance.populate();

		int poolSizeAfter = cplexInstance.getSolnPoolNsolns();

		if (poolSizeAfter > poolSizeBefore)
		{

			processInfo->outputDebug(
					"     Solution pool populated from: " + to_string(poolSizeBefore) + " to "
							+ to_string(poolSizeAfter));
		}
	}
	catch (IloException &e)
	{

		processInfo->outputError("Error when populating solution pool", e.getMessage());

		processInfo->stopTimer("PopulateSolutionPool");

	}

	processInfo->stopTimer("PopulateSolutionPool");
}

void MILPSolverCplexExperimental::setTimeLimit(double seconds)
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
		processInfo->outputError("Error when setting time limit", e.getMessage());
	}
}

void MILPSolverCplexExperimental::setCutOff(double cutOff)
{
	try
	{
		if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			cplexInstance.setParam(IloCplex::CutUp, cutOff);
			processInfo->outputInfo("     Setting cutoff value to " + to_string(cutOff) + " for minimization.");

		}
		else
		{
			cplexInstance.setParam(IloCplex::CutLo, cutOff);
			processInfo->outputInfo("     Setting cutoff value to " + to_string(cutOff) + " for maximization.");

		}
	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when setting cut off value", e.getMessage());
	}
}

void MILPSolverCplexExperimental::addMIPStart(std::vector<double> point)
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
		processInfo->outputError("Error when adding MIP starting point", e.getMessage());
	}

	processInfo->outputInfo("     Added MIP starting point.");
}

void MILPSolverCplexExperimental::deleteMIPStarts()
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
			processInfo->outputError("Error when deleting MIP starting points", e.getMessage());
		}

		processInfo->outputInfo("    Deleted " + to_string(numStarts) + " MIP starting points.");
	}
}

void MILPSolverCplexExperimental::writeProblemToFile(std::string filename)
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
		processInfo->outputError("Error when saving model to file", e.getMessage());
	}
}

void MILPSolverCplexExperimental::changeConstraintToLazy(GeneratedHyperplane &hyperplane)
{
	try
	{
		//std::cout << "Starting conversion to lazy "<< std::endl;
		//std::cout << "Number start " << cplexConstrs.getSize()<< std::endl;

		//UtilityFunctions::displayVector(constrIdxs);
		//std::cout << "Converted to lazy: " << constrIdxs[i] << std::endl;
		IloRange tmpRange = cplexConstrs[hyperplane.generatedConstraintIndex];

		try
		{
			cplexModel.remove(tmpRange);
			cplexInstance.extract(cplexModel);
			//cplexInstance.addLazyConstraint(tmpRange);
			cplexLazyConstrs.add(tmpRange);
			modelUpdated = true;
			hyperplane.isLazy = true;
			hyperplane.convertedToLazyIter = processInfo->getCurrentIteration()->iterationNumber;

			processInfo->outputInfo(
					"     Changed constraint " + to_string(hyperplane.generatedConstraintIndex)
							+ "generated in iteration" + to_string(hyperplane.generatedIter) + "to lazy.");
		}
		catch (IloException &e)
		{
			processInfo->outputError(e.getMessage());
		}
	}
	catch (IloException &e)
	{
		processInfo->outputError(e.getMessage());
	}
}

void MILPSolverCplexExperimental::fixVariable(int varIndex, double value)
{
	updateVariableBound(varIndex, value, value);
}

void MILPSolverCplexExperimental::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
	try
	{
		cplexVars[varIndex].setBounds(lowerBound, upperBound);
		cplexInstance.extract(cplexModel);
	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when updating variable bounds for variable index" + to_string(varIndex),
				e.getMessage());
	}
}

pair<double, double> MILPSolverCplexExperimental::getCurrentVariableBounds(int varIndex)
{
	pair<double, double> tmpBounds;

	try
	{
		tmpBounds.first = cplexVars[varIndex].getLB();
		tmpBounds.second = cplexVars[varIndex].getUB();
	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when obtaining variable bounds for variable index" + to_string(varIndex),
				e.getMessage());
	}
	return (tmpBounds);
}

bool MILPSolverCplexExperimental::supportsQuadraticObjective()
{
	return (true);
}
bool MILPSolverCplexExperimental::supportsQuadraticConstraints()
{
	return (true);
}

double MILPSolverCplexExperimental::getDualObjectiveValue()
{

	bool isMILP = getDiscreteVariableStatus();
	double objVal = NAN;

	try
	{
		objVal = cplexInstance.getBestObjValue();

	}
	catch (IloException &e)
	{
		processInfo->outputError("Error when obtaining dual objective value", e.getMessage());
	}

	return (objVal);
}

bool MILPSolverCplexExperimental::supportsLazyConstraints()
{
	return (true);
}

void MILPSolverCplexExperimental::checkParameters()
{

}
