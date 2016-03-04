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
double minConstrDev;
bool useMinConstrDev;
int lazyBreakCtr;

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
			minConstrDev = 1000;
			useMinConstrDev = false;
			lazyBreakCtr = 500;
			settings = SHOTSettings::Settings::getInstance();

			if (settings->getIntSetting("LinesearchMethod", "Linesearch")
					== static_cast<int>(ES_LinesearchMethod::Boost))
			{
				this->processInfo->logger.message(2) << "Boost linesearch implementation selected" << CoinMessageEol;
				linesearchMethod = new LinesearchMethodBoost();
			}
			else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
					== static_cast<int>(ES_LinesearchMethod::Bisection))
			{
				this->processInfo->logger.message(2) << "Bisection linesearch selected" << CoinMessageEol;
				linesearchMethod = new LinesearchMethodBisection();
			}
		}
		void main();	// the call back function
};

IloCplex::Callback CtCallback(IloEnv env, IloNumVarArray cplexVars, ProcessInfo *pInfo)
{
	return (IloCplex::Callback(new (env) CtCallbackI(env, cplexVars, pInfo)));
}

void CtCallbackI::main()
{
	//return;
	int currIterNum = processInfo->getCurrentIteration()->iterationNumber;

	while (isBusy)
	{
		sleep(0.000001);
	}

	isBusy = true;

	if (lazyiters == 50)
	{

		if (minConstrDev > settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
		{
			useMinConstrDev = true;

		}
	}

	if (lazyiters == lazyBreakCtr)
	{
		lazyBreakCtr = lazyBreakCtr * 1.2;
		minConstrDev = 1000;
		lazyiters = 0;
		isBusy = false;
		return;
	}
	else
	{
		lazyiters++;
	}

	/*
	 if (processInfo->lastLazyAddedIter < currIterNum)
	 {
	 //numLazyAdded = 0;

	 //		int numAdd = min(300,(int)cplexLazyConstrs.size());
	 int numAdd = 0.2 * cplexLazyConstrs.size();
	 int numAdd2 = 0.2 * cplexLazyConstrs.size();

	 for (int i = 0; i < numAdd; i++)
	 {
	 this->add(cplexLazyConstrs.at(cplexLazyConstrs.size() - i - 1));
	 }

	 for (int i = 0; i < numAdd2; i++)
	 {
	 this->add(cplexLazyConstrs.at(i));
	 }

	 std::cout << "Added " << numAdd + numAdd2 << " lazy constraints!" << std::endl;

	 processInfo->lastLazyAddedIter = currIterNum;

	 }
	 */
//	if (numLazyAdded > 500) return;
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

	minConstrDev = min(mostDevConstr.value, minConstrDev);
	/*if (mostDevConstr.value <= settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	 {
	 processInfo->addPrimalSolutionCandidate(solution, E_PrimalSolutionSource::LazyConstraintCallback, currIterNum);
	 //		processInfo->logger.message(1) << "MILP point is on the boundary or interior!" << CoinMessageEol;
	 //	return;
	 }*/

	if (mostDevConstr.value <= settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		//		processInfo->logger.message(1) << "MILP point is on the interior!" << CoinMessageEol;
		processInfo->addPrimalSolutionCandidate(solution, E_PrimalSolutionSource::LazyConstraintCallback, currIterNum);
	}

	if (mostDevConstr.value <= settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm") / 10.0)
	{
//		processInfo->logger.message(1) << "MILP point is on the interior!" << CoinMessageEol;
		//processInfo->addPrimalSolutionCandidate(solution, E_PrimalSolutionSource::LazyConstraintCallback, currIterNum);
		isBusy = false;
		return;
	}
	/*else if (lazyiters > 100 && mostDevConstr.value < minConstrDev * 10.0)
	 {
	 processInfo->logger.message(1) << "Using minconstrdev!" << CoinMessageEol;
	 isBusy = false;
	 return;
	 }*/
	/*else if(mostDevConstr.value > 0 && mostDevConstr.value < 10^(-12)) //Kommentera för tls
	 {
	 processInfo->logger.message(1) << "MILP point is on the boundary!" << CoinMessageEol;

	 isBusy = false;
	 return;
	 }*/
	else
	{
		/*Hyperplane hyperplane;
		 hyperplane.sourceConstraintIndex = tmpMostDevConstr.idx;
		 hyperplane.generatedPoint = solution;*/

		Hyperplane hyperplane;

		for (int j = 0; j < processInfo->interiorPts.size(); j++)
		{
			//processInfo->startTimer(E_TimerTypes::Linesearch);
			auto xNLP = processInfo->interiorPts.at(j).point;

			processInfo->startTimer("HyperplaneLinesearch");
			auto xNewc = linesearchMethod->findZero(xNLP, solution,
					settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
					settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"));

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
				processInfo->logger.message(1) << "Hyperplane point is on the interior." << CoinMessageEol;

				return;
			}
			else
			{
				processInfo->logger.message(6) << "Hyperplane point is on the exterior." << CoinMessageEol;

				hyperplane.sourceConstraintIndex = tmpMostDevConstr.idx;
				hyperplane.generatedPoint = xNewc;

			}
		}

		/*
		 if (i == 0 && currIter->isMILP())
		 {
		 hyperplane.source = E_HyperplaneSource::MIPOptimalSolutionPoint;
		 }
		 else if (currIter->isMILP())
		 {
		 hyperplane.source = E_HyperplaneSource::MIPSolutionPoolSolutionPoint;
		 }
		 else
		 {
		 hyperplane.source = E_HyperplaneSource::LPRelaxedSolutionPoint;
		 }*/

		//processInfo->hyperplaneWaitingList.push_back(hyperplane);
		auto varNames = processInfo->originalProblem->getVariableNames();
		/*
		 processInfo->logger.message(1) << " HP point is: " << CoinMessageEol;


		 for (int i = 0; i < point.size(); i++)
		 {
		 processInfo->logger.message(3) << "  " << varNames.at(i) << ": " << point[i] << CoinMessageEol;
		 }*/

		std::vector < IndexValuePair > elements;
		double constant = processInfo->originalProblem->calculateConstraintFunctionValue(
				hyperplane.sourceConstraintIndex, hyperplane.generatedPoint);

		if (hyperplane.sourceConstraintIndex == -1
				|| hyperplane.sourceConstraintIndex
						== processInfo->originalProblem->getNonlinearObjectiveConstraintIdx())
		{
			processInfo->logger.message(3) << " HP point generated for auxiliary objective function constraint"
					<< CoinMessageEol;

			auto tmpArray = processInfo->originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
					&hyperplane.generatedPoint.at(0), -1, true);
			int number = processInfo->originalProblem->getNumberOfVariables();

			for (int i = 0; i < number - 1; i++)
			{
				if (tmpArray[i] != 0)
				{
					IndexValuePair pair;
					pair.idx = i;
					pair.value = tmpArray[i];

					elements.push_back(pair);
					constant += -tmpArray[i] * hyperplane.generatedPoint.at(i);

					processInfo->logger.message(3) << " Gradient for variable" << varNames.at(i) << ": " << tmpArray[i]
							<< CoinMessageEol;
				}
			}

			processInfo->logger.message(3) << " Gradient for obj.var.: -1" << CoinMessageEol;

			IndexValuePair pair;
			pair.idx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();
			pair.value = -1.0;

			elements.push_back(pair);
			constant += /*-(-1) **/hyperplane.generatedPoint.at(pair.idx);
		}
		else
		{
			processInfo->logger.message(3) << " HP point generated for constraint index"
					<< hyperplane.sourceConstraintIndex << CoinMessageEol;

			auto nablag = processInfo->originalProblem->calculateConstraintFunctionGradient(
					hyperplane.sourceConstraintIndex, hyperplane.generatedPoint);
			processInfo->numGradientEvals++;

			for (int i = 0; i < nablag->number; i++)
			{
				IndexValuePair pair;
				pair.idx = nablag->indexes[i];
				pair.value = nablag->values[i];

				elements.push_back(pair);
				constant += -nablag->values[i] * hyperplane.generatedPoint.at(nablag->indexes[i]);

				processInfo->logger.message(3) << " Gradient for variable" << varNames.at(nablag->indexes[i]) << ": "
						<< nablag->values[i] << CoinMessageEol;
			}
		}

		/*
		 for (auto E : elements)
		 {
		 processInfo->logger.message(3) << " HP coefficient for variable " << varNames.at(E.idx) << ": " << E.value
		 << CoinMessageEol;
		 }

		 processInfo->logger.message(3) << " HP constant " << constant << CoinMessageEol;
		 */

		bool hyperplaneIsOk = true;

		for (auto E : elements)
		{
			if (E.value != E.value) //Check for NaN
			{
				processInfo->logger.message(0) << "Warning: hyperplane not generated, NaN found in linear terms!"
						<< CoinMessageEol;
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

			std::cout << "     LAZY OBJ: " << this->getObjValue() << "\t BEST: " << this->getBestObjValue()
					<< "\t\t DEV: " << mostDevConstr.value << " < " << minConstrDev << " NUM: "
					<< cplexLazyConstrs.size() << std::endl;
			this->add(tmpRange);
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
			processInfo->logger.message(1) << "Error variable type " << tmpTypes.at(i) << " for "
					<< tmpNames.at(i).c_str() << CoinMessageEol;
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

		cplexInstance.use(CtCallback(cplexEnv, cplexVars, this->processInfo));
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

		cplexInstance.setParam(IloCplex::SolnPoolIntensity, settings->getIntSetting("SolnPoolIntensity", "CPLEX"));	// Don't use 3 with heuristics
		cplexInstance.setParam(IloCplex::SolnPoolReplace, settings->getIntSetting("SolnPoolReplace", "CPLEX"));

		cplexInstance.setParam(IloCplex::RepairTries, 5);
		//cplexInstance.setParam(IloCplex::HeurFreq,2);
		//cplexInstance.setParam(IloCplex::AdvInd,2);

		cplexInstance.setParam(IloCplex::SolnPoolGap, settings->getDoubleSetting("SolnPoolGap", "CPLEX"));
		cplexInstance.setParam(IloCplex::SolnPoolCapacity,
				settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm"));

		cplexInstance.setParam(IloCplex::Probe, settings->getIntSetting("Probe", "CPLEX"));
		cplexInstance.setParam(IloCplex::MIPEmphasis, settings->getIntSetting("MIPEmphasis", "CPLEX"));

		//cplexInstance.setParam(IloCplex::ParallelMode,1);
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
		processInfo->logger.message(2) << "Error when adding linear constraint:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

		return (-1);
	}

	return (cplexInstance.getNrows() - 1);

//std::cout << "Last nonlazy: " << this->firstNonLazyHyperplane << std::endl;

//if (processInfo->getCurrentIteration()->iterationNumber == 10)
//if (settings->getBoolSetting("DelayedConstraints", "MILP") && currIter->isMILP() && currIter->MILPSolutionLimitUpdated)

	if (processInfo->getCurrentIteration()->iterationNumber - iterLastLazyConvert > 10)
	{
		//if (!processInfo->getCurrentIteration()->isMILP() || !(settings->getBoolSetting("DelayedConstraints", "MILP") && processInfo->getCurrentIteration()->MILPSolutionLimitUpdated))
		//{
		std::vector<int> idxs;

		//int numCons = processInfo->originalProblem->getNumberOfConstraints();

		/*idxs.push_back(1);
		 idxs.push_back(2);
		 idxs.push_back(3);*/

		int percentage = max(20.0, ceil(processInfo->getCurrentIteration()->numHyperplanesAdded * 0.05));

		//std::cout << "perc: " << percentage << std::endl;

		for (int i = this->firstNonLazyHyperplane; i < cplexConstrs.getSize() - percentage; i++)
		{
			idxs.push_back(i);
		}

		/*
		 //UtilityFunctions::displayVector(lastSolutionConstrSlacks);
		 signed int numCheck = (lastSolutionConstrSlacks.size() - percentage);

		 //std::cout << "perc: " << numCheck << std::endl;

		 for (int i = 0; i < numCheck; i++)
		 {
		 //std::cout << i << std::endl;
		 if (abs(lastSolutionConstrSlacks.at(i)) > 1)
		 {
		 idxs.push_back(this->firstNonLazyHyperplane + i);

		 }

		 std::cout << "Slack: " << abs(lastSolutionConstrSlacks.at(i)) << std::endl;
		 }*/

		//std::cout << "finished: "<< std::endl;
		/*for (int i = 0; i < lastLazyUpdateConstrSlacks.size(); i++)
		 {
		 if (abs(lastLazyUpdateConstrSlacks.at(i)-lastSolutionConstrSlacks.at(i)) > 1.0)
		 {
		 idxs.push_back(this->firstNonLazyHyperplane + i);
		 std::cout << "Slack change: " << abs(lastLazyUpdateConstrSlacks.at(i)-lastSolutionConstrSlacks.at(i)) << std::endl;
		 }
		 }*/

		if (idxs.size() > 0)
		{
			//changeConstraintsToLazy(idxs);

			this->firstNonLazyHyperplane = idxs.at(idxs.size() - 1) + 1;
			iterLastLazyConvert = processInfo->getCurrentIteration()->iterationNumber;
			lastLazyUpdateConstrSlacks = lastSolutionConstrSlacks;
		}
		//}
	}
//UtilityFunctions::displayVector(idxs);

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
			processInfo->logger.message(3) << "Activating MILP strategy" << CoinMessageEol;

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
			processInfo->logger.message(3) << "Activating LP strategy" << CoinMessageEol;
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
		if (activate) processInfo->logger.message(2) << "Error when activating discrete variables:"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
		else processInfo->logger.message(2) << "Error when deactivating discrete variables:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;
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
			processInfo->logger.message(1) << "MILP solver return status unknown = " << status << CoinMessageEol;
			MILPSolutionStatus = E_ProblemSolutionStatus::Error;
		}
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when obtaining solution status:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;

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

		/*
		 int currSolLim = getSolutionLimit();

		 IloInt tunestat = cplexInstance.tuneParam();

		 setSolutionLimit(currSolLim);
		 setTimeLimit(10000);

		 try
		 {
		 cplexInstance.writeParam("test.param");
		 }
		 catch (IloException &e)
		 {
		 processInfo->logger.message(0) << "Error when saving parameters to file:" << CoinMessageNewline
		 << e.getMessage() << CoinMessageEol;

		 }*/

		/*if (tunestat == IloCplex::TuningComplete) cout << "Tuning complete." << endl;
		 else if (tunestat == IloCplex::TuningAbort) cout << "Tuning abort." << endl;
		 else if (tunestat == IloCplex::TuningTimeLim) cout << "Tuning time limit." << endl;
		 else cout << "Tuning status unknown." << endl;
		 */
		processInfo->logger.message(3) << " Solving MILP..." << CoinMessageEol;
		double timeStart = processInfo->getElapsedTime("Total");

		cplexInstance.solve();
		double timeEnd = processInfo->getElapsedTime("Total");

		iterDurations.push_back(timeEnd - timeStart);
		processInfo->logger.message(3) << " MILP solved..." << CoinMessageEol;
		MILPSolutionStatus = getSolutionStatus();
		processInfo->logger.message(3) << " Solution status obtained.." << CoinMessageEol;

	}
	catch (IloException &e)
	{
		processInfo->logger.message(2) << "Error when solving MILP/LP problem:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	stopTimer();

	return (MILPSolutionStatus);
}
/*
 double MILPSolverCplexExperimental::getObjectiveValue()
 {
 double objval = getObjectiveValue(0);
 return (objval);
 }*/

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
		processInfo->logger.message(0) << "Error when increasing solution limit:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;

	}

	return (sollim);

}

void MILPSolverCplexExperimental::setSolutionLimit(int limit)
{

	if (processInfo->originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
	{
		limit = 1;
	}

	try
	{
		cplexInstance.setParam(IloCplex::IntSolLim, limit);
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when setting solution limit:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

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
		processInfo->logger.message(0) << "Error when obtaining solution limit:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}

	return (solLim);
}

/*
 std::vector<SolutionPoint> MILPSolverCplexExperimental::getAllVariableSolutions()
 {
 return (MILPSolverBase::getAllVariableSolutions());
 }*/

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
		processInfo->logger.message(0) << "Error when reading solution with index " << solIdx << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
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
		processInfo->logger.message(0) << "Error when obtaining number of solutions:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;

	}

	return (numSols);
}

double MILPSolverCplexExperimental::getObjectiveValue(int solIdx)
{
	bool isMILP = getDiscreteVariableStatus();

	double objVal = NAN;

	if (!isMILP && solIdx > 0) // LP problems only have one solution!
	{
		processInfo->logger.message(0) << "Cannot obtain solution with index " << solIdx
				<< " since the problem is LP/QP!" << CoinMessageEol;

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
		processInfo->logger.message(0) << "Error when obtaining objective value for solution index " << solIdx << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;

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
			processInfo->logger.message(3) << "    Solution pool populated from: " << poolSizeBefore << " to "
					<< poolSizeAfter << CoinMessageEol;
		}
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when populating solution pool:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;
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
		processInfo->logger.message(0) << "Error when setting time limit:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}
}

void MILPSolverCplexExperimental::setCutOff(double cutOff)
{
	try
	{
		if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			cplexInstance.setParam(IloCplex::CutUp, cutOff);
			processInfo->logger.message(3) << "Setting cutoff value to " << cutOff << " for minimization."
					<< CoinMessageEol;
		}
		else
		{
			cplexInstance.setParam(IloCplex::CutLo, cutOff);
			processInfo->logger.message(3) << "Setting cutoff value to " << cutOff << " for maximization."
					<< CoinMessageEol;
		}
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when setting cut off value:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

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
		processInfo->logger.message(1) << "Error when adding MIP starting point:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;
	}

	processInfo->logger.message(3) << "Added MIP starting point" << CoinMessageEol;

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
			processInfo->logger.message(1) << "Error when deleting MIP starting points:" << CoinMessageNewline
					<< e.getMessage() << CoinMessageEol;
		}

		processInfo->logger.message(3) << "Deleted " << numStarts << " MIP starting points" << CoinMessageEol;
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
		processInfo->logger.message(0) << "Error when saving model to file:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

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

			processInfo->logger.message(3) << "    Changed constraint " << hyperplane.generatedConstraintIndex
					<< "generated in iteration" << hyperplane.generatedIter << "to lazy." << CoinMessageEol;
		}
		catch (IloException &e)
		{
			processInfo->logger.message(1) << e.getMessage() << CoinMessageEol;
		}

		//std::cout << "End  conversion to lazy "<< std::endl;
		//std::cout << "Number stop " << cplexConstrs.getSize()<< std::endl;
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << e.getMessage() << CoinMessageEol;
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
		processInfo->logger.message(0) << "Error when updating variable bounds for variable index" << varIndex << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
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
		processInfo->logger.message(0) << "Error when obtaining variable bounds for variable index" << varIndex << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
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
		processInfo->logger.message(0) << "Error when obtaining dual objective value: " << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;

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
