#include "IMILPSolver.h"
#include "MILPSolverCplex.h"
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

MILPSolverCplex::MILPSolverCplex()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	discreteVariablesActivated = true;

	cplexModel = IloModel(cplexEnv);

	cplexVars = IloNumVarArray(cplexEnv);
	cplexConstrs = IloRangeArray(cplexEnv);
	cplexLazyConstrs = IloRangeArray(cplexEnv);

	firstNonLazyHyperplane = processInfo->originalProblem->getNumberOfLinearConstraints();
	iterLastLazyConvert = 0;

}

MILPSolverCplex::~MILPSolverCplex()
{
	cplexEnv.end();
}

/*
 bool MILPSolverCplex::createLinearProblem(OptProblem* origProblem)
 {
 // Adds the variables
 auto numVars = origProblem->getNumberOfVariables();

 auto tmpLBs = origProblem->getVariableLowerBounds();
 auto tmpUBs = origProblem->getVariableUpperBounds();
 auto tmpNames = origProblem->getVariableNames();
 auto tmpTypes = origProblem->getVariableTypes();

 //std::vector<double> tmpLBs(origInstance->getVariableLowerBounds());
 //std::vector<double> tmpUBs(origInstance->getVariableUpperBounds());
 //std::vector < string > tmpNames(origInstance->getVariableNames());
 //std::vector<char> tmpTypes(origInstance->getVariableTypes());

 for (int i = 0; i < numVars; i++)
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
 else
 {
 processInfo->logger.message(1) << "Error variable type for " << tmpNames.at(i).c_str() << CoinMessageEol;
 }
 }

 cplexModel.add(cplexVars);

 IloExpr objExpr(cplexEnv);

 auto tmpObjPairs = processInfo->originalProblem->getObjectiveFunctionVarCoeffPairs();



 if (processInfo->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
 {
 auto quadTerms = processInfo->originalProblem->getQuadraticTermsInConstraint(-1);

 for (auto T : quadTerms)
 {
 objExpr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
 }
 }

 double objConstant = origInstance->getObjectiveConstants()[0];

 if (objConstant != 0.0) objExpr += objConstant;

 if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
 {
 cplexModel.add(IloMinimize(cplexEnv, objExpr));
 }
 else
 {
 cplexModel.add(IloMaximize(cplexEnv, objExpr));
 }

 objExpr.end();

 int numCon = processInfo->originalProblem->getNumberOfConstraints();

 if (processInfo->originalProblem->isObjectiveFunctionNonlinear()) numCon--;

 int row_nonz = 0;
 int obj_nonz = 0;
 int varIdx = 0;

 SparseMatrix *m_linearConstraintCoefficientsInRowMajor =
 processInfo->originalProblem->getProblemInstance()->getLinearConstraintCoefficientsInRowMajor();

 auto constrTypes = processInfo->originalProblem->getProblemInstance()->getConstraintTypes();
 auto constrNames = processInfo->originalProblem->getProblemInstance()->getConstraintNames();
 auto constrLBs = processInfo->originalProblem->getProblemInstance()->getConstraintLowerBounds();
 auto constrUBs = processInfo->originalProblem->getProblemInstance()->getConstraintUpperBounds();

 try
 {
 for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
 {
 // Only use constraints that don't contain a nonlinear part (may include a quadratic part)
 if (!processInfo->originalProblem->isConstraintNonlinear(rowIdx))
 {
 IloExpr expr(cplexEnv);

 if (processInfo->originalProblem->getProblemInstance()->instanceData->linearConstraintCoefficients
 != NULL
 && processInfo->originalProblem->getProblemInstance()->instanceData->linearConstraintCoefficients->numberOfValues
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

 auto quadTerms = processInfo->originalProblem->getQuadraticTermsInConstraint(rowIdx);

 for (auto T : quadTerms)
 {
 expr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
 //std::cout << "Objective term in constraints!" << std::endl;
 }

 expr +=
 processInfo->originalProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

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
 }
 catch (IloAlgorithm::CannotChangeException& e)
 {
 std::cerr << "CannotChangeException:" << e << std::endl;
 IloExtractableArray& es = e.getExtractables();

 for (IloInt i = 0; i < es.getSize(); ++i)
 std::cerr << "  " << i << ": " << es[i] << std::endl;
 }

 try
 {
 cplexInstance = IloCplex(cplexModel);
 cplexInstance.setOut(cplexEnv.getNullStream());
 cplexInstance.setWarning(cplexEnv.getNullStream());

 cplexInstance.setParam(IloCplex::SolnPoolIntensity, 2); // Don't use 3 with heuristics
 cplexInstance.setParam(IloCplex::SolnPoolReplace, 2);

 cplexInstance.setParam(IloCplex::RepairTries, 5);
 //cplexInstance.setParam(IloCplex::HeurFreq,2);
 //		cplexInstance.setParam(IloCplex::AdvInd,2);

 cplexInstance.setParam(IloCplex::SolnPoolGap, 0.001);
 cplexInstance.setParam(IloCplex::SolnPoolCapacity,
 settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm"));

 //cplexInstance.setParam(IloCplex::PopulateLim, 10);

 //cplexInstance.setParam(IloCplex::SolnPoolGap, 0);

 //cplexInstance.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.1);
 //cplexInstance.setParam(IloCplex::WorkMem, 1900);

 cplexInstance.setParam(IloCplex::NodeFileInd, 2);

 //cplexInstance.setParam(IloCplex::NumericalEmphasis, 1);

 //cplexInstance.setParam(IloCplex::WorkDir, "c:\\cplextemp\\");

 //cplexInstance.setParam(IloCplex::MemoryEmphasis, 1);

 //cplexInstance.setParam(IloCplex::NodeSel, CPX_NODESEL_BESTEST);

 cplexInstance.setParam(IloCplex::IntSolLim, 2100000000);
 //cplexInstance.setParam(IloCplex::ParallelMode, 1);
 //cplexInstance.setParam(IloCplex::EpMrk, 0.9);

 }
 catch (IloException& e)
 {
 std::cout << "Concert exception caught: " << e << std::endl;
 }
 catch (...)
 {
 std::cerr << "Unknown exception caught" << std::endl;
 }

 return true;
 }*/

bool MILPSolverCplex::createLinearProblem(OptProblem * origProblem)
{
// Adds the variables
	auto num = origProblem->getNumberOfVariables();

	auto tmpLBs = origProblem->getVariableLowerBounds();
	auto tmpUBs = origProblem->getVariableUpperBounds();
	auto tmpNames = origProblem->getVariableNames();

	auto tmpTypes = origProblem->getVariableTypes();

	for (int i = 0; i < origProblem->getNumberOfVariables(); i++)
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
		else
		{
			processInfo->logger.message(1) << "Error variable type for " << tmpNames.at(i).c_str() << CoinMessageEol;
		}
	}

	cplexModel.add(cplexVars);

	IloExpr objExpr(cplexEnv);

	auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

	for (int i = 0; i < tmpObjPairs.size(); i++)
	{
		objExpr += tmpObjPairs.at(i).second * cplexVars[tmpObjPairs.at(i).first];
	}

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

	int numCon = origProblem->getNumberOfConstraints();

	if (origProblem->isObjectiveFunctionNonlinear()) numCon--;

	int row_nonz = 0;
	int obj_nonz = 0;
	int varIdx = 0;

	SparseMatrix *m_linearConstraintCoefficientsInRowMajor =
			origProblem->getProblemInstance()->getLinearConstraintCoefficientsInRowMajor();

	auto constrTypes = origProblem->getProblemInstance()->getConstraintTypes();
	auto constrNames = origProblem->getProblemInstance()->getConstraintNames();
	auto constrLBs = origProblem->getProblemInstance()->getConstraintLowerBounds();
	auto constrUBs = origProblem->getProblemInstance()->getConstraintUpperBounds();

	try
	{
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

				auto quadTerms = origProblem->getQuadraticTermsInConstraint(rowIdx);

				for (auto T : quadTerms)
				{
					expr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
					//std::cout << "Objective term in constraints!" << std::endl;
				}

				expr += origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

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
	}
	catch (IloAlgorithm::CannotChangeException& e)
	{
		std::cerr << "CannotChangeException:" << e << std::endl;
		IloExtractableArray& es = e.getExtractables();

		for (IloInt i = 0; i < es.getSize(); ++i)
			std::cerr << "  " << i << ": " << es[i] << std::endl;
	}

	try
	{
		cplexInstance = IloCplex(cplexModel);
		//cplexInstance.setOut(cplexEnv.getNullStream());
		//cplexInstance.setWarning(cplexEnv.getNullStream());

		cplexInstance.setParam(IloCplex::SolnPoolIntensity, settings->getIntSetting("SolnPoolIntensity", "CPLEX")); // Don't use 3 with heuristics
		cplexInstance.setParam(IloCplex::SolnPoolReplace, settings->getIntSetting("SolnPoolReplace", "CPLEX"));

		cplexInstance.setParam(IloCplex::RepairTries, 5);
		//cplexInstance.setParam(IloCplex::HeurFreq,2);
//		cplexInstance.setParam(IloCplex::AdvInd,2);

		cplexInstance.setParam(IloCplex::SolnPoolGap, settings->getDoubleSetting("SolnPoolGap", "CPLEX"));
		cplexInstance.setParam(IloCplex::SolnPoolCapacity,
				settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm"));

		//cplexInstance.setParam(IloCplex::PopulateLim, 10);

		//cplexInstance.setParam(IloCplex::SolnPoolGap, 0);

		//cplexInstance.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.1);
		//cplexInstance.setParam(IloCplex::WorkMem, 1900);

		cplexInstance.setParam(IloCplex::NodeFileInd, 2);

		//cplexInstance.setParam(IloCplex::NumericalEmphasis, 1);

		//cplexInstance.setParam(IloCplex::WorkDir, "c:\\cplextemp\\");

		//cplexInstance.setParam(IloCplex::MemoryEmphasis, 1);

		//cplexInstance.setParam(IloCplex::NodeSel, CPX_NODESEL_BESTEST);

		cplexInstance.setParam(IloCplex::IntSolLim, 2100000000);
		//cplexInstance.setParam(IloCplex::ParallelMode, 1);
		//cplexInstance.setParam(IloCplex::EpMrk, 0.9);

	}
	catch (IloException& e)
	{
		std::cout << "Concert exception caught: " << e << std::endl;
	}
	catch (...)
	{
		std::cerr << "Unknown exception caught" << std::endl;
	}

	return true;
}

bool MILPSolverCplex::addLinearConstraint(std::vector<IndexValuePair> elements, int numNonZero, double constant)
{
	IloExpr expr(cplexEnv);

	for (int i = 0; i < numNonZero; i++)
	{
		expr += elements.at(i).value * cplexVars[elements.at(i).idx];
	}

	IloRange tmpRange(cplexEnv, -IloInfinity, expr, -constant);
	cplexConstrs.add(tmpRange);

	if (settings->getBoolSetting("UseLazyConstraints", "MILP"))
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
		cplexModel.add(tmpRange);
		cplexInstance.extract(cplexModel);
	}

	expr.end();

	return true;

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
		/*
		 int percentage = max(20.0,ceil(processInfo->getCurrentIteration()->numHyperplanesAdded * 0.05));

		 //std::cout << "perc: " << percentage << std::endl;


		 for (int i=this->firstNonLazyHyperplane; i < cplexConstrs.getSize()-percentage; i++)
		 {
		 idxs.push_back(i);
		 }*/

		//UtilityFunctions::displayVector(lastSolutionConstrSlacks);
		/*signed int numCheck =(lastSolutionConstrSlacks.size()-percentage);

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
			changeConstraintToLazy(idxs);

			this->firstNonLazyHyperplane = idxs.at(idxs.size() - 1) + 1;
			iterLastLazyConvert = processInfo->getCurrentIteration()->iterationNumber;
			lastLazyUpdateConstrSlacks = lastSolutionConstrSlacks;
		}
		//}
	}
//UtilityFunctions::displayVector(idxs);

	if (cplexLazyConstrs.getSize() > 0) cplexInstance.addLazyConstraints(cplexLazyConstrs);

	return true;
}

/*

 bool MILPSolverCplex::addLinearConstraint(std::vector<IndexValuePair> elements, int numNonZero, double constant)
 {
 IloExpr expr(cplexEnv);

 for (int i = 0; i < numNonZero; i++)
 {
 expr += elements.at(i).value * cplexVars[elements.at(i).idx];
 }

 IloRange tmpRange(cplexEnv, -IloInfinity, expr, -constant);
 cplexConstrs.add(tmpRange);

 if (settings->getBoolSetting("UseLazyConstraints", "MILP"))
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
 cplexModel.add(tmpRange);
 cplexInstance.extract(cplexModel);
 }

 expr.end();

 return true;
 }
 */

std::vector<double> MILPSolverCplex::getVariableSolution()
{
	std::vector<double> tmpSlacks;

//std::cout << "slack test"<< std::endl;

//tmpSlacks.reserve(cplexConstrs.getSize()-processInfo->originalProblem->getNumberOfConstraints());
//std::cout << "slack test"<< std::endl;
	/*for (int i = this->firstNonLazyHyperplane; i < cplexConstrs.getSize(); i++)
	 {
	 //std::cout << "slack test2"<< std::endl;
	 tmpSlacks.push_back(cplexInstance.getSlack(cplexConstrs[i]));
	 //std::cout << "slack " << i << ": " << tmpSlacks << std::endl;
	 }*/

//std::cout << "slack test2"<< std::endl;
//lastSolutionConstrSlacks = tmpSlacks;
//if (lastLazyUpdateConstrSlacks.size() == 0) lastLazyUpdateConstrSlacks = tmpSlacks;
//int numVar = processInfo->originalProblem->getNumberOfVariables();
	int numVar = cplexVars.getSize();
	std::vector<double> solution(numVar);

	IloNumArray tmpSols(cplexEnv);

	try
	{

		cplexInstance.getValues(cplexVars, tmpSols);

		for (int i = 0; i < numVar; i++)
		{
			solution.at(i) = tmpSols[i];
		}
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when obtaining variable solution:" << e.getMessage() << CoinMessageEol;

		auto numSols = cplexInstance.getSolnPoolNsolns();

		if (numSols > 0)
		{
			std::vector<double> solution(numVar);

			IloNumArray tmpSols(cplexEnv);

			cplexInstance.getValues(cplexVars, tmpSols, 0);

			for (int i = 0; i < numVar; i++)
			{
				solution.at(i) = tmpSols[i];
			}

		}

	}

	return solution;
}

void MILPSolverCplex::activateDiscreteVariables(bool activate)
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

		cout << e << endl;
	}
}

E_ProblemSolutionStatus MILPSolverCplex::getSolutionStatus()
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
		/*else if (status == GRB_NODE_LIMIT)
		 {
		 MILPSolutionStatus = EMILPStatus::node_limit;
		 }
		 else if (status == GRB_TIME_LIMIT)
		 {
		 MILPSolutionStatus = EMILPStatus::time_limit;
		 }
		 else if (status == GRB_SOLUTION_LIMIT)
		 {
		 MILPSolutionStatus = EMILPStatus::solution_limit;
		 }
		 else if (status == IloAlgorithm::Status)
		 {
		 MILPSolutionStatus = EMILPStatus::interrupted;
		 }
		 else if (status == GRB_NUMERIC)
		 {
		 MILPSolutionStatus = EMILPStatus::numeric;
		 }
		 else if (status == GRB_SUBOPTIMAL)
		 {
		 MILPSolutionStatus = EMILPStatus::suboptimal;
		 }
		 else if (status == GRB_INPROGRESS)
		 {
		 MILPSolutionStatus = EMILPStatus::in_progress;
		 }*/
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

E_ProblemSolutionStatus MILPSolverCplex::solveProblem()
{
	startTimer();

	E_ProblemSolutionStatus MILPSolutionStatus;

	if (processInfo->getPrimalBound() < DBL_MAX) setCutOff(processInfo->getPrimalBound());

	if (getDiscreteVariableStatus() && processInfo->primalSolution.size() > 0) addMIPStart(processInfo->primalSolution);

	try
	{
		processInfo->logger.message(4) << " Solving MILP..." << CoinMessageEol;
		cplexInstance.solve();

		processInfo->logger.message(4) << " MILP solved..." << CoinMessageEol;
		MILPSolutionStatus = getSolutionStatus();

	}
	catch (IloException &e)
	{
		processInfo->logger.message(2) << "Error when solving MILP/LP problem:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	stopTimer();
	return MILPSolutionStatus;
}

double MILPSolverCplex::getLastObjectiveValue()
{
	double objval;
	try
	{
		objval = cplexInstance.getObjValue();
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when obtaining objective value:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;
		processInfo->logger.message(0) << cplexInstance.getBestObjValue() << CoinMessageEol;

	}

	return objval;
}

double MILPSolverCplex::getBestObjectiveValue()
{
	double objval;
	try
	{
		objval = cplexInstance.getBestObjValue();
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when obtaining best objective value:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;

	}

	return objval;
}

int MILPSolverCplex::increaseSolutionLimit(int increment)
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

	return sollim;

}

void MILPSolverCplex::setSolutionLimit(int limit)
{
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

int MILPSolverCplex::getSolutionLimit()
{
	int solLim;

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

std::vector<std::vector<double>> MILPSolverCplex::getAllVariableSolutions()
{
	std::vector < std::vector<double> > allSolutions;
	try
	{
		if (getDiscreteVariableStatus() && settings->getBoolSetting("PopulateSolutionPool", "MILP"))
		{
			int poolSizeBefore = cplexInstance.getSolnPoolNsolns();

			auto tmpTimeLim = cplexInstance.getParam(IloCplex::TiLim);
			setTimeLimit(0.5);
			cplexInstance.populate();
			//setTimeLimit(tmpTimeLim);

			int poolSizeAfter = cplexInstance.getSolnPoolNsolns();
			if (poolSizeAfter > poolSizeBefore)
			{
				processInfo->logger.message(2) << "    Solution pool populated from: " << poolSizeBefore << " to "
						<< poolSizeAfter << CoinMessageEol;
			}

		}
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when populating solution pool:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}

	try
	{
		//IloCplex::Callback mycallback = cplexInstance.use(SolutionFilterCallback(cplexEnv, cplexVars, processInfo));
		//std::cout << "Sol pool size after:" << cplexInstance.getSolnPoolNsolns() << std::endl;

		int numVar = processInfo->originalProblem->getNumberOfVariables();

		if (getDiscreteVariableStatus())
		{
			//allSolutions.push_back(getVariableSolution()); // Already included in solution

			auto numSols = cplexInstance.getSolnPoolNsolns();

			for (int i = 0; i < numSols; i++)
			{
				std::vector<double> solution(numVar);

				IloNumArray tmpSols(cplexEnv);

				cplexInstance.getValues(cplexVars, tmpSols, i);

				//std::cout << "Obj value for " << i << ": " << cplexInstance.getObjValue(i) << std::endl;

				for (int i = 0; i < numVar; i++)
				{
					solution.at(i) = tmpSols[i];
				}

				allSolutions.push_back(solution);
			}
		}
		else
		{
			allSolutions.push_back(getVariableSolution());
		}
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when reading solutions:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}

	return allSolutions;
}

void MILPSolverCplex::setTimeLimit(double seconds)
{
	try
	{
		cplexInstance.setParam(IloCplex::TiLim, seconds);
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when setting time limit:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}
}

void MILPSolverCplex::setCutOff(double cutOff)
{
	try
	{
		processInfo->logger.message(3) << "Setting cutoff value to " << cutOff << CoinMessageEol;
		if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			cplexInstance.setParam(IloCplex::CutUp, cutOff);
		}
		else
		{
			cplexInstance.setParam(IloCplex::CutLo, cutOff);
		}
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when setting cut off value:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}
}

void MILPSolverCplex::addMIPStart(std::vector<double> point)
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
		processInfo->logger.message(2) << "Error when adding MIP starting point:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;
	}

	processInfo->logger.message(3) << "Added MIP start" << CoinMessageEol;

}

void MILPSolverCplex::writeProblemToFile(std::string filename)
{
	try
	{
		cplexInstance.exportModel(filename.c_str());
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << "Error when saving model to file:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}
}

bool MILPSolverCplex::getDiscreteVariableStatus()
{
	return (MILPSolverBase::getDiscreteVariableStatus());
}

void MILPSolverCplex::changeConstraintToLazy(std::vector<int> constrIdxs)
{
	try
	{
//std::cout << "Starting conversion to lazy "<< std::endl;
//std::cout << "Number start " << cplexConstrs.getSize()<< std::endl;

//UtilityFunctions::displayVector(constrIdxs);
		for (int i = 0; i < constrIdxs.size(); i++)
		{
			//std::cout << "Converted to lazy: " << constrIdxs[i] << std::endl;
			IloRange tmpRange = cplexConstrs[constrIdxs[i]];

			try
			{
				cplexModel.remove(tmpRange);
				//cplexInstance.extract(cplexModel);
				//cplexInstance.addLazyConstraint(tmpRange);
				cplexLazyConstrs.add(tmpRange);

				std::cout << "Changed constraint " << constrIdxs[i] << " to lazy" << std::endl;
				//cplexModel.remove(tmpRange);
			}
			catch (IloException &e)
			{
				processInfo->logger.message(1) << e.getMessage() << CoinMessageEol;
			}
		}

//std::cout << "End  conversion to lazy "<< std::endl;
//std::cout << "Number stop " << cplexConstrs.getSize()<< std::endl;
	}
	catch (IloException &e)
	{
		processInfo->logger.message(0) << e.getMessage() << CoinMessageEol;

	}
}
