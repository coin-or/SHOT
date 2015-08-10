#pragma once
#include "IMILPSolver.h"
#include "MILPSolverBase.h"

#include <ilcplex/ilocplex.h>

class MILPSolverCplex: public IMILPSolver, MILPSolverBase
{
	public:
		MILPSolverCplex();
		virtual ~MILPSolverCplex();

		virtual bool createLinearProblem(OptProblem *origProblem);
		virtual void initializeSolverSettings();
		virtual void writeProblemToFile(std::string filename);

		virtual bool addLinearConstraint(std::vector<IndexValuePair> elements, double constant);
		virtual void changeConstraintToLazy(std::vector<int> constrIdxs);
		virtual void createHyperplane(int constrIdx, std::vector<double> point)
		{
			MILPSolverBase::createHyperplane(constrIdx, point);
		}

		virtual void fixVariable(int varIndex, double value);
		virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound);
		virtual pair<double, double> getCurrentVariableBounds(int varIndex);

		virtual void activateDiscreteVariables(bool activate);
		virtual bool getDiscreteVariableStatus()
		{
			return (MILPSolverBase::getDiscreteVariableStatus());
		}

		virtual E_ProblemSolutionStatus solveProblem();
		virtual E_ProblemSolutionStatus getSolutionStatus();
		virtual int getNumberOfSolutions();
		virtual std::vector<double> getVariableSolution(int solIdx);
		virtual std::vector<SolutionPoint> getAllVariableSolutions()
		{
			return (MILPSolverBase::getAllVariableSolutions());
		}
		virtual double getObjectiveValue(int solIdx);
		virtual double getObjectiveValue()
		{
			return (MILPSolverBase::getObjectiveValue());
		}

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(int limit);
		virtual int getSolutionLimit();

		virtual void setTimeLimit(double seconds);

		virtual void setCutOff(double cutOff);

		virtual void addMIPStart(std::vector<double> point);
		virtual void deleteMIPStarts();

		virtual void populateSolutionPool();

	private:

		int firstNonLazyHyperplane;
		int iterLastLazyConvert;
		//double timeLastIter;
		std::vector<double> iterDurations;

		//double bestCutoff = DBL_MAX;

	protected:

		IloEnv cplexEnv;
		IloModel cplexModel;
		IloCplex cplexInstance;
		IloNumVarArray cplexVars;
		IloRangeArray cplexConstrs;
		IloRangeArray cplexLazyConstrs;
		vector<IloConversion> cplexVarConvers;

};
/*
 class SolutionFilterCallbackI: public IloCplex::IncumbentCallbackI
 {
 private:
 //build solution index
 IloNumVarArray xVar;
 ProcessInfo *processInfo;

 public:
 SolutionFilterCallbackI(IloEnv env, IloNumVarArray x, ProcessInfo *pInfo) :
 IloCplex::IncumbentCallbackI(env), xVar(x)
 {

 this->processInfo = pInfo;
 }
 ;
 void main();	// the call back function
 //the duplicate function to create new call back object
 IloCplex::CallbackI* duplicateCallback() const
 {
 return (new (getEnv()) SolutionFilterCallbackI(*this));
 }

 };

 IloCplex::Callback SolutionFilterCallback(IloEnv env, IloNumVarArray x, ProcessInfo *pInfo);
 */

