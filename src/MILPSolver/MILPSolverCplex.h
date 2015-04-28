#pragma once
#include "IMILPSolver.h"
#include "MILPSolverBase.h"

#include <ilcplex/ilocplex.h>

class MILPSolverCplex: public IMILPSolver, MILPSolverBase
{
	public:
		MILPSolverCplex();
		virtual ~MILPSolverCplex();

		//virtual bool createLinearProblem();
		virtual bool createLinearProblem(OptProblem *origProblem);
		virtual bool addLinearConstraint(std::vector<IndexValuePair> elements, int numNonZero, double constant);
		virtual std::vector<double> getVariableSolution();
		virtual void activateDiscreteVariables(bool activate);
		virtual bool getDiscreteVariableStatus();
		virtual E_ProblemSolutionStatus solveProblem();
		virtual E_ProblemSolutionStatus getSolutionStatus();
		virtual double getLastObjectiveValue();
		virtual double getBestObjectiveValue();

		virtual int increaseSolutionLimit(int increment);
		virtual void setSolutionLimit(int limit);
		virtual int getSolutionLimit();

		virtual void writeProblemToFile(std::string filename);

		virtual std::vector<std::vector<double>> getAllVariableSolutions();

		virtual void setTimeLimit(double seconds);

		virtual void setCutOff(double cutOff);

		virtual void addMIPStart(std::vector<double> point);

		virtual void changeConstraintToLazy(std::vector<int> constrIdxs);

	private:

		int firstNonLazyHyperplane;
		int iterLastLazyConvert;

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

