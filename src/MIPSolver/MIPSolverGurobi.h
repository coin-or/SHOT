/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "MIPSolverBase.h"

#include "gurobi_c++.h"

namespace SHOT
{
class MIPSolverGurobi : public IMIPSolver, public MIPSolverBase
{
  public:
    MIPSolverGurobi();
    MIPSolverGurobi(EnvironmentPtr envPtr);
    virtual ~MIPSolverGurobi();

    virtual bool initializeProblem();

    virtual void checkParameters();

    //virtual bool createLinearProblem(OptProblem *origProblem);
    //virtual bool createLinearProblem(ProblemPtr sourceProblem){};

    virtual bool addVariable(std::string name, E_VariableType type, double lowerBound, double upperBound);

    virtual bool initializeObjective();
    virtual bool addLinearTermToObjective(double coefficient, int variableIndex);
    virtual bool addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex);
    virtual bool finalizeObjective(bool isMinimize, double constant = 0.0);

    virtual bool initializeConstraint();
    virtual bool addLinearTermToConstraint(double coefficient, int variableIndex);
    virtual bool addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex);
    virtual bool finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant = 0.0);

    virtual bool finalizeProblem();

    virtual void initializeSolverSettings();
    virtual void writeProblemToFile(std::string filename);
    virtual void writePresolvedToFile(std::string filename);

    virtual int addLinearConstraint(const std::vector<PairIndexValue> &elements, double constant)
    {
        return (addLinearConstraint(elements, constant, false));
    }
    virtual int addLinearConstraint(const std::vector<PairIndexValue> &elements, double constant, bool isGreaterThan);

    virtual void createHyperplane(Hyperplane hyperplane)
    {
        MIPSolverBase::createHyperplane(hyperplane);
    }

    virtual void createIntegerCut(VectorInteger binaryIndexes)
    {
        MIPSolverBase::createIntegerCut(binaryIndexes);
    }

    virtual void createInteriorHyperplane(Hyperplane hyperplane)
    {
        MIPSolverBase::createInteriorHyperplane(hyperplane);
    }

    virtual boost::optional<std::pair<std::vector<PairIndexValue>, double>> createHyperplaneTerms(
        Hyperplane hyperplane)
    {
        return (MIPSolverBase::createHyperplaneTerms(hyperplane));
    }

    virtual void fixVariable(int varIndex, double value);

    virtual void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
    {
        MIPSolverBase::fixVariables(variableIndexes, variableValues);
    }

    virtual void unfixVariables()
    {
        MIPSolverBase::unfixVariables();
    }

    virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound);
    virtual PairDouble getCurrentVariableBounds(int varIndex);

    virtual void presolveAndUpdateBounds()
    {
        return (MIPSolverBase::presolveAndUpdateBounds());
    }

    virtual std::pair<VectorDouble, VectorDouble> presolveAndGetNewBounds();

    virtual void activateDiscreteVariables(bool activate);
    virtual bool getDiscreteVariableStatus()
    {
        return (MIPSolverBase::getDiscreteVariableStatus());
    }

    virtual E_ProblemSolutionStatus solveProblem();
    virtual E_ProblemSolutionStatus getSolutionStatus();
    virtual int getNumberOfSolutions();
    virtual VectorDouble getVariableSolution(int solIdx);
    virtual std::vector<SolutionPoint> getAllVariableSolutions()
    {
        return (MIPSolverBase::getAllVariableSolutions());
    }
    virtual double getDualObjectiveValue();
    virtual double getObjectiveValue(int solIdx);
    virtual double getObjectiveValue()
    {
        return (MIPSolverBase::getObjectiveValue());
    }

    virtual int increaseSolutionLimit(int increment);
    virtual void setSolutionLimit(long limit);
    virtual int getSolutionLimit();

    virtual void setTimeLimit(double seconds);

    virtual void setCutOff(double cutOff);

    virtual void addMIPStart(VectorDouble point);
    virtual void deleteMIPStarts();

    virtual bool supportsQuadraticObjective();
    virtual bool supportsQuadraticConstraints();

    virtual std::vector<GeneratedHyperplane> *getGeneratedHyperplanes()
    {
        return (MIPSolverBase::getGeneratedHyperplanes());
    }

    virtual int getNumberOfExploredNodes();

    virtual int getNumberOfOpenNodes()
    {
        return (MIPSolverBase::getNumberOfOpenNodes());
    }

    virtual bool hasAuxilliaryObjectiveVariable()
    {
        return (MIPSolverBase::hasAuxilliaryObjectiveVariable());
    }

    virtual int getAuxilliaryObjectiveVariableIndex()
    {
        return (MIPSolverBase::getAuxilliaryObjectiveVariableIndex());
    }

    virtual void setAuxilliaryObjectiveVariableIndex(int index)
    {
        return (MIPSolverBase::setAuxilliaryObjectiveVariableIndex(index));
    }

    GRBEnv *gurobiEnv;
    GRBModel *gurobiModel;
    GRBLinExpr objectiveLinearExpression;
    GRBQuadExpr objectiveQuadraticExpression;
    GRBLinExpr constraintLinearExpression;
    GRBQuadExpr constraintQuadraticExpression;

  private:
};

class GurobiInfoCallback : public GRBCallback
{
  public:
    GurobiInfoCallback(EnvironmentPtr envPtr);

  protected:
    void callback();

  private:
    int numVar = 0;
    int lastExploredNodes = 0;
    int lastOpenNodes = 0;
    EnvironmentPtr env;
};
} // namespace SHOT