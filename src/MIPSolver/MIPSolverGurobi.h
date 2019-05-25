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
    ~MIPSolverGurobi() override;

    bool initializeProblem() override;

    void checkParameters() override;

    bool addVariable(std::string name, E_VariableType type, double lowerBound, double upperBound) override;

    bool initializeObjective() override;
    bool addLinearTermToObjective(double coefficient, int variableIndex) override;
    bool addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex) override;
    bool finalizeObjective(bool isMinimize, double constant = 0.0) override;

    bool initializeConstraint() override;
    bool addLinearTermToConstraint(double coefficient, int variableIndex) override;
    bool addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex) override;
    bool finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant = 0.0) override;

    bool finalizeProblem() override;

    void initializeSolverSettings() override;

    void writeProblemToFile(std::string filename) override;
    void writePresolvedToFile(std::string filename) override;

    int addLinearConstraint(const std::vector<PairIndexValue>& elements, double constant, std::string name) override
    {
        return (addLinearConstraint(elements, constant, name, false));
    }

    int addLinearConstraint(
        const std::vector<PairIndexValue>& elements, double constant, std::string name, bool isGreaterThan) override;

    void createHyperplane(Hyperplane hyperplane) override { MIPSolverBase::createHyperplane(hyperplane); }

    void createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes) override;

    void createInteriorHyperplane(Hyperplane hyperplane) override
    {
        MIPSolverBase::createInteriorHyperplane(hyperplane);
    }

    std::optional<std::pair<std::vector<PairIndexValue>, double>> createHyperplaneTerms(Hyperplane hyperplane) override
    {
        return (MIPSolverBase::createHyperplaneTerms(hyperplane));
    }

    void fixVariable(int varIndex, double value) override;

    void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues) override
    {
        MIPSolverBase::fixVariables(variableIndexes, variableValues);
    }

    void unfixVariables() override { MIPSolverBase::unfixVariables(); }

    void updateVariableBound(int varIndex, double lowerBound, double upperBound) override;
    void updateVariableLowerBound(int varIndex, double lowerBound) override;
    void updateVariableUpperBound(int varIndex, double upperBound) override;

    PairDouble getCurrentVariableBounds(int varIndex) override;

    void presolveAndUpdateBounds() override { return (MIPSolverBase::presolveAndUpdateBounds()); }

    std::pair<VectorDouble, VectorDouble> presolveAndGetNewBounds() override;

    void activateDiscreteVariables(bool activate) override;
    bool getDiscreteVariableStatus() override { return (MIPSolverBase::getDiscreteVariableStatus()); }

    void executeRelaxationStrategy() override { MIPSolverBase::executeRelaxationStrategy(); }

    E_ProblemSolutionStatus solveProblem() override;
    bool repairInfeasibility() override;

    E_ProblemSolutionStatus getSolutionStatus() override;
    int getNumberOfSolutions() override;
    VectorDouble getVariableSolution(int solIdx) override;
    std::vector<SolutionPoint> getAllVariableSolutions() override { return (MIPSolverBase::getAllVariableSolutions()); }
    double getDualObjectiveValue() override;
    double getObjectiveValue(int solIdx) override;
    double getObjectiveValue() override { return (MIPSolverBase::getObjectiveValue()); }

    int increaseSolutionLimit(int increment) override;
    void setSolutionLimit(long limit) override;
    int getSolutionLimit() override;

    void setTimeLimit(double seconds) override;

    void setCutOff(double cutOff) override;
    void setCutOffAsConstraint(double cutOff) override;

    void addMIPStart(VectorDouble point) override;
    void deleteMIPStarts() override;

    bool supportsQuadraticObjective() override;
    bool supportsQuadraticConstraints() override;

    int getNumberOfExploredNodes() override;

    int getNumberOfOpenNodes() override { return (MIPSolverBase::getNumberOfOpenNodes()); }

    bool hasAuxiliaryObjectiveVariable() override { return (MIPSolverBase::hasAuxiliaryObjectiveVariable()); }

    int getAuxiliaryObjectiveVariableIndex() override { return (MIPSolverBase::getAuxiliaryObjectiveVariableIndex()); }

    void setAuxiliaryObjectiveVariableIndex(int index) override
    {
        return (MIPSolverBase::setAuxiliaryObjectiveVariableIndex(index));
    }

    std::string getConstraintIdentifier(E_HyperplaneSource source) override
    {
        return (MIPSolverBase::getConstraintIdentifier(source));
    };

    std::shared_ptr<GRBEnv> gurobiEnv;
    std::shared_ptr<GRBModel> gurobiModel;
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
    void callback() override;

private:
    int lastExploredNodes = 0;
    int lastOpenNodes = 0;
    EnvironmentPtr env;
};
} // namespace SHOT