/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "MIPSolverBase.h"

#include "CoinBuild.hpp"
#include "CoinModel.hpp"
#include "CoinPragma.hpp"
#include "CbcModel.hpp"
#include "OsiClpSolverInterface.hpp"

namespace SHOT
{
class MIPSolverOsiCbc : public IMIPSolver, MIPSolverBase
{
public:
    MIPSolverOsiCbc(EnvironmentPtr envPtr);
    virtual ~MIPSolverOsiCbc();

    virtual bool initializeProblem();

    virtual void checkParameters();

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

    virtual int addLinearConstraint(const std::vector<PairIndexValue>& elements, double constant, std::string name)
    {
        return (addLinearConstraint(elements, constant, name, false));
    }
    virtual int addLinearConstraint(
        const std::vector<PairIndexValue>& elements, double constant, std::string name, bool isGreaterThan);

    virtual void createHyperplane(Hyperplane hyperplane) { MIPSolverBase::createHyperplane(hyperplane); }

    virtual void createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes)
    { /*MIPSolverBase::createIntegerCut(binaryIndexes); TODO*/
    }

    virtual void createInteriorHyperplane(Hyperplane hyperplane)
    {
        MIPSolverBase::createInteriorHyperplane(hyperplane);
    }

    virtual std::optional<std::pair<std::vector<PairIndexValue>, double>> createHyperplaneTerms(Hyperplane hyperplane)
    {
        return (MIPSolverBase::createHyperplaneTerms(hyperplane));
    }

    virtual void fixVariable(int varIndex, double value);

    virtual void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
    {
        MIPSolverBase::fixVariables(variableIndexes, variableValues);
    }

    virtual void unfixVariables() { MIPSolverBase::unfixVariables(); }

    virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound);
    virtual void updateVariableLowerBound(int varIndex, double lowerBound);
    virtual void updateVariableUpperBound(int varIndex, double upperBound);

    virtual PairDouble getCurrentVariableBounds(int varIndex);

    virtual void presolveAndUpdateBounds() { return (MIPSolverBase::presolveAndUpdateBounds()); }

    virtual std::pair<VectorDouble, VectorDouble> presolveAndGetNewBounds();

    virtual void activateDiscreteVariables(bool activate);
    virtual bool getDiscreteVariableStatus() { return (MIPSolverBase::getDiscreteVariableStatus()); }

    virtual E_IterationProblemType getCurrentProblemType() { return (MIPSolverBase::getCurrentProblemType()); }

    virtual void executeRelaxationStrategy() { MIPSolverBase::executeRelaxationStrategy(); }

    virtual E_ProblemSolutionStatus solveProblem();
    virtual bool repairInfeasibility();

    virtual E_ProblemSolutionStatus getSolutionStatus();
    virtual int getNumberOfSolutions();
    virtual VectorDouble getVariableSolution(int solIdx);
    virtual std::vector<SolutionPoint> getAllVariableSolutions() { return (MIPSolverBase::getAllVariableSolutions()); }
    virtual double getDualObjectiveValue();
    virtual double getObjectiveValue(int solIdx);
    virtual double getObjectiveValue() { return (MIPSolverBase::getObjectiveValue()); }

    virtual int increaseSolutionLimit(int increment);
    virtual void setSolutionLimit(long limit);
    virtual int getSolutionLimit();

    virtual void setTimeLimit(double seconds);

    virtual void setCutOff(double cutOff);
    virtual void setCutOffAsConstraint(double cutOff);

    virtual void addMIPStart(VectorDouble point);
    virtual void deleteMIPStarts();

    virtual bool supportsQuadraticObjective();
    virtual bool supportsQuadraticConstraints();

    virtual int getNumberOfExploredNodes();

    virtual int getNumberOfOpenNodes() { return (MIPSolverBase::getNumberOfOpenNodes()); }

    virtual bool hasAuxilliaryObjectiveVariable() { return (MIPSolverBase::hasAuxilliaryObjectiveVariable()); }

    virtual int getAuxilliaryObjectiveVariableIndex() { return (MIPSolverBase::getAuxilliaryObjectiveVariableIndex()); }

    virtual void setAuxilliaryObjectiveVariableIndex(int index)
    {
        return (MIPSolverBase::setAuxilliaryObjectiveVariableIndex(index));
    }

    virtual std::string getConstraintIdentifier(E_HyperplaneSource source)
    {
        return (MIPSolverBase::getConstraintIdentifier(source));
    };

private:
    std::unique_ptr<OsiClpSolverInterface> osiInterface;
    std::unique_ptr<CbcModel> cbcModel;
    std::unique_ptr<CoinModel> coinModel;

    long int solLimit;
    double cutOff;

    std::vector<std::vector<std::pair<std::string, double>>> MIPStarts;

    std::vector<E_VariableType> variableTypes;
};
} // namespace SHOT