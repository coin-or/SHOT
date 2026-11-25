/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "MIPSolverBase.h"
#include "MIPSolverCapabilities.h"
#include "MIPSolverCallbackBase.h"

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "scip/type_scip.h"
#include "scip/type_var.h"

namespace SHOT
{

// Makes the termination check of the callbacks available to SCIP's event handler
class SCIPTerminationHelper : public MIPSolverCallbackBase
{
public:
    SCIPTerminationHelper(EnvironmentPtr envPtr) { env = envPtr; }
    bool checkTermination() { return checkUserTermination(); }
};

// SCIP keeps the solutions it has found when the problem is changed after a solve, and they count towards the solution
// limit of the next solve, which then stops at once. The model is therefore kept here, and a new SCIP instance is
// created from it for every solve. The instance is kept until the next solve, so that the status, the dual bound and
// the number of nodes can be obtained.
class MIPSolverSCIP : public IMIPSolver, public MIPSolverBase
{
public:
    inline static constexpr MIPSolverCapabilities capabilities = []
    {
        MIPSolverCapabilities result;
        // SCIP solves problems with nonconvex quadratic terms to global optimality
        result.convexQuadraticObjective = true;
        result.nonconvexQuadraticObjective = true;
        result.convexQuadraticConstraints = true;
        result.nonconvexQuadraticConstraints = true;
        result.nonconvexObjectiveExcludesQuadraticConstraints = false;
        result.SOS1 = true;
        result.SOS2 = true;
        result.singleTree = false;
        result.semicontinuous = E_SemiVariableSupport::NonnegativeInterval;
        result.semiinteger = E_SemiVariableSupport::NonnegativeInterval;
        result.unboundedVariableBound = 1e20;

        return result;
    }();

    MIPSolverSCIP(EnvironmentPtr envPtr);
    ~MIPSolverSCIP() override;

    bool initializeProblem() override;

    void checkParameters() override;

    bool addVariable(
        std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound) override;

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

    int addLinearConstraint(std::map<int, double>& elements, double constant, std::string name) override
    {
        return (addLinearConstraint(elements, constant, name, false, true));
    }

    int addLinearConstraint(
        const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan) override
    {
        return (addLinearConstraint(elements, constant, name, isGreaterThan, true));
    }

    int addLinearConstraint(const std::map<int, double>& elements, double constant, std::string name,
        bool isGreaterThan, bool allowRepair) override;

    VectorInteger addLinearConstraints(const std::vector<std::map<int, double>>& elements,
        const VectorDouble& constants, const VectorString& names, bool isGreaterThan, bool allowRepair) override
    {
        return (MIPSolverBase::addLinearConstraints(elements, constants, names, isGreaterThan, allowRepair));
    }

    bool addSpecialOrderedSet(
        E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights = { }) override;

    bool createHyperplane(HyperplanePtr hyperplane) override { return (MIPSolverBase::createHyperplane(hyperplane)); }

    bool createIntegerCut(IntegerCut& integerCut) override;

    int relaxRepairableCutsViolatedByPoint(const VectorDouble& point, size_t firstCut) override
    {
        return (MIPSolverBase::relaxRepairableCutsViolatedByPoint(point, firstCut));
    }

    VectorInteger removeRepairableCuts() override { return (MIPSolverBase::removeRepairableCuts()); }
    size_t getNumberOfRepairableCuts() override { return (MIPSolverBase::getNumberOfRepairableCuts()); }

    double getConstraintUpperBound(int constraintIndex) override;
    bool setConstraintUpperBound(int constraintIndex, double upperBound) override;

    std::optional<std::pair<std::map<int, double>, double>> createHyperplaneTerms(HyperplanePtr hyperplane) override
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

    void activateDiscreteVariables(bool activate) override;
    bool getDiscreteVariableStatus() override { return (MIPSolverBase::getDiscreteVariableStatus()); }

    E_DualProblemClass getProblemClass() override { return (MIPSolverBase::getProblemClass()); }

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
    void addPartialMIPStart(VectorInteger variableIndexes, VectorDouble variableValues) override;
    void deleteMIPStarts() override;

    double getUnboundedVariableBoundValue() override;

    int getNumberOfExploredNodes() override;

    int getNumberOfOpenNodes() override { return (MIPSolverBase::getNumberOfOpenNodes()); }

    int getNumberOfVariables() override { return (MIPSolverBase::getNumberOfVariables()); }

    bool hasDualAuxiliaryObjectiveVariable() override { return (MIPSolverBase::hasDualAuxiliaryObjectiveVariable()); }

    int getDualAuxiliaryObjectiveVariableIndex() override
    {
        return (MIPSolverBase::getDualAuxiliaryObjectiveVariableIndex());
    }

    void setDualAuxiliaryObjectiveVariableIndex(int index) override
    {
        MIPSolverBase::setDualAuxiliaryObjectiveVariableIndex(index);
    }

    std::string getConstraintIdentifier(E_HyperplaneSource source) override
    {
        return (MIPSolverBase::getConstraintIdentifier(source));
    };

    std::string getSolverVersion() override;

    int getNumberOfThreads() override;

    // Used by SCIP's event handler, which interrupts the solve when SHOT terminates
    std::unique_ptr<SCIPTerminationHelper> terminationHelper;

private:
    struct QuadraticTerm
    {
        int firstVariableIndex;
        int secondVariableIndex;
        double coefficient;
    };

    // A constraint lhs <= sum(coefficients * variables) + quadratic terms <= rhs, where infinite sides are SCIP's
    // infinity
    struct Row
    {
        std::string name;
        VectorInteger variableIndexes;
        VectorDouble coefficients;
        double lhs;
        double rhs;
        std::vector<QuadraticTerm> quadraticTerms;
    };

    struct SpecialOrderedSet
    {
        E_SOSType type;
        VectorInteger variableIndexes;
        VectorDouble weights;
    };

    // The SCIP instance created for a solve, and its variables in the order of the variables in SHOT
    struct SCIPInstance
    {
        SCIP* scip = nullptr;
        std::vector<SCIP_VAR*> variables;

        ~SCIPInstance();
    };

    std::unique_ptr<SCIPInstance> instance;

    VectorDouble currentLowerBounds;
    VectorDouble currentUpperBounds;

    VectorDouble objectiveCoefficients;
    double objectiveConstant = 0.0;

    std::vector<QuadraticTerm> objectiveQuadraticTerms;

    // The linear constraints, whose indexes are the constraint indexes in SHOT, e.g., of cuts, and the quadratic
    // constraints, which are kept separately so that they do not change these indexes
    std::vector<Row> rows;
    std::vector<Row> quadraticRows;
    std::vector<SpecialOrderedSet> specialOrderedSets;

    // The terms of the constraint being created by addLinearTermToConstraint() and addQuadraticTermToConstraint()
    std::map<int, double> currentConstraintTerms;
    std::vector<QuadraticTerm> currentConstraintQuadraticTerms;

    // No limit until a solution limit strategy sets one, since, e.g., the direct MIQCQP strategy does not use any
    long solutionLimit = std::numeric_limits<int>::max();

    // Whether SCIP's numerics emphasis is used, when a problem is solved again after numerical trouble
    bool useNumericsEmphasis = false;
    double timeLimit = 1e20;
    double cutOff = 1e20;
    bool cutOffDefined = false;

    // The start is kept until it is replaced or deleteMIPStarts() is called, and a partial start only for the next
    // solve
    VectorDouble MIPStart;
    VectorInteger partialMIPStartIndexes;
    VectorDouble partialMIPStartValues;

    // The solutions of the latest solve, the best first
    std::vector<SolutionPoint> currentSolutions;

    // The objective coefficients temporarily removed to find a point of an unbounded dual problem
    std::vector<PairIndexValue> objectiveCoefficientsToRestore;

    // Creates a SCIP instance of the model. With repair slacks, the constraints that may be repaired get a nonnegative
    // slack variable each, whose sum, weighted by the given penalties, is minimized instead of the objective function.
    std::unique_ptr<SCIPInstance> createInstance(bool allowDualReductions = true,
        const std::map<int, double>* repairPenalties = nullptr, std::map<int, SCIP_VAR*>* repairSlacks = nullptr);

    // Solves the current model with a new instance and stores its solutions
    E_ProblemSolutionStatus solveInstance(bool allowDualReductions = true);

    E_ProblemSolutionStatus getSolutionStatus(SCIP* scip);

    void restoreObjectiveCoefficients();

    // The value of the objective function, with the removed objective coefficients, in a point
    double calculateObjectiveValue(const VectorDouble& point);

    // Adds a quadratic constraint lhs <= linear terms + quadratic terms <= rhs to a SCIP instance
    void addQuadraticConstraint(SCIP* scip, const std::vector<SCIP_VAR*>& variables, const std::string& name,
        const VectorInteger& variableIndexes, const VectorDouble& coefficients,
        const std::vector<QuadraticTerm>& quadraticTerms, double lhs, double rhs,
        SCIP_VAR* objectiveVariable = nullptr);

    int addRow(const std::string& name, VectorInteger variableIndexes, VectorDouble coefficients, double lhs,
        double rhs, bool allowRepair);
};

} // namespace SHOT
