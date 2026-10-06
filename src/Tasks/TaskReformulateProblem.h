/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include <functional>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>

#include "../Model/AuxiliaryVariables.h"
#include "../Model/Constraints.h"
#include "../Model/NonlinearExpressions.h"
#include "../Model/Problem.h"
#include "../Model/Terms.h"
#include "../Model/Variables.h"

namespace SHOT
{
struct Reformulation
{
    LinearConstraints linearConstraints;
    QuadraticConstraints quadraticConstraints;
    NonlinearConstraint nonlinearConstraint;

    AuxiliaryVariables reformulationVariables;
};

class TaskReformulateProblem : public TaskBase
{
public:
    TaskReformulateProblem(EnvironmentPtr envPtr);
    ~TaskReformulateProblem() override;

    void run() override;
    std::string getType() override;

private:
    bool useConvexQuadraticConstraints = false;
    bool useNonconvexQuadraticConstraints = false;

    // If the constraint is convex in SHOT with respect to the
    // Model.Convexity.Quadratics.EigenValueTolerance, but the MIP solver does not support such a tolerance
    // (Cplex)
    bool useConvexQuadraticConstraintsWithinTolerance = false;

    bool useConvexQuadraticObjective = false;
    bool useNonconvexQuadraticObjective = false;
    bool quadraticObjectiveRegardedAsNonlinear = false;
    bool partitionQuadraticTermsInObjective = false;
    bool partitionQuadraticTermsInConstraint = false;

    bool extractQuadraticTermsFromNonconvexExpressions = false;
    bool extractQuadraticTermsFromConvexExpressions = false;

    int maxBilinearIntegerReformulationDomain = 2;

    bool useIntegerBilinearTermReformulation = false; // integer term i1*i2 or i1*x2

    void reformulateObjectiveFunction();
    void createEpigraphConstraint();

    NumericConstraints reformulateConstraint(NumericConstraintPtr constraint);

    // Constraints that are added to the reformulated problem after all the other ones
    NonlinearConstraints deferredConstraints;

    // Whether the constraint is L <= f(x) <= U with both bounds, where f is not linear
    bool isNonlinearWithBothBounds(const NumericConstraintPtr& constraint);

    // Reformulates all constraints of the original problem
    void reformulateConstraints();

    // Reformulates the lower side -f(x) <= -L of a constraint of the original problem, and adds it to the problem
    void reformulateLowerSide(const NumericConstraintPtr& constraint, std::set<std::string>& usedNames);

    template <class T> void copyLinearTermsToConstraint(const LinearTerms& terms, T destination, bool reversedSigns = false);

    template <class T>
    void copyQuadraticTermsToConstraint(const QuadraticTerms& terms, T destination, bool reversedSigns = false);

    template <class T>
    void copyMonomialTermsToConstraint(const MonomialTerms& terms, T destination, bool reversedSigns = false);

    template <class T>
    void copySignomialTermsToConstraint(const SignomialTerms& terms, T destination, bool reversedSigns = false);

    template <class T>
    void copyLinearTermsToObjectiveFunction(const LinearTerms& terms, T destination, bool reversedSigns = false);

    template <class T>
    void copyQuadraticTermsToObjectiveFunction(const QuadraticTerms& terms, T destination, bool reversedSigns = false);

    template <class T>
    void copyMonomialTermsToObjectiveFunction(const MonomialTerms& terms, T destination, bool reversedSigns = false);

    template <class T>
    void copySignomialTermsToObjectiveFunction(const SignomialTerms& terms, T destination, bool reversedSigns = false);

    LinearTerms partitionNonlinearSum(const std::shared_ptr<ExpressionSum>& source, bool reversedSigns);
    LinearTerms partitionMonomialTerms(const MonomialTerms& sourceTerms, bool reversedSigns);
    LinearTerms partitionSignomialTerms(const SignomialTerms& sourceTerms, bool reversedSigns);

    LinearTerms partitionNonlinearBinaryProduct(const std::shared_ptr<ExpressionSum> source, bool reversedSigns);

    // Returns the linear terms and the quadratic terms replacing the terms, and a constant from fixed variables
    std::tuple<LinearTerms, QuadraticTerms, double> reformulateAndPartitionQuadraticSum(
        QuadraticTerms& quadraticTerms, bool reversedSigns, ES_PartitionNonlinearSums partitionStrategy);
    std::tuple<LinearTerms, MonomialTerms> reformulateMonomialSum(
        const MonomialTerms& monomialTerms, bool reversedSigns);

    LinearTerms doEigenvalueDecomposition(QuadraticTerms& quadraticTerms);
    LinearTerms doLDLDecomposition(QuadraticTerms& quadraticTerms);

    // Adds the term value * y^2 of a decomposition with y given by the linear terms
    void addDecompositionComponent(const LinearTerms& componentTerms, double value,
        E_AuxiliaryVariableType auxVariableType, const std::string& name, std::vector<LinearTermPtr>& resultTerms);

    NonlinearExpressionPtr reformulateNonlinearExpression(NonlinearExpressionPtr source);
    NonlinearExpressionPtr reformulateNonlinearExpression(std::shared_ptr<ExpressionAbs> source);
    NonlinearExpressionPtr reformulateNonlinearExpression(std::shared_ptr<ExpressionSquare> source);
    NonlinearExpressionPtr reformulateNonlinearExpression(std::shared_ptr<ExpressionProduct> source);

    std::pair<AuxiliaryVariablePtr, bool> getSquareAuxiliaryVariable(
        VariablePtr firstVariable, double coefficient, E_AuxiliaryVariableType auxVariableType);

    std::pair<AuxiliaryVariablePtr, bool> getBilinearAuxiliaryVariable(
        VariablePtr firstVariable, VariablePtr secondVariable);

    std::pair<AuxiliaryVariablePtr, bool> getAbsoluteValueAuxiliaryVariable(std::shared_ptr<ExpressionAbs> source);

    void createSquareReformulations();
    void createBilinearReformulations();

    void reformulateSquareTerm(VariablePtr variable, AuxiliaryVariablePtr auxVariable, double coefficient = 1.0);

    void reformulateBinaryBilinearTerm(
        VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable);
    void reformulateBinaryContinuousBilinearTerm(
        VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable);
    void reformulateIntegerBilinearTerm(
        VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable);
    void reformulateRealBilinearTerm(
        VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable);

    void addBilinearMcCormickEnvelope(VariablePtr auxVariable, VariablePtr firstVariable, VariablePtr secondVariable);

    std::optional<QuadraticTermPtr> reformulateProductToQuadraticTerm(std::shared_ptr<ExpressionProduct> product);
    std::optional<MonomialTermPtr> reformulateProductToMonomialTerm(std::shared_ptr<ExpressionProduct> product);

    int auxVariableCounter = 0;
    int auxConstraintCounter = 0;

    std::map<VariablePtr, Variables, VariableIndexComparator> integerAuxiliaryBinaryVariables;

    std::map<std::pair<VariablePtr, double>, AuxiliaryVariablePtr, VariableIndexComparator> squareAuxVariables;
    std::map<int, int> squareAuxVariableCounts; // The number of square auxiliary variables of each variable

    std::map<std::tuple<VariablePtr, VariablePtr>, AuxiliaryVariablePtr, VariableIndexComparator>
        bilinearAuxVariables;

    std::map<std::string, AuxiliaryVariablePtr> absoluteExpressionsAuxVariables;

    // The auxiliary variables w >= f(x) or w >= -f(x) of the partitioned terms, found by whether the sign is positive
    // and a key of the term built from the indexes of its variables, or by the coefficient of a monomial
    std::map<std::pair<bool, std::string>, AuxiliaryVariablePtr> nonlinearExpressionAuxVariables;
    std::map<std::pair<double, std::vector<int>>, AuxiliaryVariablePtr> monomialAuxVariables;
    std::map<std::pair<bool, std::vector<std::pair<int, double>>>, AuxiliaryVariablePtr> signomialAuxVariables;

    struct BinaryMonomialKeyHash
    {
        size_t operator()(const std::vector<int>& indexes) const noexcept
        {
            size_t hash = indexes.size();
            for(int index : indexes)
                hash = hash * 1315423911U + std::hash<int> {}(index);
            return hash;
        }
    };

    // The auxiliary variables w = b1 * ... * bn of the products of binary variables, found by the variable indexes
    std::unordered_map<std::vector<int>, AuxiliaryVariablePtr, BinaryMonomialKeyHash> binaryMonomialAuxVariables;

    ProblemPtr reformulatedProblem;
};
} // namespace SHOT