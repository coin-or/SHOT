/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

namespace SHOT
{

// Forward declarations

class Problem;
typedef std::shared_ptr<Problem> ProblemPtr;

class Variable;
typedef std::shared_ptr<Variable> VariablePtr;
typedef std::map<VariablePtr, double> SparseVariableVector;
typedef std::map<std::pair<VariablePtr, VariablePtr>, double> SparseVariableMatrix;
typedef std::vector<VariablePtr> Variables;

class AuxiliaryVariable;
typedef std::shared_ptr<AuxiliaryVariable> AuxiliaryVariablePtr;
typedef std::vector<AuxiliaryVariablePtr> AuxiliaryVariables;

class ObjectiveFunction;
typedef std::shared_ptr<ObjectiveFunction> ObjectiveFunctionPtr;

class LinearObjectiveFunction;
typedef std::shared_ptr<LinearObjectiveFunction> LinearObjectiveFunctionPtr;

class QuadraticObjectiveFunction;
typedef std::shared_ptr<QuadraticObjectiveFunction> QuadraticObjectiveFunctionPtr;

class NonlinearObjectiveFunction;
typedef std::shared_ptr<NonlinearObjectiveFunction> NonlinearObjectiveFunctionPtr;

class Term;

class LinearTerm;
typedef std::shared_ptr<LinearTerm> LinearTermPtr;

class QuadraticTerm;
typedef std::shared_ptr<QuadraticTerm> QuadraticTermPtr;

class MonomialTerm;
typedef std::shared_ptr<MonomialTerm> MonomialTermPtr;

template <class T> class Terms;

typedef Terms<LinearTermPtr> LinearTerms;
typedef Terms<QuadraticTermPtr> QuadraticTerms;
typedef Terms<MonomialTermPtr> MonomialTerms;

class NonlinearExpression;
typedef std::shared_ptr<NonlinearExpression> NonlinearExpressionPtr;

class Constraint;
typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::vector<ConstraintPtr> Constraints;

class NumericConstraint;
typedef std::shared_ptr<NumericConstraint> NumericConstraintPtr;
typedef std::vector<NumericConstraintPtr> NumericConstraints;

class LinearConstraint;
typedef std::shared_ptr<LinearConstraint> LinearConstraintPtr;
typedef std::vector<LinearConstraintPtr> LinearConstraints;

class QuadraticConstraint;
typedef std::shared_ptr<QuadraticConstraint> QuadraticConstraintPtr;
typedef std::vector<QuadraticConstraintPtr> QuadraticConstraints;

class NonlinearConstraint;
typedef std::shared_ptr<NonlinearConstraint> NonlinearConstraintPtr;
typedef std::vector<NonlinearConstraintPtr> NonlinearConstraints;

class ExpressionVariable;
typedef std::shared_ptr<ExpressionVariable> ExpressionVariablePtr;

// typedef std::vector<PairCoordinateValue> SparseMatrix;

typedef mc::FFGraph FactorableFunctionGraph;
typedef std::shared_ptr<FactorableFunctionGraph> FactorableFunctionGraphPtr;
typedef mc::FFVar FactorableFunction;
typedef std::shared_ptr<FactorableFunction> FactorableFunctionPtr;
typedef std::vector<FactorableFunctionPtr> FactorableFunctions;

typedef mc::Interval Interval;
typedef std::vector<Interval> IntervalVector;

struct NumericConstraintValue
{
    NumericConstraintPtr constraint;

    // Considering a constraint L <= f(x) <= U:
    double functionValue; // This is the function value of f(x)
    bool isFulfilledLHS; // Is L <= f(x)?
    double normalizedLHSValue; // This is the value of L - f(x)
    bool isFulfilledRHS; // Is f(x) <= U
    double normalizedRHSValue; // This is the value of f(x) - U
    bool isFulfilled; // Is L <= f(x) <= U?
    double error; // max(0, max(L - f(x), f(x) - U)
    double normalizedValue; // max(L - f(x), f(x)-U)

    // Sorts in reverse order, i.e. so that larger errors are before smaller ones
    bool operator>(const NumericConstraintValue& otherValue) const
    {
        return normalizedValue > otherValue.normalizedValue;
    }
};

typedef std::vector<NumericConstraintValue> NumericConstraintValues;

// Begin exception definitions

class VariableNotFoundException : public std::exception
{
private:
    std::string errorMessage;

public:
    VariableNotFoundException(std::string message) : errorMessage(message) {}

    inline const char* what() const throw()
    {
        std::stringstream message;
        message << "Could not find variable ";
        message << errorMessage;

        return (message.str().c_str());
    }
};

class ConstraintNotFoundException : public std::exception
{
private:
    std::string errorMessage;

public:
    ConstraintNotFoundException(std::string message) : errorMessage(message) {}

    inline const char* what() const throw()
    {
        std::stringstream message;
        message << "Could not find constraint ";
        message << errorMessage;

        return (message.str().c_str());
    }
};

class OperationNotImplementedException : public std::exception
{
private:
    std::string errorMessage;

public:
    OperationNotImplementedException(std::string message) : errorMessage(message) {}

    inline const char* what() const throw()
    {
        std::stringstream message;
        message << "The following operation is not implemented: ";
        message << errorMessage;

        return (message.str().c_str());
    }
};

// End exception definitions

} // namespace SHOT
