/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/typing.h>

// Prevent CppAD template instantiation in this compilation unit
// to avoid ODR violations with libSHOTSolver.so
// The templates are explicitly instantiated in Problem.cpp
#include "cppad/cppad.hpp"
extern template class CppAD::AD<double>;
extern template class CppAD::ADFun<double>;

// Custom CppAD error handler to avoid abort on cleanup errors
// This is needed because CppAD's thread_alloc has ODR issues with shared libraries
namespace
{
void cppad_python_error_handler(bool known, int line, const char* file, const char* exp, const char* msg)
{
    // Check if this is the known cleanup error in thread_alloc
    std::string fileStr(file ? file : "");
    std::string expStr(exp ? exp : "");
    if(fileStr.find("thread_alloc.hpp") != std::string::npos && expStr.find("count_inuse_") != std::string::npos)
    {
        // Suppress this error - it's a harmless ODR issue during cleanup
        return;
    }

    // For other errors, throw an exception
    std::string error_msg = std::string("CppAD error at ") + file + ":" + std::to_string(line);
    if(msg)
        error_msg += std::string(" - ") + msg;
    throw std::runtime_error(error_msg);
}

// Register the custom error handler at module load time
struct CppADErrorHandlerRegistrar
{
    CppAD::ErrorHandler handler;
    CppADErrorHandlerRegistrar() : handler(cppad_python_error_handler) { }
};
static CppADErrorHandlerRegistrar cppad_error_handler_registrar;
}

#include "Solver.h"
#include "Callback.h"

#include "DualSolver.h"
#include "PrimalSolver.h"
#include "Report.h"
#include "Results.h"
#include "Settings.h"
#include "TaskHandler.h"
#include "Timing.h"
#include "Utilities.h"

#include "Model/Problem.h"
#include "Model/Variables.h"
#include "Model/Terms.h"
#include "Model/Constraints.h"
#include "Model/ObjectiveFunction.h"
#include "Model/NonlinearExpressions.h"
#include "Model/Simplifications.h"

#include <sstream>

#ifdef HAS_GAMS
#include "ModelingSystem/ModelingSystemGAMS.h"
#endif
#ifdef HAS_AMPL
#include "ModelingSystem/ModelingSystemAMPL.h"
#endif
#include "ModelingSystem/ModelingSystemOSiL.h"

#include "SolutionStrategy/SolutionStrategySingleTree.h"
#include "SolutionStrategy/SolutionStrategyMultiTree.h"
#include "SolutionStrategy/SolutionStrategyMIQCQP.h"
#include "SolutionStrategy/SolutionStrategyNLP.h"

#include "../Tasks/TaskPerformBoundTightening.h"
#include "../Tasks/TaskReformulateProblem.h"

#include <map>
#include <unordered_set>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

#ifdef HAS_GUROBI
#include "gurobi_c++.h"
#endif

// pybind11 converts None to an empty pointer, which SHOT dereferences without checking, e.g., when a variable or a
// constraint of None is added to a problem or printed. For the classes of the model, None therefore does not match an
// argument, so that a function given it, or a list containing it, raises TypeError
#define SHOTPY_REJECT_NONE(Type)                                                                                       \
    template <> class type_caster<std::shared_ptr<Type>> : public copyable_holder_caster<Type, std::shared_ptr<Type>>  \
    {                                                                                                                  \
    public:                                                                                                            \
        bool load(handle source, bool convert)                                                                         \
        {                                                                                                              \
            if(source.is_none())                                                                                       \
                return false;                                                                                          \
                                                                                                                       \
            return copyable_holder_caster<Type, std::shared_ptr<Type>>::load(source, convert);                         \
        }                                                                                                              \
    };

namespace pybind11::detail
{
SHOTPY_REJECT_NONE(SHOT::Variable)
SHOTPY_REJECT_NONE(SHOT::NonlinearExpression)
SHOTPY_REJECT_NONE(SHOT::LinearTerm)
SHOTPY_REJECT_NONE(SHOT::QuadraticTerm)
SHOTPY_REJECT_NONE(SHOT::MonomialTerm)
SHOTPY_REJECT_NONE(SHOT::SignomialTerm)
SHOTPY_REJECT_NONE(SHOT::SignomialElement)
SHOTPY_REJECT_NONE(SHOT::NumericConstraint)
SHOTPY_REJECT_NONE(SHOT::LinearConstraint)
SHOTPY_REJECT_NONE(SHOT::QuadraticConstraint)
SHOTPY_REJECT_NONE(SHOT::NonlinearConstraint)
SHOTPY_REJECT_NONE(SHOT::ObjectiveFunction)
SHOTPY_REJECT_NONE(SHOT::LinearObjectiveFunction)
SHOTPY_REJECT_NONE(SHOT::QuadraticObjectiveFunction)
SHOTPY_REJECT_NONE(SHOT::NonlinearObjectiveFunction)
SHOTPY_REJECT_NONE(SHOT::SpecialOrderedSet)
SHOTPY_REJECT_NONE(SHOT::Problem)
SHOTPY_REJECT_NONE(SHOT::Environment)
} // namespace pybind11::detail

#undef SHOTPY_REJECT_NONE

namespace SHOT
{
namespace py = pybind11;

// Forward declarations for operator overloads
NonlinearExpressionPtr wrapInExpression(VariablePtr var);
NonlinearExpressionPtr wrapInExpression(double value);
NonlinearExpressionPtr wrapInExpression(NonlinearExpressionPtr expr);

// Helper to wrap a Variable in an ExpressionVariable
NonlinearExpressionPtr wrapInExpression(VariablePtr var) { return std::make_shared<ExpressionVariable>(var); }

// Helper to wrap a constant in an ExpressionConstant
NonlinearExpressionPtr wrapInExpression(double value) { return std::make_shared<ExpressionConstant>(value); }

// Pass-through for expressions
NonlinearExpressionPtr wrapInExpression(NonlinearExpressionPtr expr) { return expr; }

// A constraint L <= f(x) <= U given by comparing variables, expressions and numbers, e.g., x1 * x2 <= 5, which
// Problem.addConstraint() turns into a constraint of the problem
struct ConstraintExpression
{
    NonlinearExpressionPtr expression;
    double lowerBound = SHOT_DBL_MIN;
    double upperBound = SHOT_DBL_MAX;
};

// SHOT marks a missing bound with SHOT_DBL_MIN or SHOT_DBL_MAX and compares with these values exactly, so an infinite
// bound of a constraint or variable is given that value. A bound that is NaN, or that no value can fulfil, e.g.,
// f <= -inf, raises ValueError
double toLowerBound(double value)
{
    if(std::isnan(value))
        throw py::value_error("A bound cannot be NaN.");

    if(value == std::numeric_limits<double>::infinity())
        throw py::value_error("No value is larger than or equal to inf, so the lower bound cannot be fulfilled.");

    return (value <= SHOT_DBL_MIN ? SHOT_DBL_MIN : value);
}

double toUpperBound(double value)
{
    if(std::isnan(value))
        throw py::value_error("A bound cannot be NaN.");

    if(value == -std::numeric_limits<double>::infinity())
        throw py::value_error("No value is smaller than or equal to -inf, so the upper bound cannot be fulfilled.");

    return (value >= SHOT_DBL_MAX ? SHOT_DBL_MAX : value);
}

// A bound given to a variable or set on it later: an infinite bound is a missing bound, and NaN raises ValueError. A
// bound that cannot be fulfilled, e.g., a lower bound of inf, is not rejected, since the bounds may be set one at a
// time
double toVariableBound(double value)
{
    if(std::isnan(value))
        throw py::value_error("A bound cannot be NaN.");

    if(value <= SHOT_DBL_MIN)
        return (SHOT_DBL_MIN);

    if(value >= SHOT_DBL_MAX)
        return (SHOT_DBL_MAX);

    return (value);
}

double toEqualityValue(double value)
{
    if(!std::isfinite(value))
        throw py::value_error(
            "The value of an equality constraint must be finite, but it is " + Utilities::toString(value) + ".");

    return (value);
}

// The value of an expression without variables, e.g., 2 * 3, or nullopt if it has a variable
std::optional<double> constantValue(const NonlinearExpressionPtr& expression)
{
    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Constant:
        return (std::static_pointer_cast<ExpressionConstant>(expression)->constant);
    case E_NonlinearExpressionTypes::Variable:
        return (std::nullopt);
    case E_NonlinearExpressionTypes::Negate:
    {
        auto value = constantValue(std::static_pointer_cast<ExpressionNegate>(expression)->child);
        return (value ? std::optional<double>(-*value) : std::nullopt);
    }
    case E_NonlinearExpressionTypes::Sum:
    case E_NonlinearExpressionTypes::Product:
    {
        bool isSum = (expression->getType() == E_NonlinearExpressionTypes::Sum);
        double result = isSum ? 0.0 : 1.0;

        for(auto& C : std::static_pointer_cast<ExpressionGeneral>(expression)->children)
        {
            auto value = constantValue(C);

            if(!value)
                return (std::nullopt);

            result = isSum ? result + *value : result * *value;
        }

        return (result);
    }
    case E_NonlinearExpressionTypes::Divide:
    {
        auto divide = std::static_pointer_cast<ExpressionDivide>(expression);
        auto numerator = constantValue(divide->firstChild);
        auto denominator = numerator ? constantValue(divide->secondChild) : std::nullopt;
        return (denominator ? std::optional<double>(*numerator / *denominator) : std::nullopt);
    }
    default:
        // e.g. exp(2), which has a value only if its argument has no variables
        if(auto unary = std::dynamic_pointer_cast<ExpressionUnary>(expression); unary && constantValue(unary->child))
            return (expression->calculate(VectorDouble()));

        if(auto binary = std::dynamic_pointer_cast<ExpressionBinary>(expression);
            binary && constantValue(binary->firstChild) && constantValue(binary->secondChild))
            return (expression->calculate(VectorDouble()));

        return (std::nullopt);
    }
}

// The terms and the constant of a linear expression, with the terms in the order their variables first appear
struct LinearParts
{
    std::vector<std::pair<VariablePtr, double>> terms;
    std::unordered_map<Variable*, size_t> positions;
    double constant = 0.0;

    void add(const VariablePtr& variable, double coefficient)
    {
        auto [position, isNew] = positions.emplace(variable.get(), terms.size());

        if(isNew)
            terms.emplace_back(variable, coefficient);
        else
            terms[position->second].second += coefficient;
    }
};

// Adds the expression, multiplied by the factor, to the linear parts. Returns false if the expression is not linear
bool collectLinearParts(const NonlinearExpressionPtr& expression, double factor, LinearParts& parts)
{
    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Constant:
        parts.constant += factor * std::static_pointer_cast<ExpressionConstant>(expression)->constant;
        return (true);
    case E_NonlinearExpressionTypes::Variable:
        parts.add(std::static_pointer_cast<ExpressionVariable>(expression)->variable, factor);
        return (true);
    case E_NonlinearExpressionTypes::Negate:
        return (collectLinearParts(std::static_pointer_cast<ExpressionNegate>(expression)->child, -factor, parts));
    case E_NonlinearExpressionTypes::Sum:
    {
        for(auto& C : std::static_pointer_cast<ExpressionSum>(expression)->children)
        {
            if(!collectLinearParts(C, factor, parts))
                return (false);
        }

        return (true);
    }
    case E_NonlinearExpressionTypes::Product:
    {
        // Linear if all factors but one are constants
        NonlinearExpressionPtr nonconstantFactor;

        for(auto& C : std::static_pointer_cast<ExpressionProduct>(expression)->children)
        {
            if(auto value = constantValue(C))
                factor *= *value;
            else if(nonconstantFactor)
                return (false);
            else
                nonconstantFactor = C;
        }

        if(!nonconstantFactor)
        {
            parts.constant += factor;
            return (true);
        }

        return (collectLinearParts(nonconstantFactor, factor, parts));
    }
    case E_NonlinearExpressionTypes::Divide:
    {
        auto divide = std::static_pointer_cast<ExpressionDivide>(expression);
        auto denominator = constantValue(divide->secondChild);

        if(!denominator || *denominator == 0.0)
            return (false);

        return (collectLinearParts(divide->firstChild, factor / *denominator, parts));
    }
    default:
    {
        auto value = constantValue(expression);

        if(!value)
            return (false);

        parts.constant += factor * *value;
        return (true);
    }
    }
}

const char* constraintExpressionInBooleanContext
    = "A comparison of SHOTpy variables or expressions creates a constraint, not a truth value. Use 'is' or 'is not' "
      "to check whether two variables are the same object, and SHOTpy.inequality(lower, expression, upper) instead of "
      "a chained comparison such as lower <= expression <= upper.";

// Adds the comparison operators <=, >= and ==, which create a ConstraintExpression, to the class of variables or
// expressions. A comparison with a variable or an expression on both sides moves everything to the left side, e.g.,
// f <= g becomes f - g <= 0
template <typename Self, typename Class> void addComparisonOperators(Class& pythonClass)
{
    auto difference = [](NonlinearExpressionPtr first, NonlinearExpressionPtr second) -> NonlinearExpressionPtr
    { return std::make_shared<ExpressionSum>(first, std::make_shared<ExpressionNegate>(second)); };

    // pybind11 passes None as a null pointer to an argument of a class, so the operators with a variable or an
    // expression on the other side do not accept None, and Python compares the objects instead, e.g., x == None is
    // False
    auto other = py::arg("other").none(false);

    pythonClass
        .def(
            "__le__", [](Self self, double value)
            { return ConstraintExpression { wrapInExpression(self), SHOT_DBL_MIN, toUpperBound(value) }; },
            py::is_operator())
        .def(
            "__le__",
            [difference](Self self, VariablePtr variable)
            {
                return ConstraintExpression { difference(wrapInExpression(self), wrapInExpression(variable)),
                    SHOT_DBL_MIN, 0.0 };
            },
            py::is_operator(), other)
        .def(
            "__le__", [difference](Self self, NonlinearExpressionPtr expression)
            { return ConstraintExpression { difference(wrapInExpression(self), expression), SHOT_DBL_MIN, 0.0 }; },
            py::is_operator(), other)
        .def(
            "__ge__", [](Self self, double value)
            { return ConstraintExpression { wrapInExpression(self), toLowerBound(value), SHOT_DBL_MAX }; },
            py::is_operator())
        .def(
            "__ge__",
            [difference](Self self, VariablePtr variable)
            {
                return ConstraintExpression { difference(wrapInExpression(self), wrapInExpression(variable)), 0.0,
                    SHOT_DBL_MAX };
            },
            py::is_operator(), other)
        .def(
            "__ge__", [difference](Self self, NonlinearExpressionPtr expression)
            { return ConstraintExpression { difference(wrapInExpression(self), expression), 0.0, SHOT_DBL_MAX }; },
            py::is_operator(), other)
        .def(
            "__eq__",
            [](Self self, double value)
            {
                auto equalityValue = toEqualityValue(value);
                return ConstraintExpression { wrapInExpression(self), equalityValue, equalityValue };
            },
            py::is_operator())
        .def(
            "__eq__",
            [difference](Self self, VariablePtr variable)
            {
                return ConstraintExpression { difference(wrapInExpression(self), wrapInExpression(variable)), 0.0,
                    0.0 };
            },
            py::is_operator(), other)
        .def(
            "__eq__", [difference](Self self, NonlinearExpressionPtr expression)
            { return ConstraintExpression { difference(wrapInExpression(self), expression), 0.0, 0.0 }; },
            py::is_operator(), other)
        // != would negate the ConstraintExpression of ==, which is not a truth value
        .def(
            "__ne__", [](Self, double) -> bool { throw py::type_error(constraintExpressionInBooleanContext); },
            py::is_operator())
        .def(
            "__ne__", [](Self, VariablePtr) -> bool { throw py::type_error(constraintExpressionInBooleanContext); },
            py::is_operator(), other)
        .def(
            "__ne__", [](Self, NonlinearExpressionPtr) -> bool
            { throw py::type_error(constraintExpressionInBooleanContext); }, py::is_operator(), other)
        // Defining __eq__ removes the hash, which keeps variables and expressions usable in sets and as dict keys
        .def("__hash__", [](Self self) { return std::hash<const void*>()(self.get()); });
}

// Throws ValueError if the point does not have a value for every variable of the problem, since the functions index the
// point by the indexes of the variables without checking it. A longer point is allowed: the reformulated problem has
// the variables of the original problem first, so a function of the original problem can be evaluated at its points
void checkPointSize(const Problem& problem, const VectorDouble& point)
{
    if(point.size() < problem.allVariables.size())
    {
        throw py::value_error("The point has " + std::to_string(point.size()) + " values, but the problem has "
            + std::to_string(problem.allVariables.size()) + " variables.");
    }
}

// The variables of a constraint or objective function that has not been added to a problem have no indexes in the point
void checkPointSize(const std::weak_ptr<Problem>& ownerProblem, const VectorDouble& point)
{
    auto problem = ownerProblem.lock();

    if(!problem)
        throw py::value_error("The function cannot be evaluated before it has been added to a problem.");

    checkPointSize(*problem, point);
}

// finalize() decides the classes of the constraints and the objective function and rewrites the model, and it is not
// run again, so what is added after it would not be part of the model that is solved
void checkCanAdd(const Problem& problem)
{
    if(problem.hasBeenFinalized())
        throw std::runtime_error("The problem has been finalized, so nothing can be added to it.");
}

// A variable, constraint or objective function belongs to the problem it is added to, which gives it its index, so it
// cannot be added twice or to another problem
template <typename T> void checkNotAdded(const T& element, const std::string& description)
{
    if(!element.ownerProblem.expired())
        throw py::value_error("The " + description + " has already been added to a problem.");
}

// Checks all the elements before any of them is added, so that a list that cannot be added leaves the problem unchanged
template <typename Container, typename Describe>
void checkNotAddedOrRepeated(const Container& elements, Describe describe)
{
    std::unordered_set<const void*> seen;

    for(auto& E : elements)
    {
        checkNotAdded(*E, describe(E));

        if(!seen.insert(E.get()).second)
            throw py::value_error("The " + describe(E) + " is in the list more than once.");
    }
}

// A function with a variable that has not been added to the problem would be evaluated with the index the variable has
// in another problem, or with the index -1
void checkVariableInProblem(const Problem& problem, const VariablePtr& variable, const std::string& owner)
{
    int index = variable->getIndex();

    if(index < 0 || index >= (int)problem.allVariables.size() || problem.allVariables[index] != variable)
    {
        throw py::value_error(
            "The variable '" + variable->name + "' in " + owner + " has not been added to the problem.");
    }
}

void checkVariablesInProblem(const Problem& problem, const NonlinearExpressionPtr& expression, const std::string& owner)
{
    if(!expression)
        return;

    if(auto variable = std::dynamic_pointer_cast<ExpressionVariable>(expression))
    {
        checkVariableInProblem(problem, variable->variable, owner);
    }
    else if(auto unary = std::dynamic_pointer_cast<ExpressionUnary>(expression))
    {
        checkVariablesInProblem(problem, unary->child, owner);
    }
    else if(auto binary = std::dynamic_pointer_cast<ExpressionBinary>(expression))
    {
        checkVariablesInProblem(problem, binary->firstChild, owner);
        checkVariablesInProblem(problem, binary->secondChild, owner);
    }
    else if(auto general = std::dynamic_pointer_cast<ExpressionGeneral>(expression))
    {
        for(auto& C : general->children)
            checkVariablesInProblem(problem, C, owner);
    }
}

// The constraints and the objective function have the same terms, in classes of their own
template <typename Linear, typename Quadratic, typename Nonlinear, typename Function>
void checkFunctionVariablesInProblem(const Problem& problem, const Function& function, const std::string& owner)
{
    if(auto linear = std::dynamic_pointer_cast<Linear>(function))
    {
        for(auto& T : linear->linearTerms)
            checkVariableInProblem(problem, T->variable, owner);
    }

    if(auto quadratic = std::dynamic_pointer_cast<Quadratic>(function))
    {
        for(auto& T : quadratic->quadraticTerms)
        {
            checkVariableInProblem(problem, T->firstVariable, owner);
            checkVariableInProblem(problem, T->secondVariable, owner);
        }
    }

    if(auto nonlinear = std::dynamic_pointer_cast<Nonlinear>(function))
    {
        for(auto& T : nonlinear->monomialTerms)
        {
            for(auto& V : T->variables)
                checkVariableInProblem(problem, V, owner);
        }

        for(auto& T : nonlinear->signomialTerms)
        {
            for(auto& E : T->elements)
                checkVariableInProblem(problem, E->variable, owner);
        }

        checkVariablesInProblem(problem, nonlinear->nonlinearExpression, owner);
    }
}

// The variables are checked when a constraint or the objective function is added, since the problem takes ownership of
// the variables of its expression, and again by finalize(), since terms can be added to it afterwards
void checkVariablesInProblem(const Problem& problem, const NumericConstraintPtr& constraint)
{
    checkFunctionVariablesInProblem<LinearConstraint, QuadraticConstraint, NonlinearConstraint>(
        problem, constraint, "the constraint '" + constraint->name + "'");
}

void checkVariablesInProblem(const Problem& problem, const ObjectiveFunctionPtr& objective)
{
    checkFunctionVariablesInProblem<LinearObjectiveFunction, QuadraticObjectiveFunction, NonlinearObjectiveFunction>(
        problem, objective, "the objective function");
}

void checkVariablesInProblem(const Problem& problem)
{
    for(auto& C : problem.numericConstraints)
        checkVariablesInProblem(problem, C);

    if(problem.objectiveFunction)
        checkVariablesInProblem(problem, problem.objectiveFunction);

    for(auto& S : problem.specialOrderedSets)
    {
        for(auto& V : S->variables)
            checkVariableInProblem(problem, V, "a special ordered set");
    }
}

// Adds a constraint given by a comparison. Without a name, it is named constraint_<index>
void addConstraintExpression(Problem& problem, const ConstraintExpression& constraint, std::string name)
{
    // A prefix that is unlikely to be used in a name the user gives, since checking the names of all the
    // constraints would make adding constraints quadratic in their number
    if(name.empty())
        name = "constraint_" + std::to_string(problem.numericConstraints.size());

    checkVariablesInProblem(problem, constraint.expression, "the constraint '" + name + "'");

    // A linear constraint is created as such, as it would be from its class. finalize() splits a nonlinear
    // constraint with two bounds before it extracts its terms, so a linear range given as a nonlinear
    // constraint would become two constraints
    if(LinearParts parts; collectLinearParts(constraint.expression, 1.0, parts))
    {
        std::vector<LinearTermPtr> terms;

        for(auto& [variable, coefficient] : parts.terms)
        {
            if(coefficient != 0.0)
                terms.push_back(std::make_shared<LinearTerm>(coefficient, variable));
        }

        auto linearConstraint = std::make_shared<LinearConstraint>(
            name, LinearTerms(terms), constraint.lowerBound, constraint.upperBound);
        linearConstraint->constant = parts.constant;
        problem.add(linearConstraint);
        return;
    }

    // finalize() extracts the terms of the expression and changes the class of the constraint if nothing
    // nonlinear is left
    problem.add(std::make_shared<NonlinearConstraint>(
        name, constraint.expression, constraint.lowerBound, constraint.upperBound));
}

// SHOT gives a missing bound or objective gap as SHOT_DBL_MAX or SHOT_DBL_MIN, and a dual bound that has not been set
// yet as NaN. The results of the solver are given as inf or -inf instead, as in the callback contexts, with the sign of
// a bound that excludes nothing. Before a problem has been set, the direction is taken to be minimization
double toResult(double value, double missing)
{
    return ((std::isnan(value) || std::abs(value) >= SHOT_DBL_MAX) ? missing : value);
}

bool isMinimization(Solver& solver)
{
    auto problem = solver.getOriginalProblem();
    return (!problem || !problem->objectiveFunction || problem->objectiveFunction->properties.isMinimize);
}

// The sequence protocol of a container of the model: len(), indexing, where a negative index counts from the end and an
// index outside the container raises IndexError, and iteration. Without __iter__, Python iterates by indexing until
// IndexError, so an unchecked index read past the end
template <typename Class> Class addSequenceProtocol(Class pythonClass)
{
    using Container = typename Class::type;

    pythonClass.def("__len__", [](const Container& self) { return self.size(); })
        .def(
            "__getitem__",
            [](const Container& self, py::ssize_t index)
            {
                auto size = static_cast<py::ssize_t>(self.size());

                if(index < 0)
                    index += size;

                if(index < 0 || index >= size)
                    throw py::index_error("Index out of range");

                return self[index];
            },
            py::arg("index"))
        .def(
            "__iter__", [](const Container& self) { return py::make_iterator(self.begin(), self.end()); },
            py::keep_alive<0, 1>());

    return pythonClass;
}

// The locations of a callback, given as a CallbackLocation, an integer mask (e.g., from combining them with |), or an
// iterable of CallbackLocation
E_CallbackLocation toCallbackLocations(py::handle locations)
{
    std::uint64_t mask = 0;

    if(py::isinstance<E_CallbackLocation>(locations))
    {
        mask = static_cast<std::uint64_t>(locations.cast<E_CallbackLocation>());
    }
    else if(py::isinstance<py::int_>(locations) && !py::isinstance<py::bool_>(locations))
    {
        if(locations.cast<py::int_>() < py::int_(0))
            throw py::value_error("The mask of callback locations cannot be negative.");

        mask = locations.cast<std::uint64_t>();
    }
    else if(py::isinstance<py::iterable>(locations) && !py::isinstance<py::str>(locations))
    {
        for(auto location : locations)
        {
            if(!py::isinstance<E_CallbackLocation>(location))
                throw py::type_error("The callback locations must be CallbackLocation values.");

            mask |= static_cast<std::uint64_t>(location.cast<E_CallbackLocation>());
        }
    }
    else
    {
        throw py::type_error(
            "The callback locations must be a CallbackLocation, a combination of them with |, or a list of them.");
    }

    if(mask == 0)
        throw py::value_error("At least one callback location must be given.");

    if((mask & ~static_cast<std::uint64_t>(AllCallbackLocations)) != 0)
        throw py::value_error("The mask " + std::to_string(mask) + " contains an unknown callback location.");

    return (static_cast<E_CallbackLocation>(mask));
}

PYBIND11_MODULE(SHOTpy, m)
{
    m.doc() = "SHOTpy";

    // The classes and enums that the signatures of functions bound before them use are created here and given their
    // methods below. pybind11 writes the signature of a function when it is bound, and gives a type that is not
    // registered yet by its C++ name, e.g., SHOT::Variables, which is then also what the type stubs show
    py::enum_<E_VariableType> variableTypeEnum(m, "VariableType");
    py::enum_<E_ObjectiveFunctionDirection> objectiveDirectionEnum(m, "ObjectiveDirection");
    py::enum_<ES_MIPSolver> mipSolverEnum(m, "MIPSolver");
    py::enum_<ES_PrimalNLPSolver> nlpSolverEnum(m, "PrimalNLPSolver");
    py::enum_<E_NonlinearExpressionTypes> expressionTypeEnum(m, "ExpressionType");
    py::enum_<E_ModelReturnStatus> modelReturnStatusEnum(m, "ModelReturnStatus", py::arithmetic());
    py::enum_<E_TerminationReason> terminationReasonEnum(m, "TerminationReason", py::arithmetic());

    py::class_<Variable, std::shared_ptr<Variable>> variableClass(m, "Variable");
    py::class_<VariableProperties> variablePropertiesClass(m, "VariableProperties");
    py::class_<Variables> variablesClass(m, "Variables");
    py::class_<NonlinearExpression, NonlinearExpressionPtr> expressionClass(m, "Expression");
    py::class_<ConstraintExpression> constraintExpressionClass(m, "ConstraintExpression",
        "A constraint lowerBound <= expression <= upperBound created by comparing variables, expressions and numbers,\n"
        "e.g., x1 * x2 <= 5, which Problem.addConstraint() adds to the problem");
    py::class_<NumericConstraintValue> numericConstraintValueClass(m, "NumericConstraintValue",
        "The value of a constraint L <= f(x) <= U at a point, and how much it deviates from its bounds");
    py::class_<Environment, std::shared_ptr<Environment>> environmentClass(m, "Environment");
    py::class_<Solver> solverClass(m, "Solver");
    py::class_<PrimalSolution> primalSolutionClass(m, "PrimalSolution");
    py::class_<SolutionStatistics> solutionStatisticsClass(m, "SolutionStatistics");
    py::enum_<E_CallbackLocation> callbackLocation(m, "CallbackLocation");
    py::class_<CallbackContext, std::shared_ptr<CallbackContext>> callbackContextClass(m, "CallbackContext",
        "The state of the solver and the actions available to a callback. Values that are not available yet, e.g.,\n"
        "the dual bound before the first dual problem has been solved, are infinite.");

    // ===== Constants =====
    m.attr("SHOT_DBL_MAX") = SHOT_DBL_MAX;
    m.attr("SHOT_DBL_MIN") = SHOT_DBL_MIN;

    // ===== Modeling System Availability =====
    // These constants indicate which modeling systems are available in this build
#ifdef HAS_GAMS
    m.attr("HAS_GAMS") = true;
#else
    m.attr("HAS_GAMS") = false;
#endif

#ifdef HAS_AMPL
    m.attr("HAS_AMPL") = true;
#else
    m.attr("HAS_AMPL") = false;
#endif

    // OSiL is always available
    m.attr("HAS_OSIL") = true;

    // Modeling system enum
    py::enum_<ES_ModelingSystem>(m, "ModelingSystem")
        .value("OSiL", ES_ModelingSystem::OSiL)
        .value("GAMS", ES_ModelingSystem::GAMS)
        .value("AMPL", ES_ModelingSystem::AMPL)
        .value("API", ES_ModelingSystem::API);

    // Function to get list of supported modeling systems - uses C++ API directly
    m.def("getSupportedModelingSystems", &Solver::getSupportedModelingSystems,
        "Returns a list of modeling systems supported in this build");

    // ===== MIP Solver Availability =====
    // These constants indicate which MIP solvers are available in this build
#ifdef HAS_CPLEX
    m.attr("HAS_CPLEX") = true;
#else
    m.attr("HAS_CPLEX") = false;
#endif

#ifdef HAS_GUROBI
    m.attr("HAS_GUROBI") = true;
#else
    m.attr("HAS_GUROBI") = false;
#endif

#ifdef HAS_CBC
    m.attr("HAS_CBC") = true;
#else
    m.attr("HAS_CBC") = false;
#endif

#ifdef HAS_HIGHS
    m.attr("HAS_HIGHS") = true;
#else
    m.attr("HAS_HIGHS") = false;
#endif

    // Function to get list of supported MIP solvers - uses C++ API directly
    m.def("getSupportedMIPSolvers", &Solver::getSupportedMIPSolvers,
        "Returns a list of MIP solvers supported in this build");

    // ===== NLP Solver Availability =====
    // These constants indicate which NLP solvers are available in this build
#ifdef HAS_IPOPT
    m.attr("HAS_IPOPT") = true;
#else
    m.attr("HAS_IPOPT") = false;
#endif

    // SHOT's internal NLP solver is always available
    m.attr("HAS_SHOT_NLP") = true;

    // GAMS NLP solver is available when GAMS is available
#ifdef HAS_GAMS
    m.attr("HAS_GAMS_NLP") = true;
#else
    m.attr("HAS_GAMS_NLP") = false;
#endif

    // Function to get list of supported NLP solvers - uses C++ API directly
    m.def("getSupportedNLPSolvers", &Solver::getSupportedNLPSolvers,
        "Returns a list of NLP solvers supported in this build");

    // ===== Variable Types Enum =====
    variableTypeEnum.value("Real", E_VariableType::Real)
        .value("Binary", E_VariableType::Binary)
        .value("Integer", E_VariableType::Integer)
        .value("Semicontinuous", E_VariableType::Semicontinuous)
        .value("Semiinteger", E_VariableType::Semiinteger);

    // ===== SOS Type Enum =====
    py::enum_<E_SOSType>(m, "SOSType").value("One", E_SOSType::One).value("Two", E_SOSType::Two);

    // ===== Objective Direction Enum =====
    objectiveDirectionEnum.value("Minimize", E_ObjectiveFunctionDirection::Minimize)
        .value("Maximize", E_ObjectiveFunctionDirection::Maximize);

    // ===== Convexity Enum =====
    py::enum_<E_Convexity>(m, "Convexity")
        .value("Linear", E_Convexity::Linear)
        .value("Convex", E_Convexity::Convex)
        .value("Concave", E_Convexity::Concave)
        .value("Nonconvex", E_Convexity::Nonconvex)
        .value("Unknown", E_Convexity::Unknown)
        .value("NotSet", E_Convexity::NotSet);

    // ===== Problem Convexity Enum =====
    py::enum_<E_ProblemConvexity>(m, "ProblemConvexity")
        .value("Convex", E_ProblemConvexity::Convex)
        .value("Nonconvex", E_ProblemConvexity::Nonconvex)
        .value("NotSet", E_ProblemConvexity::NotSet);

    // ===== Settings Enums =====
    // These enums are used for type-safe setting values

    // Hyperplane cut strategy: ESH or ECP
    py::enum_<ES_HyperplaneCutStrategy>(m, "HyperplaneCutStrategy")
        .value("ESH", ES_HyperplaneCutStrategy::ESH)
        .value("ECP", ES_HyperplaneCutStrategy::ECP);

    // Iteration output detail level
    py::enum_<ES_IterationOutputDetail>(m, "IterationOutputDetail")
        .value("Full", ES_IterationOutputDetail::Full)
        .value("ObjectiveGapUpdates", ES_IterationOutputDetail::ObjectiveGapUpdates)
        .value("ObjectiveGapUpdatesAndNLPCalls", ES_IterationOutputDetail::ObjectiveGapUpdatesAndNLPCalls);

    // MIP solver selection
    mipSolverEnum.value("Cplex", ES_MIPSolver::Cplex)
        .value("Gurobi", ES_MIPSolver::Gurobi)
        .value("Cbc", ES_MIPSolver::Cbc)
        .value("Highs", ES_MIPSolver::Highs)
        .value("NotUsed", ES_MIPSolver::NotUsed);

    // Source of fixed MIP solution point for NLP
    py::enum_<ES_PrimalNLPFixedPoint>(m, "PrimalNLPFixedPoint")
        .value("AllSolutions", ES_PrimalNLPFixedPoint::AllSolutions)
        .value("FirstSolution", ES_PrimalNLPFixedPoint::FirstSolution)
        .value("AllFeasibleSolutions", ES_PrimalNLPFixedPoint::AllFeasibleSolutions)
        .value("FirstAndFeasibleSolutions", ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions)
        .value("SmallestDeviationSolution", ES_PrimalNLPFixedPoint::SmallestDeviationSolution);

    // Problem formulation source for NLP
    py::enum_<ES_PrimalNLPProblemSource>(m, "PrimalNLPProblemSource")
        .value("OriginalProblem", ES_PrimalNLPProblemSource::OriginalProblem)
        .value("ReformulatedProblem", ES_PrimalNLPProblemSource::ReformulatedProblem)
        .value("Both", ES_PrimalNLPProblemSource::Both);

    // NLP solver selection
    nlpSolverEnum.value("Ipopt", ES_PrimalNLPSolver::Ipopt)
        .value("GAMS", ES_PrimalNLPSolver::GAMS)
        .value("SHOT", ES_PrimalNLPSolver::SHOT)
        .value("Uno", ES_PrimalNLPSolver::Uno)
        .value("NotUsed", ES_PrimalNLPSolver::NotUsed);

    // NLP solver call strategy
    py::enum_<ES_PrimalNLPStrategy>(m, "PrimalNLPStrategy")
        .value("AlwaysUse", ES_PrimalNLPStrategy::AlwaysUse)
        .value("IterationOrTime", ES_PrimalNLPStrategy::IterationOrTime)
        .value("IterationOrTimeAndAllFeasibleSolutions", ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions);

    // Quadratic problem handling strategy
    py::enum_<ES_QuadraticProblemStrategy>(m, "QuadraticProblemStrategy")
        .value("Nonlinear", ES_QuadraticProblemStrategy::Nonlinear)
        .value("QuadraticObjective", ES_QuadraticProblemStrategy::QuadraticObjective)
        .value("ConvexQuadraticallyConstrained", ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained)
        .value("NonconvexQuadraticallyConstrained", ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained);

    // Tree strategy: single-tree or multi-tree
    py::enum_<ES_TreeStrategy>(m, "TreeStrategy")
        .value("MultiTree", ES_TreeStrategy::MultiTree)
        .value("SingleTree", ES_TreeStrategy::SingleTree);

    // ===== Variable Class =====
    variableClass
        .def(py::init(
                 [](std::string name, E_VariableType type, double lowerBound, double upperBound)
                 {
                     return std::make_shared<Variable>(
                         name, type, toVariableBound(lowerBound), toVariableBound(upperBound));
                 }),
            py::arg("name"), py::arg("type"), py::arg("lowerBound"), py::arg("upperBound"),
            "Create a variable, which is added to a problem with Problem.addVariable(). Problem.addVariable(name,\n"
            "type, lowerBound, upperBound) creates and adds it in one step. inf and -inf mean that there is no bound")
        .def(py::init(
                 [](std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound)
                 {
                     return std::make_shared<Variable>(
                         name, type, toVariableBound(lowerBound), toVariableBound(upperBound), semiBound);
                 }),
            py::arg("name"), py::arg("type"), py::arg("lowerBound"), py::arg("upperBound"), py::arg("semiBound"),
            "Create a variable, which is added to a problem with Problem.addVariable(). Problem.addVariable(name,\n"
            "type, lowerBound, upperBound) creates and adds it in one step. inf and -inf mean that there is no bound")
        .def_readwrite("name", &Variable::name, "The name of the variable")
        // Assigned by the problem the variable is added to, so read only
        .def_property_readonly("index", &Variable::getIndex,
            "The index of the variable in the problem it has been added to, -1 before that. It is the position of\n"
            "its value in a point, e.g., of a solution")
        // The bound vectors of the problem are updated as well, since they are otherwise only recalculated when
        // variables are added
        .def_property(
            "lowerBound", [](const Variable& self) { return (self.lowerBound); },
            [](Variable& self, double value)
            {
                self.lowerBound = toVariableBound(value);

                if(auto problem = self.ownerProblem.lock())
                    problem->updateVariableBoundVectors(self);
            },
            "The lower bound; -inf is stored as SHOT_DBL_MIN, i.e., no bound. Changing it after finalize()\n"
            "updates the bounds of the problem")
        .def_property(
            "upperBound", [](const Variable& self) { return (self.upperBound); },
            [](Variable& self, double value)
            {
                self.upperBound = toVariableBound(value);

                if(auto problem = self.ownerProblem.lock())
                    problem->updateVariableBoundVectors(self);
            },
            "The upper bound; inf is stored as SHOT_DBL_MAX, i.e., no bound. Changing it after finalize()\n"
            "updates the bounds of the problem")
        .def_readwrite("semiBound", &Variable::semiBound,
            "For a semicontinuous or semiinteger variable, the value is 0 or between semiBound and upperBound (or\n"
            "between lowerBound and semiBound if semiBound is negative)")
        .def_readonly("properties", &Variable::properties,
            "Properties of the variable, e.g., its type and whether it is in nonlinear terms")
        .def("__repr__",
            [](const Variable& v) { return "<Variable '" + v.name + "' index=" + std::to_string(v.getIndex()) + ">"; })
        // Operator overloads for natural expression building
        .def(
            "__add__", [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(wrapInExpression(self), wrapInExpression(other)); },
            py::is_operator())
        .def(
            "__add__", [](VariablePtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(wrapInExpression(self), wrapInExpression(other)); },
            py::is_operator())
        .def(
            "__radd__", [](VariablePtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(wrapInExpression(other), wrapInExpression(self)); },
            py::is_operator())
        .def(
            "__add__", [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(wrapInExpression(self), other); }, py::is_operator())
        .def(
            "__radd__", [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(other, wrapInExpression(self)); }, py::is_operator())
        .def(
            "__sub__",
            [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr
            {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(self), std::make_shared<ExpressionNegate>(wrapInExpression(other)));
            },
            py::is_operator())
        .def(
            "__sub__", [](VariablePtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(wrapInExpression(self), wrapInExpression(-other)); },
            py::is_operator())
        .def(
            "__rsub__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr
            {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(other), std::make_shared<ExpressionNegate>(wrapInExpression(self)));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(self), std::make_shared<ExpressionNegate>(other));
            },
            py::is_operator())
        .def(
            "__mul__", [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(wrapInExpression(self), wrapInExpression(other)); },
            py::is_operator())
        .def(
            "__mul__", [](VariablePtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(wrapInExpression(self), wrapInExpression(other)); },
            py::is_operator())
        .def(
            "__rmul__", [](VariablePtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(wrapInExpression(other), wrapInExpression(self)); },
            py::is_operator())
        .def(
            "__mul__", [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(wrapInExpression(self), other); }, py::is_operator())
        .def(
            "__rmul__", [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(other, wrapInExpression(self)); }, py::is_operator())
        .def(
            "__truediv__", [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(wrapInExpression(self), wrapInExpression(other)); },
            py::is_operator())
        .def(
            "__truediv__", [](VariablePtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(wrapInExpression(self), wrapInExpression(other)); },
            py::is_operator())
        .def(
            "__rtruediv__", [](VariablePtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(wrapInExpression(other), wrapInExpression(self)); },
            py::is_operator())
        .def(
            "__truediv__", [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(wrapInExpression(self), other); }, py::is_operator())
        .def(
            "__pow__",
            [](VariablePtr self, double exponent) -> NonlinearExpressionPtr
            {
                if(exponent == 2.0)
                    return std::make_shared<ExpressionSquare>(wrapInExpression(self));
                return std::make_shared<ExpressionPower>(wrapInExpression(self), wrapInExpression(exponent));
            },
            py::is_operator())
        .def(
            "__pow__", [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionPower>(wrapInExpression(self), wrapInExpression(other)); },
            py::is_operator())
        .def(
            "__pow__", [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionPower>(wrapInExpression(self), other); }, py::is_operator())
        .def(
            "__neg__", [](VariablePtr self) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionNegate>(wrapInExpression(self)); }, py::is_operator());

    addComparisonOperators<VariablePtr>(variableClass);

    // ===== VariableProperties Struct =====
    variablePropertiesClass
        .def_readonly("type", &VariableProperties::type, "The type of the variable, e.g., VariableType.Integer")
        .def_readonly(
            "isAuxiliary", &VariableProperties::isAuxiliary, "Whether the variable has been added by the reformulation")
        .def_readonly("isNonlinear", &VariableProperties::isNonlinear,
            "Whether the variable is in a nonlinear term of a constraint or the objective function")
        .def_readonly("inObjectiveFunction", &VariableProperties::inObjectiveFunction,
            "Whether the variable is in the objective function")
        .def_readonly("inLinearConstraints", &VariableProperties::inLinearConstraints,
            "Whether the variable is in a linear constraint")
        .def_readonly("inQuadraticConstraints", &VariableProperties::inQuadraticConstraints,
            "Whether the variable is in a quadratic constraint")
        .def_readonly("inNonlinearConstraints", &VariableProperties::inNonlinearConstraints,
            "Whether the variable is in a nonlinear constraint");

    // ===== NonlinearExpression Base Class =====
    expressionClass
        .def("getType", &NonlinearExpression::getType, "The type of the expression node, e.g., ExpressionType.Sum")
        .def("getConvexity", &NonlinearExpression::getConvexity,
            "The convexity of the expression as far as SHOT can determine it from its structure")
        .def("__repr__",
            [](NonlinearExpressionPtr self)
            {
                std::ostringstream oss;
                oss << *self;
                return "<Expression: " + oss.str() + ">";
            })
        // Operator overloads for expressions
        .def(
            "__add__", [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(self, other); }, py::is_operator())
        .def(
            "__add__", [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(self, wrapInExpression(other)); }, py::is_operator())
        .def(
            "__radd__", [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(wrapInExpression(other), self); }, py::is_operator())
        .def(
            "__add__", [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(self, wrapInExpression(other)); }, py::is_operator())
        .def(
            "__sub__", [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(self, std::make_shared<ExpressionNegate>(other)); },
            py::is_operator())
        .def(
            "__sub__", [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionSum>(self, wrapInExpression(-other)); }, py::is_operator())
        .def(
            "__rsub__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(other), std::make_shared<ExpressionNegate>(self));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr
            {
                return std::make_shared<ExpressionSum>(
                    self, std::make_shared<ExpressionNegate>(wrapInExpression(other)));
            },
            py::is_operator())
        .def(
            "__mul__", [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(self, other); }, py::is_operator())
        .def(
            "__mul__", [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(self, wrapInExpression(other)); }, py::is_operator())
        .def(
            "__rmul__", [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(wrapInExpression(other), self); }, py::is_operator())
        .def(
            "__mul__", [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionProduct>(self, wrapInExpression(other)); }, py::is_operator())
        .def(
            "__truediv__", [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(self, other); }, py::is_operator())
        .def(
            "__truediv__", [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(self, wrapInExpression(other)); }, py::is_operator())
        .def(
            "__rtruediv__", [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(wrapInExpression(other), self); }, py::is_operator())
        .def(
            "__truediv__", [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionDivide>(self, wrapInExpression(other)); }, py::is_operator())
        .def(
            "__pow__",
            [](NonlinearExpressionPtr self, double exponent) -> NonlinearExpressionPtr
            {
                if(exponent == 2.0)
                    return std::make_shared<ExpressionSquare>(self);
                return std::make_shared<ExpressionPower>(self, wrapInExpression(exponent));
            },
            py::is_operator())
        .def(
            "__pow__", [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionPower>(self, other); }, py::is_operator())
        .def(
            "__pow__", [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionPower>(self, wrapInExpression(other)); }, py::is_operator())
        .def(
            "__neg__", [](NonlinearExpressionPtr self) -> NonlinearExpressionPtr
            { return std::make_shared<ExpressionNegate>(self); }, py::is_operator());

    addComparisonOperators<NonlinearExpressionPtr>(expressionClass);

    // ===== Constraints given by comparisons =====
    constraintExpressionClass
        .def_readonly("expression", &ConstraintExpression::expression,
            "The expression f of the constraint lowerBound <= f <= upperBound")
        .def_readonly("lowerBound", &ConstraintExpression::lowerBound, "The lower bound, SHOT_DBL_MIN if there is none")
        .def_readonly("upperBound", &ConstraintExpression::upperBound, "The upper bound, SHOT_DBL_MAX if there is none")
        .def("__repr__",
            [](const ConstraintExpression& self)
            {
                auto bound = [](double value)
                {
                    if(value <= SHOT_DBL_MIN)
                        return std::string("-inf");
                    if(value >= SHOT_DBL_MAX)
                        return std::string("inf");
                    return Utilities::toString(value);
                };

                std::ostringstream oss;
                oss << *self.expression;
                return "<ConstraintExpression: " + bound(self.lowerBound) + " <= " + oss.str()
                    + " <= " + bound(self.upperBound) + ">";
            })
        // A chained comparison, e.g., 1 <= f <= 5, is evaluated as (1 <= f) and (f <= 5), which needs the truth value
        // of 1 <= f and would drop its bound
        .def("__bool__",
            [](const ConstraintExpression&) -> bool { throw py::type_error(constraintExpressionInBooleanContext); });

    auto inequality = [](double lowerBound, NonlinearExpressionPtr expression, double upperBound)
    {
        lowerBound = toLowerBound(lowerBound);
        upperBound = toUpperBound(upperBound);

        if(lowerBound > upperBound)
            throw py::value_error("The lower bound " + Utilities::toString(lowerBound)
                + " is larger than the upper bound " + Utilities::toString(upperBound) + ".");

        return ConstraintExpression { expression, lowerBound, upperBound };
    };

    m.def(
        "inequality", [inequality](double lowerBound, VariablePtr variable, double upperBound)
        { return inequality(lowerBound, wrapInExpression(variable), upperBound); },
        "The constraint lower <= variable <= upper", py::arg("lower"), py::arg("expression").none(false),
        py::arg("upper"));
    m.def("inequality", inequality, "The constraint lower <= expression <= upper", py::arg("lower"),
        py::arg("expression").none(false), py::arg("upper"));

    // ===== NonlinearExpression Type Enum =====
    expressionTypeEnum.value("Constant", E_NonlinearExpressionTypes::Constant)
        .value("Var", E_NonlinearExpressionTypes::Variable) // Renamed to avoid conflict with Variable class
        .value("Negate", E_NonlinearExpressionTypes::Negate)
        .value("Invert", E_NonlinearExpressionTypes::Invert)
        .value("SquareRoot", E_NonlinearExpressionTypes::SquareRoot)
        .value("Log", E_NonlinearExpressionTypes::Log)
        .value("Exp", E_NonlinearExpressionTypes::Exp)
        .value("Square", E_NonlinearExpressionTypes::Square)
        .value("Cos", E_NonlinearExpressionTypes::Cos)
        .value("Sin", E_NonlinearExpressionTypes::Sin)
        .value("Tan", E_NonlinearExpressionTypes::Tan)
        .value("ArcCos", E_NonlinearExpressionTypes::ArcCos)
        .value("ArcSin", E_NonlinearExpressionTypes::ArcSin)
        .value("ArcTan", E_NonlinearExpressionTypes::ArcTan)
        .value("Abs", E_NonlinearExpressionTypes::Abs)
        .value("Divide", E_NonlinearExpressionTypes::Divide)
        .value("Power", E_NonlinearExpressionTypes::Power)
        .value("Sum", E_NonlinearExpressionTypes::Sum)
        .value("Product", E_NonlinearExpressionTypes::Product);
    // Note: Not using .export_values() to avoid polluting module namespace

    // ===== Nonlinear Expression Helper Functions =====
    m.def(
        "exp", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionExp>(wrapInExpression(var)); }, "Exponential function", py::arg("x"));

    m.def(
        "exp", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionExp>(expr); }, "Exponential function", py::arg("x"));

    m.def(
        "log", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionLog>(wrapInExpression(var)); }, "Natural logarithm", py::arg("x"));

    m.def(
        "log", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionLog>(expr); }, "Natural logarithm", py::arg("x"));

    m.def(
        "sqrt", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionSquareRoot>(wrapInExpression(var)); }, "Square root", py::arg("x"));

    m.def(
        "sqrt", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionSquareRoot>(expr); }, "Square root", py::arg("x"));

    m.def(
        "sin", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionSin>(wrapInExpression(var)); }, "Sine function", py::arg("x"));

    m.def(
        "sin", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionSin>(expr); }, "Sine function", py::arg("x"));

    m.def(
        "cos", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionCos>(wrapInExpression(var)); }, "Cosine function", py::arg("x"));

    m.def(
        "cos", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionCos>(expr); }, "Cosine function", py::arg("x"));

    m.def(
        "tan", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionTan>(wrapInExpression(var)); }, "Tangent function", py::arg("x"));

    m.def(
        "tan", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionTan>(expr); }, "Tangent function", py::arg("x"));

    m.def(
        "asin", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionArcSin>(wrapInExpression(var)); }, "Arc sine function", py::arg("x"));

    m.def(
        "asin", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionArcSin>(expr); }, "Arc sine function", py::arg("x"));

    m.def(
        "acos", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionArcCos>(wrapInExpression(var)); }, "Arc cosine function", py::arg("x"));

    m.def(
        "acos", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionArcCos>(expr); }, "Arc cosine function", py::arg("x"));

    m.def(
        "atan", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionArcTan>(wrapInExpression(var)); }, "Arc tangent function", py::arg("x"));

    m.def(
        "atan", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionArcTan>(expr); }, "Arc tangent function", py::arg("x"));

    m.def(
        "abs", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionAbs>(wrapInExpression(var)); }, "Absolute value", py::arg("x"));

    m.def(
        "abs", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionAbs>(expr); }, "Absolute value", py::arg("x"));

    m.def(
        "square", [](VariablePtr var) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionSquare>(wrapInExpression(var)); }, "Square function", py::arg("x"));

    m.def(
        "square", [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr
        { return std::make_shared<ExpressionSquare>(expr); }, "Square function", py::arg("x"));

    // ===== LinearTerm Class =====
    // The fields of the terms are read only, since a container caches what is calculated from them, e.g., the Hessian
    // and the convexity of QuadraticTerms, and a term does not know the containers it is in. A term is changed by
    // creating a new one
    py::class_<LinearTerm, std::shared_ptr<LinearTerm>>(m, "LinearTerm")
        .def(py::init<double, VariablePtr>(), py::arg("coefficient"), py::arg("variable"),
            "The term coefficient * variable. It cannot be changed once created")
        .def_readonly("coefficient", &LinearTerm::coefficient, "The coefficient of the term")
        .def_readonly("variable", &LinearTerm::variable, "The variable of the term")
        .def("__repr__", [](const LinearTerm& t)
            { return "<LinearTerm: " + std::to_string(t.coefficient) + "*" + t.variable->name + ">"; });

    // ===== QuadraticTerm Class =====
    py::class_<QuadraticTerm, std::shared_ptr<QuadraticTerm>>(m, "QuadraticTerm")
        .def(py::init<double, VariablePtr, VariablePtr>(), py::arg("coefficient"), py::arg("firstVariable"),
            py::arg("secondVariable"),
            "The term coefficient * firstVariable * secondVariable. It cannot be changed once created")
        .def_readonly("coefficient", &QuadraticTerm::coefficient, "The coefficient of the term")
        .def_readonly("firstVariable", &QuadraticTerm::firstVariable, "The first variable of the term")
        .def_readonly("secondVariable", &QuadraticTerm::secondVariable, "The second variable of the term")
        .def_readonly(
            "isBilinear", &QuadraticTerm::isBilinear, "Whether the term is a product of two different variables")
        .def_readonly("isSquare", &QuadraticTerm::isSquare, "Whether the term is the square of a variable")
        .def("__repr__",
            [](const QuadraticTerm& t)
            {
                return "<QuadraticTerm: " + std::to_string(t.coefficient) + "*" + t.firstVariable->name + "*"
                    + t.secondVariable->name + ">";
            });

    // ===== LinearTerms Collection =====
    addSequenceProtocol(py::class_<LinearTerms>(m, "LinearTerms"))
        .def(py::init<>())
        // Creating the whole container at once is what a problem of any size should use. The terms are taken as they
        // are here, and merged in one pass when the container is given to a constraint or an objective function
        .def(py::init<std::vector<LinearTermPtr>>(), py::arg("terms"))
        .def(py::init(
                 [](const VectorDouble& coefficients, const std::vector<VariablePtr>& variables)
                 {
                     if(coefficients.size() != variables.size())
                         throw py::value_error("The number of coefficients and the number of variables must be equal");

                     std::vector<LinearTermPtr> terms;
                     terms.reserve(coefficients.size());

                     for(size_t i = 0; i < coefficients.size(); i++)
                         terms.push_back(std::make_shared<LinearTerm>(coefficients[i], variables[i]));

                     return LinearTerms(std::move(terms));
                 }),
            py::arg("coefficients"), py::arg("variables"),
            "Create the terms from a list of coefficients and a list of variables, without a Python object per term")
        .def("add", py::overload_cast<LinearTermPtr>(&LinearTerms::add), py::arg("term"),
            "Add a single term, merging it with an existing term of the same variable. All existing terms are "
            "searched, so adding T terms one at a time is quadratic in T: build the container from a list instead")
        .def("add", py::overload_cast<const LinearTerms&>(&LinearTerms::add), py::arg("terms"),
            "Add all the terms of another container at once, which is how a large number of terms is added")
        .def(
            "append", [](LinearTerms& self, LinearTermPtr term) { self.push_back(term); }, py::arg("term"),
            "Append a term without merging it with an existing term of the same variable")
        .def(
            "extend",
            [](LinearTerms& self, const std::vector<LinearTermPtr>& terms)
            {
                self.reserve(self.size() + terms.size());
                for(auto& T : terms)
                    self.push_back(T);
            },
            py::arg("terms"), "Append the terms of a list without merging them")
        .def(
            "reserve", [](LinearTerms& self, size_t size) { self.reserve(size); }, py::arg("size"),
            "Reserve room for the given total number of terms")
        .def("size", [](LinearTerms& self) { return self.size(); }, "The number of terms");

    py::implicitly_convertible<py::list, LinearTerms>();

    // ===== QuadraticTerms Collection =====
    addSequenceProtocol(py::class_<QuadraticTerms>(m, "QuadraticTerms"))
        .def(py::init<>())
        // Creating the whole container at once is what a problem of any size should use. The terms are taken as they
        // are here, and merged in one pass when the container is given to a constraint or an objective function
        .def(py::init<std::vector<QuadraticTermPtr>>(), py::arg("terms"))
        .def(py::init(
                 [](const VectorDouble& coefficients, const std::vector<VariablePtr>& firstVariables,
                     const std::vector<VariablePtr>& secondVariables)
                 {
                     if(coefficients.size() != firstVariables.size() || coefficients.size() != secondVariables.size())
                         throw py::value_error("The number of coefficients and the numbers of variables must be equal");

                     std::vector<QuadraticTermPtr> terms;
                     terms.reserve(coefficients.size());

                     for(size_t i = 0; i < coefficients.size(); i++)
                         terms.push_back(
                             std::make_shared<QuadraticTerm>(coefficients[i], firstVariables[i], secondVariables[i]));

                     return QuadraticTerms(std::move(terms));
                 }),
            py::arg("coefficients"), py::arg("firstVariables"), py::arg("secondVariables"),
            "Create the terms from a list of coefficients and two lists of variables, without a Python object per "
            "term")
        .def("add", py::overload_cast<QuadraticTermPtr>(&QuadraticTerms::add), py::arg("term"),
            "Add a single term. The terms are not merged here, since searching all terms for every added term is "
            "quadratic in the number of terms")
        .def("add", py::overload_cast<const QuadraticTerms&>(&QuadraticTerms::add), py::arg("terms"),
            "Add all the terms of another container at once, which is how a large number of terms is added")
        .def(
            "append", [](QuadraticTerms& self, QuadraticTermPtr term) { self.push_back(term); }, py::arg("term"),
            "Append a term without merging it with an existing term of the same variables")
        .def(
            "extend",
            [](QuadraticTerms& self, const std::vector<QuadraticTermPtr>& terms)
            {
                self.reserve(self.size() + terms.size());
                for(auto& T : terms)
                    self.push_back(T);
            },
            py::arg("terms"), "Append the terms of a list without merging them")
        .def(
            "reserve", [](QuadraticTerms& self, size_t size) { self.reserve(size); }, py::arg("size"),
            "Reserve room for the given total number of terms")
        .def("size", [](QuadraticTerms& self) { return self.size(); }, "The number of terms");

    py::implicitly_convertible<py::list, QuadraticTerms>();

    // ===== SignomialElement Class =====
    py::class_<SignomialElement, std::shared_ptr<SignomialElement>>(m, "SignomialElement")
        .def(py::init<VariablePtr, double>(), py::arg("variable"), py::arg("power"),
            "The factor variable^power of a signomial term")
        .def_readonly("variable", &SignomialElement::variable, "The variable of the factor")
        .def_readonly("power", &SignomialElement::power, "The power of the factor")
        .def("__repr__",
            [](const SignomialElement& e)
            {
                if(e.power == 1.0)
                    return "<SignomialElement: " + e.variable->name + ">";
                else
                    return "<SignomialElement: " + e.variable->name + "^" + std::to_string(e.power) + ">";
            });

    // ===== SignomialElements =====
    // SignomialElements is a std::vector, which pybind11/stl.h converts to and from a Python list, e.g., in
    // SignomialTerm(coefficient, [SignomialElement(x, 2.0)]) and SignomialTerm.elements. It is therefore not bound as a
    // class: the methods of such a class would convert the object itself to a list, which calls them again

    // ===== SignomialTerm Class =====
    py::class_<SignomialTerm, std::shared_ptr<SignomialTerm>>(m, "SignomialTerm")
        .def(py::init<double, SignomialElements>(), py::arg("coefficient"), py::arg("elements"))
        .def(py::init(
                 [](double coeff, std::vector<std::pair<VariablePtr, double>>& varPowerPairs)
                 {
                     SignomialElements elements;
                     for(auto& [var, power] : varPowerPairs)
                     {
                         elements.push_back(std::make_shared<SignomialElement>(var, power));
                     }
                     return std::make_shared<SignomialTerm>(coeff, elements);
                 }),
            py::arg("coefficient"), py::arg("variablePowerPairs"),
            "Create a signomial term from coefficient and list of (variable, power) tuples")
        .def_readonly("coefficient", &SignomialTerm::coefficient, "The coefficient of the term")
        .def_readonly("elements", &SignomialTerm::elements, "The factors variable^power of the term")
        .def("__repr__",
            [](const SignomialTerm& t)
            {
                std::ostringstream oss;
                oss << "<SignomialTerm: " << t.coefficient;
                for(auto& e : t.elements)
                {
                    oss << " * " << e->variable->name;
                    if(e->power != 1.0)
                        oss << "^" << e->power;
                }
                oss << ">";
                return oss.str();
            });

    // ===== SignomialTerms Collection =====
    addSequenceProtocol(py::class_<SignomialTerms>(m, "SignomialTerms"))
        .def(py::init<>(), "A container of signomial terms. A Python list of terms can be used wherever it is expected")
        // Creating the whole container at once is what a problem of any size should use
        .def(py::init<std::vector<SignomialTermPtr>>(), py::arg("terms"),
            "A container of signomial terms. A Python list of terms can be used wherever it is expected")
        .def("add", py::overload_cast<SignomialTermPtr>(&SignomialTerms::add), py::arg("term"), "Add a single term")
        .def("add", py::overload_cast<const SignomialTerms&>(&SignomialTerms::add), py::arg("terms"),
            "Add all the terms of another container at once, which is how a large number of terms is added")
        .def(
            "append", [](SignomialTerms& self, SignomialTermPtr term) { self.push_back(term); }, py::arg("term"),
            "Append a term without merging it with an existing term of the same variables")
        .def(
            "extend",
            [](SignomialTerms& self, const std::vector<SignomialTermPtr>& terms)
            {
                self.reserve(self.size() + terms.size());
                for(auto& T : terms)
                    self.push_back(T);
            },
            py::arg("terms"), "Append the terms of a list without merging them")
        .def(
            "reserve", [](SignomialTerms& self, size_t size) { self.reserve(size); }, py::arg("size"),
            "Reserve room for the given total number of terms")
        .def("size", [](SignomialTerms& self) { return self.size(); }, "The number of terms");

    py::implicitly_convertible<py::list, SignomialTerms>();

    // ===== MonomialTerm Class =====
    // Note: MonomialTerm uses Variables (each variable has implicit power 1)
    py::class_<MonomialTerm, std::shared_ptr<MonomialTerm>>(m, "MonomialTerm")
        .def(py::init(
                 [](double coeff, std::vector<VariablePtr>& varList)
                 {
                     Variables vars;
                     for(auto& v : varList)
                     {
                         vars.push_back(v);
                     }
                     return std::make_shared<MonomialTerm>(coeff, vars);
                 }),
            py::arg("coefficient"), py::arg("variables"),
            "Create a monomial term from coefficient and list of variables")
        .def_readonly("coefficient", &MonomialTerm::coefficient, "The coefficient of the term")
        .def_property_readonly(
            "variables",
            [](const MonomialTerm& t)
            {
                std::vector<VariablePtr> result;
                for(auto& v : t.variables)
                    result.push_back(v);
                return result;
            },
            "The variables whose product the term is")
        .def_readonly(
            "isBilinear", &MonomialTerm::isBilinear, "Whether the term is a product of two different variables")
        .def_readonly("isSquare", &MonomialTerm::isSquare, "Whether the term is the square of a variable")
        .def_readonly("isBinary", &MonomialTerm::isBinary, "Whether all the variables of the term are binary")
        .def("__repr__",
            [](const MonomialTerm& t)
            {
                std::ostringstream oss;
                oss << "<MonomialTerm: " << t.coefficient;
                for(auto& v : t.variables)
                {
                    oss << " * " << v->name;
                }
                oss << ">";
                return oss.str();
            });

    // ===== MonomialTerms Collection =====
    addSequenceProtocol(py::class_<MonomialTerms>(m, "MonomialTerms"))
        .def(py::init<>(), "A container of monomial terms. A Python list of terms can be used wherever it is expected")
        // Creating the whole container at once is what a problem of any size should use
        .def(py::init<std::vector<MonomialTermPtr>>(), py::arg("terms"),
            "A container of monomial terms. A Python list of terms can be used wherever it is expected")
        .def("add", py::overload_cast<MonomialTermPtr>(&MonomialTerms::add), py::arg("term"), "Add a single term")
        .def("add", py::overload_cast<const MonomialTerms&>(&MonomialTerms::add), py::arg("terms"),
            "Add all the terms of another container at once, which is how a large number of terms is added")
        .def(
            "append", [](MonomialTerms& self, MonomialTermPtr term) { self.push_back(term); }, py::arg("term"),
            "Append a term without merging it with an existing term of the same variables")
        .def(
            "extend",
            [](MonomialTerms& self, const std::vector<MonomialTermPtr>& terms)
            {
                self.reserve(self.size() + terms.size());
                for(auto& T : terms)
                    self.push_back(T);
            },
            py::arg("terms"), "Append the terms of a list without merging them")
        .def(
            "reserve", [](MonomialTerms& self, size_t size) { self.reserve(size); }, py::arg("size"),
            "Reserve room for the given total number of terms")
        .def("size", [](MonomialTerms& self) { return self.size(); }, "The number of terms");

    py::implicitly_convertible<py::list, MonomialTerms>();

    // ===== ConstraintProperties Struct =====
    py::class_<ConstraintProperties>(m, "ConstraintProperties")
        .def_readonly("convexity", &ConstraintProperties::convexity,
            "The convexity of the constraint as far as SHOT can determine it, which finalize() calculates")
        .def_readonly("functionConvexity", &ConstraintProperties::functionConvexity,
            "The convexity of the function f(x) of the constraint L <= f(x) <= U, regardless of its bounds. An\n"
            "equality constraint with a convex function is nonconvex")
        .def_readonly("hasLinearTerms", &ConstraintProperties::hasLinearTerms, "Whether it has linear terms")
        .def_readonly("hasQuadraticTerms", &ConstraintProperties::hasQuadraticTerms, "Whether it has quadratic terms")
        .def_readonly("hasMonomialTerms", &ConstraintProperties::hasMonomialTerms, "Whether it has monomial terms")
        .def_readonly("hasSignomialTerms", &ConstraintProperties::hasSignomialTerms, "Whether it has signomial terms")
        .def_readonly("hasNonlinearExpression", &ConstraintProperties::hasNonlinearExpression,
            "Whether it has a nonlinear expression");

    // ===== NumericConstraint Base Class =====
    py::class_<NumericConstraint, std::shared_ptr<NumericConstraint>>(m, "NumericConstraint")
        // Assigned by the problem the constraint is added to, so read only
        .def_property_readonly("index", &NumericConstraint::getIndex,
            "The index of the constraint in the problem it has been added to, -1 before that")
        .def_readwrite("name", &NumericConstraint::name, "The name of the constraint")
        .def_readwrite("valueLHS", &NumericConstraint::valueLHS,
            "The lower bound L of the constraint L <= f(x) <= U, SHOT_DBL_MIN if there is none")
        .def_readwrite("valueRHS", &NumericConstraint::valueRHS,
            "The upper bound U of the constraint L <= f(x) <= U, SHOT_DBL_MAX if there is none")
        .def_readwrite("constant", &NumericConstraint::constant, "The constant term of f(x)")
        .def_readonly("properties", &NumericConstraint::properties,
            "Properties of the constraint, e.g., its convexity, which finalize() calculates")
        .def(
            "calculateFunctionValue",
            [](NumericConstraint& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                return (self.calculateFunctionValue(point));
            },
            py::arg("point"), "Calculate the value of f(x) of the constraint L <= f(x) <= U at the point")
        .def(
            "calculateNumericValue",
            [](NumericConstraint& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                return (self.calculateNumericValue(point));
            },
            py::arg("point"),
            "Calculate the value of the constraint and how much it deviates from its bounds at the point")
        .def(
            "isFulfilled",
            [](NumericConstraint& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                return (self.isFulfilled(point));
            },
            py::arg("point"), "Whether the constraint is fulfilled at the point")
        .def(
            "calculateGradient",
            [](NumericConstraint& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                auto gradient = self.calculateGradient(point, true);
                std::map<int, double> result;
                for(auto& G : gradient)
                    result[G.first->getIndex()] = G.second;
                return result;
            },
            py::arg("point"), "Calculate gradient at point, returns dict of {var_index: value}")
        .def(
            "calculateHessian",
            [](NumericConstraint& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                auto hessian = self.calculateHessian(point, true);
                std::map<std::pair<int, int>, double> result;
                for(auto& H : hessian)
                    result[std::make_pair(H.first.first->getIndex(), H.first.second->getIndex())] = H.second;
                return result;
            },
            py::arg("point"), "Calculate Hessian at point, returns dict of {(var1_index, var2_index): value}")
        .def(
            "getGradientSparsityPattern",
            [](NumericConstraint& self)
            {
                auto pattern = self.getGradientSparsityPattern();
                std::vector<int> result;
                for(auto& V : *pattern)
                    result.push_back(V->getIndex());
                return result;
            },
            "Get gradient sparsity pattern as list of variable indices")
        .def(
            "getHessianSparsityPattern",
            [](NumericConstraint& self)
            {
                auto pattern = self.getHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->getIndex(), E.second->getIndex()));
                return result;
            },
            "Get Hessian sparsity pattern as list of (var1_index, var2_index)");

    // ===== NumericConstraintValue =====
    numericConstraintValueClass
        .def_readonly("constraint", &NumericConstraintValue::constraint, "The constraint the value is for")
        .def_readonly("functionValue", &NumericConstraintValue::functionValue, "f(x)")
        .def_readonly("isFulfilledLHS", &NumericConstraintValue::isFulfilledLHS, "Whether L <= f(x)")
        .def_readonly("normalizedLHSValue", &NumericConstraintValue::normalizedLHSValue, "L - f(x)")
        .def_readonly("isFulfilledRHS", &NumericConstraintValue::isFulfilledRHS, "Whether f(x) <= U")
        .def_readonly("normalizedRHSValue", &NumericConstraintValue::normalizedRHSValue, "f(x) - U")
        .def_readonly("isFulfilled", &NumericConstraintValue::isFulfilled, "Whether L <= f(x) <= U")
        .def_readonly("error", &NumericConstraintValue::error, "max(0, L - f(x), f(x) - U)")
        .def_readonly("normalizedValue", &NumericConstraintValue::normalizedValue, "max(L - f(x), f(x) - U)");

    // ===== LinearConstraint Class =====
    py::class_<LinearConstraint, NumericConstraint, std::shared_ptr<LinearConstraint>>(m, "LinearConstraint")
        .def(py::init<std::string, double, double>(), py::arg("name"), py::arg("lhs"), py::arg("rhs"),
            "A linear constraint lhs <= f(x) <= rhs with the name, where f(x) is a sum of linear terms; use\n"
            "SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound. Comparisons such as x + 2*y <= 5 with\n"
            "Problem.addConstraint() are usually simpler")
        .def(py::init<std::string, LinearTerms, double, double>(), py::arg("name"), py::arg("linearTerms"),
            py::arg("lhs"), py::arg("rhs"),
            "A linear constraint lhs <= f(x) <= rhs with the name, where f(x) is a sum of linear terms; use\n"
            "SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound. Comparisons such as x + 2*y <= 5 with\n"
            "Problem.addConstraint() are usually simpler")
        .def_readwrite("linearTerms", &LinearConstraint::linearTerms, "The linear terms of the constraint")
        .def("add", py::overload_cast<const LinearTerms&>(&LinearConstraint::add), py::arg("terms"),
            "Add linear terms, merging a term with an existing term of the same variable")
        .def("add", py::overload_cast<LinearTermPtr>(&LinearConstraint::add), py::arg("term"),
            "Add linear terms, merging a term with an existing term of the same variable")
        .def("__repr__",
            [](LinearConstraintPtr c)
            {
                std::ostringstream oss;
                oss << *c;
                return "<LinearConstraint '" + c->name + "': " + oss.str() + ">";
            });

    // ===== QuadraticConstraint Class =====
    py::class_<QuadraticConstraint, LinearConstraint, std::shared_ptr<QuadraticConstraint>>(m, "QuadraticConstraint")
        .def(py::init<std::string, double, double>(), py::arg("name"), py::arg("lhs"), py::arg("rhs"),
            "A quadratic constraint lhs <= f(x) <= rhs with the name, where f(x) is a sum of linear and quadratic\n"
            "terms; use SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound")
        .def(py::init<std::string, LinearTerms, QuadraticTerms, double, double>(), py::arg("name"),
            py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("lhs"), py::arg("rhs"),
            "A quadratic constraint lhs <= f(x) <= rhs with the name, where f(x) is a sum of linear and quadratic\n"
            "terms; use SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound")
        .def_readwrite("quadraticTerms", &QuadraticConstraint::quadraticTerms, "The quadratic terms of the constraint")
        // Inherited add methods from LinearConstraint
        .def("add", py::overload_cast<const LinearTerms&>(&QuadraticConstraint::add), py::arg("terms"),
            "Add linear or quadratic terms")
        .def("add", py::overload_cast<LinearTermPtr>(&QuadraticConstraint::add), py::arg("term"),
            "Add linear or quadratic terms")
        // QuadraticConstraint-specific add methods
        .def("add", py::overload_cast<const QuadraticTerms&>(&QuadraticConstraint::add), py::arg("terms"),
            "Add linear or quadratic terms")
        .def("add", py::overload_cast<QuadraticTermPtr>(&QuadraticConstraint::add), py::arg("term"),
            "Add linear or quadratic terms")
        .def("__repr__",
            [](QuadraticConstraintPtr c)
            {
                std::ostringstream oss;
                oss << *c;
                return "<QuadraticConstraint '" + c->name + "': " + oss.str() + ">";
            });

    // ===== NonlinearConstraint Class =====
    py::class_<NonlinearConstraint, QuadraticConstraint, std::shared_ptr<NonlinearConstraint>>(m, "NonlinearConstraint")
        .def(py::init<std::string, double, double>(), py::arg("name"), py::arg("lhs"), py::arg("rhs"),
            "A nonlinear constraint lhs <= f(x) <= rhs with the name, where f(x) has linear and quadratic terms and\n"
            "a nonlinear expression; use SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound. finalize() extracts\n"
            "linear and quadratic terms from the expression, and may replace the constraint with one of another\n"
            "class")
        .def(py::init<std::string, NonlinearExpressionPtr, double, double>(), py::arg("name"), py::arg("expression"),
            py::arg("lhs"), py::arg("rhs"),
            "A nonlinear constraint lhs <= f(x) <= rhs with the name, where f(x) has linear and quadratic terms and\n"
            "a nonlinear expression; use SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound. finalize() extracts\n"
            "linear and quadratic terms from the expression, and may replace the constraint with one of another\n"
            "class")
        .def(py::init<std::string, LinearTerms, NonlinearExpressionPtr, double, double>(), py::arg("name"),
            py::arg("linearTerms"), py::arg("expression"), py::arg("lhs"), py::arg("rhs"),
            "A nonlinear constraint lhs <= f(x) <= rhs with the name, where f(x) has linear and quadratic terms and\n"
            "a nonlinear expression; use SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound. finalize() extracts\n"
            "linear and quadratic terms from the expression, and may replace the constraint with one of another\n"
            "class")
        .def(py::init<std::string, LinearTerms, QuadraticTerms, NonlinearExpressionPtr, double, double>(),
            py::arg("name"), py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("expression"), py::arg("lhs"),
            py::arg("rhs"),
            "A nonlinear constraint lhs <= f(x) <= rhs with the name, where f(x) has linear and quadratic terms and\n"
            "a nonlinear expression; use SHOT_DBL_MIN or SHOT_DBL_MAX for a missing bound. finalize() extracts\n"
            "linear and quadratic terms from the expression, and may replace the constraint with one of another\n"
            "class")
        .def_readwrite("nonlinearExpression", &NonlinearConstraint::nonlinearExpression,
            "The nonlinear expression of the constraint, None if it has none")
        .def_readwrite("monomialTerms", &NonlinearConstraint::monomialTerms, "The monomial terms of the constraint")
        .def_readwrite("signomialTerms", &NonlinearConstraint::signomialTerms, "The signomial terms of the constraint")
        // Inherited add methods from LinearConstraint
        .def("add", py::overload_cast<const LinearTerms&>(&NonlinearConstraint::add), py::arg("terms"))
        .def("add", py::overload_cast<LinearTermPtr>(&NonlinearConstraint::add), py::arg("term"))
        // Inherited add methods from QuadraticConstraint
        .def("add", py::overload_cast<const QuadraticTerms&>(&NonlinearConstraint::add), py::arg("terms"))
        .def("add", py::overload_cast<QuadraticTermPtr>(&NonlinearConstraint::add), py::arg("term"))
        // NonlinearConstraint-specific add methods
        .def("add", py::overload_cast<NonlinearExpressionPtr>(&NonlinearConstraint::add), py::arg("expression"))
        .def("add", py::overload_cast<const MonomialTerms&>(&NonlinearConstraint::add), py::arg("terms"))
        .def("add", py::overload_cast<MonomialTermPtr>(&NonlinearConstraint::add), py::arg("term"))
        .def("add", py::overload_cast<const SignomialTerms&>(&NonlinearConstraint::add), py::arg("terms"))
        .def("add", py::overload_cast<SignomialTermPtr>(&NonlinearConstraint::add), py::arg("term"))
        .def("__repr__",
            [](NonlinearConstraintPtr c)
            {
                std::ostringstream oss;
                oss << *c;
                return "<NonlinearConstraint '" + c->name + "': " + oss.str() + ">";
            });

    // ===== ObjectiveFunctionProperties Struct =====
    py::class_<ObjectiveFunctionProperties>(m, "ObjectiveFunctionProperties")
        .def_readonly(
            "isMinimize", &ObjectiveFunctionProperties::isMinimize, "Whether the objective function is minimized")
        .def_readonly(
            "isMaximize", &ObjectiveFunctionProperties::isMaximize, "Whether the objective function is maximized")
        .def_readonly("convexity", &ObjectiveFunctionProperties::convexity,
            "The convexity of the objective function as far as SHOT can determine it, which finalize() calculates")
        .def_readonly("hasLinearTerms", &ObjectiveFunctionProperties::hasLinearTerms, "Whether it has linear terms")
        .def_readonly(
            "hasQuadraticTerms", &ObjectiveFunctionProperties::hasQuadraticTerms, "Whether it has quadratic terms")
        .def_readonly(
            "hasMonomialTerms", &ObjectiveFunctionProperties::hasMonomialTerms, "Whether it has monomial terms")
        .def_readonly(
            "hasSignomialTerms", &ObjectiveFunctionProperties::hasSignomialTerms, "Whether it has signomial terms")
        .def_readonly("hasNonlinearExpression", &ObjectiveFunctionProperties::hasNonlinearExpression,
            "Whether it has a nonlinear expression");

    // ===== ObjectiveFunction Base Class =====
    py::class_<ObjectiveFunction, std::shared_ptr<ObjectiveFunction>>(m, "ObjectiveFunction")
        .def_readwrite(
            "direction", &ObjectiveFunction::direction, "Whether the objective function is minimized or maximized")
        .def_readwrite("constant", &ObjectiveFunction::constant, "The constant term of the objective function")
        .def_readonly("properties", &ObjectiveFunction::properties,
            "Properties of the objective function, e.g., its convexity, which finalize() calculates")
        .def(
            "calculateValue",
            [](ObjectiveFunction& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                return (self.calculateValue(point));
            },
            py::arg("point"), "Calculate the value of the objective function, including its constant, at the point")
        .def(
            "calculateGradient",
            [](ObjectiveFunction& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                auto gradient = self.calculateGradient(point, true);
                std::map<int, double> result;
                for(auto& G : gradient)
                    result[G.first->getIndex()] = G.second;
                return result;
            },
            py::arg("point"), "Calculate gradient at point, returns dict of {var_index: value}")
        .def(
            "calculateHessian",
            [](ObjectiveFunction& self, const std::vector<double>& point)
            {
                checkPointSize(self.ownerProblem, point);
                auto hessian = self.calculateHessian(point, true);
                std::map<std::pair<int, int>, double> result;
                for(auto& H : hessian)
                    result[std::make_pair(H.first.first->getIndex(), H.first.second->getIndex())] = H.second;
                return result;
            },
            py::arg("point"), "Calculate Hessian at point, returns dict of {(var1_index, var2_index): value}")
        .def(
            "getGradientSparsityPattern",
            [](ObjectiveFunction& self)
            {
                auto pattern = self.getGradientSparsityPattern();
                std::vector<int> result;
                for(auto& V : *pattern)
                    result.push_back(V->getIndex());
                return result;
            },
            "Get gradient sparsity pattern as list of variable indices")
        .def(
            "getHessianSparsityPattern",
            [](ObjectiveFunction& self)
            {
                auto pattern = self.getHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->getIndex(), E.second->getIndex()));
                return result;
            },
            "Get Hessian sparsity pattern as list of (var1_index, var2_index)");

    // ===== LinearObjectiveFunction Class =====
    py::class_<LinearObjectiveFunction, ObjectiveFunction, std::shared_ptr<LinearObjectiveFunction>>(
        m, "LinearObjectiveFunction")
        .def(py::init<E_ObjectiveFunctionDirection>(), py::arg("direction"),
            "A linear objective function, a sum of linear terms and a constant. Problem.setObjective() with an\n"
            "expression, e.g., 2*x + 3*y, is usually simpler")
        .def(py::init<E_ObjectiveFunctionDirection, double>(), py::arg("direction"), py::arg("constant"),
            "A linear objective function, a sum of linear terms and a constant. Problem.setObjective() with an\n"
            "expression, e.g., 2*x + 3*y, is usually simpler")
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, double>(), py::arg("direction"),
            py::arg("linearTerms"), py::arg("constant"),
            "A linear objective function, a sum of linear terms and a constant. Problem.setObjective() with an\n"
            "expression, e.g., 2*x + 3*y, is usually simpler")
        .def_readwrite(
            "linearTerms", &LinearObjectiveFunction::linearTerms, "The linear terms of the objective function")
        .def("add", py::overload_cast<const LinearTerms&>(&LinearObjectiveFunction::add), py::arg("terms"),
            "Add linear terms, merging a term with an existing term of the same variable")
        .def("add", py::overload_cast<LinearTermPtr>(&LinearObjectiveFunction::add), py::arg("term"),
            "Add linear terms, merging a term with an existing term of the same variable");

    // ===== QuadraticObjectiveFunction Class =====
    py::class_<QuadraticObjectiveFunction, LinearObjectiveFunction, std::shared_ptr<QuadraticObjectiveFunction>>(
        m, "QuadraticObjectiveFunction")
        .def(py::init<E_ObjectiveFunctionDirection>(), py::arg("direction"),
            "A quadratic objective function, a sum of linear and quadratic terms and a constant")
        .def(py::init<E_ObjectiveFunctionDirection, double>(), py::arg("direction"), py::arg("constant"),
            "A quadratic objective function, a sum of linear and quadratic terms and a constant")
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, QuadraticTerms, double>(), py::arg("direction"),
            py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("constant"),
            "A quadratic objective function, a sum of linear and quadratic terms and a constant")
        .def_readwrite("quadraticTerms", &QuadraticObjectiveFunction::quadraticTerms,
            "The quadratic terms of the objective function")
        // Inherited add methods from LinearObjectiveFunction
        .def("add", py::overload_cast<const LinearTerms&>(&QuadraticObjectiveFunction::add), py::arg("terms"),
            "Add linear or quadratic terms")
        .def("add", py::overload_cast<LinearTermPtr>(&QuadraticObjectiveFunction::add), py::arg("term"),
            "Add linear or quadratic terms")
        // QuadraticObjectiveFunction-specific add methods
        .def("add", py::overload_cast<const QuadraticTerms&>(&QuadraticObjectiveFunction::add), py::arg("terms"),
            "Add linear or quadratic terms")
        .def("add", py::overload_cast<QuadraticTermPtr>(&QuadraticObjectiveFunction::add), py::arg("term"),
            "Add linear or quadratic terms");

    // ===== NonlinearObjectiveFunction Class =====
    py::class_<NonlinearObjectiveFunction, QuadraticObjectiveFunction, std::shared_ptr<NonlinearObjectiveFunction>>(
        m, "NonlinearObjectiveFunction")
        .def(py::init<E_ObjectiveFunctionDirection>(), py::arg("direction"),
            "A nonlinear objective function with linear and quadratic terms, a nonlinear expression and a constant.\n"
            "finalize() extracts linear and quadratic terms from the expression, and may replace the objective\n"
            "function with one of another class")
        .def(py::init<E_ObjectiveFunctionDirection, double>(), py::arg("direction"), py::arg("constant"),
            "A nonlinear objective function with linear and quadratic terms, a nonlinear expression and a constant.\n"
            "finalize() extracts linear and quadratic terms from the expression, and may replace the objective\n"
            "function with one of another class")
        .def(py::init<E_ObjectiveFunctionDirection, NonlinearExpressionPtr, double>(), py::arg("direction"),
            py::arg("expression"), py::arg("constant"),
            "A nonlinear objective function with linear and quadratic terms, a nonlinear expression and a constant.\n"
            "finalize() extracts linear and quadratic terms from the expression, and may replace the objective\n"
            "function with one of another class")
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, NonlinearExpressionPtr, double>(),
            py::arg("direction"), py::arg("linearTerms"), py::arg("expression"), py::arg("constant"),
            "A nonlinear objective function with linear and quadratic terms, a nonlinear expression and a constant.\n"
            "finalize() extracts linear and quadratic terms from the expression, and may replace the objective\n"
            "function with one of another class")
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, QuadraticTerms, NonlinearExpressionPtr, double>(),
            py::arg("direction"), py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("expression"),
            py::arg("constant"),
            "A nonlinear objective function with linear and quadratic terms, a nonlinear expression and a constant.\n"
            "finalize() extracts linear and quadratic terms from the expression, and may replace the objective\n"
            "function with one of another class")
        .def_readwrite("nonlinearExpression", &NonlinearObjectiveFunction::nonlinearExpression,
            "The nonlinear expression of the objective function, None if it has none")
        .def_readwrite(
            "monomialTerms", &NonlinearObjectiveFunction::monomialTerms, "The monomial terms of the objective function")
        .def_readwrite("signomialTerms", &NonlinearObjectiveFunction::signomialTerms,
            "The signomial terms of the objective function")
        .def_readonly("variablesInNonlinearExpression", &NonlinearObjectiveFunction::variablesInNonlinearExpression,
            "The variables in the nonlinear expression")
        .def_readonly("nonlinearExpressionIndex", &NonlinearObjectiveFunction::nonlinearExpressionIndex,
            "The position of the nonlinear expression among those that SHOT differentiates automatically, -1 if\n"
            "it has none")
        // Inherited add methods from LinearObjectiveFunction
        .def("add", py::overload_cast<const LinearTerms&>(&NonlinearObjectiveFunction::add), py::arg("terms"))
        .def("add", py::overload_cast<LinearTermPtr>(&NonlinearObjectiveFunction::add), py::arg("term"))
        // Inherited add methods from QuadraticObjectiveFunction
        .def("add", py::overload_cast<const QuadraticTerms&>(&NonlinearObjectiveFunction::add), py::arg("terms"))
        .def("add", py::overload_cast<QuadraticTermPtr>(&NonlinearObjectiveFunction::add), py::arg("term"))
        // NonlinearObjectiveFunction-specific add methods
        .def("add", py::overload_cast<NonlinearExpressionPtr>(&NonlinearObjectiveFunction::add), py::arg("expression"))
        .def("add", py::overload_cast<const MonomialTerms&>(&NonlinearObjectiveFunction::add), py::arg("terms"))
        .def("add", py::overload_cast<MonomialTermPtr>(&NonlinearObjectiveFunction::add), py::arg("term"))
        .def("add", py::overload_cast<const SignomialTerms&>(&NonlinearObjectiveFunction::add), py::arg("terms"))
        .def("add", py::overload_cast<SignomialTermPtr>(&NonlinearObjectiveFunction::add), py::arg("term"));

    // ===== ProblemProperties Struct =====
    py::class_<ProblemProperties>(m, "ProblemProperties")
        .def_readonly(
            "isValid", &ProblemProperties::isValid, "Whether the properties are up to date; finalize() calculates them")
        .def_readonly(
            "convexity", &ProblemProperties::convexity, "The convexity of the problem as far as SHOT can determine it")
        .def_readonly("isNonlinear", &ProblemProperties::isNonlinear,
            "Whether the problem has nonlinear terms other than quadratic ones")
        .def_readonly("isDiscrete", &ProblemProperties::isDiscrete,
            "Whether the problem has binary, integer, semicontinuous or semi-integer variables, or special ordered\n"
            "sets")
        .def_readonly("isMINLPProblem", &ProblemProperties::isMINLPProblem,
            "Whether the problem is a mixed-integer problem with nonlinear terms other than quadratic ones")
        .def_readonly("isNLPProblem", &ProblemProperties::isNLPProblem,
            "Whether the problem is a continuous problem with nonlinear terms other than quadratic ones")
        .def_readonly("isMIQPProblem", &ProblemProperties::isMIQPProblem,
            "Whether the problem is a mixed-integer problem with a quadratic objective function and linear constraints")
        .def_readonly("isQPProblem", &ProblemProperties::isQPProblem,
            "Whether the problem is a continuous problem with a quadratic objective function and linear constraints")
        .def_readonly("isMIQCQPProblem", &ProblemProperties::isMIQCQPProblem,
            "Whether the problem is a mixed-integer problem with quadratic constraints and a linear or quadratic "
            "objective function")
        .def_readonly("isQCQPProblem", &ProblemProperties::isQCQPProblem,
            "Whether the problem is a continuous problem with quadratic constraints and a linear or quadratic "
            "objective function")
        .def_readonly(
            "isMILPProblem", &ProblemProperties::isMILPProblem, "Whether the problem is a mixed-integer linear problem")
        .def_readonly(
            "isLPProblem", &ProblemProperties::isLPProblem, "Whether the problem is a continuous linear problem")
        .def_readonly("numberOfVariables", &ProblemProperties::numberOfVariables, "The number of variables")
        .def_readonly(
            "numberOfRealVariables", &ProblemProperties::numberOfRealVariables, "The number of continuous variables")
        .def_readonly("numberOfDiscreteVariables", &ProblemProperties::numberOfDiscreteVariables,
            "The number of binary and integer variables")
        .def_readonly(
            "numberOfBinaryVariables", &ProblemProperties::numberOfBinaryVariables, "The number of binary variables")
        .def_readonly("numberOfIntegerVariables", &ProblemProperties::numberOfIntegerVariables,
            "The number of integer variables, not counting binary or semi-integer ones")
        .def_readonly("numberOfSemicontinuousVariables", &ProblemProperties::numberOfSemicontinuousVariables,
            "The number of semicontinuous variables")
        .def_readonly("numberOfSpecialOrderedSets", &ProblemProperties::numberOfSpecialOrderedSets,
            "The number of special ordered sets")
        .def_readonly(
            "numberOfNumericConstraints", &ProblemProperties::numberOfNumericConstraints, "The number of constraints")
        .def_readonly("numberOfLinearConstraints", &ProblemProperties::numberOfLinearConstraints,
            "The number of linear constraints")
        .def_readonly("numberOfQuadraticConstraints", &ProblemProperties::numberOfQuadraticConstraints,
            "The number of quadratic constraints")
        .def_readonly("numberOfConvexQuadraticConstraints", &ProblemProperties::numberOfConvexQuadraticConstraints,
            "The number of quadratic constraints that are convex")
        .def_readonly("numberOfNonconvexQuadraticConstraints",
            &ProblemProperties::numberOfNonconvexQuadraticConstraints,
            "The number of quadratic constraints that are not convex")
        .def_readonly("numberOfNonlinearConstraints", &ProblemProperties::numberOfNonlinearConstraints,
            "The number of nonlinear constraints")
        .def_readonly("numberOfConvexNonlinearConstraints", &ProblemProperties::numberOfConvexNonlinearConstraints,
            "The number of nonlinear constraints that are convex")
        .def_readonly("numberOfNonconvexNonlinearConstraints",
            &ProblemProperties::numberOfNonconvexNonlinearConstraints,
            "The number of nonlinear constraints that are not convex")
        .def_readonly("numberOfVariablesInNonlinearExpressions",
            &ProblemProperties::numberOfVariablesInNonlinearExpressions,
            "The number of variables in the nonlinear expressions")
        .def_readonly("numberOfNonlinearExpressions", &ProblemProperties::numberOfNonlinearExpressions,
            "The number of nonlinear expressions, including one in the objective function")
        .def_readonly("name", &ProblemProperties::name, "The name of the problem")
        .def_readonly("description", &ProblemProperties::description, "A description of the problem")
        .def_readonly("isReformulated", &ProblemProperties::isReformulated,
            "Whether this is the reformulated problem that SHOT solves");

    // ===== SpecialOrderedSet Class =====
    py::class_<SpecialOrderedSet, std::shared_ptr<SpecialOrderedSet>>(m, "SpecialOrderedSet")
        .def(py::init(
                 [](E_SOSType type, std::vector<VariablePtr> varList, VectorDouble weights)
                 {
                     Variables vars;
                     for(auto& v : varList)
                         vars.push_back(v);
                     return std::make_shared<SpecialOrderedSet>(type, vars, weights);
                 }),
            py::arg("sosType"), py::arg("variables"), py::arg("weights") = VectorDouble { },
            "A special ordered set of variables of the problem: of type One, at most one of them is nonzero, and of\n"
            "type Two, at most two consecutive ones are. The weights give the order of the variables")
        .def_readwrite("type", &SpecialOrderedSet::type, "The type of the set, SOSType.One or SOSType.Two")
        .def_property(
            "variables",
            [](const SpecialOrderedSet& s) { return std::vector<VariablePtr>(s.variables.begin(), s.variables.end()); },
            [](SpecialOrderedSet& s, std::vector<VariablePtr> varList)
            {
                s.variables.clear();
                for(auto& v : varList)
                    s.variables.push_back(v);
            },
            "The variables of the set")
        .def_readwrite("weights", &SpecialOrderedSet::weights, "The weights of the variables, which give their order");

    // ===== Problem Class =====
    // Problem uses enable_shared_from_this, pybind11 handles this automatically
    // when we specify shared_ptr as the holder type
    py::class_<Problem, std::shared_ptr<Problem>>(m, "Problem")
        .def(py::init<EnvironmentPtr>(), py::arg("environment"))
        // The problem uses the solver's environment, e.g., its settings in finalize()
        .def(py::init([](Solver& solver) { return std::make_shared<Problem>(solver.getEnvironment()); }),
            py::arg("solver"), "Create a problem in the environment of the solver")
        .def_readwrite("name", &Problem::name, "The name of the problem, used in the log and the results")
        .def_property_readonly("isFinalized", &Problem::hasBeenFinalized,
            "Whether finalize() has been called, after which nothing can be added to the problem")
        .def_readonly("properties", &Problem::properties,
            "Properties of the problem, e.g., its convexity and the numbers of variables and constraints, which\n"
            "finalize() calculates")
        .def_readonly("allVariables", &Problem::allVariables, "All the variables, in the order of their indexes")
        .def_readonly("realVariables", &Problem::realVariables, "The continuous variables")
        .def_readonly("binaryVariables", &Problem::binaryVariables, "The binary variables")
        .def_readonly("integerVariables", &Problem::integerVariables, "The integer variables")
        .def_readonly("nonlinearExpressionVariables", &Problem::nonlinearExpressionVariables,
            "The variables in the nonlinear expressions of the constraints and the objective function, which\n"
            "finalize() collects")
        .def_readonly("objectiveFunction", &Problem::objectiveFunction,
            "The objective function. finalize() may replace it with one of another class, so read it after\n"
            "finalize()")
        .def_readonly(
            "linearConstraints", &Problem::linearConstraints, "The linear constraints, which finalize() sorts out")
        .def_readonly("quadraticConstraints", &Problem::quadraticConstraints,
            "The quadratic constraints, which finalize() sorts out")
        .def_readonly("nonlinearConstraints", &Problem::nonlinearConstraints,
            "The nonlinear constraints, which finalize() sorts out")
        .def_readonly(
            "numericConstraints", &Problem::numericConstraints, "All the constraints, in the order of their indexes")
        // Add methods - using lambdas since these are separate method overloads
        .def(
            "addVariable",
            [](Problem& self, VariablePtr var)
            {
                checkCanAdd(self);
                checkNotAdded(*var, "variable '" + var->name + "'");
                self.add(var);
            },
            py::arg("variable").none(false),
            "Add a variable to the problem, which gives it its index. Raises ValueError if it has already been\n"
            "added to a problem, and RuntimeError if the problem has been finalized")
        .def(
            "addVariable",
            [](Problem& self, std::string name, E_VariableType type, std::optional<double> lowerBound,
                std::optional<double> upperBound, std::optional<double> semiBound)
            {
                checkCanAdd(self);

                bool isBinary = (type == E_VariableType::Binary);

                double lower = lowerBound ? toLowerBound(*lowerBound) : (isBinary ? 0.0 : SHOT_DBL_MIN);
                double upper = upperBound ? toUpperBound(*upperBound) : (isBinary ? 1.0 : SHOT_DBL_MAX);

                if(lower > upper)
                    throw py::value_error("The lower bound " + Utilities::toString(lower)
                        + " is larger than the upper bound " + Utilities::toString(upper) + ".");

                if(name.empty())
                    name = "variable_" + std::to_string(self.allVariables.size());

                auto variable = semiBound ? std::make_shared<Variable>(name, type, lower, upper, *semiBound)
                                          : std::make_shared<Variable>(name, type, lower, upper);
                self.add(variable);

                return (variable);
            },
            py::arg("name") = "", py::arg_v("type", E_VariableType::Real, "VariableType.Real"),
            py::arg("lowerBound") = py::none(), py::arg("upperBound") = py::none(), py::arg("semiBound") = py::none(),
            "Create a variable, add it to the problem and return it. Without bounds, a binary variable has the bounds\n"
            "0 and 1, and other variables have none; inf and -inf also mean no bound. Without a name, it is named\n"
            "variable_<index>.")
        .def(
            "addVariables",
            [](Problem& self, Variables vars)
            {
                checkCanAdd(self);
                checkNotAddedOrRepeated(vars, [](const VariablePtr& V) { return "variable '" + V->name + "'"; });
                self.add(vars);
            },
            py::arg("variables"),
            "Add all the variables of a list at once. Raises ValueError, adding none of them, if one has already\n"
            "been added to a problem or is in the list twice")
        // Order matters for pybind11 overload resolution - most specific types first
        .def(
            "addConstraint",
            [](Problem& self, NonlinearConstraintPtr c)
            {
                checkCanAdd(self);
                checkNotAdded(*c, "constraint '" + c->name + "'");
                checkVariablesInProblem(self, c);
                self.add(c);
            },
            py::arg("constraint").none(false),
            "Add a constraint created from its class. Its variables must have been added to the problem. Raises\n"
            "ValueError if it has already been added to a problem or has a variable that is not in the problem, and\n"
            "RuntimeError if the problem has been finalized")
        .def(
            "addConstraint",
            [](Problem& self, QuadraticConstraintPtr c)
            {
                checkCanAdd(self);
                checkNotAdded(*c, "constraint '" + c->name + "'");
                checkVariablesInProblem(self, c);
                self.add(c);
            },
            py::arg("constraint").none(false),
            "Add a constraint created from its class. Its variables must have been added to the problem. Raises\n"
            "ValueError if it has already been added to a problem or has a variable that is not in the problem, and\n"
            "RuntimeError if the problem has been finalized")
        .def(
            "addConstraint",
            [](Problem& self, LinearConstraintPtr c)
            {
                checkCanAdd(self);
                checkNotAdded(*c, "constraint '" + c->name + "'");
                checkVariablesInProblem(self, c);
                self.add(c);
            },
            py::arg("constraint").none(false),
            "Add a constraint created from its class. Its variables must have been added to the problem. Raises\n"
            "ValueError if it has already been added to a problem or has a variable that is not in the problem, and\n"
            "RuntimeError if the problem has been finalized")
        .def(
            "addConstraint",
            [](Problem& self, NumericConstraintPtr c)
            {
                checkCanAdd(self);
                checkNotAdded(*c, "constraint '" + c->name + "'");
                checkVariablesInProblem(self, c);
                self.add(c);
            },
            py::arg("constraint").none(false),
            "Add a constraint created from its class. Its variables must have been added to the problem. Raises\n"
            "ValueError if it has already been added to a problem or has a variable that is not in the problem, and\n"
            "RuntimeError if the problem has been finalized")
        .def(
            "addConstraint",
            [](Problem& self, const ConstraintExpression& constraint, std::string name)
            {
                checkCanAdd(self);
                addConstraintExpression(self, constraint, std::move(name));
            },
            py::arg("constraint"), py::arg("name") = "",
            "Add a constraint given by a comparison, e.g., x1 * x2 <= 5 or SHOTpy.inequality(1, x1 * x2, 5).\n"
            "Without a name, it is named constraint_<index>. The class of the constraint is decided by\n"
            "finalize(), which may replace it, so read it back from the problem afterwards, e.g., by its name.")
        // Problem::add(NumericConstraintPtr) dispatches on the properties of the constraint, so one overload takes
        // every kind. Adding a constraint is constant time, so this only saves the calls across the binding
        .def(
            "addConstraints",
            [](Problem& self, const std::vector<NumericConstraintPtr>& constraints)
            {
                checkCanAdd(self);
                checkNotAddedOrRepeated(
                    constraints, [](const NumericConstraintPtr& C) { return "constraint '" + C->name + "'"; });

                for(auto& C : constraints)
                    checkVariablesInProblem(self, C);

                for(auto& C : constraints)
                    self.add(C);
            },
            py::arg("constraints"), "Add all the constraints of a list")
        .def(
            "addConstraints",
            [](Problem& self, const std::vector<ConstraintExpression>& constraints,
                const std::vector<std::string>& names)
            {
                checkCanAdd(self);

                if(!names.empty() && names.size() != constraints.size())
                    throw py::value_error("The number of names and the number of constraints must be equal.");

                // Checked for all the constraints before any of them is added, as in the other bulk adds
                for(size_t i = 0; i < constraints.size(); i++)
                {
                    checkVariablesInProblem(self, constraints[i].expression,
                        "the constraint at position " + std::to_string(i) + " of the list");
                }

                for(size_t i = 0; i < constraints.size(); i++)
                    addConstraintExpression(self, constraints[i], names.empty() ? "" : names[i]);
            },
            py::arg("constraints"), py::arg("names") = std::vector<std::string>(),
            "Add all the constraints of a list of comparisons, e.g., [x <= 1, x + y >= 2], with the names of a\n"
            "list of the same length. Without names, they are named constraint_<index>.")
        .def(
            "addSpecialOrderedSet",
            [](Problem& self, SpecialOrderedSetPtr sos)
            {
                checkCanAdd(self);

                for(auto& V : sos->variables)
                    checkVariableInProblem(self, V, "the special ordered set");

                self.add(sos);
            },
            py::arg("sos"), "Add a special ordered set of variables of the problem")
        // Order matters for pybind11 overload resolution - most specific types first
        .def(
            "setObjective",
            [](Problem& self, NonlinearObjectiveFunctionPtr obj)
            {
                checkCanAdd(self);
                checkNotAdded(*obj, "objective function");
                checkVariablesInProblem(self, obj);
                self.add(obj);
            },
            py::arg("objective").none(false),
            "Set the objective function to one created from its class. Its variables must have been added to the\n"
            "problem. Raises ValueError if it has already been added to a problem or has a variable that is not in\n"
            "the problem, and RuntimeError if the problem has been finalized")
        .def(
            "setObjective",
            [](Problem& self, QuadraticObjectiveFunctionPtr obj)
            {
                checkCanAdd(self);
                checkNotAdded(*obj, "objective function");
                checkVariablesInProblem(self, obj);
                self.add(obj);
            },
            py::arg("objective").none(false),
            "Set the objective function to one created from its class. Its variables must have been added to the\n"
            "problem. Raises ValueError if it has already been added to a problem or has a variable that is not in\n"
            "the problem, and RuntimeError if the problem has been finalized")
        .def(
            "setObjective",
            [](Problem& self, LinearObjectiveFunctionPtr obj)
            {
                checkCanAdd(self);
                checkNotAdded(*obj, "objective function");
                checkVariablesInProblem(self, obj);
                self.add(obj);
            },
            py::arg("objective").none(false),
            "Set the objective function to one created from its class. Its variables must have been added to the\n"
            "problem. Raises ValueError if it has already been added to a problem or has a variable that is not in\n"
            "the problem, and RuntimeError if the problem has been finalized")
        .def(
            "setObjective",
            [](Problem& self, ObjectiveFunctionPtr obj)
            {
                checkCanAdd(self);
                checkNotAdded(*obj, "objective function");
                checkVariablesInProblem(self, obj);
                self.add(obj);
            },
            py::arg("objective").none(false),
            "Set the objective function to one created from its class. Its variables must have been added to the\n"
            "problem. Raises ValueError if it has already been added to a problem or has a variable that is not in\n"
            "the problem, and RuntimeError if the problem has been finalized")
        .def(
            "setObjective",
            [](Problem& self, NonlinearExpressionPtr expression, E_ObjectiveFunctionDirection direction)
            {
                checkCanAdd(self);
                checkVariablesInProblem(self, expression, "the objective function");
                self.add(std::make_shared<NonlinearObjectiveFunction>(direction, expression, 0.0));
            },
            py::arg("expression").none(false),
            py::arg_v("direction", E_ObjectiveFunctionDirection::Minimize, "ObjectiveDirection.Minimize"),
            "Set the objective function to an expression, e.g., SHOTpy.exp(x) + x * y. The class of the objective\n"
            "function is decided by finalize(), which may replace it, so read it back from problem.objectiveFunction\n"
            "afterwards.")
        .def(
            "setObjective",
            [](Problem& self, VariablePtr variable, E_ObjectiveFunctionDirection direction)
            {
                checkCanAdd(self);
                checkVariableInProblem(self, variable, "the objective function");
                self.add(std::make_shared<NonlinearObjectiveFunction>(direction, wrapInExpression(variable), 0.0));
            },
            py::arg("variable").none(false),
            py::arg_v("direction", E_ObjectiveFunctionDirection::Minimize, "ObjectiveDirection.Minimize"),
            "Set the objective function to a variable")
        .def(
            "setObjective",
            [](Problem& self, double constant, E_ObjectiveFunctionDirection direction)
            {
                checkCanAdd(self);

                if(!std::isfinite(constant))
                    throw py::value_error("The objective function must be finite.");

                self.add(std::make_shared<LinearObjectiveFunction>(direction, constant));
            },
            py::arg("constant"),
            py::arg_v("direction", E_ObjectiveFunctionDirection::Minimize, "ObjectiveDirection.Minimize"),
            "Set the objective function to a constant")
        // Finalize: simplify expressions, extract terms (linear, quadratic, monomial, signomial),
        // update properties, and prepare factorable functions
        .def(
            "finalize",
            [](Problem& self)
            {
                if(!self.hasBeenFinalized())
                    checkVariablesInProblem(self);

                self.finalize();
            },
            "Finalize the problem: extract terms from expressions, update properties, and prepare for solving.\n"
            "Raises ValueError if a constraint or the objective function has a variable that has not been added to\n"
            "the problem.")
        .def("updateProperties", &Problem::updateProperties, "Update problem properties")
        // Getters
        .def("getVariable", &Problem::getVariable, py::arg("index"), "The variable with the index")
        .def(
            "getConstraint", [](Problem& self, int index)
            { return std::dynamic_pointer_cast<NumericConstraint>(self.getConstraint(index)); }, py::arg("index"),
            "Get constraint by index")
        .def(
            "getConstraint",
            [](Problem& self, const std::string& name)
            {
                NumericConstraintPtr found;

                for(auto& C : self.numericConstraints)
                {
                    if(C->name != name)
                        continue;

                    if(found)
                        throw py::value_error("The problem has several constraints named " + name + ".");

                    found = C;
                }

                if(!found)
                    throw py::key_error("The problem has no constraint named " + name + ".");

                return (found);
            },
            py::arg("name"),
            "Get the constraint with the name. Raises KeyError if there is none, and ValueError if several\n"
            "constraints have the name. finalize() can replace a constraint with one of another class, so a\n"
            "constraint is read back with this after finalize()")
        .def("getVariableLowerBound", &Problem::getVariableLowerBound, py::arg("index"),
            "The lower bound of the variable with the index")
        .def("getVariableUpperBound", &Problem::getVariableUpperBound, py::arg("index"),
            "The upper bound of the variable with the index")
        .def("getVariableLowerBounds", &Problem::getVariableLowerBounds,
            "The lower bounds of all the variables, in the order of their indexes")
        .def("getVariableUpperBounds", &Problem::getVariableUpperBounds,
            "The upper bounds of all the variables, in the order of their indexes")
        .def(
            "getMostDeviatingNumericConstraint",
            [](Problem& self, const std::vector<double>& point)
            {
                checkPointSize(self, point);
                return (self.getMostDeviatingNumericConstraint(point));
            },
            py::arg("point"),
            "The value of the constraint that deviates most from its bounds at the point,\n"
            "or None if all constraints are fulfilled")
        // Sparsity patterns
        .def(
            "getConstraintsJacobianSparsityPattern",
            [](Problem& self)
            {
                auto pattern = self.getConstraintsJacobianSparsityPattern();
                std::vector<std::pair<int, std::vector<int>>> result;
                for(auto& E : *pattern)
                {
                    std::vector<int> varIndices;
                    for(auto& V : E.second)
                        varIndices.push_back(V->getIndex());
                    result.push_back(std::make_pair(E.first->getIndex(), varIndices));
                }
                return result;
            },
            "Get Jacobian sparsity pattern as list of (constraint_index, [variable_indices])")
        .def(
            "getConstraintsHessianSparsityPattern",
            [](Problem& self)
            {
                auto pattern = self.getConstraintsHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->getIndex(), E.second->getIndex()));
                return result;
            },
            "Get Hessian sparsity pattern for constraints only as list of (var1_index, var2_index)")
        .def(
            "getLagrangianHessianSparsityPattern",
            [](Problem& self)
            {
                auto pattern = self.getLagrangianHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->getIndex(), E.second->getIndex()));
                return result;
            },
            "Get Hessian sparsity pattern including objective as list of (var1_index, var2_index)")
        // String representation
        .def("__repr__",
            [](ProblemPtr p)
            {
                std::string repr = "<Problem";
                if(!p->name.empty())
                    repr += " '" + p->name + "'";
                repr += " vars=" + std::to_string(p->properties.numberOfVariables);
                repr += " constrs=" + std::to_string(p->properties.numberOfNumericConstraints);
                repr += ">";
                return repr;
            })
        .def("__str__",
            [](ProblemPtr p)
            {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
        .def(
            "toString",
            [](ProblemPtr p)
            {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            },
            "Get the full string representation of the problem");

    // ===== Variables Collection =====
    addSequenceProtocol(variablesClass)
        .def(py::init<>(),
            "A container of variables, e.g., for Problem.addVariables(). A Python list of variables can be used\n"
            "wherever it is expected")
        // Without this, the container could not be filled from Python at all, which left Problem.addVariables
        // unreachable. A plain list is converted to it, since Variables inherits std::vector privately
        .def(py::init<std::vector<VariablePtr>>(), py::arg("variables"),
            "A container of variables, e.g., for Problem.addVariables(). A Python list of variables can be used\n"
            "wherever it is expected")
        .def(
            "append", [](Variables& self, VariablePtr variable) { self.push_back(variable); }, py::arg("variable"),
            "Append a variable")
        .def(
            "extend",
            [](Variables& self, const std::vector<VariablePtr>& variables)
            {
                self.reserve(self.size() + variables.size());
                for(auto& V : variables)
                    self.push_back(V);
            },
            py::arg("variables"), "Append the variables of a list")
        .def(
            "reserve", [](Variables& self, size_t size) { self.reserve(size); }, py::arg("size"),
            "Reserve room for the given total number of variables")
        .def("size", [](Variables& self) { return self.size(); }, "The number of variables");

    py::implicitly_convertible<py::list, Variables>();

    // ===== Environment Class =====
    environmentClass.def_readonly("problem", &Environment::problem, "The problem given to the solver")
        .def_readonly("reformulatedProblem", &Environment::reformulatedProblem,
            "The problem that SHOT solves, created from the given one by the reformulation");

    // ===== Solver Class =====
    solverClass
        .def(py::init(),
            "Create a solver with the default settings. A problem is created for it with Problem(solver), or read\n"
            "from a file with setProblem(filename)")
        .def("getEnvironment", &Solver::getEnvironment,
            "The environment of the solver, which holds its settings, results and problems. A problem is created\n"
            "in it with Problem(solver)")
        .def("getOriginalProblem", &Solver::getOriginalProblem,
            "The problem given to setProblem(), or None before it has been set")
        .def("getReformulatedProblem", &Solver::getReformulatedProblem,
            "The problem that SHOT solves, created from the original problem by setProblem(), e.g., with\n"
            "auxiliary variables for nonlinear terms, or None before the problem has been set")
        .def(
            "getAbsoluteObjectiveGap", [](Solver& self)
            { return (toResult(self.getAbsoluteObjectiveGap(), std::numeric_limits<double>::infinity())); },
            "The absolute difference between the primal bound and the global dual bound, inf if either is missing")
        .def(
            "getCurrentDualBound",
            [](Solver& self)
            {
                return (toResult(self.getCurrentDualBound(),
                    isMinimization(self) ? -std::numeric_limits<double>::infinity()
                                         : std::numeric_limits<double>::infinity()));
            },
            "The dual bound of the current dual problem, -inf (inf when maximizing) if there is none. For a\n"
            "nonconvex problem, it is not a valid bound for the problem once cuts have been added to nonconvex\n"
            "functions; use getGlobalDualBound() for a valid bound")
        .def(
            "getGlobalDualBound",
            [](Solver& self)
            {
                return (toResult(self.getGlobalDualBound(),
                    isMinimization(self) ? -std::numeric_limits<double>::infinity()
                                         : std::numeric_limits<double>::infinity()));
            },
            "The best dual bound that is valid for the problem, also when it is nonconvex, -inf (inf when\n"
            "maximizing) if there is none. The objective gaps are calculated from it")
        .def("getModelReturnStatus", &Solver::getModelReturnStatus,
            "The status of the solution, e.g., ModelReturnStatus.OptimalGlobal when the solution is proven\n"
            "optimal, or FeasibleSolution when a solution has been found without proving it optimal")
        .def("getOptions", &Solver::getOptions,
            "The settings that differ from their defaults, in the format of an options file")
        .def("getOptionsOSoL", &Solver::getOptionsOSoL,
            "The settings that differ from their defaults, in the OSoL format")
        .def(
            "getPrimalBound",
            [](Solver& self)
            {
                return (toResult(self.getPrimalBound(),
                    isMinimization(self) ? std::numeric_limits<double>::infinity()
                                         : -std::numeric_limits<double>::infinity()));
            },
            "The objective value of the best solution found, inf (-inf when maximizing) if none has been found")
        .def("getPrimalSolution", &Solver::getPrimalSolution,
            "The best solution found. Raises an exception if none has been found; check hasPrimalSolution() first")
        .def("getPrimalSolutions", &Solver::getPrimalSolutions, "All the solutions found, the best first")
        .def(
            "getRelativeObjectiveGap", [](Solver& self)
            { return (toResult(self.getRelativeObjectiveGap(), std::numeric_limits<double>::infinity())); },
            "The relative difference between the primal bound and the global dual bound, inf if either is missing")
        .def("getResultsOSrL", &Solver::getResultsOSrL, "The results in the OSrL format")
        .def("getResultsSol", &Solver::getResultsSol, "The results in the AMPL .sol format")
        .def("getResultsTrace", &Solver::getResultsTrace, "The results as a line in the GAMS trace file format")

        .def("getSolutionStatistics", &Solver::getSolutionStatistics,
            "Statistics of the solution process, e.g., the number of iterations and of solved subproblems")
        .def("getSettingsAsMarkup", &Solver::getSettingsAsMarkup,
            "All the settings, with descriptions, valid values and defaults, as Markdown")

        .def("getBoolSetting", &Solver::getSetting<bool>, py::arg("name"),
            "The value of a boolean setting, e.g., 'Model.Convexity.AssumeConvex'. Raises RuntimeError if there\n"
            "is no boolean setting with the name")
        .def("getStringSetting", &Solver::getSetting<std::string>, py::arg("name"),
            "The value of a string setting. Raises RuntimeError if there is no string setting with the name")
        .def("getIntSetting", &Solver::getSetting<int>, py::arg("name"),
            "The value of an integer or enum setting, e.g., 'Dual.MIP.Solver'. Raises RuntimeError if there is no\n"
            "such setting with the name")
        .def("getDoubleSetting", &Solver::getSetting<double>, py::arg("name"),
            "The value of a floating point setting, e.g., 'Termination.TimeLimit'. Raises RuntimeError if there\n"
            "is no floating point setting with the name")

        .def("getTerminationReason", &Solver::getTerminationReason,
            "Why SHOT terminated, e.g., TerminationReason.RelativeGap when the objective gap was closed")
        .def("hasPrimalSolution", &Solver::hasPrimalSolution,
            "Whether the problem has been solved and a solution has been found")

        .def("outputSolverHeader", &Solver::outputSolverHeader,
            "Write the header of SHOT, with its version and the solvers it uses, to the log")
        .def("outputOptionsReport", &Solver::outputOptionsReport,
            "Write the settings that differ from their defaults to the log")
        .def("outputProblemInstanceReport", &Solver::outputProblemInstanceReport,
            "Write the properties of the problem to the log")
        .def("outputSolutionReport", &Solver::outputSolutionReport,
            "Write the solution report, with the bounds, the status and the statistics, to the log")

        .def("setLogFile", &Solver::setLogFile, py::arg("filename"), "Also write the log to a file")
        .def("setOptionsFromFile", &Solver::setOptionsFromFile, py::arg("filename"),
            "Read settings from an options file (.opt) or an OSoL file (.osol or .xml); returns False if it fails")
        .def("setOptionsFromOSoL", &Solver::setOptionsFromOSoL, py::arg("osol"),
            "Read settings from a string in the OSoL format; returns False if it fails")
        .def("setOptionsFromString", &Solver::setOptionsFromString, py::arg("options"),
            "Read settings from a string in the format of an options file, e.g., 'Termination.TimeLimit = 10';\n"
            "returns False if it fails")
        .def("setProblem", py::overload_cast<std::string>(&Solver::setProblem), "Load problem from file",
            py::arg("filename"))
        .def(
            "setProblem", [](Solver& self, ProblemPtr problem) { return self.setProblem(problem, nullptr, nullptr); },
            "Set problem from Problem object", py::arg("problem"))
        .def(
            "setProblem", [](Solver& self, ProblemPtr problem, ProblemPtr reformulatedProblem)
            { return self.setProblem(problem, reformulatedProblem, nullptr); }, "Set problem with reformulated problem",
            py::arg("problem"), py::arg("reformulatedProblem"))
        // The GIL is released while the problem is solved, so that the MIP solver can call Python callbacks from its
        // own threads
        .def("solveProblem", &Solver::solveProblem, py::call_guard<py::gil_scoped_release>(),
            "Solve the problem given to setProblem(). Returns False if it could not be solved, e.g., if no problem\n"
            "has been set; the outcome is given by getModelReturnStatus() and getTerminationReason(). An exception\n"
            "raised in a callback is raised again here")
        .def("updateLogLevels", &Solver::updateLogLevels,
            "Apply the settings Output.Console.LogLevel and Output.File.LogLevel to the log")
        .def("updateSetting", py::overload_cast<std::string, bool>(&Solver::updateSetting), py::arg("name"),
            py::arg("value"),
            "Change a setting, e.g., updateSetting('Termination.TimeLimit', 10.0). Raises RuntimeError if there is\n"
            "no setting with the name, if the value is of the wrong type, or if it is outside the valid values; an\n"
            "integer is accepted for a floating point setting. Change settings before setProblem(), since most of\n"
            "them are fixed by it")
        .def("updateSetting", py::overload_cast<std::string, int>(&Solver::updateSetting), py::arg("name"),
            py::arg("value"),
            "Change a setting, e.g., updateSetting('Termination.TimeLimit', 10.0). Raises RuntimeError if there is\n"
            "no setting with the name, if the value is of the wrong type, or if it is outside the valid values; an\n"
            "integer is accepted for a floating point setting. Change settings before setProblem(), since most of\n"
            "them are fixed by it")
        .def("updateSetting", py::overload_cast<std::string, std::string>(&Solver::updateSetting), py::arg("name"),
            py::arg("value"),
            "Change a setting, e.g., updateSetting('Termination.TimeLimit', 10.0). Raises RuntimeError if there is\n"
            "no setting with the name, if the value is of the wrong type, or if it is outside the valid values; an\n"
            "integer is accepted for a floating point setting. Change settings before setProblem(), since most of\n"
            "them are fixed by it")
        .def("updateSetting", py::overload_cast<std::string, double>(&Solver::updateSetting), py::arg("name"),
            py::arg("value"),
            "Change a setting, e.g., updateSetting('Termination.TimeLimit', 10.0). Raises RuntimeError if there is\n"
            "no setting with the name, if the value is of the wrong type, or if it is outside the valid values; an\n"
            "integer is accepted for a floating point setting. Change settings before setProblem(), since most of\n"
            "them are fixed by it")
        .def(
            "registerCallback",
            // The types only give the signature; toCallbackLocations() checks the locations
            [](Solver& self,
                py::typing::Union<E_CallbackLocation, int, py::typing::Iterable<E_CallbackLocation>> locations,
                py::typing::Callable<void(std::shared_ptr<CallbackContext>)> callback)
            {
                auto mask = toCallbackLocations(locations);

                // The Python function is released with the GIL held, since the callback can be removed or the solver
                // destroyed on a thread that does not hold it
                auto function = std::shared_ptr<py::function>(new py::function(std::move(callback)),
                    [](py::function* F)
                    {
                        if(!Py_IsInitialized())
                            return;

                        py::gil_scoped_acquire gil;
                        delete F;
                    });

                return (self.registerCallback(mask,
                    [function](CallbackContext& context)
                    {
                        py::gil_scoped_acquire gil;
                        (*function)(context.shared_from_this());
                    }));
            },
            "Register a callback for one or several locations in the solution process.\n\n"
            "The locations are given as a CallbackLocation, several combined with |, or a list of them.\n"
            "The callback is called as fn(context), where the context is of the class for the location,\n"
            "e.g., a PrimalCandidateCheckContext at CallbackLocation.PrimalCandidateCheck, and has the\n"
            "state of the solver and the actions available at the location. The context can only be used\n"
            "while the callback runs. If the callback raises an exception, SHOT terminates and\n"
            "solveProblem() raises it.\n\n"
            "Returns a handle for removeCallback().",
            py::arg("locations"), py::arg("callback"))
        .def("removeCallback", &Solver::removeCallback,
            "Remove a registered callback; returns False if there is no callback with the handle.", py::arg("handle"));

    // The operators combine the locations into an integer mask, which registerCallback() accepts; pybind11 only
    // defines them for enums that convert implicitly to integers, which a scoped enum does not
    callbackLocation.value("InteriorPointSearch", E_CallbackLocation::InteriorPointSearch)
        .value("DualBoundUpdate", E_CallbackLocation::DualBoundUpdate)
        .value("PrimalCandidateSearch", E_CallbackLocation::PrimalCandidateSearch)
        .value("PrimalCandidateCheck", E_CallbackLocation::PrimalCandidateCheck)
        .value("NewPrimalSolution", E_CallbackLocation::NewPrimalSolution)
        .value("TerminationCheck", E_CallbackLocation::TerminationCheck)
        .value("HyperplaneSelection", E_CallbackLocation::HyperplaneSelection)
        .def(
            "__or__", [](E_CallbackLocation first, E_CallbackLocation second)
            { return (static_cast<std::uint64_t>(first) | static_cast<std::uint64_t>(second)); }, py::is_operator())
        .def(
            "__or__", [](E_CallbackLocation first, std::uint64_t second)
            { return (static_cast<std::uint64_t>(first) | second); }, py::is_operator())
        .def(
            "__ror__", [](E_CallbackLocation first, std::uint64_t second)
            { return (static_cast<std::uint64_t>(first) | second); }, py::is_operator())
        .def(
            "__and__", [](E_CallbackLocation first, E_CallbackLocation second)
            { return (static_cast<std::uint64_t>(first) & static_cast<std::uint64_t>(second)); }, py::is_operator())
        .def(
            "__and__", [](E_CallbackLocation first, std::uint64_t second)
            { return (static_cast<std::uint64_t>(first) & second); }, py::is_operator())
        .def(
            "__rand__", [](E_CallbackLocation first, std::uint64_t second)
            { return (static_cast<std::uint64_t>(first) & second); }, py::is_operator());

    py::enum_<E_HyperplaneSource>(m, "HyperplaneSource", py::arithmetic())
        .value("Unknown", E_HyperplaneSource::Unknown)
        .value("MIPOptimalRootsearch", E_HyperplaneSource::MIPOptimalRootsearch)
        .value("MIPSolutionPoolRootsearch", E_HyperplaneSource::MIPSolutionPoolRootsearch)
        .value("LPRelaxedRootsearch", E_HyperplaneSource::LPRelaxedRootsearch)
        .value("MIPOptimalSolutionPoint", E_HyperplaneSource::MIPOptimalSolutionPoint)
        .value("MIPSolutionPoolSolutionPoint", E_HyperplaneSource::MIPSolutionPoolSolutionPoint)
        .value("LPRelaxedSolutionPoint", E_HyperplaneSource::LPRelaxedSolutionPoint)
        .value("LPFixedIntegers", E_HyperplaneSource::LPFixedIntegers)
        .value("PrimalSolutionSearch", E_HyperplaneSource::PrimalSolutionSearch)
        .value("PrimalSolutionSearchInteriorObjective", E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
        .value("InteriorPointSearch", E_HyperplaneSource::InteriorPointSearch)
        .value("MIPCallbackRelaxed", E_HyperplaneSource::MIPCallbackRelaxed)
        .value("ObjectiveRootsearch", E_HyperplaneSource::ObjectiveRootsearch)
        .value("ObjectiveCuttingPlane", E_HyperplaneSource::ObjectiveCuttingPlane)
        .value("External", E_HyperplaneSource::External);

    py::enum_<E_PrimalSolutionSource>(m, "PrimalSolutionSource", py::arithmetic())
        .value("Rootsearch", E_PrimalSolutionSource::Rootsearch)
        .value("RootsearchFixedIntegers", E_PrimalSolutionSource::RootsearchFixedIntegers)
        .value("NLPFixedIntegers", E_PrimalSolutionSource::NLPFixedIntegers)
        .value("NLPRelaxed", E_PrimalSolutionSource::NLPRelaxed)
        .value("MIPSolutionPool", E_PrimalSolutionSource::MIPSolutionPool)
        .value("LPFixedIntegers", E_PrimalSolutionSource::LPFixedIntegers)
        .value("MIPCallback", E_PrimalSolutionSource::MIPCallback)
        .value("InteriorPointSearch", E_PrimalSolutionSource::InteriorPointSearch)
        .value("ConvexBounding", E_PrimalSolutionSource::ConvexBounding)
        .value("ExternalPrimalSolution", E_PrimalSolutionSource::ExternalPrimalSolution);

    modelReturnStatusEnum.value("NotSet", E_ModelReturnStatus::NotSet)
        .value("OptimalGlobal", E_ModelReturnStatus::OptimalGlobal)
        .value("Unbounded", E_ModelReturnStatus::Unbounded)
        .value("UnboundedNoSolution", E_ModelReturnStatus::UnboundedNoSolution)
        .value("InfeasibleGlobal", E_ModelReturnStatus::InfeasibleGlobal)
        .value("InfeasibleLocal", E_ModelReturnStatus::InfeasibleLocal)
        .value("FeasibleSolution", E_ModelReturnStatus::FeasibleSolution)
        .value("NoSolutionReturned", E_ModelReturnStatus::NoSolutionReturned)
        .value("ErrorUnknown", E_ModelReturnStatus::ErrorUnknown)
        .value("ErrorNoSolution", E_ModelReturnStatus::ErrorNoSolution);

    terminationReasonEnum.value("ConstraintTolerance", E_TerminationReason::ConstraintTolerance)
        .value("ObjectiveStagnation", E_TerminationReason::ObjectiveStagnation)
        .value("IterationLimit", E_TerminationReason::IterationLimit)
        .value("TimeLimit", E_TerminationReason::TimeLimit)
        .value("InfeasibleProblem", E_TerminationReason::InfeasibleProblem)
        .value("UnboundedProblem", E_TerminationReason::UnboundedProblem)
        .value("Error", E_TerminationReason::Error)
        .value("AbsoluteGap", E_TerminationReason::AbsoluteGap)
        .value("RelativeGap", E_TerminationReason::RelativeGap)
        .value("UserAbort", E_TerminationReason::UserAbort)
        .value("NoDualCutsAdded", E_TerminationReason::NoDualCutsAdded)
        .value("NotTerminated", E_TerminationReason::NotTerminated)
        .value("NumericIssues", E_TerminationReason::NumericIssues);

    py::class_<PairIndexValue>(m, "PairIndexValue")
        .def_readwrite("index", &PairIndexValue::index, "The index, e.g., of a constraint")
        .def_readwrite("value", &PairIndexValue::value, "The value for the index");

    primalSolutionClass
        .def_readwrite("point", &PrimalSolution::point, "The values of the variables, in the order of their indexes")
        .def_readwrite("sourceType", &PrimalSolution::sourceType,
            "Where the solution comes from, e.g., PrimalSolutionSource.NLPFixedIntegers")
        .def_readwrite(
            "sourceDescription", &PrimalSolution::sourceDescription, "A description of where the solution comes from")
        .def_readwrite("objValue", &PrimalSolution::objValue, "The objective value of the solution")
        .def_readwrite("iterFound", &PrimalSolution::iterFound, "The iteration in which the solution was found")
        .def_readwrite("maxDeviatingConstraintLinear", &PrimalSolution::maxDevatingConstraintLinear,
            "The index of the linear constraint the solution violates the most and the violation, index -1 if\n"
            "there is none")
        .def_readwrite("maxDevatingConstraintLinear", &PrimalSolution::maxDevatingConstraintLinear,
            "The same as maxDeviatingConstraintLinear; the misspelled name is kept for existing code")
        .def_readwrite("maxDeviatingConstraintQuadratic", &PrimalSolution::maxDevatingConstraintQuadratic,
            "The index of the quadratic constraint the solution violates the most and the violation, index -1 if\n"
            "there is none")
        .def_readwrite("maxDevatingConstraintQuadratic", &PrimalSolution::maxDevatingConstraintQuadratic,
            "The same as maxDeviatingConstraintQuadratic; the misspelled name is kept for existing code")
        .def_readwrite("maxDeviatingConstraintNonlinear", &PrimalSolution::maxDevatingConstraintNonlinear,
            "The index of the nonlinear constraint the solution violates the most and the violation, index -1 if\n"
            "there is none")
        .def_readwrite("maxDevatingConstraintNonlinear", &PrimalSolution::maxDevatingConstraintNonlinear,
            "The same as maxDeviatingConstraintNonlinear; the misspelled name is kept for existing code")
        .def_readwrite("maxIntegerToleranceError", &PrimalSolution::maxIntegerToleranceError,
            "The largest distance of an integer variable from an integer value before rounding")
        .def_readwrite("boundProjectionPerformed", &PrimalSolution::boundProjectionPerformed,
            "Whether values outside the variable bounds were moved to the bounds")
        .def_readwrite("integerRoundingPerformed", &PrimalSolution::integerRoundingPerformed,
            "Whether the values of integer variables were rounded")
        .def_readwrite("displayed", &PrimalSolution::displayed, "Whether the solution has been shown in the log");

    solutionStatisticsClass
        .def_readwrite("numberOfIterations", &SolutionStatistics::numberOfIterations, "The number of main iterations")
        .def_readwrite("numberOfProblemsLP", &SolutionStatistics::numberOfProblemsLP,
            "The number of LP problems solved as dual problems")
        .def_readwrite("numberOfProblemsQP", &SolutionStatistics::numberOfProblemsQP,
            "The number of QP problems solved as dual problems")
        .def_readwrite("numberOfProblemsQCQP", &SolutionStatistics::numberOfProblemsQCQP,
            "The number of QCQP problems solved as dual problems")
        .def_readwrite("numberOfProblemsFeasibleMILP", &SolutionStatistics::numberOfProblemsFeasibleMILP,
            "The number of MILP problems solved until a feasible solution was found, e.g., with a solution limit")
        .def_readwrite("numberOfProblemsOptimalMILP", &SolutionStatistics::numberOfProblemsOptimalMILP,
            "The number of MILP problems solved to optimality")
        .def_readwrite("numberOfProblemsFeasibleMIQP", &SolutionStatistics::numberOfProblemsFeasibleMIQP,
            "The number of MIQP problems solved until a feasible solution was found")
        .def_readwrite("numberOfProblemsOptimalMIQP", &SolutionStatistics::numberOfProblemsOptimalMIQP,
            "The number of MIQP problems solved to optimality")
        .def_readwrite("numberOfProblemsFeasibleMIQCQP", &SolutionStatistics::numberOfProblemsFeasibleMIQCQP,
            "The number of MIQCQP problems solved until a feasible solution was found")
        .def_readwrite("numberOfProblemsOptimalMIQCQP", &SolutionStatistics::numberOfProblemsOptimalMIQCQP,
            "The number of MIQCQP problems solved to optimality")
        .def_readwrite("numberOfFunctionEvaluations", &SolutionStatistics::numberOfFunctionEvalutions,
            "The number of evaluations of nonlinear functions")
        .def_readwrite("numberOfFunctionEvalutions", &SolutionStatistics::numberOfFunctionEvalutions,
            "The same as numberOfFunctionEvaluations; the misspelled name is kept for existing code")
        .def_readwrite("numberOfGradientEvaluations", &SolutionStatistics::numberOfGradientEvaluations,
            "The number of evaluations of gradients of nonlinear functions")
        .def_readwrite("numberOfProblemsMinimaxLP", &SolutionStatistics::numberOfProblemsMinimaxLP,
            "The number of LP problems solved in the search for an interior point")
        .def_readwrite("numberOfProblemsFixedNLP", &SolutionStatistics::numberOfProblemsFixedNLP,
            "The number of NLP problems with fixed integer variables solved to find primal solutions")
        .def_readwrite("hasFixedIntegerEnumerationBeenRun", &SolutionStatistics::hasFixedIntegerEnumerationBeenRun,
            "Whether NLP problems have been solved for all combinations of the discrete variables")
        .def_readwrite("hasFixedIntegerEnumerationFallbackBeenRun",
            &SolutionStatistics::hasFixedIntegerEnumerationFallbackBeenRun,
            "Whether this has been done as a fallback when the objective gap could not be closed")
        .def_readwrite("numberOfFixedIntegerEnumerationCombinations",
            &SolutionStatistics::numberOfFixedIntegerEnumerationCombinations,
            "The number of combinations of the discrete variables in the exhaustive search")
        .def_readwrite("numberOfFixedIntegerEnumerationCombinationsFeasible",
            &SolutionStatistics::numberOfFixedIntegerEnumerationCombinationsFeasible,
            "The number of combinations whose NLP problem gave a solution")
        .def_readwrite("numberOfFixedIntegerEnumerationCombinationsInfeasible",
            &SolutionStatistics::numberOfFixedIntegerEnumerationCombinationsInfeasible,
            "The number of combinations whose NLP problem was infeasible")
        .def_readwrite("numberOfFixedIntegerEnumerationCombinationsUnresolved",
            &SolutionStatistics::numberOfFixedIntegerEnumerationCombinationsUnresolved,
            "The number of combinations whose NLP problem ended at a limit or with an error")
        .def_readwrite("numberOfFixedIntegerEnumerationCombinationsSkipped",
            &SolutionStatistics::numberOfFixedIntegerEnumerationCombinationsSkipped,
            "The number of combinations not solved since they had been used in the fixed-integer strategy")
        .def_readwrite("numberOfHyperplanesWithConvexSource", &SolutionStatistics::numberOfHyperplanesWithConvexSource,
            "The number of cuts generated for convex functions")
        .def_readwrite("numberOfHyperplanesWithNonconvexSource",
            &SolutionStatistics::numberOfHyperplanesWithNonconvexSource,
            "The number of cuts generated for nonconvex functions, which are not valid for the whole problem")
        .def_readwrite(
            "numberOfIntegerCuts", &SolutionStatistics::numberOfIntegerCuts, "The number of integer cuts added")
        .def_readwrite("numberOfIterationsWithDualStagnation",
            &SolutionStatistics::numberOfIterationsWithDualStagnation,
            "The number of iterations since the dual bound last improved significantly")
        .def_readwrite("lastIterationWithSignificantDualUpdate",
            &SolutionStatistics::lastIterationWithSignificantDualUpdate,
            "The last iteration in which the dual bound improved significantly")
        .def_readwrite("numberOfIterationsWithPrimalStagnation",
            &SolutionStatistics::numberOfIterationsWithPrimalStagnation,
            "The number of iterations since the primal bound last improved significantly")
        .def_readwrite("lastIterationWithSignificantPrimalUpdate",
            &SolutionStatistics::lastIterationWithSignificantPrimalUpdate,
            "The last iteration in which the primal bound improved significantly")
        .def_readwrite("numberOfIterationsWithoutNLPCallMIP", &SolutionStatistics::numberOfIterationsWithoutNLPCallMIP,
            "The number of iterations with a MIP problem since an NLP problem was last solved")
        .def_readwrite("iterationLastPrimalBoundUpdate", &SolutionStatistics::iterationLastPrimalBoundUpdate,
            "The last iteration in which the primal bound improved")
        .def_readwrite("iterationLastDualBoundUpdate", &SolutionStatistics::iterationLastDualBoundUpdate,
            "The last iteration in which the dual bound improved")
        .def_readwrite("iterationLastLazyAdded", &SolutionStatistics::iterationLastLazyAdded,
            "The last iteration in which a lazy constraint was added in the single-tree strategy")
        .def_readwrite("iterationLastDualCutAdded", &SolutionStatistics::iterationLastDualCutAdded,
            "The last iteration in which a cut was added to the dual problem")
        .def_readwrite("timeLastDualBoundUpdate", &SolutionStatistics::timeLastDualBoundUpdate,
            "The time in seconds when the dual bound last improved")
        .def_readwrite("timeLastFixedNLPCall", &SolutionStatistics::timeLastFixedNLPCall,
            "The time in seconds when an NLP problem with fixed integer variables was last solved")
        .def_readwrite("numberOfOriginalInteriorPoints", &SolutionStatistics::numberOfOriginalInteriorPoints,
            "The number of interior points found for the ESH algorithm")
        .def_readwrite("numberOfFoundPrimalSolutions", &SolutionStatistics::numberOfFoundPrimalSolutions,
            "The number of primal solutions found")
        .def_readwrite("numberOfExploredNodes", &SolutionStatistics::numberOfExploredNodes,
            "The number of branch-and-bound nodes explored by the MIP solver")
        .def_readwrite("numberOfOpenNodes", &SolutionStatistics::numberOfOpenNodes,
            "The number of open branch-and-bound nodes of the MIP solver")
        .def_readwrite("numberOfPrimalReductionCutsUpdatesWithoutEffect",
            &SolutionStatistics::numberOfPrimalReductionCutsUpdatesWithoutEffect,
            "The number of primal reduction cuts that did not improve the primal bound")
        .def_readwrite("numberOfDualRepairsSinceLastPrimalUpdate",
            &SolutionStatistics::numberOfDualRepairsSinceLastPrimalUpdate,
            "The number of repairs of an infeasible dual problem since the primal bound last improved")
        .def_readwrite("numberOfPrimalReductionsPerformed", &SolutionStatistics::numberOfPrimalReductionsPerformed,
            "The number of primal reduction cuts added")
        .def_readwrite("numberOfSuccessfulDualRepairsPerformed",
            &SolutionStatistics::numberOfSuccessfulDualRepairsPerformed,
            "The number of successful repairs of an infeasible dual problem")
        .def_readwrite("numberOfUnsuccessfulDualRepairsPerformed",
            &SolutionStatistics::numberOfUnsuccessfulDualRepairsPerformed,
            "The number of unsuccessful repairs of an infeasible dual problem")
        .def_readwrite("numberOfPrimalImprovementsAfterInfeasibilityRepair",
            &SolutionStatistics::numberOfPrimalImprovementsAfterInfeasibilityRepair,
            "The number of primal bound improvements after a repair of the dual problem")
        .def_readwrite("numberOfPrimalImprovementsAfterReductionCut",
            &SolutionStatistics::numberOfPrimalImprovementsAfterReductionCut,
            "The number of primal bound improvements after a primal reduction cut")
        .def_readwrite("hasInfeasibilityRepairBeenPerformedSincePrimalImprovement",
            &SolutionStatistics::hasInfeasibilityRepairBeenPerformedSincePrimalImprovement,
            "Whether the dual problem has been repaired since the primal bound last improved")
        .def_readwrite("hasReductionCutBeenAddedSincePrimalImprovement",
            &SolutionStatistics::hasReductionCutBeenAddedSincePrimalImprovement,
            "Whether a primal reduction cut has been added since the primal bound last improved")
        .def("getNumberOfTotalDualProblems", &SolutionStatistics::getNumberOfTotalDualProblems,
            "The total number of dual problems solved");

    // -------------------------------------------------------------------------
    // Supporting types for callbacks
    // -------------------------------------------------------------------------

    py::class_<SolutionPoint>(m, "SolutionPoint")
        .def(py::init<>(), "A point found by SHOT, e.g., a solution of a dual problem")
        .def_readwrite("point", &SolutionPoint::point, "The values of the variables, in the order of their indexes")
        .def_readwrite("objectiveValue", &SolutionPoint::objectiveValue, "The objective value at the point")
        .def_readwrite("iterFound", &SolutionPoint::iterFound, "The iteration in which the point was found")
        .def_readwrite("maxDeviation", &SolutionPoint::maxDeviation,
            "The index of the constraint the point violates the most and the violation")
        .def_readwrite("isRelaxedPoint", &SolutionPoint::isRelaxedPoint,
            "Whether the point is a solution of a relaxation, e.g., of an LP problem")
        .def_readwrite("hashValue", &SolutionPoint::hashValue,
            "A hash of the point, used to recognize points that have been seen before");

    // Hyperplane base must be registered before ExternalHyperplane
    py::class_<Hyperplane>(m, "Hyperplane")
        .def_readwrite(
            "source", &Hyperplane::source, "Where the hyperplane comes from, e.g., HyperplaneSource.External")
        .def_readwrite("isGlobal", &Hyperplane::isGlobal,
            "Whether the hyperplane is valid for the whole problem, e.g., it is generated for a convex function.\n"
            "Adding one that is not means that SHOT no longer proves the solution optimal");

    py::class_<ExternalHyperplane, Hyperplane>(m, "ExternalHyperplane")
        .def(py::init<>(),
            "A cut sum(variableCoefficients[i] * x[variableIndexes[i]]) <= rhsValue, added to the dual problem\n"
            "with HyperplaneSelectionContext.addHyperplane()")
        .def_readwrite("variableIndexes", &ExternalHyperplane::variableIndexes,
            "The indexes of the variables of the hyperplane in the reformulated problem")
        .def_readwrite("variableCoefficients", &ExternalHyperplane::variableCoefficients,
            "The coefficients of the variables, so that the cut is sum(coefficients[i] * x[indexes[i]]) <= rhsValue")
        .def_readwrite(
            "description", &ExternalHyperplane::description, "A description of the hyperplane, used in the log")
        .def_readwrite("rhsValue", &ExternalHyperplane::rhsValue, "The right-hand side of the cut");

    // -------------------------------------------------------------------------
    // Callback contexts (passed to Python callbacks, only valid while the callback runs)
    // -------------------------------------------------------------------------

    py::register_exception<CallbackContextExpired>(m, "CallbackContextExpired", PyExc_RuntimeError);

    // The getters return copies, so what a callback keeps from a context is still valid after it has returned
    callbackContextClass
        .def_property_readonly("location", &CallbackContext::getLocation, "The location the callback is called at")
        .def_property_readonly("isValid", &CallbackContext::isValid,
            "Whether the context can still be used, i.e., the callback it was given to has not returned")
        .def_property_readonly("isMinimization", &CallbackContext::isMinimization, "Whether the problem is minimized")
        .def_property_readonly("iterationNumber", &CallbackContext::getIterationNumber,
            "The number of the current iteration, or 0 before the first iteration")
        .def_property_readonly(
            "elapsedTime", &CallbackContext::getElapsedTime, "The time since SHOT was started, in seconds")
        .def_property_readonly("dualBound", &CallbackContext::getDualBound,
            "The current dual bound, which the termination criteria use. For a nonconvex problem it is not a valid\n"
            "bound once cuts have been added to nonconvex functions; globalDualBound is")
        .def_property_readonly("globalDualBound", &CallbackContext::getGlobalDualBound,
            "The dual bound that is valid for the whole problem")
        .def_property_readonly("primalBound", &CallbackContext::getPrimalBound,
            "The objective value of the best primal solution, infinite if there is none")
        .def_property_readonly("relativeGap", &CallbackContext::getRelativeGap,
            "The relative gap between the current dual bound and the primal bound")
        .def_property_readonly("absoluteGap", &CallbackContext::getAbsoluteGap,
            "The absolute gap between the current dual bound and the primal bound")
        .def_property_readonly("solutionStatistics", &CallbackContext::getSolutionStatistics,
            "A copy of the statistics of the solution process")
        .def_property_readonly(
            "originalProblem", &CallbackContext::getOriginalProblem, "The problem given to the solver")
        .def_property_readonly("reformulatedProblem", &CallbackContext::getReformulatedProblem,
            "The problem that SHOT solves, created from the original one by the reformulation")
        .def_property_readonly(
            "hasPrimalSolution", &CallbackContext::hasPrimalSolution, "Whether a primal solution has been found")
        .def_property_readonly(
            "primalSolution",
            [](const CallbackContext& self) -> std::optional<VectorDouble>
            {
                if(!self.hasPrimalSolution())
                    return (std::nullopt);

                return (self.getPrimalSolution());
            },
            "The best primal solution in the variables of the original problem, or None if there is none")
        .def_property_readonly("isTerminationRequested", &CallbackContext::isTerminationRequested,
            "Whether terminate() has been called in this context, by this or an earlier callback at the location")
        .def_property_readonly("isTerminationPending", &CallbackContext::isTerminationPending,
            "Whether termination was requested before SHOT reached the location, so that SHOT is stopping")
        .def_property_readonly(
            "isFinalizing", &CallbackContext::isFinalizing, "Whether SHOT is finalizing the solution, for any reason")
        .def("terminate", &CallbackContext::terminate, "Request SHOT to terminate at its next termination check");

    py::class_<PrimalCandidateCheckContext, CallbackContext, std::shared_ptr<PrimalCandidateCheckContext>>(
        m, "PrimalCandidateCheckContext")
        .def_property_readonly(
            "point", [](const PrimalCandidateCheckContext& self) { return (VectorDouble(self.getPoint())); },
            "The candidate, in the variables of the original problem")
        .def_property_readonly(
            "objectiveValue", &PrimalCandidateCheckContext::getObjectiveValue, "The objective value of the candidate")
        .def_property_readonly("source", &PrimalCandidateCheckContext::getSource,
            "Where the candidate comes from, e.g., PrimalSolutionSource.NLPFixedIntegers")
        .def_property_readonly("isCandidateRejected", &PrimalCandidateCheckContext::isCandidateRejected,
            "Whether the candidate has been rejected, by this or an earlier callback")
        .def("rejectCandidate", &PrimalCandidateCheckContext::rejectCandidate,
            "SHOT will not check the candidate, so it cannot become a primal solution");

    py::class_<NewPrimalSolutionContext, CallbackContext, std::shared_ptr<NewPrimalSolutionContext>>(
        m, "NewPrimalSolutionContext")
        .def_property_readonly(
            "point", [](const NewPrimalSolutionContext& self) { return (VectorDouble(self.getPoint())); },
            "The solution, in the variables of the original problem")
        .def_property_readonly(
            "objectiveValue", &NewPrimalSolutionContext::getObjectiveValue, "The objective value of the solution")
        .def_property_readonly("source", &NewPrimalSolutionContext::getSource,
            "Where the solution comes from, e.g., PrimalSolutionSource.NLPFixedIntegers")
        .def_property_readonly("isIncumbent", &NewPrimalSolutionContext::isIncumbent,
            "Whether the solution is better than the best one SHOT had before it");

    py::class_<DualBoundUpdateContext, CallbackContext, std::shared_ptr<DualBoundUpdateContext>>(
        m, "DualBoundUpdateContext")
        .def_property_readonly("proposedDualBound", &DualBoundUpdateContext::getProposedDualBound,
            "The dual bound proposed with setDualBound() in this context, None if none has been")
        .def("setDualBound", &DualBoundUpdateContext::setDualBound,
            "Propose a dual bound; SHOT uses it if it is better than the current one", py::arg("value"));

    py::class_<PrimalCandidateSearchContext, CallbackContext, std::shared_ptr<PrimalCandidateSearchContext>>(
        m, "PrimalCandidateSearchContext")
        .def_property_readonly(
            "addedPrimalSolutions", [](const PrimalCandidateSearchContext& self)
            { return (std::vector<VectorDouble>(self.getAddedPrimalSolutions())); },
            "The points added with addPrimalSolution() in this context")
        .def("addPrimalSolution", &PrimalCandidateSearchContext::addPrimalSolution,
            "Add a primal solution candidate in the variables of the original or the reformulated problem",
            py::arg("point"));

    py::class_<HyperplaneSelectionContext, CallbackContext, std::shared_ptr<HyperplaneSelectionContext>>(
        m, "HyperplaneSelectionContext")
        .def_property_readonly(
            "solutionPoints", [](const HyperplaneSelectionContext& self)
            { return (std::vector<SolutionPoint>(self.getSolutionPoints())); },
            "The solution points of the dual problem, in the variables of the reformulated problem")
        .def_property_readonly(
            "addedHyperplanes", [](const HyperplaneSelectionContext& self)
            { return (std::vector<ExternalHyperplane>(self.getAddedHyperplanes())); },
            "The hyperplanes added with addHyperplane() in this context")
        .def("addHyperplane", &HyperplaneSelectionContext::addHyperplane,
            "Add a hyperplane in the variables of the reformulated problem", py::arg("hyperplane"));

    py::class_<InteriorPointSearchContext, CallbackContext, std::shared_ptr<InteriorPointSearchContext>>(
        m, "InteriorPointSearchContext")
        .def_property_readonly(
            "interiorPoints", [](const InteriorPointSearchContext& self)
            { return (std::vector<VectorDouble>(self.getInteriorPoints())); },
            "The interior points SHOT has found, in the variables of the reformulated problem")
        .def_property_readonly("replacementInteriorPoints", &InteriorPointSearchContext::getReplacementInteriorPoints,
            "The points set with setInteriorPoints() in this context, None if none have been")
        .def("setInteriorPoints", &InteriorPointSearchContext::setInteriorPoints,
            "Replace the interior points with at least one point in the variables of the original or the\n"
            "reformulated problem",
            py::arg("points"));

    py::class_<TerminationCheckContext, CallbackContext, std::shared_ptr<TerminationCheckContext>>(
        m, "TerminationCheckContext");
}
}
