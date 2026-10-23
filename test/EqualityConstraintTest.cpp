/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

// Tests that constraints with both a lower and an upper bound, L <= f(x) <= U, e.g. equality constraints, are kept as
// they are in the original problem, and are rewritten as f(x) <= U and -f(x) <= -L in the reformulated problem.

#include "../src/Solver.h"
#include "../src/DualSolver.h"
#include "../src/Environment.h"
#include "../src/PrimalSolver.h"
#include "../src/Results.h"
#include "../src/Settings.h"
#include "../src/Utilities.h"

#include "../src/Model/Variables.h"
#include "../src/Model/AuxiliaryVariables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"

#include "../src/Tasks/TaskPerformBoundTightening.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <set>

using namespace SHOT;

namespace
{

NonlinearExpressionPtr variable(VariablePtr V) { return (std::make_shared<ExpressionVariable>(V)); }

NonlinearExpressionPtr sum(NonlinearExpressionPtr first, NonlinearExpressionPtr second)
{
    return (std::make_shared<ExpressionSum>(first, second));
}

NonlinearExpressionPtr exponential(VariablePtr V) { return (std::make_shared<ExpressionExp>(variable(V))); }

NonlinearExpressionPtr logarithm(VariablePtr V) { return (std::make_shared<ExpressionLog>(variable(V))); }

NumericConstraintPtr getConstraint(ProblemPtr problem, const std::string& name)
{
    for(auto& C : problem->numericConstraints)
    {
        if(C->name == name)
            return (C);
    }

    return (nullptr);
}

std::unique_ptr<Solver> createSolver(ES_MIPSolver mipSolver = ES_MIPSolver::Highs)
{
    auto solver = std::make_unique<Solver>();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    return (solver);
}

bool expect(bool condition, const std::string& description)
{
    if(!condition)
        std::cout << "  FAILED: " << description << "\n";

    return (condition);
}

bool expectBounds(ProblemPtr problem, const std::string& name, double valueLHS, double valueRHS)
{
    auto constraint = getConstraint(problem, name);

    if(!constraint)
    {
        std::cout << "  FAILED: the problem has no constraint " << name << "\n";
        return (false);
    }

    if(constraint->valueLHS != valueLHS || constraint->valueRHS != valueRHS)
    {
        std::cout << "  FAILED: constraint " << name << " has the bounds [" << constraint->valueLHS << ", "
                  << constraint->valueRHS << "] instead of [" << valueLHS << ", " << valueRHS << "]\n";
        return (false);
    }

    return (true);
}

// The largest violation of the constraints of the problem in the point
double maxError(ProblemPtr problem, const VectorDouble& point)
{
    double error = 0.0;

    for(auto& C : problem->numericConstraints)
        error = std::max(error, C->calculateNumericValue(point).error);

    return (error);
}

// Calls the function for all points of a grid over the bounds of the variables of the problem
void forGridPoints(ProblemPtr problem, int pointsPerVariable, const std::function<void(const VectorDouble&)>& function)
{
    int numberOfVariables = problem->allVariables.size();
    VectorInteger position(numberOfVariables, 0);

    while(true)
    {
        VectorDouble point(numberOfVariables);

        for(int i = 0; i < numberOfVariables; i++)
        {
            auto V = problem->allVariables[i];
            point[i] = V->lowerBound + position[i] * (V->upperBound - V->lowerBound) / (pointsPerVariable - 1.0);
        }

        function(point);

        int i = 0;

        while(i < numberOfVariables && ++position[i] == pointsPerVariable)
            position[i++] = 0;

        if(i == numberOfVariables)
            break;
    }
}

// Checks that the reformulated problem describes the same set as the original one:
// - a point is feasible in the original problem exactly when it is feasible in the reformulated one, with the values
//   of the auxiliary variables calculated from their definitions,
// - all nonlinear constraints are of the form f(x) <= U, and
// - an auxiliary variable from a partitioning can not make an infeasible point feasible by taking another value than
//   that of the term it replaces. Such a variable w is only bounded from below by its definition g(x) - w <= 0, so
//   all other constraints with the variable must have only an upper bound and a positive coefficient for w: they are
//   then the most relaxed when w has its smallest value g(x).
bool isReformulationEquivalent(ProblemPtr original, ProblemPtr reformulated, int pointsPerVariable)
{
    bool passed = true;

    for(auto& C : reformulated->nonlinearConstraints)
        passed = expect(C->valueLHS == SHOT_DBL_MIN, "nonlinear constraint " + C->name + " has a lower bound")
            && passed;

    for(auto& V : reformulated->auxiliaryVariables)
    {
        auto type = V->properties.auxiliaryType;

        if(type != E_AuxiliaryVariableType::NonlinearExpressionPartitioning
            && type != E_AuxiliaryVariableType::MonomialTermsPartitioning
            && type != E_AuxiliaryVariableType::SignomialTermsPartitioning
            && type != E_AuxiliaryVariableType::SquareTermsPartitioning)
            continue;

        int numberOfDefinitions = 0;

        for(auto& C : reformulated->numericConstraints)
        {
            auto linearConstraint = std::dynamic_pointer_cast<LinearConstraint>(C);

            for(auto& T : linearConstraint->linearTerms)
            {
                if(T->variable != V || T->coefficient == 0.0)
                    continue;

                passed = expect(C->valueLHS == SHOT_DBL_MIN,
                             "constraint " + C->name + " with the partitioning variable " + V->name
                                 + " has a lower bound")
                    && passed;

                if(T->coefficient < 0.0)
                    numberOfDefinitions++;
            }
        }

        passed = expect(numberOfDefinitions == 1,
                     "the partitioning variable " + V->name + " has a negative coefficient in "
                         + std::to_string(numberOfDefinitions) + " constraints")
            && passed;
    }

    int numberOfFeasible = 0;
    int numberOfInfeasible = 0;
    int numberOfDifferent = 0;

    forGridPoints(original, pointsPerVariable, [&](const VectorDouble& point) {
        double originalError = maxError(original, point);

        auto reformulatedPoint = point;
        reformulated->augmentAuxiliaryVariableValues(reformulatedPoint);
        double reformulatedError = maxError(reformulated, reformulatedPoint);

        if(originalError <= 1e-9)
            numberOfFeasible++;
        else
            numberOfInfeasible++;

        // The constraints are scaled differently in the problems, so only whether there is an error is compared
        if((originalError > 1e-6 && reformulatedError < 1e-9) || (originalError < 1e-9 && reformulatedError > 1e-6))
        {
            if(numberOfDifferent == 0)
                std::cout << "  FAILED: the error is " << originalError << " in the original problem and "
                          << reformulatedError << " in the reformulated one in a point\n";

            numberOfDifferent++;
        }
    });

    std::cout << "  " << numberOfFeasible << " feasible and " << numberOfInfeasible << " infeasible grid points, "
              << numberOfDifferent << " with different feasibility in the problems\n";

    passed = expect(numberOfInfeasible > 0, "no grid point is infeasible") && passed;

    return (passed && numberOfDifferent == 0);
}

} // namespace

bool EqualityConstraintTestPreserved()
{
    bool passed = true;

    auto createProblem = [](EnvironmentPtr env) {
        auto problem = std::make_shared<Problem>(env);

        auto x = std::make_shared<Variable>("x", E_VariableType::Real, -2.0, 2.0);
        auto y = std::make_shared<Variable>("y", E_VariableType::Real, -2.0, 2.0);
        auto z = std::make_shared<Variable>("z", E_VariableType::Real, 0.5, 2.0);
        problem->add({ x, y, z });

        auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(std::make_shared<LinearTerm>(1.0, x));
        objective->add(std::make_shared<LinearTerm>(1.0, y));
        problem->add(objective);

        // x^2 = 1
        QuadraticTerms squareTerms;
        squareTerms.add(std::make_shared<QuadraticTerm>(1.0, x, x));
        problem->add(std::make_shared<QuadraticConstraint>("square", squareTerms, 1.0, 1.0));

        // 1 <= x * y + y * z <= 5
        QuadraticTerms bilinearTerms;
        bilinearTerms.add(std::make_shared<QuadraticTerm>(1.0, x, y));
        bilinearTerms.add(std::make_shared<QuadraticTerm>(1.0, y, z));
        problem->add(std::make_shared<QuadraticConstraint>("range", bilinearTerms, 1.0, 5.0));

        // exp(x) + exp(y) = 4
        problem->add(std::make_shared<NonlinearConstraint>("exp", sum(exponential(x), exponential(y)), 4.0, 4.0));

        // log(z) + exp(y) >= 0.5
        problem->add(
            std::make_shared<NonlinearConstraint>("lower", sum(logarithm(z), exponential(y)), 0.5, SHOT_DBL_MAX));

        // 0 <= x + y + z <= 3
        LinearTerms linearTerms;
        linearTerms.add(std::make_shared<LinearTerm>(1.0, x));
        linearTerms.add(std::make_shared<LinearTerm>(1.0, y));
        linearTerms.add(std::make_shared<LinearTerm>(1.0, z));
        problem->add(std::make_shared<LinearConstraint>("linear", linearTerms, 0.0, 3.0));

        return (problem);
    };

    auto checkOriginal = [&](ProblemPtr problem, const std::string& description) {
        bool isOk = true;

        std::cout << " The original problem " << description << ":\n";

        isOk = expect(problem->numericConstraints.size() == 5,
                   "the problem has " + std::to_string(problem->numericConstraints.size()) + " constraints")
            && isOk;

        isOk = expectBounds(problem, "square", 1.0, 1.0) && isOk;
        isOk = expectBounds(problem, "range", 1.0, 5.0) && isOk;
        isOk = expectBounds(problem, "exp", 4.0, 4.0) && isOk;
        isOk = expectBounds(problem, "lower", SHOT_DBL_MIN, -0.5) && isOk;
        isOk = expectBounds(problem, "linear", 0.0, 3.0) && isOk;

        if(!isOk)
            return (false);

        // The errors of x^2 = 1
        auto square = getConstraint(problem, "square");

        for(auto& [value, error] : std::vector<std::pair<double, double>> { { 0.0, 1.0 }, { 1.0, 0.0 },
                { -1.0, 0.0 }, { 1.5, 1.25 } })
        {
            double calculated = square->calculateNumericValue({ value, 0.0, 1.0 }).error;
            isOk = expect(std::abs(calculated - error) < 1e-12,
                       "the error of x^2 = 1 is " + std::to_string(calculated) + " for x = " + std::to_string(value))
                && isOk;
        }

        // Points below, in and above the range of x * y + y * z
        auto range = getConstraint(problem, "range");
        isOk = expect(!range->isFulfilled({ 0.5, 0.5, 1.0 }), "0.75 is in the range [1,5]") && isOk;
        isOk = expect(range->isFulfilled({ 1.0, 1.0, 1.0 }), "2 is not in the range [1,5]") && isOk;
        isOk = expect(!range->isFulfilled({ 2.0, 2.0, 2.0 }), "8 is in the range [1,5]") && isOk;

        // The function of an equality constraint can be convex, while the constraint is not
        auto exp = getConstraint(problem, "exp");
        isOk = expect(exp->properties.functionConvexity == E_Convexity::Convex, "exp(x) + exp(y) is not convex")
            && isOk;
        isOk = expect(exp->properties.convexity == E_Convexity::Nonconvex, "exp(x) + exp(y) = 4 is not nonconvex")
            && isOk;
        isOk = expect(square->properties.functionConvexity == E_Convexity::Convex, "x^2 is not convex") && isOk;
        isOk = expect(square->properties.convexity == E_Convexity::Nonconvex, "x^2 = 1 is not nonconvex") && isOk;
        isOk = expect(getConstraint(problem, "linear")->properties.convexity == E_Convexity::Linear,
                   "the linear range is not linear")
            && isOk;
        isOk = expect(problem->properties.convexity == E_ProblemConvexity::Nonconvex, "the problem is not nonconvex")
            && isOk;

        return (isOk);
    };

    {
        auto solver = createSolver();
        auto env = solver->getEnvironment();
        auto problem = createProblem(env);
        problem->finalize();

        passed = checkOriginal(problem, "after finalize") && passed;

        problem->updateProperties();
        problem->updateProperties();
        passed = checkOriginal(problem, "after updating the properties again") && passed;

        auto copy = problem->createCopy(env);
        passed = checkOriginal(copy, "as a copy") && passed;
        passed = checkOriginal(problem, "after being copied") && passed;
    }

    // The quadratic constraints with both bounds are only given as they are to a MIP solver that supports nonconvex
    // quadratic constraints, which is Gurobi with the strategy for nonconvex quadratic constraints
    struct Configuration
    {
        std::string description;
        ES_MIPSolver mipSolver;
        ES_QuadraticProblemStrategy quadraticStrategy;
        bool hasNativeQuadraticEqualities;
    };

    std::vector<Configuration> configurations;

#ifdef HAS_HIGHS
    configurations.push_back({ "HiGHS", ES_MIPSolver::Highs, ES_QuadraticProblemStrategy::Nonlinear, false });
#endif

#ifdef HAS_CBC
    configurations.push_back({ "Cbc", ES_MIPSolver::Cbc, ES_QuadraticProblemStrategy::Nonlinear, false });
#endif

#ifdef HAS_CPLEX
    configurations.push_back({ "Cplex with quadratics as nonlinear", ES_MIPSolver::Cplex,
        ES_QuadraticProblemStrategy::Nonlinear, false });
    configurations.push_back({ "Cplex with convex quadratic constraints", ES_MIPSolver::Cplex,
        ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained, false });
    configurations.push_back({ "Cplex with nonconvex quadratic constraints", ES_MIPSolver::Cplex,
        ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained, false });
#endif

#ifdef HAS_GUROBI
    configurations.push_back({ "Gurobi with quadratics as nonlinear", ES_MIPSolver::Gurobi,
        ES_QuadraticProblemStrategy::Nonlinear, false });
    configurations.push_back({ "Gurobi with convex quadratic constraints", ES_MIPSolver::Gurobi,
        ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained, false });
    configurations.push_back({ "Gurobi with nonconvex quadratic constraints", ES_MIPSolver::Gurobi,
        ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained, true });
#endif

    for(auto& C : configurations)
    {
        auto solver = createSolver(C.mipSolver);
        auto env = solver->getEnvironment();
        solver->updateSetting("Model.Reformulation.Quadratics.Strategy", static_cast<int>(C.quadraticStrategy));
        solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
        solver->updateSetting("Model.BoundTightening.InitialPOA.Use", false);

        auto problem = createProblem(env);
        problem->finalize();

        if(!solver->setProblem(problem))
        {
            std::cout << "  FAILED: could not set the problem.\n";
            return (false);
        }

        passed = checkOriginal(env->problem, "after setProblem with " + C.description) && passed;

        std::cout << " The reformulated problem:\n";

        auto reformulated = env->reformulatedProblem;

        for(auto& RC : reformulated->numericConstraints)
            std::cout << "  " << RC << "\n";

        passed = expect(getConstraint(reformulated, "exp_rf") != nullptr,
                     "the reformulated problem has no constraint exp_rf")
            && passed;

        if(C.hasNativeQuadraticEqualities)
        {
            passed = expectBounds(reformulated, "square", 1.0, 1.0) && passed;
            passed = expectBounds(reformulated, "range", 1.0, 5.0) && passed;
        }

        for(auto& name : { "square_rf", "range_rf" })
            passed = expect((getConstraint(reformulated, name) != nullptr) == !C.hasNativeQuadraticEqualities,
                         std::string("the constraint ") + name + " is "
                             + (C.hasNativeQuadraticEqualities ? "" : "not ") + "in the reformulated problem")
                && passed;

        for(auto& name : { "lower_rf", "linear_rf" })
            passed = expect(getConstraint(reformulated, name) == nullptr,
                         std::string("the reformulated problem has the constraint ") + name)
                && passed;

        // A quadratic constraint given to the MIP solver has a lower bound only if the solver supports it
        for(auto& QC : reformulated->quadraticConstraints)
            passed = expect(QC->valueLHS == SHOT_DBL_MIN || C.hasNativeQuadraticEqualities,
                         "the quadratic constraint " + QC->name + " has a lower bound")
                && passed;

        passed = expectBounds(reformulated, "linear", 0.0, 3.0) && passed;
        passed = isReformulationEquivalent(env->problem, reformulated, 21) && passed;
    }

    return (passed);
}

bool EqualityConstraintTestSides()
{
    // The sides of a constraint with all kinds of terms and a constant: the lower side is the negated function, with
    // the negated lower bound as upper bound

    bool passed = true;

    auto solver = createSolver();
    auto env = solver->getEnvironment();

    // The terms are to be kept as they are given
    solver->updateSetting("Model.Reformulation.Quadratics.ExtractStrategy", 0);
    solver->updateSetting("Model.Reformulation.Monomials.Extract", false);
    solver->updateSetting("Model.Reformulation.Signomials.Extract", false);

    auto problem = std::make_shared<Problem>(env);

    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.5, 3.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, 0.5, 3.0);
    auto z = std::make_shared<Variable>("z", E_VariableType::Real, 0.5, 3.0);
    problem->add({ x, y, z });

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(1.0, x));
    problem->add(objective);

    // 2 <= 1.5 + 2x - 3y + x^2 + 2xy - yz + 3xyz - 0.5x^1.5y^-1 + exp(x) + log(z) * y <= 30
    auto mixed = std::make_shared<NonlinearConstraint>("mixed", 2.0, 30.0);
    mixed->constant = 1.5;
    mixed->add(std::make_shared<LinearTerm>(2.0, x));
    mixed->add(std::make_shared<LinearTerm>(-3.0, y));
    mixed->add(std::make_shared<QuadraticTerm>(1.0, x, x));
    mixed->add(std::make_shared<QuadraticTerm>(2.0, x, y));
    mixed->add(std::make_shared<QuadraticTerm>(-1.0, y, z));
    mixed->add(std::make_shared<MonomialTerm>(3.0, Variables({ x, y, z })));
    mixed->add(std::make_shared<SignomialTerm>(-0.5,
        SignomialElements(
            { std::make_shared<SignomialElement>(x, 1.5), std::make_shared<SignomialElement>(y, -1.0) })));
    mixed->add(sum(exponential(x), std::make_shared<ExpressionProduct>(logarithm(z), variable(y))));
    problem->add(mixed);

    // x^2 + 2xy = 4, with the same quadratic terms
    QuadraticTerms quadraticTerms;
    quadraticTerms.add(std::make_shared<QuadraticTerm>(1.0, x, x));
    quadraticTerms.add(std::make_shared<QuadraticTerm>(2.0, x, y));
    LinearTerms linearTerms;
    linearTerms.add(std::make_shared<LinearTerm>(-1.0, z));
    auto quadratic = std::make_shared<QuadraticConstraint>("quadratic", linearTerms, quadraticTerms, 4.0, 4.0);
    quadratic->constant = -0.25;
    problem->add(quadratic);

    // x + exp(y) <= 10 has no lower bound
    LinearTerms upperTerms;
    upperTerms.add(std::make_shared<LinearTerm>(1.0, x));
    problem->add(std::make_shared<NonlinearConstraint>("upper", upperTerms, exponential(y), SHOT_DBL_MIN, 10.0));

    problem->finalize();

    // A second problem with the same variables, which the sides are created in. They are added to it under other
    // names, since the derivatives of a nonlinear expression can only be calculated for a constraint in a problem.
    auto destination = std::make_shared<Problem>(env);

    for(auto& V : problem->allVariables)
        destination->add(std::make_shared<Variable>(V->name, V->properties.type, V->lowerBound, V->upperBound));

    auto destinationObjective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    destinationObjective->add(std::make_shared<LinearTerm>(1.0, destination->getVariable(0)));
    destination->add(destinationObjective);

    std::map<std::string, std::string> printedBefore;

    for(auto& name : { "mixed", "quadratic" })
    {
        auto source = getConstraint(problem, name);

        std::stringstream stream;
        stream << source;
        printedBefore[name] = stream.str();

        for(auto& side : { E_ConstraintSide::Upper, E_ConstraintSide::Lower })
        {
            bool isUpper = (side == E_ConstraintSide::Upper);
            std::string description = std::string(isUpper ? "upper" : "lower") + " side of " + name;

            auto created = destination->createConstraintSide(source, side);

            passed = expect(created->name == source->name, "the " + description + " has another name") && passed;
            passed = expect(created->ownerProblem.lock() == destination,
                         "the " + description + " does not belong to the problem it was created in")
                && passed;
            passed = expect(destination->numericConstraints.size() == 0 || getConstraint(destination, name) == nullptr,
                         "the " + description + " was added to the problem")
                && passed;

            created->name = std::string(name) + (isUpper ? "_upper" : "_lower");
            destination->add(created);
        }
    }

    destination->finalize();

    std::vector<VectorDouble> points
        = { { 1.0, 1.0, 1.0 }, { 0.7, 2.3, 1.9 }, { 2.9, 0.6, 0.8 }, { 1.4, 1.4, 2.6 }, { 0.5, 3.0, 0.5 } };

    for(auto& name : { "mixed", "quadratic" })
    {
        auto source = getConstraint(problem, name);

        for(auto& side : { E_ConstraintSide::Upper, E_ConstraintSide::Lower })
        {
            bool isUpper = (side == E_ConstraintSide::Upper);
            double sign = isUpper ? 1.0 : -1.0;
            std::string description = std::string(isUpper ? "upper" : "lower") + " side of " + name;

            auto created = getConstraint(destination, std::string(name) + (isUpper ? "_upper" : "_lower"));

            std::cout << " The " << description << ": " << created << "\n";

            passed = expect(created->valueLHS == SHOT_DBL_MIN, "the " + description + " has a lower bound") && passed;
            passed = expect(created->valueRHS == (isUpper ? source->valueRHS : -source->valueLHS),
                         "the " + description + " has the upper bound " + std::to_string(created->valueRHS))
                && passed;
            passed = expect((std::dynamic_pointer_cast<NonlinearConstraint>(created) != nullptr)
                             == (std::dynamic_pointer_cast<NonlinearConstraint>(source) != nullptr),
                         "the " + description + " is of another class")
                && passed;

            for(auto& V : *created->getGradientSparsityPattern())
                passed = expect(V == destination->getVariable(V->getIndex()),
                             "the " + description + " has a variable of another problem")
                    && passed;

            for(auto& point : points)
            {
                double sourceValue = source->calculateFunctionValue(point);
                double createdValue = created->calculateFunctionValue(point);

                passed = expect(
                             std::abs(createdValue - sign * sourceValue) < 1e-10 * std::max(1.0, std::abs(sourceValue)),
                             "the function of the " + description + " is " + std::to_string(createdValue)
                                 + " where the one of the constraint is " + std::to_string(sourceValue))
                    && passed;

                // A point is feasible for both sides exactly when it is for the constraint
                auto sourceGradient = source->calculateGradient(point, false);
                auto createdGradient = created->calculateGradient(point, false);

                for(auto& G : sourceGradient)
                {
                    double createdElement = 0.0;

                    for(auto& CG : createdGradient)
                    {
                        if(CG.first->getIndex() == G.first->getIndex())
                            createdElement = CG.second;
                    }

                    passed = expect(
                                 std::abs(createdElement - sign * G.second) < 1e-9 * std::max(1.0, std::abs(G.second)),
                                 "the gradient of the " + description + " is " + std::to_string(createdElement)
                                     + " for " + G.first->name + " instead of " + std::to_string(sign * G.second))
                        && passed;
                }

                auto sourceHessian = source->calculateHessian(point, false);
                auto createdHessian = created->calculateHessian(point, false);

                for(auto& H : sourceHessian)
                {
                    double createdElement = 0.0;

                    for(auto& CH : createdHessian)
                    {
                        if(CH.first.first->getIndex() == H.first.first->getIndex()
                            && CH.first.second->getIndex() == H.first.second->getIndex())
                            createdElement = CH.second;
                    }

                    passed = expect(
                                 std::abs(createdElement - sign * H.second) < 1e-9 * std::max(1.0, std::abs(H.second)),
                                 "the Hessian of the " + description + " is " + std::to_string(createdElement)
                                     + " instead of " + std::to_string(sign * H.second))
                        && passed;
                }
            }
        }

        // Both sides together are the constraint
        auto upperSide = getConstraint(destination, std::string(name) + "_upper");
        auto lowerSide = getConstraint(destination, std::string(name) + "_lower");

        for(auto& point : points)
        {
            auto value = source->calculateNumericValue(point);
            double sidesError = std::max(
                upperSide->calculateNumericValue(point).error, lowerSide->calculateNumericValue(point).error);

            passed = expect(std::abs(value.error - sidesError) < 1e-10 * std::max(1.0, value.error),
                         std::string("the error of ") + name + " is " + std::to_string(value.error)
                             + " and the one of its sides is " + std::to_string(sidesError))
                && passed;
        }

        std::stringstream after;
        after << source;

        passed = expect(printedBefore[name] == after.str(), std::string("the constraint ") + name + " was changed")
            && passed;
        passed = expect(source->ownerProblem.lock() == problem, std::string(name) + " has another owner") && passed;
    }

    passed = expect(problem->numericConstraints.size() == 3, "constraints were added to the source problem") && passed;

    // A side without a bound can not be created
    bool hasThrown = false;

    try
    {
        destination->createConstraintSide(getConstraint(problem, "upper"), E_ConstraintSide::Lower);
    }
    catch(const std::invalid_argument&)
    {
        hasThrown = true;
    }

    passed = expect(hasThrown, "a lower side was created for a constraint without a lower bound") && passed;

    return (passed);
}

bool EqualityConstraintTestPartitioning()
{
    // The sums in equality constraints and ranges are partitioned after the constraints have been rewritten as two
    // constraints with upper bounds. If exp(x) + exp(y) = 4 was partitioned as w1 + w2 = 4, exp(x) <= w1 and
    // exp(y) <= w2, then x = y = 0 would be feasible with w1 = w2 = 2.

    bool passed = true;

    for(auto strategy :
        { ES_PartitionNonlinearSums::Always, ES_PartitionNonlinearSums::IfConvex, ES_PartitionNonlinearSums::Never })
    {
        std::cout << " Partitioning strategy " << static_cast<int>(strategy) << ":\n";

        auto solver = createSolver();
        auto env = solver->getEnvironment();
        solver->updateSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms", static_cast<int>(strategy));
        solver->updateSetting("Model.Reformulation.Constraint.PartitionQuadraticTerms", static_cast<int>(strategy));
        solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
        solver->updateSetting("Model.BoundTightening.InitialPOA.Use", false);

        auto problem = std::make_shared<Problem>(env);

        auto x = std::make_shared<Variable>("x", E_VariableType::Real, -2.0, 2.0);
        auto y = std::make_shared<Variable>("y", E_VariableType::Real, -2.0, 2.0);
        auto z = std::make_shared<Variable>("z", E_VariableType::Real, 0.5, 2.0);
        problem->add({ x, y, z });

        auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(std::make_shared<LinearTerm>(1.0, x));
        objective->add(std::make_shared<LinearTerm>(1.0, y));
        problem->add(objective);

        // exp(x) + exp(y) = 4, convex function
        problem->add(std::make_shared<NonlinearConstraint>("convex", sum(exponential(x), exponential(y)), 4.0, 4.0));

        // 2 <= exp(x) + z^2 + 0.5 z <= 6, convex function with the same term exp(x), a quadratic and a linear term
        auto range = std::make_shared<NonlinearConstraint>("range", exponential(x), 2.0, 6.0);
        range->add(std::make_shared<QuadraticTerm>(1.0, z, z));
        range->add(std::make_shared<LinearTerm>(0.5, z));
        range->constant = 0.25;
        problem->add(range);

        // log(z) + log(y + 3) - x^2 = 0.5, concave function
        auto concave = std::make_shared<NonlinearConstraint>("concave",
            sum(logarithm(z),
                std::make_shared<ExpressionLog>(sum(variable(y), std::make_shared<ExpressionConstant>(3.0)))),
            0.5, 0.5);
        concave->add(std::make_shared<QuadraticTerm>(-1.0, x, x));
        problem->add(concave);

        // exp(x) - exp(y) + sin(z) = 0.3, neither convex nor concave
        problem->add(std::make_shared<NonlinearConstraint>("nonconvex",
            sum(sum(exponential(x), std::make_shared<ExpressionNegate>(exponential(y))),
                std::make_shared<ExpressionSin>(variable(z))),
            0.3, 0.3));

        problem->finalize();

        if(!solver->setProblem(problem))
        {
            std::cout << "  FAILED: could not set the problem.\n";
            return (false);
        }

        passed = expect(env->problem->numericConstraints.size() == 4, "the original problem was changed") && passed;

        std::cout << "  " << env->reformulatedProblem->numericConstraints.size() << " constraints and "
                  << env->reformulatedProblem->auxiliaryVariables.size()
                  << " auxiliary variables in the reformulated problem\n";

        passed = isReformulationEquivalent(env->problem, env->reformulatedProblem, 41) && passed;

        // x = y = 0 fulfills exp(x) + exp(y) <= 4 but not the equality
        VectorDouble point = { 0.0, 0.0, 1.0 };
        env->reformulatedProblem->augmentAuxiliaryVariableValues(point);

        bool isLowerSideViolated = false;

        for(auto& C : env->reformulatedProblem->numericConstraints)
        {
            if(C->name.rfind("convex_rf", 0) == 0 && C->calculateNumericValue(point).error > 1.0)
                isLowerSideViolated = true;
        }

        passed = expect(isLowerSideViolated, "the lower side of exp(x) + exp(y) = 4 is not violated for x = y = 0")
            && passed;
    }

    return (passed);
}

bool EqualityConstraintTestSimplifications()
{
    // The square root of g(x)^2 and the square of sqrt(g(x)) are taken on both sides of a constraint when the
    // problem is finalized. This must give the same set also for constraints with both bounds, and for a negative
    // g(x).

    bool passed = true;

    auto solver = createSolver();
    auto env = solver->getEnvironment();

    // Otherwise the square roots are extracted as signomial terms
    solver->updateSetting("Model.Reformulation.Quadratics.ExtractStrategy", 0);
    solver->updateSetting("Model.Reformulation.Signomials.Extract", false);

    auto problem = std::make_shared<Problem>(env);

    auto x = std::make_shared<Variable>("x", E_VariableType::Real, -3.0, 3.0);
    auto p = std::make_shared<Variable>("p", E_VariableType::Real, 1.0, 5.0);
    auto q = std::make_shared<Variable>("q", E_VariableType::Real, 0.2, 0.8);
    auto s = std::make_shared<Variable>("s", E_VariableType::Real, 0.0, 9.0);
    problem->add({ x, p, q, s });

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(1.0, x));
    problem->add(objective);

    auto square = [](NonlinearExpressionPtr child) { return (std::make_shared<ExpressionSquare>(child)); };
    auto squareRoot = [](NonlinearExpressionPtr child) { return (std::make_shared<ExpressionSquareRoot>(child)); };

    // sin(x) has both signs, log(p + 2) is positive and log(q) is negative
    auto sinX = [&]() { return (std::make_shared<ExpressionSin>(variable(x))); };
    auto logP = [&]() {
        return (std::make_shared<ExpressionLog>(sum(variable(p), std::make_shared<ExpressionConstant>(2.0))));
    };
    auto logQ = [&]() { return (logarithm(q)); };

    struct Case
    {
        std::string name;
        int variableIndex;
        std::function<double(double)> function; // The function as given, of the variable
        double valueLHS;
        double valueRHS;
        std::vector<double> solutions; // Values of the variable that fulfill the constraint
    };

    std::vector<Case> cases;

    auto add = [&](const std::string& name, NonlinearExpressionPtr expression, double valueLHS, double valueRHS,
                   int variableIndex, std::function<double(double)> function, std::vector<double> solutions) {
        problem->add(std::make_shared<NonlinearConstraint>(name, expression, valueLHS, valueRHS));
        cases.push_back({ name, variableIndex, function, valueLHS, valueRHS, solutions });
    };

    auto valueSinX = [](double value) { return (std::sin(value) * std::sin(value)); };
    auto valueLogP = [](double value) { return (std::log(value + 2.0) * std::log(value + 2.0)); };
    auto valueLogQ = [](double value) { return (std::log(value) * std::log(value)); };
    auto valueRootS = [](double value) { return (std::sqrt(value)); };

    add("sin_eq", square(sinX()), 0.25, 0.25, 0, valueSinX, { std::asin(0.5), -std::asin(0.5) });
    add("sin_range", square(sinX()), 0.25, 0.64, 0, valueSinX, { 0.7, -0.7 });
    add("sin_upper", square(sinX()), SHOT_DBL_MIN, 0.25, 0, valueSinX, { 0.3, -0.3 });

    // log(p + 2) = 1.5 for p = exp(1.5) - 2
    add("positive_eq", square(logP()), 2.25, 2.25, 1, valueLogP, { std::exp(1.5) - 2.0 });
    add("positive_range", square(logP()), 1.44, 2.89, 1, valueLogP, { std::exp(1.5) - 2.0 });
    add("positive_upper", square(logP()), SHOT_DBL_MIN, 2.89, 1, valueLogP, { 1.0 });
    add("positive_lower", square(logP()), 1.44, SHOT_DBL_MAX, 1, valueLogP, { 5.0 });
    add("positive_redundant_lower", square(logP()), -1.0, 2.89, 1, valueLogP, { 1.0 });

    // log(q) = -1 for q = exp(-1)
    add("negative_eq", square(logQ()), 1.0, 1.0, 2, valueLogQ, { std::exp(-1.0) });
    add("negative_range", square(logQ()), 0.25, 1.0, 2, valueLogQ, { 0.5 });
    add("negative_upper", square(logQ()), SHOT_DBL_MIN, 1.0, 2, valueLogQ, { 0.5 });
    add("negative_lower", square(logQ()), 0.25, SHOT_DBL_MAX, 2, valueLogQ, { 0.25 });

    add("root_eq", squareRoot(variable(s)), 2.0, 2.0, 3, valueRootS, { 4.0 });
    add("root_range", squareRoot(variable(s)), 1.0, 2.0, 3, valueRootS, { 2.0 });
    add("root_negative_lower", squareRoot(variable(s)), -1.0, 2.0, 3, valueRootS, { 0.0, 0.5 });
    add("root_upper", squareRoot(variable(s)), SHOT_DBL_MIN, 2.0, 3, valueRootS, { 0.5 });
    add("root_infeasible", squareRoot(variable(s)), SHOT_DBL_MIN, -1.0, 3, valueRootS, {});

    problem->finalize();

    passed = expect(problem->numericConstraints.size() == cases.size(), "the number of constraints has changed")
        && passed;

    for(auto& C : cases)
    {
        auto constraint = getConstraint(problem, C.name);

        if(!constraint)
        {
            std::cout << "  FAILED: the problem has no constraint " << C.name << "\n";
            passed = false;
            continue;
        }

        std::cout << " " << constraint << "\n";

        auto V = problem->getVariable(C.variableIndex);
        VectorDouble point = { 0.0, 1.0, 0.5, 1.0 };

        int numberOfFeasible = 0;
        int numberOfDifferent = 0;

        for(int i = 0; i <= 1000; i++)
        {
            point[C.variableIndex] = V->lowerBound + i * (V->upperBound - V->lowerBound) / 1000.0;

            double value = C.function(point[C.variableIndex]);

            // Points on the bounds are skipped, since rounding errors decide their feasibility
            if(std::abs(value - C.valueLHS) < 1e-9 || std::abs(value - C.valueRHS) < 1e-9)
                continue;

            bool isFeasible = (value >= C.valueLHS && value <= C.valueRHS);

            if(isFeasible)
                numberOfFeasible++;

            if(constraint->isFulfilled(point) != isFeasible)
                numberOfDifferent++;
        }

        passed = expect(numberOfDifferent == 0,
                     "the constraint " + C.name + " is not the given one in " + std::to_string(numberOfDifferent)
                         + " points")
            && passed;

        bool isEquality = (C.valueLHS == C.valueRHS);

        passed = expect((numberOfFeasible > 0) == (!isEquality && C.name != "root_infeasible"),
                     std::to_string(numberOfFeasible) + " points are feasible for " + C.name)
            && passed;

        for(double solution : C.solutions)
        {
            point[C.variableIndex] = solution;

            passed = expect(constraint->calculateNumericValue(point).error < 1e-9,
                         "the constraint " + C.name + " is not fulfilled for the value " + std::to_string(solution))
                && passed;
        }
    }

    return (passed);
}

bool EqualityConstraintTestConvexRelaxation()
{
    // The convex relaxation of a problem has the convex side of a constraint with both bounds: f(x) <= U for a convex
    // function and L <= f(x) for a concave one. The cuts of the initial polyhedral outer approximation are generated
    // for these sides, so they must not cut off points fulfilling the constraints.

    bool passed = true;

    auto createProblem = [](EnvironmentPtr env) {
        auto problem = std::make_shared<Problem>(env);

        auto x = std::make_shared<Variable>("x", E_VariableType::Real, -2.0, 2.0);
        auto y = std::make_shared<Variable>("y", E_VariableType::Real, -2.0, 2.0);
        auto u = std::make_shared<Variable>("u", E_VariableType::Real, 0.2, 5.0);
        auto v = std::make_shared<Variable>("v", E_VariableType::Real, 0.2, 5.0);
        problem->add({ x, y, u, v });

        auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(std::make_shared<LinearTerm>(1.0, x));
        objective->add(std::make_shared<LinearTerm>(-1.0, u));
        problem->add(objective);

        // exp(x) + exp(y) = 4: convex function, the upper side is convex
        problem->add(std::make_shared<NonlinearConstraint>("convex_eq", sum(exponential(x), exponential(y)), 4.0, 4.0));

        // 1 <= exp(x) + exp(2y) <= 6: convex function, the upper side is convex
        problem->add(std::make_shared<NonlinearConstraint>("convex_range",
            sum(exponential(x),
                std::make_shared<ExpressionExp>(std::make_shared<ExpressionProduct>(
                    std::make_shared<ExpressionConstant>(2.0), variable(y)))),
            1.0, 6.0));

        // log(u) + log(v) = 0.5: concave function, the lower side is convex
        problem->add(std::make_shared<NonlinearConstraint>("concave_eq", sum(logarithm(u), logarithm(v)), 0.5, 0.5));

        // -1 <= log(u) + log(x + 3) <= 1.5: concave function, the lower side is convex
        problem->add(std::make_shared<NonlinearConstraint>("concave_range",
            sum(logarithm(u),
                std::make_shared<ExpressionLog>(sum(variable(x), std::make_shared<ExpressionConstant>(3.0)))),
            -1.0, 1.5));

        // exp(x) - exp(y) + sin(u) = 0.3: no side is convex
        problem->add(std::make_shared<NonlinearConstraint>("nonconvex_eq",
            sum(sum(exponential(x), std::make_shared<ExpressionNegate>(exponential(y))),
                std::make_shared<ExpressionSin>(variable(u))),
            0.3, 0.3));

        // exp(y) + v <= 8: convex
        LinearTerms linearTerms;
        linearTerms.add(std::make_shared<LinearTerm>(1.0, v));
        problem->add(
            std::make_shared<NonlinearConstraint>("convex_upper", linearTerms, exponential(y), SHOT_DBL_MIN, 8.0));

        // 1 <= x + y + u <= 6: linear
        LinearTerms rangeTerms;
        rangeTerms.add(std::make_shared<LinearTerm>(1.0, x));
        rangeTerms.add(std::make_shared<LinearTerm>(1.0, y));
        rangeTerms.add(std::make_shared<LinearTerm>(1.0, u));
        problem->add(std::make_shared<LinearConstraint>("linear_range", rangeTerms, 1.0, 6.0));

        problem->finalize();

        return (problem);
    };

    {
        auto solver = createSolver();
        auto env = solver->getEnvironment();
        auto problem = createProblem(env);
        auto relaxed = problem->createCopy(env, true, true);

        std::cout << " The convex relaxation:\n";

        for(auto& C : relaxed->numericConstraints)
            std::cout << "  " << C << "\n";

        passed = expect(relaxed->numericConstraints.size() == problem->numericConstraints.size(),
                     "the relaxation has another number of constraints")
            && passed;

        // The constraints have the same indices in both problems
        for(auto& C : problem->numericConstraints)
            passed = expect(relaxed->getConstraint(C->getIndex())->name == C->name,
                         "the constraint " + C->name + " has another index in the relaxation")
                && passed;

        passed = expectBounds(relaxed, "convex_eq", SHOT_DBL_MIN, 4.0) && passed;
        passed = expectBounds(relaxed, "convex_range", SHOT_DBL_MIN, 6.0) && passed;
        passed = expectBounds(relaxed, "concave_eq", SHOT_DBL_MIN, -0.5) && passed;
        passed = expectBounds(relaxed, "concave_range", SHOT_DBL_MIN, 1.0) && passed;
        passed = expectBounds(relaxed, "convex_upper", SHOT_DBL_MIN, 8.0) && passed;
        passed = expectBounds(relaxed, "linear_range", 1.0, 6.0) && passed;

        for(auto& name : { "convex_eq", "convex_range", "concave_eq", "concave_range", "convex_upper" })
            passed = expect(getConstraint(relaxed, name)->properties.convexity == E_Convexity::Convex,
                         std::string("the kept side of ") + name + " is not convex")
                && passed;

        VectorDouble point = { 0.3, -0.4, 1.7, 2.1 };

        // The kept side is the function of the constraint, or the negated function
        for(auto& [name, sign] : std::vector<std::pair<std::string, double>> { { "convex_eq", 1.0 },
                { "convex_range", 1.0 }, { "concave_eq", -1.0 }, { "concave_range", -1.0 } })
        {
            double value = getConstraint(problem, name)->calculateFunctionValue(point);
            double relaxedValue = getConstraint(relaxed, name)->calculateFunctionValue(point);

            passed = expect(std::abs(relaxedValue - sign * value) < 1e-12,
                         "the function of " + name + " is " + std::to_string(relaxedValue) + " in the relaxation")
                && passed;
        }

        // The nonconvex constraint is replaced by an empty one
        auto nonconvex = getConstraint(relaxed, "nonconvex_eq");
        passed = expect(nonconvex->properties.classification == E_ConstraintClassification::Linear
                         && !nonconvex->properties.hasLinearTerms,
                     "the nonconvex constraint is in the relaxation")
            && passed;

        // The problem is not changed, and a copy that is not relaxed has both bounds
        passed = expectBounds(problem, "convex_eq", 4.0, 4.0) && passed;
        passed = expectBounds(problem, "concave_range", -1.0, 1.5) && passed;

        auto copy = problem->createCopy(env, true, false);
        passed = expectBounds(copy, "convex_eq", 4.0, 4.0) && passed;
        passed = expectBounds(copy, "concave_range", -1.0, 1.5) && passed;
        passed = expectBounds(copy, "nonconvex_eq", 0.3, 0.3) && passed;
    }

    // The outer approximation is generated for the reformulated problem when the problem is set, where the
    // constraints have been rewritten as f(x) <= U and -f(x) <= -L (named _rf). It is also generated directly for the
    // original problem here, where the convex side has to be selected from the constraints with both bounds.
    struct Case
    {
        std::string description;
        bool usePartitioning;
        bool isForOriginalProblem;
        std::vector<std::string> constraintsWithCuts;
        std::vector<std::string> constraintsWithoutCuts;
    };

    std::vector<Case> cases = {
        { "the reformulated problem with partitioning", true, false, {}, { "nonconvex_eq", "nonconvex_eq_rf" } },
        { "the reformulated problem without partitioning", false, false, { "convex_eq", "concave_eq_rf" },
            { "convex_eq_rf", "concave_eq", "convex_range_rf", "concave_range", "nonconvex_eq", "nonconvex_eq_rf" } },
        { "the original problem", false, true, { "convex_eq", "concave_eq" }, { "nonconvex_eq" } },
    };

    for(auto& C : cases)
    {
        auto solver = createSolver();
        auto env = solver->getEnvironment();
        solver->updateSetting("Model.BoundTightening.InitialPOA.Use", !C.isForOriginalProblem);
        solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
        solver->updateSetting("Model.BoundTightening.InitialPOA.DirectionalSolves", 20);

        if(!C.usePartitioning)
            solver->updateSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms",
                static_cast<int>(ES_PartitionNonlinearSums::Never));

        if(!solver->setProblem(createProblem(env)))
        {
            std::cout << "  FAILED: could not set the problem.\n";
            return (false);
        }

        if(C.isForOriginalProblem)
        {
            solver->updateSetting("Model.BoundTightening.InitialPOA.Use", true);
            auto task = std::make_unique<TaskPerformBoundTightening>(env, env->problem);
            task->run();
        }

        auto problemWithCuts = C.isForOriginalProblem ? env->problem : env->reformulatedProblem;

        std::map<std::string, int> numberOfCuts;
        LinearConstraints cuts;

        for(auto& LC : problemWithCuts->linearConstraints)
        {
            if(LC->name.rfind("initPOA_", 0) != 0)
                continue;

            cuts.push_back(LC);

            // The name is initPOA_<constraint>_<number>
            auto name = LC->name.substr(8, LC->name.rfind('_') - 8);
            numberOfCuts[name]++;
        }

        std::cout << " Initial POA cuts for " << C.description << ":";

        for(auto& [name, number] : numberOfCuts)
            std::cout << " " << name << ": " << number;

        std::cout << "\n";

        passed = expect(cuts.size() > 0, "no cuts were generated") && passed;

        for(auto& name : C.constraintsWithCuts)
            passed = expect(numberOfCuts[name] > 0, "no cuts were generated for " + name) && passed;

        for(auto& name : C.constraintsWithoutCuts)
            passed = expect(numberOfCuts[name] == 0, "cuts were generated for " + name) && passed;

        // The original problem has the same constraints as before
        passed = expectBounds(env->problem, "convex_eq", 4.0, 4.0) && passed;
        passed = expectBounds(env->problem, "concave_eq", 0.5, 0.5) && passed;

        // No cut may be violated in a point fulfilling the nonlinear constraints. Points on the curves of the
        // equality constraints are used, since those are the ones a cut for the wrong side would cut off: the cut for
        // L <= f(x) of a convex function f is f(x0) + grad f(x0) (x - x0) >= L, which is violated on the curve.
        int numberOfPoints = 0;
        int numberOfViolations = 0;

        for(int i = 0; i <= 200; i++)
        {
            // exp(x) + exp(y) = 4
            double xValue = -2.0 + i * (std::log(4.0 - std::exp(-2.0)) + 2.0) / 200.0;
            double yValue = std::log(4.0 - std::exp(xValue));

            if(yValue < -2.0 || yValue > 2.0)
                continue;

            for(int j = 0; j <= 200; j++)
            {
                // log(u) + log(v) = 0.5
                double uValue = 0.2 + j * 4.8 / 200.0;
                double vValue = std::exp(0.5) / uValue;

                if(vValue < 0.2 || vValue > 5.0)
                    continue;

                VectorDouble point = { xValue, yValue, uValue, vValue };

                // The points also fulfill the other constraints with a convex side
                bool isFeasible = true;

                for(auto& name : { "convex_eq", "convex_range", "concave_eq", "concave_range", "convex_upper" })
                {
                    if(getConstraint(env->problem, name)->calculateNumericValue(point).error > 1e-9)
                        isFeasible = false;
                }

                if(!isFeasible)
                    continue;

                numberOfPoints++;

                if(!C.isForOriginalProblem)
                    env->reformulatedProblem->augmentAuxiliaryVariableValues(point);

                for(auto& LC : cuts)
                {
                    if(LC->calculateNumericValue(point).error > 1e-7)
                    {
                        if(numberOfViolations == 0)
                            std::cout << "  FAILED: the cut " << LC << " is violated by "
                                      << LC->calculateNumericValue(point).error << " in a feasible point\n";

                        numberOfViolations++;
                    }
                }
            }
        }

        std::cout << "  " << cuts.size() << " cuts checked in " << numberOfPoints << " points\n";

        passed = expect(numberOfPoints > 100, "too few points fulfill the constraints") && passed;
        passed = expect(numberOfViolations == 0, "cuts are violated in feasible points") && passed;
    }

    return (passed);
}

bool EqualityConstraintTestBilinear(ES_MIPSolver mipSolver, ES_QuadraticProblemStrategy quadraticStrategy)
{
    // Bilinear terms are replaced by auxiliary variables w, defined with the equality constraints x * y - w = 0. These
    // are nonconvex, so they are rewritten as two constraints unless the MIP solver gets nonconvex quadratic
    // constraints. The original problem has the same terms in an equality constraint, a range and the objective.

    bool passed = true;

    auto solver = createSolver(mipSolver);
    auto env = solver->getEnvironment();
    solver->updateSetting("Model.Reformulation.Quadratics.Strategy", static_cast<int>(quadraticStrategy));

    // The bilinear terms are otherwise kept in the constraints
    solver->updateSetting("Model.Reformulation.Quadratics.ExtractStrategy",
        static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractToEqualityConstraintIfNonconvex));

    solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
    solver->updateSetting("Model.BoundTightening.InitialPOA.Use", false);

    auto problem = std::make_shared<Problem>(env);

    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.5, 3.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, 0.5, 3.0);
    auto z = std::make_shared<Variable>("z", E_VariableType::Real, 0.5, 3.0);
    auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
    problem->add({ x, y, z, b });

    // minimize x*y + z + b
    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(1.0, x, y));
    objective->add(std::make_shared<LinearTerm>(1.0, z));
    objective->add(std::make_shared<LinearTerm>(1.0, b));
    problem->add(objective);

    // x*y + y*z = 4 + b
    QuadraticTerms equalityTerms;
    equalityTerms.add(std::make_shared<QuadraticTerm>(1.0, x, y));
    equalityTerms.add(std::make_shared<QuadraticTerm>(1.0, y, z));
    LinearTerms equalityLinearTerms;
    equalityLinearTerms.add(std::make_shared<LinearTerm>(-1.0, b));
    problem->add(
        std::make_shared<QuadraticConstraint>("equality", equalityLinearTerms, equalityTerms, 4.0, 4.0));

    // 1 <= y*z - x*z <= 2
    QuadraticTerms rangeTerms;
    rangeTerms.add(std::make_shared<QuadraticTerm>(1.0, y, z));
    rangeTerms.add(std::make_shared<QuadraticTerm>(-1.0, x, z));
    problem->add(std::make_shared<QuadraticConstraint>("range", rangeTerms, 1.0, 2.0));

    // exp(x) + x*y + y*z <= 30, with two of the terms in a constraint that is not quadratic
    QuadraticTerms nonlinearTerms;
    nonlinearTerms.add(std::make_shared<QuadraticTerm>(1.0, x, y));
    nonlinearTerms.add(std::make_shared<QuadraticTerm>(1.0, y, z));
    problem->add(
        std::make_shared<NonlinearConstraint>("nonlinear", nonlinearTerms, exponential(x), SHOT_DBL_MIN, 30.0));

    problem->finalize();

    if(!solver->setProblem(problem))
    {
        std::cout << "  FAILED: could not set the problem.\n";
        return (false);
    }

    passed = expect(env->problem->numericConstraints.size() == 3, "the original problem was changed") && passed;
    passed = expectBounds(env->problem, "equality", 4.0, 4.0) && passed;
    passed = expectBounds(env->problem, "range", 1.0, 2.0) && passed;

    auto reformulated = env->reformulatedProblem;

    std::cout << " The reformulated problem:\n";

    for(auto& C : reformulated->numericConstraints)
        std::cout << "  " << C << "\n";

    bool isNative = (mipSolver == ES_MIPSolver::Gurobi
        && quadraticStrategy == ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained);

    if(isNative)
    {
        // The quadratic constraints are given to the MIP solver as they are, with both bounds
        passed = expectBounds(reformulated, "equality", 4.0, 4.0) && passed;
        passed = expectBounds(reformulated, "range", 1.0, 2.0) && passed;

        for(auto& name : { "equality", "range" })
            passed = expect(std::dynamic_pointer_cast<QuadraticConstraint>(getConstraint(reformulated, name))
                             && !std::dynamic_pointer_cast<NonlinearConstraint>(getConstraint(reformulated, name)),
                         std::string(name) + " is not a quadratic constraint")
                && passed;

        // So are the equality constraints defining the auxiliary variables for the two terms in the constraint that
        // is not quadratic
        int numberOfDefinitions = 0;

        for(auto& C : reformulated->quadraticConstraints)
        {
            if(C->name.rfind("s_blcc_", 0) != 0)
                continue;

            numberOfDefinitions++;

            passed = expect(C->valueLHS == 0.0 && C->valueRHS == 0.0, C->name + " is not an equality constraint")
                && passed;
        }

        passed = expect(numberOfDefinitions == 2,
                     "there are " + std::to_string(numberOfDefinitions) + " definitions for the 2 bilinear terms")
            && passed;
    }
    else
    {
        // One auxiliary variable for each of the terms x*y, y*z and x*z, also when they are in several constraints
        int numberOfBilinear = 0;

        for(auto& V : reformulated->auxiliaryVariables)
        {
            if(V->properties.auxiliaryType == E_AuxiliaryVariableType::ContinuousBilinear)
                numberOfBilinear++;
        }

        passed = expect(numberOfBilinear == 3,
                     std::to_string(numberOfBilinear) + " auxiliary variables were created for the 3 bilinear terms")
            && passed;

        int numberOfUpperSides = 0;
        int numberOfLowerSides = 0;

        for(auto& C : reformulated->numericConstraints)
        {
            if(C->name.rfind("s_blcc_", 0) != 0)
                continue;

            bool isLowerSide = (C->name.size() > 3 && C->name.compare(C->name.size() - 3, 3, "_rf") == 0);

            (isLowerSide ? numberOfLowerSides : numberOfUpperSides)++;

            passed = expect(C->valueLHS == SHOT_DBL_MIN && C->valueRHS == 0.0,
                         "the constraint " + C->name + " is not of the form f(x) <= 0")
                && passed;
        }

        passed = expect(numberOfUpperSides == 3 && numberOfLowerSides == 3,
                     "there are " + std::to_string(numberOfUpperSides) + " upper and "
                         + std::to_string(numberOfLowerSides) + " lower sides for the 3 bilinear terms")
            && passed;
    }

    passed = isReformulationEquivalent(env->problem, reformulated, 9) && passed;

    solver->updateSetting("Termination.TimeLimit", 30.0);
    solver->solveProblem();

    if(!solver->hasPrimalSolution())
    {
        std::cout << "  FAILED: no solution was found\n";
        return (false);
    }

    auto solution = solver->getPrimalSolution();
    double error = maxError(env->problem, solution.point);

    std::cout << " Solution with objective value " << solution.objValue << ", dual bound "
              << solver->getCurrentDualBound() << " and error " << error << "\n";

    passed = expect(error < 1e-5, "the solution does not fulfill the constraints of the original problem") && passed;
    passed = expect(solver->getCurrentDualBound() <= solution.objValue + 1e-6, "the dual bound is above the solution")
        && passed;

    // The best solution on a grid, where z is given by the equality constraint, is an upper bound of the optimal
    // objective value, so the dual bound may not be above it
    double bestValue = SHOT_DBL_MAX;

    for(double bValue : { 0.0, 1.0 })
    {
        for(int i = 0; i <= 1000; i++)
        {
            for(int j = 0; j <= 1000; j++)
            {
                double xValue = 0.5 + i * 2.5 / 1000.0;
                double yValue = 0.5 + j * 2.5 / 1000.0;
                double zValue = (4.0 + bValue - xValue * yValue) / yValue;
                double rangeValue = yValue * zValue - xValue * zValue;

                if(zValue < 0.5 || zValue > 3.0 || rangeValue < 1.0 || rangeValue > 2.0
                    || std::exp(xValue) + xValue * yValue + yValue * zValue > 30.0)
                    continue;

                bestValue = std::min(bestValue, xValue * yValue + zValue + bValue);
            }
        }
    }

    std::cout << " Best objective value on a grid: " << bestValue << "\n";

    passed = expect(solver->getCurrentDualBound() <= bestValue + 1e-6, "the dual bound is above a feasible solution")
        && passed;
    passed = expect(solution.objValue >= bestValue - 1e-2, "the solution is better than the optimal one") && passed;

    if(isNative)
        passed = expect(std::abs(solution.objValue - bestValue) < 1e-2
                         && solver->getCurrentDualBound() > bestValue - 2e-2,
                     "the problem was not solved to global optimality")
            && passed;

    return (passed);
}

bool EqualityConstraintTestNLPSources(ES_PrimalNLPSolver nlpSolver)
{
    // The NLP problems with fixed integer variables are solved for the original problem, where the equality
    // constraints are given as such, for the reformulated problem, where they are two constraints each, or for both.
    // The integer variables are fixed to several different values during the solution.

    bool passed = true;

    for(auto source : { ES_PrimalNLPProblemSource::OriginalProblem, ES_PrimalNLPProblemSource::ReformulatedProblem,
            ES_PrimalNLPProblemSource::Both })
    {
        auto solver = createSolver();
        auto env = solver->getEnvironment();
        solver->updateSetting("Primal.FixedInteger.Solver", static_cast<int>(nlpSolver));
        solver->updateSetting("Primal.FixedInteger.SourceProblem", static_cast<int>(source));
        solver->updateSetting("Termination.TimeLimit", 30.0);

        auto problem = std::make_shared<Problem>(env);

        auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.1, 4.0);
        auto y = std::make_shared<Variable>("y", E_VariableType::Real, 0.1, 4.0);
        auto z = std::make_shared<Variable>("z", E_VariableType::Real, 0.1, 4.0);
        auto i = std::make_shared<Variable>("i", E_VariableType::Integer, 0.0, 3.0);
        auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
        problem->add({ x, y, z, i, b });

        // minimize (x - 2)^2 + y + 2 z - 0.5 i + 1.5 b
        auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(std::make_shared<QuadraticTerm>(1.0, x, x));
        objective->add(std::make_shared<LinearTerm>(-4.0, x));
        objective->add(std::make_shared<LinearTerm>(1.0, y));
        objective->add(std::make_shared<LinearTerm>(2.0, z));
        objective->add(std::make_shared<LinearTerm>(-0.5, i));
        objective->add(std::make_shared<LinearTerm>(1.5, b));
        objective->constant = 4.0;
        problem->add(objective);

        // exp(x) + exp(y) = 6 + 2 i
        LinearTerms firstTerms;
        firstTerms.add(std::make_shared<LinearTerm>(-2.0, i));
        problem->add(std::make_shared<NonlinearConstraint>(
            "exp_eq", firstTerms, sum(exponential(x), exponential(y)), 6.0, 6.0));

        // log(y) + log(z) = 0.2 - b, which is z = exp(0.2 - b) / y
        LinearTerms secondTerms;
        secondTerms.add(std::make_shared<LinearTerm>(1.0, b));
        problem->add(
            std::make_shared<NonlinearConstraint>("log_eq", secondTerms, sum(logarithm(y), logarithm(z)), 0.2, 0.2));

        // 1 <= x * z + i <= 4
        QuadraticTerms rangeTerms;
        rangeTerms.add(std::make_shared<QuadraticTerm>(1.0, x, z));
        LinearTerms rangeLinearTerms;
        rangeLinearTerms.add(std::make_shared<LinearTerm>(1.0, i));
        problem->add(std::make_shared<QuadraticConstraint>("range", rangeLinearTerms, rangeTerms, 1.0, 4.0));

        problem->finalize();

        if(!solver->setProblem(problem))
        {
            std::cout << "  FAILED: could not set the problem.\n";
            return (false);
        }

        solver->solveProblem();

        std::cout << " NLP source " << static_cast<int>(source) << ": "
                  << env->solutionStatistics.numberOfProblemsFixedNLP << " NLP problems solved\n";

        passed = expect(env->problem->numericConstraints.size() - env->problem->properties.numberOfAddedLinearizations
                         == 3,
                     "the original problem was changed")
            && passed;
        passed = expectBounds(env->problem, "exp_eq", 6.0, 6.0) && passed;
        passed = expectBounds(env->problem, "log_eq", 0.2, 0.2) && passed;
        passed = expectBounds(env->problem, "range", 1.0, 4.0) && passed;

        // The bounds of the integer variables are restored after the NLP problems
        passed = expect(env->problem->getVariable(3)->lowerBound == 0.0
                         && env->problem->getVariable(3)->upperBound == 3.0
                         && env->problem->getVariable(4)->lowerBound == 0.0
                         && env->problem->getVariable(4)->upperBound == 1.0,
                     "the bounds of the integer variables were changed")
            && passed;

        if(!solver->hasPrimalSolution())
        {
            std::cout << "  FAILED: no solution was found\n";
            passed = false;
            continue;
        }

        passed = expect(env->solutionStatistics.numberOfProblemsFixedNLP > 0, "no NLP problems were solved") && passed;

        // All solutions found fulfill the equality constraints
        for(auto& S : env->results->primalSolutions)
        {
            double error = maxError(env->problem, S.point);

            passed = expect(error < 1e-5,
                         "a solution violates the constraints of the original problem by " + std::to_string(error))
                && passed;

            passed = expect(std::abs(S.point[3] - std::round(S.point[3])) < 1e-6
                             && std::abs(S.point[4] - std::round(S.point[4])) < 1e-6,
                         "a solution is not integer")
                && passed;
        }

        auto solution = solver->getPrimalSolution();

        std::cout << "  Solution with objective value " << solution.objValue << " and dual bound "
                  << solver->getCurrentDualBound() << ": x = " << solution.point[0] << ", y = " << solution.point[1]
                  << ", z = " << solution.point[2] << ", i = " << solution.point[3] << ", b = " << solution.point[4]
                  << "\n";

        passed = expect(solver->getCurrentDualBound() <= solution.objValue + 1e-6,
                     "the dual bound is above the solution")
            && passed;
    }

    return (passed);
}

bool EqualityConstraintTestSquareNLP(ES_PrimalNLPSolver nlpSolver)
{
    // The instance ex9_1_8 from MINLPLib, with the optimal objective value -3.25. It has complementarity constraints
    // x_i * x_j = 0, and with these as equality constraints, i.e. with the original problem as the source of the NLP
    // problem, the NLP problem solved for it gets as many equality constraints as free variables. Ipopt solves such a problem as a system of equations, and returns that a feasible
    // point was found instead of an optimal one.

    bool passed = true;

    auto solver = createSolver();
    auto env = solver->getEnvironment();
    solver->updateSetting("Primal.FixedInteger.Solver", static_cast<int>(nlpSolver));
    solver->updateSetting(
        "Primal.FixedInteger.SourceProblem", static_cast<int>(ES_PrimalNLPProblemSource::OriginalProblem));
    solver->updateSetting("Termination.TimeLimit", 30.0);

    auto problem = std::make_shared<Problem>(env);

    // The variables x2, ..., x15
    std::map<int, VariablePtr> x;

    for(int i = 2; i <= 15; i++)
    {
        x[i] = std::make_shared<Variable>("x" + std::to_string(i), E_VariableType::Real, 0.0, SHOT_DBL_MAX);
        problem->add(x[i]);
    }

    auto linearTerms = [&](std::vector<std::pair<double, int>> terms) {
        LinearTerms result;

        for(auto& [coefficient, index] : terms)
            result.add(std::make_shared<LinearTerm>(coefficient, x[index]));

        return (result);
    };

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(linearTerms({ { -2.0, 2 }, { 1.0, 3 }, { 0.5, 4 } }));
    problem->add(objective);

    problem->add(std::make_shared<LinearConstraint>("e2", linearTerms({ { 1.0, 2 }, { 1.0, 3 } }), SHOT_DBL_MIN, 2.0));
    problem->add(std::make_shared<LinearConstraint>(
        "e3", linearTerms({ { -2.0, 2 }, { 1.0, 4 }, { -1.0, 5 }, { 1.0, 6 } }), -2.5, -2.5));
    problem->add(std::make_shared<LinearConstraint>(
        "e4", linearTerms({ { 1.0, 2 }, { -3.0, 3 }, { 1.0, 5 }, { 1.0, 7 } }), 2.0, 2.0));
    problem->add(std::make_shared<LinearConstraint>("e5", linearTerms({ { -1.0, 4 }, { 1.0, 8 } }), 0.0, 0.0));
    problem->add(std::make_shared<LinearConstraint>("e6", linearTerms({ { -1.0, 5 }, { 1.0, 9 } }), 0.0, 0.0));

    // The complementarity constraints e7, ..., e11
    int constraintNumber = 7;

    for(auto& [first, second] : std::vector<std::pair<int, int>> { { 11, 6 }, { 12, 7 }, { 13, 8 }, { 14, 9 },
            { 15, 10 } })
    {
        QuadraticTerms terms;
        terms.add(std::make_shared<QuadraticTerm>(1.0, x[first], x[second]));
        problem->add(
            std::make_shared<QuadraticConstraint>("e" + std::to_string(constraintNumber++), terms, 0.0, 0.0));
    }

    problem->add(std::make_shared<LinearConstraint>("e12", linearTerms({ { 1.0, 11 }, { -1.0, 13 } }), 4.0, 4.0));
    problem->add(std::make_shared<LinearConstraint>(
        "e13", linearTerms({ { 1.0, 11 }, { 1.0, 12 }, { -1.0, 14 } }), -1.0, -1.0));

    problem->finalize();

    if(!solver->setProblem(problem))
    {
        std::cout << "  FAILED: could not set the problem.\n";
        return (false);
    }

    solver->solveProblem();

    std::cout << " Problem with complementarity constraints: " << env->solutionStatistics.numberOfProblemsFixedNLP
              << " NLP problems solved\n";

    if(!solver->hasPrimalSolution())
    {
        std::cout << "  FAILED: no solution was found\n";
        return (false);
    }

    auto solution = solver->getPrimalSolution();

    std::cout << "  Solution with objective value " << solution.objValue << "\n";

    passed = expect(std::abs(solution.objValue + 3.25) < 1e-4, "the objective value is not -3.25") && passed;
    passed = expect(maxError(env->problem, solution.point) < 1e-6, "the solution violates the equality constraints")
        && passed;

    return (passed);
}

int EqualityConstraintTest(int argc, char* argv[])
{
    int choice = 1;

    if(argc > 1)
    {
        if(sscanf(argv[1], "%d", &choice) != 1)
        {
            printf("Couldn't parse that input as a number\n");
            return -1;
        }
    }

    bool passed = true;

    switch(choice)
    {
    case 1:
        std::cout << "Starting test that equality constraints and ranges are kept in the original problem:\n";
        passed = EqualityConstraintTestPreserved();
        break;
    case 2:
        std::cout << "Starting test of creating the sides of a constraint:\n";
        passed = EqualityConstraintTestSides();
        break;
    case 3:
        std::cout << "Starting test of partitioning equality constraints and ranges:\n";
        passed = EqualityConstraintTestPartitioning();
        break;
    case 4:
        std::cout << "Starting test of simplifying squares and square roots in constraints with two bounds:\n";
        passed = EqualityConstraintTestSimplifications();
        break;
    case 5:
        std::cout << "Starting test of the convex relaxation of equality constraints and ranges:\n";
        passed = EqualityConstraintTestConvexRelaxation();
        break;
    case 6:
        std::cout << "Starting test of bilinear terms in equality constraints with HiGHS:\n";
#ifdef HAS_HIGHS
        passed = EqualityConstraintTestBilinear(ES_MIPSolver::Highs, ES_QuadraticProblemStrategy::Nonlinear);
#endif
        break;
    case 7:
        std::cout << "Starting test of bilinear terms in equality constraints with Cbc:\n";
#ifdef HAS_CBC
        passed = EqualityConstraintTestBilinear(ES_MIPSolver::Cbc, ES_QuadraticProblemStrategy::Nonlinear);
#endif
        break;
    case 8:
        std::cout << "Starting test of bilinear terms in equality constraints with Gurobi:\n";
#ifdef HAS_GUROBI
        passed = EqualityConstraintTestBilinear(
            ES_MIPSolver::Gurobi, ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained);
        passed = EqualityConstraintTestBilinear(
                     ES_MIPSolver::Gurobi, ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained)
            && passed;
#endif
        break;
    case 9:
        std::cout << "Starting test of bilinear terms in equality constraints with Cplex:\n";
#ifdef HAS_CPLEX
        passed = EqualityConstraintTestBilinear(
            ES_MIPSolver::Cplex, ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained);
        passed = EqualityConstraintTestBilinear(
                     ES_MIPSolver::Cplex, ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained)
            && passed;
#endif
        break;
    case 10:
        std::cout << "Starting test of equality constraints in NLP problems solved with Ipopt:\n";
#ifdef HAS_IPOPT
        passed = EqualityConstraintTestNLPSources(ES_PrimalNLPSolver::Ipopt);
        passed = EqualityConstraintTestSquareNLP(ES_PrimalNLPSolver::Ipopt) && passed;
#endif
        break;
    case 11:
        std::cout << "Starting test of equality constraints in NLP problems solved with Uno:\n";
#ifdef HAS_UNO
        passed = EqualityConstraintTestNLPSources(ES_PrimalNLPSolver::Uno);
        passed = EqualityConstraintTestSquareNLP(ES_PrimalNLPSolver::Uno) && passed;
#endif
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    if(passed)
        return 0;
    else
        return -1;
}
