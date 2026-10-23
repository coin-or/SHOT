/** The Supporting Hyperplane Optimization Toolkit (SHOT).
 * Licensed under the Eclipse Public License 2.0. See LICENSE.
 */
#include "NLPSolverConopt.h"
#include "../Model/Problem.h"
#include "../Output.h"
#include "../Callback.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"
#include <conopt.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <stdexcept>
#include <unistd.h>

namespace SHOT
{
namespace
{
    bool userTerminated(const EnvironmentPtr& env)
    {
        if(!env->tasks->isTerminated() && env->callbacks->isActive(E_CallbackLocation::TerminationCheck))
        {
            auto context = std::make_shared<TerminationCheckContext>(env);
            env->callbacks->invoke(*context);
            context->invalidate();
        }
        return env->tasks->isTerminated();
    }

    double conoptBound(double value)
    {
        if(value <= SHOT_DBL_MIN)
            return -Conopt::Infinity;
        if(value >= SHOT_DBL_MAX)
            return Conopt::Infinity;
        // Do not silently turn a finite bound into CONOPT infinity.
        if(!std::isfinite(value) || std::abs(value) >= Conopt::Infinity)
            throw std::runtime_error("Finite bound exceeds CONOPT's supported range");
        return value;
    }

    // Constant derivatives can live inside expression trees. Conservatively retain every
    // expression variable as nonlinear; use structural Hessians for algebraic terms.
    template <class Function>
    void rowStructure(Function& function, VectorInteger& columns, VectorDouble& coefficients, VectorInteger& nonlinear,
        const VectorDouble& point)
    {
        std::set<int> nonlinearVariables;

        for(const auto& entry : *function.getHessianSparsityPattern())
        {
            nonlinearVariables.insert(entry.first->getIndex());
            nonlinearVariables.insert(entry.second->getIndex());
        }

        SparseVariableVector gradient;
        bool gradientCalculated = false;

        for(const auto& variable : *function.getGradientSparsityPattern())
        {
            int index = variable->getIndex();
            bool nl = function.properties.hasNonlinearExpression || nonlinearVariables.count(index);
            columns.push_back(index);
            nonlinear.push_back(nl ? 1 : 0);

            if(!nl && !gradientCalculated)
            {
                gradient = function.calculateGradient(point, false);
                gradientCalculated = true;
            }

            double coefficient = nl ? 0.0 : gradient[variable];
            if(!std::isfinite(coefficient))
                throw std::runtime_error("Non-finite linear coefficient");
            coefficients.push_back(coefficient);
        }
    }

    class ConoptProblem : public ConoptModelData
    {
    public:
        ConoptProblem(
            ProblemPtr problem, const VectorDouble& lower, const VectorDouble& upper, const VectorDouble& initial)
            : problem(problem), n(initial.size())
        {
            for(size_t i = 0; i < n; ++i)
                addVariable(conoptBound(lower[i]), conoptBound(upper[i]), initial[i]);

            for(const auto& constraint : problem->numericConstraints)
            {
                VectorInteger columns, nonlinear;
                VectorDouble coefficients;
                rowStructure(*constraint, columns, coefficients, nonlinear, initial);
                double offset = constraint->constant;
                if(std::none_of(nonlinear.begin(), nonlinear.end(), [](int nl) { return nl != 0; }))
                {
                    offset = constraint->calculateFunctionValue(initial);

                    for(size_t i = 0; i < columns.size(); ++i)
                        offset -= coefficients[i] * initial[columns[i]];
                }
                offsets.push_back(offset);
                double lowerRow = constraint->valueLHS;
                double upperRow = constraint->valueRHS;
                bool hasLower = lowerRow > SHOT_DBL_MIN;
                bool hasUpper = upperRow < SHOT_DBL_MAX;
                int slack = -1;
                double rhs = 0.0;
                auto type = ConoptConstraintType::Free;
                if(hasLower && hasUpper && lowerRow != upperRow)
                {
                    double lb = conoptBound(lowerRow - offset);
                    double ub = conoptBound(upperRow - offset);
                    double value = constraint->calculateFunctionValue(initial) - offset;
                    if(!std::isfinite(value))
                        value = 0.0;
                    slack = addVariable(lb, ub, std::clamp(value, lb, ub));
                    columns.push_back(slack);
                    coefficients.push_back(-1.0);
                    nonlinear.push_back(0);
                    type = ConoptConstraintType::Eq;
                }
                else if(hasLower || hasUpper)
                {
                    rhs = (hasLower ? lowerRow : upperRow) - offset;
                    type = hasLower && hasUpper ? ConoptConstraintType::Eq
                                                : (hasLower ? ConoptConstraintType::GtEq : ConoptConstraintType::LtEq);
                }
                addConstraint(type, rhs, columns, coefficients, nonlinear);
                slacks.push_back(slack);
            }
            VectorInteger columns, nonlinear;
            VectorDouble coefficients;
            rowStructure(*problem->objectiveFunction, columns, coefficients, nonlinear, initial);
            objectiveOffset = problem->objectiveFunction->constant;
            if(std::none_of(nonlinear.begin(), nonlinear.end(), [](int nl) { return nl != 0; }))
            {
                objectiveOffset = problem->objectiveFunction->calculateValue(initial);

                for(size_t i = 0; i < columns.size(); ++i)
                    objectiveOffset -= coefficients[i] * initial[columns[i]];
            }
            if(columns.empty())
            {
                // CONOPT's matrix loader requires at least one nonzero, even for a
                // constant model. A fixed zero column leaves the objective unchanged.
                columns.push_back(addVariable(0.0, 0.0, 0.0));
                coefficients.push_back(1.0);
                nonlinear.push_back(0);
            }
            // A free objective row reports value minus RHS. Move its constant to RHS,
            // so purely linear models do not need FDEval (the SDK does not register it).
            int objective
                = addConstraint(ConoptConstraintType::Free, -objectiveOffset, columns, coefficients, nonlinear);
            setObjectiveElement(ConoptObjectiveElement::Constraint, objective);
            setOptimizationSense(problem->objectiveFunction->direction == E_ObjectiveFunctionDirection::Minimize
                    ? ConoptSense::Minimize
                    : ConoptSense::Maximize);
            VectorInteger rows, cols;

            for(const auto& entry : *problem->getLagrangianHessianSparsityPattern())
            {
                int i = entry.first->getIndex(), j = entry.second->getIndex();
                std::pair<int, int> pair(std::min(i, j), std::max(i, j));
                if(hessianPositions.count(pair))
                    continue;
                hessianPositions[pair] = rows.size();
                rows.push_back(pair.second);
                cols.push_back(pair.first);
            }
            if(!rows.empty())
                setSDLagrangianStructure(rows, cols);

            // Combine expression Hessians on the shared AD tape, as in the Ipopt adapter.
            size_t expressionRows = 0;
            std::set<std::pair<size_t, size_t>> entries;

            for(const auto& constraint : problem->nonlinearConstraints)
            {
                if(!constraint->properties.hasNonlinearExpression)
                    continue;
                ++expressionRows;
                const auto& pattern = constraint->nonlinearHessianSparsityPattern;

                for(size_t k = 0; k < pattern.nnz(); ++k)
                    entries.emplace(pattern.row()[k], pattern.col()[k]);
            }

            if(expressionRows > 1)
            {
                const size_t dimension = problem->nonlinearExpressionVariables.size();
                expressionHessianPattern.resize(dimension, dimension, entries.size());
                size_t k = 0;

                for(const auto& entry : entries)
                    expressionHessianPattern.set(k++, entry.first, entry.second);
            }

            evaluationPoint.resize(n);
            batchGradients.resize(slacks.size());
        }

        // The source model is finalized and structurally fixed for this NLP solver's lifetime.
        // A fresh CONOPT solve reads this retained matrix with the new bounds and initial point.
        void update(const VectorDouble& lower, const VectorDouble& upper, const VectorDouble& initial)
        {
            batchReady = false;

            for(size_t i = 0; i < n; ++i)
            {
                auto& variable = getVariable(i);
                variable.lower = conoptBound(lower[i]);
                variable.upper = conoptBound(upper[i]);
                variable.curr = initial[i];
            }

            for(size_t row = 0; row < slacks.size(); ++row)
            {
                if(slacks[row] < 0)
                    continue;
                auto& slack = getVariable(slacks[row]);
                double value = problem->numericConstraints[row]->calculateFunctionValue(initial) - offsets[row];
                slack.curr = std::clamp(std::isfinite(value) ? value : 0.0, slack.lower, slack.upper);
            }
        }

        int FDEvalIni(
            const double x[], const int rowlist[], int mode, int listsize, int, int, int* errors, int) override
        {
            *errors = 0;
            batchReady = false;
            if(mode != 2 && mode != 3)
                return 0;
            try
            {
                VectorInteger requested;

                for(int k = 0; k < listsize; ++k)
                {
                    int row = rowlist[k];
                    if(row >= 0 && row < static_cast<int>(slacks.size())
                        && problem->numericConstraints[row]->properties.hasNonlinearExpression)
                        requested.push_back(row);
                }

                std::sort(requested.begin(), requested.end());
                requested.erase(std::unique(requested.begin(), requested.end()), requested.end());

                if(requested.size() < 2)
                    return 0;

                if(requested != batchRows)
                {
                    batchRows = requested;
                    size_t nonzeros = 0;

                    for(int row : batchRows)
                        nonzeros += std::static_pointer_cast<NonlinearConstraint>(problem->numericConstraints[row])
                                        ->nonlinearGradientSparsityPattern.nnz();
                    expressionJacobianPattern.resize(
                        problem->ADFunctions.Range(), problem->nonlinearExpressionVariables.size(), nonzeros);
                    expressionConstraintRows.assign(problem->ADFunctions.Range(), -1);
                    size_t entry = 0;

                    for(int row : batchRows)
                    {
                        auto constraint
                            = std::static_pointer_cast<NonlinearConstraint>(problem->numericConstraints[row]);
                        expressionConstraintRows[constraint->nonlinearExpressionIndex] = row;

                        for(size_t col : constraint->nonlinearGradientSparsityPattern.col())
                            expressionJacobianPattern.set(entry++, constraint->nonlinearExpressionIndex, col);
                    }
                }
                batchPoint.resize(problem->nonlinearExpressionVariables.size());

                for(const auto& variable : problem->nonlinearExpressionVariables)
                    batchPoint[variable->properties.nonlinearVariableIndex] = x[variable->getIndex()];
                CppAD::sparse_rcv<std::vector<size_t>, std::vector<double>> subset(expressionJacobianPattern);
                problem->calculateNonlinearJacobian(batchPoint, subset);

                for(int row : batchRows)
                    batchGradients[row].clear();

                for(size_t k = 0; k < subset.nnz(); ++k)
                {
                    const int row = expressionConstraintRows.at(subset.row()[k]);
                    if(row < 0)
                        throw std::runtime_error("Missing batched CONOPT Jacobian row");
                    batchGradients[row].emplace_back(
                        problem->nonlinearExpressionVariables[subset.col()[k]], subset.val()[k]);
                }
                batchReady = true;
            }
            catch(...)
            {
                batchRows.clear();
                // Let individual row evaluation handle domain errors; a batch must not turn
                // one failing row into an error for every requested constraint.
                batchReady = false;
            }
            return 0;
        }

        int FDEvalEnd(int, int* errors) override
        {
            *errors = 0;
            batchReady = false;
            return 0;
        }

        int FDEval(const double x[], double* value, double jac[], int row, const int jacnum[], int mode, int,
            int* errors, int, int numjac, int) override
        {
            *errors = 0;
            try
            {
                // Use the callback point directly: linear variables can change between batches.
                evaluationPoint.assign(x, x + n);
                const auto& point = evaluationPoint;
                bool objective = row == static_cast<int>(slacks.size());
                if(mode == 1 || mode == 3)
                {
                    *value = objective
                        ? problem->objectiveFunction->calculateValue(point) - objectiveOffset
                        : problem->numericConstraints.at(row)->calculateFunctionValue(point) - offsets.at(row);
                    if(!objective && slacks.at(row) >= 0)
                        *value -= x[slacks[row]];
                    if(!std::isfinite(*value))
                        *errors = 1;
                }
                if(mode == 2 || mode == 3)
                {
                    bool useBatch
                        = !objective && batchReady && std::binary_search(batchRows.begin(), batchRows.end(), row);
                    if(useBatch)

                        for(const auto& variable : problem->nonlinearExpressionVariables)
                            if(x[variable->getIndex()] != batchPoint[variable->properties.nonlinearVariableIndex])
                            {
                                useBatch = false;
                                break;
                            }
                    SparseVariableVector gradient;
                    if(useBatch)
                    {
                        gradient = std::static_pointer_cast<NonlinearConstraint>(problem->numericConstraints.at(row))
                                       ->calculateGradientWithoutNonlinearExpression(point, false);

                        for(const auto& entry : batchGradients[row])
                            gradient[entry.first] += entry.second;
                    }
                    else
                        gradient = objective ? problem->objectiveFunction->calculateGradient(point, false)
                                             : problem->numericConstraints.at(row)->calculateGradient(point, false);

                    for(int k = 0; k < numjac; ++k)
                    {
                        int index = jacnum[k];
                        jac[index] = gradient[problem->allVariables.at(index)];
                        if(!std::isfinite(jac[index]))
                            *errors = 1;
                    }
                }
            }
            catch(...)
            {
                *errors = 1;
            }
            return 0;
        }

        int SDLagrVal(const double x[], const double u[], const int[], const int[], double values[], int* nodrv, int,
            int, int count) override
        {
            *nodrv = 0;
            std::fill(values, values + count, 0.0);
            try
            {
                VectorDouble point(x, x + n);

                for(size_t row = 0; row <= slacks.size(); ++row)
                {
                    if(u[row] == 0.0)
                        continue;
                    if(row < slacks.size()
                        && problem->numericConstraints[row]->properties.classification
                            == E_ConstraintClassification::Linear)
                        continue;
                    auto constantHessian = row == slacks.size()
                        ? problem->objectiveFunction->getConstantHessian()
                        : problem->numericConstraints[row]->getConstantHessian();
                    SparseVariableMatrix calculatedHessian;
                    if(!constantHessian)
                        calculatedHessian = row == slacks.size()
                            ? problem->objectiveFunction->calculateHessian(point, false)
                            : (expressionHessianPattern.nnz() > 0
                                          && problem->numericConstraints[row]->properties.hasNonlinearExpression
                                      ? std::static_pointer_cast<NonlinearConstraint>(problem->numericConstraints[row])
                                            ->calculateHessianWithoutNonlinearExpression(point, false)
                                      : problem->numericConstraints[row]->calculateHessian(point, false));

                    for(const auto& entry : (constantHessian ? *constantHessian : calculatedHessian))
                    {
                        int i = entry.first.first->getIndex(), j = entry.first.second->getIndex();
                        auto key = std::make_pair(std::min(i, j), std::max(i, j));
                        values[hessianPositions.at(key)] += u[row] * entry.second;
                    }
                }
                if(expressionHessianPattern.nnz() > 0)
                {
                    VectorDouble weights(problem->ADFunctions.Range(), 0.0);
                    bool anyWeight = false;

                    for(size_t row = 0; row < slacks.size(); ++row)
                    {
                        const auto& constraint = problem->numericConstraints[row];
                        if(constraint->properties.hasNonlinearExpression)
                        {
                            weights[std::static_pointer_cast<NonlinearConstraint>(constraint)->nonlinearExpressionIndex]
                                = u[row];
                            anyWeight |= u[row] != 0.0;
                        }
                    }
                    if(anyWeight)
                    {
                        VectorDouble nonlinearPoint(problem->nonlinearExpressionVariables.size());

                        for(const auto& variable : problem->nonlinearExpressionVariables)
                            nonlinearPoint[variable->properties.nonlinearVariableIndex] = point[variable->getIndex()];
                        CppAD::sparse_rcv<std::vector<size_t>, std::vector<double>> subset(expressionHessianPattern);
                        problem->ADFunctions.sparse_hes(nonlinearPoint, weights, subset, expressionHessianPattern,
                            "cppad.symmetric", expressionHessianWork);

                        for(size_t k = 0; k < subset.nnz(); ++k)
                        {
                            int i = problem->nonlinearExpressionVariables[subset.row()[k]]->getIndex();
                            int j = problem->nonlinearExpressionVariables[subset.col()[k]]->getIndex();
                            if(i <= j)
                                values[hessianPositions.at({ i, j })] += subset.val()[k];
                        }
                    }
                }

                for(int i = 0; i < count; ++i)
                    if(!std::isfinite(values[i]))
                        *nodrv = 1;
            }
            catch(...)
            {
                *nodrv = 1;
            }
            return 0;
        }

    private:
        ProblemPtr problem;
        size_t n;
        VectorInteger slacks;
        VectorDouble offsets;
        double objectiveOffset = 0.0;
        std::map<std::pair<int, int>, int> hessianPositions;
        VectorDouble evaluationPoint, batchPoint;
        VectorInteger batchRows, expressionConstraintRows;
        std::vector<std::vector<std::pair<VariablePtr, double>>> batchGradients;
        bool batchReady = false;
        CppAD::sparse_rc<std::vector<size_t>> expressionJacobianPattern, expressionHessianPattern;
        CppAD::sparse_hes_work expressionHessianWork;
    };

    class ConoptOutput : public ConoptMessageHandler
    {
    public:
        ConoptOutput(EnvironmentPtr env, double seconds)
            : env(env), seconds(seconds), start(std::chrono::steady_clock::now())
        {
        }
        int message(int screen, int, int, const std::vector<std::string>& lines) override
        {
            for(size_t i = 0; i < lines.size(); ++i)
            {
                if(static_cast<int>(i) < screen && env->settings->getSetting<bool>("Output.Console.PrimalSolver.Show"))
                    env->output->outputInfo("      | " + lines[i]);
                else
                    env->output->outputTrace("      | " + lines[i]);
            }
            return 0;
        }
        int errorMessage(int row, int col, int, const std::string& text) override
        {
            env->output->outputDebug(fmt::format(" CONOPT row {}, column {}: {}", row, col, text));
            return 0;
        }
        int progress(const ConoptAlgProgress&) override
        {
            timedOut = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= seconds;
            try
            {
                return timedOut || userTerminated(env);
            }
            catch(...)
            {
                return 1;
            } // Never unwind a user callback through CONOPT's C/Fortran stack.
        }
        bool timedOut = false;

    private:
        EnvironmentPtr env;
        double seconds;
        std::chrono::steady_clock::time_point start;
    };

    // The C++ wrapper has no option setter. Register a private option file through its
    // public control vector; do not replace the wrapper's own callback user memory.
    class OptionFile
    {
    public:
        OptionFile()
        {
            char path[] = "/tmp/shot-conopt-XXXXXX";
            int fd = mkstemp(path);
            if(fd < 0)
                throw std::runtime_error("Cannot create CONOPT option file");
            close(fd);
            name = path;
        }
        ~OptionFile() { std::remove(name.c_str()); }
        std::string name;
    };
} // namespace

struct NLPSolverConopt::ModelCache
{
    std::shared_ptr<ConoptProblem> model;
};

NLPSolverConopt::NLPSolverConopt(EnvironmentPtr envPtr, ProblemPtr source)
    : INLPSolver(envPtr)
    , sourceProblem(source)
    , lowerBounds(source->getVariableLowerBounds())
    , upperBounds(source->getVariableUpperBounds())
{
}

void NLPSolverConopt::setStartingPoint(VectorInteger indexes, VectorDouble values)
{
    if(indexes.size() != values.size())
        throw std::invalid_argument("Starting point sizes differ");
    start.assign(lowerBounds.size(), 0.0);

    for(size_t i = 0; i < indexes.size(); ++i)
        start.at(indexes[i]) = values[i];
}
void NLPSolverConopt::clearStartingPoint() { start.clear(); }
void NLPSolverConopt::fixVariables(VectorInteger indexes, VectorDouble values)
{
    unfixVariables();
    lowerBeforeFix = lowerBounds;
    upperBeforeFix = upperBounds;
    invalidFix = indexes.size() != values.size();
    if(invalidFix)
        return;

    for(size_t i = 0; i < indexes.size(); ++i)
    {
        int index = indexes[i];
        if(index < 0 || static_cast<size_t>(index) >= lowerBounds.size() || !std::isfinite(values[i]))
        {
            invalidFix = true;
            continue;
        }
        double value = std::round(values[i]);
        if(value == 0.0)
            value = 0.0;
        if(value < lowerBounds[index] || value > upperBounds[index])
        {
            invalidFix = true;
            continue;
        }
        lowerBounds[index] = upperBounds[index] = value;
    }
}
void NLPSolverConopt::unfixVariables()
{
    if(!lowerBeforeFix.empty())
    {
        lowerBounds.swap(lowerBeforeFix);
        upperBounds.swap(upperBeforeFix);
        lowerBeforeFix.clear();
        upperBeforeFix.clear();
    }
    invalidFix = false;
}
void NLPSolverConopt::updateVariableLowerBound(int i, double bound) { lowerBounds.at(i) = bound; }
void NLPSolverConopt::updateVariableUpperBound(int i, double bound) { upperBounds.at(i) = bound; }
double NLPSolverConopt::getObjectiveValue()
{
    return solution.empty() ? std::numeric_limits<double>::quiet_NaN()
                            : sourceProblem->objectiveFunction->calculateValue(solution);
}
std::string NLPSolverConopt::getSolverDescription()
{
    auto version = Conopt::version();
    return fmt::format("CONOPT {}.{}.{}", version[0], version[1], version[2]);
}
void NLPSolverConopt::saveProblemToFile(std::string filename)
{
    std::ofstream file(filename);
    file.precision(17);
    file << sourceProblem;

    for(size_t i = 0; i < lowerBounds.size(); ++i)
        file << "\nNLP bounds " << i << ": " << lowerBounds[i] << " " << upperBounds[i];
}
void NLPSolverConopt::saveOptionsToFile(std::string filename)
{
    std::ofstream file(filename);
    // Keep the exponent within CONOPT's option reader numeric field width.
    file << std::scientific << std::uppercase << std::setprecision(8);
    file << "Rtnwma " << env->settings->getSetting<double>("Subsolver.Conopt.FeasibilityTolerance") << '\n'
         << "Rtredg " << env->settings->getSetting<double>("Subsolver.Conopt.OptimalityTolerance") << '\n';
    if(!file)
        throw std::runtime_error("Cannot write CONOPT options");
}

E_NLPSolutionStatus NLPSolverConopt::solveProblemInstance()
{
    solution.clear();
    if(invalidFix)
        return E_NLPSolutionStatus::Infeasible;
    try
    {
        if(userTerminated(env))
            return E_NLPSolutionStatus::Error;
        VectorDouble initial = start.empty() ? VectorDouble(lowerBounds.size(), 0.0) : start;

        for(size_t i = 0; i < initial.size(); ++i)
        {
            if(lowerBounds[i] > upperBounds[i])
                return E_NLPSolutionStatus::Infeasible;
            if(!std::isfinite(initial[i]))
                initial[i] = 0.0;
            initial[i] = std::clamp(initial[i], lowerBounds[i], upperBounds[i]);
        }
        double seconds = std::min(env->settings->getSetting<double>("Primal.FixedInteger.TimeLimit"),
            env->settings->getSetting<double>("Termination.TimeLimit") - env->timing->getElapsedTime("Total"));
        if(seconds <= 0.0)
            return E_NLPSolutionStatus::TimeLimit;
        if(!modelCache)
            modelCache = std::make_shared<ModelCache>();
        if(!modelCache->model)
            modelCache->model = std::make_shared<ConoptProblem>(sourceProblem, lowerBounds, upperBounds, initial);
        else
            modelCache->model->update(lowerBounds, upperBounds, initial);
        ConoptOutput output(env, seconds);
        OptionFile options;
        saveOptionsToFile(options.name);
        Conopt solver;
        solver.setMessageHandler(output);
        solver.loadModel(*modelCache->model);
        auto check = [](int rc)
        {
            if(rc)
                throw std::runtime_error("CONOPT setup error " + std::to_string(rc));
        };
        check(solver.fVincLin(1));
        check(solver.allowEmptyRow(1));
        check(solver.allowEmptyCol(1));
        check(solver.setThreadS(1));
        check(solver.setThreadF(1));
        check(solver.setThread2D(1));
        check(solver.setErrLim(100));
        check(solver.setItLim(env->settings->getSetting<int>("Primal.FixedInteger.IterationLimit")));
        check(solver.setResLim(seconds));
        check(COIDEF_Optfile(solver.controlVector(), options.name.c_str()));
        const char* license[] = { std::getenv("CONOPT_LICENSE_INT_1"), std::getenv("CONOPT_LICENSE_INT_2"),
            std::getenv("CONOPT_LICENSE_INT_3"), std::getenv("CONOPT_LICENSE_TEXT") };
        int count = 0;

        for(auto value : license)
            if(value)
                ++count;
        if(count == 4)
        {
            int integers[3];
            try
            {
                for(int i = 0; i < 3; ++i)
                {
                    size_t used;
                    integers[i] = std::stoi(license[i], &used);
                    if(license[i][used] != '\0')
                        throw std::runtime_error("Invalid license integer");
                }
            }
            catch(const std::exception&)
            {
                if(!warnedLicense)
                    env->output->outputWarning(" Invalid CONOPT license integer environment variables.");
                warnedLicense = true;
                return E_NLPSolutionStatus::Error;
            }
            int licenseCode = solver.setLicense(integers[0], integers[1], integers[2], license[3]);
            if(licenseCode && !warnedLicense)
            {
                env->output->outputWarning(" CONOPT rejected the configured license.");
                warnedLicense = true;
            }
            check(licenseCode);
        }
        else if(count && !warnedIncompleteLicense)
        {
            env->output->outputWarning(" Incomplete CONOPT license environment; using demo mode.");
            warnedIncompleteLicense = true;
        }
        else if(!count)
            env->output->outputDebug(" CONOPT demo mode.");
        int rc = solver.solve();
        if(rc)
        {
            if((rc == 3 || rc == 110) && !warnedLicense)
            {
                env->output->outputWarning(rc == 110
                        ? " CONOPT demo size limit exceeded (NLP: 1000 variables and rows, including added slacks and "
                          "objective)."
                        : " CONOPT license error. Check CONOPT_LICENSE_INT_1/2/3 and CONOPT_LICENSE_TEXT.");
                warnedLicense = true;
            }
            env->output->outputDebug(fmt::format(" CONOPT returned error {}.", rc));
            return E_NLPSolutionStatus::Error;
        }
        int modelStatus = solver.modelStatus(), status = solver.solutionStatus();
        env->output->outputDebug(fmt::format(" CONOPT model status {}, solver status {}.", modelStatus, status));
        const auto& point = solver.getVariableValues();
        if(point.size() >= lowerBounds.size()
            && std::all_of(point.begin(), point.end(), [](double v) { return std::isfinite(v); }))
            solution.assign(point.begin(), point.begin() + lowerBounds.size());
        bool feasible = !solution.empty() && std::isfinite(getObjectiveValue());
        double boundTol = env->settings->getSetting<double>("Primal.Tolerance.LinearConstraint");

        for(size_t i = 0; feasible && i < solution.size(); ++i)
            feasible = solution[i] >= lowerBounds[i] - boundTol && solution[i] <= upperBounds[i] + boundTol;

        for(const auto& constraint : sourceProblem->numericConstraints)
        {
            if(!feasible)
                break;
            double value = constraint->calculateFunctionValue(solution);
            double tolerance = env->settings->getSetting<double>(
                constraint->properties.classification == E_ConstraintClassification::Linear
                    ? "Primal.Tolerance.LinearConstraint"
                    : "Primal.Tolerance.NonlinearConstraint");
            feasible = std::isfinite(value) && value >= constraint->valueLHS - tolerance
                && value <= constraint->valueRHS + tolerance;
        }
        // SHOT submits only Optimal/Feasible points; preserve the termination reason in diagnostics.
        if(feasible)
            return status == 1 && (modelStatus == 1 || modelStatus == 2) ? E_NLPSolutionStatus::Optimal
                                                                         : E_NLPSolutionStatus::Feasible;
        // Do not expose an unvalidated iterate through the solution accessors.
        solution.clear();
        if(status == 2)
            return E_NLPSolutionStatus::IterationLimit;
        if(status == 3 || output.timedOut)
            return E_NLPSolutionStatus::TimeLimit;
        // A solver error or user interruption is not an infeasibility proof,
        // even when the last model status describes a locally infeasible point.
        if(status >= 6 && status != 15)
            return E_NLPSolutionStatus::Error;
        if(modelStatus == 4 || modelStatus == 5)
            return E_NLPSolutionStatus::Infeasible;
        if(modelStatus == 3)
            return E_NLPSolutionStatus::Unbounded;
        return E_NLPSolutionStatus::Error;
    }
    catch(const std::exception& error)
    {
        solution.clear();
        env->output->outputDebug(std::string(" CONOPT: ") + error.what());
        return E_NLPSolutionStatus::Error;
    }
}
} // namespace SHOT
