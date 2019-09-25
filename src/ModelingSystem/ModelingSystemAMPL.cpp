/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ModelingSystemAMPL.h"

#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"

#include "../Model/NonlinearExpressions.h"
#include "../Model/Problem.h"
#include "../Model/Simplifications.h"
#include "../Model/Variables.h"

#include "mp/nl.h"
#include "mp/problem.h"
#include "mp/nl-reader.h"
#include "mp/sol.h"

#include <cstdlib>
#include <cstring>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

#include <memory>
#include <stdexcept>
#include <string>

namespace SHOT
{

using MPProblem = mp::Problem;
using MPProblemPtr = std::shared_ptr<MPProblem>;

class AMPLProblemHandler : public mp::NullNLHandler<NonlinearExpressionPtr>
{
private:
    EnvironmentPtr env;
    ProblemPtr destination;

    NonlinearExpressions nonlinearExpressions;

    double minLBCont;
    double maxUBCont;
    double minLBInt;
    double maxUBInt;

    void reset() { nonlinearExpressions.clear(); }

public:
    AMPLProblemHandler(EnvironmentPtr envPtr, ProblemPtr problem) : env(envPtr), destination(problem)
    {
        this->minLBCont = env->settings->getSetting<double>("ContinuousVariable.MinimumLowerBound", "Model");
        this->maxUBCont = env->settings->getSetting<double>("ContinuousVariable.MaximumUpperBound", "Model");
        this->minLBInt = env->settings->getSetting<double>("IntegerVariable.MinimumLowerBound", "Model");
        this->maxUBInt = env->settings->getSetting<double>("IntegerVariable.MaximumUpperBound", "Model");
    }

    void OnHeader(const mp::NLHeader& h)
    {
        destination->allVariables.reserve(h.num_vars);
        destination->integerVariables.reserve(h.num_integer_vars());
        destination->realVariables.reserve(h.num_continuous_vars());

        for(int i = 0; i < h.num_continuous_vars(); i++)
        {
            destination->add(std::make_shared<SHOT::Variable>(
                "x_" + std::to_string(i), i, E_VariableType::Real, SHOT_DBL_MIN, SHOT_DBL_MAX));
        }

        for(int i = h.num_continuous_vars(); i < h.num_vars; i++)
        {
            destination->add(std::make_shared<SHOT::Variable>(
                "i_" + std::to_string(i), i, E_VariableType::Integer, SHOT_DBL_MIN, SHOT_DBL_MAX));
        }

        destination->numericConstraints.reserve(h.num_algebraic_cons);
        destination->linearConstraints.reserve(h.num_algebraic_cons - h.num_nl_cons);
        destination->nonlinearConstraints.reserve(h.num_nl_cons);

        for(int i = 0; i < h.num_nl_cons; i++)
        {
            destination->add(
                std::make_shared<NonlinearConstraint>(i, "nlc_" + std::to_string(i), SHOT_DBL_MIN, SHOT_DBL_MAX));
        }

        for(int i = h.num_nl_cons; i < h.num_algebraic_cons; i++)
        {
            destination->add(
                std::make_shared<LinearConstraint>(i, "lc_" + std::to_string(i), SHOT_DBL_MIN, SHOT_DBL_MAX));
        }

        if(h.num_nl_objs == 1)
        {
            destination->add(std::make_shared<NonlinearObjectiveFunction>());
        }
        else if(h.num_nl_objs == 0)
        {
            destination->add(std::make_shared<LinearObjectiveFunction>());
        }
    }

    NonlinearExpressionPtr OnNumber(double value) { return std::make_shared<ExpressionConstant>(value); }

    NonlinearExpressionPtr OnVariableRef(int variableIndex)
    {
        return std::make_shared<ExpressionVariable>(destination->getVariable(variableIndex));
    }

    NonlinearExpressionPtr OnUnary(mp::expr::Kind kind, NonlinearExpressionPtr child)
    {
        switch(kind)
        {

        case mp::expr::MINUS:
            return std::make_shared<ExpressionNegate>(child);

        case mp::expr::ABS:
            return std::make_shared<ExpressionAbs>(child);

        case mp::expr::POW2:
            return std::make_shared<ExpressionSquare>(child);

        case mp::expr::SQRT:
            return std::make_shared<ExpressionSquareRoot>(child);

        case mp::expr::LOG:
            return std::make_shared<ExpressionLog>(child);

        case mp::expr::EXP:
            return std::make_shared<ExpressionExp>(child);

        case mp::expr::SIN:
            return std::make_shared<ExpressionSin>(child);

        case mp::expr::COS:
            return std::make_shared<ExpressionCos>(child);

        case mp::expr::TAN:
            return std::make_shared<ExpressionTan>(child);

        case mp::expr::ASIN:
            return std::make_shared<ExpressionArcSin>(child);

        case mp::expr::ACOS:
            return std::make_shared<ExpressionArcCos>(child);

        case mp::expr::ATAN:
            return std::make_shared<ExpressionArcTan>(child);

        default:
            throw OperationNotImplementedException(fmt::format("Error: Unsupported AMPL function {}", kind));
            break;
        }

        return nullptr;
    }

    NonlinearExpressionPtr OnBinary(
        mp::expr::Kind kind, NonlinearExpressionPtr firstChild, NonlinearExpressionPtr secondChild)
    {
        switch(kind)
        {
        case mp::expr::ADD:
            return std::make_shared<ExpressionSum>(firstChild, secondChild);

        case mp::expr::SUB:
            return std::make_shared<ExpressionSum>(firstChild, std::make_shared<ExpressionNegate>(secondChild));

        case mp::expr::MUL:
            return std::make_shared<ExpressionProduct>(firstChild, secondChild);

        case mp::expr::DIV:
            return std::make_shared<ExpressionDivide>(firstChild, secondChild);

        case mp::expr::POW:
            return std::make_shared<ExpressionPower>(firstChild, secondChild);

        case mp::expr::POW_CONST_BASE:
            return std::make_shared<ExpressionPower>(firstChild, secondChild);

        case mp::expr::POW_CONST_EXP:
            return std::make_shared<ExpressionPower>(firstChild, secondChild);

        default:
            throw OperationNotImplementedException(fmt::format("Error: Unsupported AMPL function {}", kind));
            break;
        }

        return nullptr;
    }

    // Used for creating a list of terms in a sum
    struct NumericArgHandler
    {
        NonlinearExpressions terms;

        void AddArg(NonlinearExpressionPtr term) { terms.add(term); }
    };

    NumericArgHandler BeginSum(int) { return NumericArgHandler(); }

    NonlinearExpressionPtr EndSum(NumericArgHandler handler) { return std::make_shared<ExpressionSum>(handler.terms); }

    void OnObj([[maybe_unused]] int objectiveIndex, mp::obj::Type type, NonlinearExpressionPtr nonlinearExpression)
    {
        if(type == mp::obj::Type::MAX)
            destination->objectiveFunction->direction = E_ObjectiveFunctionDirection::Maximize;
        else
            destination->objectiveFunction->direction = E_ObjectiveFunctionDirection::Minimize;

        if(nonlinearExpression)
        {
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination->objectiveFunction)
                ->add(nonlinearExpression);
        }

        reset();
    }

    void OnAlgebraicCon(int constraintIndex, NonlinearExpressionPtr nonlinearExpression)
    {
        if(nonlinearExpression)
        {
            std::dynamic_pointer_cast<NonlinearConstraint>(destination->numericConstraints[constraintIndex])
                ->add(nonlinearExpression);
        }

        reset();
    }

    void OnVarBounds(int variableIndex, double variableLB, double variableUB)
    {
        switch(destination->allVariables[variableIndex]->properties.type)
        {
        case(E_VariableType::Real):
            if(variableLB < minLBCont)
                variableLB = minLBCont;

            if(variableUB > maxUBCont)
                variableUB = maxUBCont;

            break;

        case(E_VariableType::Binary):

            if(variableLB < 0.0)
                variableLB = 0.0;

            if(variableUB > 1.0)
                variableUB = 1.0;

            break;

        case(E_VariableType::Integer):

            if(variableLB < minLBInt)
                variableLB = minLBInt;

            if(variableUB > maxUBInt)
                variableUB = maxUBInt;

            break;

        default:
            break;
        }

        destination->setVariableBounds(variableIndex, variableLB, variableUB);
    }

    void OnConBounds(int index, double lb, double ub)
    {
        if(lb == -SHOT_DBL_INF)
            lb = SHOT_DBL_MIN;

        if(ub == SHOT_DBL_INF)
            ub = SHOT_DBL_MAX;

        destination->numericConstraints[index]->valueLHS = lb;
        destination->numericConstraints[index]->valueRHS = ub;
    }

    class LinearPartHandler
    {
    private:
        EnvironmentPtr env;
        ProblemPtr destination;

        int constraintIndex;
        bool inObjectiveFunction = false;

    public:
        explicit LinearPartHandler(EnvironmentPtr envPtr, ProblemPtr problem, int constraintIndex)
            : env(envPtr), destination(problem), constraintIndex(constraintIndex)
        {
        }

        explicit LinearPartHandler(EnvironmentPtr envPtr, ProblemPtr problem) : env(envPtr), destination(problem)
        {
            constraintIndex = -1;
            inObjectiveFunction = true;
        }

        void AddTerm(int variableIndex, double coefficient)
        {
            if(coefficient == 0.0)
                return;

            if(inObjectiveFunction)
                std::dynamic_pointer_cast<LinearObjectiveFunction>(destination->objectiveFunction)
                    ->add(std::make_shared<LinearTerm>(coefficient, destination->getVariable(variableIndex)));
            else
                std::dynamic_pointer_cast<LinearConstraint>(destination->numericConstraints[constraintIndex])
                    ->add(std::make_shared<LinearTerm>(coefficient, destination->getVariable(variableIndex)));
        }
    };

    typedef LinearPartHandler LinearObjHandler;

    LinearPartHandler OnLinearObjExpr([[maybe_unused]] int objectiveIndex, [[maybe_unused]] int numLinearTerms)
    {
        return LinearObjHandler(env, destination);
    }

    typedef LinearPartHandler LinearConHandler;

    LinearConHandler OnLinearConExpr(int constraintIndex, [[maybe_unused]] int numLinearTerms)
    {
        return LinearConHandler(env, destination, constraintIndex);
    }
};

ModelingSystemAMPL::ModelingSystemAMPL(EnvironmentPtr envPtr) : IModelingSystem(envPtr) {}

ModelingSystemAMPL::~ModelingSystemAMPL() = default;

void ModelingSystemAMPL::augmentSettings([[maybe_unused]] SettingsPtr settings) {}

void ModelingSystemAMPL::updateSettings([[maybe_unused]] SettingsPtr settings) {}

E_ProblemCreationStatus ModelingSystemAMPL::createProblem(ProblemPtr& problem, const std::string& filename)
{
    if(false && !fs::filesystem::exists(fs::filesystem::path(filename)))
    {
        env->output->outputError("Problem file \"" + filename + "\" does not exist.");

        return (E_ProblemCreationStatus::FileDoesNotExist);
    }

    fs::filesystem::path problemFile(filename);
    fs::filesystem::path problemPath = problemFile.parent_path();

    try
    {
        AMPLProblemHandler handler(env, problem);
        mp::ReadNLFile(filename, handler);
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when reading AMPL model from \"{}\": {}", filename, e.what()));

        return (E_ProblemCreationStatus::Error);
    }

    problem->updateProperties();

    bool extractMonomialTerms = env->settings->getSetting<bool>("Reformulation.Monomials.Extract", "Model");
    bool extractSignomialTerms = env->settings->getSetting<bool>("Reformulation.Signomials.Extract", "Model");
    bool extractQuadraticTerms = env->settings->getSetting<bool>("Reformulation.Quadratics.Extract", "Model");

    simplifyNonlinearExpressions(problem, extractMonomialTerms, extractSignomialTerms, extractQuadraticTerms);

    problem->finalize();

    return (E_ProblemCreationStatus::NormalCompletion);
}

void ModelingSystemAMPL::finalizeSolution() {}

} // Namespace SHOT