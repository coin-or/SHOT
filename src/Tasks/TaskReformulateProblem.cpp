/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskReformulateProblem.h"

namespace SHOT
{

TaskReformulateProblem::TaskReformulateProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("ProblemReformulation");

    auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(env->settings->getIntSetting("Reformulation.Quadratics.Strategy", "Model"));

    bool useQuadraticConstraints = (quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained);
    bool useQuadraticObjective = (useQuadraticConstraints || quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticObjective);
    bool quadraticObjectiveRegardedAsNonlinear = false;

    bool useEpigraphReformulation = env->settings->getBoolSetting("Reformulation.Epigraph.Use", "Model");
    int additionalNumberOfNonlinearConstraints = 0;

    if (useEpigraphReformulation)
    {
        additionalNumberOfNonlinearConstraints = 1;
    }

    int auxVarCounter = 0;
    int auxConstrCounter = 0;

    auto newProblem = std::make_shared<Problem>(env);
    newProblem->name = env->problem->name + " (reformulated)";

    newProblem->allVariables.reserve(env->problem->allVariables.size());
    newProblem->realVariables.reserve(env->problem->realVariables.size());
    newProblem->binaryVariables.reserve(env->problem->binaryVariables.size());
    newProblem->integerVariables.reserve(env->problem->integerVariables.size());
    newProblem->semicontinuousVariables.reserve(env->problem->semicontinuousVariables.size());
    newProblem->nonlinearVariables.reserve(env->problem->nonlinearVariables.size());

    newProblem->variableLowerBounds = env->problem->variableLowerBounds;
    newProblem->variableUpperBounds = env->problem->variableUpperBounds;

    newProblem->numericConstraints.reserve(env->problem->numericConstraints.size());
    newProblem->linearConstraints.reserve(env->problem->linearConstraints.size());

    if (useQuadraticConstraints)
    {
        newProblem->quadraticConstraints.reserve(env->problem->quadraticConstraints.size());
        newProblem->nonlinearConstraints.reserve(env->problem->nonlinearConstraints.size() + additionalNumberOfNonlinearConstraints);
    }
    else
    {
        newProblem->nonlinearConstraints.reserve(env->problem->nonlinearConstraints.size() + env->problem->quadraticConstraints.size() + additionalNumberOfNonlinearConstraints);
    }

    newProblem->factorableFunctionVariables.reserve(env->problem->factorableFunctionVariables.size());
    newProblem->factorableFunctions.reserve(env->problem->factorableFunctions.size());

    // Copying variables
    for (auto &V : env->problem->allVariables)
    {
        auto variable = std::make_shared<Variable>(V->name, V->index, V->type, V->lowerBound, V->upperBound);
        newProblem->add(std::move(variable));
    }

    // Copying constraints
    for (auto &C : env->problem->numericConstraints)
    {
        double valueLHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueLHS;
        double valueRHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueRHS;
        double constant = std::dynamic_pointer_cast<NumericConstraint>(C)->constant;

        if (C->properties.classification == E_ConstraintClassification::Linear)
        {
            LinearConstraintPtr constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);

            for (auto &T : std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms.terms)
            {
                auto variable = newProblem->getVariable(T->variable->index);

                constraint->add(std::make_shared<LinearTerm>(T->coefficient, variable));
            }

            constraint->constant = constant;
            newProblem->add(std::move(constraint));
        }
        else if (useQuadraticConstraints && C->properties.classification == E_ConstraintClassification::Quadratic)
        {
            QuadraticConstraintPtr constraint = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);

            for (auto &T : std::dynamic_pointer_cast<QuadraticConstraint>(C)->linearTerms.terms)
            {
                auto variable = newProblem->getVariable(T->variable->index);

                constraint->add(std::make_shared<LinearTerm>(T->coefficient, variable));
            }

            for (auto &T : std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms.terms)
            {
                auto firstVariable = newProblem->getVariable(T->firstVariable->index);
                auto secondVariable = newProblem->getVariable(T->secondVariable->index);

                constraint->add(std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
            }

            constraint->constant = constant;
            newProblem->add(std::move(constraint));
        }
        else if (!useQuadraticConstraints || C->properties.classification == E_ConstraintClassification::Nonlinear)
        {
            double signfactor = 1.0;

            if (valueLHS > SHOT_DBL_MIN && valueRHS == SHOT_DBL_MAX)
            {
                // Constraint is of type l <= g(x). Rewrite as -g(x) <= -l

                signfactor = -1;
                valueRHS = -valueLHS;
                valueLHS = SHOT_DBL_MIN;
            }
            else if (valueLHS != SHOT_DBL_MIN && valueRHS != SHOT_DBL_MAX)
            {
                // Constraint is of type l <= g(x) <= u. Rewrite as -g(x) <= -l and g(x) <= u
                // TODO
            }

            NumericConstraintPtr constraint;

            if (C->properties.hasNonlinearExpression && std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression->getType() != E_NonlinearExpressionTypes::Sum)
            {
                constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
            }
            else
            {
                constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
            }

            for (auto &T : std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms.terms)
            {
                auto variable = newProblem->getVariable(T->variable->index);
                std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(std::make_shared<LinearTerm>(signfactor * T->coefficient, variable));
            }

            if (C->properties.hasQuadraticTerms)
            {
                for (auto &T : std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms.terms)
                {
                    auto firstVariable = newProblem->getVariable(T->firstVariable->index);
                    auto secondVariable = newProblem->getVariable(T->secondVariable->index);

                    auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_aux_" + std::to_string(auxVarCounter), newProblem->allVariables.size(), E_VariableType::Real, -100000000.0, 100000000.0);

                    std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(std::make_shared<LinearTerm>(1.0, auxVariable));

                    auto auxConstraint = std::make_shared<NonlinearConstraint>(newProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 0.0);
                    auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

                    if (signfactor == -1)
                    {
                        auxConstraint->add(std::make_shared<QuadraticTerm>(-1.0, firstVariable, secondVariable));
                        auxVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(-1.0 * T->coefficient, firstVariable, secondVariable));
                    }
                    else
                    {
                        auxConstraint->add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));
                        auxVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(1.0 * T->coefficient, firstVariable, secondVariable));
                    }

                    if (firstVariable == secondVariable && signfactor * T->coefficient > 0.0)
                    {
                        auxVariable->lowerBound = T->coefficient * firstVariable->lowerBound * firstVariable->lowerBound;
                    }

                    newProblem->add(std::move(auxVariable));
                    newProblem->add(std::move(auxConstraint));
                    auxVarCounter++;
                    auxConstrCounter++;
                }
            }

            if (C->properties.hasNonlinearExpression && std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
            {
                auto sum = std::dynamic_pointer_cast<ExpressionSum>(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression);

                for (auto &T : sum->children.expressions)
                {
                    auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_aux_" + std::to_string(auxVarCounter), newProblem->allVariables.size(), E_VariableType::Real, -9999999999.0, 9999999999.0);

                    std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(std::make_shared<LinearTerm>(1.0, auxVariable));

                    auto auxConstraint = std::make_shared<NonlinearConstraint>(newProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 0.0);

                    auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

                    if (signfactor == -1)
                    {
                        auxConstraint->add(std::make_shared<ExpressionNegate>(copyNonlinearExpression(T.get(), newProblem)));
                    }
                    else
                    {
                        auxConstraint->add(copyNonlinearExpression(T.get(), newProblem));
                    }

                    auxVariable->nonlinearExpression = auxConstraint->nonlinearExpression;

                    /*if (auxVariable->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Square)
                    {
                        auto child = std::dynamic_pointer_cast<ExpressionSquare>(auxVariable->nonlinearExpression)->child;

                        if (child->getType() == E_NonlinearExpressionTypes::Variable)
                        {
                            auto variable = std::dynamic_pointer_cast<ExpressionVariable>(child);
                            auxVariable->lowerBound = variable->variable->lowerBound * variable->variable->lowerBound;
                            auxVariable->upperBound = variable->variable->upperBound * variable->variable->upperBound;
                        }
                        else
                        {
                            auxVariable->lowerBound = 0.0;
                        }
                    }
                    else if (auxVariable->nonlinearExpression->getType() == E_NonlinearExpressionTypes::SquareRoot)
                    {
                        auxVariable->lowerBound = 0.0;
                    }
                    else if (auxVariable->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Negate)
                    {
                        auto child = std::dynamic_pointer_cast<ExpressionNegate>(auxVariable->nonlinearExpression)->child;

                        if (child->getType() == E_NonlinearExpressionTypes::Square)
                        {
                            auxVariable->upperBound = 0.0;
                        }
                        else if (child->getType() == E_NonlinearExpressionTypes::SquareRoot)
                        {
                            auxVariable->upperBound = 0.0;
                        }
                    }*/

                    newProblem->add(std::move(auxVariable));
                    newProblem->add(std::move(auxConstraint));
                    auxVarCounter++;
                    auxConstrCounter++;
                }
            }

            constraint->constant = signfactor * constant;

            if (C->properties.hasNonlinearExpression && std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression->getType() != E_NonlinearExpressionTypes::Sum)
            {

                if (signfactor == -1)
                {
                    std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(std::make_shared<ExpressionNegate>(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), newProblem)));
                }
                else
                {
                    std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), newProblem));
                }

                newProblem->add(std::move(std::dynamic_pointer_cast<NonlinearConstraint>(constraint)));
            }
            else
            {
                newProblem->add(std::move(std::dynamic_pointer_cast<LinearConstraint>(constraint)));
            }
        }
        else
        {
            env->output->outputAlways("Could not copy constraint with index" + std::to_string(C->index));
        }
    }

    // Copy objective function
    if (env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Linear)
    {
        auto objective = std::make_shared<LinearObjectiveFunction>();

        for (auto &T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms.terms)
        {
            auto variable = newProblem->getVariable(T->variable->index);
            objective->add(std::make_shared<LinearTerm>(T->coefficient, variable));
        }

        objective->direction = env->problem->objectiveFunction->direction;
        newProblem->add(objective);

        newProblem->objectiveFunction->constant = env->problem->objectiveFunction->constant;
    }
    else if (useEpigraphReformulation)
    {
        // Adding new linear objective function
        auto objective = std::make_shared<LinearObjectiveFunction>();
        objective->direction = E_ObjectiveFunctionDirection::Minimize;
        objective->constant = 0.0;
        double objVarBound = env->settings->getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");
        auto objectiveVariable = std::make_shared<AuxilliaryVariable>("shot_objvar", newProblem->allVariables.size(), E_VariableType::Real, -objVarBound, objVarBound);
        objective->add(std::make_shared<LinearTerm>(1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));

        // Adding the auxilliary objective constraint
        double signfactor = (env->problem->objectiveFunction->properties.isMinimize) ? 1.0 : -1.0;
        auto constraint = std::make_shared<NonlinearConstraint>(newProblem->numericConstraints.size(), "shot_objconstr", SHOT_DBL_MIN, -1.0 * signfactor * env->problem->objectiveFunction->constant);

        for (auto &T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms.terms)
        {
            auto variable = newProblem->getVariable(T->variable->index);
            constraint->add(std::make_shared<LinearTerm>(T->coefficient, variable));
        }

        for (auto &T : constraint->linearTerms.terms)
        {
            objectiveVariable->linearTerms.add(T);
        }

        constraint->add(std::make_shared<LinearTerm>(-1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));

        for (auto &T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms.terms)
        {
            auto firstVariable = newProblem->getVariable(T->firstVariable->index);
            auto secondVariable = newProblem->getVariable(T->secondVariable->index);

            constraint->add(std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
        }

        for (auto &T : constraint->quadraticTerms.terms)
        {
            objectiveVariable->quadraticTerms.add(T);
        }

        if (env->problem->objectiveFunction->properties.hasNonlinearExpression)
        {
            if (signfactor == -1)
            {
                constraint->add(std::make_shared<ExpressionNegate>(
                    copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression.get(), newProblem)));
            }
            else
            {
                constraint->add(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression.get(), newProblem));
            }

            objectiveVariable->nonlinearExpression = constraint->nonlinearExpression;
        }

        newProblem->add(std::move(objectiveVariable));
        newProblem->add(std::move(objective));
        newProblem->add(std::move(constraint));
    }
    else if (useQuadraticObjective && env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic)
    {
        auto objective = std::make_shared<QuadraticObjectiveFunction>();

        for (auto &T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms.terms)
        {
            auto variable = newProblem->getVariable(T->variable->index);
            objective->add(std::make_shared<LinearTerm>(T->coefficient, variable));
        }

        for (auto &T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms.terms)
        {
            auto firstVariable = newProblem->getVariable(T->firstVariable->index);
            auto secondVariable = newProblem->getVariable(T->secondVariable->index);

            objective->add(std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
        }

        objective->direction = env->problem->objectiveFunction->direction;
        newProblem->add(objective);

        newProblem->objectiveFunction->constant = env->problem->objectiveFunction->constant;
    }
    else if (env->problem->objectiveFunction->properties.hasNonlinearExpression)
    {
        double signfactor = (env->problem->objectiveFunction->properties.isMinimize) ? 1.0 : -1.0;
        double objVarBound = env->settings->getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");
        ObjectiveFunctionPtr objective;

        if (env->problem->objectiveFunction->properties.hasQuadraticTerms)
        {
            objective = std::make_shared<QuadraticObjectiveFunction>();

            for (auto &T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms.terms)
            {
                auto firstVariable = newProblem->getVariable(T->firstVariable->index);
                auto secondVariable = newProblem->getVariable(T->secondVariable->index);

                std::dynamic_pointer_cast<QuadraticObjectiveFunction>(objective)->add(std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
            }
        }
        else
        {
            objective = std::make_shared<LinearObjectiveFunction>();
        }

        for (auto &T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms.terms)
        {
            auto variable = newProblem->getVariable(T->variable->index);
            std::dynamic_pointer_cast<LinearObjectiveFunction>(objective)->add(std::make_shared<LinearTerm>(T->coefficient, variable));
        }

        if (std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto sum = std::dynamic_pointer_cast<ExpressionSum>(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression);

            for (auto &T : sum->children.expressions)
            {
                auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_aux_" + std::to_string(auxVarCounter), newProblem->allVariables.size(), E_VariableType::Real, -objVarBound, objVarBound);

                std::dynamic_pointer_cast<LinearObjectiveFunction>(objective)->add(std::make_shared<LinearTerm>(1.0, auxVariable));

                auto constraint = std::make_shared<NonlinearConstraint>(newProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxVarCounter), SHOT_DBL_MIN, 0.0);

                constraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

                if (signfactor == -1)
                {
                    constraint->add(std::make_shared<ExpressionNegate>(copyNonlinearExpression(T.get(), newProblem)));
                }
                else
                {
                    constraint->add(copyNonlinearExpression(T.get(), newProblem));
                }

                auxVariable->nonlinearExpression = constraint->nonlinearExpression;

                newProblem->add(std::move(auxVariable));
                newProblem->add(std::move(constraint));
                auxVarCounter++;
            }
        }

        objective->constant = env->problem->objectiveFunction->constant;
        objective->direction = env->problem->objectiveFunction->direction;
        newProblem->add(std::move(objective));
    }
    else if (!useQuadraticObjective || env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Nonlinear)
    {
        auto objective = std::make_shared<NonlinearObjectiveFunction>();

        for (auto &T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms.terms)
        {
            auto variable = newProblem->getVariable(T->variable->index);
            objective->add(std::make_shared<LinearTerm>(T->coefficient, variable));
        }

        for (auto &T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms.terms)
        {
            auto firstVariable = newProblem->getVariable(T->firstVariable->index);
            auto secondVariable = newProblem->getVariable(T->secondVariable->index);

            objective->add(std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
        }

        if (env->problem->objectiveFunction->properties.hasNonlinearExpression)
        {
            objective->add(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression.get(), newProblem));
        }
        else
        {
            quadraticObjectiveRegardedAsNonlinear = true;
        }

        objective->direction = env->problem->objectiveFunction->direction;
        newProblem->add(objective);

        newProblem->objectiveFunction->constant = env->problem->objectiveFunction->constant;
    }

    newProblem->finalize();

    // Fixing that a quadratic objective changed into a nonlinear constraint is correctly identified
    if (quadraticObjectiveRegardedAsNonlinear && newProblem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic)
    {
        newProblem->objectiveFunction->properties.classification = E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear;
        newProblem->properties.isMIQPProblem = false;
        newProblem->properties.isMINLPProblem = true;
    }

    env->reformulatedProblem = newProblem;

    std::cout << env->problem << std::endl;

    std::cout << env->reformulatedProblem << std::endl;

    env->timing->stopTimer("ProblemReformulation");
} // namespace SHOT

TaskReformulateProblem::~TaskReformulateProblem()
{
}

void TaskReformulateProblem::run()
{
}

std::string TaskReformulateProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

NonlinearExpressionPtr TaskReformulateProblem::copyNonlinearExpression(NonlinearExpression *expression, const ProblemPtr destination)
{
    unsigned int i;
    std::ostringstream outStr;
    int numChildren;

    switch (expression->getType())
    {
    case E_NonlinearExpressionTypes::Plus:
        return std::make_shared<ExpressionPlus>(copyNonlinearExpression(((ExpressionPlus *)expression)->firstChild.get(), destination),
                                                copyNonlinearExpression(((ExpressionPlus *)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Sum:
        numChildren = ((ExpressionSum *)expression)->getNumberOfChildren();
        switch (numChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return copyNonlinearExpression(((ExpressionSum *)expression)->children.get(0).get(), destination);
        default:
            NonlinearExpressions terms;
            for (i = 0; i < numChildren; i++)
                terms.expressions.push_back(copyNonlinearExpression(((ExpressionSum *)expression)->children.get(i).get(), destination));
            return std::make_shared<ExpressionSum>(terms);
        }

    case E_NonlinearExpressionTypes::Minus:
        return std::make_shared<ExpressionMinus>(copyNonlinearExpression(((ExpressionMinus *)expression)->firstChild.get(), destination),
                                                 copyNonlinearExpression(((ExpressionMinus *)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Negate:
        return std::make_shared<ExpressionNegate>(copyNonlinearExpression(((ExpressionNegate *)expression)->child.get(), destination));

    case E_NonlinearExpressionTypes::Times:
        return std::make_shared<ExpressionTimes>(copyNonlinearExpression(((ExpressionTimes *)expression)->firstChild.get(), destination),
                                                 copyNonlinearExpression(((ExpressionTimes *)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Divide:
        return std::make_shared<ExpressionDivide>(copyNonlinearExpression(((ExpressionDivide *)expression)->firstChild.get(), destination),
                                                  copyNonlinearExpression(((ExpressionDivide *)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Power:
        return std::make_shared<ExpressionPower>(copyNonlinearExpression(((ExpressionPower *)expression)->firstChild.get(), destination),
                                                 copyNonlinearExpression(((ExpressionPower *)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Product:
        numChildren = ((ExpressionProduct *)expression)->getNumberOfChildren();
        switch (numChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return copyNonlinearExpression(((ExpressionProduct *)expression)->children.get(0).get(), destination);
        default:
            NonlinearExpressions factors;
            for (i = 0; i < numChildren; i++)
                factors.expressions.push_back(copyNonlinearExpression(((ExpressionProduct *)expression)->children.get(i).get(), destination));
            return std::make_shared<ExpressionProduct>(factors);
        }

    case E_NonlinearExpressionTypes::Abs:
        return std::make_shared<ExpressionAbs>(copyNonlinearExpression((((ExpressionAbs *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Square:
        return std::make_shared<ExpressionSquare>(copyNonlinearExpression((((ExpressionSquare *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::SquareRoot:
        return std::make_shared<ExpressionSquareRoot>(copyNonlinearExpression((((ExpressionSquareRoot *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Log:
        return std::make_shared<ExpressionLog>(copyNonlinearExpression((((ExpressionLog *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Exp:
        return std::make_shared<ExpressionExp>(copyNonlinearExpression((((ExpressionExp *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Sin:
        return std::make_shared<ExpressionSin>(copyNonlinearExpression((((ExpressionSin *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Cos:
        return std::make_shared<ExpressionCos>(copyNonlinearExpression((((ExpressionCos *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Tan:
        return std::make_shared<ExpressionTan>(copyNonlinearExpression((((ExpressionTan *)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Constant:
        return std::make_shared<ExpressionConstant>((((ExpressionConstant *)expression)->constant));

    case E_NonlinearExpressionTypes::Variable:
    {
        int variableIndex = ((ExpressionVariable *)expression)->variable->index;
        return std::make_shared<ExpressionVariable>(destination->getVariable(variableIndex));
    }
    default:
        throw new OperationNotImplementedException(std::to_string((int)(expression->getType())));
        break;
    }

    return nullptr;
}

} // namespace SHOT