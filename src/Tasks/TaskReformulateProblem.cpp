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

    useQuadraticConstraints = (quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained);
    useQuadraticObjective = (useQuadraticConstraints || quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticObjective);
    quadraticObjectiveRegardedAsNonlinear = false;

    int additionalNumberOfNonlinearConstraints = 0;

    if (env->settings->getBoolSetting("Reformulation.Epigraph.Use", "Model"))
    {
        additionalNumberOfNonlinearConstraints = 1;
    }

    int auxVarCounter = 0;
    int auxConstrCounter = 0;

    reformulatedProblem = std::make_shared<Problem>(env);
    reformulatedProblem->name = env->problem->name + " (reformulated)";

    reformulatedProblem->allVariables.reserve(env->problem->allVariables.size());
    reformulatedProblem->realVariables.reserve(env->problem->realVariables.size());
    reformulatedProblem->binaryVariables.reserve(env->problem->binaryVariables.size());
    reformulatedProblem->integerVariables.reserve(env->problem->integerVariables.size());
    reformulatedProblem->semicontinuousVariables.reserve(env->problem->semicontinuousVariables.size());
    reformulatedProblem->nonlinearVariables.reserve(env->problem->nonlinearVariables.size());

    reformulatedProblem->variableLowerBounds = env->problem->variableLowerBounds;
    reformulatedProblem->variableUpperBounds = env->problem->variableUpperBounds;

    reformulatedProblem->numericConstraints.reserve(env->problem->numericConstraints.size());
    reformulatedProblem->linearConstraints.reserve(env->problem->linearConstraints.size());

    if (useQuadraticConstraints)
    {
        reformulatedProblem->quadraticConstraints.reserve(env->problem->quadraticConstraints.size());
        reformulatedProblem->nonlinearConstraints.reserve(env->problem->nonlinearConstraints.size() + additionalNumberOfNonlinearConstraints);
    }
    else
    {
        reformulatedProblem->nonlinearConstraints.reserve(env->problem->nonlinearConstraints.size() + env->problem->quadraticConstraints.size() + additionalNumberOfNonlinearConstraints);
    }

    reformulatedProblem->factorableFunctionVariables.reserve(env->problem->factorableFunctionVariables.size());
    reformulatedProblem->factorableFunctions.reserve(env->problem->factorableFunctions.size());

    // Copying variables
    for (auto &V : env->problem->allVariables)
    {
        auto variable = std::make_shared<Variable>(V->name, V->index, V->type, V->lowerBound, V->upperBound);
        reformulatedProblem->add(std::move(variable));
    }

    for (auto &C : env->problem->numericConstraints)
    {
        reformulateConstraint(C);
    }

    /*
        if (C->properties.hasQuadraticTerms)
        {
            for (auto &T : std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms.terms)
            {
                auto firstVariable = reformulatedProblem->getVariable(T->firstVariable->index);
                auto secondVariable = reformulatedProblem->getVariable(T->secondVariable->index);

                auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_aux_" + std::to_string(auxVarCounter), reformulatedProblem->allVariables.size(), E_VariableType::Real, -100000000.0, 100000000.0);

                std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

                if (T->isBilinear && T->isBinary)
                {
                    auxVariable->auxilliaryType = E_AuxilliaryVariableType::BinaryBilinear;
                    auto auxConstraint = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 1.0);

                    auto linearTerm1 = std::make_shared<LinearTerm>(1.0, firstVariable);
                    auto linearTerm2 = std::make_shared<LinearTerm>(1.0, secondVariable);
                    auto linearTerm3 = std::make_shared<LinearTerm>(-1.0, auxVariable);

                    auxConstraint->add(linearTerm1);
                    auxConstraint->add(linearTerm2);
                    auxConstraint->add(linearTerm3);

                    auxVariable->linearTerms.add(linearTerm1);
                    auxVariable->linearTerms.add(linearTerm2);
                    auxVariable->linearTerms.add(linearTerm3);

                    auto auxConstraintBound1 = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 0.0);
                    auxConstraintBound1->add(std::make_shared<LinearTerm>(1.0, auxVariable));
                    auxConstraintBound1->add(std::make_shared<LinearTerm>(-1.0, firstVariable));

                    auto auxConstraintBound2 = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 0.0);
                    auxConstraintBound2->add(std::make_shared<LinearTerm>(1.0, auxVariable));
                    auxConstraintBound2->add(std::make_shared<LinearTerm>(-1.0, secondVariable));

                    auxVariable->lowerBound = 0.0;
                    auxVariable->upperBound = 1.0;

                    reformulatedProblem->add(std::move(auxVariable));
                    reformulatedProblem->add(std::move(auxConstraint));
                    reformulatedProblem->add(std::move(auxConstraintBound1));
                    reformulatedProblem->add(std::move(auxConstraintBound2));
                }
                else if (T->isBilinear && T->firstVariable->type == E_VariableType::Binary || T->secondVariable->type == E_VariableType::Binary)
                {
                    auxVariable->auxilliaryType = E_AuxilliaryVariableType::BinaryContinuousOrIntegerBilinear;
                    auxVariable->quadraticTerms.add(T);

                    auto binaryVariable = (T->firstVariable->type == E_VariableType::Binary) ? T->firstVariable : T->secondVariable;
                    auto otherVariable = (T->firstVariable->type == E_VariableType::Binary) ? T->secondVariable : T->firstVariable;

                    auto auxConstraint1 = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, otherVariable->upperBound);
                    auxConstraint1->add(std::make_shared<LinearTerm>(1.0, otherVariable));
                    auxConstraint1->add(std::make_shared<LinearTerm>(secondVariable->upperBound, binaryVariable));
                    auxConstraint1->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

                    auto auxConstraint2 = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, -otherVariable->lowerBound);
                    auxConstraint2->add(std::make_shared<LinearTerm>(-1.0, otherVariable));
                    auxConstraint2->add(std::make_shared<LinearTerm>(-secondVariable->lowerBound, binaryVariable));
                    auxConstraint2->add(std::make_shared<LinearTerm>(1.0, auxVariable));

                    auxVariable->lowerBound = std::min(0.0, otherVariable->lowerBound);
                    auxVariable->upperBound = otherVariable->upperBound;

                    reformulatedProblem->add(std::move(auxVariable));
                    reformulatedProblem->add(std::move(auxConstraint1));
                    reformulatedProblem->add(std::move(auxConstraint2));
                }
                else
                {

                    auxVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearConstraintPartitioning;
                    auto auxConstraint = std::make_shared<NonlinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 0.0);
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

                    reformulatedProblem->add(std::move(auxVariable));
                    reformulatedProblem->add(std::move(auxConstraint));
                }

                auxVarCounter++;
                auxConstrCounter++;
            }
        }

        // Rewrite sums in nonlinear constraint as individual constraints
        if (C->properties.hasNonlinearExpression && std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto resultLinearTerms = partitionNonlinearSum(std::dynamic_pointer_cast<ExpressionSum>(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression), isSignReversed);

            for (auto &T : resultLinearTerms.terms)
            {
                std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(std::move(T));
            }
        }

        constraint->constant = signfactor * constant;

        if (C->properties.hasNonlinearExpression && std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression->getType() != E_NonlinearExpressionTypes::Sum)
        {
            if (signfactor == -1)
            {
                std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(std::make_shared<ExpressionNegate>(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), reformulatedProblem)));
            }
            else
            {
                std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), reformulatedProblem));
            }

            reformulatedProblem->add(std::move(std::dynamic_pointer_cast<NonlinearConstraint>(constraint)));
        }
        else
        {
            reformulatedProblem->add(std::move(std::dynamic_pointer_cast<LinearConstraint>(constraint)));
        }
    }*/

    // Copy objective function

    reformulatedProblem->finalize();

    // Fixing that a quadratic objective changed into a nonlinear constraint is correctly identified
    if (quadraticObjectiveRegardedAsNonlinear && reformulatedProblem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic)
    {
        reformulatedProblem->objectiveFunction->properties.classification = E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear;
        reformulatedProblem->properties.isMIQPProblem = false;
        reformulatedProblem->properties.isMINLPProblem = true;
    }

    env->reformulatedProblem = reformulatedProblem;

    std::cout << env->problem << std::endl;

    std::cout << env->reformulatedProblem << std::endl;

    env->timing->stopTimer("ProblemReformulation");
}

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

void TaskReformulateProblem::reformulateObjectiveFunction()
{
    if (env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Linear)
    {
        // Linear objective function
        auto destinationObjective = std::make_shared<LinearObjectiveFunction>();
        auto sourceObjective = std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction);

        copyLinearTermsToObjectiveFunction(sourceObjective->linearTerms, destinationObjective);

        destinationObjective->direction = env->problem->objectiveFunction->direction;
        destinationObjective->constant = env->problem->objectiveFunction->constant;

        reformulatedProblem->add(std::move(destinationObjective));

        return;
    }

    if (useQuadraticObjective && (env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic || !env->problem->objectiveFunction->properties.hasNonlinearExpression))
    {
        // Quadratic objective function
        auto destinationObjective = std::make_shared<QuadraticObjectiveFunction>();
        auto sourceObjective = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction);

        copyLinearTermsToObjectiveFunction(sourceObjective->linearTerms, destinationObjective);
        copyQuadraticTermsToObjectiveFunction(sourceObjective->quadraticTerms, destinationObjective);

        destinationObjective->direction = env->problem->objectiveFunction->direction;
        destinationObjective->constant = env->problem->objectiveFunction->constant;

        reformulatedProblem->add(std::move(destinationObjective));

        return;
    }

    if (env->settings->getBoolSetting("Reformulation.Epigraph.Use", "Model"))
    {
        bool isSignReversed = env->problem->objectiveFunction->properties.isMaximize;
        double signfactor = (env->problem->objectiveFunction->properties.isMinimize) ? 1.0 : -1.0;

        // Adding new linear objective function
        auto objective = std::make_shared<LinearObjectiveFunction>();
        objective->direction = E_ObjectiveFunctionDirection::Minimize;
        objective->constant = 0.0;
        double objVarBound = env->settings->getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");
        auto objectiveVariable = std::make_shared<AuxilliaryVariable>("shot_objvar", reformulatedProblem->allVariables.size(), E_VariableType::Real, -objVarBound, objVarBound);

        objectiveVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearObjectiveFunction;
        objective->add(std::make_shared<LinearTerm>(1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));

        // Adding the auxilliary objective constraint
        auto constraint = std::make_shared<NonlinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_objconstr", SHOT_DBL_MIN, -1.0 * signfactor * env->problem->objectiveFunction->constant);

        copyLinearTermsToConstraint(std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms, constraint);

        for (auto &T : constraint->linearTerms.terms)
        {
            objectiveVariable->linearTerms.add(T);
        }

        constraint->add(std::make_shared<LinearTerm>(-1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));

        copyQuadraticTermsToConstraint(std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms, constraint);

        for (auto &T : constraint->quadraticTerms.terms)
        {
            objectiveVariable->quadraticTerms.add(T);
        }

        if (env->problem->objectiveFunction->properties.hasNonlinearExpression)
        {
            if (signfactor == -1)
            {
                constraint->add(simplify(std::make_shared<ExpressionNegate>(
                    copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression.get(), reformulatedProblem))));
            }
            else
            {
                constraint->add(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression.get(), reformulatedProblem));
            }

            objectiveVariable->nonlinearExpression = constraint->nonlinearExpression;
        }

        reformulatedProblem->add(std::move(objectiveVariable));
        reformulatedProblem->add(std::move(objective));
        reformulateConstraint(constraint);

        return;
    }

    double signfactor = (env->problem->objectiveFunction->properties.isMinimize) ? 1.0 : -1.0;
    double objVarBound = env->settings->getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");
    ObjectiveFunctionPtr objective;

    if (!env->problem->objectiveFunction->properties.hasNonBinaryBilinearTerms && !env->problem->objectiveFunction->properties.hasNonBinarySquareTerms)
    {
        objective = std::make_shared<LinearObjectiveFunction>();

        for (auto &T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms.terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(T->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(T->secondVariable->index);

            auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_aux_" + std::to_string(auxVarCounter), reformulatedProblem->allVariables.size(), E_VariableType::Real, -100000000.0, 100000000.0);

            std::dynamic_pointer_cast<LinearObjectiveFunction>(objective)->add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

            auxVariable->auxilliaryType = E_AuxilliaryVariableType::BinaryBilinear;
            auto auxConstraint = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 1.0);

            auto linearTerm1 = std::make_shared<LinearTerm>(1.0, firstVariable);
            auto linearTerm2 = std::make_shared<LinearTerm>(1.0, secondVariable);
            auto linearTerm3 = std::make_shared<LinearTerm>(-1.0, auxVariable);

            auxConstraint->add(linearTerm1);
            auxConstraint->add(linearTerm2);
            auxConstraint->add(linearTerm3);

            auxVariable->linearTerms.add(linearTerm1);
            auxVariable->linearTerms.add(linearTerm2);
            auxVariable->linearTerms.add(linearTerm3);

            auto auxConstraintBound1 = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintBound1->add(std::make_shared<LinearTerm>(1.0, auxVariable));
            auxConstraintBound1->add(std::make_shared<LinearTerm>(-1.0, firstVariable));

            auto auxConstraintBound2 = std::make_shared<LinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxConstrCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintBound2->add(std::make_shared<LinearTerm>(1.0, auxVariable));
            auxConstraintBound2->add(std::make_shared<LinearTerm>(-1.0, secondVariable));

            auxVariable->lowerBound = 0.0;
            auxVariable->upperBound = 1.0;

            reformulatedProblem->add(std::move(auxVariable));
            reformulatedProblem->add(std::move(auxConstraint));
            reformulatedProblem->add(std::move(auxConstraintBound1));
            reformulatedProblem->add(std::move(auxConstraintBound2));
        }
    }
    else if (env->problem->objectiveFunction->properties.hasQuadraticTerms)
    {
        objective = std::make_shared<QuadraticObjectiveFunction>();

        for (auto &T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms.terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(T->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(T->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(objective)->add(std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
        }
    }
    else
    {
        objective = std::make_shared<LinearObjectiveFunction>();
    }

    for (auto &T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms.terms)
    {
        auto variable = reformulatedProblem->getVariable(T->variable->index);
        std::dynamic_pointer_cast<LinearObjectiveFunction>(objective)->add(std::make_shared<LinearTerm>(T->coefficient, variable));
    }

    if (std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
    {
        auto sum = std::dynamic_pointer_cast<ExpressionSum>(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression);

        for (auto &T : sum->children.expressions)
        {
            auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_aux_" + std::to_string(auxVarCounter), reformulatedProblem->allVariables.size(), E_VariableType::Real, -objVarBound, objVarBound);

            auxVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearObjectiveFunctionPartitioning;
            std::dynamic_pointer_cast<LinearObjectiveFunction>(objective)->add(std::make_shared<LinearTerm>(1.0, auxVariable));

            auto constraint = std::make_shared<NonlinearConstraint>(reformulatedProblem->numericConstraints.size(), "shot_auxconstr_" + std::to_string(auxVarCounter), SHOT_DBL_MIN, 0.0);

            constraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

            if (signfactor == -1)
            {
                constraint->add(std::make_shared<ExpressionNegate>(copyNonlinearExpression(T.get(), reformulatedProblem)));
            }
            else
            {
                constraint->add(copyNonlinearExpression(T.get(), reformulatedProblem));
            }

            auxVariable->nonlinearExpression = constraint->nonlinearExpression;

            reformulatedProblem->add(std::move(auxVariable));
            reformulatedProblem->add(std::move(constraint));
            auxVarCounter++;
        }
    }

    objective->constant = env->problem->objectiveFunction->constant;
    objective->direction = env->problem->objectiveFunction->direction;
    reformulatedProblem->add(std::move(objective));

    /*else if (!useQuadraticObjective || env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Nonlinear)
    {
        auto objective = std::make_shared<NonlinearObjectiveFunction>();

        for (auto &T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms.terms)
        {
            auto variable = reformulatedProblem->getVariable(T->variable->index);
            objective->add(std::make_shared<LinearTerm>(T->coefficient, variable));
        }

        for (auto &T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms.terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(T->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(T->secondVariable->index);

            objective->add(std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
        }

        if (env->problem->objectiveFunction->properties.hasNonlinearExpression)
        {
            objective->add(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->nonlinearExpression.get(), reformulatedProblem));
        }
        else
        {
            quadraticObjectiveRegardedAsNonlinear = true;
        }

        objective->direction = env->problem->objectiveFunction->direction;
        reformulatedProblem->add(objective);

        reformulatedProblem->objectiveFunction->constant = env->problem->objectiveFunction->constant;
    }*/
}

void TaskReformulateProblem::reformulateConstraint(NumericConstraintPtr C)
{
    double valueLHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueLHS;
    double valueRHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueRHS;
    double constant = std::dynamic_pointer_cast<NumericConstraint>(C)->constant;

    if (C->properties.classification == E_ConstraintClassification::Linear || (!C->properties.hasNonlinearExpression && !C->properties.hasQuadraticTerms))
    {
        LinearConstraintPtr constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        auto sourceConstraint = std::dynamic_pointer_cast<LinearConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);

        constraint->constant = constant;
        reformulatedProblem->add(std::move(constraint));

        return;
    }

    if (useQuadraticConstraints && (C->properties.classification == E_ConstraintClassification::Quadratic || !C->properties.hasNonlinearExpression))
    {
        QuadraticConstraintPtr constraint = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
        auto sourceConstraint = std::dynamic_pointer_cast<QuadraticConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);
        copyQuadraticTermsToConstraint(sourceConstraint->quadraticTerms, constraint);

        constraint->constant = constant;
        reformulatedProblem->add(std::move(constraint));

        return;
    }

    // Constraint is to be regarded as nonlinear

    auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

    bool copyOriginalLinearTerms = false;
    bool copyOriginalQuadraticTerms = false;
    bool copyOriginalNonlinearExpression = false;

    // These will be added to the new constraint, and their signs have been altered
    LinearTerms destinationLinearTerms;
    QuadraticTerms destinationQuadraticTerms;

    bool isSignReversed = false;

    if (valueLHS > SHOT_DBL_MIN && valueRHS == SHOT_DBL_MAX)
    {
        // Constraint is of type l <= g(x). Rewrite as -g(x) <= -l
        isSignReversed = true;
        valueRHS = -valueLHS;
        valueLHS = SHOT_DBL_MIN;
    }
    else if (valueLHS != SHOT_DBL_MIN && valueRHS != SHOT_DBL_MAX)
    {
        // Constraint is of type l <= g(x) <= u. Rewrite as -g(x) <= -l and g(x) <= u
        // TODO
        std::cout << "Can not reformulate constraint currently\n";
    }

    if (C->properties.hasLinearTerms)
        copyOriginalLinearTerms = true;

    if (C->properties.hasQuadraticTerms)
    {
        if (env->settings->getBoolSetting("Reformulation.Constraint.PartitionQuadraticSums", "Model"))
        {
            //TODO
        }
        else
        {
            copyOriginalQuadraticTerms = true;
        }
    }

    if (C->properties.hasNonlinearExpression)
    {
        if (env->settings->getBoolSetting("Reformulation.Constraint.PartitionNonlinearSums", "Model") &&
            sourceConstraint->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            destinationLinearTerms = partitionNonlinearSum(std::dynamic_pointer_cast<ExpressionSum>(sourceConstraint->nonlinearExpression), isSignReversed);
        }
        else
        {
            copyOriginalNonlinearExpression = true;
        }
    }

    NumericConstraintPtr constraint;

    if (copyOriginalNonlinearExpression)
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
    }
    else if (copyOriginalQuadraticTerms)
    {
        constraint = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
    }
    else
    {
        constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
    }

    constraint->constant = constant;

    if (copyOriginalLinearTerms)
        copyLinearTermsToConstraint(sourceConstraint->linearTerms, std::dynamic_pointer_cast<LinearConstraint>(constraint), isSignReversed);

    if (destinationLinearTerms.terms.size() > 0)
        std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(destinationLinearTerms);

    if (copyOriginalQuadraticTerms)
        copyQuadraticTermsToConstraint(sourceConstraint->quadraticTerms, std::dynamic_pointer_cast<QuadraticConstraint>(constraint), isSignReversed);

    if (destinationQuadraticTerms.terms.size() > 0)
        std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->add(destinationQuadraticTerms);

    if (copyOriginalNonlinearExpression)
    {
        if (isSignReversed)
            std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(sourceConstraint->nonlinearExpression);
        else
            std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(std::make_shared<ExpressionNegate>(copyNonlinearExpression(sourceConstraint->nonlinearExpression.get(), reformulatedProblem)));
    }

    if (copyOriginalNonlinearExpression)
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<NonlinearConstraint>(constraint)));
    }
    else if (copyOriginalQuadraticTerms)
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<QuadraticConstraint>(constraint)));
    }
    else
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<LinearConstraint>(constraint)));
    }
}

LinearTerms TaskReformulateProblem::partitionNonlinearSum(const std::shared_ptr<ExpressionSum> source, bool reversedSigns)
{
    LinearTerms resultLinearTerms;

    if (source.get() == nullptr)
        return (resultLinearTerms);

    for (auto &T : source->children.expressions)
    {
        auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_p_" + std::to_string(auxVariableCounter), auxVariableCounter, E_VariableType::Real, -9999999999.0, 9999999999.0);
        auxVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearExpressionPartitioning;

        resultLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

        auto auxConstraint = std::make_shared<NonlinearConstraint>(auxConstraintCounter, "shot_p_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

        if (reversedSigns)
        {
            auxConstraint->add(copyNonlinearExpression(T.get(), reformulatedProblem));
        }
        else
        {
            auxConstraint->add(simplify(std::make_shared<ExpressionNegate>(copyNonlinearExpression(T.get(), reformulatedProblem))));
        }

        auxVariable->nonlinearExpression = auxConstraint->nonlinearExpression;

        reformulatedProblem->add(std::move(auxVariable));
        reformulatedProblem->add(std::move(auxConstraint));
        auxVariableCounter++;
        auxConstraintCounter++;
    }

    return (resultLinearTerms);
}

/*LinearTerms TaskReformulateProblem::partitionNonlinearSum(const std::shared_ptr<ExpressionSum> source, bool reversedSigns)
{
    LinearTerms resultLinearTerms;
    QuadraticTerms resultQuadraticTerms;
    NonlinearExpressionPtr resultExpression;

    if (source.get() == nullptr)
        return std::tuple(resultLinearTerms, resultQuadraticTerms, resultExpression);

    for (auto &T : source->children.expressions)
    {
        auto auxVariable = std::make_shared<AuxilliaryVariable>("shot_p_" + std::to_string(auxVariableCounter), auxVariableCounter, E_VariableType::Real, -9999999999.0, 9999999999.0);
        auxVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearExpressionPartitioning;

        resultLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

        auto auxConstraint = std::make_shared<NonlinearConstraint>(auxConstraintCounter, "shot_p_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

        if (reversedSigns)
        {
            auxConstraint->add(copyNonlinearExpression(T.get(), reformulatedProblem));
        }
        else
        {
            auxConstraint->add(simplify(std::make_shared<ExpressionNegate>(copyNonlinearExpression(T.get(), reformulatedProblem))));
        }

        auxVariable->nonlinearExpression = auxConstraint->nonlinearExpression;

        
                    //TODO figure out why bounds are not helping
                    if (auxVariable->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Square)
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
                    }

        reformulatedProblem->add(std::move(auxVariable));
        reformulatedProblem->add(std::move(auxConstraint));
        auxVariableCounter++;
        auxConstraintCounter++;
    }

    return std::tuple(resultLinearTerms, resultQuadraticTerms, resultExpression);

    constraint->constant = signfactor * constant;

    if (C->properties.hasNonlinearExpression && std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression->getType() != E_NonlinearExpressionTypes::Sum)
    {

        if (signfactor == -1)
        {
            std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(std::make_shared<ExpressionNegate>(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), reformulatedProblem)));
        }
        else
        {
            std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), reformulatedProblem));
        }

        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<NonlinearConstraint>(constraint)));
    }
    else
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<LinearConstraint>(constraint)));
    }
} // namespace SHOT*/

template <class T>
void TaskReformulateProblem::copyLinearTermsToConstraint(LinearTerms terms, T destination, bool reversedSigns)
{
    if (reversedSigns)
    {
        for (auto &LT : terms.terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearConstraint>(destination)->add(std::make_shared<LinearTerm>(-LT->coefficient, variable));
        }
    }
    else
    {
        for (auto &LT : terms.terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearConstraint>(destination)->add(std::make_shared<LinearTerm>(LT->coefficient, variable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToConstraint(QuadraticTerms terms, T destination, bool reversedSigns)
{
    if (reversedSigns)
    {
        for (auto &QT : terms.terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticConstraint>(destination)->add(std::make_shared<QuadraticTerm>(-QT->coefficient, firstVariable, secondVariable));
        }
    }
    else
    {
        for (auto &QT : terms.terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticConstraint>(destination)->add(std::make_shared<QuadraticTerm>(QT->coefficient, firstVariable, secondVariable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyLinearTermsToObjectiveFunction(LinearTerms terms, T destination, bool reversedSigns)
{
    if (reversedSigns)
    {
        for (auto &LT : terms.terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearObjectiveFunction>(destination)->add(std::make_shared<LinearTerm>(-LT->coefficient, variable));
        }
    }
    else
    {
        for (auto &LT : terms.terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearObjectiveFunction>(destination)->add(std::make_shared<LinearTerm>(LT->coefficient, variable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToObjectiveFunction(QuadraticTerms terms, T destination, bool reversedSigns)
{
    if (reversedSigns)
    {
        for (auto &QT : terms.terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(destination)->add(std::make_shared<QuadraticTerm>(-QT->coefficient, firstVariable, secondVariable));
        }
    }
    else
    {
        for (auto &QT : terms.terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(destination)->add(std::make_shared<QuadraticTerm>(QT->coefficient, firstVariable, secondVariable));
        }
    }
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