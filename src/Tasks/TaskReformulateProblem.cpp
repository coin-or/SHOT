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

    auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
        env->settings->getIntSetting("Reformulation.Quadratics.Strategy", "Model"));

    useQuadraticConstraints = (quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained);

    useQuadraticObjective
        = (useQuadraticConstraints || quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticObjective);

    quadraticObjectiveRegardedAsNonlinear = false;

    PartitionQuadraticTermsInObjective
        = env->settings->getBoolSetting("Reformulation.ObjectiveFunction.PartitionQuadraticTerms", "Model");

    PartitionQuadraticTermsInConstraint
        = env->settings->getBoolSetting("Reformulation.Constraint.PartitionQuadraticTerms", "Model");

    int additionalNumberOfNonlinearConstraints = 0;

    if(env->settings->getBoolSetting("Reformulation.ObjectiveFunction.Epigraph.Use", "Model"))
    {
        additionalNumberOfNonlinearConstraints = 1;
    }

    auxVariableCounter = 0;
    auxConstraintCounter = env->problem->properties.numberOfVariables;

    reformulatedProblem = std::make_shared<Problem>(env);
    reformulatedProblem->name = env->problem->name + " (reformulated)";

    reformulatedProblem->variableLowerBounds = env->problem->variableLowerBounds;
    reformulatedProblem->variableUpperBounds = env->problem->variableUpperBounds;

    // Copying variables
    for(auto& V : env->problem->allVariables)
    {
        auto variable = std::make_shared<Variable>(V->name, V->index, V->type, V->lowerBound, V->upperBound);
        reformulatedProblem->add(std::move(variable));
    }

    // Reformulating constraints
    for(auto& C : env->problem->numericConstraints)
    {
        reformulateConstraint(C);
    }

    // Reformulating objective function
    reformulateObjectiveFunction();

    reformulatedProblem->finalize();

    // Fixing that a quadratic objective changed into a nonlinear constraint is correctly identified
    if(quadraticObjectiveRegardedAsNonlinear
        && reformulatedProblem->objectiveFunction->properties.classification
            == E_ObjectiveFunctionClassification::Quadratic)
    {
        reformulatedProblem->objectiveFunction->properties.classification
            = E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear;
        reformulatedProblem->properties.isMIQPProblem = false;
        reformulatedProblem->properties.isMINLPProblem = true;
    }

    int index = 0;
    for(auto& C : reformulatedProblem->numericConstraints)
    {
        C->index = index;
        index++;
    }

    env->reformulatedProblem = reformulatedProblem;

    if(env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        std::stringstream filename;
        filename << env->settings->getStringSetting("Debug.Path", "Output");
        filename << "/reformulatedproblem";
        filename << ".txt";

        std::stringstream problem;
        problem << env->reformulatedProblem;

        UtilityFunctions::writeStringToFile(filename.str(), problem.str());
    }

    env->timing->stopTimer("ProblemReformulation");
}

TaskReformulateProblem::~TaskReformulateProblem() {}

void TaskReformulateProblem::run() {}

std::string TaskReformulateProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

void TaskReformulateProblem::reformulateObjectiveFunction()
{
    if(env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Linear)
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

    if(useQuadraticObjective
        && (env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic
               || !env->problem->objectiveFunction->properties.hasNonlinearExpression))
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

    if(env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic)
    {
        quadraticObjectiveRegardedAsNonlinear = true;
    }

    if(env->settings->getBoolSetting("Reformulation.ObjectiveFunction.Epigraph.Use", "Model"))
    {
        bool isSignReversed = env->problem->objectiveFunction->properties.isMaximize;
        double signfactor = (env->problem->objectiveFunction->properties.isMinimize) ? 1.0 : -1.0;

        // Adding new linear objective function
        auto objective = std::make_shared<LinearObjectiveFunction>();
        objective->direction = E_ObjectiveFunctionDirection::Minimize;
        objective->constant = 0.0;
        double objVarBound = env->settings->getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");
        auto objectiveVariable = std::make_shared<AuxilliaryVariable>(
            "shot_objvar", reformulatedProblem->allVariables.size(), E_VariableType::Real, -objVarBound, objVarBound);

        objectiveVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearObjectiveFunction;
        objective->add(std::make_shared<LinearTerm>(1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));

        // Adding the auxilliary objective constraint
        auto constraint = std::make_shared<NonlinearConstraint>(reformulatedProblem->numericConstraints.size(),
            "shot_objconstr", SHOT_DBL_MIN, -1.0 * signfactor * env->problem->objectiveFunction->constant);

        copyLinearTermsToConstraint(
            std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms,
            constraint);

        for(auto& T : constraint->linearTerms)
        {
            objectiveVariable->linearTerms.add(T);
        }

        constraint->add(std::make_shared<LinearTerm>(-1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));

        copyQuadraticTermsToConstraint(
            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms,
            constraint);

        for(auto& T : constraint->quadraticTerms)
        {
            objectiveVariable->quadraticTerms.add(T);
        }

        if(env->problem->objectiveFunction->properties.hasNonlinearExpression)
        {
            if(signfactor == -1)
            {
                constraint->add(simplify(std::make_shared<ExpressionNegate>(copyNonlinearExpression(
                    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                        ->nonlinearExpression.get(),
                    reformulatedProblem))));
            }
            else
            {
                constraint->add(copyNonlinearExpression(
                    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                        ->nonlinearExpression.get(),
                    reformulatedProblem));
            }

            objectiveVariable->nonlinearExpression = constraint->nonlinearExpression;
        }

        reformulatedProblem->add(std::move(objectiveVariable));
        reformulatedProblem->add(std::move(objective));
        reformulateConstraint(constraint);

        return;
    }

    // Objective is to be regarded as nonlinear

    bool copyOriginalLinearTerms = false;
    bool copyOriginalNonlinearExpression = false;

    // These will be added to the new constraint, and their signs have been altered
    LinearTerms destinationLinearTerms;
    QuadraticTerms destinationQuadraticTerms;

    bool isSignReversed = env->problem->objectiveFunction->properties.isMaximize;

    if(env->problem->objectiveFunction->properties.hasLinearTerms)
        copyOriginalLinearTerms = true;

    if(env->problem->objectiveFunction->properties.hasQuadraticTerms)
    {
        auto sourceObjective = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction);

        auto [tmpLinearTerms, tmpQuadraticTerms] = reformulateAndPartitionQuadraticSum(
            sourceObjective->quadraticTerms, isSignReversed, PartitionQuadraticTermsInObjective);

        destinationLinearTerms.add(tmpLinearTerms);
        destinationQuadraticTerms.add(tmpQuadraticTerms);
    }

    if(env->problem->objectiveFunction->properties.hasNonlinearExpression)
    {
        auto sourceObjective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction);

        if(env->settings->getBoolSetting("Reformulation.Constraint.PartitionNonlinearTerms", "Model")
            && sourceObjective->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto tmpLinearTerms = partitionNonlinearSum(
                std::dynamic_pointer_cast<ExpressionSum>(sourceObjective->nonlinearExpression), isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else
        {
            copyOriginalNonlinearExpression = true;
        }
    }

    ObjectiveFunctionPtr objective;

    if(copyOriginalNonlinearExpression)
    {
        objective = std::make_shared<NonlinearObjectiveFunction>();
    }
    else if(destinationQuadraticTerms.size() > 0)
    {
        objective = std::make_shared<QuadraticObjectiveFunction>();
    }
    else
    {
        objective = std::make_shared<LinearObjectiveFunction>();
    }

    objective->constant = env->problem->objectiveFunction->constant;
    objective->direction = env->problem->objectiveFunction->direction;

    if(copyOriginalLinearTerms)
        copyLinearTermsToObjectiveFunction(
            std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms,
            std::dynamic_pointer_cast<LinearObjectiveFunction>(objective), isSignReversed);

    if(destinationLinearTerms.size() > 0)
        std::dynamic_pointer_cast<LinearObjectiveFunction>(objective)->add(destinationLinearTerms);

    if(destinationQuadraticTerms.size() > 0)
        std::dynamic_pointer_cast<QuadraticObjectiveFunction>(objective)->add(destinationQuadraticTerms);

    if(copyOriginalNonlinearExpression)
    {
        auto sourceObjective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction);

        if(isSignReversed)
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)->add(
                simplify(std::make_shared<ExpressionNegate>(
                    copyNonlinearExpression(sourceObjective->nonlinearExpression.get(), reformulatedProblem))));
        else
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)->add(
                copyNonlinearExpression(sourceObjective->nonlinearExpression.get(), reformulatedProblem));
    }

    if(copyOriginalNonlinearExpression)
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)));
    }
    else if(destinationQuadraticTerms.size() > 0)
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<QuadraticObjectiveFunction>(objective)));
    }
    else
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<LinearObjectiveFunction>(objective)));
    }
}

void TaskReformulateProblem::reformulateConstraint(NumericConstraintPtr C)
{
    double valueLHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueLHS;
    double valueRHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueRHS;
    double constant = std::dynamic_pointer_cast<NumericConstraint>(C)->constant;

    if(C->properties.classification == E_ConstraintClassification::Linear
        || (!C->properties.hasNonlinearExpression && !C->properties.hasQuadraticTerms))
    {
        LinearConstraintPtr constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        auto sourceConstraint = std::dynamic_pointer_cast<LinearConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);

        constraint->constant = constant;
        reformulatedProblem->add(std::move(constraint));

        return;
    }

    if(useQuadraticConstraints
        && (C->properties.classification == E_ConstraintClassification::Quadratic
               || !C->properties.hasNonlinearExpression))
    {
        QuadraticConstraintPtr constraint
            = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
        auto sourceConstraint = std::dynamic_pointer_cast<QuadraticConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);
        copyQuadraticTermsToConstraint(sourceConstraint->quadraticTerms, constraint);

        constraint->constant = constant;
        reformulatedProblem->add(std::move(constraint));

        return;
    }

    // Constraint is to be regarded as nonlinear

    bool copyOriginalLinearTerms = false;
    bool copyOriginalNonlinearExpression = false;

    // These will be added to the new constraint, and their signs have been altered
    LinearTerms destinationLinearTerms;
    QuadraticTerms destinationQuadraticTerms;

    bool isSignReversed = false;

    if(valueLHS > SHOT_DBL_MIN && valueRHS == SHOT_DBL_MAX)
    {
        // Constraint is of type l <= g(x). Rewrite as -g(x) <= -l
        isSignReversed = true;
        valueRHS = -valueLHS;
        valueLHS = SHOT_DBL_MIN;
    }
    else if(valueLHS == valueRHS)
    {
        // Have a nonlinear equality constraint, writing it as g(x)<= u and -g(x) <= -u

        auto auxConstraint1
            = std::make_shared<NonlinearConstraint>(auxConstraintCounter, C->name + "_a", SHOT_DBL_MIN, valueRHS);
        auxConstraintCounter++;

        auxConstraint1->constant = C->constant;

        if(C->properties.hasLinearTerms)
            copyLinearTermsToConstraint(std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms, auxConstraint1);

        if(C->properties.hasQuadraticTerms)
            copyQuadraticTermsToConstraint(
                std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms, auxConstraint1);

        if(C->properties.hasNonlinearExpression)
            auxConstraint1->add(copyNonlinearExpression(
                std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), reformulatedProblem));

        reformulateConstraint(auxConstraint1);

        auto auxConstraint2
            = std::make_shared<NonlinearConstraint>(auxConstraintCounter, C->name + "_b", SHOT_DBL_MIN, -valueRHS);
        auxConstraintCounter++;

        auxConstraint2->constant = C->constant;

        if(C->properties.hasLinearTerms)
            copyLinearTermsToConstraint(
                std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms, auxConstraint2, true);

        if(C->properties.hasQuadraticTerms)
            copyQuadraticTermsToConstraint(
                std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms, auxConstraint2, true);

        if(C->properties.hasNonlinearExpression)
            auxConstraint2->add(simplify(std::make_shared<ExpressionNegate>(copyNonlinearExpression(
                std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), reformulatedProblem))));

        reformulateConstraint(auxConstraint2);

        return;
    }
    else if(valueLHS != SHOT_DBL_MIN && valueRHS != SHOT_DBL_MAX)
    {
        // Constraint is of type l <= g(x) <= u. Rewrite as -g(x) <= -l and g(x) <= u

        std::cout << "Can not reformulate constraint currently\n";
    }

    if(C->properties.hasLinearTerms)
        copyOriginalLinearTerms = true;

    if(C->properties.hasQuadraticTerms)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<QuadraticConstraint>(C);

        auto [tmpLinearTerms, tmpQuadraticTerms] = reformulateAndPartitionQuadraticSum(
            sourceConstraint->quadraticTerms, isSignReversed, PartitionQuadraticTermsInConstraint);

        destinationLinearTerms.add(tmpLinearTerms);
        destinationQuadraticTerms.add(tmpQuadraticTerms);
    }

    if(C->properties.hasNonlinearExpression)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

        if(env->settings->getBoolSetting("Reformulation.Constraint.PartitionNonlinearTerms", "Model")
            && sourceConstraint->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto tmpLinearTerms = partitionNonlinearSum(
                std::dynamic_pointer_cast<ExpressionSum>(sourceConstraint->nonlinearExpression), isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else
        {
            copyOriginalNonlinearExpression = true;
        }
    }

    NumericConstraintPtr constraint;

    if(copyOriginalNonlinearExpression)
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
    }
    else if(destinationQuadraticTerms.size() > 0 && !useQuadraticConstraints)
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
    }
    else if(destinationQuadraticTerms.size() > 0)
    {
        constraint = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
    }
    else
    {
        constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
    }

    constraint->constant = constant;

    if(copyOriginalLinearTerms)
        copyLinearTermsToConstraint(std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms,
            std::dynamic_pointer_cast<LinearConstraint>(constraint), isSignReversed);

    if(destinationLinearTerms.size() > 0)
        std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(destinationLinearTerms);

    if(destinationQuadraticTerms.size() > 0)
        std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->add(destinationQuadraticTerms);

    if(copyOriginalNonlinearExpression)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

        if(isSignReversed)
            std::dynamic_pointer_cast<NonlinearConstraint>(constraint)
                ->add(simplify(std::make_shared<ExpressionNegate>(
                    copyNonlinearExpression(sourceConstraint->nonlinearExpression.get(), reformulatedProblem))));
        else
            std::dynamic_pointer_cast<NonlinearConstraint>(constraint)
                ->add(copyNonlinearExpression(sourceConstraint->nonlinearExpression.get(), reformulatedProblem));
    }

    if(copyOriginalNonlinearExpression)
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<NonlinearConstraint>(constraint)));
    }
    else if(destinationQuadraticTerms.size() > 0 && !useQuadraticConstraints)
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<NonlinearConstraint>(constraint)));
    }
    else if(destinationQuadraticTerms.size() > 0)
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<QuadraticConstraint>(constraint)));
    }
    else
    {
        reformulatedProblem->add(std::move(std::dynamic_pointer_cast<LinearConstraint>(constraint)));
    }
}

LinearTerms TaskReformulateProblem::partitionNonlinearSum(
    const std::shared_ptr<ExpressionSum> source, bool reversedSigns)
{
    LinearTerms resultLinearTerms;

    if(source.get() == nullptr)
        return (resultLinearTerms);

    bool allNonlinearExpressionsReformulated = false;

    for(auto& T : source->children.expressions)
    {
        if(T->getType() == E_NonlinearExpressionTypes::Product) // Might be able to reuse auxilliary variable further if
                                                                // e.g. bilinear term
        {
            auto optionalQuadraticTerm = convertProductToQuadraticTerm(std::dynamic_pointer_cast<ExpressionProduct>(T));

            if(optionalQuadraticTerm) // The product was a quadratic term
            {
                QuadraticTerms quadTerms;
                quadTerms.add(optionalQuadraticTerm.value());

                auto [tmpLinearTerms, tmpQuadraticTerms]
                    = reformulateAndPartitionQuadraticSum(quadTerms, reversedSigns, false);

                if(tmpQuadraticTerms.size()
                    == 0) // Otherwise we cannot proceed and will continue as if nonbilinear term
                {
                    resultLinearTerms.add(tmpLinearTerms);
                    continue; // Continue to next nonlinear term
                }
            }

            auto optionalMonomialTerm = convertProductToMonomialTerm(std::dynamic_pointer_cast<ExpressionProduct>(T));

            if(optionalMonomialTerm
                && env->settings->getIntSetting("Reformulation.Monomials.Formulation", "Model")
                    != static_cast<int>(ES_ReformulationBinaryMonomials::None))
            // The product was a monomial term
            {
                MonomialTerms monomialTerms;
                monomialTerms.add(optionalMonomialTerm.value());

                auto [tmpLinearTerms, tmpMonomialTerms] = reformulateMonomialSum(monomialTerms, reversedSigns);

                if(tmpMonomialTerms.size() == 0) // Otherwise we cannot proceed and will continue as if nonbilinear term
                {
                    resultLinearTerms.add(tmpLinearTerms);

                    continue; // Continue to next nonlinear term
                }
            }

            allNonlinearExpressionsReformulated = false;
        }

        if(!allNonlinearExpressionsReformulated)
        {
            auto auxVariable = std::make_shared<AuxilliaryVariable>("s_p_" + std::to_string(auxVariableCounter),
                reformulatedProblem->allVariables.size(), E_VariableType::Real, -9999999999.0, 9999999999.0);
            auxVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearExpressionPartitioning;
            auxVariableCounter++;

            resultLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

            auto auxConstraint = std::make_shared<NonlinearConstraint>(
                auxConstraintCounter, "s_p_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
            auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
            auxConstraintCounter++;

            if(reversedSigns)
            {
                auxConstraint->add(simplify(
                    std::make_shared<ExpressionNegate>(copyNonlinearExpression(T.get(), reformulatedProblem))));
            }
            else
            {
                auxConstraint->add(copyNonlinearExpression(T.get(), reformulatedProblem));
            }

            auxVariable->nonlinearExpression = auxConstraint->nonlinearExpression;

            reformulatedProblem->add(std::move(auxVariable));
            reformulatedProblem->add(std::move(auxConstraint));
        }
    }

    return (resultLinearTerms);
}

std::tuple<LinearTerms, QuadraticTerms> TaskReformulateProblem::reformulateAndPartitionQuadraticSum(
    const QuadraticTerms& quadraticTerms, bool reversedSigns, bool partitionNonBinaryTerms)
{
    LinearTerms resultLinearTerms;
    QuadraticTerms resultQuadraticTerms;

    double signfactor = reversedSigns ? -1.0 : 1.0;

    bool allTermsAreBinary = true;

    for(auto& T : quadraticTerms)
    {
        if(!T->isBinary)
        {
            allTermsAreBinary = false;
            break;
        }
    }

    for(auto& T : quadraticTerms)
    {
        double coeffSign = (T->coefficient > 0) ? 1.0 : -1.0;
        auto firstVariable = reformulatedProblem->getVariable(T->firstVariable->index);
        auto secondVariable = reformulatedProblem->getVariable(T->secondVariable->index);

        if(T->isSquare && allTermsAreBinary) // Square term b^2 -> b
        {
            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, firstVariable));
        }
        else if(T->isBilinear && T->isBinary) // Bilinear term b1*b2
        {
            auto auxVariable = std::make_shared<AuxilliaryVariable>("s_binbl_" + std::to_string(auxVariableCounter),
                reformulatedProblem->allVariables.size(), E_VariableType::Binary, 0.0, 1.0);
            auxVariable->auxilliaryType = E_AuxilliaryVariableType::BinaryBilinear;

            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

            auto auxConstraint = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_binbl_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 1.0);
            auxConstraintCounter++;

            auto linearTerm1 = std::make_shared<LinearTerm>(1.0, firstVariable);
            auto linearTerm2 = std::make_shared<LinearTerm>(1.0, secondVariable);
            auto linearTerm3 = std::make_shared<LinearTerm>(-1.0, auxVariable);

            auxConstraint->add(linearTerm1);
            auxConstraint->add(linearTerm2);
            auxConstraint->add(linearTerm3);

            auxVariable->linearTerms.add(linearTerm1);
            auxVariable->linearTerms.add(linearTerm2);
            auxVariable->linearTerms.add(linearTerm3);
            auxVariable->constant = 1.0;

            auto auxConstraintBound1 = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_blbb_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintBound1->add(std::make_shared<LinearTerm>(1.0, auxVariable));
            auxConstraintBound1->add(std::make_shared<LinearTerm>(-1.0, firstVariable));
            auxConstraintCounter++;

            auto auxConstraintBound2 = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_blbb_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintBound2->add(std::make_shared<LinearTerm>(1.0, auxVariable));
            auxConstraintBound2->add(std::make_shared<LinearTerm>(-1.0, secondVariable));
            auxConstraintCounter++;

            reformulatedProblem->add(std::move(auxVariable));
            reformulatedProblem->add(std::move(auxConstraint));
            reformulatedProblem->add(std::move(auxConstraintBound1));
            reformulatedProblem->add(std::move(auxConstraintBound2));

            auxVariableCounter++;
        }
        else if(T->isBilinear
            && (T->firstVariable->type == E_VariableType::Binary || T->secondVariable->type == E_VariableType::Binary))
        // Bilinear term b1*x2 or x1*b2
        {
            auto auxVariable = std::make_shared<AuxilliaryVariable>("s_blbc_" + std::to_string(auxVariableCounter),
                reformulatedProblem->allVariables.size(), E_VariableType::Real, -100000000.0, 100000000.0);
            auxVariable->auxilliaryType = E_AuxilliaryVariableType::BinaryContinuousOrIntegerBilinear;
            auxVariable->quadraticTerms.add(T);
            auxVariableCounter++;

            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

            auto binaryVariable
                = (T->firstVariable->type == E_VariableType::Binary) ? T->firstVariable : T->secondVariable;
            auto otherVariable
                = (T->firstVariable->type == E_VariableType::Binary) ? T->secondVariable : T->firstVariable;

            auto auxConstraint1 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, otherVariable->upperBound);
            auxConstraint1->add(std::make_shared<LinearTerm>(1.0, otherVariable));
            auxConstraint1->add(std::make_shared<LinearTerm>(secondVariable->upperBound, binaryVariable));
            auxConstraint1->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
            auxConstraintCounter++;

            auto auxConstraint2 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, -otherVariable->lowerBound);
            auxConstraint2->add(std::make_shared<LinearTerm>(-1.0, otherVariable));
            auxConstraint2->add(std::make_shared<LinearTerm>(-secondVariable->lowerBound, binaryVariable));
            auxConstraint2->add(std::make_shared<LinearTerm>(1.0, auxVariable));
            auxConstraintCounter++;

            auxVariable->lowerBound = std::min(0.0, otherVariable->lowerBound);
            auxVariable->upperBound = otherVariable->upperBound;

            reformulatedProblem->add(std::move(auxVariable));
            reformulatedProblem->add(std::move(auxConstraint1));
            reformulatedProblem->add(std::move(auxConstraint2));
        }
        else if(partitionNonBinaryTerms) // Square term x1^2 or general bilinear term x1*x2 will be partitioned into
                                         // multiple constraints
        {
            auto auxVariable = std::make_shared<AuxilliaryVariable>("s_blcc_" + std::to_string(auxVariableCounter),
                reformulatedProblem->allVariables.size(), E_VariableType::Real, -100000000.0, 100000000.0);
            auxVariable->auxilliaryType = E_AuxilliaryVariableType::NonlinearExpressionPartitioning;
            auxVariableCounter++;

            // auxVariable->lowerBound = firstVariable->lowerBound * secondVariable->lowerBound;
            // auxVariable->upperBound = firstVariable->upperBound * secondVariable->upperBound;

            /*
                        if(reversedSigns)
                        {
                            auxVariable->upperBound = firstVariable->lowerBound * secondVariable->lowerBound;
                            auxVariable->lowerBound = firstVariable->upperBound * secondVariable->upperBound;
                        }
                        else
                        {
                            auxVariable->lowerBound = firstVariable->lowerBound * secondVariable->lowerBound;
                            auxVariable->upperBound = firstVariable->upperBound * secondVariable->upperBound;
                        }*/

            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

            auto auxConstraint = std::make_shared<NonlinearConstraint>(
                auxConstraintCounter, "s_blcc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintCounter++;

            auxConstraint->add(std::make_shared<LinearTerm>(-1.0 * coeffSign * signfactor, auxVariable));

            auxConstraint->add(std::make_shared<QuadraticTerm>(coeffSign * signfactor, firstVariable, secondVariable));

            auxVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));

            if(env->settings->getBoolSetting("Reformulation.Bilinear.AddConvexEnvelope",
                   "Model")) // Also adds the McCormick envelopes to the dual model
            {
                auto auxConstraintU1 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                    "s_blmc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN,
                    firstVariable->lowerBound * secondVariable->lowerBound);
                auxConstraintU1->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
                auxConstraintU1->add(std::make_shared<LinearTerm>(firstVariable->lowerBound, secondVariable));
                auxConstraintU1->add(std::make_shared<LinearTerm>(secondVariable->lowerBound, firstVariable));
                auxConstraintCounter++;

                auto auxConstraintU2 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                    "s_blmc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN,
                    firstVariable->upperBound * secondVariable->upperBound);
                auxConstraintU2->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
                auxConstraintU2->add(std::make_shared<LinearTerm>(firstVariable->upperBound, secondVariable));
                auxConstraintU2->add(std::make_shared<LinearTerm>(secondVariable->upperBound, firstVariable));
                auxConstraintCounter++;

                auto auxConstraintU3 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                    "s_blmc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN,
                    -firstVariable->upperBound * secondVariable->lowerBound);
                auxConstraintU3->add(std::make_shared<LinearTerm>(1.0, auxVariable));
                auxConstraintU3->add(std::make_shared<LinearTerm>(-firstVariable->upperBound, secondVariable));
                auxConstraintU3->add(std::make_shared<LinearTerm>(-secondVariable->lowerBound, firstVariable));
                auxConstraintCounter++;

                auto auxConstraintU4 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                    "s_blmc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN,
                    firstVariable->lowerBound * secondVariable->upperBound);
                auxConstraintU4->add(std::make_shared<LinearTerm>(1.0, auxVariable));
                auxConstraintU4->add(std::make_shared<LinearTerm>(-firstVariable->lowerBound, secondVariable));
                auxConstraintU4->add(std::make_shared<LinearTerm>(-secondVariable->upperBound, firstVariable));
                auxConstraintCounter++;

                reformulatedProblem->add(std::move(auxConstraintU1));
                reformulatedProblem->add(std::move(auxConstraintU2));
                reformulatedProblem->add(std::move(auxConstraintU3));
                reformulatedProblem->add(std::move(auxConstraintU4));
            }

            reformulatedProblem->add(std::move(auxVariable));
            reformulatedProblem->add(std::move(auxConstraint));
        }
        else // Square term x1^2 or general bilinear term x1*x2 will remain as is
        {
            if(reversedSigns)
            {
                resultQuadraticTerms.add(
                    std::make_shared<QuadraticTerm>(-1.0 * T->coefficient, firstVariable, secondVariable));
            }
            else
            {
                resultQuadraticTerms.add(
                    std::make_shared<QuadraticTerm>(T->coefficient, firstVariable, secondVariable));
            }
        }
    }

    return std::tuple(resultLinearTerms, resultQuadraticTerms);
}

std::tuple<LinearTerms, MonomialTerms> TaskReformulateProblem::reformulateMonomialSum(
    const MonomialTerms& monomialTerms, bool reversedSigns)
{
    LinearTerms resultLinearTerms;
    MonomialTerms resultMonomialTerms;

    double signfactor = reversedSigns ? -1.0 : 1.0;

    bool allTermsAreBinary = true;

    for(auto& T : monomialTerms)
    {
        if(!T->isBinary)
        {
            allTermsAreBinary = false;
            break;
        }
    }

    for(auto& T : monomialTerms)
    {
        if(T->isBinary
            && env->settings->getIntSetting("Reformulation.Monomials.Formulation", "Model")
                == static_cast<int>(ES_ReformulationBinaryMonomials::Simple))
        {
            double N = T->variables.size();

            auto auxConstraint1 = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_mon1" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintCounter++;

            auto auxConstraint2 = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_mon2" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, N - 1.0);
            auxConstraintCounter++;

            auto auxbVar = std::make_shared<AuxilliaryVariable>("s_monb" + std::to_string(auxVariableCounter),
                reformulatedProblem->allVariables.size(), E_VariableType::Binary, 0.0, 1.0);
            auxVariableCounter++;

            auxbVar->monomialTerms.add(T);

            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxbVar));

            auxConstraint1->add(std::make_shared<LinearTerm>(N, auxbVar));
            auxConstraint2->add(std::make_shared<LinearTerm>(-1.0, auxbVar));

            for(auto& V : T->variables)
            {
                auxConstraint1->add(std::make_shared<LinearTerm>(-1.0, V));
                auxConstraint2->add(std::make_shared<LinearTerm>(1.0, V));
            }

            reformulatedProblem->add(std::move(auxbVar));
            reformulatedProblem->add(std::move(auxConstraint1));
            reformulatedProblem->add(std::move(auxConstraint2));
        }
        else if(T->isBinary
            && env->settings->getIntSetting("Reformulation.Monomials.Formulation", "Model")
                == static_cast<int>(ES_ReformulationBinaryMonomials::CostaLiberti))
        {
            int variableOffset = 0;
            int k = T->variables.size();

            Variables lambdas;

            auto auxLambdaSum = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_monlam" + std::to_string(auxConstraintCounter), 1.0, 1.0);
            auxConstraintCounter++;

            int numLambdas = std::pow(2, k);

            for(auto i = 1; numLambdas; i++)
            {
                auto auxLambda = std::make_shared<AuxilliaryVariable>("s_monlam" + std::to_string(auxVariableCounter),
                    reformulatedProblem->allVariables.size() + variableOffset, E_VariableType::Real, 0.0, 1.0);
                auxLambda->constant = 1.0 / ((double)numLambdas);

                auxLambdaSum->add(std::make_shared<LinearTerm>(1.0, auxLambda));
                lambdas.push_back(auxLambda);
                auxVariableCounter++;
                variableOffset++;
            }

            reformulatedProblem->add(std::move(auxLambdaSum));

            auto auxwVar = std::make_shared<AuxilliaryVariable>("s_monw" + std::to_string(auxVariableCounter),
                reformulatedProblem->allVariables.size() + variableOffset, E_VariableType::Real, SHOT_DBL_MIN,
                SHOT_DBL_MAX);
            auxwVar->constant = 1.0 / ((double)numLambdas);
            auxVariableCounter++;
            variableOffset++;

            auto auxwSum = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_monw" + std::to_string(auxConstraintCounter), 0.0, 0.0);
            auxConstraintCounter++;
            auxwSum->add(std::make_shared<LinearTerm>(-1.0, auxwVar));

            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxwVar));

            for(auto i = 1; i <= std::pow(2, k); i++)
            {
                double bProd = 1.0;

                for(int j = 1; j <= k; j++)
                {
                    double d = std::fmod(std::floor((i - 1.0) / std::pow(2, k - j)), 2.0);
                    double lowerBound = reformulatedProblem->getVariable(T->variables.at(j - 1)->index)->lowerBound;
                    double upperBound = reformulatedProblem->getVariable(T->variables.at(j - 1)->index)->upperBound;
                    bProd *= ((d == 0.0) ? lowerBound : upperBound);
                }

                if(bProd != 0.0)
                    auxwSum->add(std::make_shared<LinearTerm>(bProd, lambdas.at(i - 1)));
            }

            for(int j = 1; j <= k; j++)
            {
                auto auxxSum = std::make_shared<LinearConstraint>(
                    auxConstraintCounter, "s_monx" + std::to_string(auxConstraintCounter), 0.0, 0.0);
                auxConstraintCounter++;

                for(auto i = 1; i <= std::pow(2, k); i++)
                {
                    double d = std::fmod(std::floor((i - 1.0) / std::pow(2, k - j)), 2.0);
                    double lowerBound = reformulatedProblem->getVariable(T->variables.at(j - 1)->index)->lowerBound;
                    double upperBound = reformulatedProblem->getVariable(T->variables.at(j - 1)->index)->upperBound;
                    double b = (d == 0.0) ? lowerBound : upperBound;

                    if(b != 0.0)
                        auxxSum->add(std::make_shared<LinearTerm>(b, lambdas.at(i - 1)));
                }

                auxxSum->add(std::make_shared<LinearTerm>(
                    -1.0, reformulatedProblem->getVariable(T->variables.at(j - 1)->index)));

                reformulatedProblem->add(std::move(auxxSum));
            }

            for(auto& L : lambdas)
            {
                reformulatedProblem->add(std::move(L));
            }

            reformulatedProblem->add(std::move(auxwVar));
            reformulatedProblem->add(std::move(auxwSum));
        }
        else
        {
            resultMonomialTerms.add(T);
        }
    }

    return std::tuple(resultLinearTerms, resultMonomialTerms);
}

template <class T>
void TaskReformulateProblem::copyLinearTermsToConstraint(LinearTerms terms, T destination, bool reversedSigns)
{
    if(reversedSigns)
    {
        for(auto& LT : terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearConstraint>(destination)
                ->add(std::make_shared<LinearTerm>(-LT->coefficient, variable));
        }
    }
    else
    {
        for(auto& LT : terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearConstraint>(destination)
                ->add(std::make_shared<LinearTerm>(LT->coefficient, variable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToConstraint(QuadraticTerms terms, T destination, bool reversedSigns)
{
    if(reversedSigns)
    {
        for(auto& QT : terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticConstraint>(destination)
                ->add(std::make_shared<QuadraticTerm>(-QT->coefficient, firstVariable, secondVariable));
        }
    }
    else
    {
        for(auto& QT : terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticConstraint>(destination)
                ->add(std::make_shared<QuadraticTerm>(QT->coefficient, firstVariable, secondVariable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyLinearTermsToObjectiveFunction(LinearTerms terms, T destination, bool reversedSigns)
{
    if(reversedSigns)
    {
        for(auto& LT : terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearObjectiveFunction>(destination)
                ->add(std::make_shared<LinearTerm>(-LT->coefficient, variable));
        }
    }
    else
    {
        for(auto& LT : terms)
        {
            auto variable = reformulatedProblem->getVariable(LT->variable->index);

            std::dynamic_pointer_cast<LinearObjectiveFunction>(destination)
                ->add(std::make_shared<LinearTerm>(LT->coefficient, variable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToObjectiveFunction(
    QuadraticTerms terms, T destination, bool reversedSigns)
{
    if(reversedSigns)
    {
        for(auto& QT : terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(destination)
                ->add(std::make_shared<QuadraticTerm>(-QT->coefficient, firstVariable, secondVariable));
        }
    }
    else
    {
        for(auto& QT : terms)
        {
            auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(destination)
                ->add(std::make_shared<QuadraticTerm>(QT->coefficient, firstVariable, secondVariable));
        }
    }
}

NonlinearExpressionPtr TaskReformulateProblem::copyNonlinearExpression(
    NonlinearExpression* expression, const ProblemPtr destination)
{
    unsigned int i;
    std::ostringstream outStr;
    int numChildren;

    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Plus:
        return std::make_shared<ExpressionPlus>(
            copyNonlinearExpression(((ExpressionPlus*)expression)->firstChild.get(), destination),
            copyNonlinearExpression(((ExpressionPlus*)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Sum:
        numChildren = ((ExpressionSum*)expression)->getNumberOfChildren();
        switch(numChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return copyNonlinearExpression(((ExpressionSum*)expression)->children.get(0).get(), destination);
        default:
            NonlinearExpressions terms;
            for(i = 0; i < numChildren; i++)
                terms.expressions.push_back(
                    copyNonlinearExpression(((ExpressionSum*)expression)->children.get(i).get(), destination));
            return std::make_shared<ExpressionSum>(terms);
        }

    case E_NonlinearExpressionTypes::Minus:
        return std::make_shared<ExpressionMinus>(
            copyNonlinearExpression(((ExpressionMinus*)expression)->firstChild.get(), destination),
            copyNonlinearExpression(((ExpressionMinus*)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Negate:
        return std::make_shared<ExpressionNegate>(
            copyNonlinearExpression(((ExpressionNegate*)expression)->child.get(), destination));

    case E_NonlinearExpressionTypes::Times:
        return std::make_shared<ExpressionTimes>(
            copyNonlinearExpression(((ExpressionTimes*)expression)->firstChild.get(), destination),
            copyNonlinearExpression(((ExpressionTimes*)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Divide:
        return std::make_shared<ExpressionDivide>(
            copyNonlinearExpression(((ExpressionDivide*)expression)->firstChild.get(), destination),
            copyNonlinearExpression(((ExpressionDivide*)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Power:
        return std::make_shared<ExpressionPower>(
            copyNonlinearExpression(((ExpressionPower*)expression)->firstChild.get(), destination),
            copyNonlinearExpression(((ExpressionPower*)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Product:
        numChildren = ((ExpressionProduct*)expression)->getNumberOfChildren();
        switch(numChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return copyNonlinearExpression(((ExpressionProduct*)expression)->children.get(0).get(), destination);
        default:
            NonlinearExpressions factors;
            for(i = 0; i < numChildren; i++)
                factors.expressions.push_back(
                    copyNonlinearExpression(((ExpressionProduct*)expression)->children.get(i).get(), destination));
            return std::make_shared<ExpressionProduct>(factors);
        }

    case E_NonlinearExpressionTypes::Abs:
        return std::make_shared<ExpressionAbs>(
            copyNonlinearExpression((((ExpressionAbs*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Square:
        return std::make_shared<ExpressionSquare>(
            copyNonlinearExpression((((ExpressionSquare*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::SquareRoot:
        return std::make_shared<ExpressionSquareRoot>(
            copyNonlinearExpression((((ExpressionSquareRoot*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Log:
        return std::make_shared<ExpressionLog>(
            copyNonlinearExpression((((ExpressionLog*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Exp:
        return std::make_shared<ExpressionExp>(
            copyNonlinearExpression((((ExpressionExp*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Sin:
        return std::make_shared<ExpressionSin>(
            copyNonlinearExpression((((ExpressionSin*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Cos:
        return std::make_shared<ExpressionCos>(
            copyNonlinearExpression((((ExpressionCos*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Tan:
        return std::make_shared<ExpressionTan>(
            copyNonlinearExpression((((ExpressionTan*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Constant:
        return std::make_shared<ExpressionConstant>((((ExpressionConstant*)expression)->constant));

    case E_NonlinearExpressionTypes::Variable:
    {
        int variableIndex = ((ExpressionVariable*)expression)->variable->index;
        return std::make_shared<ExpressionVariable>(destination->getVariable(variableIndex));
    }
    default:
        throw new OperationNotImplementedException(std::to_string((int)(expression->getType())));
        break;
    }

    return nullptr;
}

} // namespace SHOT