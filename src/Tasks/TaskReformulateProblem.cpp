/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskReformulateProblem.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"
#include "../Timing.h"

#include "../Model/Simplifications.h"

namespace SHOT
{

TaskReformulateProblem::TaskReformulateProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("ProblemReformulation");

    auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
        env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"));

    useQuadraticConstraints = (quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained);

    useQuadraticObjective
        = (useQuadraticConstraints || quadraticStrategy == ES_QuadraticProblemStrategy::QuadraticObjective);

    quadraticObjectiveRegardedAsNonlinear = false;

    PartitionQuadraticTermsInObjective
        = env->settings->getSetting<bool>("Reformulation.ObjectiveFunction.PartitionQuadraticTerms", "Model");

    PartitionQuadraticTermsInConstraint
        = env->settings->getSetting<bool>("Reformulation.Constraint.PartitionQuadraticTerms", "Model");

    auxVariableCounter = env->problem->properties.numberOfVariables;
    auxConstraintCounter = env->problem->properties.numberOfNumericConstraints;

    reformulatedProblem = std::make_shared<Problem>(env);
    reformulatedProblem->name = env->problem->name + " (reformulated)";

    reformulatedProblem->variableLowerBounds = env->problem->variableLowerBounds;
    reformulatedProblem->variableUpperBounds = env->problem->variableUpperBounds;

    // Copying variables
    for(auto& V : env->problem->allVariables)
    {
        auto variable = std::make_shared<Variable>(V->name, V->index, V->properties.type, V->lowerBound, V->upperBound);
        reformulatedProblem->add(std::move(variable));
    }

    // Reformulating constraints
    for(auto& C : env->problem->numericConstraints)
    {
        auto reformulatedConstraints = reformulateConstraint(C);

        for(auto& RC : reformulatedConstraints)
        {
            reformulatedProblem->add(std::move(RC));
        }
    }

    // Reformulating objective function
    reformulateObjectiveFunction();

    if(env->settings->getSetting<bool>("Reformulation.Bilinear.AddConvexEnvelope",
           "Model")) // Also adds the McCormick envelopes to the dual model
    {
        for(auto& VAR : bilinearAuxVariables)
        {
            addBilinearMcCormickEnvelope(VAR.second, std::get<0>(VAR.first), std::get<1>(VAR.first));
        }
    }

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

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        std::stringstream filename;
        filename << env->settings->getSetting<std::string>("Debug.Path", "Output");
        filename << "/reformulatedproblem";
        filename << ".txt";

        std::stringstream problem;
        problem << env->reformulatedProblem;

        Utilities::writeStringToFile(filename.str(), problem.str());
    }

    env->timing->stopTimer("ProblemReformulation");
}

TaskReformulateProblem::~TaskReformulateProblem() = default;

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
        && env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic)
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

    if(env->problem->objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic)
    {
        bool useEpigraph = env->settings->getSetting<bool>("Reformulation.ObjectiveFunction.Epigraph.Use", "Model");

        if(useEpigraph)
        {
            double objVarBound = env->settings->getSetting<double>("NonlinearObjectiveVariable.Bound", "Model");

            Interval objectiveBound;

            try
            {
                objectiveBound = env->problem->objectiveFunction->getBounds();
            }
            catch(mc::Interval::Exceptions& e)
            {
                objectiveBound = Interval(-objVarBound, objVarBound);
            }

            auto objectiveVariable = std::make_shared<AuxiliaryVariable>(
                "shot_objvar", auxVariableCounter, E_VariableType::Real, objectiveBound.l(), objectiveBound.u());
            objectiveVariable->properties.auxiliaryType = E_AuxiliaryVariableType::NonlinearObjectiveFunction;

            if(env->problem->objectiveFunction->properties.hasLinearTerms)
            {
                for(auto& T :
                    std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms)
                {
                    objectiveVariable->linearTerms.add(std::make_shared<LinearTerm>(
                        T->coefficient, reformulatedProblem->getVariable(T->variable->index)));
                }
            }

            if(env->problem->objectiveFunction->properties.hasQuadraticTerms)
            {
                for(auto& T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)
                                  ->quadraticTerms)
                {
                    objectiveVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(T->coefficient,
                        reformulatedProblem->getVariable(T->firstVariable->index),
                        reformulatedProblem->getVariable(T->secondVariable->index)));
                }
            }

            if(env->problem->objectiveFunction->properties.hasMonomialTerms)
            {
                for(auto& T : std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                                  ->monomialTerms)
                {
                    objectiveVariable->monomialTerms.add(std::make_shared<MonomialTerm>(T.get(), reformulatedProblem));
                }
            }

            if(env->problem->objectiveFunction->properties.hasSignomialTerms)
            {
                for(auto& T : std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                                  ->signomialTerms)
                {
                    objectiveVariable->signomialTerms.add(
                        std::make_shared<SignomialTerm>(T.get(), reformulatedProblem));
                }
            }

            if(env->problem->objectiveFunction->properties.hasNonlinearExpression)
            {
                objectiveVariable->nonlinearExpression
                    = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                          ->nonlinearExpression;
            }

            bool isSignReversed = env->problem->objectiveFunction->properties.isMaximize;
            double signfactor = (env->problem->objectiveFunction->properties.isMinimize) ? 1.0 : -1.0;

            // Adding new linear objective function
            auto objective = std::make_shared<LinearObjectiveFunction>();
            objective->direction = E_ObjectiveFunctionDirection::Minimize;
            objective->constant = 0.0;

            objective->add(std::make_shared<LinearTerm>(1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));

            // Adding the auxiliary objective constraint
            auto constraint = std::make_shared<NonlinearConstraint>(reformulatedProblem->numericConstraints.size(),
                "shot_objconstr", SHOT_DBL_MIN, -1.0 * signfactor * env->problem->objectiveFunction->constant);

            if(env->problem->objectiveFunction->properties.hasLinearTerms)
            {
                copyLinearTermsToConstraint(
                    std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms,
                    constraint, isSignReversed);
            }

            if(env->problem->objectiveFunction->properties.hasQuadraticTerms)
            {
                copyQuadraticTermsToConstraint(
                    std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)
                        ->quadraticTerms,
                    constraint);
            }

            if(env->problem->objectiveFunction->properties.hasMonomialTerms)
            {
                copyMonomialTermsToConstraint(
                    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                        ->monomialTerms,
                    constraint, isSignReversed);
            }

            if(env->problem->objectiveFunction->properties.hasSignomialTerms)
            {
                copySignomialTermsToConstraint(
                    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                        ->signomialTerms,
                    constraint, isSignReversed);
            }

            if(env->problem->objectiveFunction->properties.hasNonlinearExpression)
            {
                if(isSignReversed)
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
            }

            reformulatedProblem->add(objectiveVariable);
            constraint->add(std::make_shared<LinearTerm>(-1.0, std::dynamic_pointer_cast<Variable>(objectiveVariable)));
            auto reformulatedConstraints = reformulateConstraint(constraint);

            for(auto& RC : reformulatedConstraints)
            {
                reformulatedProblem->add(std::move(RC));
            }

            objectiveVariable->index = auxVariableCounter;
            reformulatedProblem->add(std::move(objective));

            return;
        }
    }

    // Objective is to be regarded as nonlinear

    bool copyOriginalLinearTerms = false;
    bool copyOriginalNonlinearExpression = false;

    // These will be added to the new constraint, and their signs have been altered
    LinearTerms destinationLinearTerms;
    QuadraticTerms destinationQuadraticTerms;
    MonomialTerms destinationMonomialTerms;
    SignomialTerms destinationSignomialTerms;

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

    if(env->problem->objectiveFunction->properties.hasMonomialTerms)
    {
        auto sourceObjective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction);

        if(static_cast<ES_PartitionNonlinearSums>(
               env->settings->getSetting<int>("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::Always
            && sourceObjective->monomialTerms.size() > 1)
        {
            auto tmpLinearTerms = partitionMonomialTerms(sourceObjective->monomialTerms, isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else // Monomials are always nonconvex
        {
            for(auto& T : sourceObjective->monomialTerms)
                destinationMonomialTerms.add(std::make_shared<MonomialTerm>(T.get(), reformulatedProblem));
        }
    }

    if(env->problem->objectiveFunction->properties.hasSignomialTerms)
    {
        auto sourceObjective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction);

        if(static_cast<ES_PartitionNonlinearSums>(
               env->settings->getSetting<int>("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::Always
            && sourceObjective->signomialTerms.size() > 1)
        {
            auto tmpLinearTerms = partitionSignomialTerms(sourceObjective->signomialTerms, isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else if(static_cast<ES_PartitionNonlinearSums>(
                    env->settings->getSetting<int>("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::IfConvex
            && sourceObjective->signomialTerms.size() > 1)
        {
            bool areAllConvex = false;

            if(!isSignReversed && sourceObjective->signomialTerms.checkAllForConvexityType(E_Convexity::Convex))
                areAllConvex = true;
            else if(isSignReversed && sourceObjective->signomialTerms.checkAllForConvexityType(E_Convexity::Concave))
                areAllConvex = true;

            if(areAllConvex)
            {
                auto tmpLinearTerms = partitionSignomialTerms(sourceObjective->signomialTerms, isSignReversed);
                destinationLinearTerms.add(tmpLinearTerms);
            }
            else
            {
                for(auto& T : sourceObjective->signomialTerms)
                    destinationSignomialTerms.add(std::make_shared<SignomialTerm>(T.get(), reformulatedProblem));
            }
        }
        else
        {
            for(auto& T : sourceObjective->signomialTerms)
                destinationSignomialTerms.add(std::make_shared<SignomialTerm>(T.get(), reformulatedProblem));
        }
    }

    if(env->problem->objectiveFunction->properties.hasNonlinearExpression)
    {
        auto sourceObjective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction);

        if(static_cast<ES_PartitionNonlinearSums>(
               env->settings->getSetting<int>("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::Always
            && sourceObjective->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto tmpLinearTerms = partitionNonlinearSum(
                std::dynamic_pointer_cast<ExpressionSum>(sourceObjective->nonlinearExpression), isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else if(static_cast<ES_PartitionNonlinearSums>(
                    env->settings->getSetting<int>("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::IfConvex
            && sourceObjective->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto sum = std::dynamic_pointer_cast<ExpressionSum>(sourceObjective->nonlinearExpression);
            bool areAllConvex = false;

            if(!isSignReversed && sum->checkAllForConvexityType(E_Convexity::Convex))
                areAllConvex = true;
            else if(isSignReversed && sum->checkAllForConvexityType(E_Convexity::Concave))
                areAllConvex = true;

            if(areAllConvex)
            {
                auto tmpLinearTerms = partitionNonlinearSum(sum, isSignReversed);
                destinationLinearTerms.add(tmpLinearTerms);
            }
            else
            {
                copyOriginalNonlinearExpression = true;
            }
        }
        else
        {
            copyOriginalNonlinearExpression = true;
        }
    }

    ObjectiveFunctionPtr objective;

    if(copyOriginalNonlinearExpression || destinationMonomialTerms.size() > 0 || destinationSignomialTerms.size() > 0)
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

    if(destinationMonomialTerms.size() > 0)
        std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)->add(destinationMonomialTerms);

    if(destinationSignomialTerms.size() > 0)
        std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)->add(destinationSignomialTerms);

    if(copyOriginalNonlinearExpression || destinationMonomialTerms.size() > 0 || destinationSignomialTerms.size() > 0)
    {
        auto sourceObjective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction);

        if(sourceObjective->properties.hasNonlinearExpression)
        {
            if(isSignReversed)
                std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)->add(
                    simplify(std::make_shared<ExpressionNegate>(
                        copyNonlinearExpression(sourceObjective->nonlinearExpression.get(), reformulatedProblem))));
            else
                std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)->add(
                    copyNonlinearExpression(sourceObjective->nonlinearExpression.get(), reformulatedProblem));
        }
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

NumericConstraints TaskReformulateProblem::reformulateConstraint(NumericConstraintPtr C)
{
    double valueLHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueLHS;
    double valueRHS = std::dynamic_pointer_cast<NumericConstraint>(C)->valueRHS;
    double constant = std::dynamic_pointer_cast<NumericConstraint>(C)->constant;

    if(C->properties.classification == E_ConstraintClassification::Linear
        || (!C->properties.hasNonlinearExpression && !C->properties.hasQuadraticTerms && !C->properties.hasMonomialTerms
               && !C->properties.hasSignomialTerms))
    {
        // Linear constraint
        LinearConstraintPtr constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Linear;
        auto sourceConstraint = std::dynamic_pointer_cast<LinearConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);

        constraint->constant = constant;

        return (NumericConstraints({ constraint }));
    }

    if(useQuadraticConstraints
        && (C->properties.classification == E_ConstraintClassification::Quadratic
               || (!C->properties.hasNonlinearExpression && !C->properties.hasMonomialTerms
                      && !C->properties.hasSignomialTerms)))
    {
        // Quadratic constraint
        QuadraticConstraintPtr constraint
            = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Quadratic;
        auto sourceConstraint = std::dynamic_pointer_cast<QuadraticConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);
        copyQuadraticTermsToConstraint(sourceConstraint->quadraticTerms, constraint);

        constraint->constant = constant;

        return (NumericConstraints({ constraint }));
    }

    // Constraint is to be regarded as nonlinear

    bool copyOriginalLinearTerms = false;
    bool copyOriginalNonlinearExpression = false;

    // These will be added to the new constraint, and their signs have been altered
    LinearTerms destinationLinearTerms;
    QuadraticTerms destinationQuadraticTerms;
    MonomialTerms destinationMonomialTerms;
    SignomialTerms destinationSignomialTerms;

    bool isSignReversed = false;

    if(valueLHS == valueRHS)
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

        if(C->properties.hasMonomialTerms)
            copyMonomialTermsToConstraint(
                std::dynamic_pointer_cast<NonlinearConstraint>(C)->monomialTerms, auxConstraint1);

        if(C->properties.hasSignomialTerms)
            copySignomialTermsToConstraint(
                std::dynamic_pointer_cast<NonlinearConstraint>(C)->signomialTerms, auxConstraint1);

        if(C->properties.hasNonlinearExpression)
            auxConstraint1->add(copyNonlinearExpression(
                std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), reformulatedProblem));

        auto reformulatedConstraint1 = reformulateConstraint(auxConstraint1);

        if(reformulatedConstraint1.size() > 1)
            return (reformulatedConstraint1);

        if(reformulatedConstraint1.at(0)->properties.classification == E_ConstraintClassification::Linear)
        {
            reformulatedConstraint1.at(0)->valueLHS = reformulatedConstraint1.at(0)->valueRHS;
            return (reformulatedConstraint1);
        }

        // Will rewrite it as (f(x))^2 <= 0

        auto nonlinearConstraint = std::make_shared<NonlinearConstraint>(auxConstraintCounter, C->name, SHOT_DBL_MIN,
            reformulatedConstraint1.at(0)->valueRHS * reformulatedConstraint1.at(0)->valueRHS);

        nonlinearConstraint->properties.classification = E_ConstraintClassification::Nonlinear;

        if(reformulatedConstraint1.at(0)->properties.hasLinearTerms)
        {
            for(auto& LT : std::dynamic_pointer_cast<LinearConstraint>(reformulatedConstraint1.at(0))->linearTerms)
            {
                if(LT->coefficient == 1.0)
                    nonlinearConstraint->add(std::make_shared<ExpressionVariable>(LT->variable));
                else
                {
                    nonlinearConstraint->add(
                        std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(LT->coefficient),
                            std::make_shared<ExpressionVariable>(LT->variable)));
                }
            }
        }

        if(reformulatedConstraint1.at(0)->properties.hasQuadraticTerms)
        {
            for(auto& QT :
                std::dynamic_pointer_cast<QuadraticConstraint>(reformulatedConstraint1.at(0))->quadraticTerms)
            {
                NonlinearExpressions product;

                if(QT->coefficient != 1.0)
                {
                    product.push_back(std::make_shared<ExpressionConstant>(QT->coefficient));
                }

                product.push_back(std::make_shared<ExpressionVariable>(QT->firstVariable));
                product.push_back(std::make_shared<ExpressionVariable>(QT->secondVariable));

                nonlinearConstraint->add(std::make_shared<ExpressionProduct>(product));
            }
        }
        // TODO add monomials and signomials

        if(reformulatedConstraint1.at(0)->properties.hasNonlinearExpression)
        {
            nonlinearConstraint->add(
                copyNonlinearExpression(std::dynamic_pointer_cast<NonlinearConstraint>(reformulatedConstraint1.at(0))
                                            ->nonlinearExpression.get(),
                    reformulatedProblem));
        }

        nonlinearConstraint->nonlinearExpression
            = std::make_shared<ExpressionSquare>(nonlinearConstraint->nonlinearExpression);

        nonlinearConstraint->properties.convexity = E_Convexity::Nonconvex;
        return (NumericConstraints({ nonlinearConstraint }));
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

    if(C->properties.hasMonomialTerms)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

        if(static_cast<ES_PartitionNonlinearSums>(
               env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::Always
            && sourceConstraint->monomialTerms.size() > 1)
        {
            auto tmpLinearTerms = partitionMonomialTerms(sourceConstraint->monomialTerms, isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else // Monomials are always nonconvex
        {
            for(auto& T : sourceConstraint->monomialTerms)
                destinationMonomialTerms.add(std::make_shared<MonomialTerm>(T.get(), reformulatedProblem));
        }
    }

    if(C->properties.hasSignomialTerms)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

        if(static_cast<ES_PartitionNonlinearSums>(
               env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::Always
            && sourceConstraint->signomialTerms.size() > 1)
        {
            auto tmpLinearTerms = partitionSignomialTerms(sourceConstraint->signomialTerms, isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else if(static_cast<ES_PartitionNonlinearSums>(
                    env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::IfConvex
            && sourceConstraint->signomialTerms.size() > 1)
        {
            bool areAllConvex = false;

            if(!isSignReversed && sourceConstraint->signomialTerms.checkAllForConvexityType(E_Convexity::Convex))
                areAllConvex = true;
            else if(isSignReversed && sourceConstraint->signomialTerms.checkAllForConvexityType(E_Convexity::Concave))
                areAllConvex = true;

            if(areAllConvex)
            {
                auto tmpLinearTerms = partitionSignomialTerms(sourceConstraint->signomialTerms, isSignReversed);
                destinationLinearTerms.add(tmpLinearTerms);
            }
            else
            {
                for(auto& T : sourceConstraint->signomialTerms)
                    destinationSignomialTerms.add(std::make_shared<SignomialTerm>(T.get(), reformulatedProblem));
            }
        }
        else
        {
            for(auto& T : sourceConstraint->signomialTerms)
                destinationSignomialTerms.add(std::make_shared<SignomialTerm>(T.get(), reformulatedProblem));
        }
    }

    if(C->properties.hasNonlinearExpression)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

        if(static_cast<ES_PartitionNonlinearSums>(
               env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::Always
            && sourceConstraint->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto tmpLinearTerms = partitionNonlinearSum(
                std::dynamic_pointer_cast<ExpressionSum>(sourceConstraint->nonlinearExpression), isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else if(static_cast<ES_PartitionNonlinearSums>(
                    env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::IfConvex
            && sourceConstraint->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto sum = std::dynamic_pointer_cast<ExpressionSum>(sourceConstraint->nonlinearExpression);
            bool areAllConvex = false;

            if(!isSignReversed && sum->checkAllForConvexityType(E_Convexity::Convex))
                areAllConvex = true;
            else if(isSignReversed && sum->checkAllForConvexityType(E_Convexity::Concave))
                areAllConvex = true;

            if(areAllConvex)
            {
                auto tmpLinearTerms = partitionNonlinearSum(sum, isSignReversed);
                destinationLinearTerms.add(tmpLinearTerms);
            }
            else
            {
                copyOriginalNonlinearExpression = true;
            }
        }
        else
        {
            copyOriginalNonlinearExpression = true;
        }
    }

    NumericConstraintPtr constraint;

    if(copyOriginalNonlinearExpression || destinationMonomialTerms.size() > 0 || destinationSignomialTerms.size() > 0)
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Nonlinear;
    }
    else if(destinationQuadraticTerms.size() > 0 && !useQuadraticConstraints)
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Nonlinear;
    }
    else if(destinationQuadraticTerms.size() > 0 && useQuadraticConstraints)
    {
        constraint = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::QuadraticConsideredAsNonlinear;
    }
    else if(destinationQuadraticTerms.size() > 0)
    {
        constraint = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Quadratic;
    }
    else
    {
        constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Linear;
    }

    constraint->constant = constant;

    if(copyOriginalLinearTerms)
        copyLinearTermsToConstraint(std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms,
            std::dynamic_pointer_cast<LinearConstraint>(constraint), isSignReversed);

    if(destinationLinearTerms.size() > 0)
        std::dynamic_pointer_cast<LinearConstraint>(constraint)->add(destinationLinearTerms);

    if(destinationQuadraticTerms.size() > 0)
        std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->add(destinationQuadraticTerms);

    if(destinationMonomialTerms.size() > 0)
        std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(destinationMonomialTerms);

    if(destinationSignomialTerms.size() > 0)
        std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(destinationSignomialTerms);

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

    return (NumericConstraints({ constraint }));
}

LinearTerms TaskReformulateProblem::partitionNonlinearSum(
    const std::shared_ptr<ExpressionSum> source, bool reversedSigns)
{
    LinearTerms resultLinearTerms;

    if(source.get() == nullptr)
        return (resultLinearTerms);

    bool allNonlinearExpressionsReformulated = false;

    for(auto& T : source->children)
    {
        if(T->getType() == E_NonlinearExpressionTypes::Product) // Might be able to reuse auxiliary variable
                                                                // further if e.g. bilinear term
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
                && env->settings->getSetting<int>("Reformulation.Monomials.Formulation", "Model")
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
            Interval bounds;

            double varLowerBound = env->settings->getSetting<double>("ContinuousVariable.MinimumLowerBound", "Model");
            double varUpperBound = env->settings->getSetting<double>("ContinuousVariable.MaximumUpperBound", "Model");

            try
            {
                bounds = T->getBounds();
            }
            catch(mc::Interval::Exceptions& e)
            {
                bounds = Interval(varLowerBound, varUpperBound);
            }

            auto auxVariable = std::make_shared<AuxiliaryVariable>("s_pnl_" + std::to_string(auxVariableCounter + 1),
                auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
            auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::NonlinearExpressionPartitioning;
            auxVariableCounter++;

            resultLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

            auto auxConstraint = std::make_shared<NonlinearConstraint>(
                auxConstraintCounter, "s_pnl_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
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

LinearTerms TaskReformulateProblem::partitionMonomialTerms(const MonomialTerms sourceTerms, bool reversedSigns)
{
    LinearTerms resultLinearTerms;

    if(sourceTerms.size() == 0)
        return (resultLinearTerms);

    bool allNonlinearExpressionsReformulated = false;

    for(auto& T : sourceTerms)
    {
        Interval bounds;

        double varLowerBound = env->settings->getSetting<double>("ContinuousVariable.MinimumLowerBound", "Model");
        double varUpperBound = env->settings->getSetting<double>("ContinuousVariable.MaximumUpperBound", "Model");

        try
        {
            bounds = T->getBounds();
        }
        catch(mc::Interval::Exceptions& e)
        {
            bounds = Interval(varLowerBound, varUpperBound);
        }

        auto auxVariable = std::make_shared<AuxiliaryVariable>("s_pmon_" + std::to_string(auxVariableCounter + 1),
            auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::MonomialTermsPartitioning;
        auxVariableCounter++;

        resultLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

        auto auxConstraint = std::make_shared<NonlinearConstraint>(
            auxConstraintCounter, "s_pmon_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
        auxConstraintCounter++;

        auto monomialTerm = std::make_shared<MonomialTerm>(T.get(), reformulatedProblem);

        if(reversedSigns)
            monomialTerm->coefficient *= -1.0;

        auxConstraint->add(monomialTerm);

        auxVariable->monomialTerms.push_back(monomialTerm);

        reformulatedProblem->add(std::move(auxVariable));
        reformulatedProblem->add(std::move(auxConstraint));
    }

    return (resultLinearTerms);
}

LinearTerms TaskReformulateProblem::partitionSignomialTerms(const SignomialTerms sourceTerms, bool reversedSigns)
{
    LinearTerms resultLinearTerms;

    if(sourceTerms.size() == 0)
        return (resultLinearTerms);

    bool allNonlinearExpressionsReformulated = false;

    for(auto& T : sourceTerms)
    {
        Interval bounds;

        double varLowerBound = env->settings->getSetting<double>("ContinuousVariable.MinimumLowerBound", "Model");
        double varUpperBound = env->settings->getSetting<double>("ContinuousVariable.MaximumUpperBound", "Model");

        try
        {
            bounds = T->getBounds();
        }
        catch(mc::Interval::Exceptions& e)
        {
            bounds = Interval(varLowerBound, varUpperBound);
        }

        auto auxVariable = std::make_shared<AuxiliaryVariable>("s_psig_" + std::to_string(auxVariableCounter + 1),
            auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::SignomialTermsPartitioning;
        auxVariableCounter++;

        resultLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

        auto auxConstraint = std::make_shared<NonlinearConstraint>(
            auxConstraintCounter, "s_psig_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
        auxConstraintCounter++;

        auto signomialTerm = std::make_shared<SignomialTerm>(T.get(), reformulatedProblem);

        if(reversedSigns)
        {
            signomialTerm->coefficient *= -1.0;
        }
        /*
                if(signomialTerm->coefficient < 0)
                {
                    auxVariable->upperBound = 0.0;
                }
                else
                {
                    auxVariable->lowerBound = 0.0;
                }*/

        auxConstraint->add(signomialTerm);

        auxVariable->signomialTerms.push_back(signomialTerm);

        reformulatedProblem->add(std::move(auxVariable));
        reformulatedProblem->add(std::move(auxConstraint));
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
            auto auxVariable = getBilinearAuxiliaryVariable(firstVariable, secondVariable);

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

            reformulatedProblem->add(std::move(auxConstraint));
            reformulatedProblem->add(std::move(auxConstraintBound1));
            reformulatedProblem->add(std::move(auxConstraintBound2));
        }
        else if(T->isBilinear
            && (T->firstVariable->properties.type == E_VariableType::Binary
                   || T->secondVariable->properties.type == E_VariableType::Binary))
        // Bilinear term b1*x2 or x1*b2
        {
            auto auxVariable = getBilinearAuxiliaryVariable(firstVariable, secondVariable);

            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

            auto binaryVariable
                = (T->firstVariable->properties.type == E_VariableType::Binary) ? T->firstVariable : T->secondVariable;
            auto otherVariable
                = (T->firstVariable->properties.type == E_VariableType::Binary) ? T->secondVariable : T->firstVariable;

            auto auxConstraint1 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, otherVariable->upperBound);
            auxConstraint1->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
            auxConstraint1->add(std::make_shared<LinearTerm>(1.0, otherVariable));
            auxConstraint1->add(std::make_shared<LinearTerm>(otherVariable->upperBound, binaryVariable));
            auxConstraintCounter++;

            auto auxConstraint2 = std::make_shared<LinearConstraint>(auxConstraintCounter,
                "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, otherVariable->upperBound);
            auxConstraint2->add(std::make_shared<LinearTerm>(1.0, auxVariable));
            auxConstraint2->add(std::make_shared<LinearTerm>(-1.0, otherVariable));
            auxConstraint2->add(std::make_shared<LinearTerm>(otherVariable->upperBound, binaryVariable));
            auxConstraintCounter++;

            reformulatedProblem->add(std::move(auxConstraint1));
            reformulatedProblem->add(std::move(auxConstraint2));
        }
        else if(T->isBilinear && T->firstVariable->properties.type == E_VariableType::Integer
            && T->secondVariable->properties.type == E_VariableType::Integer && T->firstVariable->lowerBound >= 0
            && T->secondVariable->lowerBound >= 0 && T->firstVariable->upperBound <= 100
            && T->secondVariable->upperBound <= 100)
        // bilinear term i1*i2
        {
            if(env->settings->getSetting<int>("Reformulation.Bilinear.IntegerFormulation", "Model")
                == static_cast<int>(ES_ReformulatiomBilinearInteger::TwoDiscretization))
            {
                auto auxVariable = getBilinearAuxiliaryVariable(T->firstVariable, T->secondVariable);

                resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

                Variables firstBinaries;
                Variables secondBinaries;

                auto auxFirstSum = std::make_shared<LinearConstraint>(
                    auxConstraintCounter, "s_bli" + std::to_string(auxConstraintCounter), 1.0, 1.0);
                auxConstraintCounter++;

                auto auxFirstSumVarDef = std::make_shared<LinearConstraint>(
                    auxConstraintCounter, "s_blx" + std::to_string(auxConstraintCounter), 0, 0);
                auxConstraintCounter++;

                auxFirstSumVarDef->add(std::make_shared<LinearTerm>(-1.0, firstVariable));

                for(auto i = 0; i <= firstVariable->upperBound; i++)
                {
                    auto auxBinary
                        = std::make_shared<AuxiliaryVariable>("s_bli" + std::to_string(auxVariableCounter + 1),
                            auxVariableCounter, E_VariableType::Binary, 0.0, 1.0);

                    auxFirstSum->add(std::make_shared<LinearTerm>(1.0, auxBinary));
                    auxFirstSumVarDef->add(std::make_shared<LinearTerm>(i, auxBinary));

                    firstBinaries.push_back(auxBinary);
                    auxVariableCounter++;
                }

                auto auxSecondSum = std::make_shared<LinearConstraint>(
                    auxConstraintCounter, "s_blj" + std::to_string(auxConstraintCounter), 1.0, 1.0);
                auxConstraintCounter++;

                auto auxSecondSumVarDef = std::make_shared<LinearConstraint>(
                    auxConstraintCounter, "s_bly" + std::to_string(auxConstraintCounter), 0, 0);
                auxConstraintCounter++;

                auxSecondSumVarDef->add(std::make_shared<LinearTerm>(-1.0, secondVariable));

                for(auto i = 0; i <= secondVariable->upperBound; i++)
                {
                    auto auxBinary
                        = std::make_shared<AuxiliaryVariable>("s_blj" + std::to_string(auxVariableCounter + 1),
                            auxVariableCounter, E_VariableType::Binary, 0.0, 1.0);

                    auxSecondSum->add(std::make_shared<LinearTerm>(1.0, auxBinary));
                    auxSecondSumVarDef->add(std::make_shared<LinearTerm>(i, auxBinary));

                    secondBinaries.push_back(auxBinary);
                    auxVariableCounter++;
                }

                for(auto i = 0; i <= firstVariable->upperBound; i++)
                {
                    for(auto j = 0; j <= secondVariable->upperBound; j++)
                    {
                        double ijprod = -1.0 * coeffSign * signfactor * i * j;

                        if(coeffSign * signfactor == -1.0)
                        {
                            auto auxConstraint = std::make_shared<LinearConstraint>(auxConstraintCounter,
                                "s_blwn" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN,
                                -ijprod + 2 * firstVariable->upperBound * secondVariable->upperBound);

                            auxConstraintCounter++;

                            auxConstraint->add(std::make_shared<LinearTerm>(1.0, auxVariable));
                            auxConstraint->add(std::make_shared<LinearTerm>(
                                firstVariable->upperBound * secondVariable->upperBound - ijprod, firstBinaries[i]));
                            auxConstraint->add(std::make_shared<LinearTerm>(
                                firstVariable->upperBound * secondVariable->upperBound - ijprod, secondBinaries[j]));

                            reformulatedProblem->add(std::move(auxConstraint));
                        }
                        else
                        {
                            auto auxConstraint = std::make_shared<LinearConstraint>(auxConstraintCounter,
                                "s_blwp" + std::to_string(auxConstraintCounter), ijprod, SHOT_DBL_MAX);

                            auxConstraintCounter++;

                            auxConstraint->add(std::make_shared<LinearTerm>(1.0, auxVariable));
                            auxConstraint->add(std::make_shared<LinearTerm>(ijprod, firstBinaries[i]));
                            auxConstraint->add(std::make_shared<LinearTerm>(ijprod, secondBinaries[j]));

                            reformulatedProblem->add(std::move(auxConstraint));
                        }
                    }
                }

                for(auto& V : firstBinaries)
                    reformulatedProblem->add(std::move(V));

                for(auto& V : secondBinaries)
                    reformulatedProblem->add(std::move(V));

                reformulatedProblem->add(std::move(auxFirstSum));
                reformulatedProblem->add(std::move(auxFirstSumVarDef));
                reformulatedProblem->add(std::move(auxSecondSum));
                reformulatedProblem->add(std::move(auxSecondSumVarDef));
            }
            else if(env->settings->getSetting<int>("Reformulation.Bilinear.IntegerFormulation", "Model")
                == static_cast<int>(ES_ReformulatiomBilinearInteger::OneDiscretization))
            {
                VariablePtr discretizationVariable;
                VariablePtr nonDiscretizationVariable;
                Variables discretizationBinaries;

                bool foundFirstVariable
                    = (integerAuxiliaryBinaryVariables.find(firstVariable) != integerAuxiliaryBinaryVariables.end());
                bool foundSecondVariable
                    = (integerAuxiliaryBinaryVariables.find(secondVariable) != integerAuxiliaryBinaryVariables.end());

                if(foundFirstVariable && foundSecondVariable)
                {
                    discretizationVariable
                        = (firstVariable->upperBound < secondVariable->upperBound) ? firstVariable : secondVariable;
                    nonDiscretizationVariable
                        = (firstVariable->upperBound > secondVariable->upperBound) ? firstVariable : secondVariable;

                    discretizationBinaries = integerAuxiliaryBinaryVariables[discretizationVariable];
                }
                else if(foundFirstVariable)
                {
                    discretizationVariable = firstVariable;
                    nonDiscretizationVariable = secondVariable;

                    discretizationBinaries = integerAuxiliaryBinaryVariables[discretizationVariable];
                }
                else if(foundSecondVariable)
                {
                    discretizationVariable = secondVariable;
                    nonDiscretizationVariable = firstVariable;

                    discretizationBinaries = integerAuxiliaryBinaryVariables[discretizationVariable];
                }
                else
                {
                    discretizationVariable
                        = (firstVariable->upperBound < secondVariable->upperBound) ? firstVariable : secondVariable;

                    nonDiscretizationVariable
                        = (firstVariable->upperBound > secondVariable->upperBound) ? firstVariable : secondVariable;

                    auto auxFirstSum = std::make_shared<LinearConstraint>(
                        auxConstraintCounter, "s_bli" + std::to_string(auxConstraintCounter), 1.0, 1.0);
                    auxConstraintCounter++;

                    auto auxFirstSumVarDef = std::make_shared<LinearConstraint>(
                        auxConstraintCounter, "s_blx" + std::to_string(auxConstraintCounter), 0, 0);
                    auxConstraintCounter++;

                    auxFirstSumVarDef->add(std::make_shared<LinearTerm>(-1.0, discretizationVariable));

                    for(auto i = 0; i <= discretizationVariable->upperBound; i++)
                    {
                        auto auxBinary
                            = std::make_shared<AuxiliaryVariable>("s_bli" + std::to_string(auxVariableCounter + 1),
                                auxVariableCounter, E_VariableType::Binary, 0.0, 1.0);

                        auxFirstSum->add(std::make_shared<LinearTerm>(1.0, auxBinary));
                        auxFirstSumVarDef->add(std::make_shared<LinearTerm>(i, auxBinary));

                        discretizationBinaries.push_back(auxBinary);
                        reformulatedProblem->add(auxBinary);
                        auxVariableCounter++;
                    }

                    reformulatedProblem->add(std::move(auxFirstSum));
                    reformulatedProblem->add(std::move(auxFirstSumVarDef));

                    integerAuxiliaryBinaryVariables.emplace(discretizationVariable, discretizationBinaries);
                }

                auto auxVariable = getBilinearAuxiliaryVariable(T->firstVariable, T->secondVariable);

                resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

                for(auto i = 0; i <= discretizationVariable->upperBound; i++)
                {
                    // double ijprod = -1.0 * coeffSign * signfactor * i * j;

                    if(coeffSign * signfactor == -1.0)
                    {
                        auto auxConstraint = std::make_shared<LinearConstraint>(auxConstraintCounter,
                            "s_blwn" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN,
                            discretizationVariable->upperBound * nonDiscretizationVariable->upperBound);

                        auxConstraintCounter++;

                        auxConstraint->add(std::make_shared<LinearTerm>(1.0, auxVariable));
                        auxConstraint->add(std::make_shared<LinearTerm>(-1.0 * i, nonDiscretizationVariable));
                        auxConstraint->add(std::make_shared<LinearTerm>(
                            discretizationVariable->upperBound * nonDiscretizationVariable->upperBound,
                            discretizationBinaries[i]));

                        reformulatedProblem->add(std::move(auxConstraint));
                    }
                    else
                    {
                        auto auxConstraint = std::make_shared<LinearConstraint>(auxConstraintCounter,
                            "s_blwp" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN,
                            discretizationVariable->upperBound * nonDiscretizationVariable->upperBound);

                        auxConstraintCounter++;

                        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
                        auxConstraint->add(std::make_shared<LinearTerm>(i, nonDiscretizationVariable));
                        auxConstraint->add(std::make_shared<LinearTerm>(
                            discretizationVariable->upperBound * nonDiscretizationVariable->upperBound,
                            discretizationBinaries[i]));

                        reformulatedProblem->add(std::move(auxConstraint));
                    }
                }
            }
        }
        else if(partitionNonBinaryTerms) // Square term x1^2 or general bilinear term x1*x2 will be partitioned into
                                         // multiple constraints
        {
            auto auxVariable = getBilinearAuxiliaryVariable(firstVariable, secondVariable);
            resultLinearTerms.add(std::make_shared<LinearTerm>(signfactor * T->coefficient, auxVariable));

            auto auxConstraint = std::make_shared<NonlinearConstraint>(
                auxConstraintCounter, "s_blcc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintCounter++;

            if(coeffSign * signfactor > 0 && T->getConvexity() == E_Convexity::Convex)
            {
                auxConstraint->properties.convexity = E_Convexity::Convex;
            }
            else
            {
                auxConstraint->properties.convexity = E_Convexity::Nonconvex;
            }

            auxConstraint->add(std::make_shared<LinearTerm>(-1.0 * coeffSign * signfactor, auxVariable));
            auxConstraint->add(std::make_shared<QuadraticTerm>(coeffSign * signfactor, firstVariable, secondVariable));

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
            && env->settings->getSetting<int>("Reformulation.Monomials.Formulation", "Model")
                == static_cast<int>(ES_ReformulationBinaryMonomials::Simple))
        {
            auto N = T->variables.size();

            auto auxConstraint1 = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_mon1" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
            auxConstraintCounter++;

            auto auxConstraint2 = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_mon2" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, N - 1.0);
            auxConstraintCounter++;

            auto auxbVar = std::make_shared<AuxiliaryVariable>("s_monb" + std::to_string(auxVariableCounter + 1),
                auxVariableCounter, E_VariableType::Binary, 0.0, 1.0);
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
            && env->settings->getSetting<int>("Reformulation.Monomials.Formulation", "Model")
                == static_cast<int>(ES_ReformulationBinaryMonomials::CostaLiberti))
        {
            int variableOffset = 0;
            int k = T->variables.size();

            Variables lambdas;

            auto auxLambdaSum = std::make_shared<LinearConstraint>(
                auxConstraintCounter, "s_monlam" + std::to_string(auxConstraintCounter), 1.0, 1.0);
            auxConstraintCounter++;

            auto numLambdas = std::pow(2, k);

            for(auto i = 1; numLambdas; i++)
            {
                auto auxLambda
                    = std::make_shared<AuxiliaryVariable>("s_monlam" + std::to_string(auxVariableCounter + 1),
                        auxVariableCounter + variableOffset, E_VariableType::Real, 0.0, 1.0);
                auxLambda->constant = 1.0 / numLambdas;

                auxLambdaSum->add(std::make_shared<LinearTerm>(1.0, auxLambda));
                lambdas.push_back(auxLambda);
                auxVariableCounter++;
                variableOffset++;
            }

            reformulatedProblem->add(std::move(auxLambdaSum));

            auto auxwVar = std::make_shared<AuxiliaryVariable>("s_monw" + std::to_string(auxVariableCounter + 1),
                auxVariableCounter + variableOffset, E_VariableType::Real, SHOT_DBL_MIN, SHOT_DBL_MAX);
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
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& LT : terms)
    {
        auto variable = reformulatedProblem->getVariable(LT->variable->index);

        std::dynamic_pointer_cast<LinearConstraint>(destination)
            ->add(std::make_shared<LinearTerm>(signCoefficient * LT->coefficient, variable));
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToConstraint(QuadraticTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& QT : terms)
    {
        auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
        auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

        std::dynamic_pointer_cast<QuadraticConstraint>(destination)
            ->add(std::make_shared<QuadraticTerm>(signCoefficient * QT->coefficient, firstVariable, secondVariable));
    }
}

template <class T>
void TaskReformulateProblem::copyMonomialTermsToConstraint(MonomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& MT : terms)
    {
        Variables variables;

        for(auto& V : MT->variables)
            variables.push_back(reformulatedProblem->getVariable(V->index));

        std::dynamic_pointer_cast<NonlinearConstraint>(destination)
            ->add(std::make_shared<MonomialTerm>(signCoefficient * MT->coefficient, variables));
    }
}

template <class T>
void TaskReformulateProblem::copySignomialTermsToConstraint(SignomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& ST : terms)
    {
        SignomialElements elements;

        for(auto& E : ST->elements)
            elements.push_back(
                std::make_shared<SignomialElement>(reformulatedProblem->getVariable(E->variable->index), E->power));

        std::dynamic_pointer_cast<NonlinearConstraint>(destination)
            ->add(std::make_shared<SignomialTerm>(signCoefficient * ST->coefficient, elements));
    }
}

template <class T>
void TaskReformulateProblem::copyLinearTermsToObjectiveFunction(LinearTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& LT : terms)
    {
        auto variable = reformulatedProblem->getVariable(LT->variable->index);

        std::dynamic_pointer_cast<LinearObjectiveFunction>(destination)
            ->add(std::make_shared<LinearTerm>(signCoefficient * LT->coefficient, variable));
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToObjectiveFunction(
    QuadraticTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& QT : terms)
    {
        auto firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
        auto secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

        std::dynamic_pointer_cast<QuadraticObjectiveFunction>(destination)
            ->add(std::make_shared<QuadraticTerm>(signCoefficient * QT->coefficient, firstVariable, secondVariable));
    }
}

template <class T>
void TaskReformulateProblem::copyMonomialTermsToObjectiveFunction(
    MonomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& MT : terms)
    {
        Variables variables;

        for(auto& V : MT->variables)
            variables.push_back(reformulatedProblem->getVariable(V->index));

        std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination)
            ->add(std::make_shared<MonomialTerm>(signCoefficient * MT->coefficient, variables));
    }
}

template <class T>
void TaskReformulateProblem::copySignomialTermsToObjectiveFunction(
    SignomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& ST : terms)
    {
        SignomialElements elements;

        for(auto& E : ST->elements)
            elements.push_back(
                std::make_shared<SignomialElement>(reformulatedProblem->getVariable(E->variable->index), E->power));

        std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination)
            ->add(std::make_shared<SignomialTerm>(signCoefficient * ST->coefficient, elements));
    }
}

AuxiliaryVariablePtr TaskReformulateProblem::getBilinearAuxiliaryVariable(
    VariablePtr firstVariable, VariablePtr secondVariable)
{
    // The variable with lower index is stored first in the tuple
    if(firstVariable->index < secondVariable->index)
    {
        auto auxVariableIterator = bilinearAuxVariables.find(std::make_tuple(firstVariable, secondVariable));

        if(auxVariableIterator != bilinearAuxVariables.end())
        {
            return (auxVariableIterator->second);
        }
    }
    else
    {
        auto auxVariableIterator = bilinearAuxVariables.find(std::make_tuple(secondVariable, firstVariable));

        if(auxVariableIterator != bilinearAuxVariables.end())
        {
            return (auxVariableIterator->second);
        }
    }

    auto valueList = { firstVariable->lowerBound * secondVariable->lowerBound,
        firstVariable->lowerBound * secondVariable->upperBound, firstVariable->upperBound * secondVariable->lowerBound,
        firstVariable->upperBound * secondVariable->upperBound };

    double lowerBound = std::min(valueList);
    double upperBound = std::max(valueList);

    // Create a new variable
    auto auxVariable = std::make_shared<AuxiliaryVariable>("s_bl_" + firstVariable->name + "_" + secondVariable->name,
        auxVariableCounter, E_VariableType::Real, lowerBound, upperBound);
    auxVariableCounter++;

    if(firstVariable->properties.type == E_VariableType::Binary
        && secondVariable->properties.type == E_VariableType::Binary)
    {
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::BinaryBilinear;
    }
    else if(firstVariable->properties.type == E_VariableType::Integer
        && secondVariable->properties.type == E_VariableType::Integer)
    {
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::IntegerBilinear;
    }
    else if(firstVariable->properties.type == E_VariableType::Real
        && secondVariable->properties.type == E_VariableType::Real)
    {
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::ContinuousBilinear;
    }
    else
    {
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::BinaryContinuousOrIntegerBilinear;
    }

    reformulatedProblem->add((auxVariable));
    auxVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));

    return (auxVariable);
}

void TaskReformulateProblem::addBilinearMcCormickEnvelope(
    AuxiliaryVariablePtr auxVariable, VariablePtr firstVariable, VariablePtr secondVariable)
{
    auto auxConstraintU1
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, firstVariable->lowerBound * secondVariable->lowerBound);
    auxConstraintU1->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
    auxConstraintU1->add(std::make_shared<LinearTerm>(firstVariable->lowerBound, secondVariable));
    auxConstraintU1->add(std::make_shared<LinearTerm>(secondVariable->lowerBound, firstVariable));
    auxConstraintCounter++;

    auto auxConstraintU2
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, firstVariable->upperBound * secondVariable->upperBound);
    auxConstraintU2->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
    auxConstraintU2->add(std::make_shared<LinearTerm>(firstVariable->upperBound, secondVariable));
    auxConstraintU2->add(std::make_shared<LinearTerm>(secondVariable->upperBound, firstVariable));
    auxConstraintCounter++;

    auto auxConstraintU3
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, -firstVariable->upperBound * secondVariable->lowerBound);
    auxConstraintU3->add(std::make_shared<LinearTerm>(1.0, auxVariable));
    auxConstraintU3->add(std::make_shared<LinearTerm>(-firstVariable->upperBound, secondVariable));
    auxConstraintU3->add(std::make_shared<LinearTerm>(-secondVariable->lowerBound, firstVariable));
    auxConstraintCounter++;

    auto auxConstraintU4
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, firstVariable->lowerBound * secondVariable->upperBound);
    auxConstraintU4->add(std::make_shared<LinearTerm>(1.0, auxVariable));
    auxConstraintU4->add(std::make_shared<LinearTerm>(-firstVariable->lowerBound, secondVariable));
    auxConstraintU4->add(std::make_shared<LinearTerm>(-secondVariable->upperBound, firstVariable));
    auxConstraintCounter++;

    reformulatedProblem->add(std::move(auxConstraintU1));
    reformulatedProblem->add(std::move(auxConstraintU2));
    reformulatedProblem->add(std::move(auxConstraintU3));
    reformulatedProblem->add(std::move(auxConstraintU4));
}

} // namespace SHOT