/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Problem.h"
#include "../Enums.h"
#include "../Output.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"
#include "../Model/Simplifications.h"
#include "../Tasks/TaskReformulateProblem.h"

namespace SHOT
{

void Problem::updateConstraints()
{
    NumericConstraints auxConstraints;

    for(auto& C : numericConstraints)
    {
        if(C->valueLHS > C->valueRHS)
            std::swap(C->valueRHS, C->valueLHS);
    }

    for(auto& C : linearConstraints)
    {
        if(C->valueRHS == SHOT_DBL_MAX && C->valueLHS != SHOT_DBL_MIN)
        {
            if(C->valueRHS != 0.0)
                C->valueRHS = -C->valueLHS;

            C->valueLHS = SHOT_DBL_MIN;

            for(auto& T : C->linearTerms)
                T->coefficient *= -1.0;

            C->constant *= -1.0;
        }
    }

    for(auto& C : quadraticConstraints)
    {
        if(C->valueRHS == SHOT_DBL_MAX && C->valueLHS != SHOT_DBL_MIN)
        {
            if(C->valueRHS != 0.0)
                C->valueRHS = -C->valueLHS;

            C->valueLHS = SHOT_DBL_MIN;

            for(auto& T : C->linearTerms)
                T->coefficient *= -1.0;

            for(auto& T : C->quadraticTerms)
                T->coefficient *= -1.0;

            C->constant *= -1.0;
        }
        else if(C->valueLHS != SHOT_DBL_MIN && C->valueRHS != SHOT_DBL_MAX
            && static_cast<ES_QuadraticProblemStrategy>(
                   env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"))
                != ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained)
        {
            double valueLHS = C->valueLHS;
            C->valueLHS = SHOT_DBL_MIN;

            auto auxConstraint = std::make_shared<QuadraticConstraint>();

            auxConstraint->constant = -C->constant;

            if(valueLHS != 0.0)
                auxConstraint->valueRHS = -valueLHS;
            else
                auxConstraint->valueRHS = 0.0;

            auxConstraint->name = C->name + "_rf";
            auxConstraint->ownerProblem = C->ownerProblem;
            auxConstraint->index = this->numericConstraints.size() - 1;

            for(auto& T : C->linearTerms)
                auxConstraint->add(std::make_shared<LinearTerm>(-1.0 * T->coefficient, T->variable));

            for(auto& T : C->quadraticTerms)
                auxConstraint->add(
                    std::make_shared<QuadraticTerm>(-1.0 * T->coefficient, T->firstVariable, T->secondVariable));

            auxConstraint->updateProperties();
            auxConstraints.push_back(auxConstraint);
        }
    }

    for(auto& C : nonlinearConstraints)
    {
        C->variablesInNonlinearExpression.clear();

        if(C->valueRHS == SHOT_DBL_MAX && C->valueLHS != SHOT_DBL_MIN)
        {
            if(C->valueRHS != 0.0)
                C->valueRHS = -C->valueLHS;

            C->valueLHS = SHOT_DBL_MIN;

            for(auto& T : C->linearTerms)
                T->coefficient *= -1.0;

            for(auto& T : C->quadraticTerms)
                T->coefficient *= -1.0;

            for(auto& T : C->monomialTerms)
                T->coefficient *= -1.0;

            for(auto& T : C->signomialTerms)
                T->coefficient *= -1.0;

            if(C->nonlinearExpression)
                C->nonlinearExpression = simplify(std::make_shared<ExpressionNegate>(C->nonlinearExpression));

            C->constant *= -1.0;
        }
        else if(C->valueLHS == C->valueRHS
            && ((C->properties.convexity == E_Convexity::Convex
                    && C->properties.monotonicity == E_Monotonicity::Nondecreasing)
                   || (C->properties.convexity == E_Convexity::Concave
                          && C->properties.monotonicity == E_Monotonicity::Nonincreasing)))
        {
            // Will rewrite as ()^2 <=c^2

            auto auxConstraint = std::make_shared<NonlinearConstraint>(
                this->numericConstraints.size(), C->name + "_eqrf", SHOT_DBL_MIN, C->valueRHS * C->valueRHS);

            auxConstraint->properties.classification = E_ConstraintClassification::Nonlinear;

            if(C->constant != 0.0)
                auxConstraint->add(std::make_shared<ExpressionConstant>(C->constant));

            if(C->properties.hasLinearTerms)
            {
                for(auto& LT : std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms)
                {
                    if(LT->coefficient == 1.0)
                        auxConstraint->add(std::make_shared<ExpressionVariable>(LT->variable));
                    else
                    {
                        auxConstraint->add(
                            std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(LT->coefficient),
                                std::make_shared<ExpressionVariable>(LT->variable)));
                    }
                }
            }

            if(C->properties.hasQuadraticTerms)
            {
                for(auto& QT : std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms)
                {
                    NonlinearExpressions product;

                    if(QT->coefficient != 1.0)
                        product.push_back(std::make_shared<ExpressionConstant>(QT->coefficient));

                    product.push_back(std::make_shared<ExpressionVariable>(QT->firstVariable));
                    product.push_back(std::make_shared<ExpressionVariable>(QT->secondVariable));

                    C->add(std::make_shared<ExpressionProduct>(product));
                }
            }

            if(C->properties.hasMonomialTerms)
            {
                for(auto& MT : std::dynamic_pointer_cast<NonlinearConstraint>(C)->monomialTerms)
                {
                    NonlinearExpressions product;

                    if(MT->coefficient != 1.0)
                        product.push_back(std::make_shared<ExpressionConstant>(MT->coefficient));

                    for(auto& VAR : MT->variables)
                        product.push_back(std::make_shared<ExpressionVariable>(VAR));

                    C->add(std::make_shared<ExpressionProduct>(product));
                }
            }

            if(C->properties.hasSignomialTerms)
            {
                for(auto& ST : std::dynamic_pointer_cast<NonlinearConstraint>(C)->signomialTerms)
                {
                    NonlinearExpressions product;

                    if(ST->coefficient != 1.0)
                        product.push_back(std::make_shared<ExpressionConstant>(ST->coefficient));

                    for(auto& E : ST->elements)
                    {
                        if(E->power == 1.0)
                            product.push_back(std::make_shared<ExpressionVariable>(E->variable));
                        else if(E->power == 2.0)
                            product.push_back(
                                std::make_shared<ExpressionSquare>(std::make_shared<ExpressionVariable>(E->variable)));
                        else
                            product.push_back(
                                std::make_shared<ExpressionPower>(std::make_shared<ExpressionVariable>(E->variable),
                                    std::make_shared<ExpressionConstant>(E->power)));
                    }

                    C->add(std::make_shared<ExpressionProduct>(product));
                }
            }

            if(C->properties.hasNonlinearExpression)
            {
                auxConstraint->add(copyNonlinearExpression(
                    std::dynamic_pointer_cast<NonlinearConstraint>(C)->nonlinearExpression.get(), this));
            }

            auxConstraint->nonlinearExpression = std::make_shared<ExpressionSquare>(auxConstraint->nonlinearExpression);

            auxConstraint->ownerProblem = C->ownerProblem;

            auxConstraint->updateProperties();

            // We know this is a convex constraint
            auxConstraint->properties.convexity = E_Convexity::Convex;
            auxConstraints.push_back(auxConstraint);
        }
        else if(C->valueLHS != SHOT_DBL_MIN && C->valueRHS != SHOT_DBL_MAX)
        {
            double valueLHS = C->valueLHS;
            C->valueLHS = SHOT_DBL_MIN;

            auto auxConstraint = std::make_shared<NonlinearConstraint>();

            auxConstraint->constant = -C->constant;

            if(valueLHS != 0.0)
                auxConstraint->valueRHS = -valueLHS;
            else
                auxConstraint->valueRHS = 0.0;

            auxConstraint->name = C->name + "_rf";
            auxConstraint->ownerProblem = C->ownerProblem;
            auxConstraint->index = this->numericConstraints.size() - 1;

            for(auto& T : C->linearTerms)
                auxConstraint->add(std::make_shared<LinearTerm>(-1.0 * T->coefficient, T->variable));

            for(auto& T : C->quadraticTerms)
                auxConstraint->add(
                    std::make_shared<QuadraticTerm>(-1.0 * T->coefficient, T->firstVariable, T->secondVariable));

            for(auto& T : C->monomialTerms)
                auxConstraint->add(std::make_shared<MonomialTerm>(-1.0 * T->coefficient, T->variables));

            for(auto& T : C->signomialTerms)
                auxConstraint->add(std::make_shared<SignomialTerm>(-1.0 * T->coefficient, T->elements));

            if(C->nonlinearExpression)
                auxConstraint->nonlinearExpression = simplify(
                    std::make_shared<ExpressionNegate>(copyNonlinearExpression(C->nonlinearExpression.get(), this)));

            auxConstraint->updateProperties();
            auxConstraints.push_back(auxConstraint);
        }
    }

    for(auto& C : auxConstraints)
        this->add(C);

    this->objectiveFunction->takeOwnership(shared_from_this());

    for(auto& C : numericConstraints)
    {
        C->updateProperties();
        C->takeOwnership(shared_from_this());
    }
}

void Problem::updateConvexity()
{
    bool assumeConvex = env->settings->getSetting<bool>("Convexity.AssumeConvex", "Model");

    if(assumeConvex && objectiveFunction->properties.convexity != E_Convexity::Linear)
        objectiveFunction->properties.convexity
            = (objectiveFunction->properties.isMinimize) ? E_Convexity::Convex : E_Convexity::Concave;

    for(auto& C : numericConstraints)
    {
        if(assumeConvex && C->properties.convexity != E_Convexity::Linear)
            C->properties.convexity = E_Convexity::Convex;
    }

    if(assumeConvex)
    {
        properties.convexity = E_ProblemConvexity::Convex;
    }
    else
    {
        if(objectiveFunction->properties.isMinimize
            && (objectiveFunction->properties.convexity == E_Convexity::Linear
                   || objectiveFunction->properties.convexity == E_Convexity::Convex))
        {
            properties.convexity = E_ProblemConvexity::Convex;
        }
        else if(objectiveFunction->properties.isMaximize
            && (objectiveFunction->properties.convexity == E_Convexity::Linear
                   || objectiveFunction->properties.convexity == E_Convexity::Concave))
        {
            properties.convexity = E_ProblemConvexity::Convex;
        }
        else if(objectiveFunction->properties.convexity == E_Convexity::Nonconvex)
        {
            properties.convexity = E_ProblemConvexity::Nonconvex;
        }
        else if(objectiveFunction->properties.convexity == E_Convexity::Unknown)
        {
            properties.convexity = E_ProblemConvexity::Nonconvex;
        }

        if(properties.convexity == E_ProblemConvexity::Convex)
        {
            for(auto& C : quadraticConstraints)
            {
                if(C->properties.convexity != E_Convexity::Linear && C->properties.convexity != E_Convexity::Convex)
                {
                    properties.convexity = E_ProblemConvexity::Nonconvex;
                    break;
                }
            }

            if(properties.convexity != E_ProblemConvexity::Nonconvex)
            {
                for(auto& C : nonlinearConstraints)
                {
                    if(C->properties.convexity != E_Convexity::Linear && C->properties.convexity != E_Convexity::Convex)
                    {
                        properties.convexity = E_ProblemConvexity::Nonconvex;
                        break;
                    }
                }
            }
        }
    }
}

void Problem::updateVariableBounds()
{
    auto numVariables = allVariables.size();

    // Update bound vectors
    if(variableLowerBounds.size() != numVariables)
        variableLowerBounds.resize(numVariables);

    if(variableUpperBounds.size() != numVariables)
        variableUpperBounds.resize(numVariables);

    if(variableBounds.size() != numVariables)
        variableBounds.resize(numVariables);

    for(size_t i = 0; i < numVariables; i++)
    {
        if(allVariables[i]->properties.type == E_VariableType::Integer && allVariables[i]->lowerBound > -1
            && allVariables[i]->upperBound < 2 && allVariables[i]->lowerBound != allVariables[i]->upperBound)
        {
            allVariables[i]->properties.type = E_VariableType::Binary;
            allVariables[i]->lowerBound = 0.0;
            allVariables[i]->upperBound = 1.0;
        }

        variableLowerBounds[i] = allVariables[i]->lowerBound;
        variableUpperBounds[i] = allVariables[i]->upperBound;
        variableBounds[i] = Interval(variableLowerBounds[i], variableUpperBounds[i]);
    }
}

void Problem::updateVariables()
{
    allVariables.sortByIndex();
    allVariables.sortByIndex();
    allVariables.sortByIndex();
    realVariables.sortByIndex();
    binaryVariables.sortByIndex();
    integerVariables.sortByIndex();
    semicontinuousVariables.sortByIndex();
    auxiliaryVariables.sortByIndex();

    nonlinearVariables.clear();
    nonlinearExpressionVariables.clear();

    // Reset variable properties
    for(auto& V : allVariables)
    {
        V->properties.isNonlinear = false;
        V->properties.inObjectiveFunction = false;
        V->properties.inLinearConstraints = false;
        V->properties.inQuadraticConstraints = false;
        V->properties.inNonlinearConstraints = false;
        V->properties.inMonomialTerms = false;
        V->properties.inSignomialTerms = false;
        V->properties.inNonlinearExpression = false;
    }

    updateVariableBounds();

    if(objectiveFunction->properties.hasLinearTerms)
    {
        for(auto& T : std::dynamic_pointer_cast<LinearObjectiveFunction>(objectiveFunction)->linearTerms)
        {
            T->variable->properties.inObjectiveFunction = true;
            T->variable->properties.inLinearTerms = true;
        }
    }

    if(objectiveFunction->properties.hasQuadraticTerms)
    {
        for(auto& T : std::dynamic_pointer_cast<QuadraticObjectiveFunction>(objectiveFunction)->quadraticTerms)
        {
            T->firstVariable->properties.inObjectiveFunction = true;
            T->secondVariable->properties.inObjectiveFunction = true;
            T->firstVariable->properties.inQuadraticTerms = true;
            T->secondVariable->properties.inQuadraticTerms = true;
            T->firstVariable->properties.isNonlinear = true;
            T->secondVariable->properties.isNonlinear = true;
        }
    }

    if(objectiveFunction->properties.hasMonomialTerms)
    {
        for(auto& T : std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objectiveFunction)->monomialTerms)
        {
            for(auto& V : T->variables)
            {
                V->properties.inObjectiveFunction = true;
                V->properties.inMonomialTerms = true;
                V->properties.isNonlinear = true;
            }
        }
    }

    if(objectiveFunction->properties.hasSignomialTerms)
    {
        for(auto& T : std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objectiveFunction)->signomialTerms)
        {
            for(auto& E : T->elements)
            {
                E->variable->properties.inObjectiveFunction = true;
                E->variable->properties.inSignomialTerms = true;
                E->variable->properties.isNonlinear = true;
            }
        }
    }

    if(objectiveFunction->properties.hasNonlinearExpression)
    {
        for(auto& V :
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objectiveFunction)->variablesInNonlinearExpression)
        {
            V->properties.inObjectiveFunction = true;
            V->properties.inNonlinearExpression = true;

            V->properties.isNonlinear = true;
        }
    }

    for(auto& C : linearConstraints)
    {
        for(auto& T : C->linearTerms)
        {
            T->variable->properties.inLinearConstraints = true;
            T->variable->properties.inLinearTerms = true;
        }
    }

    for(auto& C : quadraticConstraints)
    {
        for(auto& T : C->linearTerms)
            T->variable->properties.inLinearTerms = true;

        for(auto& T : C->quadraticTerms)
        {
            T->firstVariable->properties.inQuadraticConstraints = true;
            T->secondVariable->properties.inQuadraticConstraints = true;
            T->firstVariable->properties.inQuadraticTerms = true;
            T->secondVariable->properties.inQuadraticTerms = true;
            T->firstVariable->properties.isNonlinear = true;
            T->secondVariable->properties.isNonlinear = true;
        }
    }

    for(auto& C : nonlinearConstraints)
    {
        for(auto& T : C->linearTerms)
            T->variable->properties.inLinearTerms = true;

        for(auto& T : C->quadraticTerms)
        {
            T->firstVariable->properties.inQuadraticTerms = true;
            T->secondVariable->properties.inQuadraticTerms = true;
            T->firstVariable->properties.isNonlinear = true;
            T->secondVariable->properties.isNonlinear = true;
        }

        for(auto& V : C->variablesInMonomialTerms)
        {
            V->properties.inMonomialTerms = true;
            V->properties.inNonlinearConstraints = true;
            V->properties.isNonlinear = true;
        }

        for(auto& V : C->variablesInSignomialTerms)
        {
            V->properties.inSignomialTerms = true;
            V->properties.inNonlinearConstraints = true;
            V->properties.isNonlinear = true;
        }

        for(auto& V : C->variablesInNonlinearExpression)
        {
            V->properties.inNonlinearExpression = true;
            V->properties.inNonlinearConstraints = true;
            V->properties.isNonlinear = true;
        }
    }

    for(auto& V : allVariables)
    {
        if(V->properties.isNonlinear)
            nonlinearVariables.push_back(V);

        if(V->properties.inNonlinearExpression)
            nonlinearExpressionVariables.push_back(V);

        assert(!V->properties.isNonlinear
            || (V->properties.isNonlinear
                   && (V->properties.inQuadraticTerms || V->properties.inMonomialTerms || V->properties.inSignomialTerms
                          || V->properties.inNonlinearExpression)));
    }

    allVariables.takeOwnership(shared_from_this());
    auxiliaryVariables.takeOwnership(shared_from_this());

    variablesUpdated = true;
}

void Problem::updateProperties()
{
    objectiveFunction->updateProperties();

    updateConstraints();
    updateVariables();
    updateConvexity();

    properties.numberOfVariables = allVariables.size();
    properties.numberOfRealVariables = realVariables.size();
    properties.numberOfBinaryVariables = binaryVariables.size();
    properties.numberOfIntegerVariables = integerVariables.size();
    properties.numberOfDiscreteVariables = properties.numberOfBinaryVariables + properties.numberOfIntegerVariables;
    properties.numberOfSemicontinuousVariables = semicontinuousVariables.size();
    properties.numberOfNonlinearVariables = nonlinearVariables.size();
    properties.numberOfVariablesInNonlinearExpressions = nonlinearExpressionVariables.size();
    properties.numberOfAuxiliaryVariables = auxiliaryVariables.size();

    if(auxiliaryObjectiveVariable)
        properties.numberOfAuxiliaryVariables++;

    assert(properties.numberOfVariables
        == properties.numberOfRealVariables + properties.numberOfDiscreteVariables
            + properties.numberOfSemicontinuousVariables);

    properties.numberOfNumericConstraints = numericConstraints.size();
    properties.numberOfLinearConstraints = linearConstraints.size();

    bool isObjNonlinear = (objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic
        && (objectiveFunction->properties.hasQuadraticTerms || objectiveFunction->properties.hasMonomialTerms
               || objectiveFunction->properties.hasSignomialTerms
               || objectiveFunction->properties.hasNonlinearExpression));
    bool isObjQuadratic = (objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic
        && objectiveFunction->properties.hasQuadraticTerms);

    properties.numberOfQuadraticConstraints = 0;
    properties.numberOfConvexQuadraticConstraints = 0;
    properties.numberOfNonconvexQuadraticConstraints = 0;

    for(auto& C : quadraticConstraints)
    {
        if(C->properties.hasQuadraticTerms)
        {
            properties.numberOfQuadraticConstraints++;

            if(C->properties.convexity == E_Convexity::Convex)
                properties.numberOfConvexQuadraticConstraints++;
            else
                properties.numberOfNonconvexQuadraticConstraints++;
        }
    }

    properties.numberOfNonlinearConstraints = 0;
    properties.numberOfConvexNonlinearConstraints = 0;
    properties.numberOfNonconvexNonlinearConstraints = 0;
    properties.numberOfNonlinearExpressions = 0;

    for(auto& C : nonlinearConstraints)
    {
        if(C->properties.hasQuadraticTerms || C->properties.hasMonomialTerms || C->properties.hasSignomialTerms
            || C->properties.hasNonlinearExpression)
        {
            properties.numberOfNonlinearConstraints++;

            if(C->properties.convexity == E_Convexity::Convex)
                properties.numberOfConvexNonlinearConstraints++;
            else
                properties.numberOfNonconvexNonlinearConstraints++;
        }

        if(C->properties.hasNonlinearExpression)
            properties.numberOfNonlinearExpressions++;
    }

    if(objectiveFunction->properties.hasNonlinearExpression)
        properties.numberOfNonlinearExpressions++;

    assert(properties.numberOfNumericConstraints
        == properties.numberOfLinearConstraints + properties.numberOfQuadraticConstraints
            + properties.numberOfNonlinearConstraints);

    bool areConstrsNonlinear = (properties.numberOfNonlinearConstraints > 0);
    bool areConstrsQuadratic = (properties.numberOfQuadraticConstraints > 0);

    properties.isDiscrete
        = (properties.numberOfDiscreteVariables > 0 || properties.numberOfSemicontinuousVariables > 0);

    if(areConstrsNonlinear || isObjNonlinear)
        properties.isNonlinear = true;

    if(properties.isDiscrete)
    {
        if(areConstrsNonlinear || isObjNonlinear)
        {
            properties.isMINLPProblem = true;
            properties.isNLPProblem = false;
            properties.isMIQPProblem = false;
            properties.isQPProblem = false;
            properties.isMIQCQPProblem = false;
            properties.isQCQPProblem = false;
            properties.isMILPProblem = false;
            properties.isLPProblem = false;
        }
        else if(areConstrsQuadratic)
        {
            properties.isMINLPProblem = false;
            properties.isNLPProblem = false;
            properties.isMIQPProblem = false;
            properties.isQPProblem = false;
            properties.isMIQCQPProblem = true;
            properties.isQCQPProblem = false;
            properties.isMILPProblem = false;
            properties.isLPProblem = false;
        }
        else if(isObjQuadratic)
        {
            properties.isMINLPProblem = false;
            properties.isNLPProblem = false;
            properties.isMIQPProblem = true;
            properties.isQPProblem = false;
            properties.isMIQCQPProblem = false;
            properties.isQCQPProblem = false;
            properties.isMILPProblem = false;
            properties.isLPProblem = false;
        }
        else
        {
            properties.isMINLPProblem = false;
            properties.isNLPProblem = false;
            properties.isMIQPProblem = false;
            properties.isQPProblem = false;
            properties.isMIQCQPProblem = false;
            properties.isQCQPProblem = false;
            properties.isMILPProblem = true;
            properties.isLPProblem = false;
        }
    }
    else
    {
        properties.isDiscrete = false;

        if(areConstrsNonlinear || isObjNonlinear)
        {
            properties.isMINLPProblem = false;
            properties.isNLPProblem = true;
            properties.isMIQPProblem = false;
            properties.isQPProblem = false;
            properties.isMIQCQPProblem = false;
            properties.isQCQPProblem = false;
            properties.isMILPProblem = false;
            properties.isLPProblem = false;
        }
        else if(areConstrsQuadratic)
        {
            properties.isMINLPProblem = false;
            properties.isNLPProblem = false;
            properties.isMIQPProblem = false;
            properties.isQPProblem = false;
            properties.isMIQCQPProblem = false;
            properties.isQCQPProblem = true;
            properties.isMILPProblem = false;
            properties.isLPProblem = false;
        }
        else if(isObjQuadratic)
        {
            properties.isMINLPProblem = false;
            properties.isNLPProblem = false;
            properties.isMIQPProblem = false;
            properties.isQPProblem = true;
            properties.isMIQCQPProblem = false;
            properties.isQCQPProblem = false;
            properties.isMILPProblem = false;
            properties.isLPProblem = false;
        }
        else
        {
            properties.isMINLPProblem = false;
            properties.isNLPProblem = false;
            properties.isMIQPProblem = false;
            properties.isQPProblem = false;
            properties.isMIQCQPProblem = false;
            properties.isQCQPProblem = false;
            properties.isMILPProblem = false;
            properties.isLPProblem = true;
        }
    }

    properties.isValid = true;
}

void Problem::updateFactorableFunctions()
{
    if(properties.numberOfVariablesInNonlinearExpressions == 0)
        return;

    int nonlinearVariableCounter = 0;

    factorableFunctionVariables = std::vector<CppAD::AD<double>>(properties.numberOfVariablesInNonlinearExpressions);

    for(auto& V : nonlinearExpressionVariables)
    {
        assert(V->properties.inNonlinearExpression);

        factorableFunctionVariables[nonlinearVariableCounter] = 3.0;
        V->factorableFunctionVariable = &factorableFunctionVariables[nonlinearVariableCounter];
        V->properties.nonlinearVariableIndex = nonlinearVariableCounter;

        nonlinearVariableCounter++;
    }

    CppAD::Independent(factorableFunctionVariables);

    int nonlinearExpressionCounter = 0;

    for(auto& C : nonlinearConstraints)
    {
        if(C->properties.hasNonlinearExpression && C->variablesInNonlinearExpression.size() > 0)
        {
            factorableFunctions.push_back(C->nonlinearExpression->getFactorableFunction());
            constraintsWithNonlinearExpressions.push_back(C);
            C->nonlinearExpressionIndex = nonlinearExpressionCounter;
            nonlinearExpressionCounter++;
        }
    }

    if(objectiveFunction->properties.hasNonlinearExpression
        && std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objectiveFunction)
                ->variablesInNonlinearExpression.size()
            > 0)
    {
        auto objective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objectiveFunction);

        objective->updateFactorableFunction();
        factorableFunctions.push_back(objective->nonlinearExpression->getFactorableFunction());

        objective->nonlinearExpressionIndex = nonlinearExpressionCounter;
    }

    if(factorableFunctions.size() > 0)
    {
        ADFunctions.Dependent(factorableFunctionVariables, factorableFunctions);
        // ADFunctions.optimize();
    }

    CppAD::AD<double>::abort_recording();
}

Problem::Problem(EnvironmentPtr env) : env(env) {}

Problem::~Problem()
{
    allVariables.clear();
    realVariables.clear();
    binaryVariables.clear();
    integerVariables.clear();
    semicontinuousVariables.clear();
    nonlinearVariables.clear();
    nonlinearExpressionVariables.clear();

    auxiliaryVariables.clear();

    variableLowerBounds.clear();
    variableUpperBounds.clear();

    numericConstraints.clear();
    linearConstraints.clear();
    quadraticConstraints.clear();
    nonlinearConstraints.clear();

    factorableFunctionVariables.clear();
    factorableFunctions.clear();
}

void Problem::finalize()
{
    updateProperties();
    updateFactorableFunctions();
    assert(verifyOwnership());

    if(env->settings->getSetting<bool>("BoundTightening.FeasibilityBased.Use", "Model"))
    {
        bool performBoundTightening = true;

        auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
            env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"));

        // Do not do bound tightening on problems solved by MIP solver
        if(this->properties.isLPProblem || this->properties.isMILPProblem)
            performBoundTightening = false;
        else if(this->properties.isMIQPProblem && quadraticStrategy != ES_QuadraticProblemStrategy::Nonlinear)
            performBoundTightening = false;
        else if(this->properties.isMIQCQPProblem && quadraticStrategy != ES_QuadraticProblemStrategy::Nonlinear)
            performBoundTightening = false;

        if(performBoundTightening)
            doFBBT();
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        getConstraintsJacobianSparsityPattern();

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        getLagrangianHessianSparsityPattern();
}

void Problem::add(Variables variables)
{
    for(auto& V : variables)
        add(V);
}

void Problem::add(VariablePtr variable)
{
    allVariables.push_back(variable);

    switch(variable->properties.type)
    {
    case(E_VariableType::Real):
        realVariables.push_back(variable);
        break;
    case(E_VariableType::Binary):
        binaryVariables.push_back(variable);
        break;
    case(E_VariableType::Integer):
        integerVariables.push_back(variable);
        break;
    case(E_VariableType::Semicontinuous):
        semicontinuousVariables.push_back(variable);
        break;
    default:
        break;
    }

    variable->takeOwnership(shared_from_this());
    variablesUpdated = false;

    env->output->outputTrace("Added variable to problem: " + variable->name);
}

void Problem::add(AuxiliaryVariables variables)
{
    for(auto& V : variables)
        add(V);
}

void Problem::add(AuxiliaryVariablePtr variable)
{
    allVariables.push_back(std::dynamic_pointer_cast<Variable>(variable));

    if(variable->properties.auxiliaryType == E_AuxiliaryVariableType::NonlinearObjectiveFunction)
        auxiliaryObjectiveVariable = variable;
    else
        auxiliaryVariables.push_back(variable);

    switch(variable->properties.type)
    {
    case(E_VariableType::Real):
        realVariables.push_back(variable);
        break;
    case(E_VariableType::Binary):
        binaryVariables.push_back(variable);
        break;
    case(E_VariableType::Integer):
        integerVariables.push_back(variable);
        break;
    case(E_VariableType::Semicontinuous):
        semicontinuousVariables.push_back(variable);
        break;
    default:
        break;
    }

    variable->takeOwnership(shared_from_this());
    variablesUpdated = false;

    env->output->outputTrace("Added variable to problem: " + variable->name);
}

void Problem::add(NumericConstraintPtr constraint)
{
    constraint->index = numericConstraints.size();
    numericConstraints.push_back(constraint);

    if(constraint->properties.hasNonlinearExpression || constraint->properties.hasMonomialTerms
        || constraint->properties.hasSignomialTerms)
    {
        nonlinearConstraints.push_back(std::dynamic_pointer_cast<NonlinearConstraint>(constraint));
    }
    else if(constraint->properties.hasQuadraticTerms
        && constraint->properties.classification >= E_ConstraintClassification::QuadraticConsideredAsNonlinear)
    {
        nonlinearConstraints.push_back(std::dynamic_pointer_cast<NonlinearConstraint>(constraint));
    }
    else if(constraint->properties.hasQuadraticTerms)
    {
        quadraticConstraints.push_back(std::dynamic_pointer_cast<QuadraticConstraint>(constraint));
    }
    else
    {
        linearConstraints.push_back(std::dynamic_pointer_cast<LinearConstraint>(constraint));
    }

    constraint->takeOwnership(shared_from_this());

    env->output->outputTrace("Added numeric constraint to problem: " + constraint->name);
}

void Problem::add(LinearConstraintPtr constraint)
{
    constraint->index = numericConstraints.size();
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
    linearConstraints.push_back(constraint);

    constraint->takeOwnership(shared_from_this());

    env->output->outputTrace("Added linear constraint to problem: " + constraint->name);
}

void Problem::add(QuadraticConstraintPtr constraint)
{
    constraint->index = numericConstraints.size();
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
    quadraticConstraints.push_back(constraint);

    constraint->takeOwnership(shared_from_this());

    env->output->outputTrace("Added quadratic constraint to problem: " + constraint->name);
}

void Problem::add(NonlinearConstraintPtr constraint)
{
    constraint->index = numericConstraints.size();
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
    nonlinearConstraints.push_back(constraint);

    constraint->takeOwnership(shared_from_this());

    env->output->outputTrace("Added nonlinear constraint to problem: " + constraint->name);
}

void Problem::add(ObjectiveFunctionPtr objective)
{
    objectiveFunction = objective;
    objectiveFunction->updateProperties();

    objective->takeOwnership(shared_from_this());

    env->output->outputTrace("Added objective function to problem.");
}

void Problem::add(LinearObjectiveFunctionPtr objective)
{
    objectiveFunction = objective;
    objectiveFunction->updateProperties();

    objective->takeOwnership(shared_from_this());

    env->output->outputTrace("Added linear objective function to problem.");
}

void Problem::add(QuadraticObjectiveFunctionPtr objective)
{
    objectiveFunction = objective;
    objectiveFunction->updateProperties();

    objective->takeOwnership(shared_from_this());

    env->output->outputTrace("Added quadratic objective function to problem.");
}

void Problem::add(NonlinearObjectiveFunctionPtr objective)
{
    objectiveFunction = objective;
    objectiveFunction->updateProperties();

    objective->takeOwnership(shared_from_this());

    env->output->outputTrace("Added nonlinear objective function to problem.");
}

template <class T> void Problem::add(std::vector<T> elements)
{
    for(auto& E : elements)
    {
        add(E);

        E->takeOwnership(shared_from_this());
    }
}

VariablePtr Problem::getVariable(int variableIndex)
{
    if(variableIndex > (int)allVariables.size())
    {
        throw VariableNotFoundException(
            fmt::format("Cannot find variable with index {} ", std::to_string(variableIndex)));
    }

    return allVariables.at(variableIndex);
}

ConstraintPtr Problem::getConstraint(int constraintIndex)
{
    if(constraintIndex > (int)numericConstraints.size())
    {
        throw ConstraintNotFoundException(
            fmt::format("Cannot find constraint with index {}", std::to_string(constraintIndex)));
    }

    return numericConstraints.at(constraintIndex);
}

double Problem::getVariableLowerBound(int variableIndex) { return allVariables.at(variableIndex)->lowerBound; }

double Problem::getVariableUpperBound(int variableIndex) { return allVariables.at(variableIndex)->upperBound; }

VectorDouble Problem::getVariableLowerBounds()
{
    if(!variablesUpdated)
    {
        updateVariableBounds();
    }

    return variableLowerBounds;
}

VectorDouble Problem::getVariableUpperBounds()
{
    if(!variablesUpdated)
    {
        updateVariableBounds();
    }

    return variableUpperBounds;
}

IntervalVector Problem::getVariableBounds()
{
    if(!variablesUpdated)
    {
        updateVariableBounds();
    }

    return variableBounds;
}

AuxiliaryVariables Problem::getAuxiliaryVariablesOfType(E_AuxiliaryVariableType type)
{
    AuxiliaryVariables variables;

    for(auto& V : auxiliaryVariables)
    {
        if(V->properties.auxiliaryType == type)
            variables.push_back(V);
    }

    return (variables);
}

void Problem::setVariableLowerBound(int variableIndex, double bound)
{
    allVariables.at(variableIndex)->lowerBound = bound;
    variablesUpdated = true;
}

void Problem::setVariableUpperBound(int variableIndex, double bound)
{
    allVariables.at(variableIndex)->upperBound = bound;
    variablesUpdated = true;
}

void Problem::setVariableBounds(int variableIndex, double lowerBound, double upperBound)
{
    allVariables.at(variableIndex)->lowerBound = lowerBound;
    allVariables.at(variableIndex)->upperBound = upperBound;
    variablesUpdated = true;
}

std::shared_ptr<std::vector<std::pair<NumericConstraintPtr, Variables>>>
    Problem::getConstraintsJacobianSparsityPattern()
{
    if(constraintGradientSparsityPattern)
    {
        // Already defined
        return (constraintGradientSparsityPattern);
    }

    constraintGradientSparsityPattern = std::make_shared<std::vector<std::pair<NumericConstraintPtr, Variables>>>();

    for(auto& C : numericConstraints)
    {
        constraintGradientSparsityPattern->push_back(std::make_pair(C, *C->getGradientSparsityPattern()));
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        std::stringstream filename;
        filename << env->settings->getSetting<std::string>("Debug.Path", "Output");

        filename << "/sparsitypattern_jacobian";

        if(properties.isReformulated)
            filename << "_ref";

        filename << ".txt";

        std::stringstream stream;

        for(auto& P : *constraintGradientSparsityPattern)
        {
            stream << P.first->name << ":\n";

            for(auto& V : P.second)
                stream << "\t " << V->name << '\n';
        }

        Utilities::writeStringToFile(filename.str(), stream.str());
    }

    return (constraintGradientSparsityPattern);
}

std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> Problem::getConstraintsHessianSparsityPattern()
{
    if(constraintsHessianSparsityPattern)
    {
        // Already defined
        return (constraintsHessianSparsityPattern);
    }

    constraintsHessianSparsityPattern = std::make_shared<std::vector<std::pair<VariablePtr, VariablePtr>>>();

    for(auto& C : this->numericConstraints)
    {
        for(auto& E : *C->getHessianSparsityPattern())
        {
            constraintsHessianSparsityPattern->push_back(E);
        }
    }

    // Sorts the elements
    std::sort(constraintsHessianSparsityPattern->begin(), constraintsHessianSparsityPattern->end(),
        [](const std::pair<VariablePtr, VariablePtr>& elementOne,
            const std::pair<VariablePtr, VariablePtr>& elementTwo) {
            if(elementOne.first->index < elementTwo.first->index)
                return (true);
            if(elementOne.second->index == elementTwo.second->index)
                return (elementOne.first->index < elementTwo.first->index);
            return (false);
        });

    // Remove duplicates
    auto last = std::unique(constraintsHessianSparsityPattern->begin(), constraintsHessianSparsityPattern->end());
    constraintsHessianSparsityPattern->erase(last, constraintsHessianSparsityPattern->end());

    return (constraintsHessianSparsityPattern);
}

std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> Problem::getLagrangianHessianSparsityPattern()
{
    if(lagrangianHessianSparsityPattern)
    {
        // Already defined
        return (lagrangianHessianSparsityPattern);
    }

    lagrangianHessianSparsityPattern = std::make_shared<std::vector<std::pair<VariablePtr, VariablePtr>>>();

    for(auto& E : *objectiveFunction->getHessianSparsityPattern())
    {
        lagrangianHessianSparsityPattern->push_back(E);
    }

    for(auto& C : quadraticConstraints)
    {
        for(auto& E : *C->getHessianSparsityPattern())
        {
            lagrangianHessianSparsityPattern->push_back(E);
        }
    }

    for(auto& C : nonlinearConstraints)
    {
        for(auto& E : *C->getHessianSparsityPattern())
        {
            lagrangianHessianSparsityPattern->push_back(E);
        }
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        std::stringstream filename;
        filename << env->settings->getSetting<std::string>("Debug.Path", "Output");
        filename << "/sparsitypattern_hessianoflagrangian";

        if(properties.isReformulated)
            filename << "_ref";

        filename << ".txt";

        std::stringstream stream;

        for(auto& P : *lagrangianHessianSparsityPattern)
        {
            stream << P.first->name << "\t" << P.second->name << '\n';
        }

        Utilities::writeStringToFile(filename.str(), stream.str());
    }

    // Sorts the elements
    std::sort(lagrangianHessianSparsityPattern->begin(), lagrangianHessianSparsityPattern->end(),
        [](const std::pair<VariablePtr, VariablePtr>& elementOne,
            const std::pair<VariablePtr, VariablePtr>& elementTwo) {
            if(elementOne.first->index < elementTwo.first->index)
                return (true);
            if(elementOne.first->index == elementTwo.first->index)
                return (elementOne.second->index < elementTwo.second->index);
            return (false);
        });

    // Remove duplicates
    auto last = std::unique(lagrangianHessianSparsityPattern->begin(), lagrangianHessianSparsityPattern->end());
    lagrangianHessianSparsityPattern->erase(last, lagrangianHessianSparsityPattern->end());

    return (lagrangianHessianSparsityPattern);
}

std::optional<NumericConstraintValue> Problem::getMostDeviatingNumericConstraint(const VectorDouble& point)
{
    return (this->getMostDeviatingNumericConstraint(point, numericConstraints));
}

std::optional<NumericConstraintValue> Problem::getMostDeviatingNonlinearOrQuadraticConstraint(const VectorDouble& point)
{
    std::optional<NumericConstraintValue> constraintValue;

    if(properties.numberOfNonlinearConstraints > 0)
    {
        constraintValue = getMaxNumericConstraintValue(point, nonlinearConstraints);

        if(properties.numberOfQuadraticConstraints > 0)
        {
            auto quadConstraintValue = getMaxNumericConstraintValue(point, quadraticConstraints);

            if(quadConstraintValue.error > constraintValue->error)
                constraintValue = quadConstraintValue;
        }
    }
    else if(properties.numberOfQuadraticConstraints > 0)
    {
        constraintValue = getMaxNumericConstraintValue(point, quadraticConstraints);
    }

    return (constraintValue);
}

std::optional<NumericConstraintValue> Problem::getMostDeviatingNonlinearConstraint(const VectorDouble& point)
{
    return (this->getMostDeviatingNumericConstraint(point, nonlinearConstraints));
}

template <typename T>
std::optional<NumericConstraintValue> Problem::getMostDeviatingNumericConstraint(
    const VectorDouble& point, std::vector<T> constraintSelection)
{
    std::optional<NumericConstraintValue> optional;
    double error = 0;

    for(auto& C : constraintSelection)
    {
        auto constraintValue = C->calculateNumericValue(point);

        if(constraintValue.isFulfilled)
            continue;

        if(!optional) // No constraint with error found yet
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
        else if(constraintValue.error > error)
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
    }

    return optional;
}

template <typename T>
std::optional<NumericConstraintValue> Problem::getMostDeviatingNumericConstraint(
    const VectorDouble& point, std::vector<std::shared_ptr<T>> constraintSelection, std::vector<T*>& activeConstraints)
{
    assert(activeConstraints.size() == 0);

    std::optional<NumericConstraintValue> optional;
    double error = -1;

    for(auto& C : constraintSelection)
    {
        auto constraintValue = C->calculateNumericValue(point);

        if(constraintValue.isFulfilled)
            continue;
        else
            activeConstraints.push_back(C.get());

        if(!optional) // No constraint with error found yet
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
        else if(constraintValue.error > error)
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
    }

    return optional;
}

template <typename T>
std::optional<NumericConstraintValue> Problem::getMostDeviatingNumericConstraint(const VectorDouble& point,
    std::vector<std::shared_ptr<T>> constraintSelection, std::vector<std::shared_ptr<T>>& activeConstraints)
{
    assert(activeConstraints.size() == 0);

    std::optional<NumericConstraintValue> optional;
    double error = -1;

    for(auto& C : constraintSelection)
    {
        auto constraintValue = C->calculateNumericValue(point);

        if(constraintValue.isFulfilled)
            continue;
        else
            activeConstraints.push_back(C);

        if(!optional) // No constraint with error found yet
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
        else if(constraintValue.error > error)
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
    }

    return optional;
}

template <typename T>
NumericConstraintValue getMaxNumericConstraintValue(const VectorDouble& point,
    const std::vector<std::shared_ptr<T>> constraintSelection, std::vector<T*>& activeConstraints)
{
    assert(activeConstraints.size() == 0);
    assert(constraintSelection.size() > 0);

    auto value = constraintSelection[0]->calculateNumericValue(point);

    if(value.error > 0)
        activeConstraints.push_back(constraintSelection[0].get());

    for(int i = 1; i < constraintSelection.size(); i++)
    {
        auto tmpValue = constraintSelection[i]->calculateNumericValue(point);

        if(tmpValue.normalizedValue > value.normalizedValue)
        {
            value = tmpValue;
        }

        if(tmpValue.normalizedValue > 0)
            activeConstraints.push_back(constraintSelection[i].get());
    }

    return value;
}

NumericConstraintValue Problem::getMaxNumericConstraintValue(
    const VectorDouble& point, const LinearConstraints constraintSelection)
{
    assert(constraintSelection.size() > 0);

    auto value = constraintSelection[0]->calculateNumericValue(point);

    for(size_t i = 1; i < constraintSelection.size(); i++)
    {
        auto tmpValue = constraintSelection[i]->calculateNumericValue(point);

        if(tmpValue.normalizedValue > value.normalizedValue)
        {
            value = tmpValue;
        }
    }

    return value;
}

NumericConstraintValue Problem::getMaxNumericConstraintValue(
    const VectorDouble& point, const QuadraticConstraints constraintSelection)
{
    assert(constraintSelection.size() > 0);

    auto value = constraintSelection[0]->calculateNumericValue(point);

    for(size_t i = 1; i < constraintSelection.size(); i++)
    {
        auto tmpValue = constraintSelection[i]->calculateNumericValue(point);

        if(tmpValue.normalizedValue > value.normalizedValue)
        {
            value = tmpValue;
        }
    }

    return value;
}

NumericConstraintValue Problem::getMaxNumericConstraintValue(
    const VectorDouble& point, const NonlinearConstraints constraintSelection, double correction)
{
    assert(constraintSelection.size() > 0);

    auto value = constraintSelection[0]->calculateNumericValue(point, correction);

    for(size_t i = 1; i < constraintSelection.size(); i++)
    {
        auto tmpValue = constraintSelection[i]->calculateNumericValue(point, correction);

        if(tmpValue.normalizedValue > value.normalizedValue)
        {
            value = tmpValue;
        }
    }

    return value;
}

NumericConstraintValue Problem::getMaxNumericConstraintValue(
    const VectorDouble& point, const NumericConstraints constraintSelection)
{
    assert(constraintSelection.size() > 0);

    auto value = constraintSelection[0]->calculateNumericValue(point);

    for(size_t i = 1; i < constraintSelection.size(); i++)
    {
        auto tmpValue = constraintSelection[i]->calculateNumericValue(point);

        if(tmpValue.normalizedValue > value.normalizedValue)
        {
            value = tmpValue;
        }
    }

    return value;
}

NumericConstraintValue Problem::getMaxNumericConstraintValue(const VectorDouble& point,
    const std::vector<NumericConstraint*>& constraintSelection, std::vector<NumericConstraint*>& activeConstraints)
{
    assert(activeConstraints.size() == 0);
    assert(constraintSelection.size() > 0);

    auto value = constraintSelection[0]->calculateNumericValue(point);

    if(value.normalizedValue > 0)
        activeConstraints.push_back(constraintSelection[0]);

    for(size_t i = 1; i < constraintSelection.size(); i++)
    {
        auto tmpValue = constraintSelection[i]->calculateNumericValue(point);

        if(tmpValue.normalizedValue > value.normalizedValue)
        {
            value = tmpValue;
        }

        if(tmpValue.normalizedValue > 0)
            activeConstraints.push_back(constraintSelection[i]);
    }

    return value;
}

template <typename T>
NumericConstraintValues Problem::getAllDeviatingConstraints(
    const VectorDouble& point, double tolerance, std::vector<T> constraintSelection, double correction)
{
    NumericConstraintValues constraintValues;
    for(auto& C : constraintSelection)
    {
        NumericConstraintValue constraintValue = C->calculateNumericValue(point, correction);
        if(constraintValue.normalizedValue > tolerance)
            constraintValues.push_back(constraintValue);
    }

    return constraintValues;
}

NumericConstraintValues Problem::getFractionOfDeviatingNonlinearConstraints(
    const VectorDouble& point, double tolerance, double fraction, double correction)
{
    if(fraction > 1)
        fraction = 1;
    else if(fraction < 0)
        fraction = 0;

    int fractionNumbers = std::max(1, (int)ceil(fraction * this->nonlinearConstraints.size()));

    auto values = getAllDeviatingConstraints(point, tolerance, this->nonlinearConstraints, correction);

    std::sort(values.begin(), values.end(), std::greater<NumericConstraintValue>());

    if((int)values.size() <= fractionNumbers) // Not enough elements to need truncating
    {
        return values;
    }

    values.resize(fractionNumbers);
    return values;
}

NumericConstraintValues Problem::getAllDeviatingNumericConstraints(const VectorDouble& point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, numericConstraints);
}

NumericConstraintValues Problem::getAllDeviatingLinearConstraints(const VectorDouble& point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, linearConstraints);
}

NumericConstraintValues Problem::getAllDeviatingQuadraticConstraints(const VectorDouble& point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, quadraticConstraints);
}

NumericConstraintValues Problem::getAllDeviatingNonlinearConstraints(const VectorDouble& point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, nonlinearConstraints);
}

bool Problem::areLinearConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingLinearConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
}

bool Problem::areQuadraticConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingQuadraticConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
}

bool Problem::areNonlinearConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingNonlinearConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
}

bool Problem::areNumericConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingNumericConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
}

bool Problem::areIntegralityConstraintsFulfilled(VectorDouble point, double tolerance)
{
    for(auto& V : integerVariables)
    {
        if(abs(point.at(V->index) - round(point.at(V->index))) > tolerance)
            return false;
    }

    return true;
}

bool Problem::areVariableBoundsFulfilled(VectorDouble point, double tolerance)
{
    for(int i = 0; i < properties.numberOfVariables; ++i)
    {
        if(point.at(i) - tolerance > allVariables.at(i)->upperBound)
        {
            return false;
        }
        if(point.at(i) + tolerance < allVariables.at(i)->lowerBound)
        {
            return false;
        }
    }

    return true;
}

void Problem::saveProblemToFile(std::string filename)
{
    std::stringstream stream;
    stream << this;

    if(!Utilities::writeStringToFile(filename, stream.str()))
    {
        env->output->outputError("Error when writing to file " + filename);
    }
}

void Problem::doFBBT()
{
    env->timing->startTimer("BoundTightening");

    double startTime = env->timing->getElapsedTime("BoundTightening");

    if(properties.isReformulated)
    {
        env->timing->startTimer("BoundTighteningFBBTReformulated");
        env->output->outputInfo(" Performing bound tightening on reformulated problem.");
    }
    else
    {
        env->timing->startTimer("BoundTighteningFBBTOriginal");
        env->output->outputInfo(" Performing bound tightening on original problem.");
    }

    int numberOfIterations = env->settings->getSetting<int>("BoundTightening.FeasibilityBased.MaxIterations", "Model");
    double timeLimit = env->settings->getSetting<double>("BoundTightening.FeasibilityBased.TimeLimit", "Model");
    bool useNonlinearBoundTightening
        = env->settings->getSetting<bool>("BoundTightening.FeasibilityBased.UseNonlinear", "Model");

    bool stopTightening = false;

    for(int i = 0; i < numberOfIterations; i++)
    {
        bool boundsUpdated = false;

        env->output->outputDebug(fmt::format("  Bound tightening pass {} of {}.", i + 1, numberOfIterations));

        for(auto& C : linearConstraints)
        {
            if(env->timing->getElapsedTime("BoundTightening") - startTime > timeLimit)
            {
                stopTightening = true;
                break;
            }

            boundsUpdated = doFBBTOnConstraint(C, timeLimit + startTime) || boundsUpdated;
        }

        if(stopTightening)
            break;

        for(auto& C : quadraticConstraints)
        {
            if(env->timing->getElapsedTime("BoundTightening") - startTime > timeLimit)
            {
                stopTightening = true;
                break;
            }

            boundsUpdated = doFBBTOnConstraint(C, timeLimit + startTime) || boundsUpdated;
        }

        if(stopTightening)
            break;

        if(useNonlinearBoundTightening)
        {
            for(auto& C : nonlinearConstraints)
            {
                if(env->timing->getElapsedTime("BoundTightening") - startTime > timeLimit)
                {
                    stopTightening = true;
                    break;
                }

                boundsUpdated = doFBBTOnConstraint(C, timeLimit + startTime) || boundsUpdated;
            }
        }

        if(stopTightening || !boundsUpdated)
            break;
    }

    int numberOfTightenedVariables = std::count_if(allVariables.begin(), allVariables.end(),
        [](auto V) { return (V->properties.hasLowerBoundBeenTightened || V->properties.hasUpperBoundBeenTightened); });

    if(properties.isReformulated)
    {
        env->timing->stopTimer("BoundTighteningFBBTReformulated");
        env->output->outputInfo(fmt::format("  - Bounds for {} variables tightened in {:.2f} s.",
            numberOfTightenedVariables, env->timing->getElapsedTime("BoundTighteningFBBTReformulated")));
    }
    else
    {
        env->timing->stopTimer("BoundTighteningFBBTOriginal");
        env->output->outputInfo(fmt::format("  - Bounds for {} variables tightened in {:.2f} s.",
            numberOfTightenedVariables, env->timing->getElapsedTime("BoundTighteningFBBTOriginal")));
    }

    env->timing->stopTimer("BoundTightening");
}

bool Problem::doFBBTOnConstraint(NumericConstraintPtr constraint, double timeLimit)
{
    bool boundsUpdated = false;

    try
    {
        if(constraint->properties.hasLinearTerms)
        {
            Interval otherTermsBound(constraint->constant);

            if(constraint->properties.hasQuadraticTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->quadraticTerms.getBounds();

            if(constraint->properties.hasMonomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->monomialTerms.getBounds();

            if(constraint->properties.hasSignomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->signomialTerms.getBounds();

            if(constraint->properties.hasNonlinearExpression)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->nonlinearExpression->getBounds();

            auto terms = std::dynamic_pointer_cast<LinearConstraint>(constraint)->linearTerms;

            for(auto& T : terms)
            {
                if(env->timing->getElapsedTime("BoundTightening") > timeLimit)
                    break;

                if(T->coefficient == 0.0)
                    continue;

                Interval newBound = otherTermsBound;

                for(auto& T2 : terms)
                {
                    if(T2 == T)
                        continue;

                    newBound += T2->getBounds();
                }

                Interval termBound = Interval(constraint->valueLHS, constraint->valueRHS) - newBound;
                termBound = termBound / T->coefficient;

                if(T->variable->tightenBounds(termBound))
                {
                    boundsUpdated = true;
                    env->output->outputDebug(
                        fmt::format("  bound tightened using linear term in constraint {} .", constraint->name));
                }
            }
        }

        if(constraint->properties.hasQuadraticTerms && env->timing->getElapsedTime("BoundTightening") < timeLimit)
        {
            Interval otherTermsBound(constraint->constant);

            if(constraint->properties.hasLinearTerms)
                otherTermsBound += std::dynamic_pointer_cast<LinearConstraint>(constraint)->linearTerms.getBounds();

            if(constraint->properties.hasMonomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->monomialTerms.getBounds();

            if(constraint->properties.hasSignomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->signomialTerms.getBounds();

            if(constraint->properties.hasNonlinearExpression)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->nonlinearExpression->getBounds();

            auto terms = std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->quadraticTerms;

            for(auto& T : terms)
            {
                if(env->timing->getElapsedTime("BoundTightening") > timeLimit)
                    break;

                if(T->coefficient == 0.0)
                    continue;

                Interval newBound = otherTermsBound;

                for(auto& T2 : terms)
                {
                    if(T2 == T)
                        continue;

                    newBound += T2->getBounds();
                }

                Interval termBound = Interval(constraint->valueLHS, constraint->valueRHS) - newBound;
                termBound = termBound / T->coefficient;

                if(T->firstVariable == T->secondVariable)
                {
                    if(termBound.l() < 0)
                        continue;

                    if(T->firstVariable->tightenBounds(sqrt(termBound)))
                    {
                        boundsUpdated = true;
                        env->output->outputDebug(
                            fmt::format("  bound tightened using quadratic term in constraint {}.", constraint->name));
                    }
                }
                else
                {
                    Interval firstVariableBound = T->firstVariable->getBound();
                    Interval secondVariableBound = T->secondVariable->getBound();

                    if((firstVariableBound.l() > 0 || firstVariableBound.u() < 0)
                        && T->secondVariable->tightenBounds(termBound / firstVariableBound))
                    {
                        boundsUpdated = true;
                        env->output->outputDebug(
                            fmt::format("  bound tightened using quadratic term in constraint {}.", constraint->name));
                    }

                    if((secondVariableBound.l() > 0 || secondVariableBound.u() < 0)
                        && T->firstVariable->tightenBounds(termBound / secondVariableBound))
                    {
                        boundsUpdated = true;
                        env->output->outputDebug(
                            fmt::format("  bound tightened using quadratic term in constraint {}.", constraint->name));
                    }
                }
            }
        }

        if(constraint->properties.hasMonomialTerms && env->timing->getElapsedTime("BoundTightening") < timeLimit)
        {
            Interval otherTermsBound(constraint->constant);

            if(constraint->properties.hasLinearTerms)
                otherTermsBound += std::dynamic_pointer_cast<LinearConstraint>(constraint)->linearTerms.getBounds();

            if(constraint->properties.hasQuadraticTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->quadraticTerms.getBounds();

            if(constraint->properties.hasSignomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->signomialTerms.getBounds();

            if(constraint->properties.hasNonlinearExpression)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->nonlinearExpression->getBounds();

            auto terms = std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->monomialTerms;

            for(auto& T : terms)
            {
                if(env->timing->getElapsedTime("BoundTightening") > timeLimit)
                    break;

                if(T->coefficient == 0.0)
                    continue;

                Interval newBound = otherTermsBound;

                for(auto& T2 : terms)
                {
                    if(T2 == T)
                        continue;

                    newBound += T2->getBounds();
                }

                Interval termBound = Interval(constraint->valueLHS, constraint->valueRHS) - newBound;
                termBound = termBound / T->coefficient;

                for(auto& V1 : T->variables)
                {
                    Interval othersBound(1.0);

                    for(auto& V2 : T->variables)
                    {
                        if(V1 == V2)
                            continue;

                        othersBound *= V2->getBound();
                    }

                    // To avoid division by zero
                    if(othersBound.l() <= 0 && othersBound.u() >= 0)
                        continue;

                    auto childBound = termBound / othersBound;

                    if(V1->tightenBounds(childBound))
                    {
                        boundsUpdated = true;
                        env->output->outputDebug(
                            fmt::format("  bound tightened using monomial term in constraint {}.", constraint->name));
                    }
                }
            }
        }

        if(constraint->properties.hasSignomialTerms && env->timing->getElapsedTime("BoundTightening") < timeLimit)
        {
            Interval otherTermsBound(constraint->constant);

            if(constraint->properties.hasLinearTerms)
                otherTermsBound += std::dynamic_pointer_cast<LinearConstraint>(constraint)->linearTerms.getBounds();

            if(constraint->properties.hasQuadraticTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->quadraticTerms.getBounds();

            if(constraint->properties.hasMonomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->monomialTerms.getBounds();

            if(constraint->properties.hasNonlinearExpression)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->nonlinearExpression->getBounds();

            auto terms = std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->signomialTerms;

            for(auto& T : terms)
            {
                if(env->timing->getElapsedTime("BoundTightening") > timeLimit)
                    break;

                if(T->coefficient == 0.0)
                    continue;

                Interval newBound = otherTermsBound;

                for(auto& T2 : terms)
                {
                    if(T2 == T)
                        continue;

                    newBound += T2->getBounds();
                }

                Interval termBound = Interval(constraint->valueLHS, constraint->valueRHS) - newBound;

                termBound = termBound / T->coefficient;

                for(auto& E1 : T->elements)
                {
                    Interval othersBound(1.0);

                    for(auto& E2 : T->elements)
                    {
                        if(E1 == E2)
                            continue;

                        othersBound *= E2->getBounds();
                    }

                    // To avoid division by zero
                    if(othersBound.l() <= 0 && othersBound.u() >= 0)
                        continue;

                    auto childBound = termBound / othersBound;

                    if(E1->tightenBounds(childBound))
                    {
                        boundsUpdated = true;
                        env->output->outputDebug(
                            fmt::format("  bound tightened using signomial term in constraint {}.", constraint->name));
                    }
                }
            }
        }

        if(constraint->properties.hasNonlinearExpression && env->timing->getElapsedTime("BoundTightening") < timeLimit)
        {
            Interval otherTermsBound(constraint->constant);

            if(constraint->properties.hasLinearTerms)
                otherTermsBound += std::dynamic_pointer_cast<LinearConstraint>(constraint)->linearTerms.getBounds();

            if(constraint->properties.hasQuadraticTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<QuadraticConstraint>(constraint)->quadraticTerms.getBounds();

            if(constraint->properties.hasMonomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->monomialTerms.getBounds();

            if(constraint->properties.hasSignomialTerms)
                otherTermsBound
                    += std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->signomialTerms.getBounds();

            Interval candidate = Interval(constraint->valueLHS, constraint->valueRHS) - otherTermsBound;

            if(std::dynamic_pointer_cast<NonlinearConstraint>(constraint)
                    ->nonlinearExpression->tightenBounds(candidate))
            {
                env->output->outputDebug(
                    fmt::format("  bound tightened using nonlinear expression in constraint {}.", constraint->name));
                boundsUpdated = true;
            }
        }
    }
    catch(mc::Interval::Exceptions& e)
    {
        env->output->outputError(
            fmt::format("  error when tightening bound in constraint {}: {}", constraint->name, e.what()));
    }

    // Update variable bounds for original variables also in original problem if tightened in reformulated one
    if(boundsUpdated && this->properties.isReformulated)
    {
        for(size_t i = 0; i < env->problem->allVariables.size(); i++)
        {
            if(allVariables[i]->lowerBound > env->problem->allVariables[i]->lowerBound)
                env->problem->allVariables[i]->lowerBound = allVariables[i]->lowerBound;

            if(allVariables[i]->upperBound < env->problem->allVariables[i]->upperBound)
                env->problem->allVariables[i]->upperBound = allVariables[i]->upperBound;
        }
    }

    return (boundsUpdated);
}

std::ostream& operator<<(std::ostream& stream, const Problem& problem)
{
    if(problem.objectiveFunction->properties.isMinimize)
        stream << "minimize:\n";
    else
        stream << "maximize:\n";

    stream << problem.objectiveFunction << "\n\n";

    if(problem.numericConstraints.size() > 0)
        stream << "subject to:\n";

    for(auto& C : problem.numericConstraints)
    {
        stream << C << '\n';
    }

    stream << "\nvariables:\n";

    for(auto& V : problem.allVariables)
    {
        stream << V << '\n';
    }

    switch(problem.properties.convexity)
    {
    case E_ProblemConvexity::Nonconvex:
        stream << "\nProblem does not seem to be convex.\n";
        break;

    case E_ProblemConvexity::Convex:
        stream << "\nProblem is convex.\n";
        break;
    default:
        break;
    }

    return stream;
}

bool Problem::verifyOwnership()
{
    if(std::any_of(allVariables.begin(), allVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(realVariables.begin(), realVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(binaryVariables.begin(), binaryVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(integerVariables.begin(), integerVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(semicontinuousVariables.begin(), semicontinuousVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(nonlinearVariables.begin(), nonlinearVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(nonlinearExpressionVariables.begin(), nonlinearExpressionVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(auxiliaryVariables.begin(), auxiliaryVariables.end(),
           [this](VariablePtr const& V) { return (V->ownerProblem.lock().get() != this); }))
        return (false);

    if(auxiliaryObjectiveVariable && auxiliaryObjectiveVariable->ownerProblem.lock().get() != this)
        return (false);

    if(objectiveFunction->ownerProblem.lock().get() != this)
        return (false);

    if(auto objective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objectiveFunction))
    {
        if(std::any_of(objective->monomialTerms.begin(), objective->monomialTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(objective->signomialTerms.begin(), objective->signomialTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(objective->variablesInMonomialTerms.begin(), objective->variablesInMonomialTerms.end(),
               [this](auto const& V) { return (V->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(objective->variablesInSignomialTerms.begin(), objective->variablesInSignomialTerms.end(),
               [this](auto const& V) { return (V->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(objective->variablesInNonlinearExpression.begin(),
               objective->variablesInNonlinearExpression.end(),
               [this](auto const& V) { return (V->ownerProblem.lock().get() != this); }))
            return (false);
    }

    if(auto objective = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(objectiveFunction))
    {
        if(std::any_of(objective->linearTerms.begin(), objective->linearTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(objective->quadraticTerms.begin(), objective->quadraticTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);
    }

    auto objective = std::dynamic_pointer_cast<LinearObjectiveFunction>(objectiveFunction);

    if(std::any_of(objective->linearTerms.begin(), objective->linearTerms.end(),
           [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(numericConstraints.begin(), numericConstraints.end(),
           [this](ConstraintPtr const& C) { return (C->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(linearConstraints.begin(), linearConstraints.end(),
           [this](ConstraintPtr const& C) { return (C->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(quadraticConstraints.begin(), quadraticConstraints.end(),
           [this](ConstraintPtr const& C) { return (C->ownerProblem.lock().get() != this); }))
        return (false);

    if(std::any_of(nonlinearConstraints.begin(), nonlinearConstraints.end(),
           [this](ConstraintPtr const& C) { return (C->ownerProblem.lock().get() != this); }))
        return (false);

    for(auto& C : linearConstraints)
    {
        if(std::any_of(C->linearTerms.begin(), C->linearTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);
    }

    for(auto& C : quadraticConstraints)
    {
        if(std::any_of(C->linearTerms.begin(), C->linearTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(C->quadraticTerms.begin(), C->quadraticTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);
    }

    for(auto& C : nonlinearConstraints)
    {
        if(std::any_of(C->linearTerms.begin(), C->linearTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(C->quadraticTerms.begin(), C->quadraticTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(C->monomialTerms.begin(), C->monomialTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(C->signomialTerms.begin(), C->signomialTerms.end(),
               [this](auto const& T) { return (T->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(C->variablesInMonomialTerms.begin(), C->variablesInMonomialTerms.end(),
               [this](auto const& V) { return (V->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(C->variablesInSignomialTerms.begin(), C->variablesInSignomialTerms.end(),
               [this](auto const& V) { return (V->ownerProblem.lock().get() != this); }))
            return (false);

        if(std::any_of(C->variablesInNonlinearExpression.begin(), C->variablesInNonlinearExpression.end(),
               [this](auto const& V) { return (V->ownerProblem.lock().get() != this); }))
            return (false);
    }

    return (true);
}
} // namespace SHOT