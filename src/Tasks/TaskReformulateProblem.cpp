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
#include "TaskPerformBoundTightening.h"

#ifdef HAS_GUROBI
#include "gurobi_c.h"
#endif

namespace SHOT
{

TaskReformulateProblem::TaskReformulateProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("ProblemReformulation");

    auto quadraticStrategy = static_cast<ES_QuadraticProblemStrategy>(
        env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"));

    auto integerBilinearStrategy = static_cast<ES_ReformulateBilinearInteger>(
        env->settings->getSetting<int>("Reformulation.Bilinear.IntegerFormulation", "Model"));

    if(env->settings->getSetting<int>("MIP.Solver", "Dual") == (int)ES_MIPSolver::Cplex)
    {
        switch(quadraticStrategy)
        {
        case(ES_QuadraticProblemStrategy::Nonlinear):
            useConvexQuadraticConstraints = false;
            useConvexQuadraticConstraintsWithinTolerance = false;
            useNonconvexQuadraticConstraints = false;
            useConvexQuadraticObjective = false;
            useNonconvexQuadraticObjective = false;
            break;
        case(ES_QuadraticProblemStrategy::QuadraticObjective):
            useConvexQuadraticConstraints = false;
            useConvexQuadraticConstraintsWithinTolerance = false;
            useNonconvexQuadraticConstraints = false;
            useConvexQuadraticObjective = true;
            useNonconvexQuadraticObjective = false;
            break;
        case(ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained):
            useConvexQuadraticConstraints = true;
            useConvexQuadraticConstraintsWithinTolerance = false;
            useNonconvexQuadraticConstraints = false;
            useConvexQuadraticObjective = true;
            useNonconvexQuadraticObjective = false;
            break;
        case(ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained):
            useConvexQuadraticConstraints = true;
            useConvexQuadraticConstraintsWithinTolerance = false; // No support in Cplex
            useNonconvexQuadraticConstraints = false; // No support in Cplex
            useConvexQuadraticObjective = true;
            useNonconvexQuadraticObjective = true;
            break;
        default:
            break;
        }

        switch(integerBilinearStrategy)
        {
        case(ES_ReformulateBilinearInteger::No):
            useIntegerBilinearTermReformulation = false;
            break;
        case(ES_ReformulateBilinearInteger::NoIfQuadraticSupport):
            useIntegerBilinearTermReformulation = true;
            break;
        case(ES_ReformulateBilinearInteger::Yes):
            useIntegerBilinearTermReformulation = true;
            break;
        default:
            break;
        }
    }
    else if(env->settings->getSetting<int>("MIP.Solver", "Dual") == (int)ES_MIPSolver::Gurobi)
    {
        switch(quadraticStrategy)
        {
        case(ES_QuadraticProblemStrategy::Nonlinear):
            useConvexQuadraticConstraints = false;
            useConvexQuadraticConstraintsWithinTolerance = false;
            useNonconvexQuadraticConstraints = false;
            useConvexQuadraticObjective = false;
            useNonconvexQuadraticObjective = false;
            break;
        case(ES_QuadraticProblemStrategy::QuadraticObjective):
            useConvexQuadraticConstraints = false;
            useConvexQuadraticConstraintsWithinTolerance = false;
            useNonconvexQuadraticConstraints = false;
            useConvexQuadraticObjective = true;
            useNonconvexQuadraticObjective = false;
            break;
        case(ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained):
            useConvexQuadraticConstraints = true;
            useConvexQuadraticConstraintsWithinTolerance = true;
            useNonconvexQuadraticConstraints = false;
            useConvexQuadraticObjective = true;
            useNonconvexQuadraticObjective = false;
            break;
        case(ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained):
            useConvexQuadraticConstraints = true;
            useConvexQuadraticConstraintsWithinTolerance = true;
            useNonconvexQuadraticConstraints = true;
            useConvexQuadraticObjective = true;
            useNonconvexQuadraticObjective = true;
            break;
        default:
            break;
        }

        switch(integerBilinearStrategy)
        {
        case(ES_ReformulateBilinearInteger::No):
            useIntegerBilinearTermReformulation = false;
            break;

        case(ES_ReformulateBilinearInteger::NoIfQuadraticSupport):
#ifdef HAS_GUROBI
#if GRB_VERSION_MAJOR >= 9
            useIntegerBilinearTermReformulation = false;
#elif GRB_VERSION_MAJOR < 9
            useIntegerBilinearTermReformulation = true;
#endif
#endif
            break;
        case(ES_ReformulateBilinearInteger::Yes):
            useIntegerBilinearTermReformulation = true;
            break;
        default:
            break;
        }
    }
    else if(env->settings->getSetting<int>("MIP.Solver", "Dual") == (int)ES_MIPSolver::Cbc)
    {
        // Cbc does not support quadratic terms
        useConvexQuadraticConstraints = false;
        useConvexQuadraticConstraintsWithinTolerance = false;
        useNonconvexQuadraticConstraints = false;
        useConvexQuadraticObjective = false;
        useNonconvexQuadraticObjective = false;

        switch(integerBilinearStrategy)
        {
        case(ES_ReformulateBilinearInteger::No):
            useIntegerBilinearTermReformulation = false;
            break;
        case(ES_ReformulateBilinearInteger::NoIfQuadraticSupport):
            useIntegerBilinearTermReformulation = true;
            break;
        case(ES_ReformulateBilinearInteger::Yes):
            useIntegerBilinearTermReformulation = true;
            break;
        default:
            break;
        }
    }
    else
    {
        // Will not happen
    }

    extractQuadraticTermsFromNonconvexExpressions
        = (env->settings->getSetting<int>("Reformulation.Quadratics.ExtractStrategy", "Model")
            == static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractToEqualityConstraintIfNonconvex));

    extractQuadraticTermsFromConvexExpressions
        = (env->settings->getSetting<int>("Reformulation.Quadratics.ExtractStrategy", "Model")
            == static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractToEqualityConstraintAlways));

    maxBilinearIntegerReformulationDomain
        = env->settings->getSetting<int>("Reformulation.Bilinear.IntegerFormulation.MaxDomain", "Model");

    auxVariableCounter = env->problem->properties.numberOfVariables;
    auxConstraintCounter = env->problem->properties.numberOfNumericConstraints;

    reformulatedProblem = std::make_shared<Problem>(env);
    reformulatedProblem->name = env->problem->name + " (reformulated)";

    reformulatedProblem->variableLowerBounds = env->problem->variableLowerBounds;
    reformulatedProblem->variableUpperBounds = env->problem->variableUpperBounds;

    // Copying variables
    for(auto& V : env->problem->allVariables)
    {
        auto variable = std::make_shared<Variable>(
            V->name, V->index, V->properties.type, V->lowerBound, V->upperBound, V->semiBound);

        variable->properties.hasLowerBoundBeenTightened = V->properties.hasLowerBoundBeenTightened;
        variable->properties.hasUpperBoundBeenTightened = V->properties.hasUpperBoundBeenTightened;

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

    // Copying special ordered sets
    for(auto& S : env->problem->specialOrderedSets)
    {
        auto SOS = std::make_shared<SpecialOrderedSet>();
        SOS->type = S->type;
        SOS->weights = S->weights;

        for(auto& VAR : S->variables)
            SOS->variables.push_back(reformulatedProblem->getVariable(VAR->index));

        reformulatedProblem->add(std::move(SOS));
    }

    // Reformulating objective function
    reformulateObjectiveFunction();

    // Creating expressions for sums of squares partitioning
    createSquareReformulations();

    // Creating expressions for the bilinear reformulations
    createBilinearReformulations();

    reformulatedProblem->properties.isReformulated = true;
    reformulatedProblem->properties.numberOfAddedLinearizations = env->problem->properties.numberOfAddedLinearizations;
    reformulatedProblem->finalize();

    // Fixing that a quadratic objective changed into a nonlinear objective is correctly identified
    if(!(useConvexQuadraticObjective || useNonconvexQuadraticObjective)
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

    auto taskPerformBoundTightening = std::make_unique<TaskPerformBoundTightening>(env, reformulatedProblem);
    taskPerformBoundTightening->run();

    env->timing->stopTimer("ProblemReformulation");
}

TaskReformulateProblem::~TaskReformulateProblem() = default;

void TaskReformulateProblem::run() { }

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
        auto sourceObjective = std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction);

        // Let's check if we can do an anti-epigraph reformulation
        if(sourceObjective->linearTerms.size() == 1)
        {
            auto originalObjectiveVariable = sourceObjective->linearTerms.at(0)->variable;
            double originalObjectiveCoefficient = sourceObjective->linearTerms.at(0)->coefficient;

            if(originalObjectiveVariable->properties.inNumberOfLinearTerms == 2 // Objective function and constraint
                && (originalObjectiveVariable->properties.inQuadraticConstraints
                    || originalObjectiveVariable->properties
                           .inNonlinearConstraints) // In quadratic or nonlinear constraint
                && (!originalObjectiveVariable->properties.inQuadraticTerms // Not in quadratic terms
                    && !originalObjectiveVariable->properties.inNonlinearExpression // Not in nonlinear expressions
                    && !originalObjectiveVariable->properties.inMonomialTerms // Not in monomial terms
                    && !originalObjectiveVariable->properties.inSignomialTerms)) // Not in signomial terms
            {
                // Now know the objective variable is present in a nonlinear or quadratic constraint and only in one
                // unique linear term. Need to find the constraint

                ConstraintPtr epigraphConstraint;
                LinearTermPtr epigraphConstraintTerm;

                for(auto const& C : reformulatedProblem->quadraticConstraints)
                {
                    for(auto const& LT : C->linearTerms)
                    {
                        if(LT->variable->name == originalObjectiveVariable->name)
                        {
                            epigraphConstraint = C;
                            epigraphConstraintTerm = LT;
                            break;
                        }
                    }

                    if(epigraphConstraint)
                        break;
                }

                if(!epigraphConstraint)
                {
                    for(auto const& C : reformulatedProblem->nonlinearConstraints)
                    {
                        for(auto const& LT : C->linearTerms)
                        {
                            if(LT->variable->name == originalObjectiveVariable->name)
                            {
                                epigraphConstraint = C;
                                epigraphConstraintTerm = LT;
                                break;
                            }
                        }

                        if(epigraphConstraint)
                            break;
                    }
                }

                if(epigraphConstraint && epigraphConstraintTerm)
                {
                    double epigraphFactor = -originalObjectiveCoefficient / epigraphConstraintTerm->coefficient;

                    ObjectiveFunctionPtr destinationObjective;

                    if(std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint))
                    {
                        destinationObjective = std::make_shared<NonlinearObjectiveFunction>();
                    }
                    else
                    {
                        destinationObjective = std::make_shared<QuadraticObjectiveFunction>();
                    }

                    for(auto& QT : std::dynamic_pointer_cast<QuadraticConstraint>(epigraphConstraint)->quadraticTerms)
                    {
                        VariablePtr firstVariable = QT->firstVariable;
                        VariablePtr secondVariable = QT->secondVariable;

                        bool firstVariableFixed = firstVariable->lowerBound == firstVariable->upperBound;
                        bool secondVariableFixed = secondVariable->lowerBound == secondVariable->upperBound;

                        if(firstVariableFixed && secondVariableFixed)
                        {
                            (std::static_pointer_cast<LinearObjectiveFunction>(destinationObjective))->constant
                                += epigraphFactor * QT->coefficient * firstVariable->lowerBound
                                * secondVariable->lowerBound;
                        }
                        else if(firstVariableFixed)
                        {
                            (std::static_pointer_cast<LinearObjectiveFunction>(destinationObjective))
                                ->add(std::make_shared<LinearTerm>(
                                    epigraphFactor * QT->coefficient * firstVariable->lowerBound, secondVariable));
                        }
                        else if(secondVariableFixed)
                        {
                            (std::static_pointer_cast<LinearObjectiveFunction>(destinationObjective))
                                ->add(std::make_shared<LinearTerm>(
                                    epigraphFactor * QT->coefficient * secondVariable->lowerBound, firstVariable));
                        }
                        else
                        {
                            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(destinationObjective)
                                ->add(std::make_shared<QuadraticTerm>(
                                    epigraphFactor * QT->coefficient, firstVariable, secondVariable));
                        }
                    }

                    if(std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint))
                    {
                        for(auto& MT :
                            std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint)->monomialTerms)
                        {
                            double coefficient = MT->coefficient;
                            Variables variables;

                            for(auto& V : MT->variables)
                            {
                                if(V->lowerBound == V->upperBound)
                                    coefficient *= V->lowerBound;
                                else
                                    variables.push_back(V);
                            }

                            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destinationObjective)
                                ->add(std::make_shared<MonomialTerm>(epigraphFactor * coefficient, variables));
                        }

                        for(auto& ST :
                            std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint)->signomialTerms)
                        {
                            double coefficient = ST->coefficient;
                            SignomialElements elements;

                            for(auto& E : ST->elements)
                                if(E->variable->lowerBound == E->variable->upperBound)
                                    coefficient *= std::pow(E->variable->lowerBound, E->power);
                                else
                                    elements.push_back(std::make_shared<SignomialElement>(E->variable, E->power));

                            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destinationObjective)
                                ->add(std::make_shared<SignomialTerm>(epigraphFactor * coefficient, elements));
                        }

                        if(std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint)->nonlinearExpression)
                        {
                            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destinationObjective)
                                ->add(simplify(std::make_shared<ExpressionProduct>(
                                    std::make_shared<ExpressionConstant>(epigraphFactor),
                                    copyNonlinearExpression(
                                        std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint)
                                            ->nonlinearExpression.get(),
                                        reformulatedProblem))));
                        }
                    }

                    for(auto& LT : std::dynamic_pointer_cast<LinearConstraint>(epigraphConstraint)->linearTerms)
                    {
                        if(auto variable = LT->variable; variable != epigraphConstraintTerm->variable)
                        {
                            if(variable->lowerBound == variable->upperBound)
                            {
                                std::dynamic_pointer_cast<LinearObjectiveFunction>(destinationObjective)->constant
                                    += epigraphFactor * LT->coefficient * variable->lowerBound;
                            }
                            else
                            {
                                std::dynamic_pointer_cast<LinearObjectiveFunction>(destinationObjective)
                                    ->add(std::make_shared<LinearTerm>(epigraphFactor * LT->coefficient, variable));
                            }
                        }
                    }

                    destinationObjective->ownerProblem = reformulatedProblem;
                    destinationObjective->direction = sourceObjective->direction;

                    destinationObjective->constant = env->problem->objectiveFunction->constant
                        + epigraphFactor
                            * (std::dynamic_pointer_cast<LinearConstraint>(epigraphConstraint)->constant
                                - std::dynamic_pointer_cast<LinearConstraint>(epigraphConstraint)->valueRHS);

                    // Remove constraint
                    if(std::dynamic_pointer_cast<QuadraticConstraint>(epigraphConstraint))
                    {
                        reformulatedProblem->quadraticConstraints.erase(
                            std::remove(reformulatedProblem->quadraticConstraints.begin(),
                                reformulatedProblem->quadraticConstraints.end(),
                                std::dynamic_pointer_cast<QuadraticConstraint>(epigraphConstraint)),
                            reformulatedProblem->quadraticConstraints.end());

                        reformulatedProblem->numericConstraints.erase(
                            std::remove(reformulatedProblem->numericConstraints.begin(),
                                reformulatedProblem->numericConstraints.end(), epigraphConstraint),
                            reformulatedProblem->numericConstraints.end());
                    }

                    if(std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint))
                    {
                        reformulatedProblem->nonlinearConstraints.erase(
                            std::remove(reformulatedProblem->nonlinearConstraints.begin(),
                                reformulatedProblem->nonlinearConstraints.end(),
                                std::dynamic_pointer_cast<NonlinearConstraint>(epigraphConstraint)),
                            reformulatedProblem->nonlinearConstraints.end());

                        reformulatedProblem->numericConstraints.erase(
                            std::remove(reformulatedProblem->numericConstraints.begin(),
                                reformulatedProblem->numericConstraints.end(), epigraphConstraint),
                            reformulatedProblem->numericConstraints.end());
                    }

                    reformulatedProblem->antiEpigraphObjectiveVariable = epigraphConstraintTerm->variable;

                    reformulatedProblem->add(std::move(destinationObjective));

                    return;
                }
            }
        }

        auto destinationObjective = std::make_shared<LinearObjectiveFunction>();
        destinationObjective->ownerProblem = reformulatedProblem;

        copyLinearTermsToObjectiveFunction(sourceObjective->linearTerms, destinationObjective);

        destinationObjective->direction = env->problem->objectiveFunction->direction;
        destinationObjective->constant = env->problem->objectiveFunction->constant;

        reformulatedProblem->add(std::move(destinationObjective));

        return;
    }

    if(((env->problem->properties.convexity == E_ProblemConvexity::Convex && useConvexQuadraticObjective)
           || useNonconvexQuadraticObjective)
        && env->problem->objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic)
    {
        // Quadratic objective function
        auto destinationObjective = std::make_shared<QuadraticObjectiveFunction>();
        destinationObjective->ownerProblem = reformulatedProblem;
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

    if(env->problem->objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic
        && env->settings->getSetting<bool>("Reformulation.ObjectiveFunction.Epigraph.Use", "Model"))
    {
        // Rewrite objective as objective constraint, aka epigraph

        createEpigraphConstraint();
        return;
    }

    // Objective is to be regarded as nonlinear

    bool copyOriginalLinearTerms = false;
    bool copyOriginalNonlinearExpression = false;

    // These will be added to the new constraint, and their signs have been altered
    LinearTerms destinationLinearTerms;
    destinationLinearTerms.takeOwnership(reformulatedProblem);
    QuadraticTerms destinationQuadraticTerms;
    destinationQuadraticTerms.takeOwnership(reformulatedProblem);
    MonomialTerms destinationMonomialTerms;
    destinationMonomialTerms.takeOwnership(reformulatedProblem);
    SignomialTerms destinationSignomialTerms;
    destinationSignomialTerms.takeOwnership(reformulatedProblem);

    bool isSignReversed = env->problem->objectiveFunction->properties.isMaximize;

    if(env->problem->objectiveFunction->properties.hasLinearTerms)
        copyOriginalLinearTerms = true;

    if(env->problem->objectiveFunction->properties.hasQuadraticTerms)
    {
        auto sourceObjective = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction);

        auto [tmpLinearTerms, tmpQuadraticTerms] = reformulateAndPartitionQuadraticSum(sourceObjective->quadraticTerms,
            isSignReversed,
            static_cast<ES_PartitionNonlinearSums>(
                env->settings->getSetting<int>("Reformulation.ObjectiveFunction.PartitionQuadraticTerms", "Model")));

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
            == ES_PartitionNonlinearSums::Always)
        {
            auto tmpLinearTerms = partitionSignomialTerms(sourceObjective->signomialTerms, isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else if(static_cast<ES_PartitionNonlinearSums>(
                    env->settings->getSetting<int>("Reformulation.ObjectiveFunction.PartitionNonlinearTerms", "Model"))
            == ES_PartitionNonlinearSums::IfConvex)
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

    objective->ownerProblem = reformulatedProblem;

    objective->constant = env->problem->objectiveFunction->constant;
    objective->direction = env->problem->objectiveFunction->direction;

    if(copyOriginalLinearTerms)
        copyLinearTermsToObjectiveFunction(
            std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms,
            std::dynamic_pointer_cast<LinearObjectiveFunction>(objective));

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
                    reformulateNonlinearExpression(simplify(std::make_shared<ExpressionNegate>(
                        copyNonlinearExpression(sourceObjective->nonlinearExpression.get(), reformulatedProblem)))));
            else
                std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective)->add(reformulateNonlinearExpression(
                    copyNonlinearExpression(sourceObjective->nonlinearExpression.get(), reformulatedProblem)));
        }
    }

    if(copyOriginalNonlinearExpression)
    {
        reformulatedProblem->add(std::dynamic_pointer_cast<NonlinearObjectiveFunction>(objective));
    }
    else if(destinationQuadraticTerms.size() > 0)
    {
        reformulatedProblem->add(std::dynamic_pointer_cast<QuadraticObjectiveFunction>(objective));
    }
    else
    {
        reformulatedProblem->add(std::dynamic_pointer_cast<LinearObjectiveFunction>(objective));
    }
}

void TaskReformulateProblem::createEpigraphConstraint()
{
    double objVarBound = env->settings->getSetting<double>("Variables.NonlinearObjectiveVariable.Bound", "Model");

    Interval objectiveBound;

    try
    {
        objectiveBound = env->problem->objectiveFunction->getBounds();
    }
    catch(mc::Interval::Exceptions&)
    {
        objectiveBound = Interval(-objVarBound, objVarBound);
    }

    auto objectiveVariable = std::make_shared<AuxiliaryVariable>(
        "shot_objvar", auxVariableCounter, E_VariableType::Real, objectiveBound.l(), objectiveBound.u());
    auxVariableCounter++;
    objectiveVariable->properties.auxiliaryType = E_AuxiliaryVariableType::NonlinearObjectiveFunction;
    env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearObjectiveFunction);

    if(env->problem->objectiveFunction->properties.hasLinearTerms)
    {
        for(auto& T : std::dynamic_pointer_cast<LinearObjectiveFunction>(env->problem->objectiveFunction)->linearTerms)
        {
            objectiveVariable->linearTerms.add(
                std::make_shared<LinearTerm>(T->coefficient, reformulatedProblem->getVariable(T->variable->index)));
        }
    }

    if(env->problem->objectiveFunction->properties.hasQuadraticTerms)
    {
        for(auto& T :
            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms)
        {
            objectiveVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(T->coefficient,
                reformulatedProblem->getVariable(T->firstVariable->index),
                reformulatedProblem->getVariable(T->secondVariable->index)));
        }
    }

    if(env->problem->objectiveFunction->properties.hasMonomialTerms)
    {
        for(auto& T :
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->monomialTerms)
        {
            objectiveVariable->monomialTerms.add(std::make_shared<MonomialTerm>(T.get(), reformulatedProblem));
        }
    }

    if(env->problem->objectiveFunction->properties.hasSignomialTerms)
    {
        for(auto& T :
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->signomialTerms)
        {
            objectiveVariable->signomialTerms.add(std::make_shared<SignomialTerm>(T.get(), reformulatedProblem));
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
            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->problem->objectiveFunction)->quadraticTerms,
            constraint, isSignReversed);
    }

    if(env->problem->objectiveFunction->properties.hasMonomialTerms)
    {
        copyMonomialTermsToConstraint(
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->monomialTerms,
            constraint, isSignReversed);
    }

    if(env->problem->objectiveFunction->properties.hasSignomialTerms)
    {
        copySignomialTermsToConstraint(
            std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)->signomialTerms,
            constraint, isSignReversed);
    }

    if(env->problem->objectiveFunction->properties.hasNonlinearExpression)
    {
        if(isSignReversed)
        {
            constraint->add(
                reformulateNonlinearExpression(simplify(std::make_shared<ExpressionNegate>(copyNonlinearExpression(
                    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                        ->nonlinearExpression.get(),
                    reformulatedProblem)))));
        }
        else
        {
            constraint->add(reformulateNonlinearExpression(copyNonlinearExpression(
                std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->problem->objectiveFunction)
                    ->nonlinearExpression.get(),
                reformulatedProblem)));
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
        constraint->ownerProblem = reformulatedProblem;
        auto sourceConstraint = std::dynamic_pointer_cast<LinearConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);
        constraint->constant += constant;

        return (NumericConstraints({ constraint }));
    }

    bool isQuadraticConstraint = C->properties.classification == E_ConstraintClassification::Quadratic
        || (!C->properties.hasNonlinearExpression && !C->properties.hasMonomialTerms
            && !C->properties.hasSignomialTerms);

    if(isQuadraticConstraint
        && (C->properties.convexity == E_Convexity::Convex
            && ((useConvexQuadraticConstraints
                    && std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms.minEigenValue >= 0.0)
                || (useConvexQuadraticConstraintsWithinTolerance
                    && std::dynamic_pointer_cast<QuadraticConstraint>(C)->quadraticTerms.minEigenValueWithinTolerance)
                || useNonconvexQuadraticConstraints)))
    {
        // Quadratic constraint (not considered as nonlinear)
        QuadraticConstraintPtr constraint
            = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Quadratic;
        constraint->ownerProblem = reformulatedProblem;
        auto sourceConstraint = std::dynamic_pointer_cast<QuadraticConstraint>(C);

        copyLinearTermsToConstraint(sourceConstraint->linearTerms, constraint);
        copyQuadraticTermsToConstraint(sourceConstraint->quadraticTerms, constraint);

        constraint->constant += constant;

        return (NumericConstraints({ constraint }));
    }

    // Constraint is to be regarded as nonlinear

    bool copyOriginalNonlinearExpression = false;

    // These will be added to the new constraint, and their signs have been altered
    LinearTerms destinationLinearTerms;
    destinationLinearTerms.takeOwnership(reformulatedProblem);
    QuadraticTerms destinationQuadraticTerms;
    destinationQuadraticTerms.takeOwnership(reformulatedProblem);
    MonomialTerms destinationMonomialTerms;
    destinationMonomialTerms.takeOwnership(reformulatedProblem);
    SignomialTerms destinationSignomialTerms;
    destinationSignomialTerms.takeOwnership(reformulatedProblem);

    // Needs to take ownership of the terms already, otherwise we cannot access the problem from within the term
    destinationLinearTerms.takeOwnership(reformulatedProblem);
    destinationQuadraticTerms.takeOwnership(reformulatedProblem);
    destinationMonomialTerms.takeOwnership(reformulatedProblem);
    destinationSignomialTerms.takeOwnership(reformulatedProblem);

    bool isSignReversed = false;

    auto partitionQuadraticTermsStrategy = static_cast<ES_PartitionNonlinearSums>(
        env->settings->getSetting<int>("Reformulation.Constraint.PartitionQuadraticTerms", "Model"));

    if(C->properties.hasLinearTerms)
    {
        for(auto& T : std::dynamic_pointer_cast<LinearConstraint>(C)->linearTerms)
            destinationLinearTerms.add(std::make_shared<LinearTerm>(T->coefficient, T->variable));
    }

    if(C->properties.hasQuadraticTerms)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<QuadraticConstraint>(C);

        auto [tmpLinearTerms, tmpQuadraticTerms] = reformulateAndPartitionQuadraticSum(
            sourceConstraint->quadraticTerms, isSignReversed, partitionQuadraticTermsStrategy);

        destinationLinearTerms.add(tmpLinearTerms);
        destinationQuadraticTerms.add(tmpQuadraticTerms);
    }

    if(C->properties.hasMonomialTerms)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

        if(env->settings->getSetting<int>("Reformulation.Monomials.Formulation", "Model")
            != static_cast<int>(ES_ReformulationBinaryMonomials::None))
        {
            auto [tmpLinearTerms, tmpMonomialTerms]
                = reformulateMonomialSum(sourceConstraint->monomialTerms, isSignReversed);

            if(tmpMonomialTerms.size() == 0)
            {
                // All monomials have been reformulated
                destinationLinearTerms.add(tmpLinearTerms);
            }
            else
            {
                if(static_cast<ES_PartitionNonlinearSums>(
                       env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                        == ES_PartitionNonlinearSums::Always
                    && tmpMonomialTerms.size() > 1)
                {
                    auto tmpLinearTerms = partitionMonomialTerms(tmpMonomialTerms, isSignReversed);
                    destinationLinearTerms.add(tmpLinearTerms);
                }
                else // Monomials are always nonconvex
                {
                    for(auto& T : sourceConstraint->monomialTerms)
                        destinationMonomialTerms.add(std::make_shared<MonomialTerm>(T.get(), reformulatedProblem));
                }
            }
        }
        else
        {
            if(static_cast<ES_PartitionNonlinearSums>(
                   env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                    == ES_PartitionNonlinearSums::Always
                && destinationMonomialTerms.size() > 1)
            {
                auto tmpLinearTerms = partitionMonomialTerms(destinationMonomialTerms, isSignReversed);
                destinationLinearTerms.add(tmpLinearTerms);
            }
            else // Monomials are always nonconvex
            {
                for(auto& T : sourceConstraint->monomialTerms)
                    destinationMonomialTerms.add(std::make_shared<MonomialTerm>(T.get(), reformulatedProblem));
            }
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

    NonlinearExpressionPtr destinationExpression;

    if(C->properties.hasNonlinearExpression)
    {
        auto sourceConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);

        // Now trying to reformulate the nonlinear expression
        auto reformulatedExpression = simplify(reformulateNonlinearExpression(
            copyNonlinearExpression(sourceConstraint->nonlinearExpression.get(), reformulatedProblem)));

        auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression,
            tmpConstant]
            = extractTermsAndConstant(reformulatedExpression, true, true, true, true);

        if(tmpLinearTerms.size() > 0)
            destinationLinearTerms.add(tmpLinearTerms);

        if(tmpQuadraticTerms.size() > 0)
        {
            auto [tmpLinearTerms2, tmpQuadraticTerms2] = reformulateAndPartitionQuadraticSum(
                tmpQuadraticTerms, isSignReversed, partitionQuadraticTermsStrategy);

            destinationLinearTerms.add(tmpLinearTerms2);
            destinationQuadraticTerms.add(tmpQuadraticTerms2);
        }

        if(tmpMonomialTerms.size() > 0)
            destinationMonomialTerms.add(tmpMonomialTerms);

        if(tmpSignomialTerms.size() > 0)
            destinationSignomialTerms.add(tmpSignomialTerms);

        destinationExpression = tmpNonlinearExpression;
    }

    if(destinationExpression)
    {
        if(static_cast<ES_PartitionNonlinearSums>(
               env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::Always
            && destinationExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto tmpLinearTerms = partitionNonlinearSum(
                std::dynamic_pointer_cast<ExpressionSum>(destinationExpression), isSignReversed);
            destinationLinearTerms.add(tmpLinearTerms);
        }
        else if(static_cast<ES_PartitionNonlinearSums>(
                    env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model"))
                == ES_PartitionNonlinearSums::IfConvex
            && destinationExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto sum = std::dynamic_pointer_cast<ExpressionSum>(destinationExpression);
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
        else if(destinationExpression->getType() == E_NonlinearExpressionTypes::Constant
            && std::dynamic_pointer_cast<ExpressionConstant>(destinationExpression)->constant == 0.0)
        {
            // Nonlinear expression is constant zero
        }
        else
        {
            copyOriginalNonlinearExpression = true;
        }
    }

    NumericConstraints resultingConstraints;
    NumericConstraintPtr constraint;

    if(copyOriginalNonlinearExpression || destinationMonomialTerms.size() > 0)
    // We have a nonlinear constraint
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Nonlinear;
        constraint->ownerProblem = reformulatedProblem;
    }
    else if(destinationSignomialTerms.size() > 0)
    {
        bool transformed = false;

        if(destinationSignomialTerms.size() == 1 && destinationQuadraticTerms.size() == 0
            && destinationLinearTerms.size() == 0 && destinationSignomialTerms[0]->elements.size() > 1
            && destinationSignomialTerms[0]->coefficient < 0.0 && valueLHS == SHOT_DBL_MIN && valueRHS < 0.0)
        // We can perhaps use the transformation for terms of the type  c * x1^p1 * ... xn^pn <= d, c,d < 0
        {
            if(std::all_of(destinationSignomialTerms[0]->elements.begin(), destinationSignomialTerms[0]->elements.end(),
                   [](SignomialElementPtr E) { return (E->power > 0.0 && E->variable->lowerBound > 0.0); }))
            {
                // All coefficients are negative and variable positive, i.e. we can use the reformulation

                double remainingRHS = std::log(valueRHS * destinationSignomialTerms[0]->coefficient);

                constraint = std::make_shared<LinearConstraint>(C->index, C->name, SHOT_DBL_MIN, remainingRHS);
                constraint->properties.classification = E_ConstraintClassification::Linear;
                constraint->ownerProblem = reformulatedProblem;

                for(auto& E : destinationSignomialTerms[0]->elements)
                {
                    auto auxVariable = std::make_shared<AuxiliaryVariable>(
                        "s_rnsig_" + std::to_string(auxVariableCounter + 1), auxVariableCounter, E_VariableType::Real,
                        -E->power * std::log(E->variable->upperBound), SHOT_DBL_MAX);

                    auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::NonlinearExpressionPartitioning;
                    auxVariableCounter++;
                    env->results->increaseAuxiliaryVariableCounter(
                        E_AuxiliaryVariableType::NonlinearExpressionPartitioning);

                    reformulatedProblem->add(auxVariable);
                    destinationLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

                    auto auxConstraint = std::make_shared<NonlinearConstraint>(
                        auxConstraintCounter, "s_rnsig_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
                    auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

                    auxConstraint->properties.classification = E_ConstraintClassification::Nonlinear;
                    auxConstraintCounter++;

                    NonlinearExpressionPtr expression
                        = std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(-E->power),
                            std::make_shared<ExpressionLog>(std::make_shared<ExpressionVariable>(
                                reformulatedProblem->getVariable(E->variable->index))));

                    auxConstraint->add(std::move(expression));
                    auxVariable->nonlinearExpression = auxConstraint->nonlinearExpression;

                    resultingConstraints.push_back(std::move(auxConstraint));
                }

                destinationSignomialTerms.clear();
                transformed = true;
            }
        }
        else if(destinationSignomialTerms.size() == 1 && destinationQuadraticTerms.size() == 0
            && destinationLinearTerms.size() == 1 && destinationSignomialTerms[0]->elements.size() > 1
            && destinationSignomialTerms[0]->coefficient > 0.0 && destinationLinearTerms[0]->coefficient < 0
            && valueLHS == SHOT_DBL_MIN && valueRHS <= 0.0)
        // We can perhaps use the transformation for terms of the type  c * x1^p1 * ... xn^pn <= y, c > 0
        {
            if(std::all_of(destinationSignomialTerms[0]->elements.begin(), destinationSignomialTerms[0]->elements.end(),
                   [](SignomialElementPtr E) { return (E->power < 0.0 && E->variable->lowerBound > 0.0); }))
            {
                // All coefficients are negative and variable positive, i.e. we can use the reformulation

                constraint = std::make_shared<LinearConstraint>(C->index, C->name, SHOT_DBL_MIN, 0.0);
                constraint->properties.classification = E_ConstraintClassification::Linear;
                constraint->ownerProblem = reformulatedProblem;

                for(auto& E : destinationSignomialTerms[0]->elements)
                {
                    auto auxVariable
                        = std::make_shared<AuxiliaryVariable>("s_rpsig_" + std::to_string(auxVariableCounter + 1),
                            auxVariableCounter, E_VariableType::Real, SHOT_DBL_MIN, 0.0);

                    auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::NonlinearExpressionPartitioning;
                    auxVariableCounter++;
                    env->results->increaseAuxiliaryVariableCounter(
                        E_AuxiliaryVariableType::NonlinearExpressionPartitioning);

                    reformulatedProblem->add(auxVariable);

                    std::dynamic_pointer_cast<LinearConstraint>(constraint)
                        ->add(std::make_shared<LinearTerm>(1.0, auxVariable));

                    auto auxConstraint = std::make_shared<NonlinearConstraint>(
                        auxConstraintCounter, "s_rpsig_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
                    auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

                    auxConstraint->properties.classification = E_ConstraintClassification::Nonlinear;
                    auxConstraint->ownerProblem = reformulatedProblem;
                    auxConstraintCounter++;

                    NonlinearExpressionPtr expression
                        = std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(E->power),
                            std::make_shared<ExpressionLog>(std::make_shared<ExpressionVariable>(
                                reformulatedProblem->getVariable(E->variable->index))));

                    auxConstraint->add(std::move(expression));
                    auxVariable->nonlinearExpression = auxConstraint->nonlinearExpression;

                    resultingConstraints.push_back(std::move(auxConstraint));
                }

                auto auxVariable
                    = std::make_shared<AuxiliaryVariable>("s_rpsig_" + std::to_string(auxVariableCounter + 1),
                        auxVariableCounter, E_VariableType::Real, SHOT_DBL_MIN, SHOT_DBL_MAX);

                auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::NonlinearExpressionPartitioning;
                auxVariableCounter++;
                env->results->increaseAuxiliaryVariableCounter(
                    E_AuxiliaryVariableType::NonlinearExpressionPartitioning);

                reformulatedProblem->add(auxVariable);

                std::dynamic_pointer_cast<LinearConstraint>(constraint)
                    ->add(std::make_shared<LinearTerm>(1.0, auxVariable));

                auto auxConstraint = std::make_shared<NonlinearConstraint>(
                    auxConstraintCounter, "s_rpsig_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
                auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

                auxConstraint->properties.classification = E_ConstraintClassification::Nonlinear;
                auxConstraintCounter++;

                NonlinearExpressionPtr expression = std::make_shared<ExpressionProduct>(
                    std::make_shared<ExpressionConstant>(destinationLinearTerms[0]->coefficient),
                    std::make_shared<ExpressionLog>(
                        std::make_shared<ExpressionVariable>(destinationLinearTerms[0]->variable)));

                auxConstraint->add(std::move(expression));
                auxVariable->nonlinearExpression = auxConstraint->nonlinearExpression;

                resultingConstraints.push_back(std::move(auxConstraint));

                destinationLinearTerms.clear();
                destinationSignomialTerms.clear();
                transformed = true;
            }
        }

        if(!transformed)
        {
            constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
            constraint->properties.classification = E_ConstraintClassification::Nonlinear;
            constraint->ownerProblem = reformulatedProblem;
        }
    }
    else if(destinationQuadraticTerms.size() == 0)
    // We have a linear constraint
    {
        constraint = std::make_shared<LinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Linear;
        constraint->ownerProblem = reformulatedProblem;
    }
    else if(!useConvexQuadraticConstraints)
    // We have a quadratic constraint, but it will be considered as nonlinear since the user demands it
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::QuadraticConsideredAsNonlinear;
        constraint->ownerProblem = reformulatedProblem;
    }
    else if(destinationQuadraticTerms.getConvexity() != E_Convexity::Convex && !useNonconvexQuadraticConstraints)
    // We have a quadratic constraint, but it will be considered as nonlinear since it is nonconvex
    {
        constraint = std::make_shared<NonlinearConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::QuadraticConsideredAsNonlinear;
        constraint->ownerProblem = reformulatedProblem;
    }
    else
    // We have quadratic constraint
    {
        constraint = std::make_shared<QuadraticConstraint>(C->index, C->name, valueLHS, valueRHS);
        constraint->properties.classification = E_ConstraintClassification::Quadratic;
        constraint->ownerProblem = reformulatedProblem;
    }

    constraint->constant += constant;

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
                ->add(simplify(std::make_shared<ExpressionNegate>(destinationExpression)));
        else
            std::dynamic_pointer_cast<NonlinearConstraint>(constraint)->add(destinationExpression);
    }

    resultingConstraints.insert(resultingConstraints.begin(), constraint);

    return (NumericConstraints({ resultingConstraints }));
}

LinearTerms TaskReformulateProblem::partitionNonlinearSum(
    const std::shared_ptr<ExpressionSum> source, bool reversedSigns)
{
    LinearTerms resultLinearTerms;
    resultLinearTerms.takeOwnership(reformulatedProblem);

    if(source.get() == nullptr)
        return (resultLinearTerms);

    bool allNonlinearExpressionsReformulated = false;

    for(auto& T : source->children)
    {
        if(T->getType() == E_NonlinearExpressionTypes::Product) // Might be able to reuse auxiliary variable
                                                                // further if e.g. bilinear term
        {
            auto optionalQuadraticTerm
                = reformulateProductToQuadraticTerm(std::dynamic_pointer_cast<ExpressionProduct>(T));

            if(optionalQuadraticTerm) // The product was a quadratic term
            {
                QuadraticTerms quadTerms;
                quadTerms.takeOwnership(reformulatedProblem);
                quadTerms.add(optionalQuadraticTerm.value());

                auto [tmpLinearTerms, tmpQuadraticTerms] = reformulateAndPartitionQuadraticSum(quadTerms, reversedSigns,
                    static_cast<ES_PartitionNonlinearSums>(
                        env->settings->getSetting<int>("Reformulation.Constraint.PartitionNonlinearTerms", "Model")));

                if(tmpQuadraticTerms.size() == 0)
                // Otherwise we cannot proceed and will continue as if nonbilinear term
                {
                    resultLinearTerms.add(tmpLinearTerms);
                    continue; // Continue to next nonlinear term
                }
            }

            auto optionalMonomialTerm
                = reformulateProductToMonomialTerm(std::dynamic_pointer_cast<ExpressionProduct>(T));

            if(optionalMonomialTerm
                && env->settings->getSetting<int>("Reformulation.Monomials.Formulation", "Model")
                    != static_cast<int>(ES_ReformulationBinaryMonomials::None))
            // The product was a monomial term
            {
                MonomialTerms monomialTerms;
                monomialTerms.takeOwnership(reformulatedProblem);
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

            double varLowerBound = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
            double varUpperBound = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");

            try
            {
                bounds = T->getBounds();

                if(reversedSigns)
                    bounds = -1.0 * bounds;
            }
            catch(mc::Interval::Exceptions&)
            {
                bounds = Interval(varLowerBound, varUpperBound);
            }

            auto auxVariable = std::make_shared<AuxiliaryVariable>("s_pnl_" + std::to_string(auxVariableCounter + 1),
                auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
            auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::NonlinearExpressionPartitioning;
            auxVariableCounter++;
            env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearExpressionPartitioning);

            resultLinearTerms.add(std::make_shared<LinearTerm>(1.0, auxVariable));

            bool extractQuadraticTerms
                = (env->settings->getSetting<int>("Reformulation.Quadratics.ExtractStrategy", "Model")
                    >= static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame));

            if(static_cast<ES_QuadraticProblemStrategy>(
                   env->settings->getSetting<int>("Reformulation.Quadratics.Strategy", "Model"))
                < ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained)
                extractQuadraticTerms = false;

            // If the extracted term is quadratic, create a quadratic constraint instead of a nonlinear one
            if(extractQuadraticTerms && T->getType() == E_NonlinearExpressionTypes::Product
                && std::dynamic_pointer_cast<ExpressionProduct>(T)->isQuadraticTerm())
            {
                auto quadraticTerm
                    = reformulateProductToQuadraticTerm(std::dynamic_pointer_cast<ExpressionProduct>(T)).value();

                auto auxConstraint = std::make_shared<QuadraticConstraint>(
                    auxConstraintCounter, "s_pqnl_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
                auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
                auxConstraintCounter++;

                if(reversedSigns)
                {
                    quadraticTerm->coefficient *= -1.0;
                    auxConstraint->add(quadraticTerm);
                }
                else
                {
                    auxConstraint->add(quadraticTerm);
                }

                reformulatedProblem->add(std::move(auxVariable));
                reformulatedProblem->add(std::move(auxConstraint));
            }
            else if(extractQuadraticTerms && T->getType() == E_NonlinearExpressionTypes::Square
                && std::dynamic_pointer_cast<ExpressionSquare>(T)->child->getType()
                    == E_NonlinearExpressionTypes::Variable)
            {
                auto variable = std::dynamic_pointer_cast<ExpressionVariable>(
                    std::dynamic_pointer_cast<ExpressionSquare>(T)->child);

                auto quadraticTerm = std::make_shared<QuadraticTerm>(1.0, variable->variable, variable->variable);
                auto auxConstraint = std::make_shared<QuadraticConstraint>(
                    auxConstraintCounter, "s_psnl_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
                auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
                auxConstraintCounter++;

                if(reversedSigns)
                {
                    quadraticTerm->coefficient *= -1.0;
                    auxConstraint->add(quadraticTerm);
                }
                else
                {
                    auxConstraint->add(quadraticTerm);
                }

                reformulatedProblem->add(std::move(auxVariable));
                reformulatedProblem->add(std::move(auxConstraint));
            }
            else
            {
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
    }

    return (resultLinearTerms);
}

LinearTerms TaskReformulateProblem::partitionMonomialTerms(const MonomialTerms sourceTerms, bool reversedSigns)
{
    LinearTerms resultLinearTerms;
    resultLinearTerms.takeOwnership(reformulatedProblem);

    if(sourceTerms.size() == 0)
        return (resultLinearTerms);

    for(auto& T : sourceTerms)
    {
        Interval bounds;

        double varLowerBound = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
        double varUpperBound = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");

        try
        {
            bounds = T->getBounds();

            if(reversedSigns)
                bounds = -1.0 * bounds;
        }
        catch(mc::Interval::Exceptions&)
        {
            bounds = Interval(varLowerBound, varUpperBound);
        }

        auto auxVariable = std::make_shared<AuxiliaryVariable>("s_pmon_" + std::to_string(auxVariableCounter + 1),
            auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::MonomialTermsPartitioning;
        auxVariableCounter++;
        env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::MonomialTermsPartitioning);

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
    resultLinearTerms.takeOwnership(reformulatedProblem);

    if(sourceTerms.size() == 0)
        return (resultLinearTerms);

    for(auto& T : sourceTerms)
    {
        Interval bounds;

        double varLowerBound = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
        double varUpperBound = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");

        double coefficient = std::abs(T->coefficient);

        try
        {
            bounds = T->getBounds() / coefficient;

            if(reversedSigns)
                bounds = -1.0 * bounds;
        }
        catch(mc::Interval::Exceptions&)
        {
            bounds = Interval(varLowerBound, varUpperBound);
        }

        auto auxVariable = std::make_shared<AuxiliaryVariable>("s_psig_" + std::to_string(auxVariableCounter + 1),
            auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
        auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::SignomialTermsPartitioning;
        auxVariableCounter++;
        env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::SignomialTermsPartitioning);

        resultLinearTerms.add(std::make_shared<LinearTerm>(coefficient, auxVariable));

        auto auxConstraint = std::make_shared<NonlinearConstraint>(
            auxConstraintCounter, "cs_psig_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
        auxConstraintCounter++;

        auto signomialTerm = std::make_shared<SignomialTerm>(T.get(), reformulatedProblem);
        signomialTerm->coefficient /= coefficient;

        if(reversedSigns)
        {
            signomialTerm->coefficient *= -1.0;
        }

        if(signomialTerm->coefficient < 0.0 && auxVariable->upperBound > 0.0)
            auxVariable->upperBound = 0.0;
        else if(signomialTerm->coefficient > 0.0 && auxVariable->lowerBound < 0.0)
            auxVariable->lowerBound = 0.0;

        auxConstraint->add(signomialTerm);

        auxVariable->signomialTerms.push_back(signomialTerm);

        reformulatedProblem->add(std::move(auxVariable));

        auto numericConstraints = reformulateConstraint(auxConstraint);

        for(auto& C : numericConstraints)
            reformulatedProblem->add(std::move(C));
    }

    return (resultLinearTerms);
}

std::tuple<LinearTerms, QuadraticTerms> TaskReformulateProblem::reformulateAndPartitionQuadraticSum(
    QuadraticTerms& quadraticTerms, bool reversedSigns, ES_PartitionNonlinearSums partitionStrategy)
{
    LinearTerms resultLinearTerms;
    resultLinearTerms.takeOwnership(reformulatedProblem);
    QuadraticTerms resultQuadraticTerms;
    resultQuadraticTerms.takeOwnership(reformulatedProblem);

    bool performPartitioning = true;

    if(partitionStrategy != ES_PartitionNonlinearSums::Always)
    {
        for(auto& T : quadraticTerms)
        {
            if(!T->isSquare
                && (T->firstVariable->upperBound > 1e15 || T->secondVariable->upperBound > 1e15
                    || T->firstVariable->lowerBound < -1e15 || T->secondVariable->lowerBound < -1e15))
            {
                performPartitioning = false;
                break;
            }
            else if((!reversedSigns && (T->getConvexity() == E_Convexity::Convex))
                || (reversedSigns && (T->getConvexity() == E_Convexity::Concave)))
            {
            }
            else if(T->isSquare && T->isBinary) // Square term b^2 -> b
            {
            }
            else if(T->isBilinear && T->isBinary) // Bilinear term b1*b2
            {
            }
            else if(T->isBilinear
                && (T->firstVariable->properties.type == E_VariableType::Binary
                    || T->secondVariable->properties.type == E_VariableType::Binary))
            // Bilinear term b1*x2 or x1*b2
            {
            }
            else if(useIntegerBilinearTermReformulation && T->isBilinear
                && (((T->firstVariable->properties.type == E_VariableType::Integer
                         || T->firstVariable->properties.type == E_VariableType::Semiinteger)
                        && (T->firstVariable->upperBound - T->firstVariable->lowerBound
                            < maxBilinearIntegerReformulationDomain))
                    || ((T->secondVariable->properties.type == E_VariableType::Integer
                            || T->secondVariable->properties.type == E_VariableType::Semiinteger)
                        && (T->secondVariable->upperBound - T->secondVariable->lowerBound
                            < maxBilinearIntegerReformulationDomain))))
            // bilinear term i1*i2 or i1*x2
            {
            }
            else if(extractQuadraticTermsFromNonconvexExpressions) // Bilinear term +x1*x2 which will be extracted
                                                                   // to equality constraint
            {
            }
            else // Remaining nonconvex terms x1*x2 will remain as is
            {
                performPartitioning = false;
                break;
            }
        }
    }

    bool quadraticSumConvex = quadraticTerms.minEigenValueWithinTolerance;
    bool quadraticSumConcave = quadraticTerms.maxEigenValueWithinTolerance;

    bool allTermsConvex = quadraticTerms.checkAllForConvexityType(E_Convexity::Convex);
    bool allTermsConcave = quadraticTerms.checkAllForConvexityType(E_Convexity::Concave);

    if(env->settings->getSetting<bool>("Reformulation.Quadratics.EigenValueDecomposition.Use", "Model")
        && partitionStrategy <= ES_PartitionNonlinearSums::IfConvex && quadraticSumConvex
        && !quadraticTerms.allSquares) // Use the eigenvalue decomposition reformulation
    {
        auto linearTerms = doEigenvalueDecomposition(quadraticTerms);
        resultLinearTerms.add(linearTerms);
    }
    else if(partitionStrategy == ES_PartitionNonlinearSums::Always
        || (!reversedSigns && allTermsConvex && partitionStrategy == ES_PartitionNonlinearSums::IfConvex)
        || (reversedSigns && allTermsConcave && partitionStrategy == ES_PartitionNonlinearSums::IfConvex)
        || (!reversedSigns && !quadraticSumConvex // should not reformulate if sum is convex unless forced
            && performPartitioning && partitionStrategy == ES_PartitionNonlinearSums::IfConvex)
        || (reversedSigns && !quadraticSumConcave // should not reformulate if sum is concave unless forced
            && performPartitioning && partitionStrategy == ES_PartitionNonlinearSums::IfConvex))
    {
        for(auto& T : quadraticTerms)
        {
            auto firstVariable = reformulatedProblem->getVariable(T->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(T->secondVariable->index);

            if(T->isSquare && T->isBinary) // Square term b^2 -> b
            {
                resultLinearTerms.add(std::make_shared<LinearTerm>(T->coefficient, firstVariable));
            }
            else if(T->isSquare)
            {
                auto [auxVariable, newVariable]
                    = getSquareAuxiliaryVariable(firstVariable, 1.0, E_AuxiliaryVariableType::SquareTermsPartitioning);
                resultLinearTerms.add(std::make_shared<LinearTerm>(T->coefficient, auxVariable));
            }
            else if(T->isBilinear && T->isBinary) // Bilinear term b1*b2
            {
                auto [auxVariable, newVariable] = getBilinearAuxiliaryVariable(firstVariable, secondVariable);
                resultLinearTerms.add(std::make_shared<LinearTerm>(T->coefficient, auxVariable));
            }
            else if(T->isBilinear
                && (firstVariable->properties.type == E_VariableType::Binary
                    || secondVariable->properties.type == E_VariableType::Binary))
            // Bilinear term b1*x2 or x1*b2
            {
                auto [auxVariable, newVariable] = getBilinearAuxiliaryVariable(firstVariable, secondVariable);
                resultLinearTerms.add(std::make_shared<LinearTerm>(T->coefficient, auxVariable));
            }
            else if(useIntegerBilinearTermReformulation && T->isBilinear
                && (((firstVariable->properties.type == E_VariableType::Integer
                         || firstVariable->properties.type == E_VariableType::Semiinteger)
                        && (firstVariable->upperBound - firstVariable->lowerBound
                            < maxBilinearIntegerReformulationDomain))
                    || ((secondVariable->properties.type == E_VariableType::Integer
                            || secondVariable->properties.type == E_VariableType::Semiinteger)
                        && (secondVariable->upperBound - secondVariable->lowerBound
                            < maxBilinearIntegerReformulationDomain))))
            // bilinear term i1*i2 or i1*x2
            {
                auto [auxVariable, newVariable] = getBilinearAuxiliaryVariable(firstVariable, secondVariable);
                resultLinearTerms.add(std::make_shared<LinearTerm>(T->coefficient, auxVariable));
            }
            else if(extractQuadraticTermsFromNonconvexExpressions) // Bilinear term +x1*x2 which will be extracted
            // to equality constraint
            {
                auto [auxVariable, newVariable] = getBilinearAuxiliaryVariable(firstVariable, secondVariable);
                resultLinearTerms.add(std::make_shared<LinearTerm>(T->coefficient, auxVariable));
            }
            else // Square term x1^2 or general bilinear term x1*x2 will remain as is
            {
                assert(false);
            }
        }
    }
    else
    {
        for(auto& T : quadraticTerms)
        {
            auto firstVariable = reformulatedProblem->getVariable(T->firstVariable->index);
            auto secondVariable = reformulatedProblem->getVariable(T->secondVariable->index);

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
    resultLinearTerms.takeOwnership(reformulatedProblem);
    MonomialTerms resultMonomialTerms;
    resultMonomialTerms.takeOwnership(reformulatedProblem);

    double signfactor = reversedSigns ? -1.0 : 1.0;

    for(auto& T : monomialTerms)
    {
        if(!T->isBinary)
        {
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
            auxbVar->properties.auxiliaryType = E_AuxiliaryVariableType::BinaryMonomial;
            env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryMonomial);

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

            for(auto i = 1; i < numLambdas; i++)
            {
                auto auxLambda
                    = std::make_shared<AuxiliaryVariable>("s_monlam" + std::to_string(auxVariableCounter + 1),
                        auxVariableCounter + variableOffset, E_VariableType::Real, 0.0, 1.0);
                auxLambda->constant = 1.0 / numLambdas;
                auxLambda->properties.auxiliaryType = E_AuxiliaryVariableType::BinaryMonomial;

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
            auxwVar->properties.auxiliaryType = E_AuxiliaryVariableType::BinaryMonomial;
            env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryMonomial);

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

        if(variable->lowerBound == variable->upperBound)
        {
            std::dynamic_pointer_cast<LinearConstraint>(destination)->constant
                += signCoefficient * LT->coefficient * variable->lowerBound;
        }
        else
        {
            std::dynamic_pointer_cast<LinearConstraint>(destination)
                ->add(std::make_shared<LinearTerm>(signCoefficient * LT->coefficient, variable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToConstraint(QuadraticTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& QT : terms)
    {
        VariablePtr firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
        VariablePtr secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

        bool firstVariableFixed = firstVariable->lowerBound == firstVariable->upperBound;
        bool secondVariableFixed = secondVariable->lowerBound == secondVariable->upperBound;

        if(firstVariableFixed && secondVariableFixed)
        {
            (std::static_pointer_cast<LinearConstraint>(destination))->constant
                += signCoefficient * QT->coefficient * firstVariable->lowerBound * secondVariable->lowerBound;
        }
        else if(firstVariableFixed)
        {
            (std::static_pointer_cast<LinearConstraint>(destination))
                ->add(std::make_shared<LinearTerm>(
                    signCoefficient * QT->coefficient * firstVariable->lowerBound, secondVariable));
        }
        else if(secondVariableFixed)
        {
            (std::static_pointer_cast<LinearConstraint>(destination))
                ->add(std::make_shared<LinearTerm>(
                    signCoefficient * QT->coefficient * secondVariable->lowerBound, firstVariable));
        }
        else
        {
            std::dynamic_pointer_cast<QuadraticConstraint>(destination)
                ->add(
                    std::make_shared<QuadraticTerm>(signCoefficient * QT->coefficient, firstVariable, secondVariable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyMonomialTermsToConstraint(MonomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& MT : terms)
    {
        double coefficient = MT->coefficient;
        Variables variables;

        for(auto& V : MT->variables)
        {
            if(V->lowerBound == V->upperBound)
                coefficient *= V->lowerBound;
            else
                variables.push_back(reformulatedProblem->getVariable(V->index));
        }

        std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination)
            ->add(std::make_shared<MonomialTerm>(signCoefficient * coefficient, variables));
    }
}

template <class T>
void TaskReformulateProblem::copySignomialTermsToConstraint(SignomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& ST : terms)
    {
        double coefficient = ST->coefficient;
        SignomialElements elements;

        for(auto& E : ST->elements)
            if(E->variable->lowerBound == E->variable->upperBound)
                coefficient *= std::pow(E->variable->lowerBound, E->power);
            else
                elements.push_back(
                    std::make_shared<SignomialElement>(reformulatedProblem->getVariable(E->variable->index), E->power));

        std::dynamic_pointer_cast<NonlinearConstraint>(destination)
            ->add(std::make_shared<SignomialTerm>(signCoefficient * coefficient, elements));
    }
}

template <class T>
void TaskReformulateProblem::copyLinearTermsToObjectiveFunction(LinearTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& LT : terms)
    {
        auto variable = reformulatedProblem->getVariable(LT->variable->index);

        if(variable->lowerBound == variable->upperBound)
        {
            std::dynamic_pointer_cast<LinearObjectiveFunction>(destination)->constant
                += signCoefficient * LT->coefficient * variable->lowerBound;
        }
        else
        {
            std::dynamic_pointer_cast<LinearObjectiveFunction>(destination)
                ->add(std::make_shared<LinearTerm>(signCoefficient * LT->coefficient, variable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyQuadraticTermsToObjectiveFunction(
    QuadraticTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& QT : terms)
    {
        VariablePtr firstVariable = reformulatedProblem->getVariable(QT->firstVariable->index);
        VariablePtr secondVariable = reformulatedProblem->getVariable(QT->secondVariable->index);

        bool firstVariableFixed = firstVariable->lowerBound == firstVariable->upperBound;
        bool secondVariableFixed = secondVariable->lowerBound == secondVariable->upperBound;

        if(firstVariableFixed && secondVariableFixed)
        {
            (std::static_pointer_cast<LinearObjectiveFunction>(destination))->constant
                += signCoefficient * QT->coefficient * firstVariable->lowerBound * secondVariable->lowerBound;
        }
        else if(firstVariableFixed)
        {
            (std::static_pointer_cast<LinearObjectiveFunction>(destination))
                ->add(std::make_shared<LinearTerm>(
                    signCoefficient * QT->coefficient * firstVariable->lowerBound, secondVariable));
        }
        else if(secondVariableFixed)
        {
            (std::static_pointer_cast<LinearObjectiveFunction>(destination))
                ->add(std::make_shared<LinearTerm>(
                    signCoefficient * QT->coefficient * secondVariable->lowerBound, firstVariable));
        }
        else
        {
            std::dynamic_pointer_cast<QuadraticObjectiveFunction>(destination)
                ->add(
                    std::make_shared<QuadraticTerm>(signCoefficient * QT->coefficient, firstVariable, secondVariable));
        }
    }
}

template <class T>
void TaskReformulateProblem::copyMonomialTermsToObjectiveFunction(
    MonomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& MT : terms)
    {
        double coefficient = MT->coefficient;
        Variables variables;

        for(auto& V : MT->variables)
        {
            if(V->lowerBound == V->upperBound)
                coefficient *= V->lowerBound;
            else
                variables.push_back(reformulatedProblem->getVariable(V->index));
        }

        std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination)
            ->add(std::make_shared<MonomialTerm>(signCoefficient * coefficient, variables));
    }
}

template <class T>
void TaskReformulateProblem::copySignomialTermsToObjectiveFunction(
    SignomialTerms terms, T destination, bool reversedSigns)
{
    double signCoefficient = (reversedSigns) ? -1.0 : 1.0;

    for(auto& ST : terms)
    {
        double coefficient = ST->coefficient;
        SignomialElements elements;

        for(auto& E : ST->elements)
            if(E->variable->lowerBound == E->variable->upperBound)
                coefficient *= std::pow(E->variable->lowerBound, E->power);
            else
                elements.push_back(
                    std::make_shared<SignomialElement>(reformulatedProblem->getVariable(E->variable->index), E->power));

        std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination)
            ->add(std::make_shared<SignomialTerm>(signCoefficient * coefficient, elements));
    }
}

LinearTerms TaskReformulateProblem::doEigenvalueDecomposition(QuadraticTerms quadraticTerms)
{
    LinearTerms resultLinearTerms;
    resultLinearTerms.takeOwnership(reformulatedProblem);

    for(size_t i = 0; i < quadraticTerms.variableMap.size(); i++)
    {
        if(std::abs(quadraticTerms.eigenvalues[i].real())
            < env->settings->getSetting<double>("Reformulation.Quadratics.EigenValueDecomposition.Tolerance", "Model"))
            continue;

        auto auxConstraint = std::make_shared<LinearConstraint>(
            auxConstraintCounter, "q_evd" + std::to_string(auxConstraintCounter), 0, 0);
        auxConstraintCounter++;

        for(auto [VAR, j] : quadraticTerms.variableMap)
        {
            if(quadraticTerms.eigenvectors(j, i).real() != 0.0)
                auxConstraint->add(std::make_shared<LinearTerm>(quadraticTerms.eigenvectors(j, i).real(), VAR));
        }

        auto bounds = auxConstraint->linearTerms.calculate(env->problem->getVariableBounds());

        auto auxQuadVariable = std::make_shared<AuxiliaryVariable>("q_evd_" + std::to_string(auxVariableCounter),
            auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
        auxVariableCounter++;
        auxQuadVariable->properties.auxiliaryType = E_AuxiliaryVariableType::EigenvalueDecomposition;
        reformulatedProblem->add(auxQuadVariable);

        env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::EigenvalueDecomposition);

        if(env->settings->getSetting<int>("Reformulation.Quadratics.EigenValueDecomposition.Formulation", "Model")
            == static_cast<int>(ES_EigenValueDecompositionFormulation::CoefficientReformulated))
        {
            auto [auxVariable, newVariable] = getSquareAuxiliaryVariable(auxQuadVariable,
                quadraticTerms.eigenvalues[i].real(), E_AuxiliaryVariableType::EigenvalueDecomposition);
            resultLinearTerms.add(std::make_shared<LinearTerm>(0.5, auxVariable));
        }
        else
        {
            auto [auxVariable, newVariable]
                = getSquareAuxiliaryVariable(auxQuadVariable, 1.0, E_AuxiliaryVariableType::EigenvalueDecomposition);
            resultLinearTerms.add(
                std::make_shared<LinearTerm>(0.5 * quadraticTerms.eigenvalues[i].real(), auxVariable));
        }

        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, auxQuadVariable));
        reformulatedProblem->add(std::move(auxConstraint));
    }

    return (resultLinearTerms);
}

NonlinearExpressionPtr TaskReformulateProblem::reformulateNonlinearExpression(NonlinearExpressionPtr source)
{
    switch(source->getType())
    {
    case E_NonlinearExpressionTypes::Constant:
    case E_NonlinearExpressionTypes::Variable:
        break;
        /*case E_NonlinearExpressionTypes::Negate:
            std::dynamic_pointer_cast<ExpressionNegate>(source)->child
                = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionNegate>(source)->child);
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionNegate>(source)));
            break;
        case E_NonlinearExpressionTypes::Invert:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionInvert>(source)));
            break;
        case E_NonlinearExpressionTypes::SquareRoot:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionSquareRoot>(source)));
            break;
        case E_NonlinearExpressionTypes::Square:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionSquare>(source)));
            break;
        case E_NonlinearExpressionTypes::Log:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionLog>(source)));
            break;
        case E_NonlinearExpressionTypes::Exp:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionExp>(source)));
            break;
        case E_NonlinearExpressionTypes::Cos:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionCos>(source)));
            break;
        case E_NonlinearExpressionTypes::ArcCos:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionArcCos>(source)));
            break;
        case E_NonlinearExpressionTypes::Sin:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionSin>(source)));
            break;
        case E_NonlinearExpressionTypes::ArcSin:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionArcSin>(source)));
            break;
        case E_NonlinearExpressionTypes::Tan:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionTan>(source)));
            break;
        case E_NonlinearExpressionTypes::ArcTan:
            return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionArcTan>(source)));
            break;*/
    case E_NonlinearExpressionTypes::Negate:
    case E_NonlinearExpressionTypes::Invert:
    case E_NonlinearExpressionTypes::SquareRoot:
    case E_NonlinearExpressionTypes::Log:
    case E_NonlinearExpressionTypes::Exp:
    case E_NonlinearExpressionTypes::Cos:
    case E_NonlinearExpressionTypes::ArcCos:
    case E_NonlinearExpressionTypes::Sin:
    case E_NonlinearExpressionTypes::ArcSin:
    case E_NonlinearExpressionTypes::Tan:
    case E_NonlinearExpressionTypes::ArcTan:
        std::dynamic_pointer_cast<ExpressionUnary>(source)->child
            = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionUnary>(source)->child);
        break;
    case E_NonlinearExpressionTypes::Square:
        // Extract quadratics
        source = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionSquare>(source));
        break;
    case E_NonlinearExpressionTypes::Abs:
        return (reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionAbs>(source)));
        break;
    case E_NonlinearExpressionTypes::Divide:
        std::dynamic_pointer_cast<ExpressionDivide>(source)->firstChild
            = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionDivide>(source)->firstChild);
        std::dynamic_pointer_cast<ExpressionDivide>(source)->secondChild
            = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionDivide>(source)->secondChild);
        break;
    case E_NonlinearExpressionTypes::Power:
        std::dynamic_pointer_cast<ExpressionPower>(source)->firstChild
            = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionPower>(source)->firstChild);
        std::dynamic_pointer_cast<ExpressionPower>(source)->secondChild
            = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionPower>(source)->secondChild);
        break;
    case E_NonlinearExpressionTypes::Sum:
        for(auto& C : std::dynamic_pointer_cast<ExpressionSum>(source)->children)
            C = reformulateNonlinearExpression(C);
        break;
    case E_NonlinearExpressionTypes::Product:
        // Extract quadratics
        source = reformulateNonlinearExpression(std::dynamic_pointer_cast<ExpressionProduct>(source));
        break;
    }

    return (source);
}

NonlinearExpressionPtr TaskReformulateProblem::reformulateNonlinearExpression(std::shared_ptr<ExpressionAbs> source)
{
    auto [auxVariable, added] = getAbsoluteValueAuxiliaryVariable(source);

    if(!added) // Have already created the auxiliary constraints
        return (std::make_shared<ExpressionVariable>(auxVariable));

    auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression, tmpConstant]
        = extractTermsAndConstant(source->child, true, true, true, true);

    auto bounds = source->getBounds();

    NumericConstraintPtr auxConstraint1;
    NumericConstraintPtr auxConstraint2;

    if(tmpMonomialTerms.size() > 0 || tmpSignomialTerms.size() > 0 || tmpNonlinearExpression)
    {
        auxConstraint1 = std::make_shared<NonlinearConstraint>(
            auxConstraintCounter, "s_cabs_" + std::to_string(auxConstraintCounter) + "_1", SHOT_DBL_MIN, 0.0);
        auxConstraint1->properties.classification = E_ConstraintClassification::Nonlinear;
        auxConstraint1->ownerProblem = reformulatedProblem;
        auxConstraintCounter++;

        auxConstraint2 = std::make_shared<NonlinearConstraint>(
            auxConstraintCounter, "s_cabs_" + std::to_string(auxConstraintCounter) + "_2", SHOT_DBL_MIN, 0.0);
        auxConstraint2->properties.classification = E_ConstraintClassification::Nonlinear;
        auxConstraint2->ownerProblem = reformulatedProblem;
        auxConstraintCounter++;
    }
    else if(tmpQuadraticTerms.size() > 0)
    {
        auxConstraint1 = std::make_shared<QuadraticConstraint>(
            auxConstraintCounter, "s_cabs_" + std::to_string(auxConstraintCounter) + "_1", SHOT_DBL_MIN, 0.0);
        auxConstraint1->properties.classification = E_ConstraintClassification::Quadratic;
        auxConstraint1->ownerProblem = reformulatedProblem;
        auxConstraintCounter++;

        auxConstraint2 = std::make_shared<QuadraticConstraint>(
            auxConstraintCounter, "s_cabs_" + std::to_string(auxConstraintCounter) + "_2", SHOT_DBL_MIN, 0.0);
        auxConstraint2->properties.classification = E_ConstraintClassification::Quadratic;
        auxConstraint2->ownerProblem = reformulatedProblem;
        auxConstraintCounter++;
    }
    else
    {
        auxConstraint1 = std::make_shared<LinearConstraint>(
            auxConstraintCounter, "s_cabs_" + std::to_string(auxConstraintCounter) + "_1", SHOT_DBL_MIN, 0.0);
        auxConstraint1->properties.classification = E_ConstraintClassification::Linear;
        auxConstraint1->ownerProblem = reformulatedProblem;
        auxConstraintCounter++;

        auxConstraint2 = std::make_shared<LinearConstraint>(
            auxConstraintCounter, "s_cabs_" + std::to_string(auxConstraintCounter) + "_2", SHOT_DBL_MIN, 0.0);
        auxConstraint2->properties.classification = E_ConstraintClassification::Linear;
        auxConstraint2->ownerProblem = reformulatedProblem;
        auxConstraintCounter++;
    }

    if(tmpConstant > 0)
    {
        std::dynamic_pointer_cast<LinearConstraint>(auxConstraint1)->constant = tmpConstant;
        std::dynamic_pointer_cast<LinearConstraint>(auxConstraint1)->constant = -tmpConstant;
    }

    if(tmpLinearTerms.size() == 1)
    {
        std::dynamic_pointer_cast<LinearConstraint>(auxConstraint1)->add(tmpLinearTerms);

        copyLinearTermsToConstraint(tmpLinearTerms, std::dynamic_pointer_cast<LinearConstraint>(auxConstraint2), true);
    }
    else if(tmpLinearTerms.size() > 1)
    {
        std::dynamic_pointer_cast<LinearConstraint>(auxConstraint1)->add(tmpLinearTerms);

        copyLinearTermsToConstraint(tmpLinearTerms, std::dynamic_pointer_cast<LinearConstraint>(auxConstraint2), true);
    }

    if(tmpQuadraticTerms.size() > 0)
    {
        std::dynamic_pointer_cast<QuadraticConstraint>(auxConstraint1)->add(tmpQuadraticTerms);

        copyQuadraticTermsToConstraint(
            tmpQuadraticTerms, std::dynamic_pointer_cast<QuadraticConstraint>(auxConstraint2), true);
    }

    if(tmpMonomialTerms.size() > 0)
    {
        std::dynamic_pointer_cast<NonlinearConstraint>(auxConstraint1)->add(tmpMonomialTerms);

        copyMonomialTermsToConstraint(
            tmpMonomialTerms, std::dynamic_pointer_cast<NonlinearConstraint>(auxConstraint2), true);
    }

    if(tmpSignomialTerms.size() > 0)
    {
        std::dynamic_pointer_cast<NonlinearConstraint>(auxConstraint1)->add(tmpSignomialTerms);

        copySignomialTermsToConstraint(
            tmpSignomialTerms, std::dynamic_pointer_cast<NonlinearConstraint>(auxConstraint2), true);
    }

    if(tmpNonlinearExpression)
    {
        std::dynamic_pointer_cast<NonlinearConstraint>(auxConstraint1)
            ->add(copyNonlinearExpression(tmpNonlinearExpression.get(), reformulatedProblem));
        std::dynamic_pointer_cast<NonlinearConstraint>(auxConstraint2)
            ->add(std::make_shared<ExpressionNegate>(
                copyNonlinearExpression(tmpNonlinearExpression.get(), reformulatedProblem)));
    }

    std::dynamic_pointer_cast<LinearConstraint>(auxConstraint1)->add(std::make_shared<LinearTerm>(-1.0, auxVariable));
    std::dynamic_pointer_cast<LinearConstraint>(auxConstraint2)->add(std::make_shared<LinearTerm>(-1.0, auxVariable));

    reformulatedProblem->add(auxConstraint1);
    reformulatedProblem->add(auxConstraint2);

    return (std::make_shared<ExpressionVariable>(auxVariable));
}

NonlinearExpressionPtr TaskReformulateProblem::reformulateNonlinearExpression(std::shared_ptr<ExpressionSquare> source)
{
    // Extract all quadratic terms from inside of the nonlinear expression
    auto convexity = source->getConvexity();

    if((extractQuadraticTermsFromNonconvexExpressions
           && (convexity > E_Convexity::Convex || convexity == E_Convexity::Unknown))
        || (extractQuadraticTermsFromConvexExpressions && convexity == E_Convexity::Convex))
    {
        auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression,
            tmpConstant]
            = extractTermsAndConstant(source, false, false, true, false);

        if(tmpQuadraticTerms.size() > 0)
        {
            auto sum = std::make_shared<ExpressionSum>();

            for(auto& T : tmpQuadraticTerms)
            {
                auto [auxVariable, newVariable] = getBilinearAuxiliaryVariable(T->firstVariable, T->secondVariable);
                sum->children.push_back(std::make_shared<ExpressionVariable>(auxVariable));
            }

            if(tmpNonlinearExpression)
                sum->children.push_back(reformulateNonlinearExpression(tmpNonlinearExpression));

            return (simplify(sum));
        }
    }

    source->child = reformulateNonlinearExpression(source->child);

    return (simplify(source));
}

NonlinearExpressionPtr TaskReformulateProblem::reformulateNonlinearExpression(std::shared_ptr<ExpressionProduct> source)
{
    // Extract all quadratic terms from inside of the nonlinear expression
    auto convxity = source->getConvexity();

    if((extractQuadraticTermsFromNonconvexExpressions
           && !(convxity > E_Convexity::Convex || convxity == E_Convexity::Unknown))
        || extractQuadraticTermsFromConvexExpressions)
    {
        auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression,
            tmpConstant]
            = extractTermsAndConstant(source, false, false, true, false);

        if(tmpQuadraticTerms.size() > 0)
        {
            auto sum = std::make_shared<ExpressionSum>();

            for(auto& T : tmpQuadraticTerms)
            {
                auto [auxVariable, newVariable] = getBilinearAuxiliaryVariable(T->firstVariable, T->secondVariable);
                sum->children.push_back(std::make_shared<ExpressionVariable>(auxVariable));
            }

            if(tmpNonlinearExpression)
                sum->children.push_back(reformulateNonlinearExpression(tmpNonlinearExpression));

            return (simplify(sum));
        }
    }

    for(auto& C : source->children)
        C = reformulateNonlinearExpression(C);

    return (simplify(source));
}

std::pair<AuxiliaryVariablePtr, bool> TaskReformulateProblem::getSquareAuxiliaryVariable(
    VariablePtr variable, double coefficient, E_AuxiliaryVariableType auxVariableType)
{
    auto auxVariableIterator = squareAuxVariables.find(std::make_pair(variable, coefficient));

    if(auxVariableIterator != squareAuxVariables.end())
        return (std::make_pair(auxVariableIterator->second, false));

    // Create a new variable

    // Get the max bounds
    auto valueList = { variable->lowerBound * variable->lowerBound, variable->upperBound * variable->upperBound };

    double lowerBound = (variable->lowerBound < 0) ? 0.0 : std::min(valueList);
    double upperBound = std::max(valueList);

    E_VariableType variableType;

    if(variable->properties.type == E_VariableType::Binary)
    {
        variableType = E_VariableType::Binary;
    }
    else if(variable->properties.type == E_VariableType::Integer
        || variable->properties.type == E_VariableType::Semiinteger)
    {
        variableType = E_VariableType::Integer;
    }
    else
    {
        variableType = E_VariableType::Real;
    }

    auto auxVariable = std::make_shared<AuxiliaryVariable>(
        "s_sq_" + variable->name, auxVariableCounter, variableType, lowerBound, upperBound);

    auxVariableCounter++;
    auxVariable->properties.auxiliaryType = auxVariableType;
    env->results->increaseAuxiliaryVariableCounter(auxVariableType);

    reformulatedProblem->add((auxVariable));
    auxVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(coefficient, variable, variable));
    squareAuxVariables.emplace(std::make_pair(variable, coefficient), auxVariable);

    return (std::make_pair(auxVariable, true));
}

std::pair<AuxiliaryVariablePtr, bool> TaskReformulateProblem::getBilinearAuxiliaryVariable(
    VariablePtr firstVariable, VariablePtr secondVariable)
{
    std::tuple<VariablePtr, VariablePtr> key;

    // The variable with lower index is stored first in the tuple
    if(firstVariable->index < secondVariable->index)
        key = std::make_tuple(firstVariable, secondVariable);
    else
        key = std::make_tuple(secondVariable, firstVariable);

    auto auxVariableIterator = bilinearAuxVariables.find(key);

    if(auxVariableIterator != bilinearAuxVariables.end())
        return (std::make_pair(auxVariableIterator->second, false));

    // Create a new variable

    // Get the max bounds
    auto valueList = { firstVariable->lowerBound * secondVariable->lowerBound,
        firstVariable->lowerBound * secondVariable->upperBound, firstVariable->upperBound * secondVariable->lowerBound,
        firstVariable->upperBound * secondVariable->upperBound };

    double lowerBound = std::min(valueList);
    double upperBound = std::max(valueList);

    E_VariableType variableType;
    E_AuxiliaryVariableType auxVariableType;

    if(firstVariable->properties.type == E_VariableType::Binary
        && secondVariable->properties.type == E_VariableType::Binary)
    {
        variableType = E_VariableType::Binary;
        auxVariableType = E_AuxiliaryVariableType::BinaryBilinear;
    }
    else if(firstVariable->properties.type == E_VariableType::Integer
        && secondVariable->properties.type == E_VariableType::Integer)
    {
        variableType = E_VariableType::Integer;
        auxVariableType = E_AuxiliaryVariableType::IntegerBilinear;
    }
    else if(firstVariable->properties.type == E_VariableType::Semiinteger
        && secondVariable->properties.type == E_VariableType::Semiinteger)
    {
        variableType = E_VariableType::Integer;
        auxVariableType = E_AuxiliaryVariableType::IntegerBilinear;
    }
    else if((firstVariable->properties.type == E_VariableType::Binary
                && (secondVariable->properties.type == E_VariableType::Integer
                    || secondVariable->properties.type == E_VariableType::Semiinteger))
        || ((firstVariable->properties.type == E_VariableType::Integer
                || firstVariable->properties.type == E_VariableType::Semiinteger)
            && secondVariable->properties.type == E_VariableType::Binary))
    {
        variableType = E_VariableType::Integer;
        auxVariableType = E_AuxiliaryVariableType::IntegerBilinear;
    }
    else
    {
        variableType = E_VariableType::Real;
        auxVariableType = E_AuxiliaryVariableType::ContinuousBilinear;
    }

    auto auxVariable = std::make_shared<AuxiliaryVariable>("s_bl_" + firstVariable->name + "_" + secondVariable->name,
        auxVariableCounter, variableType, lowerBound, upperBound);
    auxVariableCounter++;
    auxVariable->properties.auxiliaryType = auxVariableType;
    env->results->increaseAuxiliaryVariableCounter(auxVariableType);

    reformulatedProblem->add((auxVariable));
    auxVariable->quadraticTerms.add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));
    bilinearAuxVariables.emplace(key, auxVariable);

    return (std::make_pair(auxVariable, true));
}

std::pair<AuxiliaryVariablePtr, bool> TaskReformulateProblem::getAbsoluteValueAuxiliaryVariable(
    std::shared_ptr<ExpressionAbs> source)
{
    std::stringstream expression;
    expression << source->child;

    auto key = expression.str();

    auto auxVariableIterator = absoluteExpressionsAuxVariables.find(key);

    if(auxVariableIterator != absoluteExpressionsAuxVariables.end())
        return (std::make_pair(auxVariableIterator->second, false));

    // Create a new variable

    // Get the max bound
    auto bounds = source->getBounds();

    auto auxVariable = std::make_shared<AuxiliaryVariable>("s_abs_" + std::to_string(auxVariableCounter + 1),
        auxVariableCounter, E_VariableType::Real, bounds.l(), bounds.u());
    auxVariable->properties.auxiliaryType = E_AuxiliaryVariableType::AbsoluteValue;
    auxVariableCounter++;
    env->results->increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType::AbsoluteValue);

    reformulatedProblem->add(auxVariable);
    auxVariable->nonlinearExpression = copyNonlinearExpression(source->child.get(), reformulatedProblem);

    absoluteExpressionsAuxVariables.emplace(key, auxVariable);

    return (std::make_pair(auxVariable, true));
}

void TaskReformulateProblem::createSquareReformulations()
{
    for(const auto& [PAIR, AUXVAR] : squareAuxVariables)
    {
        reformulateSquareTerm(PAIR.first, AUXVAR, PAIR.second);
        AUXVAR->properties.auxiliaryType = E_AuxiliaryVariableType::SquareTermsPartitioning;
    }
}

void TaskReformulateProblem::createBilinearReformulations()
{
    for(const auto& [VARS, AUXVAR] : bilinearAuxVariables)
    {
        auto firstVariable = std::get<0>(VARS);
        auto firstVariableType = firstVariable->properties.type;

        auto secondVariable = std::get<1>(VARS);
        auto secondVariableType = secondVariable->properties.type;

        if(firstVariableType == E_VariableType::Binary && secondVariableType == E_VariableType::Binary)
        {
            reformulateBinaryBilinearTerm(firstVariable, secondVariable, AUXVAR);
            AUXVAR->properties.auxiliaryType = E_AuxiliaryVariableType::BinaryBilinear;
        }
        else if((firstVariableType == E_VariableType::Binary && secondVariableType == E_VariableType::Real)
            || (firstVariableType == E_VariableType::Real && secondVariableType == E_VariableType::Binary))
        {
            reformulateBinaryContinuousBilinearTerm(firstVariable, secondVariable, AUXVAR);
            AUXVAR->properties.auxiliaryType = E_AuxiliaryVariableType::BinaryContinuousBilinear;
        }
        else if(firstVariableType == E_VariableType::Integer || firstVariableType == E_VariableType::Semiinteger
            || secondVariableType == E_VariableType::Integer || secondVariableType == E_VariableType::Semiinteger)
        {
            reformulateIntegerBilinearTerm(firstVariable, secondVariable, AUXVAR);
            AUXVAR->properties.auxiliaryType = E_AuxiliaryVariableType::IntegerBilinear;
        }
        else if(firstVariableType == E_VariableType::Real && secondVariableType == E_VariableType::Real)
        {
            reformulateRealBilinearTerm(firstVariable, secondVariable, AUXVAR);
            AUXVAR->properties.auxiliaryType = E_AuxiliaryVariableType::ContinuousBilinear;
        }
        else
        {
        }
    }
}

void TaskReformulateProblem::reformulateBinaryBilinearTerm(
    VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable)
{
    firstVariable = reformulatedProblem->getVariable(firstVariable->index);
    secondVariable = reformulatedProblem->getVariable(secondVariable->index);
    auto usedAuxVariable = reformulatedProblem->getVariable(auxVariable->index);

    auto auxConstraint = std::make_shared<LinearConstraint>(
        auxConstraintCounter, "s_binbl_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 1.0);
    auxConstraintCounter++;

    if(firstVariable == secondVariable)
    {
        auto linearTerm1 = std::make_shared<LinearTerm>(2.0, firstVariable);
        auto linearTerm2 = std::make_shared<LinearTerm>(-1.0, usedAuxVariable);

        auxConstraint->add(linearTerm1);
        auxConstraint->add(linearTerm2);
    }
    else
    {
        auto linearTerm1 = std::make_shared<LinearTerm>(1.0, firstVariable);
        auto linearTerm2 = std::make_shared<LinearTerm>(1.0, secondVariable);
        auto linearTerm3 = std::make_shared<LinearTerm>(-1.0, usedAuxVariable);

        auxConstraint->add(linearTerm1);
        auxConstraint->add(linearTerm2);
        auxConstraint->add(linearTerm3);
    }

    auto auxConstraintBound1 = std::make_shared<LinearConstraint>(
        auxConstraintCounter, "s_blbb_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
    auxConstraintBound1->add(std::make_shared<LinearTerm>(1.0, usedAuxVariable));
    auxConstraintBound1->add(std::make_shared<LinearTerm>(-1.0, firstVariable));
    auxConstraintCounter++;

    auto auxConstraintBound2 = std::make_shared<LinearConstraint>(
        auxConstraintCounter, "s_blbb_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
    auxConstraintBound2->add(std::make_shared<LinearTerm>(1.0, usedAuxVariable));
    auxConstraintBound2->add(std::make_shared<LinearTerm>(-1.0, secondVariable));
    auxConstraintCounter++;

    reformulatedProblem->add(std::move(auxConstraint));
    reformulatedProblem->add(std::move(auxConstraintBound1));
    reformulatedProblem->add(std::move(auxConstraintBound2));
}

void TaskReformulateProblem::reformulateBinaryContinuousBilinearTerm(
    VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable)
{
    firstVariable = reformulatedProblem->getVariable(firstVariable->index);
    secondVariable = reformulatedProblem->getVariable(secondVariable->index);
    auto usedAuxVariable = reformulatedProblem->getVariable(auxVariable->index);

    auto binaryVariable = (firstVariable->properties.type == E_VariableType::Binary) ? firstVariable : secondVariable;
    auto otherVariable = (firstVariable->properties.type == E_VariableType::Binary) ? secondVariable : firstVariable;

    auto auxConstraint1 = std::make_shared<LinearConstraint>(auxConstraintCounter,
        "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, otherVariable->upperBound);
    auxConstraint1->add(std::make_shared<LinearTerm>(-1.0, usedAuxVariable));
    auxConstraint1->add(std::make_shared<LinearTerm>(1.0, otherVariable));
    if(otherVariable->upperBound != 0.0)
        auxConstraint1->add(std::make_shared<LinearTerm>(otherVariable->upperBound, binaryVariable));
    auxConstraintCounter++;

    auto auxConstraint2 = std::make_shared<LinearConstraint>(auxConstraintCounter,
        "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, otherVariable->upperBound);
    auxConstraint2->add(std::make_shared<LinearTerm>(1.0, usedAuxVariable));
    auxConstraint2->add(std::make_shared<LinearTerm>(-1.0, otherVariable));
    if(otherVariable->upperBound != 0.0)
        auxConstraint2->add(std::make_shared<LinearTerm>(otherVariable->upperBound, binaryVariable));
    auxConstraintCounter++;

    auto auxConstraint3 = std::make_shared<LinearConstraint>(
        auxConstraintCounter, "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0);
    auxConstraint3->add(std::make_shared<LinearTerm>(-1.0, usedAuxVariable));
    if(otherVariable->lowerBound != 0.0)
        auxConstraint3->add(std::make_shared<LinearTerm>(otherVariable->lowerBound, binaryVariable));
    auxConstraintCounter++;

    auto auxConstraint4 = std::make_shared<LinearConstraint>(
        auxConstraintCounter, "s_blbc_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0);
    auxConstraint4->add(std::make_shared<LinearTerm>(1.0, usedAuxVariable));
    if(otherVariable->upperBound != 0.0)
        auxConstraint4->add(std::make_shared<LinearTerm>(-otherVariable->upperBound, binaryVariable));
    auxConstraintCounter++;

    reformulatedProblem->add(std::move(auxConstraint1));
    reformulatedProblem->add(std::move(auxConstraint2));
    reformulatedProblem->add(std::move(auxConstraint3));
    reformulatedProblem->add(std::move(auxConstraint4));
}

void TaskReformulateProblem::reformulateIntegerBilinearTerm(
    VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable)
{
    firstVariable = reformulatedProblem->getVariable(firstVariable->index);
    secondVariable = reformulatedProblem->getVariable(secondVariable->index);
    auto usedAuxVariable = reformulatedProblem->getVariable(auxVariable->index);

    VariablePtr discretizationVariable;
    VariablePtr nonDiscretizationVariable;
    Variables discretizationBinaries;

    bool foundFirstVariable = false;
    bool foundSecondVariable = false;
    bool firstVariableIsDiscrete = false;

    if(firstVariable->properties.type == E_VariableType::Binary
        || firstVariable->properties.type == E_VariableType::Integer
        || firstVariable->properties.type == E_VariableType::Semiinteger)
    {
        foundFirstVariable
            = (integerAuxiliaryBinaryVariables.find(firstVariable) != integerAuxiliaryBinaryVariables.end());
        firstVariableIsDiscrete = true;
    }

    if(secondVariable->properties.type == E_VariableType::Binary
        || secondVariable->properties.type == E_VariableType::Integer
        || secondVariable->properties.type == E_VariableType::Semiinteger)
    {
        foundSecondVariable
            = (integerAuxiliaryBinaryVariables.find(secondVariable) != integerAuxiliaryBinaryVariables.end());
    }

    bool firstVariableSmallerDomain = (firstVariable->upperBound - firstVariable->lowerBound
        < secondVariable->upperBound - secondVariable->lowerBound);

    if(foundFirstVariable && foundSecondVariable)
    {
        discretizationVariable = firstVariableSmallerDomain ? firstVariable : secondVariable;
        nonDiscretizationVariable = firstVariableSmallerDomain ? secondVariable : firstVariable;

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
    else // Need to create binary variables and SOS1 constraint
    {
        if(firstVariableIsDiscrete && firstVariableSmallerDomain)
        {
            discretizationVariable = firstVariable;
            nonDiscretizationVariable = secondVariable;
        }
        else
        {
            discretizationVariable = secondVariable;
            nonDiscretizationVariable = firstVariable;
        }

        auto auxFirstSum = std::make_shared<LinearConstraint>(
            auxConstraintCounter, "s_bli" + std::to_string(auxConstraintCounter), 1.0, 1.0);
        auxConstraintCounter++;

        auto auxFirstSumVarDef = std::make_shared<LinearConstraint>(
            auxConstraintCounter, "s_blx" + std::to_string(auxConstraintCounter), 0, 0);
        auxConstraintCounter++;

        auxFirstSumVarDef->add(std::make_shared<LinearTerm>(-1.0, discretizationVariable));

        for(auto i = discretizationVariable->lowerBound; i <= discretizationVariable->upperBound; i++)
        {
            auto auxBinary = std::make_shared<AuxiliaryVariable>(
                "s_bli" + std::to_string(auxVariableCounter + 1), auxVariableCounter, E_VariableType::Binary, 0.0, 1.0);

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

    double M = 2 * std::max(std::abs(discretizationVariable->lowerBound), std::abs(discretizationVariable->upperBound))
        * std::max(std::abs(nonDiscretizationVariable->lowerBound), std::abs(nonDiscretizationVariable->upperBound));

    for(auto i = discretizationVariable->lowerBound; i <= discretizationVariable->upperBound; i++)
    {
        auto auxConstraint1 = std::make_shared<LinearConstraint>(
            auxConstraintCounter, "s_blw1_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, M);

        auto auxConstraint2 = std::make_shared<LinearConstraint>(
            auxConstraintCounter, "s_blw2_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, M);

        auxConstraintCounter++;

        auxConstraint1->add(std::make_shared<LinearTerm>(-1.0, usedAuxVariable));
        auxConstraint2->add(std::make_shared<LinearTerm>(1.0, usedAuxVariable));

        auxConstraint1->add(std::make_shared<LinearTerm>(i, nonDiscretizationVariable));
        auxConstraint2->add(std::make_shared<LinearTerm>(-i, nonDiscretizationVariable));

        auxConstraint1->add(
            std::make_shared<LinearTerm>(M, discretizationBinaries[i - discretizationVariable->lowerBound]));

        auxConstraint2->add(
            std::make_shared<LinearTerm>(M, discretizationBinaries[i - discretizationVariable->lowerBound]));

        reformulatedProblem->add(std::move(auxConstraint1));
        reformulatedProblem->add(std::move(auxConstraint2));
    }
}

void TaskReformulateProblem::reformulateSquareTerm(
    VariablePtr variable, AuxiliaryVariablePtr auxVariable, double coefficient)
{
    variable = reformulatedProblem->getVariable(variable->index);
    auto usedAuxVariable = reformulatedProblem->getVariable(auxVariable->index);

    if(useConvexQuadraticConstraints)
    {
        auto auxConstraint = std::make_shared<QuadraticConstraint>(
            auxConstraintCounter, "s_sq_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
        auxConstraintCounter++;

        auxConstraint->add(std::make_shared<LinearTerm>(-1.0 / coefficient, usedAuxVariable));
        auxConstraint->add(std::make_shared<QuadraticTerm>(1.0, variable, variable));

        reformulatedProblem->add(std::move(auxConstraint));
    }
    else
    {
        auto auxConstraint = std::make_shared<NonlinearConstraint>(
            auxConstraintCounter, "s_sq_" + std::to_string(auxConstraintCounter), SHOT_DBL_MIN, 0.0);
        auxConstraintCounter++;

        auxConstraint->add(std::make_shared<LinearTerm>(-1.0 / coefficient, usedAuxVariable));
        auxConstraint->add(std::make_shared<QuadraticTerm>(1.0, variable, variable));

        reformulatedProblem->add(std::move(auxConstraint));
    }
}

void TaskReformulateProblem::reformulateRealBilinearTerm(
    VariablePtr firstVariable, VariablePtr secondVariable, AuxiliaryVariablePtr auxVariable)
{
    firstVariable = reformulatedProblem->getVariable(firstVariable->index);
    secondVariable = reformulatedProblem->getVariable(secondVariable->index);
    auto usedAuxVariable = reformulatedProblem->getVariable(auxVariable->index);

    bool isConvex = (firstVariable == secondVariable) ? true : false;

    if((useConvexQuadraticConstraints && isConvex) || useNonconvexQuadraticConstraints)
    {
        auto auxConstraint = std::make_shared<QuadraticConstraint>(
            auxConstraintCounter, "s_blcc_" + std::to_string(auxConstraintCounter), 0.0, 0.0);
        auxConstraintCounter++;

        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, usedAuxVariable));
        auxConstraint->add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));

        reformulatedProblem->add(std::move(auxConstraint));
    }
    else
    {
        auto auxConstraint = std::make_shared<NonlinearConstraint>(
            auxConstraintCounter, "s_blcc_" + std::to_string(auxConstraintCounter), 0.0, 0.0);
        auxConstraintCounter++;

        auxConstraint->add(std::make_shared<LinearTerm>(-1.0, usedAuxVariable));
        auxConstraint->add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));

        reformulatedProblem->add(std::move(auxConstraint));

        if(env->settings->getSetting<bool>("Reformulation.Bilinear.AddConvexEnvelope", "Model"))
        {
            addBilinearMcCormickEnvelope(usedAuxVariable, firstVariable, secondVariable);
        }
    }
}

void TaskReformulateProblem::addBilinearMcCormickEnvelope(
    VariablePtr auxVariable, VariablePtr firstVariable, VariablePtr secondVariable)
{
    firstVariable = reformulatedProblem->getVariable(firstVariable->index);
    secondVariable = reformulatedProblem->getVariable(secondVariable->index);
    auto usedAuxVariable = reformulatedProblem->getVariable(auxVariable->index);

    auto auxConstraintU1
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, firstVariable->lowerBound * secondVariable->lowerBound);
    auxConstraintU1->add(std::make_shared<LinearTerm>(-1.0, usedAuxVariable));
    auxConstraintU1->add(std::make_shared<LinearTerm>(firstVariable->lowerBound, secondVariable));
    auxConstraintU1->add(std::make_shared<LinearTerm>(secondVariable->lowerBound, firstVariable));
    auxConstraintCounter++;

    auto auxConstraintU2
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, firstVariable->upperBound * secondVariable->upperBound);
    auxConstraintU2->add(std::make_shared<LinearTerm>(-1.0, usedAuxVariable));
    auxConstraintU2->add(std::make_shared<LinearTerm>(firstVariable->upperBound, secondVariable));
    auxConstraintU2->add(std::make_shared<LinearTerm>(secondVariable->upperBound, firstVariable));
    auxConstraintCounter++;

    auto auxConstraintU3
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, -firstVariable->upperBound * secondVariable->lowerBound);
    auxConstraintU3->add(std::make_shared<LinearTerm>(1.0, usedAuxVariable));
    auxConstraintU3->add(std::make_shared<LinearTerm>(-firstVariable->upperBound, secondVariable));
    auxConstraintU3->add(std::make_shared<LinearTerm>(-secondVariable->lowerBound, firstVariable));
    auxConstraintCounter++;

    auto auxConstraintU4
        = std::make_shared<LinearConstraint>(auxConstraintCounter, "s_blmc_" + std::to_string(auxConstraintCounter),
            SHOT_DBL_MIN, firstVariable->lowerBound * secondVariable->upperBound);
    auxConstraintU4->add(std::make_shared<LinearTerm>(1.0, usedAuxVariable));
    auxConstraintU4->add(std::make_shared<LinearTerm>(-firstVariable->lowerBound, secondVariable));
    auxConstraintU4->add(std::make_shared<LinearTerm>(-secondVariable->upperBound, firstVariable));
    auxConstraintCounter++;

    reformulatedProblem->add(std::move(auxConstraintU1));
    reformulatedProblem->add(std::move(auxConstraintU2));
    reformulatedProblem->add(std::move(auxConstraintU3));
    reformulatedProblem->add(std::move(auxConstraintU4));
}

std::optional<QuadraticTermPtr> TaskReformulateProblem::reformulateProductToQuadraticTerm(
    std::shared_ptr<ExpressionProduct> product)
{
    auto optional = convertProductToQuadraticTerm(product);

    if(optional)
    {
        optional->get()->firstVariable = reformulatedProblem->getVariable(optional->get()->firstVariable->index);
        optional->get()->secondVariable = reformulatedProblem->getVariable(optional->get()->secondVariable->index);
        optional->get()->takeOwnership(reformulatedProblem);
    }

    return (optional);
}

std::optional<MonomialTermPtr> TaskReformulateProblem::reformulateProductToMonomialTerm(
    std::shared_ptr<ExpressionProduct> product)

{
    auto optional = convertProductToMonomialTerm(product);

    if(optional)
    {
        for(auto& VAR : optional->get()->variables)
            VAR = reformulatedProblem->getVariable(VAR->index);

        optional->get()->takeOwnership(reformulatedProblem);
    }

    return (optional);
}

} // namespace SHOT