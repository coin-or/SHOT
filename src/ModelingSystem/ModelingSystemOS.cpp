/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ModelingSystemOS.h"

#include "../Output.h"
#include "../Settings.h"
#include "../Utilities.h"

#include "../Model/Simplifications.h"

#include "OSOption.h"
#include "OSResult.h"

#include "OSiLReader.h"
#include "OSInstance.h"
#include "OSnl2OS.h"
#include "OSoLReader.h"
#include "OSrLWriter.h"

#include "CoinHelperFunctions.hpp" // for CoinCopyOfArrayOrZero, maybe should eliminate this
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "CoinFinite.hpp"

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

namespace SHOT
{

ModelingSystemOS::ModelingSystemOS(EnvironmentPtr envPtr) : IModelingSystem(envPtr) { }

ModelingSystemOS::~ModelingSystemOS() = default;

void ModelingSystemOS::augmentSettings([[maybe_unused]] SettingsPtr settings) { }

void ModelingSystemOS::updateSettings([[maybe_unused]] SettingsPtr settings) { }

E_ProblemCreationStatus ModelingSystemOS::createProblem(
    ProblemPtr& problem, const std::string& filename, const E_OSInputFileFormat& type)
{
    if(false && !fs::filesystem::exists(fs::filesystem::path(filename)))
    {
        env->output->outputError("Problem file \"" + filename + "\" does not exist.");

        return (E_ProblemCreationStatus::FileDoesNotExist);
    }

    fs::filesystem::path problemFile(filename);
    fs::filesystem::path problemPath = problemFile.parent_path();

    std::shared_ptr<OSInstance> instance;

    try
    {
        if(type == E_OSInputFileFormat::OSiL)
        {
            instance = std::shared_ptr<OSInstance>(readInstanceFromOSiLFile(filename));

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::OSiL));
        }
        else if(type == E_OSInputFileFormat::Ampl)
        {
            instance = std::shared_ptr<OSInstance>(readInstanceFromAmplFile(filename));

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::NL));
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when reading OS model from \"{}\": {}", filename, e.what()));

        return (E_ProblemCreationStatus::Error);
    }

    E_ProblemCreationStatus status = createProblem(problem, instance);

    return (status);
}

E_ProblemCreationStatus ModelingSystemOS::createProblem(ProblemPtr& problem, std::shared_ptr<OSInstance> instance)
{
    originalInstance = instance;

    try
    {
        problem->name = instance->getInstanceName();

        if(!copyVariables(instance.get(), problem))
            return (E_ProblemCreationStatus::ErrorInVariables);

        if(!copyObjectiveFunction(instance.get(), problem))
            return (E_ProblemCreationStatus::ErrorInObjective);

        if(!copyConstraints(instance.get(), problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if(!copyLinearTerms(instance.get(), problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if(!copyQuadraticTerms(instance.get(), problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        if(!copyNonlinearExpressions(instance.get(), problem))
            return (E_ProblemCreationStatus::ErrorInConstraints);

        problem->updateProperties();

        bool extractMonomialTerms = env->settings->getSetting<bool>("Reformulation.Monomials.Extract", "Model");
        bool extractSignomialTerms = env->settings->getSetting<bool>("Reformulation.Signomials.Extract", "Model");
        bool extractQuadraticTerms
            = (env->settings->getSetting<int>("Reformulation.Quadratics.ExtractStrategy", "Model")
                >= static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame));

        simplifyNonlinearExpressions(problem, extractMonomialTerms, extractSignomialTerms, extractQuadraticTerms);

        problem->finalize();
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when creating problem from OSInstance object.");

        return (E_ProblemCreationStatus::Error);
    }

    return (E_ProblemCreationStatus::NormalCompletion);
}

void ModelingSystemOS::finalizeSolution() { }

OSInstance* ModelingSystemOS::readInstanceFromOSiL(const std::string& text)
{
    auto osilReader = std::make_shared<OSiLReader>();
    OSInstance* instance = osilReader->readOSiL(text);

    osilReaders.push_back(osilReader); // To be able to properly deleting them without destroying the OSInstance object

    return instance;
}

OSInstance* ModelingSystemOS::readInstanceFromOSiLFile(const std::string& filename)
{
    std::string fileContents = Utilities::getFileAsString(filename);

    return (readInstanceFromOSiL(fileContents));
}

OSInstance* ModelingSystemOS::readInstanceFromAmplFile(const std::string& filename)
{
    nl2os = std::make_shared<OSnl2OS>();

    try
    {
        if(nl2os->readNl(filename))
        {
            nl2os->createOSObjects();
            OSInstance* instance = nl2os->osinstance;

            return instance;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return (nullptr);
}

bool ModelingSystemOS::copyVariables(OSInstance* source, ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy variables between OSInstance and SHOT problem objects.");

    if(source->instanceData->variables != nullptr && source->instanceData->variables->numberOfVariables > 0)
    {
        double minLBCont = env->settings->getSetting<double>("Variables.Continuous.MinimumLowerBound", "Model");
        double maxUBCont = env->settings->getSetting<double>("Variables.Continuous.MaximumUpperBound", "Model");
        double minLBInt = env->settings->getSetting<double>("Variables.Integer.MinimumLowerBound", "Model");
        double maxUBInt = env->settings->getSetting<double>("Variables.Integer.MaximumUpperBound", "Model");

        int numVariables = source->getVariableNumber();

        for(int i = 0; i < numVariables; i++)
        {
            E_VariableType variableType;

            double variableLB = source->instanceData->variables->var[i]->lb;
            double variableUB = source->instanceData->variables->var[i]->ub;

            switch(source->instanceData->variables->var[i]->type)
            {
            case 'C':
                variableType = E_VariableType::Real;

                if(variableLB < minLBCont)
                {
                    // env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableLBs[i]) + " to " + std::to_string(minLBCont));
                    variableLB = minLBCont;
                }

                if(variableUB > maxUBCont)
                {
                    // env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableUBs[i]) + " to " + std::to_string(maxUBCont));
                    variableUB = maxUBCont;
                }

                break;

            case 'B':
                variableType = E_VariableType::Binary;

                if(variableLB < 0.0)
                {
                    // env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableLBs[i]) + " to " + std::to_string(0.0));
                    variableLB = 0.0;
                }

                if(variableUB > 1.0)
                {
                    // env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableUBs[i]) + " to " + std::to_string(1.0));
                    variableUB = 1.0;
                }

                break;

            case 'I':
                variableType = E_VariableType::Integer;

                if(variableLB < minLBInt)
                {
                    // env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableLBs[i]) + " to " + std::to_string(minLBInt));
                    variableLB = minLBInt;
                }

                if(variableUB > maxUBInt)
                {
                    // env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableUBs[i]) + " to " + std::to_string(maxUBInt));
                    variableUB = maxUBInt;
                }

                break;

            case 'D':
                variableType = E_VariableType::Semicontinuous;

                if(variableLB < minLBCont)
                {
                    // env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableLBs[i]) + " to " + std::to_string(minLBCont));
                    variableLB = minLBCont;
                }

                if(variableUB > maxUBCont)
                {
                    // env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableUBs[i]) + " to " + std::to_string(maxUBCont));
                    variableUB = maxUBCont;
                }

                break;

            case 'J':
                variableType = E_VariableType::Semiinteger;

                if(variableLB < minLBInt)
                {
                    // env->output->outputDebug("Corrected lower bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableLBs[i]) + " to " + std::to_string(minLBInt));
                    variableLB = minLBInt;
                }

                if(variableUB > maxUBInt)
                {
                    // env->output->outputDebug("Corrected upper bound for variable " + variableNames[i] + " from " +
                    // std::to_string(variableUBs[i]) + " to " + std::to_string(maxUBInt));
                    variableUB = maxUBInt;
                }

                break;

            default:
                return (false);
                break;
            }

            auto variable = std::make_shared<SHOT::Variable>(
                source->instanceData->variables->var[i]->name, i, variableType, variableLB, variableUB);
            destination->add(variable);
        }
    }
    else
    {
        env->output->outputError("Problem has no variables.");
        return (false);
    }

    env->output->outputDebug("Finished copying variables between OSInstance and SHOT problem objects.");

    return (true);
}

bool ModelingSystemOS::copyObjectiveFunction(OSInstance* source, ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy objective function between OSInstance and SHOT problem objects.");

    ObjectiveFunctionPtr objectiveFunction;

    if(source->instanceData->objectives != nullptr)
    {
        std::string objectiveDirection = source->getObjectiveMaxOrMins()[0];

        if(isObjectiveGenerallyNonlinear(source))
        {
            objectiveFunction = std::make_shared<NonlinearObjectiveFunction>();
        }
        else if(isObjectiveQuadratic(source))
        {
            objectiveFunction = std::make_shared<QuadraticObjectiveFunction>();
        }
        else
        {
            objectiveFunction = std::make_shared<LinearObjectiveFunction>();
        }

        if(objectiveDirection == "min")
        {
            objectiveFunction->direction = E_ObjectiveFunctionDirection::Minimize;
            objectiveFunction->constant = source->getObjectiveConstants()[0];
        }
        else
        {
            objectiveFunction->direction = E_ObjectiveFunctionDirection::Maximize;
            objectiveFunction->constant = -source->getObjectiveConstants()[0];
        }

        // Now copying the linear terms (if any)

        int numberLinearTerms = source->instanceData->objectives->obj[0]->numberOfObjCoef;

        for(int i = 0; i < numberLinearTerms; i++)
        {
            double coefficient = source->instanceData->objectives->obj[0]->coef[i]->value;
            int variableIndex = source->instanceData->objectives->obj[0]->coef[i]->idx;

            try
            {
                VariablePtr variable = destination->getVariable(variableIndex);
                (std::static_pointer_cast<LinearObjectiveFunction>(objectiveFunction))
                    ->add(std::move(std::make_shared<LinearTerm>(coefficient, variable)));
            }
            catch(const VariableNotFoundException& e)
            {
                return (false);
            }
        }

        destination->add(objectiveFunction);
    }
    else
    {
        env->output->outputError("OSInstance object does not have an objective function.");
        return (false);
    }

    env->output->outputDebug("Finished copying objective function between OSInstance and SHOT problem objects.");

    return (true);
}

bool ModelingSystemOS::copyConstraints(OSInstance* source, ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy constraints between OSInstance and SHOT problem objects.");

    if(source->instanceData->constraints != nullptr)
    {
        int numberOfConstraints = source->getConstraintNumber();
        auto classification = getConstraintClassifications(source);

        for(int i = 0; i < numberOfConstraints; i++)
        {
            switch(classification[i])
            {
            case(E_ConstraintClassification::Linear):
            {
                LinearConstraintPtr constraint
                    = std::make_shared<LinearConstraint>(i, source->instanceData->constraints->con[i]->name,
                        source->instanceData->constraints->con[i]->lb, source->instanceData->constraints->con[i]->ub);
                constraint->constant = source->instanceData->constraints->con[i]->constant;
                destination->add(std::move(constraint));
                break;
            }
            case(E_ConstraintClassification::Quadratic):
            {
                QuadraticConstraintPtr constraint
                    = std::make_shared<QuadraticConstraint>(i, source->instanceData->constraints->con[i]->name,
                        source->instanceData->constraints->con[i]->lb, source->instanceData->constraints->con[i]->ub);
                constraint->constant = source->instanceData->constraints->con[i]->constant;
                destination->add(std::move(constraint));
                break;
            }
            case(E_ConstraintClassification::Nonlinear):
            {
                NonlinearConstraintPtr constraint
                    = std::make_shared<NonlinearConstraint>(i, source->instanceData->constraints->con[i]->name,
                        source->instanceData->constraints->con[i]->lb, source->instanceData->constraints->con[i]->ub);
                constraint->constant = source->instanceData->constraints->con[i]->constant;
                destination->add(std::move(constraint));
                break;
            }
            default:
                env->output->outputDebug("Constraint index" + std::to_string(i) + "is of unknown type.");
                return (false);
            }
        }
    }
    else
    {
        env->output->outputDebug("OSInstance object does not have any constraints.");
    }

    env->output->outputDebug("Finished copying constraints between OSInstance and SHOT problem objects.");

    return (true);
}

bool ModelingSystemOS::copyLinearTerms(OSInstance* source, ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy linear terms between OSInstance and SHOT problem objects.");

    if(source->instanceData->linearConstraintCoefficients != nullptr
        && source->instanceData->linearConstraintCoefficients->numberOfValues > 0)
    {
        int variableIndex = 0;
        int numConstraints = source->getConstraintNumber();

        SparseMatrix* linearConstraintCoefficients = source->getLinearConstraintCoefficientsInRowMajor();

        for(int constraintIndex = 0; constraintIndex < numConstraints; constraintIndex++)
        {
            int numConstraintElements = linearConstraintCoefficients->starts[constraintIndex + 1]
                - linearConstraintCoefficients->starts[constraintIndex];

            try
            {
                LinearConstraintPtr constraint
                    = std::static_pointer_cast<LinearConstraint>(destination->getConstraint(constraintIndex));

                for(int j = 0; j < numConstraintElements; j++)
                {
                    double coefficient = linearConstraintCoefficients
                                             ->values[linearConstraintCoefficients->starts[constraintIndex] + j];
                    variableIndex = linearConstraintCoefficients
                                        ->indexes[linearConstraintCoefficients->starts[constraintIndex] + j];

                    constraint->add(std::make_shared<LinearTerm>(coefficient, destination->getVariable(variableIndex)));
                }
            }
            catch(const VariableNotFoundException& e)
            {
                return (false);
            }
            catch(const ConstraintNotFoundException& e)
            {
                return (false);
            }
        }
    }
    else
    {
        env->output->outputDebug("OSInstance object does not have any linear terms.");
    }

    env->output->outputDebug("Finished copying linear terms between OSInstance and SHOT problem objects.");

    return (true);
}

bool ModelingSystemOS::copyQuadraticTerms(OSInstance* source, ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy quadratic terms between OSInstance and SHOT problem objects.");

    if(source->instanceData->quadraticCoefficients != nullptr)
    {
        int numQuadraticTerms = source->getNumberOfQuadraticTerms();

        for(int i = 0; i < numQuadraticTerms; i++)
        {
            auto term = source->instanceData->quadraticCoefficients->qTerm[i];

            try
            {
                VariablePtr firstVariable = destination->getVariable(term->idxOne);
                VariablePtr secondVariable = destination->getVariable(term->idxTwo);

                if(term->idx == -1)
                {
                    (std::static_pointer_cast<QuadraticObjectiveFunction>(destination->objectiveFunction))
                        ->add(std::make_shared<QuadraticTerm>(term->coef, firstVariable, secondVariable));
                }
                else
                {
                    auto constraint
                        = std::static_pointer_cast<QuadraticConstraint>(destination->getConstraint(term->idx));
                    constraint->add(std::make_shared<QuadraticTerm>(term->coef, firstVariable, secondVariable));
                }
            }
            catch(const VariableNotFoundException& e)
            {
                return (false);
            }
            catch(const ConstraintNotFoundException& e)
            {
                return (false);
            }
        }
    }
    else
    {
        env->output->outputDebug("OSInstance object does not have any quadratic terms.");
    }

    env->output->outputDebug("Finished copying quadratic terms between OSInstance and SHOT problem objects.");

    return (true);
}

bool ModelingSystemOS::copyNonlinearExpressions(OSInstance* source, ProblemPtr destination)
{
    env->output->outputDebug("Starting to copy nonlinear expressions between OSInstance and SHOT problem objects.");

    if(source->instanceData->nonlinearExpressions != nullptr)
    {
        int numNonlinearExpressions = source->getNumberOfNonlinearExpressions();

        for(int i = 0; i < numNonlinearExpressions; i++)
        {
            auto sourceNode = source->instanceData->nonlinearExpressions->nl[i];

            try
            {
                NonlinearExpressionPtr destinationExpression
                    = convertOSNonlinearNode(sourceNode->osExpressionTree->m_treeRoot, destination);

                if(sourceNode->idx == -1)
                {
                    auto objective
                        = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(destination->objectiveFunction);
                    objective->add(std::move(destinationExpression));
                }
                else
                {
                    auto constraint
                        = std::dynamic_pointer_cast<NonlinearConstraint>(destination->getConstraint(sourceNode->idx));
                    constraint->add(std::move(destinationExpression));
                }
            }
            catch(const ConstraintNotFoundException& e)
            {
                return (false);
            }
            catch(const OperationNotImplementedException& e)
            {
                return (false);
            }
        }
    }
    else
    {
        env->output->outputDebug("OSInstance object does not have any nonlinear expressions.");
    }

    env->output->outputDebug("Finished copying nonlinear expressions between OSInstance and SHOT problem objects.");

    return (true);
}

// Modified from Optimization Services file OSCouenneSolver.cpp
NonlinearExpressionPtr ModelingSystemOS::convertOSNonlinearNode(OSnLNode* node, const ProblemPtr& destination)
{
    unsigned int i;
    std::ostringstream outStr;

    switch(node->inodeInt)
    {
    case OS_PLUS:
        return std::make_shared<ExpressionSum>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination),
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[1]), destination));

    case OS_SUM:
        switch(node->inumberOfChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination);
        default:
            NonlinearExpressions terms;
            for(i = 0; i < node->inumberOfChildren; i++)
                terms.push_back(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[i]), destination));
            return std::make_shared<ExpressionSum>(terms);
        }

    case OS_MINUS:
        return std::make_shared<ExpressionSum>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination),
            std::make_shared<ExpressionNegate>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[1]), destination)));

    case OS_NEGATE:
        return std::make_shared<ExpressionNegate>(
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_TIMES:
        return std::make_shared<ExpressionProduct>(
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination),
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[1]), destination));

    case OS_DIVIDE:
        return std::make_shared<ExpressionDivide>(
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination),
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[1]), destination));

    case OS_POWER:
        return std::make_shared<ExpressionPower>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination),
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[1]), destination));

    case OS_PRODUCT:
        switch(node->inumberOfChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination);
        case 2:
            return std::make_shared<ExpressionProduct>(
                convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination),
                convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[1]), destination));
        default:
            NonlinearExpressions factors;
            for(i = 0; i < node->inumberOfChildren; i++)
                factors.push_back(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[i]), destination));
            return std::make_shared<ExpressionProduct>(factors);
        }

    case OS_ABS:
        return std::make_shared<ExpressionAbs>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_SQUARE:
        return std::make_shared<ExpressionSquare>(
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_SQRT:
        return std::make_shared<ExpressionSquareRoot>(
            convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_LN:
        return std::make_shared<ExpressionLog>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_EXP:
        return std::make_shared<ExpressionExp>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_SIN:
        return std::make_shared<ExpressionSin>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_COS:
        return std::make_shared<ExpressionCos>(convertOSNonlinearNode(((OSnLNode*)node->m_mChildren[0]), destination));

    case OS_MIN:
        throw OperationNotImplementedException("Error: Unsupported GAMS function min");
        break;

    case OS_MAX:
        throw OperationNotImplementedException("Error: Unsupported GAMS function max");
        break;

    case OS_NUMBER:
        return std::make_shared<ExpressionConstant>(((OSnLNodeNumber*)node)->value);

    case OS_PI:
        return std::make_shared<ExpressionConstant>(3.14159265);

    case OS_VARIABLE:
    {
        auto* varnode = (OSnLNodeVariable*)node;
        if(varnode->coef == 0.)
            return std::make_shared<ExpressionConstant>(0.);
        if(varnode->coef == 1.)
            return std::make_shared<ExpressionVariable>(destination->getVariable(varnode->idx));
        if(varnode->coef == -1.)
            return std::make_shared<ExpressionNegate>(
                std::make_shared<ExpressionVariable>(destination->getVariable(varnode->idx)));

        return std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(varnode->coef),
            std::make_shared<ExpressionVariable>(destination->getVariable(varnode->idx)));
    }
    default:
        throw OperationNotImplementedException(
            fmt::format("Error: Unsupported GAMS function {}", node->getTokenName()));
        break;
    }

    return nullptr;
}

bool ModelingSystemOS::isObjectiveGenerallyNonlinear(OSInstance* instance)
{
    for(int i = 0; i < instance->getNumberOfNonlinearExpressions(); i++)
    {
        int tmpIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;
        if(tmpIndex == -1)
            return (true);
    }
    return (false);
}

bool ModelingSystemOS::isObjectiveQuadratic(OSInstance* instance)
{
    for(int i = 0; i < instance->getNumberOfQuadraticTerms(); i++)
    {
        int tmpIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

        if(tmpIndex == -1)
            return (true);
    }

    return (false);
}

std::vector<E_ConstraintClassification> ModelingSystemOS::getConstraintClassifications(OSInstance* instance)
{
    int numConstraints = instance->getConstraintNumber();
    std::vector<E_ConstraintClassification> classifications(numConstraints, E_ConstraintClassification::Linear);

    int numQuadraticTerms = instance->getNumberOfQuadraticTerms();

    for(int i = 0; i < numQuadraticTerms; i++)
    {
        int constraintIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

        if(constraintIndex >= 0)
            classifications[constraintIndex] = E_ConstraintClassification::Quadratic;
    }

    int numNonlinearExpressions = instance->getNumberOfNonlinearExpressions();

    for(int i = 0; i < numNonlinearExpressions; i++)
    {
        int constraintIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;

        if(constraintIndex >= 0)
            classifications[constraintIndex] = E_ConstraintClassification::Nonlinear;
    }

    return classifications;
}

} // Namespace SHOT