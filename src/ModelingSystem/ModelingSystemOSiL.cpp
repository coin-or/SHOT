/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ModelingSystemOSiL.h"

#include "../Output.h"
#include "../Settings.h"
#include "../Utilities.h"

#include "../Model/Simplifications.h"

#include "tinyxml2.h"

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

ModelingSystemOSiL::ModelingSystemOSiL(EnvironmentPtr envPtr) : IModelingSystem(envPtr) {}

ModelingSystemOSiL::~ModelingSystemOSiL() = default;

void ModelingSystemOSiL::augmentSettings([[maybe_unused]] SettingsPtr settings) {}

void ModelingSystemOSiL::updateSettings([[maybe_unused]] SettingsPtr settings) {}

E_ProblemCreationStatus ModelingSystemOSiL::createProblem(ProblemPtr& problem, const std::string& filename)
{
    if(false && !fs::filesystem::exists(fs::filesystem::path(filename)))
    {
        env->output->outputError("Problem file \"" + filename + "\" does not exist.");

        return (E_ProblemCreationStatus::FileDoesNotExist);
    }

    fs::filesystem::path problemFile(filename);
    fs::filesystem::path problemPath = problemFile.parent_path();

    using namespace tinyxml2;

    XMLDocument osilDocument;

    auto result = osilDocument.LoadFile(filename.c_str());

    if(result != XML_SUCCESS)
    {
        env->output->outputError(
            fmt::format("Could not read problem from OSiL file {}:", filename), std::to_string(result));
        return (E_ProblemCreationStatus::ErrorInFile);
    }

    // Read the problem name if it exists
    if(osilDocument.FirstChildElement("osil")->FirstChildElement("instanceHeader")->FirstChildElement("name") != NULL)
    {
        problem->name = osilDocument.FirstChildElement("osil")
                            ->FirstChildElement("instanceHeader")
                            ->FirstChildElement("name")
                            ->GetText();
    }

    auto variablesNodes
        = osilDocument.FirstChildElement("osil")->FirstChildElement("instanceData")->FirstChildElement("variables");

    double minLBCont = env->settings->getSetting<double>("ContinuousVariable.MinimumLowerBound", "Model");
    double maxUBCont = env->settings->getSetting<double>("ContinuousVariable.MaximumUpperBound", "Model");
    double minLBInt = env->settings->getSetting<double>("IntegerVariable.MinimumLowerBound", "Model");
    double maxUBInt = env->settings->getSetting<double>("IntegerVariable.MaximumUpperBound", "Model");

    int variableIndex = 0;

    // Read the variables
    for(auto V = variablesNodes->FirstChildElement("var"); V != nullptr; V = V->NextSiblingElement("var"))
    {
        if(V->Attribute("name") == NULL)
            return (E_ProblemCreationStatus::ErrorInVariables);

        auto variableName = (V->Attribute("name") != NULL) ? V->Attribute("name") : "x" + std::to_string(variableIndex);

        char type = (V->Attribute("type") != NULL) ? V->Attribute("type")[0] : 'C';

        double variableLB = (V->Attribute("lb") != NULL) ? std::stod(V->Attribute("lb")) : 0.0; // By OSiL definition
        double variableUB = (V->Attribute("ub") != NULL) ? std::stod(V->Attribute("ub")) : SHOT_DBL_MAX;

        E_VariableType variableType;

        switch(type)
        {
        case 'C':
            variableType = E_VariableType::Real;

            if(variableLB < minLBCont)
                variableLB = minLBCont;

            if(variableUB > maxUBCont)
                variableUB = maxUBCont;

            break;

        case 'B':
            variableType = E_VariableType::Binary;

            if(variableLB < 0.0)
                variableLB = 0.0;

            if(variableUB > 1.0)
                variableUB = 1.0;

            break;

        case 'I':
            variableType = E_VariableType::Integer;

            if(variableLB < minLBInt)
                variableLB = minLBInt;

            if(variableUB > maxUBInt)
                variableUB = maxUBInt;

            break;

        case 'D':
            variableType = E_VariableType::Semicontinuous;

            if(variableLB < 0.0)
                variableLB = 0.0;

            if(variableUB > maxUBCont)
                variableUB = maxUBCont;

            break;

        default:
            return (E_ProblemCreationStatus::ErrorInVariables);
            break;
        }

        problem->add(
            std::make_shared<SHOT::Variable>(variableName, variableIndex, variableType, variableLB, variableUB));

        variableIndex++;
    }

    if(variableIndex == 0)
    {
        env->output->outputError(fmt::format("No variables defined."));
        return (E_ProblemCreationStatus::ErrorInVariables);
    }

    auto quadraticCoeffNodes = osilDocument.FirstChildElement("osil")
                                   ->FirstChildElement("instanceData")
                                   ->FirstChildElement("quadraticCoefficients");

    // Flag constraints (and objective) with quadratic terms, will add the terms themselves later on after objetive
    // and constraint have been created
    std::map<int, bool> containsQuadraticTerms;

    try
    {
        if(quadraticCoeffNodes != NULL)
        {
            for(auto QT = quadraticCoeffNodes->FirstChildElement("qTerm"); QT != nullptr;
                QT = QT->NextSiblingElement("qTerm"))
            {
                int constraintIndex = std::stoi(QT->Attribute("idx"));
                containsQuadraticTerms.emplace(constraintIndex, true);
            }
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when parsing quadratic terms."));
        return (E_ProblemCreationStatus::ErrorInConstraints);
    }

    auto nonlinearExprNodes = osilDocument.FirstChildElement("osil")
                                  ->FirstChildElement("instanceData")
                                  ->FirstChildElement("nonlinearExpressions");

    // Flag constraints (and objective) with nonlinear expressions, will add the expressions when creating the
    // objects
    std::map<int, NonlinearExpressionPtr> nonlinearConstraints;

    try
    {
        if(nonlinearExprNodes != NULL)
        {
            for(auto EXPR = nonlinearExprNodes->FirstChildElement("nl"); EXPR != nullptr;
                EXPR = EXPR->NextSiblingElement("nl"))
            {
                int constraintIndex = std::stoi(EXPR->Attribute("idx"));
                nonlinearConstraints.emplace(constraintIndex, convertNonlinearNode(EXPR->FirstChildElement(), problem));
            }
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when parsing nonlinear expressions."));
        return (E_ProblemCreationStatus::ErrorInConstraints);
    }

    // Read constraints
    auto constraintsNodes
        = osilDocument.FirstChildElement("osil")->FirstChildElement("instanceData")->FirstChildElement("constraints");

    int constraintCounter = 0;

    try
    {
        for(auto C = constraintsNodes->FirstChildElement("con"); C != nullptr; C = C->NextSiblingElement("con"))
        {
            double lowerBound = (C->Attribute("lb") != NULL) ? std::stod(C->Attribute("lb")) : SHOT_DBL_MIN;
            double upperBound = (C->Attribute("ub") != NULL) ? std::stod(C->Attribute("ub")) : SHOT_DBL_MAX;

            auto name
                = (C->Attribute("name") != NULL) ? C->Attribute("name") : "con" + std::to_string(constraintCounter);

            auto nonlinearExpression = nonlinearConstraints.find(constraintCounter);
            auto hasQuadraticTerms = containsQuadraticTerms.find(constraintCounter);

            if(nonlinearExpression != nonlinearConstraints.end())
                problem->add(std::make_shared<NonlinearConstraint>(
                    constraintCounter, name, nonlinearExpression->second, lowerBound, upperBound));
            else if(hasQuadraticTerms != containsQuadraticTerms.end())
                problem->add(std::make_shared<QuadraticConstraint>(constraintCounter, name, lowerBound, upperBound));
            else
                problem->add(std::make_shared<LinearConstraint>(constraintCounter, name, lowerBound, upperBound));

            constraintCounter++;
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when parsing constraints."));
        return (E_ProblemCreationStatus::ErrorInConstraints);
    }

    try
    {
        auto objectivesNodes = osilDocument.FirstChildElement("osil")
                                   ->FirstChildElement("instanceData")
                                   ->FirstChildElement("objectives");

        if(objectivesNodes->FirstChild()->NextSibling() != NULL)
        {
            // Error message here
            return (E_ProblemCreationStatus::ErrorInObjective);
        }

        auto objective = objectivesNodes->FirstChildElement("obj");

        if(objective->Attribute("maxOrMin") == NULL)
        {
            return (E_ProblemCreationStatus::ErrorInObjective);
        }

        E_ObjectiveFunctionDirection direction = (std::string("min").compare(objective->Attribute("maxOrMin")) == 0)
            ? E_ObjectiveFunctionDirection::Minimize
            : E_ObjectiveFunctionDirection::Maximize;

        double constant
            = (objective->Attribute("constant") != NULL) ? std::stod(objective->Attribute("constant")) : 0.0;

        auto nonlinearObjectiveExpression = nonlinearConstraints.find(-1);
        auto objectiveHasQuadraticTerms = containsQuadraticTerms.find(-1);

        if(nonlinearObjectiveExpression != nonlinearConstraints.end())
            problem->add(std::make_shared<NonlinearObjectiveFunction>(
                direction, nonlinearObjectiveExpression->second, constant));
        else if(objectiveHasQuadraticTerms != containsQuadraticTerms.end())
            problem->add(std::make_shared<QuadraticObjectiveFunction>(direction, constant));
        else
            problem->add(std::make_shared<LinearObjectiveFunction>(direction, constant));

        for(auto C = objective->FirstChildElement("coef"); C != nullptr; C = C->NextSiblingElement("coef"))
        {
            if(C->Attribute("idx") == NULL)
                return (E_ProblemCreationStatus::ErrorInObjective);

            double coefficient = std::stod(C->GetText());
            int index = std::stoi(C->Attribute("idx"));

            std::dynamic_pointer_cast<LinearObjectiveFunction>(problem->objectiveFunction)
                ->add(std::make_shared<LinearTerm>(coefficient, problem->allVariables[index]));
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when parsing objective function."));
        return (E_ProblemCreationStatus::ErrorInObjective);
    }
    try
    {
        if(quadraticCoeffNodes != NULL)
        {
            for(auto QT = quadraticCoeffNodes->FirstChildElement("qTerm"); QT != nullptr;
                QT = QT->NextSiblingElement("qTerm"))
            {
                int placementIndex = std::stoi(QT->Attribute("idx"));
                double coefficient = (QT->Attribute("coef") != NULL) ? std::stod(QT->Attribute("coef")) : 1.0;
                int firstVariableIndex = std::stoi(QT->Attribute("idxOne"));
                int secondVariableIndex = std::stoi(QT->Attribute("idxTwo"));

                if(placementIndex == -1)
                {
                    std::dynamic_pointer_cast<QuadraticObjectiveFunction>(problem->objectiveFunction)
                        ->add(std::make_shared<QuadraticTerm>(coefficient, problem->allVariables[firstVariableIndex],
                            problem->allVariables[secondVariableIndex]));
                }
                else
                {
                    std::dynamic_pointer_cast<QuadraticConstraint>(problem->numericConstraints[placementIndex])
                        ->add(std::make_shared<QuadraticTerm>(coefficient, problem->allVariables[firstVariableIndex],
                            problem->allVariables[secondVariableIndex]));
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when parsing quadratic terms."));
        return (E_ProblemCreationStatus::ErrorInConstraints);
    }

    try
    {
        auto linConCoeffNodes = osilDocument.FirstChildElement("osil")
                                    ->FirstChildElement("instanceData")
                                    ->FirstChildElement("linearConstraintCoefficients");

        int numberOfCoeffs = std::stoi(linConCoeffNodes->Attribute("numberOfValues"));

        if(numberOfCoeffs > 0)
        {
            VectorInteger startIndices;
            VectorInteger indices(numberOfCoeffs);
            VectorDouble coefficients(numberOfCoeffs);

            bool isRowFormat = (linConCoeffNodes->FirstChildElement("colIdx") != NULL) ? true : false;

            for(auto E = linConCoeffNodes->FirstChildElement("start")->FirstChildElement("el"); E != nullptr;
                E = E->NextSiblingElement("el"))
            {
                int mult = (E->Attribute("mult") != NULL) ? std::stoi(E->Attribute("mult")) : 1;
                int incr = (E->Attribute("incr") != NULL) ? std::stoi(E->Attribute("incr")) : 0;
                int value = std::stoi(E->GetText());

                for(int i = 0; i < mult; i++)
                    startIndices.push_back(value + i * incr);
            }

            std::string rowOrColIdx = isRowFormat ? "colIdx" : "rowIdx";

            int counter = 0;

            for(auto E = linConCoeffNodes->FirstChildElement(rowOrColIdx.c_str())->FirstChildElement("el");
                E != nullptr; E = E->NextSiblingElement("el"))
            {
                int mult = (E->Attribute("mult") != NULL) ? std::stoi(E->Attribute("mult")) : 1;
                int incr = (E->Attribute("incr") != NULL) ? std::stoi(E->Attribute("incr")) : 0;
                int value = std::stoi(E->GetText());

                for(int i = 0; i < mult; i++)
                {
                    indices[counter] = value + i * incr;
                    counter++;
                }
            }

            counter = 0;

            for(auto E = linConCoeffNodes->FirstChildElement("value")->FirstChildElement("el"); E != nullptr;
                E = E->NextSiblingElement("el"))
            {
                int mult = (E->Attribute("mult") != NULL) ? std::stoi(E->Attribute("mult")) : 1;
                int incr = (E->Attribute("incr") != NULL) ? std::stoi(E->Attribute("incr")) : 0;
                double value = std::stod(E->GetText());

                for(int i = 0; i < mult; i++)
                {
                    coefficients[counter] = value + i * incr;
                    counter++;
                }
            }

            counter = 0;

            if(isRowFormat)
            {
                for(size_t i = 0; i < problem->numericConstraints.size(); i++)
                {
                    while(counter < startIndices[i + 1])
                    {
                        std::dynamic_pointer_cast<LinearConstraint>(problem->numericConstraints[i])
                            ->add(std::make_shared<LinearTerm>(
                                coefficients[counter], problem->allVariables[indices[counter]]));
                        counter++;
                    }
                }
            }
            else
            {
                for(size_t i = 0; i < problem->allVariables.size(); i++)
                {
                    while(counter < startIndices[i + 1])
                    {
                        std::dynamic_pointer_cast<LinearConstraint>(problem->numericConstraints[indices[counter]])
                            ->add(std::make_shared<LinearTerm>(coefficients[counter], problem->allVariables[i]));
                        counter++;
                    }
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        env->output->outputError(fmt::format("Error when parsing linear terms in constraints."));
        return (E_ProblemCreationStatus::ErrorInConstraints);
    }

    problem->updateProperties();

    bool extractMonomialTerms = env->settings->getSetting<bool>("Reformulation.Monomials.Extract", "Model");
    bool extractSignomialTerms = env->settings->getSetting<bool>("Reformulation.Signomials.Extract", "Model");
    bool extractQuadraticTerms = (env->settings->getSetting<int>("Reformulation.Quadratics.ExtractStrategy", "Model")
        >= static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame));

    simplifyNonlinearExpressions(problem, extractMonomialTerms, extractSignomialTerms, extractQuadraticTerms);

    problem->finalize();

    return (E_ProblemCreationStatus::NormalCompletion);
}

NonlinearExpressionPtr ModelingSystemOSiL::convertNonlinearNode(tinyxml2::XMLNode* node, const ProblemPtr& destination)
{
    std::string expressionType = node->ToElement()->Name();

    if(expressionType.compare("plus") == 0)
    {
        auto firstChildNode = node->FirstChild();
        auto secondChildNode = firstChildNode->NextSibling();

        return std::make_shared<ExpressionSum>(
            convertNonlinearNode(firstChildNode, destination), convertNonlinearNode(secondChildNode, destination));
    }
    else if(expressionType.compare("sum") == 0)
    {
        NonlinearExpressions terms;

        for(auto C = node->FirstChildElement(); C != nullptr; C = C->NextSiblingElement())
            terms.push_back(convertNonlinearNode(C, destination));

        switch(terms.size())
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return terms[1];
        default:
            return std::make_shared<ExpressionSum>(terms);
        }
    }
    else if(expressionType.compare("minus") == 0)
    {
        auto firstChildNode = node->FirstChild();
        auto secondChildNode = firstChildNode->NextSibling();

        return std::make_shared<ExpressionSum>(convertNonlinearNode(firstChildNode, destination),
            std::make_shared<ExpressionNegate>(convertNonlinearNode(secondChildNode, destination)));
    }
    else if(expressionType.compare("negate") == 0)
    {
        auto firstChildNode = node->FirstChild();

        return std::make_shared<ExpressionNegate>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("times") == 0)
    {
        auto firstChildNode = node->FirstChild();
        auto secondChildNode = firstChildNode->NextSibling();

        return std::make_shared<ExpressionProduct>(
            convertNonlinearNode(firstChildNode, destination), convertNonlinearNode(secondChildNode, destination));
    }
    else if(expressionType.compare("divide") == 0)
    {
        auto firstChildNode = node->FirstChild();
        auto secondChildNode = firstChildNode->NextSibling();

        return std::make_shared<ExpressionDivide>(
            convertNonlinearNode(firstChildNode, destination), convertNonlinearNode(secondChildNode, destination));
    }
    else if(expressionType.compare("power") == 0)
    {
        auto firstChildNode = node->FirstChild();
        auto secondChildNode = firstChildNode->NextSibling();

        return std::make_shared<ExpressionPower>(
            convertNonlinearNode(firstChildNode, destination), convertNonlinearNode(secondChildNode, destination));
    }
    else if(expressionType.compare("product") == 0)
    {
        NonlinearExpressions factors;

        for(auto C = node->FirstChildElement(); C != nullptr; C = C->NextSiblingElement())
            factors.push_back(convertNonlinearNode(C, destination));

        switch(factors.size())
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return factors[1];
        default:
            return std::make_shared<ExpressionProduct>(factors);
        }
    }
    else if(expressionType.compare("abs") == 0)
    {
        auto firstChildNode = node->FirstChild();

        return std::make_shared<ExpressionAbs>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("square") == 0)
    {
        auto firstChildNode = node->FirstChild();
        return std::make_shared<ExpressionSquare>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("sqrt") == 0)
    {
        auto firstChildNode = node->FirstChild();
        return std::make_shared<ExpressionSquareRoot>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("ln") == 0)
    {
        auto firstChildNode = node->FirstChild();
        return std::make_shared<ExpressionLog>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("exp") == 0)
    {
        auto firstChildNode = node->FirstChild();
        return std::make_shared<ExpressionExp>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("sin") == 0)
    {
        auto firstChildNode = node->FirstChild();
        return std::make_shared<ExpressionSin>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("cos") == 0)
    {
        auto firstChildNode = node->FirstChild();
        return std::make_shared<ExpressionCos>(convertNonlinearNode(firstChildNode, destination));
    }
    else if(expressionType.compare("number") == 0)
    {
        return std::make_shared<ExpressionConstant>(std::stod(node->ToElement()->Attribute("value")));
    }
    else if(expressionType.compare("pi") == 0)
    {
        return std::make_shared<ExpressionConstant>(3.14159265);
    }
    else if(expressionType.compare("variable") == 0)
    {
        double coefficient
            = (node->ToElement()->Attribute("coef") != NULL) ? std::stod(node->ToElement()->Attribute("coef")) : 1.0;

        int variableIndex = std::stoi(node->ToElement()->Attribute("idx"));

        if(coefficient == 0.)
            return std::make_shared<ExpressionConstant>(0.);
        if(coefficient == 1.)
            return std::make_shared<ExpressionVariable>(destination->getVariable(variableIndex));
        if(coefficient == -1.)
            return std::make_shared<ExpressionNegate>(
                std::make_shared<ExpressionVariable>(destination->getVariable(variableIndex)));

        return std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(coefficient),
            std::make_shared<ExpressionVariable>(destination->getVariable(variableIndex)));
    }
    else
    {
        throw OperationNotImplementedException(fmt::format("Error: Unsupported OSiL function {}", expressionType));
    }

    return nullptr;
}

void ModelingSystemOSiL::finalizeSolution() {}

} // Namespace SHOT