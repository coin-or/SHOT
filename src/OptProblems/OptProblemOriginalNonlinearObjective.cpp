/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "OptProblemOriginalNonlinearObjective.h"

OptProblemOriginalNonlinearObjective::OptProblemOriginalNonlinearObjective()
{
}

OptProblemOriginalNonlinearObjective::~OptProblemOriginalNonlinearObjective()
{
}

bool OptProblemOriginalNonlinearObjective::setProblem(OSInstance *instance)
{
    if (UtilityFunctions::isObjectiveQuadratic(instance))
    {
        this->setObjectiveFunctionType(E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear);
    }
    else
    {
        this->setObjectiveFunctionType(E_ObjectiveFunctionType::Nonlinear);
    }

    this->setProblemInstance(instance);

    this->setTypeOfObjectiveMinimize(instance->instanceData->objectives->obj[0]->maxOrMin == "min");

    this->setObjectiveFunctionNonlinear(isConstraintNonlinear(-1));

    ProcessInfo::getInstance().setOriginalProblem(this);

    this->setNonlinearConstraintIndexes();

    if (this->getNonlinearConstraintIndexes().size() == 0)
    {
        Settings::getInstance().updateSetting("Relaxation.IterationLimit", "Dual", 0);
        Settings::getInstance().updateSetting("MIP.SolutionLimit.Initial", "Dual", 1000);
    }

    this->addedConstraintName = "objconstr";
    this->setNonlinearObjectiveConstraintIdx(getProblemInstance()->getConstraintNumber());

    this->addedObjectiveVariableName = "addobjvar";
    this->addedObjectiveVariableLowerBound = -Settings::getInstance().getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");
    this->addedObjectiveVariableUpperBound = Settings::getInstance().getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");

    this->setNonlinearObjectiveVariableIdx(getProblemInstance()->getVariableNumber());

    auto tmpBoolVector = std::vector<bool>(getProblemInstance()->getVariableNumber() + 1, false);
    this->setVariableBoundsTightened(tmpBoolVector);
    tmpBoolVector.clear();

    this->repairNonboundedVariables();
    return true;
}

double OptProblemOriginalNonlinearObjective::calculateConstraintFunctionValue(int idx, std::vector<double> point)
{
    double tmpVal = 0.0;

    if (idx != -1 && idx != this->getNonlinearObjectiveConstraintIdx()) // Not the objective function
    {
        tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
        ProcessInfo::getInstance().solutionStatistics.numberOfFunctionEvalutions++;

        if (getProblemInstance()->getConstraintTypes()[idx] == 'L')
        {
            tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->ub;
        }
        else if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
        {
            tmpVal = -tmpVal + getProblemInstance()->instanceData->constraints->con[idx]->lb;
        }
        else if (getProblemInstance()->getConstraintTypes()[idx] == 'E')
        {
            tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->lb;
        }
        else
        {
            Output::getInstance().outputWarning(
                "Constraint with index " + to_string(idx) + " of type " + to_string(getProblemInstance()->getConstraintTypes()[idx]) + " is not supported!");
        }
    }
    else // The nonlinear objective function constraint
    {
        tmpVal = getProblemInstance()->calculateFunctionValue(-1, &point.at(0), true);
        ProcessInfo::getInstance().solutionStatistics.numberOfFunctionEvalutions++;

        tmpVal = tmpVal - point.at(this->getNonlinearObjectiveVariableIdx());
    }

    return tmpVal;
}

SparseVector *OptProblemOriginalNonlinearObjective::calculateConstraintFunctionGradient(int idx,
                                                                                        std::vector<double> point)
{
    int number;
    SparseVector *tmpVector;

    // If this is the nonlinear objective constraint, we need to add the gradient for the mu variable at the end.
    if (idx == -1 || idx == this->getNonlinearObjectiveConstraintIdx())
    {
        auto tmpArray = getProblemInstance()->calculateObjectiveFunctionGradient(&point.at(0), -1, true);
        ProcessInfo::getInstance().solutionStatistics.numberOfGradientEvaluations++;

        number = getProblemInstance()->getVariableNumber();
        std::vector<int> tmpIndexes;
        std::vector<double> tmpValues;

        int nonZeroVals = 0;
        for (int i = 0; i < number; i++)
        {
            if (tmpArray[i] != 0)
            {
                tmpIndexes.push_back(i);
                tmpValues.push_back(tmpArray[i]);
                nonZeroVals++;
            }
        }

        tmpVector = new SparseVector(nonZeroVals + 1);

        tmpIndexes.push_back(this->getNonlinearObjectiveVariableIdx());
        tmpValues.push_back(-1);

        for (int i = 0; i < nonZeroVals + 1; i++)
        {
            tmpVector->indexes[i] = tmpIndexes.at(i);
            tmpVector->values[i] = tmpValues.at(i);
        }
    }
    else
    {
        tmpVector = OptProblem::calculateConstraintFunctionGradient(idx, point);
    }

    return (tmpVector);
}

// For all constraints except the additional nonlinear objective constraint
IndexValuePair OptProblemOriginalNonlinearObjective::getMostDeviatingAllConstraint(std::vector<double> point)
{
    IndexValuePair valpair;

    int numConstr = this->getNumberOfConstraints();

    std::vector<double> constrDevs(numConstr);

    for (int i = 0; i < numConstr; i++)
    {
        if (i == this->getNonlinearObjectiveConstraintIdx() || i == -1)
        {
            constrDevs.at(i) = 0.0;
        }
        else
        {
            if (getProblemInstance()->getConstraintTypes()[i] != 'E')
            {
                constrDevs.at(i) = calculateConstraintFunctionValue(i, point);
            }
            else
            {
                constrDevs.at(i) = abs(calculateConstraintFunctionValue(i, point));
            }
        }
    }

    auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
    valpair.idx = std::distance(std::begin(constrDevs), biggest);
    valpair.value = *biggest;

    return valpair;
}

int OptProblemOriginalNonlinearObjective::getNumberOfNonlinearConstraints()
{
    int ctr = OptProblem::getNumberOfNonlinearConstraints();

    ctr++; //Nonlinear objective function constraint

    return ctr;
}

int OptProblemOriginalNonlinearObjective::getNumberOfConstraints()
{
    return getProblemInstance()->getConstraintNumber() + 1;
}

std::vector<std::string> OptProblemOriginalNonlinearObjective::getConstraintNames()
{
    std::string *tmpArray = getProblemInstance()->getConstraintNames();

    std::vector<std::string> tmpVector;

    for (int i = 0; i < getProblemInstance()->getConstraintNumber(); i++)
    {
        tmpVector.push_back(tmpArray[i]);
    }

    tmpVector.push_back(this->addedConstraintName);

    return tmpVector;
}

int OptProblemOriginalNonlinearObjective::getNumberOfVariables()
{
    return getProblemInstance()->getVariableNumber() + 1;
}

int OptProblemOriginalNonlinearObjective::getNumberOfRealVariables()
{
    return getProblemInstance()->getVariableNumber() - getProblemInstance()->getNumberOfBinaryVariables() - getProblemInstance()->getNumberOfIntegerVariables() + 1;
}

std::vector<std::string> OptProblemOriginalNonlinearObjective::getVariableNames()
{
    std::string *tmpArray = getProblemInstance()->getVariableNames();

    std::vector<std::string> tmpVector;

    for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
    {
        tmpVector.push_back(tmpArray[i]);
    }

    tmpVector.push_back(this->addedObjectiveVariableName);

    return tmpVector;
}

std::vector<char> OptProblemOriginalNonlinearObjective::getVariableTypes()
{
    char *tmpArray = getProblemInstance()->getVariableTypes();

    std::vector<char> tmpVector;

    for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
    {
        tmpVector.push_back(tmpArray[i]);
    }

    tmpVector.push_back('C');

    return tmpVector;
}

std::vector<double> OptProblemOriginalNonlinearObjective::getVariableLowerBounds()
{
    double *tmpArray = getProblemInstance()->getVariableLowerBounds();

    std::vector<double> tmpVector;

    for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
    {
        tmpVector.push_back(tmpArray[i]);
    }

    tmpVector.push_back(this->addedObjectiveVariableLowerBound);

    return tmpVector;
}

std::vector<double> OptProblemOriginalNonlinearObjective::getVariableUpperBounds()
{
    double *tmpArray = getProblemInstance()->getVariableUpperBounds();

    std::vector<double> tmpVector;

    for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
    {
        tmpVector.push_back(tmpArray[i]);
    }

    tmpVector.push_back(this->addedObjectiveVariableUpperBound);

    return tmpVector;
}

double OptProblemOriginalNonlinearObjective::getVariableLowerBound(int varIdx)
{
    if (varIdx == getNonlinearObjectiveVariableIdx())
    {
        return (this->addedObjectiveVariableLowerBound);
    }
    else
    {
        return (getProblemInstance()->instanceData->variables->var[varIdx]->lb);
    }
}

double OptProblemOriginalNonlinearObjective::getVariableUpperBound(int varIdx)
{
    if (varIdx == getNonlinearObjectiveVariableIdx())
    {
        return (this->addedObjectiveVariableUpperBound);
    }
    else
    {
        return (getProblemInstance()->instanceData->variables->var[varIdx]->ub);
    }
}

void OptProblemOriginalNonlinearObjective::setVariableUpperBound(int varIdx, double value)
{
    if (varIdx == getNonlinearObjectiveVariableIdx())
    {
        this->addedObjectiveVariableUpperBound = value;
    }
    else
    {
        getProblemInstance()->instanceData->variables->var[varIdx]->ub = value;
    }
}

void OptProblemOriginalNonlinearObjective::setVariableLowerBound(int varIdx, double value)
{
    if (varIdx == getNonlinearObjectiveVariableIdx())
    {
        this->addedObjectiveVariableLowerBound = value;
    }
    else
    {
        getProblemInstance()->instanceData->variables->var[varIdx]->lb = value;
    }
}

std::vector<std::pair<int, double>> OptProblemOriginalNonlinearObjective::getObjectiveFunctionVarCoeffPairs()
{
    std::vector<std::pair<int, double>> tmpVector;

    std::pair<int, double> tmpPair;

    tmpPair.first = getNonlinearObjectiveVariableIdx();
    tmpPair.second = 1.0;

    tmpVector.push_back(tmpPair);

    return tmpVector;
}

double OptProblemOriginalNonlinearObjective::getObjectiveConstant()
{
    return 0.0;
}

void OptProblemOriginalNonlinearObjective::setNonlinearConstraintIndexes()
{
    OptProblem::setNonlinearConstraintIndexes();

    auto NLCIndexes = getNonlinearConstraintIndexes();

    std::vector<int>::iterator it;
    it = NLCIndexes.begin();

    NLCIndexes.insert(it, -1);

    setNonlinearConstraints(NLCIndexes);
}