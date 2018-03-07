/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "OptProblem.h"

OptProblem::OptProblem()
{
    m_problemInstance = NULL;
}

OptProblem::~OptProblem()
{
    delete m_problemInstance;
}

int OptProblem::getNumberOfLinearConstraints()
{
    return getProblemInstance()->getConstraintNumber() - this->getNumberOfNonlinearConstraints();
}

int OptProblem::getNumberOfConstraints()
{
    return getProblemInstance()->getConstraintNumber();
}

std::vector<std::string> OptProblem::getConstraintNames()
{
    std::string *tmpArray = getProblemInstance()->getConstraintNames();
    std::vector<std::string> tmpVector(tmpArray, tmpArray + getProblemInstance()->getConstraintNumber());

    return tmpVector;
}

int OptProblem::getNumberOfVariables()
{
    return getProblemInstance()->getVariableNumber();
}

int OptProblem::getNumberOfBinaryVariables()
{
    return getProblemInstance()->getNumberOfBinaryVariables();
}

int OptProblem::getNumberOfDiscreteVariables()
{
    return (getProblemInstance()->getNumberOfBinaryVariables() + getProblemInstance()->getNumberOfIntegerVariables());
}

int OptProblem::getNumberOfIntegerVariables()
{
    return getProblemInstance()->getNumberOfIntegerVariables();
}

int OptProblem::getNumberOfRealVariables()
{
    return getProblemInstance()->getVariableNumber() - getProblemInstance()->getNumberOfBinaryVariables() - getProblemInstance()->getNumberOfIntegerVariables();
}

std::vector<std::string> OptProblem::getVariableNames()
{
    std::string *tmpArray = getProblemInstance()->getVariableNames();
    std::vector<std::string> tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

    return tmpVector;
}

std::vector<char> OptProblem::getVariableTypes()
{
    char *tmpArray = getProblemInstance()->getVariableTypes();
    std::vector<char> tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

    return tmpVector;
}

std::vector<double> OptProblem::getVariableLowerBounds()
{
    double *tmpArray = getProblemInstance()->getVariableLowerBounds();

    std::vector<double> tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

    return tmpVector;
}

std::vector<double> OptProblem::getVariableUpperBounds()
{
    double *tmpArray = getProblemInstance()->getVariableUpperBounds();
    std::vector<double> tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

    return tmpVector;
}

double OptProblem::getVariableLowerBound(int varIdx)
{
    return (getProblemInstance()->instanceData->variables->var[varIdx]->lb);
}

double OptProblem::getVariableUpperBound(int varIdx)
{
    return (getProblemInstance()->instanceData->variables->var[varIdx]->ub);
}

void OptProblem::setVariableUpperBound(int varIdx, double value)
{
    getProblemInstance()->instanceData->variables->var[varIdx]->ub = value;
    getProblemInstance()->bVariablesModified = true;
}

void OptProblem::setVariableLowerBound(int varIdx, double value)
{
    getProblemInstance()->instanceData->variables->var[varIdx]->lb = value;
    getProblemInstance()->bVariablesModified = true;
}

std::vector<int> OptProblem::getRealVariableIndices()
{
    std::vector<int> indices;
    indices.reserve(getNumberOfRealVariables());

    auto varTypes = getVariableTypes();

    for (int i = 0; i < getNumberOfVariables(); i++)
    {
        if (varTypes.at(i) == 'C')
            indices.push_back(i);
    }

    return (indices);
}

std::vector<int> OptProblem::getDiscreteVariableIndices()
{
    std::vector<int> indices;
    indices.reserve(getNumberOfIntegerVariables() + getNumberOfBinaryVariables());

    auto varTypes = getVariableTypes();

    for (int i = 0; i < getNumberOfVariables(); i++)
    {
        if (varTypes.at(i) == 'I' || varTypes.at(i) == 'B')
            indices.push_back(i);
    }

    return (indices);
}

std::vector<int> OptProblem::getBinaryVariableIndices()
{
    std::vector<int> indices;
    indices.reserve(getNumberOfBinaryVariables());

    auto varTypes = getVariableTypes();

    for (int i = 0; i < getNumberOfVariables(); i++)
    {
        if (varTypes.at(i) == 'B')
            indices.push_back(i);
    }

    return (indices);
}

std::vector<int> OptProblem::getIntegerVariableIndices()
{
    std::vector<int> indices;
    indices.reserve(getNumberOfIntegerVariables());

    auto varTypes = getVariableTypes();

    for (int i = 0; i < getNumberOfVariables(); i++)
    {
        if (varTypes.at(i) == 'I')
            indices.push_back(i);
    }

    return (indices);
}

void OptProblem::printProblemStatistics()
{
#if _WIN32
    ProcessInfo::getInstance().outputSummary(
        "                                                                                     \n");
    ProcessInfo::getInstance().outputSummary("ÚÄÄÄ Problem statistics ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¿");

    ProcessInfo::getInstance().outputSummary(
        "³ Number of constraints (total/linear/nonlinear):   " + to_string(getProblemInstance()->getConstraintNumber()) + "/" + to_string(getNumberOfLinearConstraints()) + "/" + to_string(getNumberOfNonlinearConstraints()));

    ProcessInfo::getInstance().outputSummary(
        "³ Number of variables (total/real/binary/integer):  " + to_string(getNumberOfVariables()) + "/" + to_string(getNumberOfRealVariables()) + "/" + to_string(getNumberOfBinaryVariables()) + "/" + to_string(getNumberOfIntegerVariables()) + "/");

    ProcessInfo::getInstance().outputSummary("ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÙ");

#else

    ProcessInfo::getInstance().outputSummary("\n┌─── Problem statistics ─────────────────────────────────────────────────────────┐");

    ProcessInfo::getInstance().outputSummary(
        "│ Number of constraints (total/linear/nonlinear):   " + to_string(getProblemInstance()->getConstraintNumber()) + "/" + to_string(getNumberOfLinearConstraints()) + "/" + to_string(getNumberOfNonlinearConstraints()));

    ProcessInfo::getInstance().outputSummary(
        "│ Number of variables (total/real/binary/integer):  " + to_string(getNumberOfVariables()) + "/" + to_string(getNumberOfRealVariables()) + "/" + to_string(getNumberOfBinaryVariables()) + "/" + to_string(getNumberOfIntegerVariables()) + "/");

    ProcessInfo::getInstance().outputSummary(
        "└────────────────────────────────────────────────────────────────────────────────┘");
#endif
}

void OptProblem::exportProblemToOsil(std::string fileName)
{
    std::string osil = exportProblemToOsil();

    if (!UtilityFunctions::writeStringToFile(fileName, osil))
    {
        ProcessInfo::getInstance().outputError("Error when writing to file " + fileName);
    }
}

void OptProblem::saveProblemModelToFile(std::string fileName)
{
    std::string problem = getProblemInstance()->printModel();

    if (!UtilityFunctions::writeStringToFile(fileName, problem))
    {
        ProcessInfo::getInstance().outputError("Error when writing to file " + fileName);
    }
}

std::string OptProblem::exportProblemToOsil()
{
    OSiLWriter *osilWriter = new OSiLWriter();

    std::string osil = osilWriter->writeOSiL(getProblemInstance());

    delete osilWriter;

    return (osil);
}

IndexValuePair OptProblem::getMostDeviatingConstraint(std::vector<double> point)
{
    if (point.size() != this->getNumberOfVariables())
    {
        ProcessInfo::getInstance().outputError(
            "     Error: point size (" + to_string(point.size()) + ") does not match number of variables (" + to_string(this->getNumberOfVariables()) + ")!");
    }

    IndexValuePair valpair;

    std::vector<int> idxNLCs = this->getNonlinearConstraintIndexes();

    return (this->getMostDeviatingConstraint(point, idxNLCs).first);
}

std::pair<IndexValuePair, std::vector<int>> OptProblem::getMostDeviatingConstraint(std::vector<double> point,
                                                                                   std::vector<int> constrIdxs)
{
    IndexValuePair valpair;

    std::vector<int> activeConstraints;

    if (constrIdxs.size() == 0)
    {
        //Only a quadratic objective function and quadratic constraints
        valpair.idx = -1;
        valpair.value = 0.0;
    }
    else
    {
        std::vector<double> constrDevs(constrIdxs.size());

        for (int i = 0; i < constrIdxs.size(); i++)

        {
            constrDevs.at(i) = calculateConstraintFunctionValue(constrIdxs.at(i), point);

            if (constrDevs.at(i) >= 0)
                activeConstraints.push_back(constrIdxs.at(i));
        }

        auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
        valpair.idx = constrIdxs.at(std::distance(std::begin(constrDevs), biggest));
        valpair.value = *biggest;
    }

    return (std::make_pair(valpair, activeConstraints));
}

IndexValuePair OptProblem::getMostDeviatingAllConstraint(std::vector<double> point)
{

    IndexValuePair valpair;

    int numConstr = this->getNumberOfConstraints();

    std::vector<double> constrDevs(numConstr);

    for (int i = 0; i < numConstr; i++)
    {
        if (getProblemInstance()->getConstraintTypes()[i] == 'E')
        {
            constrDevs.at(i) = abs(calculateConstraintFunctionValue(i, point));
        }
        else
        {
            constrDevs.at(i) = calculateConstraintFunctionValue(i, point);
        }
    }

    auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
    valpair.idx = std::distance(std::begin(constrDevs), biggest);
    valpair.value = *biggest;

    return valpair;
}

vector<IndexValuePair> OptProblem::getMostDeviatingConstraints(std::vector<double> point, double tolerance)
{
    vector<IndexValuePair> valpairs;

    std::vector<int> idxNLCs = this->getNonlinearOrQuadraticConstraintIndexes();

    if (idxNLCs.size() == 0) //Only a quadratic objective function and quadratic constraints
    {
        IndexValuePair valpair;
        valpair.idx = -1;
        valpair.value = 0.0;

        valpairs.push_back(valpair);
    }
    else
    {
        if (tolerance < 0)
            tolerance = 0;
        if (tolerance > 1)
            tolerance = 1;

        std::vector<double> constrDevs(idxNLCs.size());

        for (int i = 0; i < idxNLCs.size(); i++)
        {
            constrDevs.at(i) = calculateConstraintFunctionValue(idxNLCs.at(i), point);
        }

        IndexValuePair valpair;
        auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
        valpair.idx = idxNLCs.at(std::distance(std::begin(constrDevs), biggest));
        valpair.value = *biggest;
        valpairs.push_back(valpair);

        double compVal;
        if (valpair.value >= 0)
        {
            compVal = valpair.value * (1 - tolerance);
        }
        else
        {
            compVal = valpair.value * (1 + tolerance);
        }

        for (int i = 0; i < idxNLCs.size(); i++)
        {
            if (idxNLCs.at(i) != valpair.idx)
            {
                if (constrDevs.at(i) >= compVal)
                {
                    IndexValuePair tmpPair;
                    tmpPair.idx = idxNLCs.at(i);
                    tmpPair.value = constrDevs.at(i);
                    valpairs.push_back(tmpPair);
                }
            }
        }
    }

    return (valpairs);
}

bool OptProblem::isConstraintsFulfilledInPoint(std::vector<double> point, double eps)
{
    std::vector<int> idxNLCs = this->getNonlinearConstraintIndexes();

    int numNLC = getNumberOfNonlinearConstraints();

    for (int i = 0; i < numNLC; i++)
    {
        double tmpVal = calculateConstraintFunctionValue(idxNLCs.at(i), point);
        if (tmpVal > eps)
        {
            return false;
        }
    }

    return true;
}

bool OptProblem::isConstraintsFulfilledInPoint(std::vector<double> point)
{
    return isConstraintsFulfilledInPoint(point, 0);
}

bool OptProblem::isDiscreteVariablesFulfilledInPoint(std::vector<double> point, double eps)
{
    std::vector<int> idxDiscrete = this->getDiscreteVariableIndices();

    for (int i = 0; i < idxDiscrete.size(); i++)
    {
        if (abs(point.at(i) - round(point.at(i))) > eps)
            return (false);
    }

    return (true);
}

bool OptProblem::isVariableBoundsFulfilledInPoint(std::vector<double> point, double eps)
{
    int numVars = this->getNumberOfVariables();

    for (int i = 0; i < numVars; i++)
    {
        if (point.at(i) - eps > this->getVariableUpperBound(i))
        {
            return (false);
        }
        if (point.at(i) + eps < this->getVariableLowerBound(i))
        {
            return (false);
        }
    }
}

bool OptProblem::isLinearConstraintsFulfilledInPoint(std::vector<double> point, double eps)
{

    std::vector<int> idxLCs = this->getLinearConstraintIndexes();

    int numLC = idxLCs.size();

    for (int i = 0; i < numLC; i++)
    {
        double tmpVal = calculateConstraintFunctionValue(idxLCs.at(i), point);
        //std::cout << "Lin :" << tmpVal << std::endl;
        if (abs(tmpVal) > eps)
            return false;
    }

    return true;
}

bool OptProblem::isLinearConstraintsFulfilledInPoint(std::vector<double> point)
{
    return isLinearConstraintsFulfilledInPoint(point, 0);
}

SparseVector *OptProblem::calculateConstraintFunctionGradient(int idx, std::vector<double> point)
{
    auto gradient = getProblemInstance()->calculateConstraintFunctionGradient(&point.at(0), idx, true);

    ProcessInfo::getInstance().numGradientEvals++;

    if (idx != -1 && getProblemInstance()->getConstraintTypes()[idx] == 'G')
    {
        for (int i = 0; i < gradient->number; i++)
        {
            gradient->values[i] = -gradient->values[i];
        }
    }

    if (false && Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
    {
        auto numGradient = calculateGradientNumerically(idx, point);

        std::vector<double> nonSparseGrad(point.size(), 0.0);

        for (int i = 0; i < gradient->number; i++)
        {
            nonSparseGrad.at(gradient->indexes[i]) = gradient->values[i];
        }

        for (int i = 0; i < point.size(); i++)
        {
            if (abs((numGradient.at(i) - nonSparseGrad.at(i)) / max(nonSparseGrad.at(i), numGradient.at(i))) >= 0.1)
            {
                ProcessInfo::getInstance().outputAlways(
                    "Gradient calculation error (constraint " + std::to_string(idx) + ", variable " + std::to_string(i) + "): numerical " + UtilityFunctions::toString(numGradient.at(i)) + " exact " + UtilityFunctions::toString(nonSparseGrad.at(i)));

                for (int i = 0; i < gradient->number; i++)
                {
                    ProcessInfo::getInstance().outputAlways(
                        "Variable value (index: " + std::to_string(gradient->indexes[i]) + "): " + UtilityFunctions::toString(point.at(gradient->indexes[i])));
                }

                ProcessInfo::getInstance().outputAlways(getProblemInstance()->printModel(idx));
            }
        }
    }

    return (gradient);
}

double OptProblem::calculateOriginalObjectiveValue(std::vector<double> point)
{
    auto tmpVal = getProblemInstance()->calculateAllObjectiveFunctionValues(&point[0], true)[0];
    ProcessInfo::getInstance().numFunctionEvals++;

    return tmpVal;
}

double OptProblem::calculateConstraintFunctionValue(int idx, std::vector<double> point)
{
    if (point.size() != this->getNumberOfVariables())
    {
        ProcessInfo::getInstance().outputError(
            "Point size (" + std::to_string(point.size()) + ") does not equal number of variables (" + std::to_string(this->getNumberOfVariables()) + ") when calculating function value!");
    }

    double tmpVal = 0.0;
    if (!isObjectiveFunctionNonlinear() || idx != getNonlinearObjectiveConstraintIdx() || idx != -1) // Not the objective function
    {
        tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
        ProcessInfo::getInstance().numFunctionEvals++;

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
            std::cout << "Should not happen: " << idx << " " << getProblemInstance()->getConstraintTypes()[idx] << std::endl;
        }
    }
    else
    {
        tmpVal = getProblemInstance()->calculateFunctionValue(-1, &point.at(0), true);
        ProcessInfo::getInstance().numFunctionEvals++;
    }

    return tmpVal;
}

int OptProblem::getNumberOfNonlinearConstraints()
{
    int ctr = 0;

    std::vector<bool> isNonlinear(getProblemInstance()->getConstraintNumber(), false);

    for (int i = 0; i < getProblemInstance()->getNumberOfNonlinearExpressions(); i++)
    {
        int tmpIndex = getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

        if (tmpIndex != -1)
            isNonlinear.at(tmpIndex) = true;
    }

    if (!((static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QuadraticProblemStrategy::QuadraticallyConstrained))
    {
        for (int i = 0; i < getProblemInstance()->getNumberOfQuadraticTerms(); i++)
        {
            int tmpIndex = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

            if (tmpIndex != -1)
                isNonlinear.at(tmpIndex) = true;
        }
    }

    for (int i = 0; i < isNonlinear.size(); i++)
    {
        if (isNonlinear.at(i))
            ctr++;
    }

    return ctr;
}

int OptProblem::getNumberOfNonlinearConstraints(OSInstance *instance)
{
    int ctr = 0;

    std::vector<bool> isNonlinear(instance->getConstraintNumber(), false);

    for (int i = 0; i < instance->getNumberOfNonlinearExpressions(); i++)
    {
        int tmpIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;

        if (tmpIndex != -1)
            isNonlinear.at(tmpIndex) = true;
    }

    for (int i = 0; i < instance->getNumberOfQuadraticTerms(); i++)
    {
        int tmpIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

        if (tmpIndex != -1)
            isNonlinear.at(tmpIndex) = true;
    }

    for (int i = 0; i < isNonlinear.size(); i++)
    {
        if (isNonlinear.at(i))
            ctr++;
    }

    return ctr;
}

std::vector<std::pair<int, double>> OptProblem::getObjectiveFunctionVarCoeffPairs()
{
    int numCoeffs = getProblemInstance()->instanceData->objectives->obj[0]->numberOfObjCoef;

    std::vector<std::pair<int, double>> tmpVector(numCoeffs);

    for (int i = 0; i < numCoeffs; i++)
    {
        std::pair<int, double> tmpPair;

        tmpPair.first = getProblemInstance()->instanceData->objectives->obj[0]->coef[i]->idx;
        tmpPair.second = getProblemInstance()->instanceData->objectives->obj[0]->coef[i]->value;

        tmpVector.push_back(tmpPair);
    }

    return tmpVector;
}

double OptProblem::getObjectiveConstant()
{
    return getProblemInstance()->getObjectiveConstants()[0];
}

void OptProblem::copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed)
{
    if (source->instanceData->variables != NULL && source->instanceData->variables->numberOfVariables > 0)
    {
        double LBCont = Settings::getInstance().getDoubleSetting("ContinuousVariable.EmptyLowerBound", "Model");
        double UBCont = Settings::getInstance().getDoubleSetting("ContinuousVariable.EmptyUpperBound", "Model");
        double LBInt = Settings::getInstance().getDoubleSetting("IntegerVariable.EmptyLowerBound", "Model");
        double UBInt = Settings::getInstance().getDoubleSetting("IntegerVariable.EmptyUpperBound", "Model");

        int nvar = source->getVariableNumber();

        std::string *varname = source->getVariableNames();
        char *vartype = source->getVariableTypes();
        double *varlb = source->getVariableLowerBounds();
        double *varub = source->getVariableUpperBounds();

        destination->instanceData->variables = new Variables();
        destination->instanceData->variables->numberOfVariables = nvar;
        destination->instanceData->variables->var = new Variable *[nvar];

        if (!destination->setVariables(nvar, varname, varlb, varub, vartype))
            throw ErrorClass(
                "Error duplicating variable information");

        if (integerRelaxed)
        {
            for (int i = 0; i < nvar; i++)
            {
                destination->instanceData->variables->var[i]->type = 'C';
            }
        }

        for (int i = 0; i < nvar; i++)
        {
            if (vartype[i] == 'I')
            {
                if (destination->instanceData->variables->var[i]->lb < LBInt)
                {
                    ProcessInfo::getInstance().outputInfo(
                        "Corrected lower bound for variable " + varname[i] + " from " + to_string(destination->instanceData->variables->var[i]->lb) + " to " + to_string(LBInt));
                    destination->instanceData->variables->var[i]->lb = LBInt;
                }

                if (destination->instanceData->variables->var[i]->ub > UBInt)
                {
                    ProcessInfo::getInstance().outputInfo(
                        "Corrected upper bound for variable " + varname[i] + " from " + to_string(destination->instanceData->variables->var[i]->ub) + " to " + to_string(UBInt));
                    destination->instanceData->variables->var[i]->ub = UBInt;
                }
            }
            else if (vartype[i] == 'C')
            {
                if (destination->instanceData->variables->var[i]->lb < LBCont)
                {
                    ProcessInfo::getInstance().outputInfo(
                        "Corrected lower bound for variable " + varname[i] + " from " + to_string(destination->instanceData->variables->var[i]->lb) + " to " + to_string(LBCont));
                    destination->instanceData->variables->var[i]->lb = LBCont;
                }

                if (destination->instanceData->variables->var[i]->ub > UBCont)
                {
                    ProcessInfo::getInstance().outputInfo(
                        "Corrected upper bound for variable " + varname[i] + " from " + to_string(destination->instanceData->variables->var[i]->ub) + " to " + to_string(UBCont));
                    destination->instanceData->variables->var[i]->ub = UBCont;
                }
            }
            else if (vartype[i] == 'B')
            {
                if (destination->instanceData->variables->var[i]->lb < 0.0)
                {
                    ProcessInfo::getInstance().outputInfo(
                        "Corrected lower bound for variable " + varname[i] + " from " + to_string(destination->instanceData->variables->var[i]->lb) + " to 0");
                    destination->instanceData->variables->var[i]->lb = 0.0;
                }

                if (destination->instanceData->variables->var[i]->ub > 1.0)
                {
                    ProcessInfo::getInstance().outputInfo(
                        "Corrected upper bound for variable " + varname[i] + " from " + to_string(destination->instanceData->variables->var[i]->ub) + " to 1");
                    destination->instanceData->variables->var[i]->ub = 1.0;
                }
            }
        }
    }

    destination->bVariablesModified = true;
}

void OptProblem::copyObjectiveFunction(OSInstance *source, OSInstance *destination)
{
    if (source->instanceData->objectives != NULL)
    {
        int nobj = source->getObjectiveNumber();

        std::string *objname = source->getObjectiveNames();
        std::string *objdir = source->getObjectiveMaxOrMins();
        double *objconst = source->getObjectiveConstants();
        double *objweight = source->getObjectiveWeights();
        SparseVector **objcoeff = source->getObjectiveCoefficients();

        destination->instanceData->objectives = new Objectives();
        destination->instanceData->objectives->numberOfObjectives = nobj;
        destination->instanceData->objectives->obj = new Objective *[nobj];

        if (!destination->setObjectives(nobj, objname, objdir, objconst, objweight, objcoeff))
            throw ErrorClass(
                "Error duplicating objective information");
    }
}

void OptProblem::copyConstraints(OSInstance *source, OSInstance *destination)
{
    if (source->instanceData->constraints != NULL)
    {
        int ncon = source->getConstraintNumber();

        std::string *conname = source->getConstraintNames();
        double *conlb = source->getConstraintLowerBounds();
        double *conub = source->getConstraintUpperBounds();
        double *con_c = source->getConstraintConstants();

        destination->instanceData->constraints = new Constraints();
        destination->instanceData->constraints->numberOfConstraints = ncon;
        destination->instanceData->constraints->con = new Constraint *[ncon];

        if (!destination->setConstraints(ncon, conname, conlb, conub, con_c))
            throw ErrorClass(
                "Error duplicating constraint information");
    }
}

void OptProblem::copyQuadraticTerms(OSInstance *source, OSInstance *destination)
{
    if (source->instanceData->quadraticCoefficients != NULL)
    {
        int nquad = source->getNumberOfQuadraticTerms();
        QuadraticTerms *qcoef = source->getQuadraticTerms();
#ifdef _WIN32
        if (!destination->setQuadraticCoefficients(nquad, qcoef->rowIndexes, qcoef->varOneIndexes, qcoef->varTwoIndexes,
                                                   qcoef->coefficients, 0, nquad - 1))
            throw ErrorClass("Error duplicating quadratic coefficients");
#else

        if (!destination->setQuadraticCoefficients(nquad, qcoef->rowIndexes, qcoef->varOneIndexes, qcoef->varTwoIndexes,
                                                   qcoef->coefficients, 0, nquad - 1))
            throw ErrorClass("Error duplicating quadratic coefficients");
#endif
    }
}

void OptProblem::copyNonlinearExpressions(OSInstance *source, OSInstance *destination)
{
    int numNonlinearExpressions = source->getNumberOfNonlinearExpressions();
    destination->instanceData->nonlinearExpressions = new NonlinearExpressions();
    destination->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = numNonlinearExpressions;

    if (numNonlinearExpressions > 0)
    {
        destination->instanceData->nonlinearExpressions->nl = new Nl *[numNonlinearExpressions];

        for (int i = 0; i < numNonlinearExpressions; i++)
        {
            int rowIdx = source->instanceData->nonlinearExpressions->nl[i]->idx;

            auto nlNodeVec = source->getNonlinearExpressionTreeInPrefix(rowIdx);

            destination->instanceData->nonlinearExpressions->nl[i] = new Nl();

#ifdef _WIN32
            destination->instanceData->nonlinearExpressions->nl[i]->osExpressionTree = new OSExpressionTree(
                *source->instanceData->nonlinearExpressions->nl[i]->osExpressionTree);
#else
            destination->instanceData->nonlinearExpressions->nl[i]->osExpressionTree = new ScalarExpressionTree(
                *source->instanceData->nonlinearExpressions->nl[i]->osExpressionTree);
#endif

            destination->instanceData->nonlinearExpressions->nl[i]->idx = rowIdx;
            nlNodeVec.clear();
        }
    }
}

void OptProblem::copyLinearTerms(OSInstance *source, OSInstance *destination)
{
    if (source->instanceData->linearConstraintCoefficients != NULL)
    {
        int ncoef = source->getLinearConstraintCoefficientNumber();

        if (ncoef == 0)
            return;

        bool isColMajor = source->getLinearConstraintCoefficientMajor();
        int nstart;
        SparseMatrix *coeff;

        // getLinearConstraintCoefficients returns a pointer to a sparse matrix structure
        if (isColMajor)
        {
            nstart = source->getVariableNumber();
            coeff = source->getLinearConstraintCoefficientsInColumnMajor();
        }
        else
        {
            nstart = source->getConstraintNumber();
            coeff = source->getLinearConstraintCoefficientsInRowMajor();
        }

        if (!destination->copyLinearConstraintCoefficients(ncoef, isColMajor, coeff->values, 0, ncoef - 1,
                                                           coeff->indexes, 0, ncoef - 1, coeff->starts, 0, nstart))
            throw ErrorClass(
                "Error duplicating linear constraint coefficients");
    }
}

void OptProblem::setNonlinearConstraintIndexes()
{
    std::vector<bool> isNonlinear(this->getProblemInstance()->getConstraintNumber(), false);
    std::vector<bool> isQuadratic(this->getProblemInstance()->getConstraintNumber(), false);
    int numNonlinExpr = this->getProblemInstance()->getNumberOfNonlinearExpressions();
    int numQuadTerms = this->getProblemInstance()->getNumberOfQuadraticTerms();

    for (int i = 0; i < numNonlinExpr; i++)
    {
        int tmpIndex = this->getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

        if (tmpIndex != -1)
        {
            isNonlinear.at(tmpIndex) = true;
        }
    }

    //	if (Settings::getInstance().getBoolSetting("UseQuadraticProgramming", "Algorithm"))
    if (static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual")) == ES_QuadraticProblemStrategy::QuadraticallyConstrained)
    {
        for (int i = 0; i < numQuadTerms; i++)
        {
            int tmpIndex = this->getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

            if (tmpIndex != -1)
            {
                isQuadratic.at(tmpIndex) = true;
            }
        }
    }
    else
    {
        for (int i = 0; i < numQuadTerms; i++)
        {
            int tmpIndex = this->getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

            if (tmpIndex != -1)
            {
                isNonlinear.at(tmpIndex) = true;
            }
        }
    }

    std::vector<int> NLCIndexes, QCIndexes;

    for (int i = 0; i < isNonlinear.size(); i++)
    {
        if (isNonlinear.at(i))
            NLCIndexes.push_back(i);
    }

    for (int i = 0; i < isQuadratic.size(); i++)
    {
        if (isQuadratic.at(i))
            QCIndexes.push_back(i);
    }

    //UtilityFunctions::displayVector(NLCIndexes);
    //UtilityFunctions::displayVector(QCIndexes);

    setNonlinearConstraints(NLCIndexes);
    setQuadraticConstraints(QCIndexes);
}

/*
 void OptProblemBase::setQuadraticConstraintIndexes()
 {
 std::vector<int> quadraticIndexes;

 if (Settings::getInstance().getBoolSetting("UseQuadraticProgramming", "Algorithm"))
 {
 std::vector<bool> isQuadratic(this->getProblemInstance()->getConstraintNumber(), false);

 int numNonlinExpr = this->getProblemInstance()->instanceData->nonlinearExpressions == NULL ? 0 : this->getProblemInstance()->getNumberOfNonlinearExpressions();
 int numQuadTerms = this->getProblemInstance()->instanceData->quadraticCoefficients == NULL ? 0 : this->getProblemInstance()->getNumberOfQuadraticTerms();

 for (int i = 0; i < numQuadTerms; i++)
 {
 int tmpIndex = this->getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

 if (tmpIndex != -1)
 {
 isQuadratic.at(tmpIndex) = true;
 }
 }

 for (int i = 0; i < numNonlinExpr; i++)
 {
 int tmpIndex = this->getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

 if (tmpIndex != -1)
 {
 isQuadratic.at(tmpIndex) = false;
 }
 }

 for (int i = 0; i < isQuadratic.size(); i++)
 {
 if (isQuadratic.at(i))	quadraticIndexes.push_back(i);
 }

 }

 setQuadraticConstraints(quadraticIndexes);
 }*/

std::vector<int> OptProblem::getLinearConstraintIndexes()
{
    std::vector<int> idxs;
    int numCon = this->getProblemInstance()->getConstraintNumber();
    //std::cout << "Nonlinear constraint index : " << this->getNonlinearObjectiveConstraintIdx() << std::endl;
    //int numCon = this->getNumberOfConstraints();
    //idxs.reserve(numCon);

    for (int i = 0; i < numCon; i++)
    {
        if (!isConstraintNonlinear(this->getProblemInstance(), i) && this->getNonlinearObjectiveConstraintIdx() != i)
        {
            //std::cout << "Constr " << i << " is linear!" << std::endl;
            idxs.push_back(i);
        }
    }

    return idxs;
}

bool OptProblem::hasVariableBoundsBeenTightened(int varIndex)
{
    return (m_variableBoundTightened.at(varIndex));
}

void OptProblem::setVariableBoundsAsTightened(int varIndex)
{
    if (m_variableBoundTightened.size() == 0)
        m_variableBoundTightened = std::vector<bool>(this->getNumberOfVariables(),
                                                     false);

    m_variableBoundTightened.at(varIndex) = true;
}

std::vector<int> OptProblem::getNonlinearConstraintIndexes()
{
    return m_nonlinearConstraints;
}

void OptProblem::setNonlinearConstraints(std::vector<int> idxs)
{
    m_nonlinearConstraints = idxs;

    if (m_nonlinearOrQuadraticConstraints.size() == 0)
    {
        m_nonlinearOrQuadraticConstraints = idxs;
    }
    else
    {
        vector<int> newVect;
        set_union(m_nonlinearOrQuadraticConstraints.begin(), m_nonlinearOrQuadraticConstraints.end(),
                  m_nonlinearConstraints.begin(), m_nonlinearConstraints.end(), back_inserter(newVect));

        m_nonlinearOrQuadraticConstraints = newVect;
    }
}

std::vector<int> OptProblem::getQuadraticConstraintIndexes()
{
    return m_quadraticConstraints;
}

void OptProblem::setQuadraticConstraints(std::vector<int> idxs)
{
    m_quadraticConstraints = idxs;

    if (m_nonlinearOrQuadraticConstraints.size() == 0)
    {
        m_nonlinearOrQuadraticConstraints = idxs;
    }
    else
    {
        vector<int> newVect;
        set_union(m_nonlinearOrQuadraticConstraints.begin(), m_nonlinearOrQuadraticConstraints.end(),
                  m_quadraticConstraints.begin(), m_quadraticConstraints.end(), back_inserter(newVect));

        m_nonlinearOrQuadraticConstraints = newVect;
    }
}

std::vector<int> OptProblem::getNonlinearOrQuadraticConstraintIndexes()
{
    return m_nonlinearOrQuadraticConstraints;
}

/*
 void OptProblemBase::setNonlinearOrQuadraticConstraints(std::vector<int> idxs)
 {
 m_nonlinearOrQuadraticConstraints = idxs;
 }*/

bool OptProblem::isTypeOfObjectiveMinimize()
{
    return m_isTypeOfObjectiveMinimize;
}

bool OptProblem::isObjectiveFunctionNonlinear()
{
    return m_isObjectiveFunctionNonlinear;
}

void OptProblem::setTypeOfObjectiveMinimize(bool value)
{
    m_isTypeOfObjectiveMinimize = value;
}

void OptProblem::setObjectiveFunctionNonlinear(bool value)
{
    m_isObjectiveFunctionNonlinear = value;
}

bool OptProblem::isConstraintNonlinear(OSInstance *instance, int constrIdx)
{

    for (int i = 0; i < instance->getNumberOfNonlinearExpressions(); i++)
    {
        int tmpIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;

        if (tmpIndex == constrIdx)
            return true;
    }

    for (int i = 0; i < instance->getNumberOfQuadraticTerms(); i++)
    {
        int tmpIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

        if (tmpIndex == constrIdx)
            return true;
    }

    return false;
}

bool OptProblem::isConstraintNonlinear(int constrIdx)
{
    for (int i = 0; i < getProblemInstance()->getNumberOfNonlinearExpressions(); i++)
    {
        int tmpIndex = getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

        if (tmpIndex == constrIdx)
            return true;
    }

    auto QPStrategy = static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"));

    if (QPStrategy == ES_QuadraticProblemStrategy::Nonlinear || QPStrategy != ES_QuadraticProblemStrategy::QuadraticallyConstrained)
    //	if (!Settings::getInstance().getBoolSetting("UseQuadraticProgramming", "Algorithm"))
    {
        for (int i = 0; i < getProblemInstance()->getNumberOfQuadraticTerms(); i++)
        {
            int tmpIndex = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

            if (tmpIndex == constrIdx)
                return true;
        }
    }

    return false;
}

bool OptProblem::isConstraintQuadratic(int constrIdx)
{
    auto QPStrategy = static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"));

    if (QPStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained || QPStrategy == ES_QuadraticProblemStrategy::QuadraticObjective)
    //	if (Settings::getInstance().getBoolSetting("UseQuadraticProgramming", "Algorithm"))
    {
        for (int i = 0; i < getProblemInstance()->getNumberOfQuadraticTerms(); i++)
        {
            int tmpIndex = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

            if (tmpIndex == constrIdx)
                return true;
        }
    }

    return false;
}

bool OptProblem::isProblemNonlinear(OSInstance *instance)
{
    if (instance->getNumberOfNonlinearExpressions() != 0)
        return true;

    if (instance->getNumberOfQuadraticTerms() != 0)
        return true;

    return false;
}

OSInstance *OptProblem::getProblemInstance()
{
    return m_problemInstance;
}

void OptProblem::setVariableBoundsTightened(std::vector<bool> status)
{
    m_variableBoundTightened = status;
}

std::vector<double> OptProblem::calculateGradientNumerically(int constraintIndex, std::vector<double> point)
{
    std::vector<double> point1(point);
    std::vector<double> point2(point);
    std::vector<double> numGradient(point.size(), 0.0);

    for (int i = 0; i < point.size(); i++)
    {
        double stepSize = (point.at(i) != 0) ? point.at(i) * 0.00000001 : 0.001;
        double dblStepSize = 2 * stepSize;

        point1.at(i) += stepSize;
        point2.at(i) -= stepSize;

        numGradient.at(i) = (this->calculateConstraintFunctionValue(constraintIndex, point1) - this->calculateConstraintFunctionValue(constraintIndex, point2)) / dblStepSize;

        point1.at(i) = point.at(i);
        point2.at(i) = point.at(i);
    }

    return (numGradient);
}

void OptProblem::setProblemInstance(OSInstance *instance)
{
    m_problemInstance = instance;
}

void OptProblem::setNonlinearObjectiveConstraintIdx(int idx)
{
    m_idxNonlinearObjectiveConstraint = idx;
}

int OptProblem::getNonlinearObjectiveConstraintIdx()
{
    return m_idxNonlinearObjectiveConstraint;
}

void OptProblem::setNonlinearObjectiveVariableIdx(int idx)
{
    m_idxNonlinearObjectiveVariable = idx;
}

int OptProblem::getNonlinearObjectiveVariableIdx()
{
    return m_idxNonlinearObjectiveVariable;
}

void OptProblem::setObjectiveFunctionType(E_ObjectiveFunctionType type)
{
    m_objectiveFunctionType = type;
}

E_ObjectiveFunctionType OptProblem::getObjectiveFunctionType()
{
    return m_objectiveFunctionType;
}

void OptProblem::repairNonboundedVariables()
{
    double LBCont = Settings::getInstance().getDoubleSetting("ContinuousVariable.EmptyLowerBound", "Model");
    double UBCont = Settings::getInstance().getDoubleSetting("ContinuousVariable.EmptyUpperBound", "Model");
    double LBInt = Settings::getInstance().getDoubleSetting("IntegerVariable.EmptyLowerBound", "Model");
    double UBInt = Settings::getInstance().getDoubleSetting("IntegerVariable.EmptyUpperBound", "Model");

    auto varTypes = getVariableTypes();

    int numVar = getNumberOfVariables();

    for (int i = 0; i < numVar; i++)
    {
        if (varTypes.at(i) == 'I')
        {
            if (getVariableLowerBound(i) < LBInt)
            {
                setVariableLowerBound(i, LBInt);
            }

            if (getVariableUpperBound(i) > UBInt)
            {
                setVariableUpperBound(i, UBInt);
            }
        }
        else if (varTypes.at(i) == 'C')
        {
            if (getVariableLowerBound(i) < LBCont)
            {
                setVariableLowerBound(i, LBCont);
            }

            if (getVariableUpperBound(i) > UBCont)
            {
                setVariableUpperBound(i, UBCont);
            }
        }
        else if (varTypes.at(i) == 'B')
        {
            if (getVariableLowerBound(i) < 0.0)
            {
                setVariableLowerBound(i, 0.0);
            }

            if (getVariableUpperBound(i) > 1.0)
            {
                setVariableUpperBound(i, 1.0);
            }
        }
        else if (varTypes.at(i) == 'D')
        {
            if (getVariableLowerBound(i) < 0.0)
            {
                setVariableLowerBound(i, 0.0);
            }

            if (getVariableUpperBound(i) > UBCont)
            {
                setVariableUpperBound(i, UBCont);
            }
        }
    }
}

std::vector<QuadraticTerm *> OptProblem::getQuadraticTermsInConstraint(int constrIdx)
{

    auto QPStrategy = static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"));
    std::vector<QuadraticTerm *> quadTerms;

    if (constrIdx != -1 && QPStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained)
    {
        int numQuadTerms = getProblemInstance()->getNumberOfQuadraticTerms();

        quadTerms.reserve(numQuadTerms);

        for (int i = 0; i < numQuadTerms; i++)
        {
            auto tmpTerm = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i];

            if (tmpTerm->idx == constrIdx)
                quadTerms.push_back(tmpTerm);
        }
    }
    else if (constrIdx == -1 && (QPStrategy == ES_QuadraticProblemStrategy::QuadraticallyConstrained || QPStrategy == ES_QuadraticProblemStrategy::QuadraticObjective))

    {
        int numQuadTerms = getProblemInstance()->getNumberOfQuadraticTerms();

        quadTerms.reserve(numQuadTerms);

        for (int i = 0; i < numQuadTerms; i++)
        {
            auto tmpTerm = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i];

            if (tmpTerm->idx == constrIdx)
                quadTerms.push_back(tmpTerm);
        }
    }

    return (quadTerms);
}

void OptProblem::fixVariable(int varIdx, double value)
{
    if (value >= getProblemInstance()->instanceData->variables->var[varIdx]->lb && value <= getProblemInstance()->instanceData->variables->var[varIdx]->ub)
    {
        getProblemInstance()->instanceData->variables->var[varIdx]->lb = value;
        getProblemInstance()->instanceData->variables->var[varIdx]->ub = value;
    }
    else
    {
        ProcessInfo::getInstance().outputError(
            "     Cannot fix variable value for variable with index " + to_string(varIdx) + ": not within bounds (" + UtilityFunctions::toString(getProblemInstance()->instanceData->variables->var[varIdx]->lb) + " < " + UtilityFunctions::toString(value) + " < " + UtilityFunctions::toString(getProblemInstance()->instanceData->variables->var[varIdx]->ub));
    }
}

bool OptProblem::isProblemDiscrete()
{
    if (this->getNumberOfIntegerVariables() + this->getNumberOfBinaryVariables() > 0)
        return (true);

    return (false);
}
