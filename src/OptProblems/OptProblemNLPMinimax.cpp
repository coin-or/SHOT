/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "OptProblemNLPMinimax.h"
#include "OSExpressionTree.h"
#include "vector"

OptProblemNLPMinimax::OptProblemNLPMinimax(EnvironmentPtr envPtr) : OptProblem(envPtr)
{
}

OptProblemNLPMinimax::~OptProblemNLPMinimax()
{
    /*if (osilReader != NULL)
    {
        delete osilReader;
        osilReader = NULL;
    }*/
}

void OptProblemNLPMinimax::reformulate(OSInstance *originalInstance)
{
    OSInstance *newInstance = NULL;
    newInstance = new OSInstance();

    this->setObjectiveFunctionType(E_ObjectiveFunctionType::Linear);

    this->setObjectiveFunctionNonlinear(isConstraintNonlinear(originalInstance, -1));

    this->setTypeOfObjectiveMinimize(true);

    this->copyVariables(originalInstance, newInstance, true);

    this->copyObjectiveFunction(originalInstance, newInstance);

    this->copyConstraints(originalInstance, newInstance);

    this->copyLinearTerms(originalInstance, newInstance);

    this->copyQuadraticTerms(originalInstance, newInstance);

    this->copyNonlinearExpressions(originalInstance, newInstance);

    this->setProblemInstance(newInstance);

    this->setNonlinearConstraintIndexes();

    if (this->isObjectiveFunctionNonlinear())
    {
        int tmpVal = originalInstance->getConstraintNumber();

        setNonlinearObjectiveConstraintIdx(tmpVal); // Sets a virtual constraint

        setNonlinearObjectiveVariableIdx(originalInstance->getVariableNumber());
        muindex = originalInstance->getVariableNumber() + 1;
    }
    else
    {
        muindex = originalInstance->getVariableNumber();
    }

    this->repairNonboundedVariables();
}

void OptProblemNLPMinimax::copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed)
{
    double LBCont = env->settings->getDoubleSetting("ContinuousVariable.EmptyLowerBound", "Model");
    double UBCont = env->settings->getDoubleSetting("ContinuousVariable.EmptyUpperBound", "Model");
    double LBInt = env->settings->getDoubleSetting("IntegerVariable.EmptyLowerBound", "Model");
    double UBInt = env->settings->getDoubleSetting("IntegerVariable.EmptyUpperBound", "Model");

    int numVar = source->getVariableNumber();

    if (this->isObjectiveFunctionNonlinear())
    {
        destination->setVariableNumber(numVar + 2);
    }
    else
    {
        destination->setVariableNumber(numVar + 1);
    }

    std::vector<std::string> varNames;
    varNames.assign(source->getVariableNames(), source->getVariableNames() + numVar);

    std::vector<char> varTypes;
    varTypes.assign(source->getVariableTypes(), source->getVariableTypes() + numVar);

    std::vector<double> varLBs;
    varLBs.assign(source->getVariableLowerBounds(), source->getVariableLowerBounds() + numVar);

    std::vector<double> varUBs;
    varUBs.assign(source->getVariableUpperBounds(), source->getVariableUpperBounds() + numVar);

    if (destination->getVariableNumber() == 0)
    {
        destination->setVariableNumber(numVar);
    }

    if (integerRelaxed)
    {
        for (int i = 0; i < numVar; i++)
        {
            std::string name = varNames.at(i);
            double lb = varLBs.at(i);
            double ub = varUBs.at(i);

            char type = 'C';

            destination->addVariable(i, name, lb, ub, type);
        }
    }
    else
    {
        for (int i = 0; i < numVar; i++)
        {
            std::string name = varNames.at(i);
            double lb = varLBs.at(i);
            double ub = varUBs.at(i);

            char type = varTypes.at(i);

            destination->addVariable(i, name, lb, ub, type);
        }
    }

    double tmpObjLowerBound = env->settings->getDoubleSetting("ESH.InteriorPoint.MinimaxObjectiveLowerBound", "Dual");
    double tmpObjUpperBound = env->settings->getDoubleSetting("ESH.InteriorPoint.MinimaxObjectiveUpperBound", "Dual");

    double tmpMuMaxAbsValue = env->settings->getDoubleSetting("NonlinearObjectiveVariable.Bound", "Model");

    if (this->isObjectiveFunctionNonlinear())
    {
        destination->addVariable(numVar, "mu", -tmpMuMaxAbsValue, tmpMuMaxAbsValue, 'C');
        destination->addVariable(numVar + 1, "tempobjvar", tmpObjLowerBound, tmpObjUpperBound, 'C');
    }
    else
    {
        destination->addVariable(numVar, "tempobjvar", tmpObjLowerBound, tmpObjUpperBound, 'C');
    }

    destination->bVariablesModified = true;
}

void OptProblemNLPMinimax::copyObjectiveFunction(OSInstance *source, OSInstance *destination)
{
    int numVar = source->getVariableNumber();

    destination->setObjectiveNumber(1);

    SparseVector *newobjcoeff = new SparseVector(1);

    if (this->isObjectiveFunctionNonlinear())
    {
        newobjcoeff->indexes[0] = numVar + 1;
    }
    else
    {
        newobjcoeff->indexes[0] = numVar;
    }

    newobjcoeff->values[0] = 1.0;

    destination->addObjective(-1, "newobj", "min", 0.0, 1.0, newobjcoeff);
    delete newobjcoeff;

    destination->bObjectivesModified = true;
}

void OptProblemNLPMinimax::copyConstraints(OSInstance *source, OSInstance *destination)
{
    int numCon = source->getConstraintNumber();

    if (this->isObjectiveFunctionNonlinear())
    {
        destination->setConstraintNumber(numCon + 1);
    }
    else
    {
        destination->setConstraintNumber(numCon);
    }

    for (int i = 0; i < numCon; i++)
    {
        std::string name = source->instanceData->constraints->con[i]->name;
        double lb = source->instanceData->constraints->con[i]->lb;
        double ub = source->instanceData->constraints->con[i]->ub;
        double constant = source->instanceData->constraints->con[i]->constant;

        destination->addConstraint(i, name, lb, ub, constant);
    }

    if (this->isObjectiveFunctionNonlinear())
    {
        destination->addConstraint(numCon, "objconstr", -OSDBL_MAX, -source->instanceData->objectives->obj[0]->constant,
                                   0.0);
    }

    destination->bConstraintsModified = true;
}

void OptProblemNLPMinimax::copyLinearTerms(OSInstance *source, OSInstance *destination)
{
    int row_nonz = 0;
    int obj_nonz = 0;
    int varIdx = 0;

    int rowNum = source->getConstraintNumber();
    int varNum = source->getVariableNumber();
    int elemNum = source->getLinearConstraintCoefficientNumber();

    int nonlinConstrs = getNumberOfNonlinearConstraints(source);

    SparseMatrix *m_linearConstraintCoefficientsInRowMajor = source->getLinearConstraintCoefficientsInRowMajor();

    int numTotalElements = elemNum + nonlinConstrs;

    if (this->isObjectiveFunctionNonlinear())
    {
        numTotalElements = numTotalElements + 2 + source->instanceData->objectives->obj[0]->numberOfObjCoef;
    }

    std::vector<int> rowIndices;
    std::vector<int> colIndices;
    std::vector<double> elements;

    rowIndices.reserve(numTotalElements);
    colIndices.reserve(numTotalElements);
    elements.reserve(numTotalElements);

    for (int rowIdx = 0; rowIdx < rowNum; rowIdx++)
    {
        if (m_linearConstraintCoefficientsInRowMajor != NULL && m_linearConstraintCoefficientsInRowMajor->valueSize > 0)
        {
            row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

            for (int j = 0; j < row_nonz; j++)
            {
                varIdx =
                    m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];

                double tmpVal =
                    m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];

                rowIndices.push_back(rowIdx);
                colIndices.push_back(
                    m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j]);
                elements.push_back(
                    m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j]);
            }
        }
        // Inserts the objective function variable into nonlinear constraints
        if (isConstraintNonlinear(source, rowIdx))
        {
            rowIndices.push_back(rowIdx);

            if (this->isObjectiveFunctionNonlinear())
            {
                colIndices.push_back(varNum + 1);
            }
            else
            {
                colIndices.push_back(varNum);
            }

            if (source->getConstraintTypes()[rowIdx] == 'L')
            {
                elements.push_back(-1.0);
            }
            else if (source->getConstraintTypes()[rowIdx] == 'G')
            {
                elements.push_back(1.0);
            }
        }
    }

    if (this->isObjectiveFunctionNonlinear())
    {
        auto objCoeffs = source->getObjectiveCoefficients()[0];
        int numObjCoeffs = source->getObjectiveCoefficientNumbers()[0];

        for (int i = 0; i < numObjCoeffs; i++)
        {
            rowIndices.push_back(rowNum);
            colIndices.push_back(objCoeffs->indexes[i]);
            elements.push_back(objCoeffs->values[i]);
        }

        rowIndices.push_back(rowNum);
        colIndices.push_back(varNum);
        elements.push_back(-1.0);

        rowIndices.push_back(rowNum);
        colIndices.push_back(varNum + 1);
        elements.push_back(-1.0);
    }

    CoinPackedMatrix *matrix = new CoinPackedMatrix(false, &rowIndices.at(0), &colIndices.at(0), &elements.at(0), numTotalElements);

    int numnonz = matrix->getNumElements();
    int valuesBegin = 0;
    int valuesEnd = numnonz - 1;
    int startsBegin = 0;
    int indexesBegin = 0;
    int indexesEnd = numnonz - 1;

    int startsEnd = this->isObjectiveFunctionNonlinear() ? varNum + 1 : varNum;

    auto tmp = const_cast<int *>(matrix->getVectorStarts());

    destination->setLinearConstraintCoefficients(numnonz, matrix->isColOrdered(),
                                                 const_cast<double *>(matrix->getElements()), valuesBegin, valuesEnd, const_cast<int *>(matrix->getIndices()),
                                                 indexesBegin, indexesEnd, const_cast<int *>(matrix->getVectorStarts()), startsBegin, startsEnd);

    destination->bConstraintsModified = true;

    //delete matrix; // Can not be deleted here...
}

void OptProblemNLPMinimax::copyQuadraticTerms(OSInstance *source, OSInstance *destination)
{
    int numQuadTerms = source->getNumberOfQuadraticTerms();

    if (numQuadTerms > 0)
    {
        auto quadTerms = source->getQuadraticTerms();
        auto varRowIndexes = quadTerms->rowIndexes;

        std::vector<int> varOneIndexes;
        std::vector<int> varTwoIndexes;
        std::vector<double> coefficients;
        std::vector<int> rowIndexes;

        varOneIndexes.reserve(numQuadTerms);
        varTwoIndexes.reserve(numQuadTerms);
        coefficients.reserve(numQuadTerms);
        rowIndexes.reserve(numQuadTerms);

        for (int i = 0; i < numQuadTerms; i++)
        {
            varOneIndexes.push_back(quadTerms->varOneIndexes[i]);
            varTwoIndexes.push_back(quadTerms->varTwoIndexes[i]);
            coefficients.push_back(quadTerms->coefficients[i]);

            if (varRowIndexes[i] != -1)
            {
                rowIndexes.push_back(quadTerms->rowIndexes[i]);
            }
            else
            {
                rowIndexes.push_back(source->getConstraintNumber());
            }
        }

        if (varOneIndexes.size() > 0)
        {
#ifdef linux
            destination->setQuadraticCoefficients(varOneIndexes.size(), &rowIndexes.at(0), &varOneIndexes.at(0),
                                                  &varTwoIndexes.at(0), &coefficients.at(0), 0, varOneIndexes.size() - 1);

#elif _WIN32
            destination->setQuadraticTerms(varOneIndexes.size(), &rowIndexes.at(0), &varOneIndexes.at(0),
                                           &varTwoIndexes.at(0), &coefficients.at(0), 0, varOneIndexes.size() - 1);

#else
            destination->setQuadraticCoefficients(varOneIndexes.size(), &rowIndexes.at(0), &varOneIndexes.at(0),
                                                  &varTwoIndexes.at(0), &coefficients.at(0), 0, varOneIndexes.size() - 1);
#endif
        }
    }
}

void OptProblemNLPMinimax::copyNonlinearExpressions(OSInstance *source, OSInstance *destination)
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

            destination->instanceData->nonlinearExpressions->nl[i] = new Nl();
#if OS_VERSION_MAJOR < 3
#if OS_VERSION_MINOR < 11
            destination->instanceData->nonlinearExpressions->nl[i]->osExpressionTree = new ScalarExpressionTree(
                *source->instanceData->nonlinearExpressions->nl[i]->osExpressionTree);
#else
            destination->instanceData->nonlinearExpressions->nl[i]->osExpressionTree = new RealValuedExpressionTree(
                *source->instanceData->nonlinearExpressions->nl[i]->osExpressionTree);
#endif
#else
            destination->instanceData->nonlinearExpressions->nl[i]->osExpressionTree = new RealValuedExpressionTree(
                *source->instanceData->nonlinearExpressions->nl[i]->osExpressionTree);
#endif

            if (rowIdx != -1)
            {
                destination->instanceData->nonlinearExpressions->nl[i]->idx = rowIdx;
            }
            else
            {
                destination->instanceData->nonlinearExpressions->nl[i]->idx = source->getConstraintNumber();
            }
        }
    }

    destination->bConstraintsModified = true;
}