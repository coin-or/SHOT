/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Model.h"
#include "OptProblems/OptProblemOriginal.h"

namespace SHOT
{

Model::Model(EnvironmentPtr envPtr) : env(envPtr)
{
    this->currentObjectiveBounds.first = -OSDBL_MAX;
    this->currentObjectiveBounds.second = OSDBL_MAX;

    osilWriter = new OSiLWriter();
}

Model::~Model()
{
    delete osilWriter;
    osilReaders.clear();
}

void Model::setStatistics()
{
    if (this->originalProblem == NULL)
        return;

    auto instance = this->originalProblem->getProblemInstance();

    statistics.isMinimizationProblem = this->originalProblem->isTypeOfObjectiveMinimize();

    statistics.objectiveFunctionType = this->originalProblem->getObjectiveFunctionType();

    statistics.numberOfConstraints = instance->getConstraintNumber();
    statistics.numberOfNonlinearConstraints = this->originalProblem->getNumberOfNonlinearConstraints();
    statistics.numberOfQuadraticConstraints = this->originalProblem->getNumberOfQuadraticConstraints();

    statistics.numberOfLinearConstraints = this->originalProblem->getNumberOfLinearConstraints();

    statistics.numberOfQuadraticTerms = instance->getNumberOfQuadraticTerms();

    auto QPStrategy = static_cast<ES_QuadraticProblemStrategy>(env->settings->getIntSetting("QuadraticStrategy", "Dual"));

    if (QPStrategy == ES_QuadraticProblemStrategy::Nonlinear && statistics.numberOfQuadraticTerms > 0)
    {
        statistics.quadraticTermsReformulatedAsNonlinear = true;
    }
    else if (QPStrategy == ES_QuadraticProblemStrategy::QuadraticObjective && !UtilityFunctions::areAllConstraintsQuadratic(instance))
    {
        statistics.quadraticTermsReformulatedAsNonlinear = true;
    }

    statistics.numberOfVariables = instance->getVariableNumber();
    statistics.numberOfIntegerVariables = instance->getNumberOfIntegerVariables();
    statistics.numberOfBinaryVariables = instance->getNumberOfBinaryVariables();
    statistics.numberOfSemicontinuousVariables = instance->getNumberOfSemiContinuousVariables();
    statistics.numberOfContinousVariables = statistics.numberOfVariables - (statistics.numberOfIntegerVariables + statistics.numberOfBinaryVariables + statistics.numberOfSemicontinuousVariables);

    if (statistics.numberOfIntegerVariables > 0 || statistics.numberOfBinaryVariables > 0 || statistics.numberOfSemicontinuousVariables > 0)
    {
        statistics.isDiscreteProblem = true;
    }
    else
    {
        statistics.isDiscreteProblem = false;
    }

    // Classify the problem
    if (!statistics.isDiscreteProblem)
    {
        if (statistics.numberOfNonlinearConstraints > 0 ||
            statistics.objectiveFunctionType == E_ObjectiveFunctionType::Nonlinear ||
            statistics.objectiveFunctionType == E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear)
        {
            statistics.problemType = E_ProblemType::NLP;
        }
        else if (statistics.numberOfQuadraticConstraints > 0)
        {
            statistics.problemType = E_ProblemType::QCQP;
        }
        else if (statistics.objectiveFunctionType == E_ObjectiveFunctionType::Quadratic)
        {
            statistics.problemType = E_ProblemType::QP;
        }
        else
        {
            statistics.problemType = E_ProblemType::LP;
        }
    }
    else
    {
        if (statistics.numberOfNonlinearConstraints > 0 ||
            statistics.objectiveFunctionType == E_ObjectiveFunctionType::Nonlinear ||
            statistics.objectiveFunctionType == E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear)
        {
            statistics.problemType = E_ProblemType::MINLP;
        }
        else if (statistics.numberOfQuadraticConstraints > 0)
        {
            statistics.problemType = E_ProblemType::MIQCQP;
        }
        else if (statistics.objectiveFunctionType == E_ObjectiveFunctionType::Quadratic)
        {
            statistics.problemType = E_ProblemType::MIQP;
        }
        else
        {
            statistics.problemType = E_ProblemType::MILP;
        }
    }
}

OSInstance *Model::getProblemInstanceFromOSiL(std::string osil)
{
    OSiLReader *osilReader = new OSiLReader();
    OSInstance *newInstance = osilReader->readOSiL(osil);

    osilReaders.push_back(osilReader); // To be able to properly deleting them without destroying the OSInstance object
    return (newInstance);
}

std::string Model::getOSiLFromProblemInstance(OSInstance *instance)
{
    return (osilWriter->writeOSiL(instance));
}

PairDouble Model::getCorrectedObjectiveBounds()
{
    PairDouble bounds;

    if (env->model->originalProblem->isTypeOfObjectiveMinimize())
    {
        bounds.first = currentObjectiveBounds.first;
        bounds.second = currentObjectiveBounds.second;
    }
    else
    {
        bounds.first = currentObjectiveBounds.second;
        bounds.second = currentObjectiveBounds.first;
    }

    return (bounds);
}
} // namespace SHOT