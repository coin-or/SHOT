/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverCuttingPlaneMinimax.h"

namespace SHOT
{

class MinimizationFunction
{
  private:
    VectorDouble firstPt;
    VectorDouble secondPt;
    ProblemPtr NLPProblem;

  public:
    MinimizationFunction(VectorDouble ptA, VectorDouble ptB, ProblemPtr prob)
    {
        firstPt = ptA;
        secondPt = ptB;
        NLPProblem = prob;
    }

    double operator()(const double x)
    {
        int length = secondPt.size();
        VectorDouble ptNew(length);

        for (int i = 0; i < length; i++)
        {
            ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
        }

        auto maxDev = NLPProblem->getMaxNumericConstraintValue(ptNew, NLPProblem->nonlinearConstraints);

        return (maxDev.normalizedRHSValue);
    }
};

NLPSolverCuttingPlaneMinimax::NLPSolverCuttingPlaneMinimax(EnvironmentPtr envPtr, ProblemPtr problem) : INLPSolver(envPtr), originalProblem(problem)
{
    auto solver = static_cast<ES_MIPSolver>(env->settings->getIntSetting("MIP.Solver", "Dual"));

    if (solver != ES_MIPSolver::Cplex && solver != ES_MIPSolver::Gurobi && solver != ES_MIPSolver::Cbc)
    {
        env->output->outputError("Error in solver definition for cutting plane minimax solver. Check option 'Dual.MIP.Solver'.");
        throw new ErrorClass("Error in MIP solver definition for cutting plane minimax solver. Check option 'Dual.MIP.Solver'.");
    }

#ifdef HAS_CPLEX
    if (solver == ES_MIPSolver::Cplex)
    {
        LPSolver = std::make_unique<MIPSolverCplex>(env);
        env->output->outputInfo("Cplex selected as MIP solver for minimax solver.");
    }
#endif

#ifdef HAS_GUROBI
    if (solver == ES_MIPSolver::Gurobi)
    {
        LPSolver = std::make_unique<MIPSolverGurobi>(env);
        env->output->outputInfo("Gurobi selected as MIP solver for minimax solver.");
    }
#endif

    if (solver == ES_MIPSolver::Cbc)
    {
        LPSolver = std::make_unique<MIPSolverOsiCbc>(env);
        env->output->outputInfo("Cbc selected as MIP solver for minimax solver.");
    }

    env->output->outputInfo("Creating LP problem for minimax solver");
    createProblem(LPSolver.get(), originalProblem);
    env->output->outputInfo("LP problem for minimax solver created");

    LPSolver->activateDiscreteVariables(false);
    LPSolver->initializeSolverSettings();
}

NLPSolverCuttingPlaneMinimax::~NLPSolverCuttingPlaneMinimax()
{
}

void NLPSolverCuttingPlaneMinimax::saveProblemToFile(std::string fileName)
{
}

E_NLPSolutionStatus NLPSolverCuttingPlaneMinimax::solveProblemInstance()
{
    int numVar = originalProblem->properties.numberOfVariables;

    // Sets the maximal number of iterations
    int maxIter = env->settings->getIntSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual");
    double termObjTolAbs = env->settings->getDoubleSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", "Dual");
    double termObjTolRel = env->settings->getDoubleSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", "Dual");
    double constrSelTol = env->settings->getDoubleSetting("ESH.InteriorPoint.CuttingPlane.ConstraintSelectionTolerance", "Dual");
    boost::uintmax_t maxIterSubsolver = env->settings->getIntSetting("ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", "Dual");
    int bitPrecision = env->settings->getIntSetting("ESH.InteriorPoint.CuttingPlane.BitPrecision", "Dual");

    // currSol is the current LP solution, and prevSol the previous one
    VectorDouble currSol, prevSol;

    double lambda; // Variable in the linesearch minimization
    double mu;     // Objective value for the linesearch minimization value

    // Corresponds to the difference between the LP solution objective value and
    // the objective found in the linesearch minimization procedure
    double maxObjDiffAbs = SHOT_DBL_MAX;
    double maxObjDiffRel = SHOT_DBL_MAX;

    double LPObjVar;

    E_NLPSolutionStatus statusCode;

    int numHyperAdded, numHyperTot;
    for (int i = 0; i <= maxIter; i++)
    {
        boost::uintmax_t maxIterSubsolverTmp = maxIterSubsolver;
        // Saves the LP problem to file if in debug mode
        if (env->settings->getBoolSetting("Debug.Enable", "Output"))
        {
            std::stringstream ss;
            ss << env->settings->getStringSetting("Debug.Path", "Output");
            ss << "/lpminimax";
            ss << i;
            ss << ".lp";
            LPSolver->writeProblemToFile(ss.str());
        }

        // Solves the problem and obtains the solution
        auto solStatus = LPSolver->solveProblem();
        env->solutionStatistics.numberOfProblemsMinimaxLP++;

        if (solStatus == E_ProblemSolutionStatus::Infeasible)
        {
            statusCode = E_NLPSolutionStatus::Infeasible;
            break;
        }
        else if (solStatus == E_ProblemSolutionStatus::Error)
        {
            statusCode = E_NLPSolutionStatus::Error;
            break;
        }
        else if (solStatus == E_ProblemSolutionStatus::Unbounded)
        {
            statusCode = E_NLPSolutionStatus::Unbounded;
            break;
        }
        else if (solStatus == E_ProblemSolutionStatus::TimeLimit)
        {
            statusCode = E_NLPSolutionStatus::TimeLimit;
        }
        else if (solStatus == E_ProblemSolutionStatus::IterationLimit)
        {
            statusCode = E_NLPSolutionStatus::IterationLimit;
        }
        else
        {
            statusCode = E_NLPSolutionStatus::Optimal;
        }

        auto LPVarSol = LPSolver->getVariableSolution(0);
        LPObjVar = LPSolver->getObjectiveValue();

        // Saves the LP solution to file if in debug mode
        if (env->settings->getBoolSetting("Debug.Enable", "Output"))
        {
            std::stringstream ss;
            ss << env->settings->getStringSetting("Debug.Path", "Output");
            ss << "/lpminimaxsolpt";
            ss << i;
            ss << ".txt";
            UtilityFunctions::saveVariablePointVectorToFile(LPVarSol, variableNames, ss.str());
        }

        if (isnan(LPObjVar))
        {
            statusCode = E_NLPSolutionStatus::Error;
            continue;
        }

        if (i == 0) // No linesearch minimization in first iteration, just add cutting plane in LP solution point
        {
            currSol = LPVarSol;
            lambda = -1; // For reporting purposes only
            mu = LPObjVar;
            numHyperAdded = 0;
            numHyperTot = 0;
            env->report->outputIterationDetailHeaderMinimax();
        }
        else
        {
            MinimizationFunction funct(LPVarSol, prevSol, originalProblem);

            // Solves the minization problem wrt lambda in [0, 1]

            auto minimizationResult = boost::math::tools::brent_find_minima(funct, 0.0, 1.0, bitPrecision,
                                                                            maxIterSubsolverTmp);

            lambda = minimizationResult.first;
            mu = minimizationResult.second;

            // Calculates the corresponding solution point
            for (int i = 0; i < numVar; i++)
            {
                currSol.at(i) = lambda * LPVarSol.at(i) + (1 - lambda) * prevSol.at(i);
            }

            // The difference between linesearch and LP objective values
            maxObjDiffAbs = abs(mu - LPObjVar);
            maxObjDiffRel = maxObjDiffAbs / ((1e-10) + abs(LPObjVar));

            // Saves the LP solution to file if in debug mode
            if (env->settings->getBoolSetting("Debug.Enable", "Output"))
            {
                std::stringstream ss;
                ss << env->settings->getStringSetting("Debug.Path", "Output");
                ss << "/lpminimaxlinesearchsolpt";
                ss << i;
                ss << ".txt";
                UtilityFunctions::saveVariablePointVectorToFile(currSol, variableNames, ss.str());
            }
        }

        env->report->outputIterationDetailMinimax((i + 1),
                                                  "LP",
                                                  env->process->getElapsedTime("Total"),
                                                  numHyperAdded,
                                                  numHyperTot,
                                                  LPObjVar,
                                                  mu,
                                                  maxObjDiffAbs,
                                                  maxObjDiffRel);

        if (mu <= 0 && (maxObjDiffAbs < termObjTolAbs || maxObjDiffRel < termObjTolRel))
        {
            statusCode = E_NLPSolutionStatus::Optimal;
            break;
        }

        // Gets the most deviated constraints with a tolerance
        /*NumericConstraintValues constraintValues;
        constraintValues.push_back(originalProblem->getMaxNumericConstraintValue(currSol, originalProblem->nonlinearConstraints));
        std::cout << "Constraint error " << constraintValues.at(0).error << " normalizedvalue " << constraintValues.at(0).normalizedValue << std::endl;*/
        /*double maxError = 0.0;

        NumericConstraintValues constraintValues;
        for (auto &C : originalProblem->nonlinearConstraints)
        {
            NumericConstraintValue constraintValue = calculateNumericValue(C, currSol);
            std::cout << "Constraint error " << constraintValue.error << " normalizedvalue " << constraintValue.normalizedValue << std::endl;
            if (constraintValue.error > 0)
            {
                constraintValues.push_back(constraintValue);
            }
            break;
        }*/

        auto constraintValues = originalProblem->getFractionOfDeviatingNonlinearConstraints(currSol, SHOT_DBL_MIN, 0);

        //numHyperAdded = tmpMostDevs.size();

        //numHyperTot = numHyperTot + constraintValues.size;

        //UtilityFunctions::displayVector(currSol);

        for (auto &NCV : constraintValues)
        {
            std::vector<PairIndexValue> elements;

            double constant = NCV.normalizedRHSValue;

            auto gradient = NCV.constraint->calculateGradient(currSol);

            for (auto &G : gradient)
            {
                PairIndexValue pair;
                pair.index = G.first->index;
                pair.value = G.second;

                elements.push_back(pair);

                constant -= G.second * currSol.at(G.first->index);

                //std::cout << "Constant " << G.first->index << " " << -G.second * currSol.at(G.first->index) << std::endl;

                //env->output->outputInfo("     Gradient for variable " + G.first->name + " in: " + std::to_string(G.second));
            }

            // Adding the objective term
            PairIndexValue pair;
            pair.index = numVar;
            pair.value = -1.0;

            elements.push_back(pair);

            //constant += currSol.back();

            // Adds the linear constraint
            LPSolver->addLinearConstraint(elements, constant);
            numHyperTot++;

            if (mu >= 0 && env->settings->getBoolSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual"))
            {
                auto tmpPoint = currSol;
                tmpPoint.pop_back();

                Hyperplane hyperplane;
                hyperplane.sourceConstraint = NCV.constraint;
                hyperplane.sourceConstraintIndex = NCV.constraint->index;
                hyperplane.generatedPoint = tmpPoint;
                hyperplane.source = E_HyperplaneSource::InteriorPointSearch;

                env->process->hyperplaneWaitingList.push_back(hyperplane);
            }
        }

        /*
        for (int j = 0; j < numHyperAdded; j++)
        {
            std::vector<PairIndexValue> elements; // Contains the terms in the hyperplane

            //double constant = originalProblem->calculateConstraintFunctionValue(tmpMostDevs.at(j).index, currSol);

            // Calculates the gradient
            auto nablag = originalProblem->calculateConstraintFunctionGradient(tmpMostDevs.at(j).index, currSol);
            //env->solutionStatistics.numberOfGradientEvaluations++;

            for (int i = 0; i < nablag->number; i++)
            {
                PairIndexValue pair;
                pair.index = nablag->indexes[i];
                pair.value = nablag->values[i];

                elements.push_back(pair);

                constant += -nablag->values[i] * currSol.at(nablag->indexes[i]);
            }

            delete nablag;

            // Adds the linear constraint
            LPSolver->addLinearConstraint(elements, constant);

            if (mu >= 0 && env->settings->getBoolSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual") && tmpMostDevs.at(j).index != NLPProblem->getNonlinearObjectiveConstraintIdx())
            {
                auto tmpPoint = currSol;

                while (tmpPoint.size() > env->model->originalProblem->getNumberOfVariables())
                {
                    tmpPoint.pop_back();
                }

                Hyperplane hyperplane;
                hyperplane.sourceConstraint = std::dynamic_pointer_cast<NumericConstraint>(env->reformulatedProblem->getConstraint(j));
                hyperplane.sourceConstraintIndex = j;
                hyperplane.generatedPoint = tmpPoint;
                hyperplane.source = E_HyperplaneSource::InteriorPointSearch;

                env->process->hyperplaneWaitingList.push_back(hyperplane);
            }
        }*/

        prevSol = currSol;

        if (i == maxIter - 1)
        {
            statusCode = E_NLPSolutionStatus::IterationLimit;
            break;
        }
    }

    currSol.pop_back();

    solution = currSol;
    objectiveValue = LPObjVar;

    return (statusCode);
}

NumericConstraintValue NLPSolverCuttingPlaneMinimax::calculateNumericValue(NumericConstraintPtr constraint, const VectorDouble &point)
{
    double value = constraint->calculateFunctionValue(point);

    NumericConstraintValue constrValue;
    constrValue.constraint = constraint->getPointer();
    constrValue.functionValue = value;
    constrValue.isFulfilledRHS = (value <= constraint->valueRHS);
    constrValue.normalizedRHSValue = value - constraint->valueRHS;

    constrValue.isFulfilledLHS = (value >= constraint->valueLHS);
    constrValue.normalizedLHSValue = constraint->valueLHS - value;

    constrValue.isFulfilled =
        (constrValue.isFulfilledRHS && constrValue.isFulfilledLHS);

    constrValue.normalizedValue = std::max(constrValue.normalizedRHSValue, constrValue.normalizedLHSValue);
    constrValue.error = std::max(0.0, constrValue.normalizedValue);

    return constrValue;
}

double NLPSolverCuttingPlaneMinimax::getSolution(int i)
{
    return (solution.at(i));
}

VectorDouble NLPSolverCuttingPlaneMinimax::getSolution()
{
    /*auto tmpSol = solution;

    if (env->model->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
    {
        tmpSol.pop_back();
    }*/

    return (solution);
}

double NLPSolverCuttingPlaneMinimax::getObjectiveValue()
{
    return (objectiveValue);
}

bool NLPSolverCuttingPlaneMinimax::createProblem(IMIPSolver *destination, ProblemPtr sourceProblem)
{
    // Now creating the variables

    bool variablesInitialized = true;

    for (auto &V : sourceProblem->allVariables)
    {
        variablesInitialized = variablesInitialized && destination->addVariable(V->name, V->type, V->lowerBound, V->upperBound);

        if (env->settings->getBoolSetting("Debug.Enable", "Output"))
        {
            variableNames.push_back(V->name);
        }
    }

    // Auxilliary objective variable for minimax problem
    double objLowerBound = env->settings->getDoubleSetting("ESH.InteriorPoint.MinimaxObjectiveLowerBound", "Dual");
    double objUpperBound = env->settings->getDoubleSetting("ESH.InteriorPoint.MinimaxObjectiveUpperBound", "Dual");

    variablesInitialized = variablesInitialized && destination->addVariable("shot_mmobjvar", E_VariableType::Real, -1e+10 + 1, objUpperBound);

    if (env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        variableNames.push_back("shot_mmobjvar");
    }

    if (!variablesInitialized)
        return false;

    // Now creating the objective function

    bool objectiveInitialized = true;

    objectiveInitialized = objectiveInitialized && destination->initializeObjective();

    objectiveInitialized = objectiveInitialized && destination->addLinearTermToObjective(1.0, sourceProblem->properties.numberOfVariables);

    objectiveInitialized = objectiveInitialized && destination->finalizeObjective(true);

    if (!objectiveInitialized)
        return false;

    // Now creating the constraints

    bool constraintsInitialized = true;

    for (auto &C : env->problem->linearConstraints)
    {
        constraintsInitialized = constraintsInitialized && destination->initializeConstraint();

        if (C->properties.hasLinearTerms)
        {
            for (auto &T : C->linearTerms.terms)
            {
                constraintsInitialized = constraintsInitialized && destination->addLinearTermToConstraint(T->coefficient, T->variable->index);
            }
        }

        constraintsInitialized = constraintsInitialized && destination->finalizeConstraint(C->name, C->valueLHS, C->valueRHS);
    }

    for (auto &C : env->problem->quadraticConstraints)
    {
        constraintsInitialized = constraintsInitialized && destination->initializeConstraint();

        if (C->properties.hasLinearTerms)
        {
            for (auto &T : C->linearTerms.terms)
            {
                constraintsInitialized = constraintsInitialized && destination->addLinearTermToConstraint(T->coefficient, T->variable->index);
            }
        }

        if (C->properties.hasQuadraticTerms)
        {
            for (auto &T : C->quadraticTerms.terms)
            {
                constraintsInitialized = constraintsInitialized && destination->addQuadraticTermToConstraint(T->coefficient, T->firstVariable->index, T->secondVariable->index);
            }
        }

        constraintsInitialized = constraintsInitialized && destination->finalizeConstraint(C->name, C->valueLHS, C->valueRHS);
    }

    if (!constraintsInitialized)
        return false;

    bool problemFinalized = destination->finalizeProblem();

    return (problemFinalized);
}

/*
bool NLPSolverCuttingPlaneMinimax::createProblemInstance()
{
    env->output->outputInfo("Creating NLP problem for minimax solver");
    dynamic_cast<OptProblemNLPMinimax *>(NLPProblem)->reformulate(origInstance);
    env->output->outputInfo("NLP problem for minimax solver created");

    env->output->outputInfo("Creating LP problem for minimax solver");
    LPSolver->createLinearProblem(NLPProblem);
    LPSolver->initializeSolverSettings();
    LPSolver->activateDiscreteVariables(false);
    env->output->outputInfo("LP problem for minimax solver created");

    return (true);
}*/

void NLPSolverCuttingPlaneMinimax::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    LPSolver->fixVariables(variableIndexes, variableValues);
}

void NLPSolverCuttingPlaneMinimax::unfixVariables()
{
    LPSolver->unfixVariables();
}

void NLPSolverCuttingPlaneMinimax::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues)
{
}

bool NLPSolverCuttingPlaneMinimax::isObjectiveFunctionNonlinear()
{
    return (false);
}

int NLPSolverCuttingPlaneMinimax::getObjectiveFunctionVariableIndex()
{
    return (SHOT_INT_MAX);
}

VectorDouble NLPSolverCuttingPlaneMinimax::getVariableLowerBounds()
{
    return (originalProblem->getVariableLowerBounds());
}

VectorDouble NLPSolverCuttingPlaneMinimax::getVariableUpperBounds()
{
    return (originalProblem->getVariableUpperBounds());
}

void NLPSolverCuttingPlaneMinimax::clearStartingPoint()
{
}

void NLPSolverCuttingPlaneMinimax::saveOptionsToFile(std::string fileName)
{
}
} // namespace SHOT