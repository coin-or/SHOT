/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverCuttingPlaneMinimax.h"
#include "../Tasks/TaskAddHyperplanes.h"

namespace SHOT
{

class MinimizationFunction
{
  private:
    VectorDouble firstPt;
    VectorDouble secondPt;
    OptProblem *NLPProblem;

  public:
    MinimizationFunction(VectorDouble ptA, VectorDouble ptB, OptProblem *prob)
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

        auto validNewPt = NLPProblem->getMostDeviatingConstraint(ptNew).value + ptNew.back();

        return (validNewPt);
    }
};

NLPSolverCuttingPlaneMinimax::NLPSolverCuttingPlaneMinimax(EnvironmentPtr envPtr) : INLPSolver(envPtr)
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
        LPSolver = new MIPSolverCplex(env);
        env->output->outputInfo("Cplex selected as MIP solver for minimax solver.");
    }
#endif

#ifdef HAS_GUROBI
    if (solver == ES_MIPSolver::Gurobi)
    {
        LPSolver = new MIPSolverGurobi(env);
        env->output->outputInfo("Gurobi selected as MIP solver for minimax solver.");
    }
#endif

    if (solver == ES_MIPSolver::Cbc)
    {
        LPSolver = new MIPSolverOsiCbc(env);
        env->output->outputInfo("Cbc selected as MIP solver for minimax solver.");
    }

    NLPProblem = new OptProblemNLPMinimax(env);
}

NLPSolverCuttingPlaneMinimax::~NLPSolverCuttingPlaneMinimax()
{
    delete NLPProblem;
    delete LPSolver;
}

/*void NLPSolverCuttingPlaneMinimax::saveProblemModelToFile(std::string fileName)
 {
 NLPProblem->saveProblemModelToFile(fileName);
 }*/

E_NLPSolutionStatus NLPSolverCuttingPlaneMinimax::solveProblemInstance()
{
    int numVar = NLPProblem->getNumberOfVariables();

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
    double maxObjDiffAbs = OSDBL_MAX;
    double maxObjDiffRel = OSDBL_MAX;

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
            MinimizationFunction funct(LPVarSol, prevSol, NLPProblem);

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
        auto tmpMostDevs = NLPProblem->getMostDeviatingConstraints(currSol, constrSelTol);
        numHyperAdded = tmpMostDevs.size();

        numHyperTot = numHyperTot + numHyperAdded;

        for (int j = 0; j < numHyperAdded; j++)
        {
            std::vector<PairIndexValue> elements; // Contains the terms in the hyperplane

            double constant = NLPProblem->calculateConstraintFunctionValue(tmpMostDevs.at(j).index, currSol);

            // Calculates the gradient
            auto nablag = NLPProblem->calculateConstraintFunctionGradient(tmpMostDevs.at(j).index, currSol);
            env->solutionStatistics.numberOfGradientEvaluations++;

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
                hyperplane.sourceConstraintIndex = j;
                hyperplane.generatedPoint = tmpPoint;
                hyperplane.source = E_HyperplaneSource::InteriorPointSearch;

                env->process->hyperplaneWaitingList.push_back(hyperplane);
            }
        }

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

double NLPSolverCuttingPlaneMinimax::getSolution(int i)
{
    return (solution.at(i));
}

VectorDouble NLPSolverCuttingPlaneMinimax::getSolution()
{
    auto tmpSol = solution;

    if (env->model->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
    {
        tmpSol.pop_back();
    }

    return (tmpSol);
}

double NLPSolverCuttingPlaneMinimax::getObjectiveValue()
{
    return (objectiveValue);
}

bool NLPSolverCuttingPlaneMinimax::createProblemInstance(OSInstance *origInstance)
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
}

void NLPSolverCuttingPlaneMinimax::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    LPSolver->fixVariables(variableIndexes, variableValues);
}

void NLPSolverCuttingPlaneMinimax::unfixVariables()
{
    LPSolver->unfixVariables();
}

void NLPSolverCuttingPlaneMinimax::setStartingPoint(VectorInteger variableIndexes,
                                                    VectorDouble variableValues)
{
}

bool NLPSolverCuttingPlaneMinimax::isObjectiveFunctionNonlinear()
{
    return (false);
}

int NLPSolverCuttingPlaneMinimax::getObjectiveFunctionVariableIndex()
{
    return (COIN_INT_MAX);
}

VectorDouble NLPSolverCuttingPlaneMinimax::getCurrentVariableLowerBounds()
{
    return (NLPProblem->getVariableLowerBounds());
}

VectorDouble NLPSolverCuttingPlaneMinimax::getCurrentVariableUpperBounds()
{
    return (NLPProblem->getVariableUpperBounds());
}

void NLPSolverCuttingPlaneMinimax::clearStartingPoint()
{
}

void NLPSolverCuttingPlaneMinimax::saveOptionsToFile(std::string fileName)
{
}
} // namespace SHOT