/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverCuttingPlaneRelaxed.h"
#include "../Tasks/TaskAddHyperplanes.h"

NLPSolverCuttingPlaneRelaxed::NLPSolverCuttingPlaneRelaxed()
{
    auto solver = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

    if (solver != ES_MIPSolver::Cplex && solver != ES_MIPSolver::Gurobi && solver != ES_MIPSolver::Cbc)
    {
        Output::getInstance().Output::getInstance().outputError("Error in solver definition for cutting plane minimax solver. Check option 'Dual.MIP.Solver'.");
        throw new ErrorClass("Error in MIP solver definition for cutting plane minimax solver. Check option 'Dual.MIP.Solver'.");
    }

#ifdef HAS_CPLEX
    if (solver == ES_MIPSolver::Cplex)
    {
        LPSolver = new MIPSolverCplex();
        Output::getInstance().outputInfo("Cplex selected as MIP solver for minimax solver.");
    }
#endif

#ifdef HAS_GUROBI
    if (solver == ES_MIPSolver::Gurobi)
    {
        LPSolver = new MIPSolverGurobi();
        Output::getInstance().outputInfo("Gurobi selected as MIP solver for minimax solver.");
    }
#endif

    if (solver == ES_MIPSolver::Cbc)
    {
        LPSolver = new MIPSolverOsiCbc();
        Output::getInstance().outputInfo("Cbc selected as MIP solver for minimax solver.");
    }

    lastHyperplaneAdded = 0;
}

NLPSolverCuttingPlaneRelaxed::~NLPSolverCuttingPlaneRelaxed()
{
    delete NLPProblem;
    delete LPSolver;
}

E_NLPSolutionStatus NLPSolverCuttingPlaneRelaxed::solveProblemInstance()
{
    int numVar = NLPProblem->getNumberOfVariables();

    // Sets the maximal number of iterations
    int maxIter = Settings::getInstance().getIntSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual");
    double constrSelTol = Settings::getInstance().getDoubleSetting("ESH.InteriorPoint.CuttingPlane.ConstraintSelectionTolerance", "Dual");
    double termTol = Settings::getInstance().getDoubleSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", "Dual");
    boost::uintmax_t maxIterSubsolver = Settings::getInstance().getIntSetting("ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", "Dual");

    // currSol is the current LP solution, and prevSol the previous one
    vector<double> currSol, prevSol, LPVarSol;

    double LPObjVar;

    E_NLPSolutionStatus statusCode;

    int numHyperAdded = 0;
    int numHyperTot = 0;

    // Adds the hyperplanes created elsewhere
    for (int i = lastHyperplaneAdded; i < ProcessInfo::getInstance().addedHyperplanes.size(); i++)
    {
        if (ProcessInfo::getInstance().addedHyperplanes.at(i).source == E_HyperplaneSource::LPFixedIntegers)
            continue;

        LPSolver->createHyperplane(ProcessInfo::getInstance().addedHyperplanes.at(i));
    }

    for (int i = 0; i < numVar; i++)
    {
        if (NLPProblem->hasVariableBoundsBeenTightened(i))
        {
            LPSolver->updateVariableBound(i, NLPProblem->getVariableLowerBound(i),
                                          NLPProblem->getVariableUpperBound(i));
        }
    }

    lastHyperplaneAdded = ProcessInfo::getInstance().addedHyperplanes.size();

    for (int i = 0; i < maxIter; i++)
    {
        boost::uintmax_t maxIterSubsolverTmp = maxIterSubsolver;
        // Saves the LP problem to file if in debug mode
        if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
        {
            stringstream ss;
            ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
            ss << "/nlpcuttingplanerelaxed";
            ss << i;
            ss << ".lp";
            LPSolver->writeProblemToFile(ss.str());
        }

        // Solves the problem and obtains the solution
        auto solStatus = LPSolver->solveProblem();

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
            break;
        }
        else if (solStatus == E_ProblemSolutionStatus::IterationLimit)
        {
            statusCode = E_NLPSolutionStatus::IterationLimit;
            break;
        }

        LPVarSol = LPSolver->getVariableSolution(0);
        LPObjVar = LPSolver->getObjectiveValue();

        if (true)
        {
            std::vector<double> externalPoint = LPVarSol;
            std::vector<double> internalPoint = ProcessInfo::getInstance().interiorPts.at(0)->point;

            try
            {
                auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(internalPoint, externalPoint,
                                                                                   Settings::getInstance().getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                                   Settings::getInstance().getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                                   Settings::getInstance().getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"));

                ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
                internalPoint = xNewc.first;
                externalPoint = xNewc.second;

                ProcessInfo::getInstance().addPrimalSolutionCandidate(internalPoint, E_PrimalSolutionSource::NLPRelaxed,
                                                                      ProcessInfo::getInstance().getCurrentIteration()->iterationNumber);

                auto errorExternal = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraints(
                    externalPoint, constrSelTol);

                numHyperAdded = errorExternal.size();
                numHyperTot = numHyperTot + numHyperAdded;

                for (int j = 0; j < numHyperAdded; j++)
                {
                    Hyperplane hyperplane;
                    hyperplane.sourceConstraintIndex = errorExternal.at(j).idx;
                    hyperplane.generatedPoint = externalPoint;
                    hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

                    LPSolver->createHyperplane(hyperplane);

                    if (Settings::getInstance().getBoolSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual"))
                    {
                        ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
                    }
                }

                std::string hyperplanesExpr;

                hyperplanesExpr = "+" + to_string(numHyperAdded) + " = " + to_string(numHyperTot);

                std::string tmpObjLP = UtilityFunctions::toString(LPObjVar);

                boost::format tmpLine;
                std::string tmpAbsDiff = ""; //((boost::format("%.5f") % maxObjDiffAbs).str());
                std::string tmpRelDiff = ((boost::format("%.5f") % errorExternal.at(0).value).str());
                tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % (i + 1) % "LP OPT" % hyperplanesExpr % "" % tmpObjLP % tmpAbsDiff % tmpRelDiff;

                Output::getInstance().outputSummary(tmpLine.str());

                currSol = externalPoint;
                prevSol = currSol;

                if (errorExternal.at(0).value <= termTol)
                {
                    statusCode = E_NLPSolutionStatus::Optimal;
                    break;
                }

                if (i == maxIter - 1)
                {
                    statusCode = E_NLPSolutionStatus::IterationLimit;
                    break;
                }
            }
            catch (std::exception &e)
            {

                Output::getInstance().outputWarning(
                    "     Cannot find solution with linesearch for fixed LP, using solution point instead:");
                Output::getInstance().outputWarning(e.what());
            }
        }
        else
        {

            currSol = LPVarSol;

            // Gets the most deviated constraints with a tolerance
            auto tmpMostDevs = NLPProblem->getMostDeviatingConstraints(currSol, constrSelTol);

            std::string hyperplanesExpr;

            hyperplanesExpr = "+" + to_string(numHyperAdded) + " = " + to_string(numHyperTot);

            std::string tmpObjLP = UtilityFunctions::toString(LPObjVar);

            boost::format tmpLine;
            std::string tmpAbsDiff = ""; //((boost::format("%.5f") % maxObjDiffAbs).str());
            std::string tmpRelDiff = ((boost::format("%.5f") % tmpMostDevs.at(0).value).str());
            tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % (i + 1) % "LP OPT" % hyperplanesExpr % "" % tmpObjLP % tmpAbsDiff % tmpRelDiff;

            Output::getInstance().outputSummary(tmpLine.str());

            if (tmpMostDevs.at(0).value <= termTol)
            {
                statusCode = E_NLPSolutionStatus::Optimal;

                break;
            }

            numHyperAdded = tmpMostDevs.size();
            numHyperTot = numHyperTot + numHyperAdded;

            for (int j = 0; j < numHyperAdded; j++)
            {
                Hyperplane hyperplane;
                hyperplane.sourceConstraintIndex = tmpMostDevs.at(j).idx;
                hyperplane.generatedPoint = currSol;
                hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

                LPSolver->createHyperplane(hyperplane);

                if (Settings::getInstance().getBoolSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual"))
                {
                    ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
                }
            }

            prevSol = currSol;

            if (i == maxIter - 1)
            {
                statusCode = E_NLPSolutionStatus::IterationLimit;
                break;
            }
        }
    }

    if (currSol.size() > 0 && Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
    {
        auto tmpVars = NLPProblem->getVariableNames();
        std::string filename = Settings::getInstance().getStringSetting("Debug.Path", "Output") + "/nlppoint" + to_string(ProcessInfo::getInstance().getCurrentIteration()->iterationNumber) + ".txt";
        UtilityFunctions::saveVariablePointVectorToFile(currSol, tmpVars, filename);
    }

    solution = LPVarSol;
    objectiveValue = LPObjVar;

    return (statusCode);
}

double NLPSolverCuttingPlaneRelaxed::getSolution(int i)
{
    return (solution.at(i));
}

std::vector<double> NLPSolverCuttingPlaneRelaxed::getSolution()
{
    auto tmpSol = solution;

    if (tmpSol.size() > 0 && (NLPProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Nonlinear || NLPProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic || NLPProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear))
    {
        tmpSol.pop_back();
    }

    return (tmpSol);
}

double NLPSolverCuttingPlaneRelaxed::getObjectiveValue()
{
    return (objectiveValue);
}

bool NLPSolverCuttingPlaneRelaxed::createProblemInstance(OSInstance *origInstance)
{
    Output::getInstance().outputInfo("Creating NLP problem for relaxed cutting plane solver");

    bool useQuadraticObjective = (static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QuadraticProblemStrategy::QuadraticObjective;

    bool useQuadraticConstraint = (static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QuadraticProblemStrategy::QuadraticallyConstrained;

    bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(originalInstance);
    bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(originalInstance);
    bool isQuadraticUsed = (useQuadraticObjective || (useQuadraticConstraint));

    auto solver = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

    if (solver == ES_MIPSolver::Cplex)
    {
        if (isObjNonlinear || (isObjQuadratic && !isQuadraticUsed))
        {
            NLPProblem = new OptProblemOriginalNonlinearObjective();
            dynamic_cast<OptProblemOriginalNonlinearObjective *>(NLPProblem)->setProblem(originalInstance);
        }
        else if (isObjQuadratic && isQuadraticUsed)
        {
            NLPProblem = new OptProblemOriginalQuadraticObjective();
            dynamic_cast<OptProblemOriginalQuadraticObjective *>(NLPProblem)->setProblem(originalInstance);
        }
        else
        {
            NLPProblem = new OptProblemOriginalLinearObjective();
            dynamic_cast<OptProblemOriginalLinearObjective *>(NLPProblem)->setProblem(originalInstance);
        }
    }
    else if (solver == ES_MIPSolver::Gurobi)
    {
    }
    else if (solver == ES_MIPSolver::Cbc)
    {
    }
    else
    {
        throw new ErrorClass("Error in solver definition for relaxed NLP.");
    }

    Output::getInstance().outputInfo("NLP problem for relaxed cutting plane created");

    Output::getInstance().outputInfo("Creating LP problem for relaxed cutting plane solver");
    LPSolver->createLinearProblem(NLPProblem);
    Output::getInstance().outputInfo("LP problem for relaxed cutting plane solver created");
    LPSolver->initializeSolverSettings();
    LPSolver->activateDiscreteVariables(false);

    return (true);
}

void NLPSolverCuttingPlaneRelaxed::fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
    LPSolver->fixVariables(variableIndexes, variableValues);
}

void NLPSolverCuttingPlaneRelaxed::unfixVariables()
{
    LPSolver->unfixVariables();
}

void NLPSolverCuttingPlaneRelaxed::setStartingPoint(std::vector<int> variableIndexes,
                                                    std::vector<double> variableValues)
{
}

bool NLPSolverCuttingPlaneRelaxed::isObjectiveFunctionNonlinear()
{
    return (NLPProblem->isObjectiveFunctionNonlinear());
}

int NLPSolverCuttingPlaneRelaxed::getObjectiveFunctionVariableIndex()
{
    return (NLPProblem->getNonlinearObjectiveVariableIdx());
}

std::vector<double> NLPSolverCuttingPlaneRelaxed::getCurrentVariableLowerBounds()
{
    return (NLPProblem->getVariableLowerBounds());
}

std::vector<double> NLPSolverCuttingPlaneRelaxed::getCurrentVariableUpperBounds()
{
    return (NLPProblem->getVariableUpperBounds());
}

void NLPSolverCuttingPlaneRelaxed::clearStartingPoint()
{
}

void NLPSolverCuttingPlaneRelaxed::saveOptionsToFile(std::string fileName)
{
}
