/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Results.h"

namespace SHOT
{

void Results::addDualSolution(DualSolution solution)
{
    if(dualSolutions.size() == 0)
    {
        dualSolutions.push_back(solution);
    }
    else
    {
        dualSolutions.at(0) = solution;
    }
}

void Results::addPrimalSolution(PrimalSolution solution)
{
    if(env->settings->getIntSetting("SaveNumberOfSolutions", "Output") > 1)
    {
        env->results->primalSolutions.insert(env->results->primalSolutions.begin(), solution);
    }
    else
    {
        if(env->results->primalSolutions.size() == 0)
        {
            env->results->primalSolutions.push_back(solution);
        }
        else
        {
            env->results->primalSolutions.at(0) = solution;
        }
    }

    env->results->primalSolution = solution.point;
    env->results->setPrimalBound(solution.objValue);

    // Write the new primal point to a file
    if(env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        std::stringstream fileName;
        fileName << env->settings->getStringSetting("Debug.Path", "Output");
        fileName << "/primalpoint";
        fileName << env->results->primalSolutions.size();
        fileName << ".txt";

        UtilityFunctions::savePrimalSolutionToFile(solution, env->problem->allVariables, fileName.str());
    }

    // Add primal objective cut
    if(env->settings->getBoolSetting("HyperplaneCuts.UsePrimalObjectiveCut", "Dual")
        && env->reformulatedProblem->objectiveFunction->properties.classification
            > E_ObjectiveFunctionClassification::Quadratic)
    {
        Hyperplane hyperplane;
        hyperplane.source = E_HyperplaneSource::PrimalSolutionSearchInteriorObjective;
        hyperplane.isObjectiveHyperplane = true;
        hyperplane.sourceConstraintIndex = -1;
        hyperplane.generatedPoint = solution.point;

        if(env->reformulatedProblem->objectiveFunction->properties.hasNonlinearExpression)
        {
            hyperplane.objectiveFunctionValue
                = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->calculateValue(hyperplane.generatedPoint);
        }
        else
        {
            hyperplane.objectiveFunctionValue
                = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->calculateValue(hyperplane.generatedPoint);
        }

        env->dualSolver->MIPSolver->hyperplaneWaitingList.push_back(hyperplane);

        auto tmpLine = boost::format("     Primal objective cut added.");

        env->output->outputCritical(tmpLine.str());
    }
}

bool Results::isRelativeObjectiveGapToleranceMet()
{
    if(this->getRelativeObjectiveGap() <= env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination"))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

bool Results::isAbsoluteObjectiveGapToleranceMet()
{
    if(this->getAbsoluteObjectiveGap() <= env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination"))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

Results::Results(EnvironmentPtr envPtr) : env(envPtr) {}

Results::~Results()
{
    iterations.clear();
    primalSolution.clear();
    primalSolutions.clear();
    dualSolutions.clear();
}

void Results::initializeResults(int numObj, int numVar, int numConstr)
{
    osResult = std::make_unique<OSResult>();
    osResult->setObjectiveNumber(numObj);
    osResult->setVariableNumber(numVar);
    osResult->setConstraintNumber(numConstr);
}

std::string Results::getOSrl()
{
    return "";
    int numConstr = osResult->getConstraintNumber();

    int numVar = osResult->getVariableNumber();

    int numPrimalSols = primalSolutions.size();

    std::stringstream ssSolver;
    ssSolver << "Supporting Hyperplane Optimization Toolkit, version ";
    ssSolver << SHOT_VERSION_MAJOR << "." << SHOT_VERSION_MINOR << "." << SHOT_VERSION_PATCH;
    osResult->setSolverInvoked(ssSolver.str());

    osResult->setInstanceName(env->settings->getStringSetting("ProblemName", "Input"));
    osResult->setNumberOfOtherGeneralResults(1);
    osResult->setOtherGeneralResultName(0, "UsedOptions");

    osResult->setOtherGeneralResultValue(0, env->settings->getSettingsAsString());

    if(numPrimalSols == 0)
    {
        osResult->setSolutionNumber(1);
        osResult->setNumberOfObjValues(0, 1);

        std::stringstream strstrdb;
        strstrdb << std::fixed << std::setprecision(15) << getDualBound();

        osResult->setAnOtherSolutionResult(
            0, "DualObjectiveBound", strstrdb.str(), "Final solution", "The dual bound for the objective", 0, NULL);

        if(dualSolutions.size() > 0 && dualSolutions.back().point.size() > 0)
        {
            osResult->setObjValue(0, 0, -1, "", dualSolutions.back().objValue);

            std::stringstream strstr;
            strstr << std::fixed << std::setprecision(15) << dualSolutions.back().objValue;

            osResult->setAnOtherSolutionResult(
                0, "MaxErrorConstrs", strstr.str(), "Final solution", "Maximal error in constraint", 0, NULL);
        }

        std::stringstream strstr2;
        strstr2 << std::fixed << std::setprecision(15) << getAbsoluteObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "AbsOptimalityGap", strstr2.str(), "Final solution",
            "The absolute optimality gap", 0, NULL);

        std::stringstream strstr3;
        strstr3 << std::fixed << std::setprecision(15) << getRelativeObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "RelOptimalityGap", strstr3.str(), "Final solution",
            "The relative optimality gap", 0, NULL);
    }
    else
    {
        int numSaveSolutions = env->settings->getIntSetting("SaveNumberOfSolutions", "Output");

        osResult->setSolutionNumber(numSaveSolutions);

        for(int i = 0; i < numSaveSolutions; i++)
        {
            if(i == 0)
            {
                std::string modelStatus;
                std::string modelStatusDescription;
                std::string modelSubStatus;
                std::string modelSubStatusDescription;

                osResult->setNumberOfSolutionSubstatuses(0, 1);

                if(this->primalSolutions.size() > 0)
                {
                    modelStatusDescription = "Feasible solution found";
                    modelStatus = "feasible";
                }
                else if(this->terminationReason == E_TerminationReason::AbsoluteGap)
                {
                    modelStatus = "globallyOptimal";
                    modelStatusDescription = "Solved to global optimality (assuming the problem is convex)";
                    modelSubStatus = "stoppedByBounds";
                    modelSubStatusDescription = "Objective gap fulfills absolute gap termination criterion";
                }
                else if(this->terminationReason == E_TerminationReason::RelativeGap)
                {
                    modelStatus = "globallyOptimal";
                    modelStatusDescription = "Solved to global optimality (assuming the problem is convex)";
                    modelSubStatus = "stoppedByBounds";
                    modelSubStatusDescription = "Objective gap fulfills relative gap termination criterion";
                }
                else if(this->terminationReason == E_TerminationReason::ConstraintTolerance)
                {
                    modelStatus = "locallyOptimal";
                    modelStatusDescription = "Solved to global optimality";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription
                        = "All nonlinear constraints fulfilled to given tolerance by dual solution";
                }
                else if(this->terminationReason == E_TerminationReason::InfeasibleProblem)
                {
                    modelStatus = "infeasible";
                    modelStatusDescription = "Problem may be infeasible or specified tolerances are too strict";
                }
                else if(this->terminationReason == E_TerminationReason::UnboundedProblem)
                {
                    modelStatus = "unbounded";
                    modelStatusDescription = "Problem is unbounded";
                }
                else if(this->terminationReason == E_TerminationReason::ObjectiveGapNotReached)
                {
                    modelStatusDescription = "Feasible solution found, but could not verify optimality";
                }
                else if(this->terminationReason == E_TerminationReason::ObjectiveStagnation)
                {
                    modelStatusDescription = "Terminated due to objective stagnation";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription = "Terminated due to objective stagnation";
                }
                else if(this->terminationReason == E_TerminationReason::IterationLimit)
                {
                    modelStatusDescription = "Terminated due to iteration limit";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription = "Terminated due to iteration limit";
                }
                else if(this->terminationReason == E_TerminationReason::TimeLimit)
                {
                    modelStatusDescription = "Terminated due to time limit";
                    modelSubStatus = "stoppedByLimit";
                    modelSubStatusDescription = "Terminated due to time limit";
                }
                else if(this->terminationReason == E_TerminationReason::NumericIssues)
                {
                    modelStatusDescription = "Terminated due to numeric issues";
                }
                else if(this->terminationReason == E_TerminationReason::UserAbort)
                {
                    modelStatusDescription = "User aborted solution process";
                }
                else if(this->terminationReason == E_TerminationReason::Error)
                {
                    modelStatusDescription = "Error during solution process";
                }
                else
                {
                    modelStatusDescription = "No feasible solutions found";
                    modelStatusDescription = "Unknown return code obtained from solver";
                    env->output->outputError("Unknown return code obtained from solver.");
                }

                osResult->setSolutionStatusType(0, modelStatus);
                osResult->setSolutionStatusDescription(0, modelStatusDescription);
                osResult->setSolutionSubstatusType(0, 0, modelSubStatus);
                osResult->setSolutionSubstatusDescription(0, 0, modelSubStatusDescription);
            }
            else
            {
                osResult->setSolutionStatusType(i, "feasible");
                osResult->setSolutionStatusDescription(i, "Additional primal solution");
            }

            osResult->setNumberOfObjValues(i, 1);
            osResult->setNumberOfPrimalVariableValues(i, numVar);
            osResult->setObjValue(i, 0, -1, "", primalSolutions.at(i).objValue);

            osResult->setPrimalVariableValuesDense(i, &primalSolutions.at(i).point[0]);

            VectorDouble tmpConstrVals;

            osResult->setNumberOfDualValues(i, numConstr);

            for(int j = 0; j < numConstr; j++)
            {
                osResult->setDualValue(i, j, j, env->problem->numericConstraints.at(j)->name,
                    env->problem->numericConstraints.at(j)
                        ->calculateNumericValue(primalSolutions.at(i).point)
                        .normalizedValue);
            }
        }

        std::stringstream strstrdb;
        strstrdb << std::fixed << std::setprecision(15) << getDualBound();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "DualObjectiveBound", strstrdb.str(), "Final solution",
            "The dual bound for the objective", 0, NULL);

        std::stringstream strstrpb;
        strstrpb << std::fixed << std::setprecision(15) << getPrimalBound();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "PrimalObjectiveBound", strstrpb.str(), "Final solution",
            "The primal bound for the objective", 0, NULL);

        std::stringstream strstr;
        strstr << std::fixed << std::setprecision(15) << getCurrentIteration()->maxDeviation;

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "MaxErrorConstrs", strstr.str(), "Final solution",
            "Maximal error in constraint", 0, NULL);

        std::stringstream strstr2;
        strstr2 << std::fixed << std::setprecision(15) << getAbsoluteObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "AbsOptimalityGap", strstr2.str(), "Final solution",
            "The absolute optimality gap", 0, NULL);

        std::stringstream strstr3;
        strstr3 << std::fixed << std::setprecision(15) << getRelativeObjectiveGap();

        osResult->setAnOtherSolutionResult(numPrimalSols - 1, "RelOptimalityGap", strstr3.str(), "Final solution",
            "The relative optimality gap", 0, NULL);
    }

    for(auto& T : env->timing->timers)
    {
        osResult->addTimingInformation(T.name, "SHOT", "second", T.description, T.elapsed());
    }

    numPrimalSols
        = std::max(1, numPrimalSols); // To make sure we also print the following even if we have no primal solution

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "LP",
        std::to_string(env->solutionStatistics.numberOfProblemsLP), "ProblemsSolved", "Relaxed LP problems solved", 0,
        NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "QP",
        std::to_string(env->solutionStatistics.numberOfProblemsQP), "ProblemsSolved", "Relaxed QP problems solved", 0,
        NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMILP",
        std::to_string(env->solutionStatistics.numberOfProblemsFeasibleMILP), "ProblemsSolved",
        "MILP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMILP",
        std::to_string(env->solutionStatistics.numberOfProblemsOptimalMILP), "ProblemsSolved",
        "MILP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "FeasibleMIQP",
        std::to_string(env->solutionStatistics.numberOfProblemsFeasibleMIQP), "ProblemsSolved",
        "MIQP problems solved to feasibility", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "OptimalMIQP",
        std::to_string(env->solutionStatistics.numberOfProblemsOptimalMIQP), "ProblemsSolved",
        "MIQP problems solved to optimality", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Total",
        std::to_string(env->solutionStatistics.numberOfProblemsLP + env->solutionStatistics.numberOfProblemsFeasibleMILP
            + env->solutionStatistics.numberOfProblemsOptimalMILP + env->solutionStatistics.numberOfProblemsQP
            + env->solutionStatistics.numberOfProblemsFeasibleMIQP
            + env->solutionStatistics.numberOfProblemsOptimalMIQP),
        "ProblemsSolved", "Total number of (MI)QP/(MI)LP subproblems solved", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NLP",
        std::to_string(env->solutionStatistics.getNumberOfTotalNLPProblems()), "ProblemsSolved", "NLP problems solved",
        0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Functions",
        std::to_string(env->solutionStatistics.numberOfFunctionEvalutions), "Evaluations",
        "Total number of function evaluations in SHOT", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "Gradients",
        std::to_string(env->solutionStatistics.numberOfGradientEvaluations), "Evaluations",
        "Total number of gradient evaluations in SHOT", 0, NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberVariables",
        std::to_string(env->problem->properties.numberOfVariables), "Problem", "Total number of variables", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberContinousVariables",
        std::to_string(env->problem->properties.numberOfRealVariables), "Problem", "Number of continuous variables", 0,
        NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberBinaryVariables",
        std::to_string(env->problem->properties.numberOfBinaryVariables), "Problem", "Number of binary variables", 0,
        NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberIntegerVariables",
        std::to_string(env->problem->properties.numberOfIntegerVariables), "Problem", "Number of integer variables", 0,
        NULL);

    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberConstraints",
        std::to_string(env->problem->properties.numberOfNumericConstraints), "Problem", "Number of constraints", 0,
        NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberNonlinearConstraints",
        std::to_string(env->problem->properties.numberOfNonlinearConstraints), "Problem",
        "Number of nonlinear constraints", 0, NULL);
    osResult->setAnOtherSolutionResult(numPrimalSols - 1, "NumberLinearConstraints",
        std::to_string(env->problem->properties.numberOfLinearConstraints), "Problem", "Number of linear constraints",
        0, NULL);

    OSrLWriter writer;
    writer.m_bWhiteSpace = false;

    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);

    std::stringstream ssOSrL;
    ssOSrL << writer.writeOSrL(osResult.get());

    read_xml(ssOSrL, pt, boost::property_tree::xml_parser::trim_whitespace);

    std::ostringstream ossXML;
    write_xml(ossXML, pt, settings);

    return (ossXML.str());
}

std::string Results::getTraceResult()
{
    std::stringstream ss;
    ss << env->problem->name << ",";

    switch(static_cast<E_SolutionStrategy>(env->results->usedSolutionStrategy))
    {
    case(E_SolutionStrategy::MIQP):
        ss << "MINLP";
        break;
    case(E_SolutionStrategy::MIQCQP):
        ss << "MINLP";
        break;
    case(E_SolutionStrategy::NLP):
        ss << "NLP";
        break;
    default:
        ss << "MINLP";
        break;
    }

    ss << ",";
    ss << "SHOT"
       << ",";

    switch(static_cast<ES_PrimalNLPSolver>(env->results->usedPrimalNLPSolver))
    {
    case(ES_PrimalNLPSolver::None):
        ss << "NONE";
        break;
    case(ES_PrimalNLPSolver::CuttingPlane):
        ss << "SHOT";
        break;
    case(ES_PrimalNLPSolver::GAMS):
        ss << env->settings->getStringSetting("GAMS.NLP.Solver", "Subsolver");
        break;
    case(ES_PrimalNLPSolver::Ipopt):
        ss << "Ipopt";
        break;
    default:
        ss << "NONE";
        break;
    }

    ss << ",";

    switch(static_cast<ES_MIPSolver>(env->results->usedMIPSolver))
    {
    case(ES_MIPSolver::Cplex):
        ss << "CPLEX";
        break;
    case(ES_MIPSolver::Gurobi):
        ss << "GUROBI";
        break;
    case(ES_MIPSolver::Cbc):
        ss << "CBC";
        break;
    default:
        ss << "NONE";
        break;
    }

    ss << ",";

    ss << UtilityFunctions::toStringFormat(UtilityFunctions::getJulianFractionalDate(), "%.5f", false);
    ss << ",";
    ss << (env->problem->objectiveFunction->properties.isMinimize ? "0" : "1") << ",";
    ss << env->problem->properties.numberOfNumericConstraints << ",";
    ss << env->problem->properties.numberOfVariables << ",";
    ss << env->problem->properties.numberOfDiscreteVariables << ",";

    ss << '0' << ","; // TODO: Number of nonzeroes
    ss << '0' << ","; // TODO: Number of nonlinear nonzeroes
    ss << "1"
       << ",";

    std::string solverStatus = "";
    std::string modelStatus = "";

    bool isOptimal = false;

    if(this->terminationReason == E_TerminationReason::AbsoluteGap
        || this->terminationReason == E_TerminationReason::RelativeGap)
    {
        solverStatus = "1";
        isOptimal = true;
    }
    else if(this->terminationReason == E_TerminationReason::ObjectiveStagnation
        || this->terminationReason == E_TerminationReason::IterationLimit)
    {
        solverStatus = "2";
    }
    else if(this->terminationReason == E_TerminationReason::TimeLimit)
    {
        solverStatus = "3";
    }
    else if(this->terminationReason == E_TerminationReason::NumericIssues)
    {
        solverStatus = "5";
    }
    else if(this->terminationReason == E_TerminationReason::UserAbort)
    {
        solverStatus = "8";
    }
    else if(this->terminationReason == E_TerminationReason::Error)
    {
        solverStatus = "10";
    }
    else if(this->terminationReason == E_TerminationReason::InfeasibleProblem
        || this->terminationReason == E_TerminationReason::ConstraintTolerance
        || this->terminationReason == E_TerminationReason::ObjectiveGapNotReached
        || this->terminationReason == E_TerminationReason::UnboundedProblem)
    {
        solverStatus = "1";
    }
    else
    {
        solverStatus = "10";
        env->output->outputError(
            "Unknown return code " + std::to_string((int)this->terminationReason) + " obtained from solver.");
    }

    auto solStatus = this->getCurrentIteration()->solutionStatus;

    if(isOptimal)
    {
        modelStatus = "1";
    }
    else if(this->primalSolutions.size() > 0)
    {
        modelStatus = "8";
    }
    else if(solStatus == E_ProblemSolutionStatus::Unbounded)
    {
        modelStatus = "3";
    }
    else if(solStatus == E_ProblemSolutionStatus::Infeasible)
    {
        modelStatus = "4";
    }
    else if(solStatus == E_ProblemSolutionStatus::Feasible || solStatus == E_ProblemSolutionStatus::IterationLimit
        || solStatus == E_ProblemSolutionStatus::TimeLimit || solStatus == E_ProblemSolutionStatus::NodeLimit
        || solStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        modelStatus = "7";
    }
    else if(solStatus == E_ProblemSolutionStatus::Error || solStatus == E_ProblemSolutionStatus::Numeric
        || solStatus == E_ProblemSolutionStatus::CutOff || solStatus == E_ProblemSolutionStatus::Abort)
    {
        modelStatus = "12";
    }
    else
    {
        modelStatus = "NA";
        env->output->outputError("Unknown return code " + std::to_string((int)solStatus) + " from model solution.");
    }

    ss << modelStatus << ",";
    ss << solverStatus << ",";

    ss << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    ss << this->getPrimalBound() << ",";
    ;
    ss << this->getDualBound() << ",";
    ;
    ss << env->timing->getElapsedTime("Total") << ",";
    ss << env->solutionStatistics.numberOfIterations << ",";
    ss << "0"
       << ",";
    ss << env->solutionStatistics.numberOfExploredNodes << ",";
    ss << "#";

    return (ss.str());
}

void Results::createIteration() { iterations.push_back(std::make_shared<Iteration>(env)); }

IterationPtr Results::getCurrentIteration() { return (iterations.back()); }

IterationPtr Results::getPreviousIteration()
{
    if(iterations.size() > 1)
        return (iterations[iterations.size() - 2]);
    else
        throw ErrorClass("Only one iteration!");
}

double Results::getPrimalBound() { return (this->currentPrimalBound); }

void Results::setPrimalBound(double value)
{
    this->currentPrimalBound = value;
    env->dualSolver->cutOffToUse = value;
    env->dualSolver->useCutOff = true;
    env->solutionStatistics.numberOfIterationsWithPrimalStagnation = 0;
    env->solutionStatistics.lastIterationWithSignificantPrimalUpdate = iterations.size() - 1;
    env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect = 0;
    env->solutionStatistics.numberOfDualRepairsSinceLastPrimalUpdate = 0;
}

double Results::getDualBound() { return (this->currentDualBound); }

void Results::setDualBound(double value)
{
    this->currentDualBound = value;
    env->solutionStatistics.numberOfIterationsWithDualStagnation = 0;

    env->solutionStatistics.lastIterationWithSignificantDualUpdate = iterations.size() - 1;
}

double Results::getAbsoluteObjectiveGap()
{
    double gap = abs(getDualBound() - getPrimalBound());

    return (gap);
}

double Results::getRelativeObjectiveGap()
{
    double gap = abs(getDualBound() - getPrimalBound()) / ((1e-10) + abs(getPrimalBound()));

    return (gap);
}
} // namespace SHOT