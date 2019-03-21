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

        auto tmpLine = boost::format("        Primal objective cut added.");

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

std::string Results::getResultsOSrL()
{
    using namespace tinyxml2;

    XMLDocument osrlDocument;

    auto osrlNode = osrlDocument.NewElement("osrl");
    osrlNode->SetAttribute("xmlns", "os.optimizationservices.org");
    osrlNode->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    osrlNode->SetAttribute(
        "xmlns:schemaLocation", "os.optimizationservices.org http://www.optimizationservices.org/schemas/2.0/OSrL.xsd");

    osrlDocument.InsertFirstChild(osrlNode);

    auto generalNode = osrlDocument.NewElement("general");

    auto instanceNameNode = osrlDocument.NewElement("instanceName");
    instanceNameNode->SetText(env->settings->getStringSetting("ProblemName", "Input").c_str());
    generalNode->InsertFirstChild(instanceNameNode);

    std::stringstream ssSolver;
    ssSolver << "Supporting Hyperplane Optimization Toolkit, version ";
    ssSolver << SHOT_VERSION_MAJOR << "." << SHOT_VERSION_MINOR << "." << SHOT_VERSION_PATCH;
    auto solver = ssSolver.str();

    auto solverInvoked = osrlDocument.NewElement("solverInvoked");
    solverInvoked->SetText(solver.c_str());
    generalNode->InsertFirstChild(solverInvoked);

    auto otherResultsNode = osrlDocument.NewElement("otherResults");
    otherResultsNode->SetAttribute("numberOfOtherResults", "1");
    auto otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "UsedOptions");
    otherNode->SetText(env->settings->getSettingsAsString(false, true).c_str());
    otherResultsNode->InsertFirstChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "DualObjectiveBound");
    otherNode->SetAttribute("value", currentDualBound);
    otherNode->SetAttribute("description", "The dual bound for the objective");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "PrimalObjectiveBound");
    otherNode->SetAttribute("value", currentPrimalBound);
    otherNode->SetAttribute("description", "The primal bound for the objective");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "MaxConstraintError");
    otherNode->SetAttribute("value", getCurrentIteration()->maxDeviation);
    otherNode->SetAttribute("description", "The maximal constraint error");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "AbsoluteOptimalityGap");
    otherNode->SetAttribute("value", getAbsoluteObjectiveGap());
    otherNode->SetAttribute("description", "The absolute optimality gap");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "RelativeOptimalityGap");
    otherNode->SetAttribute("value", getRelativeObjectiveGap());
    otherNode->SetAttribute("description", "The relative optimality gap");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfLPProblems");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfProblemsLP);
    otherNode->SetAttribute("description", "The number of LP problems solved in the dual strategy");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfQPProblems");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfProblemsQP);
    otherNode->SetAttribute("description", "The number of QP problems solved in the dual strategy");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfFeasibleMILPProblems");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfProblemsFeasibleMILP);
    otherNode->SetAttribute("description", "The number of MILP problems solved to feasibility in the dual strategy");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfFeasibleMIQPProblems");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfProblemsFeasibleMIQP);
    otherNode->SetAttribute("description", "The number of MIQP problems solved to feasibility in the dual strategy");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfOptimalMILPProblems");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfProblemsOptimalMILP);
    otherNode->SetAttribute("description", "The number of MILP problems solved to optimality in the dual strategy");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfOptimalMIQPProblems");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfProblemsOptimalMIQP);
    otherNode->SetAttribute("description", "The number of MIQP problems solved to optimality in the dual strategy");
    otherResultsNode->InsertEndChild(otherNode);

    int totalNumberOfProblems = env->solutionStatistics.numberOfProblemsLP
        + env->solutionStatistics.numberOfProblemsFeasibleMILP + env->solutionStatistics.numberOfProblemsOptimalMILP
        + env->solutionStatistics.numberOfProblemsQP + env->solutionStatistics.numberOfProblemsFeasibleMIQP
        + env->solutionStatistics.numberOfProblemsOptimalMIQP;

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "TotalNumberOfDualProblems");
    otherNode->SetAttribute("value", totalNumberOfProblems);
    otherNode->SetAttribute("description", "The total number of problems solved in the dual strategy");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfNLPProblems");
    otherNode->SetAttribute("value", env->solutionStatistics.getNumberOfTotalNLPProblems());
    otherNode->SetAttribute("description", "The number of NLP problems solved in the primal strategy");
    otherResultsNode->InsertEndChild(otherNode);

    generalNode->InsertFirstChild(otherResultsNode);

    osrlNode->InsertFirstChild(generalNode);

    auto jobNode = osrlDocument.NewElement("job");

    auto timingInformationNode = osrlDocument.NewElement("timingInformation");
    int numberOfTimes = 0;

    for(auto& T : env->timing->timers)
    {
        auto timeNode = osrlDocument.NewElement("time");
        timeNode->SetText(T.elapsed());
        timeNode->SetAttribute("type", T.name.c_str());
        timeNode->SetAttribute("unit", "second");
        timeNode->SetAttribute("description", T.description.c_str());
        timingInformationNode->InsertEndChild(timeNode);
        numberOfTimes++;
    }

    timingInformationNode->SetAttribute("numberOfTimes", numberOfTimes);
    jobNode->InsertEndChild(timingInformationNode);

    osrlNode->InsertEndChild(jobNode);

    auto optimizationNode = osrlDocument.NewElement("optimization");
    optimizationNode->SetAttribute("numberOfSolutions", (int)primalSolutions.size());
    optimizationNode->SetAttribute("numberOfVariables", env->problem->properties.numberOfVariables);
    optimizationNode->SetAttribute("numberOfConstraints", env->problem->properties.numberOfNumericConstraints);
    optimizationNode->SetAttribute("numberOfObjectives", 1);

    auto solutionNode = osrlDocument.NewElement("solution");

    auto otherSolutionResultsNode = osrlDocument.NewElement("otherSolutionResults");

    int numPrimalSols = primalSolutions.size();

    int numSaveSolutions = std::min(env->settings->getIntSetting("SaveNumberOfSolutions", "Output"), numPrimalSols);

    auto statusNode = osrlDocument.NewElement("status");

    if(this->terminationReason == E_TerminationReason::AbsoluteGap
        || this->terminationReason == E_TerminationReason::RelativeGap)
    {
        statusNode->SetAttribute("type", "globallyOptimal");
        statusNode->SetAttribute("description", "Solved to global optimality (assuming the problem is convex)");

        statusNode->SetAttribute("numberOfSubstatuses", 1);

        auto substatusNode = osrlDocument.NewElement("substatus");
        substatusNode->SetAttribute("type", "stoppedByBounds");
        substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
        statusNode->InsertFirstChild(substatusNode);
    }
    else if(this->terminationReason == E_TerminationReason::ConstraintTolerance)
    {
        statusNode->SetAttribute("type", "locallyOptimal");
        statusNode->SetAttribute("description", "Solved to local optimality");

        statusNode->SetAttribute("numberOfSubstatuses", 1);

        auto substatusNode = osrlDocument.NewElement("substatus");
        substatusNode->SetAttribute("type", "stoppedByBounds");
        substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
        statusNode->InsertFirstChild(substatusNode);
    }
    else if(this->primalSolutions.size() > 0)
    {
        statusNode->SetAttribute("type", "feasible");
        statusNode->SetAttribute("description", "Feasible solution found");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "other");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::InfeasibleProblem)
    {
        statusNode->SetAttribute("type", "infeasible");
        statusNode->SetAttribute("description", "No solution found since problem is infeasible");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "other");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::UnboundedProblem)
    {
        statusNode->SetAttribute("type", "unbounded");
        statusNode->SetAttribute("description", "No solution found since problem is unbounded");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "other");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::ObjectiveGapNotReached)
    {
        statusNode->SetAttribute("type", "other");
        statusNode->SetAttribute("description", "No solution found");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::ObjectiveStagnation)
    {
        statusNode->SetAttribute("type", "other");
        statusNode->SetAttribute("description", "No solution found");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::IterationLimit)
    {
        statusNode->SetAttribute("type", "other");
        statusNode->SetAttribute("description", "No solution found");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::TimeLimit)
    {
        statusNode->SetAttribute("type", "other");
        statusNode->SetAttribute("description", "No solution found");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::NumericIssues)
    {
        statusNode->SetAttribute("type", "error");
        statusNode->SetAttribute("description", "No solution found since an error occured");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::UserAbort)
    {
        statusNode->SetAttribute("type", "other");
        statusNode->SetAttribute("description", "No solution found due to user abort");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else if(this->terminationReason == E_TerminationReason::Error)
    {
        statusNode->SetAttribute("type", "error");
        statusNode->SetAttribute("description", "No solution found since an error occured");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }
    }
    else
    {
        statusNode->SetAttribute("type", "other");
        statusNode->SetAttribute("description", "Unknown return code obtained from solver");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "stoppedByLimit");
            substatusNode->SetAttribute("description", terminationReasonDescription.c_str());
            statusNode->InsertFirstChild(substatusNode);
        }

        env->output->outputError("Unknown return code obtained from solver.");
    }

    solutionNode->InsertFirstChild(statusNode);

    for(int i = 0; i < numSaveSolutions; i++)
    {
        if(i > 0)
        {
            solutionNode = osrlDocument.NewElement("solution");
            statusNode = osrlDocument.NewElement("status");
            statusNode->SetAttribute("type", "feasible");
            statusNode->SetAttribute("description", "Additional primal solution");
            solutionNode->InsertFirstChild(statusNode);
        }

        auto objectivesNode = osrlDocument.NewElement("objectives");

        auto objectiveValueNode = osrlDocument.NewElement("values");
        objectiveValueNode->SetAttribute("numberOfObj", 1);

        auto objectiveSolutionNode = osrlDocument.NewElement("obj");
        objectiveSolutionNode->SetAttribute("idx", -1);
        objectiveSolutionNode->SetText(std::to_string(primalSolutions.at(i).objValue).c_str());
        objectiveValueNode->InsertFirstChild(objectiveSolutionNode);

        objectivesNode->InsertFirstChild(objectiveValueNode);
        solutionNode->InsertFirstChild(objectivesNode);

        auto variablesNode = osrlDocument.NewElement("variables");

        auto variableValueNode = osrlDocument.NewElement("values");
        variableValueNode->SetAttribute("numberOfVar", (int)primalSolutions.at(i).point.size());

        for(int j = 0; j < primalSolutions.at(i).point.size(); j++)
        {
            auto variableSolutionNode = osrlDocument.NewElement("var");
            variableSolutionNode->SetAttribute("idx", j);
            variableSolutionNode->SetAttribute("name", env->problem->allVariables.at(j)->name.c_str());
            variableSolutionNode->SetText(std::to_string(primalSolutions.at(i).point.at(j)).c_str());
            variableValueNode->InsertEndChild(variableSolutionNode);
        }

        variablesNode->InsertFirstChild(variableValueNode);
        solutionNode->InsertFirstChild(variablesNode);

        auto constraintsNode = osrlDocument.NewElement("constraints");

        auto dualValuesNode = osrlDocument.NewElement("dualValues");
        dualValuesNode->SetAttribute("numberOfCon", (int)env->problem->properties.numberOfNumericConstraints);

        for(int j = 0; j < env->problem->numericConstraints.size(); j++)
        {
            auto dualValueNode = osrlDocument.NewElement("con");
            dualValueNode->SetAttribute("idx", j);
            dualValueNode->SetAttribute("name", env->problem->numericConstraints.at(j)->name.c_str());
            dualValueNode->SetText(std::to_string(env->problem->numericConstraints.at(j)
                                                      ->calculateNumericValue(primalSolutions.at(i).point)
                                                      .normalizedValue)
                                       .c_str());
            dualValuesNode->InsertEndChild(dualValueNode);
        }

        constraintsNode->InsertFirstChild(dualValuesNode);
        solutionNode->InsertFirstChild(constraintsNode);

        optimizationNode->InsertEndChild(solutionNode);
    }

    osrlNode->InsertEndChild(optimizationNode);

    XMLPrinter printer;
    osrlDocument.Print(&printer);

    return (printer.CStr());
}

std::string Results::getResultsTrace()
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