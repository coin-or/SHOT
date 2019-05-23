/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Results.h"

#include <algorithm>
#include <limits>

#include "Iteration.h"
#include "Output.h"
#include "Results.h"
#include "Settings.h"
#include "Timing.h"
#include "Utilities.h"

#include "Model/ObjectiveFunction.h"
#include "Model/Problem.h"
#include "Model/Variables.h"

#include "DualSolver.h"
#include "MIPSolver/IMIPSolver.h"

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
    if(env->settings->getSetting<int>("SaveNumberOfSolutions", "Output") > 1)
    {
        env->results->primalSolutions.insert(env->results->primalSolutions.begin(), solution);
    }
    else
    {
        if(!env->results->hasPrimalSolution())
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
    env->solutionStatistics.numberOfFoundPrimalSolutions++;

    // Write the new primal point to a file
    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        std::stringstream fileName;
        fileName << env->settings->getSetting<std::string>("Debug.Path", "Output");
        fileName << "/primalpoint";
        fileName << env->results->primalSolutions.size();
        fileName << ".txt";

        savePrimalSolutionToFile(solution, env->problem->allVariables, fileName.str());
    }

    // TODO: Add primal objective cut
    /*if(env->settings->getSetting<bool>("HyperplaneCuts.UsePrimalObjectiveCut", "Dual")
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

        env->output->outputCritical("        Primal objective cut added.");
    }*/
}

bool Results::isRelativeObjectiveGapToleranceMet()
{
    if(this->getRelativeGlobalObjectiveGap()
        <= env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination"))
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
    if(this->getAbsoluteGlobalObjectiveGap()
        <= env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination"))
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
    instanceNameNode->SetText(env->settings->getSetting<std::string>("ProblemName", "Input").c_str());
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
    otherNode->SetAttribute("value", globalDualBound);
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
    otherNode->SetAttribute("value", getAbsoluteGlobalObjectiveGap());
    otherNode->SetAttribute("description", "The absolute optimality gap");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "RelativeOptimalityGap");
    otherNode->SetAttribute("value", getRelativeGlobalObjectiveGap());
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

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFound");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfFoundPrimalSolutions);
    otherNode->SetAttribute("description", "The number of primal solutions found");
    otherResultsNode->InsertEndChild(otherNode);

    for(auto& S : env->results->primalSolutionSourceStatistics)
    {
        otherNode = osrlDocument.NewElement("other");

        switch(S.first)
        {
        case E_PrimalSolutionSource::Rootsearch:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundRootSearch");
            otherNode->SetAttribute("description", "The number of primal solutions found with root search");
            break;
        case E_PrimalSolutionSource::RootsearchFixedIntegers:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundRootSearchFixedIntegers");
            otherNode->SetAttribute(
                "description", "The number of primal solutions found with root search and fixed integers");
            break;
        case E_PrimalSolutionSource::NLPFixedIntegers:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundNLPFixedIntegers");
            otherNode->SetAttribute(
                "description", "The number of primal solutions found by solving integer-fixed NLP problems");
            break;
        case E_PrimalSolutionSource::MIPSolutionPool:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundMIPSolutionPool");
            otherNode->SetAttribute("description", "The number of primal solutions found from the MIP solution pool");
            break;
        case E_PrimalSolutionSource::LPFixedIntegers:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundLPFixedIntegers");
            otherNode->SetAttribute(
                "description", "The number of primal solutions found by solving integer-fixed LP problems");
            break;
        case E_PrimalSolutionSource::LazyConstraintCallback:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundLazyCallback");
            otherNode->SetAttribute("description", "The number of primal solutions found in lazy constraint callback");
            break;
        case E_PrimalSolutionSource::HeuristicCallback:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundHeuristicCallback");
            otherNode->SetAttribute(
                "description", "The number of primal solutions found in heuristic constraint callback");
            break;
        case E_PrimalSolutionSource::IncumbentCallback:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundIncumbentCallback");
            otherNode->SetAttribute(
                "description", "The number of primal solutions found in incumbent constraint callback");
            break;
        default:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundOther");
            otherNode->SetAttribute("description", "The number of primal solutions found with unknown method");
            break;
        }

        otherNode->SetAttribute("value", S.second);
        otherResultsNode->InsertEndChild(otherNode);
    }

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

    int numSaveSolutions = std::min(env->settings->getSetting<int>("SaveNumberOfSolutions", "Output"), numPrimalSols);

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
    else if(hasPrimalSolution())
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
    else if(this->terminationReason == E_TerminationReason::NoDualCutsAdded)
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

    if(env->problem->properties.isLPProblem)
        ss << "LP";
    else if(env->problem->properties.isMILPProblem)
        ss << "MIP";
    else if(env->problem->properties.isQPProblem)
        ss << "QCP";
    else if(env->problem->properties.isQCQPProblem)
        ss << "QCP";
    else if(env->problem->properties.isMIQPProblem)
        ss << "MIQCP";
    else if(env->problem->properties.isMIQCQPProblem)
        ss << "MIQCP";
    else if(env->problem->properties.isNLPProblem)
        ss << "NLP";
    else if(env->problem->properties.isMINLPProblem)
        ss << "MINLP";
    else
        ss << "UNKNOWN";

    ss << ",";
    ss << "SHOT"
       << ",";

    switch(static_cast<ES_PrimalNLPSolver>(env->results->usedPrimalNLPSolver))
    {
    case(ES_PrimalNLPSolver::None):
        ss << "NONE";
        break;
    case(ES_PrimalNLPSolver::GAMS):
        ss << env->settings->getSetting<std::string>("GAMS.NLP.Solver", "Subsolver");
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

    ss << Utilities::toStringFormat(Utilities::getJulianFractionalDate(), "{:.5f}", false);
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

    // set solve status
    switch(this->terminationReason)
    {
    case E_TerminationReason::IterationLimit:
        solverStatus = "2";
        break;
    case E_TerminationReason::TimeLimit:
        solverStatus = "3";
        break;
    case E_TerminationReason::UserAbort:
        solverStatus = "8";
        break;
    case E_TerminationReason::InfeasibleProblem:
    case E_TerminationReason::UnboundedProblem:
    case E_TerminationReason::ConstraintTolerance:
    case E_TerminationReason::AbsoluteGap:
    case E_TerminationReason::RelativeGap:
    case E_TerminationReason::ObjectiveGapNotReached:
        solverStatus = "1";
        break;
    case E_TerminationReason::ObjectiveStagnation:
    case E_TerminationReason::NoDualCutsAdded:
    case E_TerminationReason::Error:
    case E_TerminationReason::NumericIssues:
        solverStatus = "10";
        break;
    case E_TerminationReason::None:
        solverStatus = "13";
        break;
    }

    // set model status
    switch(this->getModelReturnStatus())
    {
    case E_ModelReturnStatus::OptimalLocal:
        modelStatus = "2";
        break;
    case E_ModelReturnStatus::OptimalGlobal:
        modelStatus = "1";
        break;
    case E_ModelReturnStatus::FeasibleSolution:
        modelStatus = (env->problem->properties.isDiscrete) ? "8" : "7";
        break;
    case E_ModelReturnStatus::InfeasibleLocal:
        modelStatus = "5";
        break;
    case E_ModelReturnStatus::InfeasibleGlobal:
        modelStatus = "4";
        break;
    case E_ModelReturnStatus::Unbounded:
        modelStatus = "3";
        break;
    case E_ModelReturnStatus::UnboundedNoSolution:
        modelStatus = "18";
        break;
    case E_ModelReturnStatus::NoSolutionReturned:
        modelStatus = "14";
        break;
    case E_ModelReturnStatus::ErrorUnknown:
        modelStatus = "12";
        break;
    case E_ModelReturnStatus::None:
    case E_ModelReturnStatus::ErrorNoSolution:
        modelStatus = "13";
    };

    ss << modelStatus << ",";
    ss << solverStatus << ",";

    ss << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    ss << this->getPrimalBound() << ",";
    ;
    ss << this->getGlobalDualBound() << ",";
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
        throw Error("Only one iteration!");
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

double Results::getCurrentDualBound() { return (this->currentDualBound); }

double Results::getGlobalDualBound() { return (globalDualBound); }

void Results::setDualBound(double value)
{
    this->currentDualBound = value;

    if(this->solutionIsGlobal)
        this->globalDualBound = value;

    env->solutionStatistics.numberOfIterationsWithDualStagnation = 0;

    env->solutionStatistics.lastIterationWithSignificantDualUpdate = iterations.size() - 1;
}

double Results::getAbsoluteGlobalObjectiveGap()
{
    double gap = abs(getGlobalDualBound() - getPrimalBound());

    return (gap);
}

double Results::getRelativeGlobalObjectiveGap()
{
    double gap = abs(getGlobalDualBound() - getPrimalBound()) / ((1e-10) + abs(getPrimalBound()));

    return (gap);
}

double Results::getAbsoluteCurrentObjectiveGap()
{
    double gap = abs(getCurrentDualBound() - getPrimalBound());

    return (gap);
}

double Results::getRelativeCurrentObjectiveGap()
{
    double gap = abs(getCurrentDualBound() - getPrimalBound()) / ((1e-10) + abs(getPrimalBound()));

    return (gap);
}

E_ModelReturnStatus Results::getModelReturnStatus()
{
    if(isRelativeObjectiveGapToleranceMet() || isAbsoluteObjectiveGapToleranceMet())
    {
        return (solutionIsGlobal ? E_ModelReturnStatus::OptimalGlobal : E_ModelReturnStatus::OptimalLocal);
    }

    if(terminationReason == E_TerminationReason::UnboundedProblem)
        return (hasPrimalSolution() ? E_ModelReturnStatus::Unbounded : E_ModelReturnStatus::UnboundedNoSolution);

    if(terminationReason == E_TerminationReason::InfeasibleProblem)
        return (solutionIsGlobal ? E_ModelReturnStatus::InfeasibleGlobal : E_ModelReturnStatus::InfeasibleLocal);

    if(terminationReason == E_TerminationReason::Error || terminationReason == E_TerminationReason::NumericIssues)
        return (hasPrimalSolution() ? E_ModelReturnStatus::ErrorUnknown : E_ModelReturnStatus::ErrorNoSolution);

    if(hasPrimalSolution())
        return (E_ModelReturnStatus::FeasibleSolution);

    return (E_ModelReturnStatus::NoSolutionReturned);
}

bool Results::hasPrimalSolution() { return (primalSolutions.size() > 0); }

void Results::savePrimalSolutionToFile(
    const PrimalSolution& solution, const VectorString& variables, const std::string& fileName)
{
    std::stringstream str;

    str << "Source: " << solution.sourceDescription;
    str << '\n';

    str << "Iteration found: " << solution.iterFound;
    str << '\n';

    str << "Objective value: " << Utilities::toStringFormat(solution.objValue, "{:.8f}", false);
    str << '\n';

    str << "Largest nonlinear error (in constraint " << solution.maxDevatingConstraintNonlinear.index
        << "): " << Utilities::toStringFormat(solution.maxDevatingConstraintNonlinear.value, "{:.8f}", false);
    str << '\n';

    str << "Largest linear error (in constraint " << solution.maxDevatingConstraintLinear.index
        << "): " << Utilities::toStringFormat(solution.maxDevatingConstraintLinear.value, "{:.8f}", false);
    str << '\n';

    str << "Projection to variable bounds performed: " << (solution.boundProjectionPerformed ? "true" : "false");
    str << '\n';

    str << "Integer rounding performed: " << (solution.integerRoundingPerformed ? "true" : "false");
    str << '\n';

    str << "Max integer rounding error: "
        << Utilities::toStringFormat(solution.maxIntegerToleranceError, "{:.8f}", false);

    str << '\n';
    str << '\n';

    str << "Solution point: ";
    str << '\n';

    str << std::setprecision(std::numeric_limits<double>::digits10);

    for(int i = 0; i < solution.point.size(); i++)
    {
        str << variables.at(i);
        str << "\t";
        str << solution.point.at(i);
        str << '\n';
    }

    Utilities::writeStringToFile(fileName, str.str());
};

void Results::savePrimalSolutionToFile(
    const PrimalSolution& solution, const Variables& variables, const std::string& fileName)
{
    std::stringstream str;

    str << "Source: " << solution.sourceDescription;
    str << '\n';

    str << "Iteration found: " << solution.iterFound;
    str << '\n';

    str << "Objective value: " << Utilities::toStringFormat(solution.objValue, "{:.8f}", false);
    str << '\n';

    str << "Largest nonlinear error (in constraint " << solution.maxDevatingConstraintNonlinear.index
        << "): " << Utilities::toStringFormat(solution.maxDevatingConstraintNonlinear.value, "{:.8f}", false);
    str << '\n';

    str << "Largest linear error (in constraint " << solution.maxDevatingConstraintLinear.index
        << "): " << Utilities::toStringFormat(solution.maxDevatingConstraintLinear.value, "{:.8f}", false);
    str << '\n';

    str << "Projection to variable bounds performed: " << (solution.boundProjectionPerformed ? "true" : "false");
    str << '\n';

    str << "Integer rounding performed: " << (solution.integerRoundingPerformed ? "true" : "false");
    str << '\n';

    str << "Max integer rounding error: "
        << Utilities::toStringFormat(solution.maxIntegerToleranceError, "{:.8f}", false);

    str << '\n';
    str << '\n';

    str << "Solution point: ";
    str << '\n';

    str << std::setprecision(std::numeric_limits<double>::digits10);

    for(int i = 0; i < solution.point.size(); i++)
    {
        if(i < variables.size())
            str << variables.at(i)->name;
        else
            str << '\t';
        str << '\t';
        str << solution.point.at(i);
        str << '\n';
    }

    Utilities::writeStringToFile(fileName, str.str());
};

} // namespace SHOT