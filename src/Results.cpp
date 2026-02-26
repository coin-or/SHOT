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

#include "EventHandler.h"
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
    bool isDifferent = true;

    for(auto& S : this->primalSolutions)
    {
        if(Utilities::isDifferent(S.point, solution.point))
            continue;

        isDifferent = false;
        break;
    }

    if(!isDifferent) // The same solution point is already saved
    {
        env->output->outputDebug(fmt::format(
            "         Primal solution candidate with objective value {} already known.", solution.objValue));
        return;
    }

    if(this->primalSolutions.size() == 0)
    {
        // This is the first solution, save it
        this->primalSolutions.push_back(solution);
        this->primalSolution = solution.point;
        this->setPrimalBound(solution.objValue);

        env->output->outputDebug(fmt::format(
            "        First primal solution {} from {} found.", solution.objValue, solution.sourceDescription));
    }
    else if(auto primalsol = this->primalSolutions.back();
            (env->problem->objectiveFunction->properties.isMinimize && solution.objValue < primalsol.objValue)
            || (!env->problem->objectiveFunction->properties.isMinimize && solution.objValue > primalsol.objValue))
    {
        // Have a solution which is better than the worst one in the solution pool
        this->primalSolutions.back() = solution;
        this->primalSolution = solution.point;
        this->setPrimalBound(solution.objValue);

        env->output->outputDebug(fmt::format("        New (currently best) primal solution {} from {} found.",
            solution.objValue, solution.sourceDescription));
    }
    else if(Utilities::isAlmostEqual(solution.objValue, primalsol.objValue, 1e-10)
        && (std::max({ solution.maxDevatingConstraintLinear.value, solution.maxDevatingConstraintQuadratic.value,
                solution.maxDevatingConstraintNonlinear.value })
            < std::max({ primalsol.maxDevatingConstraintLinear.value, primalsol.maxDevatingConstraintQuadratic.value,
                primalsol.maxDevatingConstraintNonlinear.value })))
    {
        // Have a solution which is similar to the best known, but with smaller constraint error
        this->primalSolutions.back() = solution;
        this->primalSolution = solution.point;
        this->setPrimalBound(solution.objValue);

        env->output->outputDebug(fmt::format("        New (currently best) primal solution {} from {} found.",
            solution.objValue, solution.sourceDescription));
    }
    else if((int)this->primalSolutions.size() < env->settings->getSetting<int>("SaveNumberOfSolutions", "Output"))
    {
        // The solution pool is not yet full, save the solution
        this->primalSolutions.push_back(solution);

        env->output->outputDebug(fmt::format("        New primal solution {} from {} found and added to solution pool.",
            solution.objValue, solution.sourceDescription));
    }
    else
    {
        env->output->outputDebug(fmt::format(
            "        Primal solution {} from {} is not an improvement of the current value {} or the solution "
            "pool is full, so it will not be saved.",
            solution.objValue, solution.sourceDescription, primalsol.objValue));
        // Will not save this solution
        return;
    }

    // Sorts the solutions so that the best one is at the first position
    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        std::sort(this->primalSolutions.begin(), this->primalSolutions.end(),
            [](const PrimalSolution& firstSolution, const PrimalSolution& secondSolution) {
                return (firstSolution.objValue < secondSolution.objValue);
            });
    }
    else
    {
        std::sort(this->primalSolutions.begin(), this->primalSolutions.end(),
            [](const PrimalSolution& firstSolution, const PrimalSolution& secondSolution) {
                return (firstSolution.objValue > secondSolution.objValue);
            });
    }

    env->solutionStatistics.numberOfFoundPrimalSolutions++;

    if(env->solutionStatistics.hasInfeasibilityRepairBeenPerformedSincePrimalImprovement)
    {
        env->solutionStatistics.numberOfPrimalImprovementsAfterInfeasibilityRepair++;
        env->solutionStatistics.hasInfeasibilityRepairBeenPerformedSincePrimalImprovement = false;
    }

    if(env->solutionStatistics.hasReductionCutBeenAddedSincePrimalImprovement)
    {
        env->solutionStatistics.numberOfPrimalImprovementsAfterReductionCut++;
        env->solutionStatistics.hasReductionCutBeenAddedSincePrimalImprovement = false;
    }

    // Saves statistics for the sources of primal solutions
    auto element = this->primalSolutionSourceStatistics.emplace(solution.sourceType, 1);

    if(!element.second)
    {
        // Element already exists
        element.first->second += 1;
    }

    // Write the new primal point to a file
    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        auto filename
            = fmt::format("{}/primal_solpt{}.txt", env->settings->getSetting<std::string>("Debug.Path", "Output"),
                env->solutionStatistics.numberOfFoundPrimalSolutions);

        savePrimalSolutionToFile(solution, env->problem->allVariables, filename);
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
                =
    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->calculateValue(hyperplane.generatedPoint);
        }
        else
        {
            hyperplane.objectiveFunctionValue
                =
    std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->calculateValue(hyperplane.generatedPoint);
        }

        env->dualSolver->hyperplaneWaitingList.push_back(hyperplane);

        env->output->outputCritical("        Primal objective cut added.");
    }*/

    env->events->notify(E_EventType::NewPrimalSolution);
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

Results::Results(EnvironmentPtr envPtr) : env(envPtr) { }

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

    // Problem classification and type information
    bool isReformulated = (env->problem != env->reformulatedProblem);

    // Original problem classification
    std::string problemClassificationOrig;
    if(env->problem->properties.isLPProblem)
        problemClassificationOrig = "LP";
    else if(env->problem->properties.isMILPProblem)
        problemClassificationOrig = "MILP";
    else if(env->problem->properties.isQPProblem)
        problemClassificationOrig = "QP";
    else if(env->problem->properties.isQCQPProblem)
        problemClassificationOrig = "QCQP";
    else if(env->problem->properties.isMIQPProblem)
        problemClassificationOrig = "MIQP";
    else if(env->problem->properties.isMIQCQPProblem)
        problemClassificationOrig = "MIQCQP";
    else if(env->problem->properties.isNLPProblem)
        problemClassificationOrig = "NLP";
    else if(env->problem->properties.isMINLPProblem)
        problemClassificationOrig = "MINLP";

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalProblemClassification");
    otherNode->SetAttribute("value", problemClassificationOrig.c_str());
    otherNode->SetAttribute("description", "Classification of the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    if(env->problem->properties.convexity == E_ProblemConvexity::Convex)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "OriginalProblemConvexity");
        otherNode->SetAttribute("value", "convex");
        otherNode->SetAttribute("description", "Convexity of the original problem");
        otherResultsNode->InsertEndChild(otherNode);
    }
    else if(env->problem->properties.convexity == E_ProblemConvexity::Nonconvex)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "OriginalProblemConvexity");
        otherNode->SetAttribute("value", "nonconvex");
        otherNode->SetAttribute("description", "Convexity of the original problem");
        otherResultsNode->InsertEndChild(otherNode);
    }

    // Reformulated problem classification
    if(isReformulated)
    {
        std::string problemClassificationRef;
        if(env->reformulatedProblem->properties.isLPProblem)
            problemClassificationRef = "LP";
        else if(env->reformulatedProblem->properties.isMILPProblem)
            problemClassificationRef = "MILP";
        else if(env->reformulatedProblem->properties.isQPProblem)
            problemClassificationRef = "QP";
        else if(env->reformulatedProblem->properties.isQCQPProblem)
            problemClassificationRef = "QCQP";
        else if(env->reformulatedProblem->properties.isMIQPProblem)
            problemClassificationRef = "MIQP";
        else if(env->reformulatedProblem->properties.isMIQCQPProblem)
            problemClassificationRef = "MIQCQP";
        else if(env->reformulatedProblem->properties.isNLPProblem)
            problemClassificationRef = "NLP";
        else if(env->reformulatedProblem->properties.isMINLPProblem)
            problemClassificationRef = "MINLP";

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedProblemClassification");
        otherNode->SetAttribute("value", problemClassificationRef.c_str());
        otherNode->SetAttribute("description", "Classification of the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        if(env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
        {
            otherNode = osrlDocument.NewElement("other");
            otherNode->SetAttribute("name", "ReformulatedProblemConvexity");
            otherNode->SetAttribute("value", "convex");
            otherNode->SetAttribute("description", "Convexity of the reformulated problem");
            otherResultsNode->InsertEndChild(otherNode);
        }
        else if(env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Nonconvex)
        {
            otherNode = osrlDocument.NewElement("other");
            otherNode->SetAttribute("name", "ReformulatedProblemConvexity");
            otherNode->SetAttribute("value", "nonconvex");
            otherNode->SetAttribute("description", "Convexity of the reformulated problem");
            otherResultsNode->InsertEndChild(otherNode);
        }
    }

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalProblemObjectiveDirection");
    otherNode->SetAttribute("value", env->problem->objectiveFunction->properties.isMinimize ? "minimize" : "maximize");
    otherNode->SetAttribute("description", "Whether the original problem is a minimization or maximization problem");
    otherResultsNode->InsertEndChild(otherNode);

    // Original problem sizes
    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfVariables");
    otherNode->SetAttribute("value", env->problem->properties.numberOfVariables);
    otherNode->SetAttribute("description", "Number of variables in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfRealVariables");
    otherNode->SetAttribute("value", env->problem->properties.numberOfRealVariables);
    otherNode->SetAttribute("description", "Number of real variables in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfBinaryVariables");
    otherNode->SetAttribute("value", env->problem->properties.numberOfBinaryVariables);
    otherNode->SetAttribute("description", "Number of binary variables in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfIntegerVariables");
    otherNode->SetAttribute("value", env->problem->properties.numberOfIntegerVariables);
    otherNode->SetAttribute("description", "Number of integer variables in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfConstraints");
    otherNode->SetAttribute("value", env->problem->properties.numberOfNumericConstraints);
    otherNode->SetAttribute("description", "Number of constraints in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfLinearConstraints");
    otherNode->SetAttribute("value",
        env->problem->properties.numberOfLinearConstraints - env->problem->properties.numberOfAddedLinearizations);
    otherNode->SetAttribute("description", "Number of linear constraints in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfConvexQuadraticConstraints");
    otherNode->SetAttribute("value", env->problem->properties.numberOfConvexQuadraticConstraints);
    otherNode->SetAttribute("description", "Number of convex quadratic constraints in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfNonconvexQuadraticConstraints");
    otherNode->SetAttribute("value", env->problem->properties.numberOfNonconvexQuadraticConstraints);
    otherNode->SetAttribute("description", "Number of nonconvex quadratic constraints in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfConvexNonlinearConstraints");
    otherNode->SetAttribute("value", env->problem->properties.numberOfConvexNonlinearConstraints);
    otherNode->SetAttribute("description", "Number of convex nonlinear constraints in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "OriginalNumberOfNonconvexNonlinearConstraints");
    otherNode->SetAttribute("value", env->problem->properties.numberOfNonconvexNonlinearConstraints);
    otherNode->SetAttribute("description", "Number of nonconvex nonlinear constraints in the original problem");
    otherResultsNode->InsertEndChild(otherNode);

    // Reformulated problem sizes
    if(isReformulated)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedProblemObjectiveDirection");
        otherNode->SetAttribute(
            "value", env->reformulatedProblem->objectiveFunction->properties.isMinimize ? "minimize" : "maximize");
        otherNode->SetAttribute(
            "description", "Whether the reformulated problem is a minimization or maximization problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfVariables");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfVariables);
        otherNode->SetAttribute("description", "Number of variables in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfRealVariables");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfRealVariables);
        otherNode->SetAttribute("description", "Number of real variables in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfBinaryVariables");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfBinaryVariables);
        otherNode->SetAttribute("description", "Number of binary variables in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfIntegerVariables");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfIntegerVariables);
        otherNode->SetAttribute("description", "Number of integer variables in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfConstraints");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfNumericConstraints);
        otherNode->SetAttribute("description", "Number of constraints in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfLinearConstraints");
        otherNode->SetAttribute("value",
            env->reformulatedProblem->properties.numberOfLinearConstraints
                - env->reformulatedProblem->properties.numberOfAddedLinearizations);
        otherNode->SetAttribute("description", "Number of linear constraints in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfConvexQuadraticConstraints");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfConvexQuadraticConstraints);
        otherNode->SetAttribute("description", "Number of convex quadratic constraints in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfNonconvexQuadraticConstraints");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfNonconvexQuadraticConstraints);
        otherNode->SetAttribute("description", "Number of nonconvex quadratic constraints in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfConvexNonlinearConstraints");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfConvexNonlinearConstraints);
        otherNode->SetAttribute("description", "Number of convex nonlinear constraints in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);

        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "ReformulatedNumberOfNonconvexNonlinearConstraints");
        otherNode->SetAttribute("value", env->reformulatedProblem->properties.numberOfNonconvexNonlinearConstraints);
        otherNode->SetAttribute("description", "Number of nonconvex nonlinear constraints in the reformulated problem");
        otherResultsNode->InsertEndChild(otherNode);
    }

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
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfProblemsFixedNLP);
    otherNode->SetAttribute("description", "The number of NLP problems solved in the primal strategy");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFound");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfFoundPrimalSolutions);
    otherNode->SetAttribute("description", "The number of primal solutions found");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfSuccesfulInfeasibilityRepairsPerformed");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfSuccessfulDualRepairsPerformed);
    otherNode->SetAttribute(
        "description", "The number of sucessful infeasibility repairs performed for nonconvex problems");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfUnsuccesfulInfeasibilityRepairsPerformed");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfUnsuccessfulDualRepairsPerformed);
    otherNode->SetAttribute(
        "description", "The number of unsucessful infeasibility repairs performed for nonconvex problems");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfReductionCutStepsPerformed");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfPrimalReductionsPerformed);
    otherNode->SetAttribute("description", "The number of reduction cut steps performed for nonconvex problems");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfPrimalImprovementsAfterInfeasibilityRepair");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfPrimalImprovementsAfterInfeasibilityRepair);
    otherNode->SetAttribute("description",
        "The number of cases where the repairing of infeasibilities for nonconvex problems has directly resulted in "
        "improved primal solutions");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "NumberOfPrimalImprovementsAfterReductionCut");
    otherNode->SetAttribute("value", env->solutionStatistics.numberOfPrimalImprovementsAfterReductionCut);
    otherNode->SetAttribute("description",
        "The number of cases where the primal reduction cut has directly resulted in improved primal solutions");
    otherResultsNode->InsertEndChild(otherNode);

    auto dualSolver = static_cast<ES_MIPSolver>(env->settings->getSetting<int>("MIP.Solver", "Dual"));
    std::string dualSolverName;

#ifdef HAS_CPLEX
    if(dualSolver == ES_MIPSolver::Cplex)
    {
        dualSolverName = "CPLEX";
    }
#endif

#ifdef HAS_GUROBI
    if(dualSolver == ES_MIPSolver::Gurobi)
    {
        dualSolverName = "Gurobi";
    }
#endif

#ifdef HAS_CBC
    if(dualSolver == ES_MIPSolver::Cbc)
    {
        dualSolverName = "Cbc";
    }
#endif

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "DualSolver");
    otherNode->SetAttribute("value", (dualSolverName + " " + env->dualSolver->MIPSolver->getSolverVersion()).c_str());
    otherNode->SetAttribute("description", "The dual solver used");
    otherResultsNode->InsertEndChild(otherNode);

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "FixedNLPSolver");
    otherNode->SetAttribute("value", (dualSolverName + " " + env->dualSolver->MIPSolver->getSolverVersion()).c_str());
    otherNode->SetAttribute("description", "The dual solver used");
    otherResultsNode->InsertEndChild(otherNode);

    for(auto& S : this->primalSolutionSourceStatistics)
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
        case E_PrimalSolutionSource::MIPCallback:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundMIPCallback");
            otherNode->SetAttribute("description", "The number of primal solutions found in MIP callbacks");
            break;
        case E_PrimalSolutionSource::InteriorPointSearch:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundInteriorPointSearch");
            otherNode->SetAttribute(
                "description", "The number of primal solutions found when searching for interior point");
            break;
        default:
            otherNode->SetAttribute("name", "NumberOfPrimalSolutionsFoundOther");
            otherNode->SetAttribute("description", "The number of primal solutions found with unknown method");
            break;
        }

        otherNode->SetAttribute("value", S.second);
        otherResultsNode->InsertEndChild(otherNode);
    }

    // Add reformulation statistics
    int totalReformulations = 0;
    for(auto& [type, count] : auxiliaryVariablesIntroduced)
    {
        totalReformulations += count;
    }

    if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
        totalReformulations++;

    otherNode = osrlDocument.NewElement("other");
    otherNode->SetAttribute("name", "TotalNumberOfReformulations");
    otherNode->SetAttribute("value", totalReformulations);
    otherNode->SetAttribute("description", "The total number of reformulations performed");
    otherResultsNode->InsertEndChild(otherNode);

    // Add breakdown per reformulation type
    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearObjectiveFunction); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfEpigraphReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Epigraph formulation of nonlinear objective function");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::NonlinearExpressionPartitioning); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfNonlinearExpressionPartitionings");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Nonlinear expression partitioning reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::MonomialTermsPartitioning); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfMonomialTermsPartitionings");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Monomial terms partitioning reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::SignomialTermsPartitioning); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfSignomialTermsPartitionings");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Signomial terms partitioning reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::SquareTermsPartitioning); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfSquareTermsPartitionings");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Square terms partitioning reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::ContinuousBilinear); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfContinuousBilinearReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Continuous bilinear term reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryBilinear); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfBinaryBilinearReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Binary bilinear term reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryContinuousBilinear); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfBinaryContinuousBilinearReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Binary-continuous bilinear term reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::IntegerBilinear); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfIntegerBilinearReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Integer bilinear term reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::BinaryMonomial); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfBinaryMonomialReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Binary monomial term reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::AbsoluteValue); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfAbsoluteValueReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Absolute value reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::AntiEpigraph); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfAntiEpigraphReformulations");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Anti-epigraph reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::EigenvalueDecomposition); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfEigenvalueDecompositions");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "Eigenvalue decomposition reformulations");
        otherResultsNode->InsertEndChild(otherNode);
    }

    if(auto count = getAuxiliaryVariableCounter(E_AuxiliaryVariableType::LDLDecomposition); count > 0)
    {
        otherNode = osrlDocument.NewElement("other");
        otherNode->SetAttribute("name", "NumberOfLDLDecompositions");
        otherNode->SetAttribute("value", count);
        otherNode->SetAttribute("description", "LDL decomposition reformulations");
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
    optimizationNode->SetAttribute("numberOfConstraints",
        env->problem->properties.numberOfNumericConstraints - env->problem->properties.numberOfAddedLinearizations);
    optimizationNode->SetAttribute("numberOfObjectives", 1);

    auto solutionNode = osrlDocument.NewElement("solution");

    int numPrimalSols = primalSolutions.size();

    int numSaveSolutions = std::min(env->settings->getSetting<int>("SaveNumberOfSolutions", "Output"), numPrimalSols);

    auto statusNode = osrlDocument.NewElement("status");

    if(this->terminationReason == E_TerminationReason::AbsoluteGap
        || this->terminationReason == E_TerminationReason::RelativeGap)
    {
        statusNode->SetAttribute("type", "globallyOptimal");
        statusNode->SetAttribute("description", "Solved to global optimality");

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
        statusNode->SetAttribute("description", "No solution found since dual problem is infeasible");

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
        statusNode->SetAttribute("description", "No solution found since dual problem is unbounded");

        if(terminationReasonDescription != "")
        {
            statusNode->SetAttribute("numberOfSubstatuses", 1);

            auto substatusNode = osrlDocument.NewElement("substatus");
            substatusNode->SetAttribute("type", "other");
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

        env->output->outputError(
            fmt::format(" Unknown return code {} obtained from solver.", static_cast<int>(this->terminationReason)));
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

        for(size_t j = 0; j < primalSolutions.at(i).point.size(); j++)
        {
            auto variableSolutionNode = osrlDocument.NewElement("var");
            variableSolutionNode->SetAttribute("idx", (int)j);
            variableSolutionNode->SetAttribute("name", env->problem->allVariables.at(j)->name.c_str());
            variableSolutionNode->SetText(std::to_string(primalSolutions.at(i).point.at(j)).c_str());
            variableValueNode->InsertEndChild(variableSolutionNode);
        }

        variablesNode->InsertFirstChild(variableValueNode);
        solutionNode->InsertFirstChild(variablesNode);

        auto constraintsNode = osrlDocument.NewElement("constraints");

        auto dualValuesNode = osrlDocument.NewElement("dualValues");
        dualValuesNode->SetAttribute("numberOfCon", (int)env->problem->properties.numberOfNumericConstraints);

        for(size_t j = 0; j < env->problem->numericConstraints.size(); j++)
        {
            auto dualValueNode = osrlDocument.NewElement("con");
            dualValueNode->SetAttribute("idx", (int)j);
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

    switch(static_cast<ES_PrimalNLPSolver>(this->usedPrimalNLPSolver))
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

    switch(static_cast<ES_MIPSolver>(this->usedMIPSolver))
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
    ss << env->problem->properties.numberOfNumericConstraints - env->problem->properties.numberOfAddedLinearizations
       << ",";
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
    case E_TerminationReason::ObjectiveStagnation:
    case E_TerminationReason::NoDualCutsAdded:
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
        solverStatus = "1";
        break;
    case E_TerminationReason::Error:
    case E_TerminationReason::NumericIssues:
        solverStatus = "10";
        break;
    case E_TerminationReason::None:
        solverStatus = "1";
        break;
    }

    // set model status
    switch(this->getModelReturnStatus())
    {
    /* case E_ModelReturnStatus::OptimalLocal:
        modelStatus = "2";
        break;*/
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

std::string Results::getResultsSol()
{
    std::string status = "";
    std::string description = "";

    if(this->terminationReason == E_TerminationReason::AbsoluteGap
        || this->terminationReason == E_TerminationReason::RelativeGap)
    {
        status = "0";
        description = "Solved to global optimality";
    }
    else if(this->hasPrimalSolution())
    {
        status = "100";
        description = "Solved to local optimality";
    }
    else if(this->terminationReason == E_TerminationReason::InfeasibleProblem)
    {
        status = "200";
        description = "No solution found since dual problem is infeasible";
    }
    else if(this->terminationReason == E_TerminationReason::UnboundedProblem)
    {
        status = "300";
        description = "No solution found since dual problem is unbounded";
    }
    else if(this->terminationReason == E_TerminationReason::ObjectiveStagnation)
    {
        status = "400";
        description = "No solution found";
    }
    else if(this->terminationReason == E_TerminationReason::NoDualCutsAdded)
    {
        status = "400";
        description = "No solution found";
    }
    else if(this->terminationReason == E_TerminationReason::IterationLimit)
    {
        status = "400";
        description = "No solution found";
    }
    else if(this->terminationReason == E_TerminationReason::TimeLimit)
    {
        status = "400";
        description = "No solution found";
    }
    else if(this->terminationReason == E_TerminationReason::NumericIssues)
    {
        status = "500";
        description = "No solution found since an error occured";
    }
    else if(this->terminationReason == E_TerminationReason::UserAbort)
    {
        status = "600";
        description = "No solution found due to user abort";
    }
    else if(this->terminationReason == E_TerminationReason::Error)
    {
        status = "500";
        description = "No solution found since an error occured";
    }
    else
    {
        status = "500";
        description = "No solution found since an error occured";
    }

    std::stringstream ss;

    ss << fmt::format("SHOT: {}\n", description);

    ss << "\nOptions\n";

    ss << env->settings->getSetting<std::string>("AMPL.OptionsHeader", "ModelingSystem");

    ss << fmt::format("{0}\n{1}\n{2}\n{3}\n",
        env->settings->getSetting<int>("AMPL.NumberOfOriginalConstraints", "ModelingSystem"), 0,
        env->problem->properties.numberOfVariables, env->problem->properties.numberOfVariables);

    if(this->primalSolution.size() > 0)
    {
        for(auto const& V : this->primalSolution)
            ss << fmt::format("{}\n", V);
    }
    else
    {
        {
            for(int i = 0; i < env->problem->properties.numberOfVariables; i++)
                ss << fmt::format("{}\n", 0);
        }
    }

    ss << fmt::format("objno 0 {}", status);

    return (ss.str());
}

void Results::createIteration() { iterations.push_back(std::make_shared<Iteration>(env)); }

IterationPtr Results::getCurrentIteration() { return (iterations.back()); }

IterationPtr Results::getPreviousIteration()
{
    if(getNumberOfIterations() > 1)
        return (iterations[getNumberOfIterations() - 2]);
    else
        throw Exception("Only one iteration!");
}

std::optional<IterationPtr> Results::getLastFeasibleIteration()
{
    std::optional<IterationPtr> iteration;

    for(auto I = iterations.rbegin(); I != iterations.rend(); ++I)
    {
        if(I->get()->solutionPoints.size() > 0)
        {
            iteration = *I;
            break;
        }
    }

    return iteration;
}

int Results::getNumberOfIterations() { return (iterations.size()); }

double Results::getPrimalBound()
{
    if(!std::isnan(this->currentPrimalBound))
        return (this->currentPrimalBound);
    else if(env->problem->objectiveFunction->direction == E_ObjectiveFunctionDirection::Minimize)
        return (SHOT_DBL_MAX);
    else
        return (SHOT_DBL_MIN);
}

void Results::setPrimalBound(double value)
{
    this->currentPrimalBound = value;

    // In case we have crossover
    if(env->problem->objectiveFunction->direction == E_ObjectiveFunctionDirection::Minimize)
    {
        if(value < this->globalDualBound && this->solutionIsGlobal)
            this->globalDualBound = value;

        if(value < this->currentDualBound)
            this->currentDualBound = value;
    }
    else
    {
        if(value > this->globalDualBound && this->solutionIsGlobal)
            this->globalDualBound = value;

        if(value > this->currentDualBound)
            this->currentDualBound = value;
    }

    if(env->problem->objectiveFunction->properties.isMinimize)
    {
        if(value < env->dualSolver->cutOffToUse)
        {
            env->dualSolver->cutOffToUse = value;
            env->dualSolver->useCutOff = true;
        }
    }
    else
    {
        if(value > env->dualSolver->cutOffToUse)
        {
            env->dualSolver->cutOffToUse = value;
            env->dualSolver->useCutOff = true;
        }
    }

    env->solutionStatistics.numberOfIterationsWithPrimalStagnation = 0;
    env->solutionStatistics.lastIterationWithSignificantPrimalUpdate = getNumberOfIterations() - 1;
    env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect = 0;
    env->solutionStatistics.numberOfDualRepairsSinceLastPrimalUpdate = 0;
}

double Results::getCurrentDualBound() { return (this->currentDualBound); }

double Results::getGlobalDualBound() { return (globalDualBound); }

void Results::setDualBound(double value, bool forceGlobal)
{
    double primalBound = this->getPrimalBound();

    if(env->problem->objectiveFunction->direction == E_ObjectiveFunctionDirection::Minimize)
    {
        if(value > primalBound)
            value = primalBound;
    }
    else
    {
        if(value < primalBound)
            value = primalBound;
    }

    this->currentDualBound = value;

    if(this->solutionIsGlobal || forceGlobal)
    {
        this->globalDualBound = value;
        this->solutionIsGlobal = true;
    }

    env->solutionStatistics.numberOfIterationsWithDualStagnation = 0;

    env->solutionStatistics.lastIterationWithSignificantDualUpdate = getNumberOfIterations() - 1;
}

double Results::getAbsoluteGlobalObjectiveGap()
{
    double gap = std::abs(getGlobalDualBound() - getPrimalBound());

    return (gap);
}

double Results::getRelativeGlobalObjectiveGap()
{
    double gap = std::abs(getGlobalDualBound() - getPrimalBound()) / ((1e-10) + std::abs(getPrimalBound()));

    return (gap);
}

double Results::getAbsoluteCurrentObjectiveGap()
{
    double gap = std::abs(getCurrentDualBound() - getPrimalBound());

    return (gap);
}

double Results::getRelativeCurrentObjectiveGap()
{
    double gap = std::abs(getCurrentDualBound() - getPrimalBound()) / ((1e-10) + std::abs(getPrimalBound()));

    return (gap);
}

E_ModelReturnStatus Results::getModelReturnStatus()
{
    if(isRelativeObjectiveGapToleranceMet() || isAbsoluteObjectiveGapToleranceMet())
        return (E_ModelReturnStatus::OptimalGlobal);

    if(hasPrimalSolution())
        return (E_ModelReturnStatus::FeasibleSolution);

    if(terminationReason == E_TerminationReason::UnboundedProblem)
        return (hasPrimalSolution() ? E_ModelReturnStatus::Unbounded : E_ModelReturnStatus::UnboundedNoSolution);

    if(terminationReason == E_TerminationReason::InfeasibleProblem)
        return (solutionIsGlobal ? E_ModelReturnStatus::InfeasibleGlobal : E_ModelReturnStatus::InfeasibleLocal);

    if(terminationReason == E_TerminationReason::Error || terminationReason == E_TerminationReason::NumericIssues)
        return (hasPrimalSolution() ? E_ModelReturnStatus::FeasibleSolution : E_ModelReturnStatus::ErrorNoSolution);

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

    for(size_t i = 0; i < solution.point.size(); i++)
    {
        str << variables.at(i);
        str << "\t";
        str << solution.point.at(i);
        str << '\n';
    }

    Utilities::writeStringToFile(fileName, str.str());
}

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

    for(size_t i = 0; i < solution.point.size(); i++)
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
}

void Results::increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType type)
{
    auto element = this->auxiliaryVariablesIntroduced.emplace(type, 1);

    if(!element.second)
    {
        // Element already exists
        element.first->second += 1;
    }
}

int Results::getAuxiliaryVariableCounter(E_AuxiliaryVariableType type)
{
    if(this->auxiliaryVariablesIntroduced[type] == 0)
        return 0;
    else
        return (this->auxiliaryVariablesIntroduced[type]);
}

} // namespace SHOT