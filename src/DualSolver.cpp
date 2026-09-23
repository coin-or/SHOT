/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "DualSolver.h"
#include "Output.h"
#include "Settings.h"
#include "Results.h"
#include "Iteration.h"
#include "Utilities.h"
#include "Timing.h"
#include "Problem.h"
#include "ObjectiveFunction.h"
#include "MIPSolver/IMIPSolver.h"

#include <algorithm>
#include <cmath>

namespace SHOT
{

DualSolver::DualSolver(EnvironmentPtr envPtr) { env = envPtr; }

void DualSolver::addDualSolutionCandidate(DualSolution solution)
{
    dualSolutionCandidates.push_back(solution);

    this->checkDualSolutionCandidates();
}

// A dual bound that passes the primal bound by more than the tolerance is not valid for the problem: the cuts of the
// dual problem have then cut off the optimal solution, or the problem is so badly scaled that the bound is
// meaningless. The bound is ignored, which is only visible as an objective gap that does not close, so it is reported.
void DualSolver::warnAboutInvalidDualBound(double dualBound, double primalBound, double tolerance)
{
    auto message = fmt::format("        Dual bound {} passes the primal bound {} by more than the tolerance {}, so it "
                               "is not a valid bound for the problem and is ignored.",
        dualBound, primalBound, tolerance);

    if(invalidDualBoundWarningShown)
    {
        env->output->outputDebug(message);
        return;
    }

    env->output->outputWarning(message);
    invalidDualBoundWarningShown = true;
}

void DualSolver::checkDualSolutionCandidates()
{
    double currDualBound = env->results->getCurrentDualBound();
    double currPrimalBound = env->results->getPrimalBound();

    // The optimal value lies between the dual and the primal bound, so a valid dual bound can only pass the primal
    // bound by numerical error. Such a candidate is accepted as the primal bound, but only when it is within this
    // tolerance; passing the primal bound by more means that the candidate is not a valid bound for the problem,
    // and it is then ignored instead of closing the objective gap by force.
    //
    // The error is the one the primal solutions are accepted with: a primal solution whose constraint violations are
    // within the primal tolerances can have an objective value slightly better than the optimum of the problem,
    // while the dual bound is valid for the problem itself. How large the difference in the objective value is
    // depends on the problem, so the violation is scaled by the magnitude of the bound.
    double primalTolerance = std::max(env->settings->getSetting<double>("Primal.Tolerance.NonlinearConstraint"),
        env->settings->getSetting<double>("Primal.Tolerance.LinearConstraint"));

    double crossoverTolerance = primalTolerance * std::max(1.0, std::abs(currPrimalBound));

    for(auto& C : this->dualSolutionCandidates)
    {
        bool updateDual = false;

        if(env->problem->objectiveFunction->properties.isMinimize)
        {
            if(C.objValue > currPrimalBound && C.objValue <= currPrimalBound + crossoverTolerance)
            {
                C.objValue = currPrimalBound;
                updateDual = true;
            }
            else if(C.objValue > currDualBound && (C.objValue <= currPrimalBound))
            {
                updateDual = true;
            }
            else if(C.objValue > currPrimalBound + crossoverTolerance)
            {
                warnAboutInvalidDualBound(C.objValue, currPrimalBound, crossoverTolerance);
            }
        }
        else
        {
            if(C.objValue < currPrimalBound && C.objValue >= currPrimalBound - crossoverTolerance)
            {
                C.objValue = currPrimalBound;
                updateDual = true;
            }
            else if(C.objValue < currDualBound && (C.objValue >= currPrimalBound))
            {
                updateDual = true;
            }
            else if(C.objValue < currPrimalBound - crossoverTolerance)
            {
                warnAboutInvalidDualBound(C.objValue, currPrimalBound, crossoverTolerance);
            }
        }

        if(updateDual)
        {
            if(C.sourceType == E_DualSolutionSource::ConvexBounding)
                env->results->setDualBound(C.objValue, true); // Force valid dual bound update
            else
                env->results->setDualBound(C.objValue);

            currDualBound = C.objValue;

            if(env->results->getNumberOfIterations() > 0)
                env->solutionStatistics.iterationLastDualBoundUpdate
                    = env->results->getCurrentIteration()->iterationNumber;
            else
                env->solutionStatistics.iterationLastDualBoundUpdate = 0;

            env->solutionStatistics.iterationLastDualBoundUpdate = env->timing->getElapsedTime("Total");

            if(C.sourceType == E_DualSolutionSource::MIPSolutionOptimal
                || C.sourceType == E_DualSolutionSource::LPSolution
                || C.sourceType == E_DualSolutionSource::MIPSolverBound
                || C.sourceType == E_DualSolutionSource::ConvexBounding)
            {
                env->results->addDualSolution(C);
            }

            std::string sourceDesc;

            switch(C.sourceType)
            {
            case E_DualSolutionSource::LPSolution:
                sourceDesc = "LP solution";
                break;
            case E_DualSolutionSource::MIPSolutionOptimal:
                sourceDesc = "MIP solution";
                break;
            case E_DualSolutionSource::ObjectiveConstraint:
                sourceDesc = "Obj. constr. rootsearch";
                break;
            case E_DualSolutionSource::MIPSolverBound:
                sourceDesc = "MIP solver bound";
                break;
            case E_DualSolutionSource::ConvexBounding:
                sourceDesc = "Convex MIP bounding";
                break;
            case E_DualSolutionSource::InfeasibleWithCutOff:
                sourceDesc = "infeasible dual problem with cutoff";
                break;
            default:
                break;
            }

            if(C.sourceType == E_DualSolutionSource::ConvexBounding)
            {
                env->output->outputInfo(fmt::format("        New dual bound {}, source: {}", C.objValue, sourceDesc));
            }
            else
            {
                env->output->outputDebug(fmt::format("        New dual bound {}, source: {}", C.objValue, sourceDesc));
            }
        }
    }

    this->dualSolutionCandidates.clear();
}

std::pair<double, double> DualSolver::calculateHashes(const VectorDouble& point)
{
    return (Utilities::calculateHashes(point));
}

std::pair<double, double> DualSolver::calculateHyperplaneHashes(NumericHyperplanePtr hyperplane)
{
    // A constraint cut is determined by its point, but an objective cut also depends on the objective value used
    if(auto objectiveHP = std::dynamic_pointer_cast<ObjectiveHyperplane>(hyperplane))
    {
        auto pointAndValue = objectiveHP->generatedPoint;
        pointAndValue.push_back(objectiveHP->objectiveFunctionValue);
        return (calculateHashes(pointAndValue));
    }

    return (calculateHashes(hyperplane->generatedPoint));
}

void DualSolver::addHyperplane(HyperplanePtr hyperplane)
{
    if(auto objectiveHP = std::dynamic_pointer_cast<ObjectiveHyperplane>(hyperplane))
    {
        assert((int)objectiveHP->generatedPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

        auto hashes = calculateHyperplaneHashes(objectiveHP);
        objectiveHP->pointHash = hashes.first;

        if(!hasHyperplaneBeenAdded(hashes, -1))
        {
            this->hyperplaneWaitingList.push_back(hyperplane);
        }
        else
        {
            env->output->outputDebug(fmt::format(
                "        Objective hyperplane with hash {} has been added already.", objectiveHP->pointHash));
        }
    }
    else if(auto constraintHP = std::dynamic_pointer_cast<ConstraintHyperplane>(hyperplane))
    {
        assert((int)constraintHP->generatedPoint.size() == env->reformulatedProblem->properties.numberOfVariables);

        auto hashes = calculateHyperplaneHashes(constraintHP);
        constraintHP->pointHash = hashes.first;

        if(!hasHyperplaneBeenAdded(hashes, constraintHP->sourceConstraint->getIndex()))
        {
            this->hyperplaneWaitingList.push_back(hyperplane);
        }
        else
        {
            env->output->outputDebug(
                fmt::format("        Hyperplane with hash {} has been added already.", constraintHP->pointHash));
        }
    }
    else if(auto externalHP = std::dynamic_pointer_cast<ExternalHyperplane>(hyperplane))
    {
        // TODO check for already added hyperplanes
        this->hyperplaneWaitingList.push_back(externalHP);
    }
}

void DualSolver::addGeneratedHyperplane(const HyperplanePtr hyperplane)
{
    std::string source = "";

    switch(hyperplane->source)
    {
    case E_HyperplaneSource::MIPOptimalRootsearch:
        source = "MIP rootsearch";
        break;
    case E_HyperplaneSource::LPRelaxedRootsearch:
        source = "LP rootsearch";
        break;
    case E_HyperplaneSource::MIPOptimalSolutionPoint:
        source = "MIP optimal solution";
        break;
    case E_HyperplaneSource::MIPSolutionPoolSolutionPoint:
        source = "MIP solution pool";
        break;
    case E_HyperplaneSource::LPRelaxedSolutionPoint:
        source = "LP solution";
        break;
    case E_HyperplaneSource::LPFixedIntegers:
        source = "LP fixed integer";
        break;
    case E_HyperplaneSource::PrimalSolutionSearch:
        source = "primal heuristic";
        break;
    case E_HyperplaneSource::PrimalSolutionSearchInteriorObjective:
        source = "primal heuristic (interior objective)";
        break;
    case E_HyperplaneSource::InteriorPointSearch:
        source = "interior point search";
        break;
    case E_HyperplaneSource::MIPCallbackRelaxed:
        source = "MIP callback relaxed";
        break;
    case E_HyperplaneSource::ObjectiveRootsearch:
        source = "objective rootsearch";
        break;
    case E_HyperplaneSource::ObjectiveCuttingPlane:
        source = "objective cutting plane";
        break;
    case E_HyperplaneSource::External:
        source = "external";
        break;
    default:
        break;
    }

    auto genHyperplane = std::make_shared<GeneratedHyperplane>();
    genHyperplane->sourceHyperplane = hyperplane;
    genHyperplane->iterationGenerated = env->results->getCurrentIteration()->iterationNumber;
    genHyperplane->isLazy = false;

    if(!genHyperplane->sourceHyperplane->isGlobal)
    {
        if(env->results->solutionIsGlobal)
            env->output->outputDebug("        Solution is no longer global since hyperplane has been added to "
                                     "nonconvex objective or constraint.");

        env->results->solutionIsGlobal = false;
    }
    else
    {
        env->output->outputTrace("        Convex HP added.");
    }

    generatedHyperplanes.push_back(genHyperplane);

    if(auto numericHP = std::dynamic_pointer_cast<NumericHyperplane>(hyperplane))
    {
        // The hashes are recalculated since not all hyperplanes pass through addHyperplane(), e.g. in single-tree
        // callbacks
        auto hashes = calculateHyperplaneHashes(numericHP);
        numericHP->pointHash = hashes.first;

        auto constraintHP = std::dynamic_pointer_cast<ConstraintHyperplane>(numericHP);
        int constraintIndex = constraintHP ? constraintHP->sourceConstraint->getIndex() : -1;

        bool isRepeatedHyperplane = isHyperplaneInGeneratedList(hashes, constraintIndex);
        generatedHyperplaneHashes[constraintIndex].emplace(hashes.first, hashes.second);

        // A hyperplane generated again for a point it has already been generated in does not cut off the solution
        // point it was generated for, so the dual problem is not making any progress. Duplicates are not rejected
        // in the single-tree strategy, since a lazy constraint is not always kept by the MIP solver, and this is
        // therefore only reported.
        if(isRepeatedHyperplane)
        {
            numberOfRepeatedHyperplanes++;

            if(numberOfRepeatedHyperplanes == 100 && !repeatedHyperplaneWarningShown)
            {
                bool isMultiTree = env->settings->getSetting<int>("Dual.TreeStrategy")
                    == static_cast<int>(ES_TreeStrategy::MultiTree);

                env->output->outputWarning(
                    fmt::format("        {} hyperplanes have been generated in points they were already generated "
                                "in, the last one for constraint {}. The dual problem is not making progress.{}",
                        numberOfRepeatedHyperplanes, constraintIndex,
                        isMultiTree ? " Solving the remaining MIP problems to optimality." : ""));
                repeatedHyperplaneWarningShown = true;

                // Solutions found at the solution limit do not give new cuts, so the remaining MIP problems are solved
                // to optimality
                if(isMultiTree)
                    MIPSolver->setSolutionLimit(2100000000);
            }
        }
    }

    auto currentIteration = env->results->getCurrentIteration();
    currentIteration->numHyperplanesAdded++;
    currentIteration->totNumHyperplanes++;
    env->solutionStatistics.iterationLastDualCutAdded = currentIteration->iterationNumber;

    if(hyperplane->isGlobal)
        env->solutionStatistics.numberOfHyperplanesWithConvexSource++;
    else
        env->solutionStatistics.numberOfHyperplanesWithNonconvexSource++;

    env->output->outputTrace("        Hyperplane generated from: " + source);
}

bool DualSolver::hasHyperplaneBeenAdded(const std::pair<double, double>& hashes, int constraintIndex)
{
    // Cuts added as lazy might not actually always be added (e.g. in different threads), thus we have to allow them
    // to be added again
    if(env->settings->getSetting<int>("Dual.TreeStrategy") == static_cast<int>(ES_TreeStrategy::SingleTree))
        return false;

    return (isHyperplaneInGeneratedList(hashes, constraintIndex));
}

bool DualSolver::isHyperplaneInGeneratedList(const std::pair<double, double>& hashes, int constraintIndex)
{
    auto generated = generatedHyperplaneHashes.find(constraintIndex);

    if(generated == generatedHyperplaneHashes.end())
        return (false);

    // The hashes of two identical points only differ by rounding errors, and since the values are mapped into
    // (-1, 1) before they are hashed, a point differing in any single variable differs by much more than this.
    double firstTolerance = 1e-10 * std::max(1.0, std::abs(hashes.first));
    double secondTolerance = 1e-10 * std::max(1.0, std::abs(hashes.second));

    auto candidate = generated->second.lower_bound(hashes.first - firstTolerance);

    for(; candidate != generated->second.end() && candidate->first <= hashes.first + firstTolerance; candidate++)
    {
        // Both hashes are compared, since two different points can match in one of them
        if(std::abs(candidate->second - hashes.second) <= secondTolerance)
            return (true);
    }

    return (false);
}

std::optional<std::pair<double, double>> DualSolver::evaluateHyperplaneTerms(
    const VectorDouble& generationPoint, const VectorDouble& pointToCutOff, const NumericConstraintPtr& constraint)
{
    auto hyperplane = std::make_shared<ConstraintHyperplane>();
    hyperplane->sourceConstraint = constraint;
    hyperplane->generatedPoint = generationPoint;
    hyperplane->isGlobal = (constraint->properties.convexity <= E_Convexity::Convex);

    auto terms = MIPSolver->createHyperplaneTerms(hyperplane);

    if(!terms)
        return (std::nullopt);

    double magnitude = std::abs(terms->second);
    double valueInPointToCutOff = terms->second;

    for(auto& E : terms->first)
    {
        magnitude = std::max(magnitude, std::abs(E.second));

        // The objective variable of the dual problem is after the variables of the problem, and the point does not
        // contain a value for it, so its term is left out of the value. A hyperplane for a constraint does not
        // contain it, and the value is only used to see whether the point is cut off.
        if(E.first < (int)pointToCutOff.size())
            valueInPointToCutOff += E.second * pointToCutOff.at(E.first);
    }

    return (std::make_pair(magnitude, valueInPointToCutOff));
}

std::vector<VectorDouble> DualSolver::getFinitePointCandidates()
{
    std::vector<VectorDouble> candidates;

    for(auto& IP : interiorPts)
        candidates.push_back(IP->point);

    if(env->results->hasPrimalSolution())
        candidates.push_back(env->results->primalSolution);

    candidates.push_back(Utilities::calculateBoxCenterPoint(
        env->reformulatedProblem->getVariableLowerBounds(), env->reformulatedProblem->getVariableUpperBounds()));

    return (candidates);
}

std::optional<VectorDouble> DualSolver::getHyperplaneGenerationPoint(
    const VectorDouble& point, const NumericConstraintPtr& constraint)
{
    // The largest magnitude accepted of a hyperplane generated in a point that has been moved. A hyperplane whose
    // largest value is above 1e9 is rescaled by MIPSolverBase::createHyperplane, so this keeps the generated
    // constraint within a few orders of magnitude of the rest of the dual problem, where the MIP solvers behave.
    const double maximumMagnitude = 1e6;
    const double fractionMultiplier = 10.0;
    const int maximumNumberOfTrials = 10;

    if(auto terms = evaluateHyperplaneTerms(point, point, constraint); terms && std::isfinite(terms->first))
        return (point);

    double smallestFraction = env->settings->getSetting<double>("Dual.HyperplaneCuts.NonfinitePointRetreatFactor");
    int constraintIndex = constraint->getIndex();

    for(auto& target : getFinitePointCandidates())
    {
        if(target.size() != point.size())
            continue;

        std::optional<VectorDouble> firstFinitePoint;
        double fraction = smallestFraction;

        for(int i = 0; i < maximumNumberOfTrials && fraction <= 0.5; i++, fraction *= fractionMultiplier)
        {
            auto trialPoint = Utilities::getPointOnSegment(point, target, fraction);
            auto terms = evaluateHyperplaneTerms(trialPoint, point, constraint);

            // The constraint is still outside its domain in the point, or a hyperplane has already been generated
            // there, which would only repeat a cut that did not help
            if(!terms || !std::isfinite(terms->first) || hasHyperplaneBeenAdded(trialPoint, constraintIndex))
                continue;

            if(!firstFinitePoint)
                firstFinitePoint = trialPoint;

            // The hyperplanes get flatter the further away the point is, so the first one that is both usable by
            // the MIP solver and cuts the point off is the tightest one of those
            if(terms->first <= maximumMagnitude && terms->second > 0.0)
                return (trialPoint);
        }

        if(firstFinitePoint)
            return (firstFinitePoint);
    }

    return (std::nullopt);
}

bool DualSolver::hasHyperplaneBeenAdded(const VectorDouble& generatedPoint, int constraintIndex)
{
    return (hasHyperplaneBeenAdded(calculateHashes(generatedPoint), constraintIndex));
}

void DualSolver::addIntegerCut(IntegerCut integerCut)
{
    if(env->reformulatedProblem->properties.numberOfIntegerVariables > 0
        || env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0)
    {
        integerCut.areAllVariablesBinary = false;
    }
    else
    {
        integerCut.areAllVariablesBinary = true;
    }

    integerCut.pointHashes = Utilities::calculateHashes(integerCut.variableValues);

    if(!hasIntegerCutBeenAdded(integerCut.pointHashes))
        this->integerCutWaitingList.push_back(integerCut);
    else
        env->output->outputDebug(
            fmt::format("        Integer cut with hash {} has been added already.", integerCut.pointHashes.first));
}

void DualSolver::addGeneratedIntegerCut(IntegerCut integerCut)
{
    std::string source = "";

    switch(integerCut.source)
    {
    case E_IntegerCutSource::NLPFixedInteger:
        source = "NLP fixed integer";
        break;

    default:
        break;
    }

    integerCut.iterationGenerated = env->results->getCurrentIteration()->iterationNumber;

    if(env->results->solutionIsGlobal && env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        env->results->solutionIsGlobal = false;
        env->output->outputInfo("        Solution is no longer global since integer cut has been added.");
    }

    env->output->outputDebug(
        fmt::format("        Added integer cut with hash {}", integerCut.pointHashes.first));

    generatedIntegerCuts.push_back(integerCut);

    auto currentIteration = env->results->getCurrentIteration();
    currentIteration->numHyperplanesAdded++;
    currentIteration->totNumHyperplanes++;

    env->solutionStatistics.numberOfIntegerCuts++;

    env->output->outputDebug("        Integer cut generated from: " + source);
}

bool DualSolver::hasIntegerCutBeenAdded(const PairDouble& hashes)
{
    for(auto& IC : generatedIntegerCuts)
    {
        if(Utilities::haveSameHashes(IC.pointHashes, hashes))
        {
            return (true);
        }
    }

    return (false);
}

void DualSolver::removeArtificialBounds(const std::vector<VariablePtr>& variables)
{
    if(variables.size() == 0)
        return;

    double lowerLimit = env->settings->getSetting<double>("Model.Variables.Continuous.MinimumLowerBound");
    double upperLimit = env->settings->getSetting<double>("Model.Variables.Continuous.MaximumUpperBound");
    double unboundedValue = MIPSolver->getUnboundedVariableBoundValue();

    for(auto& V : variables)
    {
        if(!V->properties.hasArtificialLowerBound && !V->properties.hasArtificialUpperBound)
            continue;

        env->output->outputDebug(fmt::format("        Removing the artificial bounds of variable {}.", V->name));

        env->problem->setVariableBounds(V->getIndex(), V->properties.hasArtificialLowerBound ? lowerLimit : V->lowerBound,
            V->properties.hasArtificialUpperBound ? upperLimit : V->upperBound);

        if(env->reformulatedProblem)
            env->reformulatedProblem->setVariableBounds(V->getIndex(), V->lowerBound, V->upperBound);

        MIPSolver->updateVariableBound(V->getIndex(),
            V->properties.hasArtificialLowerBound ? -unboundedValue : V->lowerBound,
            V->properties.hasArtificialUpperBound ? unboundedValue : V->upperBound);

        V->properties.hasArtificialLowerBound = false;
        V->properties.hasArtificialUpperBound = false;
    }

    // The dual bounds may only be valid with the artificial bounds
    bool isMinimize = env->problem->objectiveFunction->properties.isMinimize;
    env->results->currentDualBound = isMinimize ? SHOT_DBL_MIN : SHOT_DBL_MAX;
    env->results->globalDualBound = isMinimize ? SHOT_DBL_MIN : SHOT_DBL_MAX;

    if(MIPSolver->hasDualAuxiliaryObjectiveVariable())
    {
        auto bounds = MIPSolver->getCurrentVariableBounds(MIPSolver->getDualAuxiliaryObjectiveVariableIndex());
        MIPSolver->updateVariableBound(
            MIPSolver->getDualAuxiliaryObjectiveVariableIndex(), -unboundedValue, bounds.second);
    }
}

bool DualSolver::isDualProblemExact()
{
    // Nonlinear constraints and nonlinear objectives are only represented by cuts in the dual problem
    return (env->reformulatedProblem->properties.numberOfNonlinearConstraints == 0
        && env->reformulatedProblem->objectiveFunction->properties.classification
            <= E_ObjectiveFunctionClassification::Quadratic);
}

} // namespace SHOT