/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "Solver.h"

#include "DualSolver.h"
#include "PrimalSolver.h"
#include "Report.h"
#include "Results.h"
#include "Settings.h"
#include "TaskHandler.h"
#include "Timing.h"
#include "Utilities.h"

#ifdef HAS_GAMS
#include "ModelingSystem/ModelingSystemGAMS.h"
#endif
#ifdef HAS_AMPL
#include "ModelingSystem/ModelingSystemAMPL.h"
#endif
#include "ModelingSystem/ModelingSystemOSiL.h"

#include "SolutionStrategy/SolutionStrategySingleTree.h"
#include "SolutionStrategy/SolutionStrategyMultiTree.h"
#include "SolutionStrategy/SolutionStrategyMIQCQP.h"
#include "SolutionStrategy/SolutionStrategyNLP.h"

#include "../Tasks/TaskPerformBoundTightening.h"
#include "../Tasks/TaskReformulateProblem.h"

#include <map>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

#ifdef HAS_GUROBI
#include "gurobi_c++.h"
#endif

namespace SHOT
{
namespace py = pybind11;

PYBIND11_MODULE(shotpy, m) {
    m.doc() = "shotpy";

    py::class_<Solver>(m, "Solver")
    .def(py::init())
    .def("getAbsoluteObjectiveGap", &Solver::getAbsoluteObjectiveGap)
    .def("getCurrentDualBound", &Solver::getCurrentDualBound)
    .def("getModelReturnStatus", &Solver::getModelReturnStatus)
    .def("getOptions", &Solver::getOptions)
    .def("getOptionsOSoL", &Solver::getOptionsOSoL)
    .def("getPrimalBound", &Solver::getPrimalBound)
    .def("getPrimalSolution", &Solver::getPrimalSolution)
    .def("getPrimalSolutions", &Solver::getPrimalSolutions)
    .def("getRelativeObjectiveGap", &Solver::getRelativeObjectiveGap)
    .def("getResultsOSrL", &Solver::getResultsOSrL)
    .def("getResultsSol", &Solver::getResultsSol)
    .def("getResultsTrace", &Solver::getResultsTrace)

    .def("getSolutionStatistics", &Solver::getSolutionStatistics)
    .def("getSettingsAsMarkup", &Solver::getSettingsAsMarkup)

    .def("getBoolSetting", py::overload_cast<std::string, std::string>(&Solver::getSetting<bool>))
    .def("getStringSetting", py::overload_cast<std::string, std::string>(&Solver::getSetting<std::string>))
    .def("getIntSetting", py::overload_cast<std::string, std::string>(&Solver::getSetting<int>))
    .def("getDoubleSetting", py::overload_cast<std::string, std::string>(&Solver::getSetting<double>))

    .def("getTerminationReason", &Solver::getTerminationReason)
    .def("hasPrimalSolution", &Solver::hasPrimalSolution)

    .def("outputSolverHeader", &Solver::outputSolverHeader)
    .def("outputOptionsReport", &Solver::outputOptionsReport)
    .def("outputProblemInstanceReport", &Solver::outputProblemInstanceReport)
    .def("outputSolutionReport", &Solver::outputSolutionReport)

    .def("setLogFile", &Solver::setLogFile)
    .def("setOptionsFromFile", &Solver::setOptionsFromFile)
    .def("setOptionsFromOSoL", &Solver::setOptionsFromOSoL)
    .def("setOptionsFromString", &Solver::setOptionsFromString)
    .def("setProblem", py::overload_cast<std::string>(&Solver::setProblem))
    .def("solveProblem", &Solver::solveProblem)
    .def("updateLogLevels", &Solver::updateLogLevels)
    .def("updateSetting", py::overload_cast<std::string, std::string, int>(&Solver::updateSetting))
    .def("updateSetting", py::overload_cast<std::string, std::string, std::string>(&Solver::updateSetting))
    .def("updateSetting", py::overload_cast<std::string, std::string, double>(&Solver::updateSetting))
    .def("updateSetting", py::overload_cast<std::string, std::string, bool>(&Solver::updateSetting))
    ;

    py::enum_<E_PrimalSolutionSource>(m, "PrimalSolutionSource", py::arithmetic())
        .value("Rootsearch", E_PrimalSolutionSource::Rootsearch)
        .value("RootsearchFixedIntegers", E_PrimalSolutionSource::RootsearchFixedIntegers)
        .value("NLPFixedIntegers", E_PrimalSolutionSource::NLPFixedIntegers)
        .value("NLPRelaxed", E_PrimalSolutionSource::NLPRelaxed)
        .value("MIPSolutionPool", E_PrimalSolutionSource::MIPSolutionPool)
        .value("LPFixedIntegers", E_PrimalSolutionSource::LPFixedIntegers)
        .value("MIPCallback", E_PrimalSolutionSource::MIPCallback)
        .value("InteriorPointSearch", E_PrimalSolutionSource::InteriorPointSearch)
    ;

    py::enum_<E_ModelReturnStatus>(m, "ModelReturnStatus", py::arithmetic())
        .value("None", E_ModelReturnStatus::None)
        .value("OptimalGlobal", E_ModelReturnStatus::OptimalGlobal)
        .value("Unbounded", E_ModelReturnStatus::Unbounded)
        .value("UnboundedNoSolution", E_ModelReturnStatus::UnboundedNoSolution)
        .value("InfeasibleGlobal", E_ModelReturnStatus::InfeasibleGlobal)
        .value("InfeasibleLocal", E_ModelReturnStatus::InfeasibleLocal)
        .value("FeasibleSolution", E_ModelReturnStatus::FeasibleSolution)
        .value("NoSolutionReturned", E_ModelReturnStatus::NoSolutionReturned)
        .value("ErrorUnknown", E_ModelReturnStatus::ErrorUnknown)
        .value("ErrorNoSolution", E_ModelReturnStatus::ErrorNoSolution)
    ;

    py::enum_<E_TerminationReason>(m, "TerminationReason", py::arithmetic())
        .value("ConstraintTolerance", E_TerminationReason::ConstraintTolerance)
        .value("ObjectiveStagnation", E_TerminationReason::ObjectiveStagnation)
        .value("IterationLimit", E_TerminationReason::IterationLimit)
        .value("TimeLimit", E_TerminationReason::TimeLimit)
        .value("InfeasibleProblem", E_TerminationReason::InfeasibleProblem)
        .value("UnboundedProblem", E_TerminationReason::UnboundedProblem)
        .value("Error", E_TerminationReason::Error)
        .value("AbsoluteGap", E_TerminationReason::AbsoluteGap)
        .value("RelativeGap", E_TerminationReason::RelativeGap)
        .value("UserAbort", E_TerminationReason::UserAbort)
        .value("NoDualCutsAdded", E_TerminationReason::NoDualCutsAdded)
        .value("None", E_TerminationReason::None)
        .value("NumericIssues", E_TerminationReason::NumericIssues)
    ;

    py::class_<PairIndexValue>(m, "PairIndexValue")
    .def_readwrite("index", &PairIndexValue::index)   
    .def_readwrite("value", &PairIndexValue::value)
    ;

    py::class_<PrimalSolution>(m, "PrimalSolution")
    .def_readwrite("point", &PrimalSolution::point)   
    .def_readwrite("sourceType", &PrimalSolution::sourceType)
    .def_readwrite("sourceDescription", &PrimalSolution::sourceDescription)
    .def_readwrite("objValue", &PrimalSolution::objValue)    
    .def_readwrite("iterFound", &PrimalSolution::iterFound)    
    .def_readwrite("maxDevatingConstraintLinear", &PrimalSolution::maxDevatingConstraintLinear)    
    .def_readwrite("maxDevatingConstraintQuadratic", &PrimalSolution::maxDevatingConstraintQuadratic)    
    .def_readwrite("maxDevatingConstraintNonlinear", &PrimalSolution::maxDevatingConstraintNonlinear)    
    .def_readwrite("maxIntegerToleranceError", &PrimalSolution::maxIntegerToleranceError)    
    .def_readwrite("boundProjectionPerformed", &PrimalSolution::boundProjectionPerformed)    
    .def_readwrite("integerRoundingPerformed", &PrimalSolution::integerRoundingPerformed) 
    .def_readwrite("displayed", &PrimalSolution::displayed) 
    ;

    py::class_<SolutionStatistics>(m, "SolutionStatistics")
    .def_readwrite("numberOfIterations", &SolutionStatistics::numberOfIterations)   
    .def_readwrite("numberOfProblemsLP", &SolutionStatistics::numberOfProblemsLP)   
    .def_readwrite("numberOfProblemsQP ", &SolutionStatistics::numberOfProblemsQP)
    .def_readwrite("numberOfProblemsQCQP", &SolutionStatistics::numberOfProblemsQCQP)
    .def_readwrite("numberOfProblemsFeasibleMILP", &SolutionStatistics::numberOfProblemsFeasibleMILP)
    .def_readwrite("numberOfProblemsOptimalMILP", &SolutionStatistics::numberOfProblemsOptimalMILP)
    .def_readwrite("numberOfProblemsFeasibleMIQP", &SolutionStatistics::numberOfProblemsFeasibleMIQP)
    .def_readwrite("numberOfProblemsOptimalMIQP", &SolutionStatistics::numberOfProblemsOptimalMIQP)
    .def_readwrite("numberOfProblemsFeasibleMIQCQP", &SolutionStatistics::numberOfProblemsFeasibleMIQCQP)
    .def_readwrite("numberOfProblemsOptimalMIQCQP", &SolutionStatistics::numberOfProblemsOptimalMIQCQP)
    .def_readwrite("numberOfFunctionEvalutions", &SolutionStatistics::numberOfFunctionEvalutions)
    .def_readwrite("numberOfGradientEvaluations", &SolutionStatistics::numberOfGradientEvaluations)
    .def_readwrite("numberOfProblemsMinimaxLP", &SolutionStatistics::numberOfProblemsMinimaxLP)
    .def_readwrite("numberOfProblemsFixedNLP", &SolutionStatistics::numberOfProblemsFixedNLP)
    .def_readwrite("numberOfConstraintsRemovedInPresolve", &SolutionStatistics::numberOfConstraintsRemovedInPresolve)
    .def_readwrite("numberOfVariableBoundsTightenedInPresolve", &SolutionStatistics::numberOfVariableBoundsTightenedInPresolve)
    .def_readwrite("numberOfHyperplanesWithConvexSource", &SolutionStatistics::numberOfHyperplanesWithConvexSource)
    .def_readwrite("numberOfHyperplanesWithNonconvexSource", &SolutionStatistics::numberOfHyperplanesWithNonconvexSource)
    .def_readwrite("numberOfIntegerCuts", &SolutionStatistics::numberOfIntegerCuts)
    .def_readwrite("numberOfIterationsWithDualStagnation", &SolutionStatistics::numberOfIterationsWithDualStagnation)
    .def_readwrite("lastIterationWithSignificantDualUpdate", &SolutionStatistics::lastIterationWithSignificantDualUpdate)
    .def_readwrite("numberOfIterationsWithPrimalStagnation", &SolutionStatistics::numberOfIterationsWithPrimalStagnation)
    .def_readwrite("lastIterationWithSignificantPrimalUpdate", &SolutionStatistics::lastIterationWithSignificantPrimalUpdate)
    .def_readwrite("numberOfIterationsWithoutNLPCallMIP", &SolutionStatistics::numberOfIterationsWithoutNLPCallMIP)
    .def_readwrite("iterationLastPrimalBoundUpdate", &SolutionStatistics::iterationLastPrimalBoundUpdate)
    .def_readwrite("iterationLastDualBoundUpdate", &SolutionStatistics::iterationLastDualBoundUpdate)
    .def_readwrite("iterationLastLazyAdded", &SolutionStatistics::iterationLastLazyAdded)
    .def_readwrite("iterationLastDualCutAdded", &SolutionStatistics::iterationLastDualCutAdded)
    .def_readwrite("timeLastDualBoundUpdate", &SolutionStatistics::timeLastDualBoundUpdate)
    .def_readwrite("timeLastFixedNLPCall", &SolutionStatistics::timeLastFixedNLPCall)
    .def_readwrite("numberOfOriginalInteriorPoints", &SolutionStatistics::numberOfOriginalInteriorPoints)
    .def_readwrite("numberOfFoundPrimalSolutions", &SolutionStatistics::numberOfFoundPrimalSolutions)
    .def_readwrite("numberOfExploredNodes", &SolutionStatistics::numberOfExploredNodes)
    .def_readwrite("numberOfOpenNodes", &SolutionStatistics::numberOfOpenNodes)
    .def_readwrite("numberOfPrimalReductionCutsUpdatesWithoutEffect", &SolutionStatistics::numberOfPrimalReductionCutsUpdatesWithoutEffect)
    .def_readwrite("numberOfDualRepairsSinceLastPrimalUpdate", &SolutionStatistics::numberOfDualRepairsSinceLastPrimalUpdate)
    .def_readwrite("numberOfPrimalReductionsPerformed", &SolutionStatistics::numberOfPrimalReductionsPerformed)
    .def_readwrite("numberOfSuccessfulDualRepairsPerformed", &SolutionStatistics::numberOfSuccessfulDualRepairsPerformed)
    .def_readwrite("numberOfUnsuccessfulDualRepairsPerformed", &SolutionStatistics::numberOfUnsuccessfulDualRepairsPerformed)
    .def_readwrite("numberOfPrimalImprovementsAfterInfeasibilityRepair", &SolutionStatistics::numberOfPrimalImprovementsAfterInfeasibilityRepair)
    .def_readwrite("numberOfPrimalImprovementsAfterReductionCut", &SolutionStatistics::numberOfPrimalImprovementsAfterReductionCut)
    .def_readwrite("hasInfeasibilityRepairBeenPerformedSincePrimalImprovement", &SolutionStatistics::hasInfeasibilityRepairBeenPerformedSincePrimalImprovement)
    .def_readwrite("hasReductionCutBeenAddedSincePrimalImprovement", &SolutionStatistics::hasReductionCutBeenAddedSincePrimalImprovement)
    .def("getNumberOfTotalDualProblems", &SolutionStatistics::getNumberOfTotalDualProblems)
    ;
}
}
