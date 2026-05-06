/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>

// Prevent CppAD template instantiation in this compilation unit
// to avoid ODR violations with libSHOTSolver.so
// The templates are explicitly instantiated in Problem.cpp
#include "cppad/cppad.hpp"
extern template class CppAD::AD<double>;
extern template class CppAD::ADFun<double>;

// Custom CppAD error handler to avoid abort on cleanup errors
// This is needed because CppAD's thread_alloc has ODR issues with shared libraries
namespace {
    void cppad_python_error_handler(
        bool known,
        int line,
        const char* file,
        const char* exp,
        const char* msg)
    {
        // Check if this is the known cleanup error in thread_alloc
        std::string fileStr(file ? file : "");
        std::string expStr(exp ? exp : "");
        if(fileStr.find("thread_alloc.hpp") != std::string::npos 
           && expStr.find("count_inuse_") != std::string::npos)
        {
            // Suppress this error - it's a harmless ODR issue during cleanup
            return;
        }
        
        // For other errors, throw an exception
        std::string error_msg = std::string("CppAD error at ") + file + ":" + std::to_string(line);
        if(msg) error_msg += std::string(" - ") + msg;
        throw std::runtime_error(error_msg);
    }
    
    // Register the custom error handler at module load time
    struct CppADErrorHandlerRegistrar {
        CppAD::ErrorHandler handler;
        CppADErrorHandlerRegistrar() : handler(cppad_python_error_handler) {}
    };
    static CppADErrorHandlerRegistrar cppad_error_handler_registrar;
}

#include "Solver.h"
#include "CallbackData.h"

#include "DualSolver.h"
#include "PrimalSolver.h"
#include "Report.h"
#include "Results.h"
#include "Settings.h"
#include "TaskHandler.h"
#include "Timing.h"
#include "Utilities.h"

#include "Model/Problem.h"
#include "Model/Variables.h"
#include "Model/Terms.h"
#include "Model/Constraints.h"
#include "Model/ObjectiveFunction.h"
#include "Model/NonlinearExpressions.h"
#include "Model/Simplifications.h"

#include <sstream>

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

// Forward declarations for operator overloads
NonlinearExpressionPtr wrapInExpression(VariablePtr var);
NonlinearExpressionPtr wrapInExpression(double value);
NonlinearExpressionPtr wrapInExpression(NonlinearExpressionPtr expr);

// Helper to wrap a Variable in an ExpressionVariable
NonlinearExpressionPtr wrapInExpression(VariablePtr var) { return std::make_shared<ExpressionVariable>(var); }

// Helper to wrap a constant in an ExpressionConstant
NonlinearExpressionPtr wrapInExpression(double value) { return std::make_shared<ExpressionConstant>(value); }

// Pass-through for expressions
NonlinearExpressionPtr wrapInExpression(NonlinearExpressionPtr expr) { return expr; }

PYBIND11_MODULE(SHOTpy, m)
{
    m.doc() = "SHOTpy";

    // ===== Constants =====
    m.attr("SHOT_DBL_MAX") = SHOT_DBL_MAX;
    m.attr("SHOT_DBL_MIN") = SHOT_DBL_MIN;

    // ===== Modeling System Availability =====
    // These constants indicate which modeling systems are available in this build
#ifdef HAS_GAMS
    m.attr("HAS_GAMS") = true;
#else
    m.attr("HAS_GAMS") = false;
#endif

#ifdef HAS_AMPL
    m.attr("HAS_AMPL") = true;
#else
    m.attr("HAS_AMPL") = false;
#endif

    // OSiL is always available
    m.attr("HAS_OSIL") = true;

    // Modeling system enum
    py::enum_<ES_ModelingSystem>(m, "ModelingSystem")
        .value("OSiL", ES_ModelingSystem::OSiL)
        .value("GAMS", ES_ModelingSystem::GAMS)
        .value("AMPL", ES_ModelingSystem::AMPL)
        .value("None", ES_ModelingSystem::None);

    // Function to get list of supported modeling systems - uses C++ API directly
    m.def("getSupportedModelingSystems", &Solver::getSupportedModelingSystems,
        "Returns a list of modeling systems supported in this build");

    // ===== MIP Solver Availability =====
    // These constants indicate which MIP solvers are available in this build
#ifdef HAS_CPLEX
    m.attr("HAS_CPLEX") = true;
#else
    m.attr("HAS_CPLEX") = false;
#endif

#ifdef HAS_GUROBI
    m.attr("HAS_GUROBI") = true;
#else
    m.attr("HAS_GUROBI") = false;
#endif

#ifdef HAS_CBC
    m.attr("HAS_CBC") = true;
#else
    m.attr("HAS_CBC") = false;
#endif

#ifdef HAS_HIGHS
    m.attr("HAS_HIGHS") = true;
#else
    m.attr("HAS_HIGHS") = false;
#endif

    // Function to get list of supported MIP solvers - uses C++ API directly
    m.def("getSupportedMIPSolvers", &Solver::getSupportedMIPSolvers,
        "Returns a list of MIP solvers supported in this build");

    // ===== NLP Solver Availability =====
    // These constants indicate which NLP solvers are available in this build
#ifdef HAS_IPOPT
    m.attr("HAS_IPOPT") = true;
#else
    m.attr("HAS_IPOPT") = false;
#endif

    // SHOT's internal NLP solver is always available
    m.attr("HAS_SHOT_NLP") = true;

    // GAMS NLP solver is available when GAMS is available
#ifdef HAS_GAMS
    m.attr("HAS_GAMS_NLP") = true;
#else
    m.attr("HAS_GAMS_NLP") = false;
#endif

    // Function to get list of supported NLP solvers - uses C++ API directly
    m.def("getSupportedNLPSolvers", &Solver::getSupportedNLPSolvers,
        "Returns a list of NLP solvers supported in this build");

    // ===== Variable Types Enum =====
    py::enum_<E_VariableType>(m, "VariableType")
        .value("Real", E_VariableType::Real)
        .value("Binary", E_VariableType::Binary)
        .value("Integer", E_VariableType::Integer)
        .value("Semicontinuous", E_VariableType::Semicontinuous)
        .value("Semiinteger", E_VariableType::Semiinteger);

    // ===== Objective Direction Enum =====
    py::enum_<E_ObjectiveFunctionDirection>(m, "ObjectiveDirection")
        .value("Minimize", E_ObjectiveFunctionDirection::Minimize)
        .value("Maximize", E_ObjectiveFunctionDirection::Maximize);

    // ===== Convexity Enum =====
    py::enum_<E_Convexity>(m, "Convexity")
        .value("Linear", E_Convexity::Linear)
        .value("Convex", E_Convexity::Convex)
        .value("Concave", E_Convexity::Concave)
        .value("Nonconvex", E_Convexity::Nonconvex)
        .value("Unknown", E_Convexity::Unknown)
        .value("NotSet", E_Convexity::NotSet);

    // ===== Problem Convexity Enum =====
    py::enum_<E_ProblemConvexity>(m, "ProblemConvexity")
        .value("Convex", E_ProblemConvexity::Convex)
        .value("Nonconvex", E_ProblemConvexity::Nonconvex)
        .value("NotSet", E_ProblemConvexity::NotSet);

    // ===== Settings Enums =====
    // These enums are used for type-safe setting values

    // Hyperplane cut strategy: ESH or ECP
    py::enum_<ES_HyperplaneCutStrategy>(m, "HyperplaneCutStrategy")
        .value("ESH", ES_HyperplaneCutStrategy::ESH)
        .value("ECP", ES_HyperplaneCutStrategy::ECP);

    // Iteration output detail level
    py::enum_<ES_IterationOutputDetail>(m, "IterationOutputDetail")
        .value("Full", ES_IterationOutputDetail::Full)
        .value("ObjectiveGapUpdates", ES_IterationOutputDetail::ObjectiveGapUpdates)
        .value("ObjectiveGapUpdatesAndNLPCalls", ES_IterationOutputDetail::ObjectiveGapUpdatesAndNLPCalls);

    // MIP solver selection
    py::enum_<ES_MIPSolver>(m, "MIPSolver")
        .value("Cplex", ES_MIPSolver::Cplex)
        .value("Gurobi", ES_MIPSolver::Gurobi)
        .value("Cbc", ES_MIPSolver::Cbc)
        .value("Highs", ES_MIPSolver::Highs)
        .value("None", ES_MIPSolver::None);

    // Source of fixed MIP solution point for NLP
    py::enum_<ES_PrimalNLPFixedPoint>(m, "PrimalNLPFixedPoint")
        .value("AllSolutions", ES_PrimalNLPFixedPoint::AllSolutions)
        .value("FirstSolution", ES_PrimalNLPFixedPoint::FirstSolution)
        .value("AllFeasibleSolutions", ES_PrimalNLPFixedPoint::AllFeasibleSolutions)
        .value("FirstAndFeasibleSolutions", ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions)
        .value("SmallestDeviationSolution", ES_PrimalNLPFixedPoint::SmallestDeviationSolution);

    // Problem formulation source for NLP
    py::enum_<ES_PrimalNLPProblemSource>(m, "PrimalNLPProblemSource")
        .value("OriginalProblem", ES_PrimalNLPProblemSource::OriginalProblem)
        .value("ReformulatedProblem", ES_PrimalNLPProblemSource::ReformulatedProblem)
        .value("Both", ES_PrimalNLPProblemSource::Both);

    // NLP solver selection
    py::enum_<ES_PrimalNLPSolver>(m, "PrimalNLPSolver")
        .value("Ipopt", ES_PrimalNLPSolver::Ipopt)
        .value("GAMS", ES_PrimalNLPSolver::GAMS)
        .value("SHOT", ES_PrimalNLPSolver::SHOT)
        .value("None", ES_PrimalNLPSolver::None);

    // NLP solver call strategy
    py::enum_<ES_PrimalNLPStrategy>(m, "PrimalNLPStrategy")
        .value("AlwaysUse", ES_PrimalNLPStrategy::AlwaysUse)
        .value("IterationOrTime", ES_PrimalNLPStrategy::IterationOrTime)
        .value("IterationOrTimeAndAllFeasibleSolutions", ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions);

    // Quadratic problem handling strategy
    py::enum_<ES_QuadraticProblemStrategy>(m, "QuadraticProblemStrategy")
        .value("Nonlinear", ES_QuadraticProblemStrategy::Nonlinear)
        .value("QuadraticObjective", ES_QuadraticProblemStrategy::QuadraticObjective)
        .value("ConvexQuadraticallyConstrained", ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained)
        .value("NonconvexQuadraticallyConstrained", ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained);

    // Tree strategy: single-tree or multi-tree
    py::enum_<ES_TreeStrategy>(m, "TreeStrategy")
        .value("MultiTree", ES_TreeStrategy::MultiTree)
        .value("SingleTree", ES_TreeStrategy::SingleTree);

    // ===== Variable Class =====
    py::class_<Variable, std::shared_ptr<Variable>>(m, "Variable")
        .def(py::init<std::string, int, E_VariableType, double, double>(), py::arg("name"), py::arg("index"),
            py::arg("type"), py::arg("lower_bound"), py::arg("upper_bound"))
        .def_readwrite("name", &Variable::name)
        .def_readwrite("index", &Variable::index)
        .def_readwrite("lowerBound", &Variable::lowerBound)
        .def_readwrite("upperBound", &Variable::upperBound)
        .def_readonly("properties", &Variable::properties)
        .def("__repr__",
            [](const Variable& v) { return "<Variable '" + v.name + "' index=" + std::to_string(v.index) + ">"; })
        // Operator overloads for natural expression building
        .def(
            "__add__",
            [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(wrapInExpression(self), wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__add__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(wrapInExpression(self), wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__radd__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(wrapInExpression(other), wrapInExpression(self));
            },
            py::is_operator())
        .def(
            "__add__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(wrapInExpression(self), other);
            },
            py::is_operator())
        .def(
            "__radd__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(other, wrapInExpression(self));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(self), std::make_shared<ExpressionNegate>(wrapInExpression(other)));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(wrapInExpression(self), wrapInExpression(-other));
            },
            py::is_operator())
        .def(
            "__rsub__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(other), std::make_shared<ExpressionNegate>(wrapInExpression(self)));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(self), std::make_shared<ExpressionNegate>(other));
            },
            py::is_operator())
        .def(
            "__mul__",
            [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(wrapInExpression(self), wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__mul__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(wrapInExpression(self), wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__rmul__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(wrapInExpression(other), wrapInExpression(self));
            },
            py::is_operator())
        .def(
            "__mul__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(wrapInExpression(self), other);
            },
            py::is_operator())
        .def(
            "__rmul__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(other, wrapInExpression(self));
            },
            py::is_operator())
        .def(
            "__truediv__",
            [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(wrapInExpression(self), wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__truediv__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(wrapInExpression(self), wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__rtruediv__",
            [](VariablePtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(wrapInExpression(other), wrapInExpression(self));
            },
            py::is_operator())
        .def(
            "__truediv__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(wrapInExpression(self), other);
            },
            py::is_operator())
        .def(
            "__pow__",
            [](VariablePtr self, double exponent) -> NonlinearExpressionPtr {
                if(exponent == 2.0)
                    return std::make_shared<ExpressionSquare>(wrapInExpression(self));
                return std::make_shared<ExpressionPower>(wrapInExpression(self), wrapInExpression(exponent));
            },
            py::is_operator())
        .def(
            "__pow__",
            [](VariablePtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionPower>(wrapInExpression(self), wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__pow__",
            [](VariablePtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionPower>(wrapInExpression(self), other);
            },
            py::is_operator())
        .def(
            "__neg__",
            [](VariablePtr self) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionNegate>(wrapInExpression(self));
            },
            py::is_operator());

    // ===== VariableProperties Struct =====
    py::class_<VariableProperties>(m, "VariableProperties")
        .def_readonly("type", &VariableProperties::type)
        .def_readonly("isAuxiliary", &VariableProperties::isAuxiliary)
        .def_readonly("isNonlinear", &VariableProperties::isNonlinear)
        .def_readonly("inObjectiveFunction", &VariableProperties::inObjectiveFunction)
        .def_readonly("inLinearConstraints", &VariableProperties::inLinearConstraints)
        .def_readonly("inQuadraticConstraints", &VariableProperties::inQuadraticConstraints)
        .def_readonly("inNonlinearConstraints", &VariableProperties::inNonlinearConstraints);

    // ===== NonlinearExpression Base Class =====
    py::class_<NonlinearExpression, NonlinearExpressionPtr>(m, "Expression")
        .def("getType", &NonlinearExpression::getType)
        .def("getConvexity", &NonlinearExpression::getConvexity)
        .def("__repr__",
            [](NonlinearExpressionPtr self) {
                std::ostringstream oss;
                oss << *self;
                return "<Expression: " + oss.str() + ">";
            })
        // Operator overloads for expressions
        .def(
            "__add__",
            [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(self, other);
            },
            py::is_operator())
        .def(
            "__add__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(self, wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__radd__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(wrapInExpression(other), self);
            },
            py::is_operator())
        .def(
            "__add__",
            [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(self, wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(self, std::make_shared<ExpressionNegate>(other));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(self, wrapInExpression(-other));
            },
            py::is_operator())
        .def(
            "__rsub__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(
                    wrapInExpression(other), std::make_shared<ExpressionNegate>(self));
            },
            py::is_operator())
        .def(
            "__sub__",
            [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionSum>(
                    self, std::make_shared<ExpressionNegate>(wrapInExpression(other)));
            },
            py::is_operator())
        .def(
            "__mul__",
            [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(self, other);
            },
            py::is_operator())
        .def(
            "__mul__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(self, wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__rmul__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(wrapInExpression(other), self);
            },
            py::is_operator())
        .def(
            "__mul__",
            [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionProduct>(self, wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__truediv__",
            [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(self, other);
            },
            py::is_operator())
        .def(
            "__truediv__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(self, wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__rtruediv__",
            [](NonlinearExpressionPtr self, double other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(wrapInExpression(other), self);
            },
            py::is_operator())
        .def(
            "__truediv__",
            [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionDivide>(self, wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__pow__",
            [](NonlinearExpressionPtr self, double exponent) -> NonlinearExpressionPtr {
                if(exponent == 2.0)
                    return std::make_shared<ExpressionSquare>(self);
                return std::make_shared<ExpressionPower>(self, wrapInExpression(exponent));
            },
            py::is_operator())
        .def(
            "__pow__",
            [](NonlinearExpressionPtr self, NonlinearExpressionPtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionPower>(self, other);
            },
            py::is_operator())
        .def(
            "__pow__",
            [](NonlinearExpressionPtr self, VariablePtr other) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionPower>(self, wrapInExpression(other));
            },
            py::is_operator())
        .def(
            "__neg__",
            [](NonlinearExpressionPtr self) -> NonlinearExpressionPtr {
                return std::make_shared<ExpressionNegate>(self);
            },
            py::is_operator());

    // ===== NonlinearExpression Type Enum =====
    py::enum_<E_NonlinearExpressionTypes>(m, "ExpressionType")
        .value("Constant", E_NonlinearExpressionTypes::Constant)
        .value("Var", E_NonlinearExpressionTypes::Variable) // Renamed to avoid conflict with Variable class
        .value("Negate", E_NonlinearExpressionTypes::Negate)
        .value("Invert", E_NonlinearExpressionTypes::Invert)
        .value("SquareRoot", E_NonlinearExpressionTypes::SquareRoot)
        .value("Log", E_NonlinearExpressionTypes::Log)
        .value("Exp", E_NonlinearExpressionTypes::Exp)
        .value("Square", E_NonlinearExpressionTypes::Square)
        .value("Cos", E_NonlinearExpressionTypes::Cos)
        .value("Sin", E_NonlinearExpressionTypes::Sin)
        .value("Tan", E_NonlinearExpressionTypes::Tan)
        .value("ArcCos", E_NonlinearExpressionTypes::ArcCos)
        .value("ArcSin", E_NonlinearExpressionTypes::ArcSin)
        .value("ArcTan", E_NonlinearExpressionTypes::ArcTan)
        .value("Abs", E_NonlinearExpressionTypes::Abs)
        .value("Divide", E_NonlinearExpressionTypes::Divide)
        .value("Power", E_NonlinearExpressionTypes::Power)
        .value("Sum", E_NonlinearExpressionTypes::Sum)
        .value("Product", E_NonlinearExpressionTypes::Product);
    // Note: Not using .export_values() to avoid polluting module namespace

    // ===== Nonlinear Expression Helper Functions =====
    m.def(
        "exp",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionExp>(wrapInExpression(var));
        },
        "Exponential function", py::arg("x"));

    m.def(
        "exp",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionExp>(expr); },
        "Exponential function", py::arg("x"));

    m.def(
        "log",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionLog>(wrapInExpression(var));
        },
        "Natural logarithm", py::arg("x"));

    m.def(
        "log",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionLog>(expr); },
        "Natural logarithm", py::arg("x"));

    m.def(
        "sqrt",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionSquareRoot>(wrapInExpression(var));
        },
        "Square root", py::arg("x"));

    m.def(
        "sqrt",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionSquareRoot>(expr);
        },
        "Square root", py::arg("x"));

    m.def(
        "sin",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionSin>(wrapInExpression(var));
        },
        "Sine function", py::arg("x"));

    m.def(
        "sin",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionSin>(expr); },
        "Sine function", py::arg("x"));

    m.def(
        "cos",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionCos>(wrapInExpression(var));
        },
        "Cosine function", py::arg("x"));

    m.def(
        "cos",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionCos>(expr); },
        "Cosine function", py::arg("x"));

    m.def(
        "tan",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionTan>(wrapInExpression(var));
        },
        "Tangent function", py::arg("x"));

    m.def(
        "tan",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionTan>(expr); },
        "Tangent function", py::arg("x"));

    m.def(
        "asin",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionArcSin>(wrapInExpression(var));
        },
        "Arc sine function", py::arg("x"));

    m.def(
        "asin",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionArcSin>(expr); },
        "Arc sine function", py::arg("x"));

    m.def(
        "acos",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionArcCos>(wrapInExpression(var));
        },
        "Arc cosine function", py::arg("x"));

    m.def(
        "acos",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionArcCos>(expr); },
        "Arc cosine function", py::arg("x"));

    m.def(
        "atan",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionArcTan>(wrapInExpression(var));
        },
        "Arc tangent function", py::arg("x"));

    m.def(
        "atan",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionArcTan>(expr); },
        "Arc tangent function", py::arg("x"));

    m.def(
        "abs",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionAbs>(wrapInExpression(var));
        },
        "Absolute value", py::arg("x"));

    m.def(
        "abs",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionAbs>(expr); },
        "Absolute value", py::arg("x"));

    m.def(
        "square",
        [](VariablePtr var) -> NonlinearExpressionPtr {
            return std::make_shared<ExpressionSquare>(wrapInExpression(var));
        },
        "Square function", py::arg("x"));

    m.def(
        "square",
        [](NonlinearExpressionPtr expr) -> NonlinearExpressionPtr { return std::make_shared<ExpressionSquare>(expr); },
        "Square function", py::arg("x"));

    // ===== LinearTerm Class =====
    py::class_<LinearTerm, std::shared_ptr<LinearTerm>>(m, "LinearTerm")
        .def(py::init<double, VariablePtr>(), py::arg("coefficient"), py::arg("variable"))
        .def_readwrite("coefficient", &LinearTerm::coefficient)
        .def_readwrite("variable", &LinearTerm::variable)
        .def("__repr__", [](const LinearTerm& t) {
            return "<LinearTerm: " + std::to_string(t.coefficient) + "*" + t.variable->name + ">";
        });

    // ===== QuadraticTerm Class =====
    py::class_<QuadraticTerm, std::shared_ptr<QuadraticTerm>>(m, "QuadraticTerm")
        .def(py::init<double, VariablePtr, VariablePtr>(), py::arg("coefficient"), py::arg("firstVariable"),
            py::arg("secondVariable"))
        .def_readwrite("coefficient", &QuadraticTerm::coefficient)
        .def_readwrite("firstVariable", &QuadraticTerm::firstVariable)
        .def_readwrite("secondVariable", &QuadraticTerm::secondVariable)
        .def_readonly("isBilinear", &QuadraticTerm::isBilinear)
        .def_readonly("isSquare", &QuadraticTerm::isSquare)
        .def("__repr__", [](const QuadraticTerm& t) {
            return "<QuadraticTerm: " + std::to_string(t.coefficient) + "*" + t.firstVariable->name + "*"
                + t.secondVariable->name + ">";
        });

    // ===== LinearTerms Collection =====
    py::class_<LinearTerms>(m, "LinearTerms")
        .def(py::init<>())
        .def("add", py::overload_cast<LinearTermPtr>(&LinearTerms::add))
        .def("add", py::overload_cast<LinearTerms>(&LinearTerms::add))
        .def("size", [](LinearTerms& self) { return self.size(); })
        .def("__len__", [](LinearTerms& self) { return self.size(); })
        .def("__getitem__", [](LinearTerms& self, size_t i) { return self[i]; });

    // ===== QuadraticTerms Collection =====
    py::class_<QuadraticTerms>(m, "QuadraticTerms")
        .def(py::init<>())
        .def("add", py::overload_cast<QuadraticTermPtr>(&QuadraticTerms::add))
        .def("add", py::overload_cast<QuadraticTerms>(&QuadraticTerms::add))
        .def("size", [](QuadraticTerms& self) { return self.size(); })
        .def("__len__", [](QuadraticTerms& self) { return self.size(); })
        .def("__getitem__", [](QuadraticTerms& self, size_t i) { return self[i]; });

    // ===== SignomialElement Class =====
    py::class_<SignomialElement, std::shared_ptr<SignomialElement>>(m, "SignomialElement")
        .def(py::init<VariablePtr, double>(), py::arg("variable"), py::arg("power"))
        .def_readwrite("variable", &SignomialElement::variable)
        .def_readwrite("power", &SignomialElement::power)
        .def("__repr__", [](const SignomialElement& e) {
            if(e.power == 1.0)
                return "<SignomialElement: " + e.variable->name + ">";
            else
                return "<SignomialElement: " + e.variable->name + "^" + std::to_string(e.power) + ">";
        });

    // ===== SignomialElements Collection =====
    // Note: SignomialElements is a typedef for std::vector<SignomialElementPtr>
    // We expose it as a simple value-type container
    py::class_<SignomialElements>(m, "SignomialElements")
        .def(py::init<>())
        .def("append", [](SignomialElements& self, SignomialElementPtr elem) {
            self.push_back(elem);
        })
        .def("__len__", [](const SignomialElements& self) { return self.size(); })
        .def("__getitem__", [](const SignomialElements& self, size_t i) {
            if (i >= self.size()) throw py::index_error();
            return self[i];
        });

    // ===== SignomialTerm Class =====
    py::class_<SignomialTerm, std::shared_ptr<SignomialTerm>>(m, "SignomialTerm")
        .def(py::init<double, SignomialElements>(), py::arg("coefficient"), py::arg("elements"))
        .def(py::init([](double coeff, std::vector<std::pair<VariablePtr, double>>& varPowerPairs) {
            SignomialElements elements;
            for(auto& [var, power] : varPowerPairs)
            {
                elements.push_back(std::make_shared<SignomialElement>(var, power));
            }
            return std::make_shared<SignomialTerm>(coeff, elements);
        }),
            py::arg("coefficient"), py::arg("variable_power_pairs"),
            "Create a signomial term from coefficient and list of (variable, power) tuples")
        .def_readwrite("coefficient", &SignomialTerm::coefficient)
        .def_readwrite("elements", &SignomialTerm::elements)
        .def("__repr__", [](const SignomialTerm& t) {
            std::ostringstream oss;
            oss << "<SignomialTerm: " << t.coefficient;
            for(auto& e : t.elements)
            {
                oss << " * " << e->variable->name;
                if(e->power != 1.0)
                    oss << "^" << e->power;
            }
            oss << ">";
            return oss.str();
        });

    // ===== SignomialTerms Collection =====
    py::class_<SignomialTerms>(m, "SignomialTerms")
        .def(py::init<>())
        .def("add", py::overload_cast<SignomialTermPtr>(&SignomialTerms::add))
        .def("add", py::overload_cast<SignomialTerms>(&SignomialTerms::add))
        .def("size", [](SignomialTerms& self) { return self.size(); })
        .def("__len__", [](SignomialTerms& self) { return self.size(); })
        .def("__getitem__", [](SignomialTerms& self, size_t i) { return self[i]; });

    // ===== MonomialTerm Class =====
    // Note: MonomialTerm uses Variables (each variable has implicit power 1)
    py::class_<MonomialTerm, std::shared_ptr<MonomialTerm>>(m, "MonomialTerm")
        .def(py::init([](double coeff, std::vector<VariablePtr>& varList) {
            Variables vars;
            for(auto& v : varList)
            {
                vars.push_back(v);
            }
            return std::make_shared<MonomialTerm>(coeff, vars);
        }),
            py::arg("coefficient"), py::arg("variables"),
            "Create a monomial term from coefficient and list of variables")
        .def_readwrite("coefficient", &MonomialTerm::coefficient)
        .def_property_readonly("variables",
            [](const MonomialTerm& t) {
                std::vector<VariablePtr> result;
                for(auto& v : t.variables)
                    result.push_back(v);
                return result;
            })
        .def_readonly("isBilinear", &MonomialTerm::isBilinear)
        .def_readonly("isSquare", &MonomialTerm::isSquare)
        .def_readonly("isBinary", &MonomialTerm::isBinary)
        .def("__repr__", [](const MonomialTerm& t) {
            std::ostringstream oss;
            oss << "<MonomialTerm: " << t.coefficient;
            for(auto& v : t.variables)
            {
                oss << " * " << v->name;
            }
            oss << ">";
            return oss.str();
        });

    // ===== MonomialTerms Collection =====
    py::class_<MonomialTerms>(m, "MonomialTerms")
        .def(py::init<>())
        .def("add", py::overload_cast<MonomialTermPtr>(&MonomialTerms::add))
        .def("add", py::overload_cast<MonomialTerms>(&MonomialTerms::add))
        .def("size", [](MonomialTerms& self) { return self.size(); })
        .def("__len__", [](MonomialTerms& self) { return self.size(); })
        .def("__getitem__", [](MonomialTerms& self, size_t i) { return self[i]; });

    // ===== ConstraintProperties Struct =====
    py::class_<ConstraintProperties>(m, "ConstraintProperties")
        .def_readonly("convexity", &ConstraintProperties::convexity)
        .def_readonly("hasLinearTerms", &ConstraintProperties::hasLinearTerms)
        .def_readonly("hasQuadraticTerms", &ConstraintProperties::hasQuadraticTerms)
        .def_readonly("hasMonomialTerms", &ConstraintProperties::hasMonomialTerms)
        .def_readonly("hasSignomialTerms", &ConstraintProperties::hasSignomialTerms)
        .def_readonly("hasNonlinearExpression", &ConstraintProperties::hasNonlinearExpression);

    // ===== NumericConstraint Base Class =====
    py::class_<NumericConstraint, std::shared_ptr<NumericConstraint>>(m, "NumericConstraint")
        .def_readwrite("index", &NumericConstraint::index)
        .def_readwrite("name", &NumericConstraint::name)
        .def_readwrite("valueLHS", &NumericConstraint::valueLHS)
        .def_readwrite("valueRHS", &NumericConstraint::valueRHS)
        .def_readwrite("constant", &NumericConstraint::constant)
        .def_readonly("properties", &NumericConstraint::properties)
        .def(
            "calculateGradient",
            [](NumericConstraint& self, const std::vector<double>& point) {
                auto gradient = self.calculateGradient(point, true);
                std::map<int, double> result;
                for(auto& G : gradient)
                    result[G.first->index] = G.second;
                return result;
            },
            py::arg("point"), "Calculate gradient at point, returns dict of {var_index: value}")
        .def(
            "calculateHessian",
            [](NumericConstraint& self, const std::vector<double>& point) {
                auto hessian = self.calculateHessian(point, true);
                std::map<std::pair<int, int>, double> result;
                for(auto& H : hessian)
                    result[std::make_pair(H.first.first->index, H.first.second->index)] = H.second;
                return result;
            },
            py::arg("point"), "Calculate Hessian at point, returns dict of {(var1_index, var2_index): value}")
        .def(
            "getGradientSparsityPattern",
            [](NumericConstraint& self) {
                auto pattern = self.getGradientSparsityPattern();
                std::vector<int> result;
                for(auto& V : *pattern)
                    result.push_back(V->index);
                return result;
            },
            "Get gradient sparsity pattern as list of variable indices")
        .def(
            "getHessianSparsityPattern",
            [](NumericConstraint& self) {
                auto pattern = self.getHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->index, E.second->index));
                return result;
            },
            "Get Hessian sparsity pattern as list of (var1_index, var2_index)");

    // ===== LinearConstraint Class =====
    py::class_<LinearConstraint, NumericConstraint, std::shared_ptr<LinearConstraint>>(m, "LinearConstraint")
        .def(py::init<int, std::string, double, double>(), py::arg("index"), py::arg("name"), py::arg("lhs"),
            py::arg("rhs"))
        .def(py::init<int, std::string, LinearTerms, double, double>(), py::arg("index"), py::arg("name"),
            py::arg("linearTerms"), py::arg("lhs"), py::arg("rhs"))
        .def_readwrite("linearTerms", &LinearConstraint::linearTerms)
        .def("add", py::overload_cast<LinearTerms>(&LinearConstraint::add))
        .def("add", py::overload_cast<LinearTermPtr>(&LinearConstraint::add))
        .def("__repr__", [](LinearConstraintPtr c) {
            std::ostringstream oss;
            oss << *c;
            return "<LinearConstraint '" + c->name + "': " + oss.str() + ">";
        });

    // ===== QuadraticConstraint Class =====
    py::class_<QuadraticConstraint, LinearConstraint, std::shared_ptr<QuadraticConstraint>>(m, "QuadraticConstraint")
        .def(py::init<int, std::string, double, double>(), py::arg("index"), py::arg("name"), py::arg("lhs"),
            py::arg("rhs"))
        .def(py::init<int, std::string, LinearTerms, QuadraticTerms, double, double>(), py::arg("index"),
            py::arg("name"), py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("lhs"), py::arg("rhs"))
        .def_readwrite("quadraticTerms", &QuadraticConstraint::quadraticTerms)
        // Inherited add methods from LinearConstraint
        .def("add", py::overload_cast<LinearTerms>(&QuadraticConstraint::add))
        .def("add", py::overload_cast<LinearTermPtr>(&QuadraticConstraint::add))
        // QuadraticConstraint-specific add methods
        .def("add", py::overload_cast<QuadraticTerms>(&QuadraticConstraint::add))
        .def("add", py::overload_cast<QuadraticTermPtr>(&QuadraticConstraint::add))
        .def("__repr__", [](QuadraticConstraintPtr c) {
            std::ostringstream oss;
            oss << *c;
            return "<QuadraticConstraint '" + c->name + "': " + oss.str() + ">";
        });

    // ===== NonlinearConstraint Class =====
    py::class_<NonlinearConstraint, QuadraticConstraint, std::shared_ptr<NonlinearConstraint>>(m, "NonlinearConstraint")
        .def(py::init<int, std::string, double, double>(), py::arg("index"), py::arg("name"), py::arg("lhs"),
            py::arg("rhs"))
        .def(py::init<int, std::string, NonlinearExpressionPtr, double, double>(), py::arg("index"), py::arg("name"),
            py::arg("expression"), py::arg("lhs"), py::arg("rhs"))
        .def(py::init<int, std::string, LinearTerms, NonlinearExpressionPtr, double, double>(), py::arg("index"),
            py::arg("name"), py::arg("linearTerms"), py::arg("expression"), py::arg("lhs"), py::arg("rhs"))
        .def(py::init<int, std::string, LinearTerms, QuadraticTerms, NonlinearExpressionPtr, double, double>(),
            py::arg("index"), py::arg("name"), py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("expression"),
            py::arg("lhs"), py::arg("rhs"))
        .def_readwrite("nonlinearExpression", &NonlinearConstraint::nonlinearExpression)
        .def_readwrite("monomialTerms", &NonlinearConstraint::monomialTerms)
        .def_readwrite("signomialTerms", &NonlinearConstraint::signomialTerms)
        // Inherited add methods from LinearConstraint
        .def("add", py::overload_cast<LinearTerms>(&NonlinearConstraint::add))
        .def("add", py::overload_cast<LinearTermPtr>(&NonlinearConstraint::add))
        // Inherited add methods from QuadraticConstraint
        .def("add", py::overload_cast<QuadraticTerms>(&NonlinearConstraint::add))
        .def("add", py::overload_cast<QuadraticTermPtr>(&NonlinearConstraint::add))
        // NonlinearConstraint-specific add methods
        .def("add", py::overload_cast<NonlinearExpressionPtr>(&NonlinearConstraint::add))
        .def("add", py::overload_cast<MonomialTerms>(&NonlinearConstraint::add))
        .def("add", py::overload_cast<MonomialTermPtr>(&NonlinearConstraint::add))
        .def("add", py::overload_cast<SignomialTerms>(&NonlinearConstraint::add))
        .def("add", py::overload_cast<SignomialTermPtr>(&NonlinearConstraint::add))
        .def("__repr__", [](NonlinearConstraintPtr c) {
            std::ostringstream oss;
            oss << *c;
            return "<NonlinearConstraint '" + c->name + "': " + oss.str() + ">";
        });

    // ===== ObjectiveFunctionProperties Struct =====
    py::class_<ObjectiveFunctionProperties>(m, "ObjectiveFunctionProperties")
        .def_readonly("isMinimize", &ObjectiveFunctionProperties::isMinimize)
        .def_readonly("isMaximize", &ObjectiveFunctionProperties::isMaximize)
        .def_readonly("convexity", &ObjectiveFunctionProperties::convexity)
        .def_readonly("hasLinearTerms", &ObjectiveFunctionProperties::hasLinearTerms)
        .def_readonly("hasQuadraticTerms", &ObjectiveFunctionProperties::hasQuadraticTerms)
        .def_readonly("hasMonomialTerms", &ObjectiveFunctionProperties::hasMonomialTerms)
        .def_readonly("hasSignomialTerms", &ObjectiveFunctionProperties::hasSignomialTerms)
        .def_readonly("hasNonlinearExpression", &ObjectiveFunctionProperties::hasNonlinearExpression);

    // ===== ObjectiveFunction Base Class =====
    py::class_<ObjectiveFunction, std::shared_ptr<ObjectiveFunction>>(m, "ObjectiveFunction")
        .def_readwrite("direction", &ObjectiveFunction::direction)
        .def_readwrite("constant", &ObjectiveFunction::constant)
        .def_readonly("properties", &ObjectiveFunction::properties)
        .def(
            "calculateGradient",
            [](ObjectiveFunction& self, const std::vector<double>& point) {
                auto gradient = self.calculateGradient(point, true);
                std::map<int, double> result;
                for(auto& G : gradient)
                    result[G.first->index] = G.second;
                return result;
            },
            py::arg("point"), "Calculate gradient at point, returns dict of {var_index: value}")
        .def(
            "calculateHessian",
            [](ObjectiveFunction& self, const std::vector<double>& point) {
                auto hessian = self.calculateHessian(point, true);
                std::map<std::pair<int, int>, double> result;
                for(auto& H : hessian)
                    result[std::make_pair(H.first.first->index, H.first.second->index)] = H.second;
                return result;
            },
            py::arg("point"), "Calculate Hessian at point, returns dict of {(var1_index, var2_index): value}")
        .def(
            "getGradientSparsityPattern",
            [](ObjectiveFunction& self) {
                auto pattern = self.getGradientSparsityPattern();
                std::vector<int> result;
                for(auto& V : *pattern)
                    result.push_back(V->index);
                return result;
            },
            "Get gradient sparsity pattern as list of variable indices")
        .def(
            "getHessianSparsityPattern",
            [](ObjectiveFunction& self) {
                auto pattern = self.getHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->index, E.second->index));
                return result;
            },
            "Get Hessian sparsity pattern as list of (var1_index, var2_index)");

    // ===== LinearObjectiveFunction Class =====
    py::class_<LinearObjectiveFunction, ObjectiveFunction, std::shared_ptr<LinearObjectiveFunction>>(
        m, "LinearObjectiveFunction")
        .def(py::init<E_ObjectiveFunctionDirection>(), py::arg("direction"))
        .def(py::init<E_ObjectiveFunctionDirection, double>(), py::arg("direction"), py::arg("constant"))
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, double>(), py::arg("direction"),
            py::arg("linearTerms"), py::arg("constant"))
        .def_readwrite("linearTerms", &LinearObjectiveFunction::linearTerms)
        .def("add", py::overload_cast<LinearTerms>(&LinearObjectiveFunction::add))
        .def("add", py::overload_cast<LinearTermPtr>(&LinearObjectiveFunction::add));

    // ===== QuadraticObjectiveFunction Class =====
    py::class_<QuadraticObjectiveFunction, LinearObjectiveFunction, std::shared_ptr<QuadraticObjectiveFunction>>(
        m, "QuadraticObjectiveFunction")
        .def(py::init<E_ObjectiveFunctionDirection>(), py::arg("direction"))
        .def(py::init<E_ObjectiveFunctionDirection, double>(), py::arg("direction"), py::arg("constant"))
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, QuadraticTerms, double>(), py::arg("direction"),
            py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("constant"))
        .def_readwrite("quadraticTerms", &QuadraticObjectiveFunction::quadraticTerms)
        // Inherited add methods from LinearObjectiveFunction
        .def("add", py::overload_cast<LinearTerms>(&QuadraticObjectiveFunction::add))
        .def("add", py::overload_cast<LinearTermPtr>(&QuadraticObjectiveFunction::add))
        // QuadraticObjectiveFunction-specific add methods
        .def("add", py::overload_cast<QuadraticTerms>(&QuadraticObjectiveFunction::add))
        .def("add", py::overload_cast<QuadraticTermPtr>(&QuadraticObjectiveFunction::add));

    // ===== NonlinearObjectiveFunction Class =====
    py::class_<NonlinearObjectiveFunction, QuadraticObjectiveFunction, std::shared_ptr<NonlinearObjectiveFunction>>(
        m, "NonlinearObjectiveFunction")
        .def(py::init<E_ObjectiveFunctionDirection>(), py::arg("direction"))
        .def(py::init<E_ObjectiveFunctionDirection, double>(), py::arg("direction"), py::arg("constant"))
        .def(py::init<E_ObjectiveFunctionDirection, NonlinearExpressionPtr, double>(), py::arg("direction"),
            py::arg("expression"), py::arg("constant"))
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, NonlinearExpressionPtr, double>(),
            py::arg("direction"), py::arg("linearTerms"), py::arg("expression"), py::arg("constant"))
        .def(py::init<E_ObjectiveFunctionDirection, LinearTerms, QuadraticTerms, NonlinearExpressionPtr, double>(),
            py::arg("direction"), py::arg("linearTerms"), py::arg("quadraticTerms"), py::arg("expression"),
            py::arg("constant"))
        .def_readwrite("nonlinearExpression", &NonlinearObjectiveFunction::nonlinearExpression)
        .def_readwrite("monomialTerms", &NonlinearObjectiveFunction::monomialTerms)
        .def_readwrite("signomialTerms", &NonlinearObjectiveFunction::signomialTerms)
        .def_readonly("variablesInNonlinearExpression", &NonlinearObjectiveFunction::variablesInNonlinearExpression)
        .def_readonly("nonlinearExpressionIndex", &NonlinearObjectiveFunction::nonlinearExpressionIndex)
        // Inherited add methods from LinearObjectiveFunction
        .def("add", py::overload_cast<LinearTerms>(&NonlinearObjectiveFunction::add))
        .def("add", py::overload_cast<LinearTermPtr>(&NonlinearObjectiveFunction::add))
        // Inherited add methods from QuadraticObjectiveFunction
        .def("add", py::overload_cast<QuadraticTerms>(&NonlinearObjectiveFunction::add))
        .def("add", py::overload_cast<QuadraticTermPtr>(&NonlinearObjectiveFunction::add))
        // NonlinearObjectiveFunction-specific add methods
        .def("add", py::overload_cast<NonlinearExpressionPtr>(&NonlinearObjectiveFunction::add))
        .def("add", py::overload_cast<MonomialTerms>(&NonlinearObjectiveFunction::add))
        .def("add", py::overload_cast<MonomialTermPtr>(&NonlinearObjectiveFunction::add))
        .def("add", py::overload_cast<SignomialTerms>(&NonlinearObjectiveFunction::add))
        .def("add", py::overload_cast<SignomialTermPtr>(&NonlinearObjectiveFunction::add));

    // ===== ProblemProperties Struct =====
    py::class_<ProblemProperties>(m, "ProblemProperties")
        .def_readonly("isValid", &ProblemProperties::isValid)
        .def_readonly("convexity", &ProblemProperties::convexity)
        .def_readonly("isNonlinear", &ProblemProperties::isNonlinear)
        .def_readonly("isDiscrete", &ProblemProperties::isDiscrete)
        .def_readonly("isMINLPProblem", &ProblemProperties::isMINLPProblem)
        .def_readonly("isNLPProblem", &ProblemProperties::isNLPProblem)
        .def_readonly("isMIQPProblem", &ProblemProperties::isMIQPProblem)
        .def_readonly("isQPProblem", &ProblemProperties::isQPProblem)
        .def_readonly("isMIQCQPProblem", &ProblemProperties::isMIQCQPProblem)
        .def_readonly("isQCQPProblem", &ProblemProperties::isQCQPProblem)
        .def_readonly("isMILPProblem", &ProblemProperties::isMILPProblem)
        .def_readonly("isLPProblem", &ProblemProperties::isLPProblem)
        .def_readonly("numberOfVariables", &ProblemProperties::numberOfVariables)
        .def_readonly("numberOfRealVariables", &ProblemProperties::numberOfRealVariables)
        .def_readonly("numberOfDiscreteVariables", &ProblemProperties::numberOfDiscreteVariables)
        .def_readonly("numberOfBinaryVariables", &ProblemProperties::numberOfBinaryVariables)
        .def_readonly("numberOfIntegerVariables", &ProblemProperties::numberOfIntegerVariables)
        .def_readonly("numberOfNumericConstraints", &ProblemProperties::numberOfNumericConstraints)
        .def_readonly("numberOfLinearConstraints", &ProblemProperties::numberOfLinearConstraints)
        .def_readonly("numberOfQuadraticConstraints", &ProblemProperties::numberOfQuadraticConstraints)
        .def_readonly("numberOfConvexQuadraticConstraints", &ProblemProperties::numberOfConvexQuadraticConstraints)
        .def_readonly(
            "numberOfNonconvexQuadraticConstraints", &ProblemProperties::numberOfNonconvexQuadraticConstraints)
        .def_readonly("numberOfNonlinearConstraints", &ProblemProperties::numberOfNonlinearConstraints)
        .def_readonly("numberOfConvexNonlinearConstraints", &ProblemProperties::numberOfConvexNonlinearConstraints)
        .def_readonly(
            "numberOfNonconvexNonlinearConstraints", &ProblemProperties::numberOfNonconvexNonlinearConstraints)
        .def_readonly(
            "numberOfVariablesInNonlinearExpressions", &ProblemProperties::numberOfVariablesInNonlinearExpressions)
        .def_readonly("numberOfNonlinearExpressions", &ProblemProperties::numberOfNonlinearExpressions)
        .def_readonly("name", &ProblemProperties::name)
        .def_readonly("description", &ProblemProperties::description)
        .def_readonly("isReformulated", &ProblemProperties::isReformulated);

    // ===== Problem Class =====
    // Problem uses enable_shared_from_this, pybind11 handles this automatically
    // when we specify shared_ptr as the holder type
    py::class_<Problem, std::shared_ptr<Problem>>(m, "Problem")
        .def(py::init<EnvironmentPtr>(), py::arg("environment"))
        .def_readwrite("name", &Problem::name)
        .def_readonly("properties", &Problem::properties)
        .def_readonly("allVariables", &Problem::allVariables)
        .def_readonly("realVariables", &Problem::realVariables)
        .def_readonly("binaryVariables", &Problem::binaryVariables)
        .def_readonly("integerVariables", &Problem::integerVariables)
        .def_readonly("nonlinearExpressionVariables", &Problem::nonlinearExpressionVariables)
        .def_readonly("objectiveFunction", &Problem::objectiveFunction)
        .def_readonly("linearConstraints", &Problem::linearConstraints)
        .def_readonly("quadraticConstraints", &Problem::quadraticConstraints)
        .def_readonly("nonlinearConstraints", &Problem::nonlinearConstraints)
        .def_readonly("numericConstraints", &Problem::numericConstraints)
        // Add methods - using lambdas since these are separate method overloads
        .def(
            "addVariable", [](Problem& self, VariablePtr var) { self.add(var); }, py::arg("variable"))
        .def(
            "addVariables", [](Problem& self, Variables vars) { self.add(vars); }, py::arg("variables"))
        // Order matters for pybind11 overload resolution - most specific types first
        .def(
            "addConstraint", [](Problem& self, NonlinearConstraintPtr c) { self.add(c); }, py::arg("constraint"))
        .def(
            "addConstraint", [](Problem& self, QuadraticConstraintPtr c) { self.add(c); }, py::arg("constraint"))
        .def(
            "addConstraint", [](Problem& self, LinearConstraintPtr c) { self.add(c); }, py::arg("constraint"))
        .def(
            "addConstraint", [](Problem& self, NumericConstraintPtr c) { self.add(c); }, py::arg("constraint"))
        // Order matters for pybind11 overload resolution - most specific types first
        .def(
            "setObjective", [](Problem& self, NonlinearObjectiveFunctionPtr obj) { self.add(obj); },
            py::arg("objective"))
        .def(
            "setObjective", [](Problem& self, QuadraticObjectiveFunctionPtr obj) { self.add(obj); },
            py::arg("objective"))
        .def(
            "setObjective", [](Problem& self, LinearObjectiveFunctionPtr obj) { self.add(obj); }, py::arg("objective"))
        .def(
            "setObjective", [](Problem& self, ObjectiveFunctionPtr obj) { self.add(obj); }, py::arg("objective"))
        .def(
            "setObjective", [](Problem& self, NonlinearObjectiveFunctionPtr obj) { self.add(obj); },
            py::arg("objective"))
        // Finalize: simplify expressions, extract terms (linear, quadratic, monomial, signomial),
        // update properties, and prepare factorable functions
        .def(
            "finalize", [](Problem& self) { self.finalize(); },
            "Finalize the problem: extract terms from expressions, update properties, and prepare for solving")
        .def("updateProperties", &Problem::updateProperties, "Update problem properties")
        // Getters
        .def("getVariable", &Problem::getVariable, py::arg("index"))
        .def("getConstraint", &Problem::getConstraint, py::arg("index"), "Get constraint by index")
        .def("getVariableLowerBound", &Problem::getVariableLowerBound, py::arg("index"))
        .def("getVariableUpperBound", &Problem::getVariableUpperBound, py::arg("index"))
        .def("getVariableLowerBounds", &Problem::getVariableLowerBounds)
        .def("getVariableUpperBounds", &Problem::getVariableUpperBounds)
        // Sparsity patterns
        .def(
            "getConstraintsJacobianSparsityPattern",
            [](Problem& self) {
                auto pattern = self.getConstraintsJacobianSparsityPattern();
                std::vector<std::pair<int, std::vector<int>>> result;
                for(auto& E : *pattern)
                {
                    std::vector<int> varIndices;
                    for(auto& V : E.second)
                        varIndices.push_back(V->index);
                    result.push_back(std::make_pair(E.first->index, varIndices));
                }
                return result;
            },
            "Get Jacobian sparsity pattern as list of (constraint_index, [variable_indices])")
        .def(
            "getConstraintsHessianSparsityPattern",
            [](Problem& self) {
                auto pattern = self.getConstraintsHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->index, E.second->index));
                return result;
            },
            "Get Hessian sparsity pattern for constraints only as list of (var1_index, var2_index)")
        .def(
            "getLagrangianHessianSparsityPattern",
            [](Problem& self) {
                auto pattern = self.getLagrangianHessianSparsityPattern();
                std::vector<std::pair<int, int>> result;
                for(auto& E : *pattern)
                    result.push_back(std::make_pair(E.first->index, E.second->index));
                return result;
            },
            "Get Hessian sparsity pattern including objective as list of (var1_index, var2_index)")
        // String representation
        .def("__repr__",
            [](ProblemPtr p) {
                std::string repr = "<Problem";
                if(!p->name.empty())
                    repr += " '" + p->name + "'";
                repr += " vars=" + std::to_string(p->properties.numberOfVariables);
                repr += " constrs=" + std::to_string(p->properties.numberOfNumericConstraints);
                repr += ">";
                return repr;
            })
        .def("__str__",
            [](ProblemPtr p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
        .def(
            "toString",
            [](ProblemPtr p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            },
            "Get the full string representation of the problem");

    // ===== Variables Collection =====
    py::class_<Variables>(m, "Variables")
        .def(py::init<>())
        .def("size", [](Variables& self) { return self.size(); })
        .def("__len__", [](Variables& self) { return self.size(); })
        .def("__getitem__", [](Variables& self, size_t i) { return self[i]; })
        .def(
            "__iter__", [](Variables& self) { return py::make_iterator(self.begin(), self.end()); },
            py::keep_alive<0, 1>());

    // ===== Environment Class =====
    py::class_<Environment, std::shared_ptr<Environment>>(m, "Environment")
        .def_readonly("problem", &Environment::problem)
        .def_readonly("reformulatedProblem", &Environment::reformulatedProblem);

    // ===== Solver Class =====
    py::class_<Solver>(m, "Solver")
        .def(py::init())
        .def("getEnvironment", &Solver::getEnvironment)
        .def("getOriginalProblem", &Solver::getOriginalProblem)
        .def("getReformulatedProblem", &Solver::getReformulatedProblem)
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
        .def("setProblem", py::overload_cast<std::string>(&Solver::setProblem), "Load problem from file",
            py::arg("filename"))
        .def(
            "setProblem", [](Solver& self, ProblemPtr problem) { return self.setProblem(problem, nullptr, nullptr); },
            "Set problem from Problem object", py::arg("problem"))
        .def(
            "setProblem",
            [](Solver& self, ProblemPtr problem, ProblemPtr reformulatedProblem) {
                return self.setProblem(problem, reformulatedProblem, nullptr);
            },
            "Set problem with reformulated problem", py::arg("problem"), py::arg("reformulatedProblem"))
        .def("solveProblem", &Solver::solveProblem)
        .def("updateLogLevels", &Solver::updateLogLevels)
        .def("updateSetting", py::overload_cast<std::string, std::string, bool>(&Solver::updateSetting))
        .def("updateSetting", py::overload_cast<std::string, std::string, int>(&Solver::updateSetting))
        .def("updateSetting", py::overload_cast<std::string, std::string, std::string>(&Solver::updateSetting))
        .def("updateSetting", py::overload_cast<std::string, std::string, double>(&Solver::updateSetting))
        .def(
            "registerCallback",
            [](Solver& self, E_EventType event, py::function callback) {
                switch(event)
                {
                case E_EventType::NewPrimalSolution:
                    self.registerCallback(event, [callback](std::any args) {
                        py::gil_scoped_acquire gil;
                        auto data = std::any_cast<PrimalSolutionCallbackData>(args);
                        callback(data);
                    });
                    break;
                case E_EventType::PrimalSolutionCandidateSelection:
                    self.registerCallback(event, [callback](std::any args) -> bool {
                        py::gil_scoped_acquire gil;
                        auto data = std::any_cast<PrimalSolutionCallbackData>(args);
                        py::object result = callback(data);
                        if(result.is_none())
                            return true; // None means accept
                        return result.cast<bool>();
                    });
                    break;
                case E_EventType::UserTerminationCheck:
                    self.registerCallback(event, [callback](std::any args) -> bool {
                        py::gil_scoped_acquire gil;
                        auto data = std::any_cast<TerminationCallbackData>(args);
                        py::object result = callback(data);
                        if(result.is_none())
                            return false;
                        return result.cast<bool>();
                    });
                    break;
                case E_EventType::ExternalDualBound:
                    self.registerCallback(event, [callback](std::any args) -> double {
                        py::gil_scoped_acquire gil;
                        auto data = std::any_cast<DualBoundCallbackData>(args);
                        py::object result = callback(data);
                        if(result.is_none())
                            return std::numeric_limits<double>::quiet_NaN();
                        return result.cast<double>();
                    });
                    break;
                case E_EventType::ExternalHyperplaneSelection:
                    self.registerCallback(
                        event, [callback](std::any args) -> std::vector<ExternalHyperplane> {
                            py::gil_scoped_acquire gil;
                            auto data
                                = std::any_cast<ExternalHyperplaneSelectionCallbackData>(args);
                            py::object result = callback(data);
                            if(result.is_none())
                                return {};
                            return result.cast<std::vector<ExternalHyperplane>>();
                        });
                    break;
                case E_EventType::ExternalPrimalSolution:
                    self.registerCallback(event, [callback](std::any args) -> std::vector<VectorDouble> {
                        py::gil_scoped_acquire gil;
                        auto data = std::any_cast<ExternalPrimalSolutionCallbackData>(args);
                        py::object result = callback(data);
                        if(result.is_none())
                            return {};
                        auto point = result.cast<VectorDouble>();
                        if(point.empty())
                            return {};
                        return { point }; // wrap single solution in a vector as the task expects
                    });
                    break;
                case E_EventType::ExternalESHRootsearchPointsSelection:
                    self.registerCallback(
                        event, [callback](std::any args) -> std::vector<VectorDouble> {
                            py::gil_scoped_acquire gil;
                            auto data = std::any_cast<ESHInteriorPointCallbackData>(args);
                            py::object result = callback(data);
                            if(result.is_none())
                                return {};
                            return result.cast<std::vector<VectorDouble>>();
                        });
                    break;
                default:
                    throw std::invalid_argument("Unknown event type for registerCallback");
                }
            },
            "Register a Python callback for an event type.\n\n"
            "Callback signatures by event type:\n"
            "  EventType.NewPrimalSolution: fn(PrimalSolutionCallbackData) -> None\n"
            "  EventType.PrimalSolutionCandidateSelection: fn(PrimalSolutionCallbackData) -> bool\n"
            "    Return False to reject the candidate (skip feasibility check). None or True to accept.\n"
            "  EventType.UserTerminationCheck: fn(TerminationCallbackData) -> bool\n"
            "    Return True to stop, False to continue. Return None to continue.\n"
            "  EventType.ExternalDualBound: fn(DualBoundCallbackData) -> float\n"
            "    Return a new dual bound value, or None to skip.\n"
            "  EventType.ExternalHyperplaneSelection: fn(ExternalHyperplaneSelectionCallbackData) -> list[ExternalHyperplane]\n"
            "    Return a list of hyperplanes to add, or None/[] to add none.\n"
            "  EventType.ExternalPrimalSolution: fn(ExternalPrimalSolutionCallbackData) -> list[float]\n"
            "    Return a new primal solution point, or None/[] to skip.\n"
            "  EventType.ExternalESHRootsearchPointsSelection: fn(ESHInteriorPointCallbackData) -> list[list[float]]\n"
            "    Return a replacement list of interior point vectors, or None/[] to keep current points.",
            py::arg("event"), py::arg("callback"));

    py::enum_<E_EventType>(m, "EventType")
        .value("ExternalDualBound", E_EventType::ExternalDualBound)
        .value("ExternalHyperplaneSelection", E_EventType::ExternalHyperplaneSelection)
        .value("ExternalPrimalSolution", E_EventType::ExternalPrimalSolution)
        .value("ExternalESHRootsearchPointsSelection", E_EventType::ExternalESHRootsearchPointsSelection)
        .value("NewPrimalSolution", E_EventType::NewPrimalSolution)
        .value("PrimalSolutionCandidateSelection", E_EventType::PrimalSolutionCandidateSelection)
        .value("UserTerminationCheck", E_EventType::UserTerminationCheck);

    py::enum_<E_HyperplaneSource>(m, "HyperplaneSource", py::arithmetic())
        .value("None", E_HyperplaneSource::None)
        .value("MIPOptimalRootsearch", E_HyperplaneSource::MIPOptimalRootsearch)
        .value("MIPSolutionPoolRootsearch", E_HyperplaneSource::MIPSolutionPoolRootsearch)
        .value("LPRelaxedRootsearch", E_HyperplaneSource::LPRelaxedRootsearch)
        .value("MIPOptimalSolutionPoint", E_HyperplaneSource::MIPOptimalSolutionPoint)
        .value("MIPSolutionPoolSolutionPoint", E_HyperplaneSource::MIPSolutionPoolSolutionPoint)
        .value("LPRelaxedSolutionPoint", E_HyperplaneSource::LPRelaxedSolutionPoint)
        .value("LPFixedIntegers", E_HyperplaneSource::LPFixedIntegers)
        .value("PrimalSolutionSearch", E_HyperplaneSource::PrimalSolutionSearch)
        .value("PrimalSolutionSearchInteriorObjective",
            E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
        .value("InteriorPointSearch", E_HyperplaneSource::InteriorPointSearch)
        .value("MIPCallbackRelaxed", E_HyperplaneSource::MIPCallbackRelaxed)
        .value("ObjectiveRootsearch", E_HyperplaneSource::ObjectiveRootsearch)
        .value("ObjectiveCuttingPlane", E_HyperplaneSource::ObjectiveCuttingPlane)
        .value("External", E_HyperplaneSource::External);

    py::enum_<E_PrimalSolutionSource>(m, "PrimalSolutionSource", py::arithmetic())
        .value("Rootsearch", E_PrimalSolutionSource::Rootsearch)
        .value("RootsearchFixedIntegers", E_PrimalSolutionSource::RootsearchFixedIntegers)
        .value("NLPFixedIntegers", E_PrimalSolutionSource::NLPFixedIntegers)
        .value("NLPRelaxed", E_PrimalSolutionSource::NLPRelaxed)
        .value("MIPSolutionPool", E_PrimalSolutionSource::MIPSolutionPool)
        .value("LPFixedIntegers", E_PrimalSolutionSource::LPFixedIntegers)
        .value("MIPCallback", E_PrimalSolutionSource::MIPCallback)
        .value("InteriorPointSearch", E_PrimalSolutionSource::InteriorPointSearch);

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
        .value("ErrorNoSolution", E_ModelReturnStatus::ErrorNoSolution);

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
        .value("NumericIssues", E_TerminationReason::NumericIssues);

    py::class_<PairIndexValue>(m, "PairIndexValue")
        .def_readwrite("index", &PairIndexValue::index)
        .def_readwrite("value", &PairIndexValue::value);

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
        .def_readwrite("displayed", &PrimalSolution::displayed);

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
        .def_readwrite(
            "numberOfConstraintsRemovedInPresolve", &SolutionStatistics::numberOfConstraintsRemovedInPresolve)
        .def_readwrite(
            "numberOfVariableBoundsTightenedInPresolve", &SolutionStatistics::numberOfVariableBoundsTightenedInPresolve)
        .def_readwrite("numberOfHyperplanesWithConvexSource", &SolutionStatistics::numberOfHyperplanesWithConvexSource)
        .def_readwrite(
            "numberOfHyperplanesWithNonconvexSource", &SolutionStatistics::numberOfHyperplanesWithNonconvexSource)
        .def_readwrite("numberOfIntegerCuts", &SolutionStatistics::numberOfIntegerCuts)
        .def_readwrite(
            "numberOfIterationsWithDualStagnation", &SolutionStatistics::numberOfIterationsWithDualStagnation)
        .def_readwrite(
            "lastIterationWithSignificantDualUpdate", &SolutionStatistics::lastIterationWithSignificantDualUpdate)
        .def_readwrite(
            "numberOfIterationsWithPrimalStagnation", &SolutionStatistics::numberOfIterationsWithPrimalStagnation)
        .def_readwrite(
            "lastIterationWithSignificantPrimalUpdate", &SolutionStatistics::lastIterationWithSignificantPrimalUpdate)
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
        .def_readwrite("numberOfPrimalReductionCutsUpdatesWithoutEffect",
            &SolutionStatistics::numberOfPrimalReductionCutsUpdatesWithoutEffect)
        .def_readwrite(
            "numberOfDualRepairsSinceLastPrimalUpdate", &SolutionStatistics::numberOfDualRepairsSinceLastPrimalUpdate)
        .def_readwrite("numberOfPrimalReductionsPerformed", &SolutionStatistics::numberOfPrimalReductionsPerformed)
        .def_readwrite(
            "numberOfSuccessfulDualRepairsPerformed", &SolutionStatistics::numberOfSuccessfulDualRepairsPerformed)
        .def_readwrite(
            "numberOfUnsuccessfulDualRepairsPerformed", &SolutionStatistics::numberOfUnsuccessfulDualRepairsPerformed)
        .def_readwrite("numberOfPrimalImprovementsAfterInfeasibilityRepair",
            &SolutionStatistics::numberOfPrimalImprovementsAfterInfeasibilityRepair)
        .def_readwrite("numberOfPrimalImprovementsAfterReductionCut",
            &SolutionStatistics::numberOfPrimalImprovementsAfterReductionCut)
        .def_readwrite("hasInfeasibilityRepairBeenPerformedSincePrimalImprovement",
            &SolutionStatistics::hasInfeasibilityRepairBeenPerformedSincePrimalImprovement)
        .def_readwrite("hasReductionCutBeenAddedSincePrimalImprovement",
            &SolutionStatistics::hasReductionCutBeenAddedSincePrimalImprovement)
        .def("getNumberOfTotalDualProblems", &SolutionStatistics::getNumberOfTotalDualProblems);

    // -------------------------------------------------------------------------
    // Supporting types for callbacks
    // -------------------------------------------------------------------------

    py::class_<SolutionPoint>(m, "SolutionPoint")
        .def(py::init<>())
        .def_readwrite("point", &SolutionPoint::point)
        .def_readwrite("objectiveValue", &SolutionPoint::objectiveValue)
        .def_readwrite("iterFound", &SolutionPoint::iterFound)
        .def_readwrite("maxDeviation", &SolutionPoint::maxDeviation)
        .def_readwrite("isRelaxedPoint", &SolutionPoint::isRelaxedPoint)
        .def_readwrite("hashValue", &SolutionPoint::hashValue);

    // Hyperplane base must be registered before ExternalHyperplane
    py::class_<Hyperplane>(m, "Hyperplane")
        .def_readwrite("source", &Hyperplane::source)
        .def_readwrite("isGlobal", &Hyperplane::isGlobal);

    py::class_<ExternalHyperplane, Hyperplane>(m, "ExternalHyperplane")
        .def(py::init<>())
        .def_readwrite("variableIndexes", &ExternalHyperplane::variableIndexes)
        .def_readwrite("variableCoefficients", &ExternalHyperplane::variableCoefficients)
        .def_readwrite("description", &ExternalHyperplane::description)
        .def_readwrite("rhsValue", &ExternalHyperplane::rhsValue);

    // -------------------------------------------------------------------------
    // Callback data structures (passed to Python callbacks as read-only data)
    // -------------------------------------------------------------------------

    py::class_<DualBoundCallbackData>(m, "DualBoundCallbackData")
        .def_readonly("isMinimization", &DualBoundCallbackData::isMinimization)
        .def_readonly("currentDualBound", &DualBoundCallbackData::currentDualBound)
        .def_readonly("currentPrimalBound", &DualBoundCallbackData::currentPrimalBound)
        .def_readonly("relativeGap", &DualBoundCallbackData::relativeGap)
        .def_readonly("absoluteGap", &DualBoundCallbackData::absoluteGap)
        .def_readonly("iterationNumber", &DualBoundCallbackData::iterationNumber)
        .def_readonly("solutionStatistics", &DualBoundCallbackData::solutionStatistics);

    py::class_<TerminationCallbackData>(m, "TerminationCallbackData")
        .def_readonly("iterationNumber", &TerminationCallbackData::iterationNumber)
        .def_readonly("currentDualBound", &TerminationCallbackData::currentDualBound)
        .def_readonly("currentPrimalBound", &TerminationCallbackData::currentPrimalBound)
        .def_readonly("relativeGap", &TerminationCallbackData::relativeGap)
        .def_readonly("absoluteGap", &TerminationCallbackData::absoluteGap)
        .def_readonly("timeElapsed", &TerminationCallbackData::timeElapsed)
        .def_readonly("solutionStatistics", &TerminationCallbackData::solutionStatistics);

    py::class_<PrimalSolutionCallbackData>(m, "PrimalSolutionCallbackData")
        .def_readonly("isMinimization", &PrimalSolutionCallbackData::isMinimization)
        .def_readonly("solution", &PrimalSolutionCallbackData::solution)
        .def_readonly("objectiveValue", &PrimalSolutionCallbackData::objectiveValue)
        .def_readonly("currentDualBound", &PrimalSolutionCallbackData::currentDualBound)
        .def_readonly("relativeGap", &PrimalSolutionCallbackData::relativeGap)
        .def_readonly("absoluteGap", &PrimalSolutionCallbackData::absoluteGap)
        .def_readonly("iterationNumber", &PrimalSolutionCallbackData::iterationNumber)
        .def_readonly("sourceType", &PrimalSolutionCallbackData::sourceType)
        .def_readonly("solutionStatistics", &PrimalSolutionCallbackData::solutionStatistics);

    py::class_<ExternalPrimalSolutionCallbackData>(m, "ExternalPrimalSolutionCallbackData")
        .def_readonly("isMinimization", &ExternalPrimalSolutionCallbackData::isMinimization)
        .def_readonly("currentDualBound", &ExternalPrimalSolutionCallbackData::currentDualBound)
        .def_readonly("currentPrimalBound", &ExternalPrimalSolutionCallbackData::currentPrimalBound)
        .def_readonly("relativeGap", &ExternalPrimalSolutionCallbackData::relativeGap)
        .def_readonly("absoluteGap", &ExternalPrimalSolutionCallbackData::absoluteGap)
        .def_readonly("iterationNumber", &ExternalPrimalSolutionCallbackData::iterationNumber)
        .def_readonly("currentSolution", &ExternalPrimalSolutionCallbackData::currentSolution)
        .def_readonly("solutionStatistics", &ExternalPrimalSolutionCallbackData::solutionStatistics);

    py::class_<ExternalHyperplaneSelectionCallbackData>(m, "ExternalHyperplaneSelectionCallbackData")
        .def_readonly("isMinimization", &ExternalHyperplaneSelectionCallbackData::isMinimization)
        .def_readonly("iterationNumber", &ExternalHyperplaneSelectionCallbackData::iterationNumber)
        .def_readonly("currentDualBound", &ExternalHyperplaneSelectionCallbackData::currentDualBound)
        .def_readonly("currentPrimalBound", &ExternalHyperplaneSelectionCallbackData::currentPrimalBound)
        .def_readonly("relativeGap", &ExternalHyperplaneSelectionCallbackData::relativeGap)
        .def_readonly("absoluteGap", &ExternalHyperplaneSelectionCallbackData::absoluteGap)
        .def_readonly("solutionPoints", &ExternalHyperplaneSelectionCallbackData::solutionPoints)
        .def_readonly("originalProblem", &ExternalHyperplaneSelectionCallbackData::originalProblem)
        .def_readonly(
            "reformulatedProblem", &ExternalHyperplaneSelectionCallbackData::reformulatedProblem)
        .def_readonly(
            "isObjectiveNonlinear", &ExternalHyperplaneSelectionCallbackData::isObjectiveNonlinear)
        .def_readonly(
            "solutionStatistics", &ExternalHyperplaneSelectionCallbackData::solutionStatistics);

    py::class_<ESHInteriorPointCallbackData>(m, "ESHInteriorPointCallbackData")
        .def_readonly("currentInteriorPoints", &ESHInteriorPointCallbackData::currentInteriorPoints)
        .def_readonly("originalProblem", &ESHInteriorPointCallbackData::originalProblem)
        .def_readonly("reformulatedProblem", &ESHInteriorPointCallbackData::reformulatedProblem)
        .def_readonly("solutionStatistics", &ESHInteriorPointCallbackData::solutionStatistics);
}
}
