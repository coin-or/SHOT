/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

namespace SHOT
{
enum class E_DualSolutionSource
{
    LPSolution,
    MIPSolutionOptimal,
    ObjectiveConstraint,
    MIPSolverBound
};

enum class E_HyperplaneSource
{
    MIPOptimalLinesearch,
    MIPSolutionPoolLinesearch,
    LPRelaxedLinesearch,
    MIPOptimalSolutionPoint,
    MIPSolutionPoolSolutionPoint,
    LPRelaxedSolutionPoint,
    LPFixedIntegers,
    PrimalSolutionSearch,
    PrimalSolutionSearchInteriorObjective,
    InteriorPointSearch
};

enum class E_IterationLineType
{
    DualSolution,
    DualCallback,
    DualIntegerFixed,
    PrimalNLP
};

enum class E_IterationProblemType
{
    MIP,
    Relaxed
};

enum class E_PrimalNLPSource
{
    FirstSolution,
    FeasibleSolution,
    InfeasibleSolution,
    SmallestDeviationSolution,
    FirstSolutionNewDualBound
};

enum class E_PrimalSolutionSource
{
    Linesearch,
    LinesearchFixedIntegers,
    NLPFixedIntegers,
    NLPRelaxed,
    MIPSolutionPool,
    ObjectiveConstraint,
    LPFixedIntegers,
    LazyConstraintCallback,
    HeuristicCallback,
    IncumbentCallback
};

enum class E_NLPSolutionStatus
{
    Feasible,
    Optimal,
    Infeasible,
    Unbounded,
    IterationLimit,
    TimeLimit,
    Error
};

enum class E_ObjectiveFunctionType
{
    Linear,
    Quadratic,
    Nonlinear,
    QuadraticConsideredAsNonlinear,
    None
};

enum class E_ProblemSolutionStatus
{
    Feasible,
    Optimal,
    Infeasible,
    Unbounded,
    IterationLimit,
    TimeLimit,
    SolutionLimit,
    Error,
    Numeric,
    CutOff,
    NodeLimit,
    Abort,
    None
};

enum class E_ProblemType
{
    LP,
    QP,
    NLP,
    QCQP,
    MILP,
    MIQP,
    MIQCQP,
    MINLP,
    None
};

enum class E_SolutionStrategy
{
    SingleTree,
    MultiTree,
    NLP,
    MIQP,
    MIQCQP,
    None
};

enum class E_TerminationReason
{
    ConstraintTolerance,
    ObjectiveStagnation,
    IterationLimit,
    TimeLimit,
    InfeasibleProblem,
    UnboundedProblem,
    Error,
    AbsoluteGap,
    RelativeGap,
    InteriorPointError,
    NumericIssues,
    UserAbort,
    ObjectiveGapNotReached,
    None
};

enum class ES_AddPrimalPointAsInteriorPoint
{
    KeepOriginal,
    KeepBoth,
    KeepNew,
    OnlyAverage
};

enum class ES_HyperplaneCutStrategy
{
    ESH,
    ECP
};

enum class ES_InteriorPointStrategy
{
    CuttingPlaneMiniMax,
    IpoptMinimax,
    IpoptRelaxed,
    IpoptMinimaxAndRelaxed
};

enum class ES_IpoptSolver
{
    ma27,
    ma57,
    ma86,
    ma97,
    mumps
};

enum class ES_IterationOutputDetail
{
    Full,
    ObjectiveGapUpdates,
    ObjectiveGapUpdatesAndNLPCalls
};

enum class ES_RootsearchConstraintStrategy
{
    AllAsMaxFunct,
    IndividualConstraints
};

enum class ES_RootsearchMethod
{
    BoostTOMS748,
    BoostBisection,
    Bisection
};

enum class ES_MIPSolver
{
    Cplex,
    Gurobi,
    Cbc,
    None
};

enum class ES_PrimalNLPFixedPoint
{
    AllSolutions,
    FirstSolution,
    AllFeasibleSolutions,
    FirstAndFeasibleSolutions,
    SmallestDeviationSolution
};

enum class ES_PrimalNLPSolver
{
    CuttingPlane,
    Ipopt,
    GAMS,
    None
};

enum class ES_MIPPresolveStrategy
{
    Never,
    Once,
    EveryIteration
};

enum class ES_OutputDirectory
{
    Problem,
    Program
};

enum class ES_PrimalNLPStrategy
{
    AlwaysUse,
    IterationOrTime,
    IterationOrTimeAndAllFeasibleSolutions
};

enum class ES_QuadraticProblemStrategy
{
    Nonlinear,
    QuadraticObjective,
    QuadraticallyConstrained
};

enum class ES_SourceFormat
{
    OSiL,
    GAMS,
    NL,
    None
};

enum class ES_TreeStrategy
{
    MultiTree,
    SingleTree
};
} // namespace SHOT
