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

enum class E_AuxiliaryVariableType
{
    None,
    NonlinearObjectiveFunction, // From epigraph formulation of (nonlinear) objective function
    NonlinearExpressionPartitioning, // From reformulating nonlinear terms as constraints
    MonomialTermsPartitioning, // From reformulating monomial terms as constraints
    SignomialTermsPartitioning, // From reformulating signomial terms as constraints
    SquareTermsPartitioning, // From reformulating sums of square terms
    ContinuousBilinear, // From linearizing a bilinear term x1 * x2 where x1 and x2 are real
    BinaryBilinear, // From linearizing a bilinear term b1 * b2 where b1 and b2 are binary
    BinaryContinuousBilinear, // From linearizing a bilinear term b1 * x2 where b1 is binary and x2 is continuous
    IntegerBilinear, // From linearizing a bilinear term i1 * x2, where i1 is integer and x2 is continuous or discrete
    BinaryMonomial, // Fram linearizing a monomial term b1 * b2 * ... * bn, where bi are binary
    AbsoluteValue, // From rewriting an absolute value
    AntiEpigraph, // For rewriting an epigraph formulation as objective
    EigenvalueDecomposition // From performing an eigenvalue decomposition on quadratic sums
};

enum class E_Convexity
{
    Linear,
    Convex,
    Concave,
    Nonconvex,
    Unknown,
    NotSet
};

enum class E_DualSolutionSource
{
    LPSolution,
    MIPSolutionOptimal,
    ObjectiveConstraint,
    MIPSolverBound
};

enum class E_EventType
{
    NewPrimalSolution,
    UserTerminationCheck,
    ExternalHyperplaneSelection,
};

enum class E_HyperplaneSource
{
    None,
    MIPOptimalRootsearch,
    MIPSolutionPoolRootsearch,
    LPRelaxedRootsearch,
    MIPOptimalSolutionPoint,
    MIPSolutionPoolSolutionPoint,
    LPRelaxedSolutionPoint,
    LPFixedIntegers,
    PrimalSolutionSearch,
    PrimalSolutionSearchInteriorObjective,
    InteriorPointSearch,
    MIPCallbackRelaxed,
    ObjectiveRootsearch,
    ObjectiveCuttingPlane,
    External
};

enum class E_IntegerCutSource
{
    None,
    NLPFixedInteger
};

enum class E_IterationLineType
{
    DualSolution,
    DualCallback,
    DualIntegerFixed,
    DualRepair,
    DualReductionCut,
    PrimalNLP
};

enum class E_DualProblemClass
{
    LP,
    QP,
    QCQP,
    MIP,
    MIQP,
    MIQCQP
};

enum class E_LogLevel
{
    Off = 6,
    Critical = 5,
    Error = 4,
    Warning = 3,
    Info = 2,
    Debug = 1,
    Trace = 0
};

enum class E_ModelReturnStatus
{
    None,
    OptimalGlobal,
    // OptimalLocal,
    Unbounded,
    UnboundedNoSolution,
    InfeasibleGlobal,
    InfeasibleLocal,
    FeasibleSolution,
    NoSolutionReturned,
    ErrorUnknown,
    ErrorNoSolution
};

enum class E_Monotonicity
{
    NotSet,
    Unknown,
    Nondecreasing,
    Nonincreasing,
    Constant
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
    Rootsearch,
    RootsearchFixedIntegers,
    NLPFixedIntegers,
    NLPRelaxed,
    MIPSolutionPool,
    LPFixedIntegers,
    MIPCallback,
    InteriorPointSearch
};

enum class E_ProblemConvexity
{
    NotSet,
    Convex,
    Nonconvex
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

enum E_SettingType
{
    String,
    Integer,
    Double,
    Enum,
    Boolean
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

enum class E_SOSType
{
    One,
    Two
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
    NumericIssues,
    UserAbort,
    NoDualCutsAdded,
    None
};

enum class E_VariableType
{
    None,
    Real,
    Binary,
    Integer,
    Semicontinuous,
    Semiinteger
};

enum class ES_AddPrimalPointAsInteriorPoint
{
    KeepOriginal,
    KeepBoth,
    KeepNew,
    OnlyAverage
};

enum class ES_EigenValueDecompositionFormulation
{
    CoefficientReformulated,
    CoefficientRemains
};

enum class ES_HyperplaneCutStrategy
{
    ESH,
    ECP,
    OnlyExternal
};

enum class ES_IpoptSolver
{
    IpoptDefault,
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

enum class ES_ObjectiveRootsearch
{
    Always,
    IfConvex,
    Never
};

enum class ES_RootsearchConstraintStrategy
{
    AllAsMaxFunct,
    IndividualConstraints
};

enum class ES_RootsearchMethod
{
    BoostTOMS748,
    BoostBisection
};

enum class ES_MIPSolver
{
    Cplex,
    Gurobi,
    Cbc,
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

enum class ES_PrimalNLPFixedPoint
{
    AllSolutions,
    FirstSolution,
    AllFeasibleSolutions,
    FirstAndFeasibleSolutions,
    SmallestDeviationSolution
};

enum class ES_PrimalNLPProblemSource
{
    OriginalProblem,
    ReformulatedProblem,
    Both
};

enum class ES_PrimalNLPSolver
{
    Ipopt,
    GAMS,
    SHOT,
    None
};

enum class ES_PrimalNLPStrategy
{
    AlwaysUse,
    IterationOrTime,
    IterationOrTimeAndAllFeasibleSolutions
};

enum class ES_ReductionCutStrategy
{
    Fraction,
    GoldenRatio
};

enum class ES_ReformulationBinaryMonomials
{
    None,
    Simple,
    CostaLiberti
};

enum class ES_ReformulateBilinearInteger
{
    No,
    NoIfQuadraticSupport,
    Yes
};

enum class ES_PartitionNonlinearSums
{
    Always,
    IfConvex,
    Never
};

enum class ES_QuadraticProblemStrategy
{
    Nonlinear,
    QuadraticObjective,
    ConvexQuadraticallyConstrained,
    NonconvexQuadraticallyConstrained
};

enum class ES_QuadraticTermsExtractStrategy
{
    DoNotExtract,
    ExtractTermsToSame, // Extract terms at first level of nonlinear expression (at model read) to the same objective or
                        // constraint
    ExtractToEqualityConstraintIfNonconvex, // Extract terms at all levels (at reformulation step) to a new equality
                                            // expression
    ExtractToEqualityConstraintAlways // Extract terms at all levels (at reformulation step) to a new equality
                                      // expression
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
