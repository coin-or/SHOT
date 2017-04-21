#pragma once
/*
 enum E_TimerTypes
 {
 Reformulation,
 InteriorPointTotal,
 InteriorPointMinimax,
 InteriorPointRelaxed,
 Subproblems,
 LP,
 MILP,
 HyperplaneLinesearch,
 PrimalBoundLinesearch,
 PrimalBoundNLP,
 PrimalBoundTotal,
 Total,
 };*/

enum class E_TerminationReason
{
	ConstraintTolerance,
	ObjectiveStagnation,
	IterationLimit,
	TimeLimit,
	InfeasibleProblem,
	Error,
	AbsoluteGap,
	RelativeGap,
	InteriorPointError
};

enum class E_ProblemSolutionStatus
{
	Feasible, Optimal, Infeasible, Unbounded, IterationLimit, TimeLimit, SolutionLimit, Error, CutOff
};

enum class E_IterationProblemType
{
	MIP, Relaxed
};

enum class E_ObjectiveFunctionType
{
	Linear, Quadratic, GeneralNonlinear
};

enum class E_ProblemConstraintType
{
	Linear, Quadratic, GeneralNonlinear
};

enum class ES_SolutionStrategy
{
	ESH, ECP
};

enum class ES_RelaxationStrategy
{
	Standard, Adaptive, None
};

enum class ES_MILPSolver
{
	Cplex, Gurobi, Cbc, CplexExperimental
};

enum class ES_QPStrategy
{
	Nonlinear, QuadraticObjective, QuadraticallyConstrained
};

enum class ES_NLPSolver
{
	CuttingPlaneMiniMax, IPOptMiniMax, IPOptRelaxed, IPOptMiniMaxAndRelaxed
};

enum class ES_IPOptSolver
{
	ma27, ma57, ma86, ma97, mumps, multiple
};

enum class ES_LinesearchMethod
{
	Boost, Bisection
};

enum class ES_PrimalNLPStrategy
{
	AlwaysUse, DoNotUse, IterationOrTime, IterationOrTimeAndAllFeasibleSolutions
};

enum class ES_PrimalBoundNLPFixedPoint
{
	AllSolutions, FirstSolution, AllFeasibleSolutions, FirstAndFeasibleSolutions, SmallestDeviationSolution
};

enum class E_PrimalSolutionSource
{
	Linesearch,
	LinesearchFixedIntegers,
	NLPFixedIntegers,
	NLPRelaxed,
	MILPSolutionPool,
	ObjectiveConstraint,
	LPFixedIntegers,
	LazyConstraintCallback,
	HeuristicCallback,
	IncumbentCallback
};

enum class E_PrimalNLPSource
{
	FirstSolution, FeasibleSolution, UnFeasibleSolution, SmallestDeviationSolution, FirstSolutionNewDualBound
};

enum class E_DualSolutionSource
{
	LPSolution, MILPSolutionOptimal, MILPSolutionFeasible,
	//Linesearch,
	//LinesearchFixedIntegers,
	//NLPFixedIntegers,
	//NLPRelaxed,
	ObjectiveConstraint,
//LPFixedIntegers
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
	PrimalSolutionSearchInteriorObjective
};

enum class ES_AddPrimalPointAsInteriorPoint
{
	KeepOriginal, KeepBoth, KeepNew, OnlyAverage
};

enum class ES_PresolveStrategy
{
	Never, Once, EveryIteration
};

enum class ES_LinesearchConstraintStrategy
{
	AllAsMaxFunct, IndividualConstrains
};
