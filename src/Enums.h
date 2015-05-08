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
	Feasible, Optimal, Infeasible, Unbounded, IterationLimit, TimeLimit, SolutionLimit, Error
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
	Cplex, Gurobi, Cbc
};

enum class ES_QPStrategy
{
	Nonlinear, QuadraticObjective, QuadraticallyConstrained
};

enum class ES_NLPSolver
{
	CuttingPlaneMiniMax, IPOptMiniMax, IPOptRelaxed, IPOptMiniMaxAndRelaxed, CouenneMiniMax
};

enum class ES_IPOptSolver
{
	ma27, ma57, ma86, ma97, mumps, multiple
};

enum class ES_LinesearchMethod
{
	Boost, Bisection
};

enum class ES_PrimalBoundNLPFixedPoint
{
	MILPSolution, SmallestDeviation, Both
};

enum class E_PrimalSolutionSource
{
	Linesearch, LinesearchFixedIntegers, NLPFixedIntegers, NLPRelaxed, MILPSolutionPool, ObjectiveConstraint
};

enum class E_DualSolutionSource
{
	LPSolution, MILPSolution, Linesearch, LinesearchFixedIntegers, NLPFixedIntegers, NLPRelaxed, ObjectiveConstraint
};
