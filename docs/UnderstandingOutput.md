# Understanding SHOT output

SHOT reports how it prepares the model, improves objective bounds, searches for
feasible solutions, and finally terminates. This guide explains the normal
terminal output, also saved in `SHOT.log` by the command-line program. The same
reporting is used by SHOTpy and C++; modeling systems may surround it with their
own output.

For running the executable and choosing result files, see
[command-line usage](CommandLineUsage.md). For the meaning of model statuses and
safe result retrieval, see [solver results](concepts.md). For intermediate models
and diagnostic files, see [debugging SHOT](DebuggingSHOT.md).

## The parts of a solve

Sections can be absent when the selected solution strategy does not need them.
Read the selected strategy in the **Options** section before comparing logs.

| Section | What it tells you |
| --- | --- |
| Header | SHOT version, Git revision, and build date. Record these when reporting a problem. |
| Modeling system | Input interface and, for file input, the problem filename. |
| Bound tightening | Variable bounds tightened on the original and reformulated problems, passes, elapsed time, and initial objective bounds. These objective bounds do not mean a feasible solution has been found. |
| Problem instance | Classification, objective direction and type, constraint and variable counts, and transformations. Compare **Original** and **Reformulated** to see the effect of auxiliary variables and rewritten expressions. |
| Options | Non-default settings and their sources, dual strategy, cut algorithm, MIP solver and threads, and primal NLP solver. The selected primal solver can still be reported as unused if no NLP call is needed. |
| Interior point search | An auxiliary search for a point strictly inside the nonlinear constraints, used to generate ESH hyperplanes. Its objective and iteration counter belong to that auxiliary problem. |
| Main iteration step | Progress of dual subproblems, primal NLP searches, and, where applicable, callbacks, repairs, and reduction cuts. |
| Solution report | Why SHOT stopped, what was established about the original model, globally valid bounds and gaps, criteria, counts, and timings. |
| Result paths | Where the executable wrote the solution and log. |

SHOT introduces auxiliary variables and linking constraints when partitioning
nonlinear expressions and reformulating individual terms, in both the objective
and constraints. These transformations explain increases in model size and can
also make a nonlinear objective linear in the auxiliary variables. The
transformation breakdown shows which reformulations were used. See
[modeling and reformulation](Modeling.md) for examples.

The source tags in **Non-default options** explain why an effective value differs
from its default. For example, `[solver (compatibility)]` beside
`Dual.TreeStrategy = 0` means that backend capabilities required multi-tree.
See [configuration and precedence](Configuration.md#which-setting-value-wins).

## Interior-point table

The cutting-plane minimax search can print a table like this:

```text
    Iteration     │  Time  │    Cuts     │     Objective value     │  Objective diff.
     #: type      │  tot.  │   + | tot.  │    problem | line srch  │    abs. | rel.
     1: LP           0.05                      -1e+12 | -1e+12          inf. | inf.
     2: LP           0.05      2 | 2         -5.49861 | 0.467842     6.0e+00 | 1.1e+00
     3: LP           0.05      2 | 4         -1.36005 | -0.0102125   1.3e+00 | 9.9e-01

 Valid interior point with constraint deviation -0.010 found.
```

The table has two header rows: the first names column groups and the second
names the columns within each group. Under **Objective value**, `problem` is
the LP approximation's objective value and `line srch` is the auxiliary
objective value at the point obtained by a line search.

The auxiliary objective measures the maximum nonlinear constraint deviation.
A negative deviation supplies an interior point for the nonlinear constraints.
This point need not satisfy integrality and does not by itself establish a
primal bound for the MINLP.

Under **Objective diff.**, `abs.` is the absolute difference between the two
auxiliary objective values and `rel.` divides it by `1e-10 + abs(problem)`.
These differences are not the original model's optimality gap. The first row
has no line search and no calculated objective difference.
Very loose early LP values, such as `-1e+12`, concern this auxiliary relaxation.
The main iteration counter starts separately afterwards.

## Main iteration columns

For minimization, the main header is:

```text
    Iteration     │  Time  │  Dual cuts  │     Objective value     │   Objective gap   │     Current solution
     #: type      │  tot.  │   + | tot.  │       dual | primal     │    abs. | rel.    │    obj.fn. | max.err.
```

| Column | Interpretation |
| --- | --- |
| `#: type` | Counter for this operation and its row label. Dual solves, NLP calls, repairs, and reduction-cut updates have separate counters. |
| `Time: tot.` | Cumulative elapsed seconds, including preparation, rather than the duration of this row. |
| `Dual cuts: + / tot.` | Number of hyperplanes added and accumulated total for the associated dual iteration. Blank when the added count is zero. NLP rows can repeat the dual iteration's counts; do not add the counts across printed rows. |
| `Objective value: dual` | Bound from the current dual problem. It is distinct from the objective of that problem's returned solution. See the global-bound caveat below. |
| `Objective value: primal` | Best accepted feasible objective for the original model (the incumbent). For minimization it is an upper bound; for maximization it is a lower bound. |
| `Objective gap: abs. / rel.` | Distance between the relevant dual bound and incumbent. `rel.` is a fraction: `1e-3` is 0.1%, not 0.001%. |
| `Current solution: obj.fn.` | Objective of the point associated with this row. A relaxation point can violate the original nonlinear model and need not improve the incumbent. |
| `Current solution: max.err.` | Constraint index followed by its signed deviation at that point, for example `10: 5.92e-01`. Positive means violation; zero or negative means the checked constraint is satisfied. This column alone does not check all primal acceptance requirements. |

For maximization, the bound columns are ordered **primal | dual**, so the lower
bound remains on the left and the upper bound on the right.

Constraint indices are zero-based. Dual and callback rows refer to the
reformulated problem; NLP rows refer to the original or reformulated problem
identified by their `O` or `R` suffix. Compare with the matching problem dump
when using debug mode. For a constraint `L <= f(x) <= U`, the signed deviation
is `max(L - f(x), f(x) - U)`; despite its internal name, it is not divided by
the constraint magnitude. If no nonlinear constraints remain in the reformulated
problem, the report prints `0` in the error column instead of an indexed value.

An infinite primal bound and gap normally mean no feasible solution has been
accepted yet. A blank current objective with `inf.` in the error column means
that the row has no reported objective/point, for example after an unsuccessful
NLP call. It does not prove that the original model is infeasible.

### A worked main-iteration example

These selected rows come from `test/data/synthes1.osil`, run from the repository
with HiGHS and Ipopt:

```bash
./build/SHOT test/data/synthes1.osil --mip=highs --nlp=ipopt --threads=1 \
  Output.Console.Iteration.Detail=0
```

Times, iteration paths, and last digits can change with the build and subsolvers.

```text
     1: LP-O         0.05                    -37.8684 | inf.            inf. | inf.        -37.8684 | 11: 2.11e+01
     6: MILP-SL1     0.17      2 | 12        0.716245 | 14.9066      1.4e+01 | 9.5e-01      7.92975 | 10: 5.92e-01
     1: NLPSOLPT-R   0.19      2 | 12        0.716245 | 10           9.3e+00 | 9.3e-01           10 | 1: 9.61e-09
     2: NLPFEASP-R   0.19      2 | 12        0.716245 | 6.00976      5.3e+00 | 8.8e-01      6.00976 | 1: 9.94e-09
    10: MILP-O       0.47                     6.00976 | 6.00976      0.0e+00 | 0.0e+00      6.00976 | 3: 9.87e-11
```

1. `LP-O` solves a continuous relaxation optimally. Its nonlinear constraint
   deviation is large and no feasible MINLP solution has been accepted.
2. `MILP-SL1` reaches the MIP solution limit of one. The dual bound is
   `0.716245`; the current MIP point has objective `7.92975` and violates a
   nonlinear constraint. A different, accepted point supplies the incumbent
   `14.9066`. A solution limit is an intentional way of limiting early MIP work.
3. `NLPSOLPT-R` fixes integers from the first MIP solution and solves an NLP on
   the reformulated model. It improves the incumbent to `10`. The row number
   restarts at one because it counts NLP solves.
4. `NLPFEASP-R` uses another solution-pool candidate and improves the incumbent
   again, to about `6.00976`.
5. `MILP-O` closes the gap. The final solution report confirms global optimality
   within tolerances. Rounded iteration values are not a substitute for that
   report: here the final reported absolute gap was about `2.68e-7`.

### Dual subproblem labels

The prefix describes the subproblem: `LP`, `QP`, `QCQP`, `MILP`, `MIQP`, or
`MIQCQP` (mixed-integer quadratically constrained quadratic programming).
Continuous relaxation rows can therefore appear during the solution of an
integer model.

| Suffix | Subproblem outcome |
| --- | --- |
| `-O` | Optimal solution of this subproblem. This does not alone prove optimality of the original MINLP. |
| `-F` | Feasible subproblem solution returned without an optimality proof. |
| `-SL<n>` | Solution limit reached; the number is the limit used. Limits above 100 are printed without the number. |
| `-TL` | Time limit reached. |
| `-IL` | Iteration limit reached. |
| `-NL` | Node limit reached. |
| `-I` | Subproblem reported infeasible; interpret the final original-model status separately. |
| `-U` | Subproblem reported unbounded; an early relaxation can be unbounded even if the original model is bounded. |
| `-N` | Numerical issue. |
| `-E` | Solver error. |
| `-A` | Solve aborted. |

### Fixed-integer NLP labels

These rows report primal searches with discrete variables fixed. Their labels
describe the source of the starting candidate, rather than the NLP termination
status. The final `-O` means **original problem**, and `-R` means **reformulated
problem**; the `O` here has a different meaning from a dual row's optimal suffix.

| Label before `-O` or `-R` | Candidate source |
| --- | --- |
| `NLPSOLPT` | First MIP solution point. |
| `NLPFEASP` | Another MIP solution-pool point. |
| `NLPUNFEA` | An infeasible MIP solution candidate. |
| `NLPSMDEV` | MIP candidate with the smallest nonlinear constraint deviation. |
| `NLPNEWDB` | First MIP solution associated with a dual-bound update. |
| `NLPENUM` | Enumeration of discrete variable assignments. |
| `NLPSTART` | Given starting point. |

An NLP call need not find or improve a primal solution. Check whether the primal
bound changes, and enable `Output.Console.PrimalSolver.Show=true` or debug logging
when you need details of an NLP failure.

### Single-tree callback rows

`CB` and `CB (th: ...)` report progress from MIP solver callbacks in the
single-tree strategy, supported by CPLEX and Gurobi. The optional `th` identifies
the callback thread. For example, a label `12: CB` refers to a callback-related
iteration inside one branch-and-cut search, rather than the twelfth independent
MIP solve. Several events can share an iteration number. Cuts are added during
that tree and returned MIP points still need the nonlinear feasibility checks.

For example, a single-tree run of the same model produced these selected rows:

```text
    17: CB           0.04      3 | 48         5.84339 | 7.09273      1.2e+00 | 1.8e-01      5.86587 | 3: 7.54e-02
    20: CB           0.04      3 | 54         5.84339 | 6.00974      1.7e-01 | 2.8e-02      6.00974 | 11: 1.39e-05
    21: MILP-O       0.05                     6.00974 | 6.00974      0.0e+00 | 0.0e+00      6.00974 | 0: 0.00e+00
```

The callback rows show progress inside the tree, including an improved
incumbent. The final `MILP-O` row reports the completed MIP solve. Default
filtering can omit callback rows between the ones shown.

### Repair and reduction-cut rows

These rows replace the usual bound and gap fields with a description of the
action. Their counters are separate from dual iterations.

| Label | Action |
| --- | --- |
| `REP-SUCC` | Infeasibility repair succeeded in relaxing the dual problem. It does not mean a feasible original-model solution was found. |
| `REP-F-<n>` | Infeasibility repair failed; the suffix counts main repair attempts. |
| `REP-RESET` | Nonconvex cuts were removed to restart cut generation after repeated points. |
| `REP-LOOP` | Repeated-point repair handling detected a loop. |
| `REDCUT-<n>` | Objective cutoff was tightened in an attempt to find an improved primal solution. The suffix counts successive updates without improvement. |

`Relaxed constraints` is the repair count; `Obj.cut` is the objective cutoff.
The following `Total primal improvements after repair / reduction cut` line
counts improvements associated with those strategies.

## Bounds, gaps, and the asterisk

With primal bound `p` and dual bound `d`, SHOT calculates:

```text
absolute gap = abs(d - p)
relative gap = abs(d - p) / (1e-10 + abs(p))
```

If either bound is unavailable, the gap is infinite. A primal objective near
zero makes the relative gap sensitive, so also consider the absolute tolerance.
Meeting either objective-gap tolerance is sufficient for gap termination.

For nonconvex problems, cuts or repairs can make the current dual problem cease
to be a globally valid relaxation. SHOT marks the displayed current dual bound
with `*`. Its proximity to the incumbent then does not prove global optimality.
For example, a schematic display `5* | 10` can coexist with a final globally
valid interval `[0, 10]`: the working bound `5` is not the certificate.

Ordinary dual-solve rows use the **current** bound for their gaps. NLP and
callback rows use the **global** bound for their gaps, although their displayed
dual-value column still shows the current bound. The **Solution report** and
the public objective-gap getters use the global bound. Consequently, when `*`
is present, recalculating a row's gap from its displayed bound columns may not
reproduce the printed gap. Use `getGlobalDualBound()` when reporting a valid
bound programmatically.

If convexity is asserted by the user, the report describes that assumption;
any optimality guarantee depends on its correctness. See
[user-specified convexity](concepts.md#convexity-specified-by-the-user).

## Reading the solution report

Read both the termination reason and the model-status sentence. For example,
`Terminated since absolute gap met requirements` together with `Globally optimal
primal solution found` is different from reaching a time limit with a feasible
solution. A limit can also be reached without finding any primal solution.

The objective interval is `[dual, primal]` for minimization and `[primal, dual]`
for maximization. The report uses the globally valid dual bound. The **Fulfilled**
and **Unfulfilled termination criteria** list shows the tested values and
tolerances; it is not a requirement that all criteria be fulfilled. In
particular, time and iteration limits are alternative stopping conditions.

Counts distinguish dual solves, interior-point LPs, fixed-integer NLPs, and
accepted primal solutions. These need not equal the number of printed rows.
Timing categories include subcategories and can overlap, so do not sum every
printed timing to reconstruct the total.

The warning about a solution at maximum variable bounds means SHOT's artificial
bound limits may affect the answer. Check actual model bounds before treating
that point as an optimum of the intended model.

A successful process exit (`0`) means the executable completed the run; inspect
the model status and solution availability before using variable values. See
[results and termination](concepts.md#model-status-and-termination).

## Choosing output detail

`Output.Console.Iteration.Detail` controls which progress rows are generated:

| Value | Rows shown |
| --- | --- |
| `0` | All progress rows. |
| `1` (default) | Bound updates, plus the first iteration, selected exceptional events, and periodic progress after more than 100 suppressed calls or more than two seconds. |
| `2` | The default selection plus all primal NLP calls. |

Both console and file logging receive the rows that survive this selection.
Lowering the file log level cannot recover suppressed rows; use detail `0` for
a complete iteration history.

Logging severity is independent: `Output.Console.LogLevel` and
`Output.File.LogLevel` accept integers `0` (Trace), `1` (Debug), `2` (Info,
default), `3` (Warning), `4` (Error), `5` (Critical), and `6` (Off). Normal tables
are Info messages. Trace messages compiled out of Release builds cannot be
enabled by changing the setting.

`Output.Console.DualSolver.Show=true` enables MIP solver output and forces full
iteration detail. `Output.Console.PrimalSolver.Show=true` enables NLP solver
output. These backend logs have their own tables and status conventions; lines
routed through SHOT's output stream are prefixed with `|`.

See [configuration recipes](Configuration.md#output-and-diagnostics) for commands
that combine these controls.
