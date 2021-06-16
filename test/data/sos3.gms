$title SOS1 and SOS2 behavior - MINLP (SOSMINLP01,SEQ=796)

$ontext
SBB and BDMLP used to have (prior to 27) a different definition of SOS1 and SOS2
variables. They counted the number of variable in a set off-bound. All other solvers
counted the non-zero variables in a set. This resulted in exceptional circumstances
to confusing results when switching the solver. In GAMS 27 the SBB and BDMLP behavior
has been adjusted the SOS variable interpretation and now counts variables that are
non-zero.

This test checks for some exceptional situations when using SOS variables.

Contributor: Michael Bussieck, January 2019
$offtext

$set MTYPE minlp

Set i /1*4/;
SOS1 Variable x1(i);
SOS2 Variable x2(i);
Variable z;
Equation e1,e2;
e1.. z =e= sum(i, ord(i)*x1(i));
e2.. z =e= sum(i, ord(i)*x2(i));
Model m1 /e1/, m2 /e2/;

option optCR=0;

x1.up(i) = 2;
solve m1 max z us %MTYPE%;

