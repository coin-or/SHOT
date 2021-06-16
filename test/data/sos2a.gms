$TITLE 'Test of SOS2 variables'  (SOS2A,SEQ=345)

$ontext

Do linear interpolation of the points
   ord(I), f(I)
(using SOS2 vars w(I)) to define a function f(x).
By bounding w(I) below we can take out a little interval from the
domain of f.  We minimize the distance between f(x) and Fbar,
with full domain and with the restriction.

Contributor: Steve Dirkse
$offtext

$if not set TESTTOL $set TESTTOL 1e-6
scalar mchecks / 0 /;
$if not %MIPMCHECKS% == 0 mchecks = 1;
scalar Fbar / 1.3 /;

set I / 1 * 3 /;

parameter f(I) /
1       1
2       2
3       3
/;

scalar wLo 'lower bound on w(1)' / -1 /;

sos2 variables w(I);
positive variables fplus, fminus;
* the optimization forces
* fplus  = min(0,fx-Fbar)
* fminus = min(0,Fbar-fx)

variables
  obj,
  x,
  fx;

w.lo(I) = 0;

equations
  wsum,
  xdef,
  fxdef,
  objdef,
  defwLo,
  gapplus,
  gapminus;

wsum..  1  =e= sum {I, w(I)};
xdef..  x  =e= sum {I, w(I)*ord(I)};
fxdef..        fx =e= sum {I, w(I)*f(I)};
gapplus..  fplus  =g= fx - Fbar;
gapminus.. fminus =g= Fbar - fx;
defwLo..   w('1') =g= wLo;
objdef..   obj =e= fplus + fminus;

model m / all /;
m.optcr = 0;

scalars
  tol / %TESTTOL% /,
  obj1    'objective of first solve'  / 0 /
  obj1_m    / 0 /
  obj2    'objective of second solve' / .1 /
  obj2_m    / 0 /
  fplus1    / 0 /
  fminus1   / 0 /
  fplus1_m  / 1 /
  fminus1_m / 1 /
  fplus2    / 0 /
  fminus2   / .1 /
  fplus2_m  / 1 /
  fminus2_m / 0 /
  fx1_L     / 1.3 /
  fx2_L     / 1.2 /
  fx1_m     / 0 /
  fx2_m     / 0 /
  x1_L      / 1.3 /
  x2_L      / 1.2 /
  x1_m      / 0 /
  x2_m      / 0 /
  defw1_m   / 0 /
  defw2_m   / 1 /
  ;
parameters
  w1_L(I) /
    1       .7
    2       .3
    3       0
  /
  w1_m(I) /
    1       0
    2       0
    3       0
  /
  w2_L(I) /
    1       .8
    2       .2
    3       0
  /
  w2_m(I) /
    1       0
    2       0
    3       -1
  /
  ;
solve m using MIP minimizing obj;

