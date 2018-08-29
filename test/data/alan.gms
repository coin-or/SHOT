$offlisting
*  MINLP written by GAMS Convert at 04/18/01 12:05:57
*  
*  Equation counts
*     Total       E       G       L       N       X
*         8       3       0       5       0       0
*  
*  Variable counts
*                 x       b       i     s1s     s2s      sc      si
*     Total    cont  binary integer    sos1    sos2   scont    sint
*         9       5       4       0       0       0       0       0
*  FX     0       0       0       0       0       0       0       0
*  
*  Nonzero counts
*     Total   const      NL     DLL
*        24      21       3       0
*
*  Solve m using MINLP minimizing objvar;


Variables  x1,x2,x3,x4,objvar,b6,b7,b8,b9;

Positive Variables x1,x2,x3,x4;

Binary Variables b6,b7,b8,b9;

Equations  e1,e2,e3,e4,e5,e6,e7,e8;


e1..    x1 + x2 + x3 + x4 =E= 1;

e2..    8*x1 + 9*x2 + 12*x3 + 7*x4 =E= 10;

e3.. x1*(4*x1 + 3*x2 - x3) + x2*(3*x1 + 6*x2 + x3) + x3*(x2 - x1 + 10*x3)
      - objvar =E= 0;

e4..    x1 - b6 =L= 0;

e5..    x2 - b7 =L= 0;

e6..    x3 - b8 =L= 0;

e7..    x4 - b9 =L= 0;

e8..    b6 + b7 + b8 + b9 =L= 3;

* set non default bounds

$if set nostart $goto modeldef
* set non default levels

x1.l = 0.302884615384618; 
x2.l = 0.0865384615384593; 
x3.l = 0.504807692307693; 
x4.l = 0.10576923076923; 
objvar.l = 2.89903846153846; 

* set non default marginals

e1.m = 1;
e2.m = 1;
e3.m = 1;
x1.m = 1; 
x2.m = 1; 

$label modeldef
Model m / all /;

m.limrow=1; m.limcol=0;

$if NOT '%gams.u1%' == '' $include '%gams.u1%'

$if not set MINLP $set MINLP MINLP
Solve m using %MINLP% minimizing objvar;

