$offlisting
*  
*  Equation counts
*      Total        E        G        L        N        X        C        B
*          7        1        2        4        0        0        0        0
*  
*  Variable counts
*                   x        b        i      s1s      s2s       sc       si
*      Total     cont   binary  integer     sos1     sos2    scont     sint
*          7        4        3        0        0        0        0        0
*  FX      0
*  
*  Nonzero counts
*      Total    const       NL      DLL
*         23       17        6        0
*
*  Solve m using MINLP minimizing objvar;


Variables  x1,x2,x3,b4,b5,b6,objvar;

Positive Variables  x1,x2,x3;

Binary Variables  b4,b5,b6;

Equations  e1,e2,e3,e4,e5,e6,e7;


e1.. -(-18*log(1 + x2) - 19.2*log(1 + x1 - x2) + 10*x1) + 7*x3 - 5*b4 - 6*b5
      - 8*b6 + objvar =E= 10;

e2.. 0.8*log(1 + x2) + 0.96*log(1 + x1 - x2) - 0.8*x3 =G= 0;

e3.. log(1 + x2) + 1.2*log(1 + x1 - x2) - x3 - 2*b6 =G= -2;

e4..  - x1 + x2 =L= 0;

e5..    x2 - 2*b4 =L= 0;

e6..    x1 - x2 - 2*b5 =L= 0;

e7..    b4 + b5 =L= 1;

* set non-default bounds
x1.up = 2;
x2.up = 2;
x3.up = 1;

Model m / all /;

m.limrow=0; m.limcol=0;
m.tolproj=0.0;

$if NOT '%gams.u1%' == '' $include '%gams.u1%'

$if not set MINLP $set MINLP MINLP
Solve m using %MINLP% minimizing objvar;
