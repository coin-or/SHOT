$offlisting
*  
*  Equation counts
*      Total        E        G        L        N        X        C        B
*         31        9        0       22        0        0        0        0
*  
*  Variable counts
*                   x        b        i      s1s      s2s       sc       si
*      Total     cont   binary  integer     sos1     sos2    scont     sint
*         36        8       14        0        0        0       14        0
*  FX      2
*  
*  Nonzero counts
*      Total    const       NL      DLL
*         83       76        7        0
*
*  Solve m using MINLP minimizing objvar;


Variables  objvar,x2,x3,x4,x5,x6,x7,x8,sc9,sc10,sc11,sc12,sc13,sc14,sc15,sc16
          ,sc17,sc18,sc19,sc20,sc21,sc22,b23,b24,b25,b26,b27,b28,b29,b30,b31
          ,b32,b33,b34,b35,b36;

Positive Variables  x2,x3,x4,x5,x6,x7,x8;

Binary Variables  b23,b24,b25,b26,b27,b28,b29,b30,b31,b32,b33,b34,b35,b36;

Semicont Variables  sc9,sc10,sc11,sc12,sc13,sc14,sc15,sc16,sc17,sc18,sc19,sc20
          ,sc21,sc22;

Equations  e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19
          ,e20,e21,e22,e23,e24,e25,e26,e27,e28,e29,e30,e31;


e1..    x2 + x3 + x4 + x5 + x6 + x7 + x8 =E= 1;

e2..    x2 - sc9 + sc16 =E= 0.2;

e3..    x3 - sc10 + sc17 =E= 0.2;

e4..    x4 - sc11 + sc18 =E= 0;

e5..    x5 - sc12 + sc19 =E= 0;

e6..    x6 - sc13 + sc20 =E= 0.2;

e7..    x7 - sc14 + sc21 =E= 0.2;

e8..    x8 - sc15 + sc22 =E= 0.2;

e9..    sc9 + sc10 + sc11 + sc12 + sc13 + sc14 + sc15 =L= 0.3;

e10..    sc9 - 0.11*b23 =L= 0;

e11..    sc10 - 0.1*b24 =L= 0;

e12..    sc11 - 0.07*b25 =L= 0;

e13..    sc12 - 0.11*b26 =L= 0;

e14..    sc13 - 0.2*b27 =L= 0;

e15..    sc14 - 0.1*b28 =L= 0;

e16..    sc15 - 0.1*b29 =L= 0;

e17..    sc16 - 0.2*b30 =L= 0;

e18..    sc17 - 0.15*b31 =L= 0;

e19..    sc18 =L= 0;

e20..    sc19 =L= 0;

e21..    sc20 - 0.1*b34 =L= 0;

e22..    sc21 - 0.15*b35 =L= 0;

e23..    sc22 - 0.2*b36 =L= 0;

e24..    b23 + b30 =L= 1;

e25..    b24 + b31 =L= 1;

e26..    b25 + b32 =L= 1;

e27..    b26 + b33 =L= 1;

e28..    b27 + b34 =L= 1;

e29..    b28 + b35 =L= 1;

e30..    b29 + b36 =L= 1;

e31.. -(42.18*x2*x2 + 40.36*x2*x3 + 21.76*x2*x4 + 10.6*x2*x5 + 24.64*x2*x6 + 
      47.68*x2*x7 + 34.82*x2*x8 + 70.89*x3*x3 + 43.16*x3*x4 + 30.82*x3*x5 + 
      46.48*x3*x6 + 47.6*x3*x7 + 25.24*x3*x8 + 25.51*x4*x4 + 19.2*x4*x5 + 45.26
      *x4*x6 + 26.44*x4*x7 + 9.4*x4*x8 + 22.33*x5*x5 + 20.64*x5*x6 + 20.92*x5*
      x7 + 2*x5*x8 + 30.01*x6*x6 + 32.72*x6*x7 + 14.4*x6*x8 + 42.23*x7*x7 + 
      19.8*x7*x8 + 16.42*x8*x8 - 0.06435*x2 - 0.0548*x3 - 0.02505*x4 - 0.0762*
      x5 - 0.03815*x6 - 0.0927*x7 - 0.031*x8) + objvar =E= 0;

* set non-default bounds
sc9.lo = 0.03; sc9.up = 0.11;
sc10.lo = 0.04; sc10.up = 0.1;
sc11.lo = 0.04; sc11.up = 0.07;
sc12.lo = 0.03; sc12.up = 0.11;
sc13.lo = 0.03; sc13.up = 0.2;
sc14.lo = 0.03; sc14.up = 0.1;
sc15.lo = 0.03; sc15.up = 0.1;
sc16.lo = 0.02; sc16.up = 0.2;
sc17.lo = 0.02; sc17.up = 0.15;
sc18.fx = 0;
sc19.fx = 0;
sc20.lo = 0.04; sc20.up = 0.1;
sc21.lo = 0.04; sc21.up = 0.15;
sc22.lo = 0.04; sc22.up = 0.2;

Model m / all /;

m.limrow=0; m.limcol=0;
m.tolproj=0.0;

$if NOT '%gams.u1%' == '' $include '%gams.u1%'

$if not set MINLP $set MINLP MINLP
Solve m using %MINLP% minimizing objvar;
