$offlisting
*  
*  Equation counts
*      Total        E        G        L        N        X        C        B
*         25        7        0       18        0        0        0        0
*  
*  Variable counts
*                   x        b        i      s1s      s2s       sc       si
*      Total     cont   binary  integer     sos1     sos2    scont     sint
*         38        5       31        2        0        0        0        0
*  FX      0
*  
*  Nonzero counts
*      Total    const       NL      DLL
*        210      202        8        0
*
*  Solve m using MINLP minimizing objvar;


Variables  b1,b2,i3,i4,x5,x6,x7,x8,b9,b10,b11,b12,b13,b14,b15,b16,b17,b18,b19
          ,b20,b21,b22,b23,b24,b25,b26,b27,b28,b29,b30,b31,b32,b33,b34,b35,b36
          ,b37,objvar;

Binary Variables  b1,b2,b9,b10,b11,b12,b13,b14,b15,b16,b17,b18,b19,b20,b21,b22
          ,b23,b24,b25,b26,b27,b28,b29,b30,b31,b32,b33,b34,b35,b36,b37;

Integer Variables  i3,i4;

Equations  e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19
          ,e20,e21,e22,e23,e24,e25;


e1..  - 0.1*b1 - 0.2*b2 - b9 - 2*b10 - 3*b11 - 4*b12 - 5*b13 - 6*b14 - 7*b15
      - 8*b16 - b17 - 2*b18 - 3*b19 - 4*b20 - 5*b21 - 6*b22 - 7*b23 + objvar
      =E= 0;

e2..    460*b24 + 920*b25 + 1380*b26 + 1840*b27 + 570*b32 + 1140*b33 + 1710*b34
      =L= 1900;

e3..    460*b28 + 920*b29 + 1380*b30 + 1840*b31 + 570*b35 + 1140*b36 + 1710*b37
      =L= 1900;

e4..  - 460*b24 - 920*b25 - 1380*b26 - 1840*b27 - 570*b32 - 1140*b33 - 1710*b34
      =L= -1700;

e5..  - 460*b28 - 920*b29 - 1380*b30 - 1840*b31 - 570*b35 - 1140*b36 - 1710*b37
      =L= -1700;

e6..    b24 + 2*b25 + 3*b26 + 4*b27 + b32 + 2*b33 + 3*b34 =L= 5;

e7..    b28 + 2*b29 + 3*b30 + 4*b31 + b35 + 2*b36 + 3*b37 =L= 5;

e8..    b1 - b9 - 2*b10 - 3*b11 - 4*b12 - 5*b13 - 6*b14 - 7*b15 - 8*b16 =L= 0;

e9..    b2 - b17 - 2*b18 - 3*b19 - 4*b20 - 5*b21 - 6*b22 - 7*b23 =L= 0;

e10..  - 8*b1 + b9 + 2*b10 + 3*b11 + 4*b12 + 5*b13 + 6*b14 + 7*b15 + 8*b16
       =L= 0;

e11..  - 7*b2 + b17 + 2*b18 + 3*b19 + 4*b20 + 5*b21 + 6*b22 + 7*b23 =L= 0;

e12..    i3 - 3*b9 - 8*b10 - 15*b11 - 24*b12 - 35*b13 - 48*b14 - 63*b15
       - 80*b16 =E= 1;

e13..    i4 - 3*b17 - 8*b18 - 15*b19 - 24*b20 - 35*b21 - 48*b22 - 63*b23 =E= 1;

e14..    b9 + b10 + b11 + b12 + b13 + b14 + b15 + b16 =L= 1;

e15..    b17 + b18 + b19 + b20 + b21 + b22 + b23 =L= 1;

e16..    x5 - 3*b24 - 8*b25 - 15*b26 - 24*b27 =E= 1;

e17..    x6 - 3*b28 - 8*b29 - 15*b30 - 24*b31 =E= 1;

e18..    x7 - 3*b32 - 8*b33 - 15*b34 =E= 1;

e19..    x8 - 3*b35 - 8*b36 - 15*b37 =E= 1;

e20..    b24 + b25 + b26 + b27 =L= 1;

e21..    b28 + b29 + b30 + b31 =L= 1;

e22..    b32 + b33 + b34 =L= 1;

e23..    b35 + b36 + b37 =L= 1;

e24.. -(sqrt(i3*x5) + sqrt(i4*x6)) + b9 + 2*b10 + 3*b11 + 4*b12 + 5*b13 + 6*b14
       + 7*b15 + 8*b16 + b17 + 2*b18 + 3*b19 + 4*b20 + 5*b21 + 6*b22 + 7*b23
       + b24 + 2*b25 + 3*b26 + 4*b27 + b28 + 2*b29 + 3*b30 + 4*b31 =L= -10;

e25.. -(sqrt(i3*x7) + sqrt(i4*x8)) + b9 + 2*b10 + 3*b11 + 4*b12 + 5*b13 + 6*b14
       + 7*b15 + 8*b16 + b17 + 2*b18 + 3*b19 + 4*b20 + 5*b21 + 6*b22 + 7*b23
       + b32 + 2*b33 + 3*b34 + b35 + 2*b36 + 3*b37 =L= -9;

* set non-default bounds
i3.lo = 1; i3.up = 100;
i4.lo = 1; i4.up = 100;
x5.lo = 1;
x6.lo = 1;
x7.lo = 1;
x8.lo = 1;

Model m / all /;

m.limrow=0; m.limcol=0;
m.tolproj=0.0;

$if NOT '%gams.u1%' == '' $include '%gams.u1%'

$if not set MINLP $set MINLP MINLP
Solve m using %MINLP% minimizing objvar;
