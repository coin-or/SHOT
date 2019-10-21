$offlisting
*  
*  Equation counts
*      Total        E        G        L        N        X        C        B
*         52       10        4       38        0        0        0        0
*  
*  Variable counts
*                   x        b        i      s1s      s2s       sc       si
*      Total     cont   binary  integer     sos1     sos2    scont     sint
*         47       43        4        0        0        0        0        0
*  FX      0
*  
*  Nonzero counts
*      Total    const       NL      DLL
*        139      137        2        0
*
*  Solve m using MINLP minimizing objvar;


Variables  x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19
          ,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36
          ,x37,x38,x39,x40,x41,x42,b43,b44,b45,b46,objvar;

Positive Variables  x1,x2,x3,x4,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20
          ,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36,x37
          ,x38,x39,x40,x41,x42;

Binary Variables  b43,b44,b45,b46;

Equations  e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19
          ,e20,e21,e22,e23,e24,e25,e26,e27,e28,e29,e30,e31,e32,e33,e34,e35,e36
          ,e37,e38,e39,e40,e41,e42,e43,e44,e45,e46,e47,e48,e49,e50,e51,e52;


e1..  - 2*x9 - 2*x10 + objvar =E= 0;

e2..  - x1 - x5 + x9 =G= 0;

e3..  - x2 - x6 + x9 =G= 0;

e4..  - x3 - x7 + x10 =G= 0;

e5..  - x4 - x8 + x10 =G= 0;

e6.. 40/x7 - x5 =L= 0;

e7.. 50/x8 - x6 =L= 0;

e8..    x1 - x11 - x12 - x13 - x14 =E= 0;

e9..    x2 - x15 - x16 - x17 - x18 =E= 0;

e10..    x3 - x19 - x20 - x21 - x22 =E= 0;

e11..    x4 - x23 - x24 - x25 - x26 =E= 0;

e12..    x5 - x27 - x28 - x29 - x30 =E= 0;

e13..    x6 - x31 - x32 - x33 - x34 =E= 0;

e14..    x7 - x35 - x36 - x37 - x38 =E= 0;

e15..    x8 - x39 - x40 - x41 - x42 =E= 0;

e16..    x11 - 29*b43 =L= 0;

e17..    x12 - 29*b44 =L= 0;

e18..    x13 - 29*b45 =L= 0;

e19..    x14 - 29*b46 =L= 0;

e20..    x15 - 29*b43 =L= 0;

e21..    x16 - 29*b44 =L= 0;

e22..    x17 - 29*b45 =L= 0;

e23..    x18 - 29*b46 =L= 0;

e24..    x19 - 29*b43 =L= 0;

e25..    x20 - 29*b44 =L= 0;

e26..    x21 - 29*b45 =L= 0;

e27..    x22 - 29*b46 =L= 0;

e28..    x23 - 29*b43 =L= 0;

e29..    x24 - 29*b44 =L= 0;

e30..    x25 - 29*b45 =L= 0;

e31..    x26 - 29*b46 =L= 0;

e32..    x27 - 40*b43 =L= 0;

e33..    x28 - 40*b44 =L= 0;

e34..    x29 - 40*b45 =L= 0;

e35..    x30 - 40*b46 =L= 0;

e36..    x31 - 40*b43 =L= 0;

e37..    x32 - 40*b44 =L= 0;

e38..    x33 - 40*b45 =L= 0;

e39..    x34 - 40*b46 =L= 0;

e40..    x35 - 40*b43 =L= 0;

e41..    x36 - 40*b44 =L= 0;

e42..    x37 - 40*b45 =L= 0;

e43..    x38 - 40*b46 =L= 0;

e44..    x39 - 40*b43 =L= 0;

e45..    x40 - 40*b44 =L= 0;

e46..    x41 - 40*b45 =L= 0;

e47..    x42 - 40*b46 =L= 0;

e48..    x11 - x15 + x27 =L= 0;

e49..  - x12 + x16 + x32 =L= 0;

e50..    x21 - x25 + x37 =L= 0;

e51..  - x22 + x26 + x42 =L= 0;

e52..    b43 + b44 + b45 + b46 =E= 1;

* set non-default bounds
x1.up = 29;
x2.up = 29;
x3.up = 29;
x4.up = 29;
x5.lo = 1; x5.up = 40;
x6.lo = 1; x6.up = 50;
x7.lo = 1; x7.up = 40;
x8.lo = 1; x8.up = 50;
x9.up = 30;
x10.up = 30;

Model m / all /;

m.limrow=0; m.limcol=0;
m.tolproj=0.0;

$if NOT '%gams.u1%' == '' $include '%gams.u1%'

$if not set MINLP $set MINLP MINLP
Solve m using %MINLP% minimizing objvar;
