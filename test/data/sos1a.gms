$TITLE Test of SOS1 variables with active upper bounds (SOS1A,SEQ=106)

$ontext
*  simple capacitated transportation model
*     source_j   -->    dest_j
*             0<=x_j<=xmax
*
*  upper bounds put a squeeze on x_j
*  when x(J) is SOS1, the MIP and RMIP solutions
*  should be very different.
*
*  N.B. We only test cases where the lower bounds on SOS1 vars are 0.
*  There is no consensus definition when lower bounds are nonzero.
$offtext

$if not set TESTTOL $set TESTTOL 1e-6
$eolcom //

set J / 1 * 3 /;

parameters
    c(J) /
        1       .9,     // less valuable commodity
        2       1,
        3       1.1     // more valuable commidity
    /,
    xmax(J) 'link capacities' /
        1       .8,
        2       .6,
        3       .6
    /;

sos1 variable
    x(J);

variable
    obj;

x.lo(J) = 0;
x.up(J) = xmax(J);

equations
    xsum        'one unit shipped across source -> destination',
    objdef;

objdef .. obj  =e=  sum {J, c(J) * x(J)};
xsum ..   sum {J, x(J)}  =l=  1;

model captrans / objdef, xsum /;
captrans.optcr = 0;

scalar tol / %TESTTOL% /,
       mipobj / .72 /,
       mipsum / .8 /,
       rmipobj / 1.06 /,
       rmipsum / 1 /;
parameter xmip(J)  / 1 .8 /,
          xrmip(J) / 2 .4
                     3 .6 /;

solve captrans using mip maximizing obj;
