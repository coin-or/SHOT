// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_MCCORMICK McCormick Relaxation Arithmetic for Factorable Functions
\author Beno&icirc;t Chachuat

A convex relaxation \f$f^{\rm cv}\f$ of a function \f$f\f$ on the convex domain \f$D\f$ is a function that is (i) convex on \f$D\f$ and (ii) underestimates \f$f\f$  on \f$D\f$. Likewise, a concave relaxation \f$f^{\rm cc}\f$ of a function \f$f\f$ on the convex domain \f$D\f$ is a function that is (i) concave on \f$D\f$ and (ii) overestimates \f$f\f$  on \f$D\f$. McCormick's technique [McCormick, 1976] provides a means for computing pairs of convex/concave relaxations of a multivariate function on interval domains provided that this function is factorable and that the intrinsic univariate functions in its factored form have known convex/concave envelopes or, at least, relaxations.

<CENTER><TABLE BORDER=0>
<TR>
<TD>\image html McCormick_relax.png</TD>
</TR>
</TABLE></CENTER>


The class mc::McCormick provides an implementation of the McCormick relaxation technique and its recent extensions; see [McCormick, 1976; Scott <i>et al.</i>, 2011; Tsoukalas & Mitsos, 2012; Wechsung & Barton, 2013]. mc::McCormick also has the capability to propagate subgradients for these relaxations, which are guaranteed to exist in the interior of the domain of definition of any convex/concave function. This propagation is similar in essence to the forward mode of automatic differentiation; see [Mitsos <i>et al.</i>, 2009]. We note that mc::McCormick is <b>not a verified implementation</b> in the sense that rounding errors are not accounted for in computing convex/concave bounds and subgradients.

The implementation of mc::McCormick relies on the operator/function overloading mechanism of C++. This makes the computation of convex/concave relaxations both simple and intuitive, similar to computing function values in real arithmetics or function bounds in interval arithmetic (see \ref page_INTERVAL). Moreover, mc::McCormick can be used as the template parameter of other classes of MC++, for instance mc::TModel and mc::TVar. Likewise, mc::McCormick can be used as the template parameter of the classes fadbad::F, fadbad::B and fadbad::T of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A> for computing McCormick relaxations and subgradients of the partial derivatives or the Taylor coefficients of a factorable function (see \ref sec_MCCORMICK_fadbad).

mc::McCormick itself is templated in the type used to propagate the supporting interval bounds. By default, mc::McCormick can be used with the non-verified interval type mc::Interval of MC++. For reliability, however, it is strongly recommended to use verified interval arithmetic such as <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> or <A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A>. We note that Taylor models as provided by the classes mc::TModel and mc::TVar can also be used as the template parameter (see \ref page_TAYLOR).

Examples of McCormick relaxations constructed with mc::McCormick are shown on the left plot of the figure below for the factorable function \f$f(x)=\cos(x^2)\,\sin(x^{-3})\f$ for \f$x\in [\frac{\pi}{6},\frac{\pi}{3}]\f$. Also shown on the right plot are the affine relaxations constructed from a subgradient at \f$\frac{\pi}{4}\f$ of the McCormick relaxations of \f$f\f$ on \f$[\frac{\pi}{6},\frac{\pi}{3}]\f$.

<CENTER><TABLE BORDER=0>
<TR>
<TD>\image html MC-1D_relax.png</TD>
<TD>\image html MC-1D_linearize.png</TD>
</TR>
</TABLE></CENTER>


\section sec_MCCORMICK_use How do I compute McCormick relaxations of a factorable function?

Suppose one wants to compute McCormick relaxation of the real-valued function \f$f(x,y)=x(\exp(x)-y)^2\f$ for \f$(x,y)\in [-2,1]\times[-1,2]\f$, at the point \f$(x,y)=(0,1)\f$. For simplicity, the supporting interval bounds are calculated using the default interval type mc::Interval here:

\code
      #include "interval.hpp"
      typedef mc::Interval I;
      typedef mc::McCormick<I> MC;
\endcode

First, the variables \f$x\f$ and \f$y\f$ are defined as follows:

\code
      MC X( I( -2., 1. ), 0. );
      MC Y( I( -1., 2. ), 1. );
\endcode

Essentially, the first line means that <tt>X</tt> is a variable of type mc::McCormick, belonging to the interval \f$[-2,1]\f$, and whose current value is \f$0\f$. The same holds for the McCormick variable <tt>Y</tt>, which belonging to the interval \f$[-1,2]\f$ and has a value of \f$1\f$.

Having defined the variables, McCormick relaxations of \f$f(x,y)=x(\exp(x)-y)^2\f$ on \f$[-2,1]\times[-1,2]\f$ at \f$(0,1)\f$ are simply computed as:

\code
      MC F = X*pow(exp(X)-Y,2);
\endcode

These relaxations can be displayed to the standard output as:

\code
      std::cout << "f relaxations at (0,1): " << F << std::endl;
\endcode

which produces the following output:

\verbatim
f relaxations at (0,1): [ -2.76512e+01 :  1.38256e+01 ] [ -1.38256e+01 :  8.52245e+00 ]
\endverbatim

Here, the first pair of bounds correspond to the supporting interval bounds, as obtained with mc::Interval, and which are valid over the entire domain \f$[-2,1]\times[-1,2]\f$. The second pair of bounds are the values of the convex and concave relaxations at the selected point \f$(0,1)\f$. In order to describe the convex and concave relaxations on the entire range \f$[-2,1]\times[-1,2]\f$, it would be necessary to repeat the computations at different points. The current point can be set/modified by using the method mc::McCormick::c, for instance at \f$(-1,0)\f$

\code
      X.c( -1. );
      Y.c( 0. );
      F = X*pow(exp(X)-Y,2);
      std::cout << "f relaxations at (-1,0): " << F << std::endl;
\endcode

producing the output:

\verbatim
f relaxations at (-1,0): [ -2.76512e+01 :  1.38256e+01 ] [ -1.75603e+01 :  8.78014e+00 ]
\endverbatim

The values of the McCormick convex and concave relaxations of \f$f(x,y)\f$ can be retrieved, respectively, as:

\code
      double Fcv = F.cv();
      double Fcc = F.cc();
\endcode

Likewise, the lower and upper bounds of the supporting interval bounds can be retrieved as:

\code
      double Flb = F.l();
      double Fub = F.u();
\endcode


\section sec_MCCORMICK_sub How do I compute a subgradient of the McCormick relaxations?

Computing a subgradient of a McCormick relaxation requires specification of the independent variables via the .sub method, prior to evaluating the function in mc::McCormick type. Continuing the previous example, the function has two independent variables \f$x\f$ and \f$y\f$. Defining \f$x\f$ and \f$y\f$ as the subgradient components \f$0\f$ and \f$1\f$ (indexing in C/C++ start at 0 by convention!), respectively, is done as follows:

\code
      X.sub( 2, 0 );
      Y.sub( 2, 1 );
\endcode

Similar to above, the McCormick convex and concave relaxations of \f$f(x,y)\f$ at \f$(-1,0)\f$ along with subgradients of these relaxations are computed as:

\code
      F = X*pow(exp(X)-Y,2);
      std::cout << "f relaxations and subgradients at (-1,0): " << F << std::endl;
\endcode

producing the output:

\verbatim
f relaxations and subgradients at (-1,0): [ -2.76512e+01 :  1.38256e+01 ] [ -1.75603e+01 :  8.78014e+00 ] [ (-3.19186e+00, 3.70723e+00) : ( 1.59593e+00,-1.85362e+00) ]
\endverbatim

The additional information displayed corresponds to, respectively, a subgradient of the McCormick convex underestimator at (-1,0) and a subgradient of the McCormick concave overestimator at (-1,0). In turn, these subgradients can be used to construct affine relaxations on the current range, or passed to a bundle solver to locate the actual minimum or maximum of the McCormick relaxations. 

The subgradients of the McCormick relaxations of \f$f(x,y)\f$ at the current point can be retrieved as follows:

\code
      const double* Fcvsub = F.cvsub();
      const double* Fccsub = F.ccsub();
\endcode

or, alternatively, componentwise as:

\code
      double Fcvsub_X = F.cvsub(0);
      double Fcvsub_Y = F.cvsub(1);
      double Fccsub_X = F.ccsub(0);
      double Fccsub_Y = F.ccsub(1);
\endcode

Directional subgradients can be propagated too. In the case that subgradients are to computed along the direction (1,-1) for both the convex and concave relaxations, we define:

\code
      const double sub_dir[2] = { 1., -1 };
      X.sub( 1, &sub_dir[0], &sub_dir[0] );
      Y.sub( 1, &sub_dir[1], &sub_dir[1] );
      F = X*pow(exp(X)-Y,2);
      std::cout << "f relaxations and subgradients along direction (1,-1) at (-1,0): " << F << std::endl;
\endcode

producing the output:

\verbatim
f relaxations and subgradients along direction (1,-1) at (-1,0): [ -2.76512e+01 :  1.38256e+01 ] [ -1.75603e+01 :  8.78014e+00 ] [ (-6.89910e+00) : ( 3.44955e+00) ]
\endverbatim


\section sec_MCCORMICK_fadbad How do I compute McCormick relaxations of the partial derivatives or the Taylor coefficients of a factorable function using FADBAD++?

Now suppose one wants to compute McCormick relaxation not only for a given factorable function, but also for its partial derivatives. Continuing the previous example, the partial derivatives of \f$f(x,y)=x(\exp(x)-y)^2\f$ with respect to its independent variables \f$x\f$ and \f$y\f$ can be obtained via automatic differentiation (AD), either forward or reverse AD. This can be done for instance using the classes fadbad::F, and fadbad::B of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A>.

Considering forward AD first, we include the following header files:

\code
      #include "mcfadbad.hpp" // available in MC++
      #include "fadiff.h"     // available in FADBAD++
      typedef fadbad::F<MC> FMC;
\endcode

The variables are initialized and the derivatives and subgradients are seeded as follows:

\code
      FMC FX = X;             // initialize FX with McCormick variable X
      FX.diff(0,2);           // differentiate with respect to x (index 0 of 2)
      FX.x().sub(2,0);        // seed subgradient of x (index 0 of 2)

      FMC FY = Y;             // initialize FY with McCormick variable Y
      FY.diff(1,2);           // differentiate with respect to y (index 1 of 2)
      FY.x().sub(2,1);        // seed subgradient of y (index 1 of 2)
\endcode

As previously, the McCormick convex and concave relaxations of \f$f\f$, \f$\frac{\partial f}{\partial x}\f$, and \f$\frac{\partial f}{\partial y}\f$ at \f$(-1,0)\f$ on the range \f$[-2,1]\times[-1,2]\f$, along with subgradients of these relaxations, are computed as:

\code
      FMC FF = FX*pow(exp(FX)-FY,2);
      std::cout << "f relaxations and subgradients at (-1,0): " << FF.x() << std::endl;
      std::cout << "df/dx relaxations and subgradients at (-1,0): " << FF.d(0) << std::endl;
      std::cout << "df/dy relaxations and subgradients at (-1,0): " << FF.d(1) << std::endl;
\endcode

producing the output:

\verbatim
f relaxations and subgradients at (-1,0): [ -2.76512e+01 :  1.38256e+01 ] [ -1.75603e+01 :  8.78014e+00 ] [ (-3.19186e+00, 3.70723e+00) : ( 1.59593e+00,-1.85362e+00) ]
df/dx relaxations and subgradients at (-1,0): [ -4.04294e+01 :  3.41004e+01 ] [ -2.33469e+01 :  2.90549e+01 ] [ (-2.31383e+01,-1.94418e-01) : ( 1.59593e+00,-1.85362e+00) ]
df/dy relaxations and subgradients at (-1,0): [ -7.45866e+00 :  1.48731e+01 ] [ -5.96505e+00 :  7.71460e+00 ] [ (-5.96505e+00,-4.00000e+00) : ( 7.17326e+00,-4.00000e+00) ]
\endverbatim

Relaxations of the partial derivatives can also be computed using the backward mode of AD, which requires the additional header file:

\code
      #include "badiff.h"     // available in FADBAD++
      typedef fadbad::B<MC> BMC;
\endcode

then initialize and seed new variables and compute the function as follows:

\code
      BMC BX = X;             // initialize FX with McCormick variable X
      BX.x().sub(2,0);        // seed subgradient as direction (1,0)
      BMC BY = Y;             // initialize FY with McCormick variable Y
      BY.x().sub(2,1);        // seed subgradient as direction (1,0)

      BMC BF = BX*pow(exp(BX)-BY,2);
      BF.diff(0,1);           // differentiate f (index 0 of 1)
      std::cout << "f relaxations and subgradients at (-1,0): " << BF.x() << std::endl;
      std::cout << "df/dx relaxations and subgradients at (-1,0): " << BX.d(0) << std::endl;
      std::cout << "df/dy relaxations and subgradients at (-1,0): " << BY.d(0) << std::endl;
\endcode

producing the output:

\verbatim
f relaxations and subgradients at (-1,0): [ -2.76512e+01 :  1.38256e+01 ] [ -1.75603e+01 :  8.78014e+00 ] [ (-3.19186e+00, 3.70723e+00) : ( 1.59593e+00,-1.85362e+00) ]
df/dx relaxations and subgradients at (-1,0): [ -4.04294e+01 :  3.41004e+01 ] [ -1.37142e+01 :  1.60092e+01 ] [ (-1.35056e+01,-1.94418e-01) : ( 8.82498e+00,-1.31228e+00) ]
df/dy relaxations and subgradients at (-1,0): [ -7.45866e+00 :  1.48731e+01 ] [ -5.96505e+00 :  7.71460e+00 ] [ (-5.96505e+00,-4.00000e+00) : ( 7.17326e+00,-4.00000e+00) ]
\endverbatim

It is noteworthy that the bounds, McCormick relaxations and subgradients for the partial derivatives as computed with the forward and backward mode, although valid, may not be the same since the computations involve different sequences or even operations. In the previous examples, for instance, forward and backward AD produce identical interval bounds for \f$\frac{\partial f}{\partial x}\f$ and \f$\frac{\partial f}{\partial y}\f$ at \f$(-1,0)\f$, yet significantly tighter McCormick relaxations are obtained with backward AD for \f$\frac{\partial f}{\partial x}\f$ at \f$(-1,0)\f$.

Another use of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A> involves computing McCormick relaxations of the Taylor coefficients in the Taylor expansion of a factorable function in a given direction up to a certain order. Suppose we want to compute McCormick relaxation of the first 5 Taylor coefficients of \f$f(x,y)=x(\exp(x)-y)^2\f$ in the direction \f$(1,0)\f$, i.e. the direction of \f$x\f$. This information can be computed by using the classes fadbad::T, which requires the following header file:

\code
      #include "tadiff.h"     // available in FADBAD++
      typedef fadbad::T<MC> TMC;
\endcode

The variables are initialized and the derivatives and subgradients are seeded as follows:

\code
      TMC TX = X;             // initialize FX with McCormick variable X
      TX[0].sub(2,0);        // seed subgradient as direction (1,0)
      TMC TY = Y;             // initialize FY with McCormick variable Y
      TY[0].sub(2,1);        // seed subgradient as direction (1,0)
      TX[1] = 1.;             // Taylor-expand with respect to x

      TMC TF = TX*pow(exp(TX)-TY,2);
      TF.eval(5);            // Taylor-expand f to degree 5
      for( unsigned int i=0; i<=5; i++ )
        std::cout << "d^" << i << "f/dx^" << i << " relaxations and subgradients at (-1,0): " << TF[i] << std::endl;
\endcode

producing the output:

\verbatim
d^0f/dx^0 relaxations and subgradients at (-1,0): [ -2.76512e+01 :  1.38256e+01 ] [ -1.75603e+01 :  8.78014e+00 ] [ (-3.19186e+00, 3.70723e+00) : ( 1.59593e+00,-1.85362e+00) ]
d^1f/dx^1 relaxations and subgradients at (-1,0): [ -4.04294e+01 :  3.41004e+01 ] [ -2.33469e+01 :  2.90549e+01 ] [ (-2.31383e+01,-1.94418e-01) : ( 1.59593e+00,-1.85362e+00) ]
d^2f/dx^2 relaxations and subgradients at (-1,0): [ -4.51302e+01 :  3.77111e+01 ] [ -1.97846e+01 :  2.25846e+01 ] [ (-1.97113e+01, 0.00000e+00) : ( 7.36023e+00,-4.06006e-01) ]
d^3f/dx^3 relaxations and subgradients at (-1,0): [ -2.65667e+01 :  2.82546e+01 ] [ -1.02662e+01 :  1.27412e+01 ] [ (-1.00820e+01,-4.51118e-02) : ( 7.66644e+00,-1.80447e-01) ]
d^4f/dx^4 relaxations and subgradients at (-1,0): [ -1.19764e+01 :  1.59107e+01 ] [ -4.29280e+00 :  6.13261e+00 ] [ (-4.25007e+00,-2.25559e-02) : ( 4.86086e+00,-5.63897e-02) ]
d^5f/dx^5 relaxations and subgradients at (-1,0): [ -4.44315e+00 :  7.16828e+00 ] [ -1.49744e+00 :  2.55611e+00 ] [ (-1.44773e+00,-6.76676e-03) : ( 2.29932e+00,-1.35335e-02) ]
\endverbatim

The zeroth Taylor coefficient corresponds to the function \f$f\f$ itself. It can also be checked that the relaxations of the first Taylor coefficient of \f$f\f$ matches those obtained with forward AD for \f$\frac{\partial f}{\partial x}\f$.

Naturally, the classes fadbad::F, fadbad::B and fadbad::T can be nested to produce relaxations of higher-order derivative information.


\section sec_MCCORMICK_fct Which functions are overloaded in McCormick relaxation arithmetic?

As well as overloading the usual functions <tt>exp</tt>, <tt>log</tt>, <tt>sqr</tt>, <tt>sqrt</tt>, <tt>pow</tt>, <tt>inv</tt>, <tt>cos</tt>, <tt>sin</tt>, <tt>tan</tt>, <tt>acos</tt>, <tt>asin</tt>, <tt>atan</tt>, <tt>erf</tt>, <tt>erfc</tt>, <tt>min</tt>, <tt>max</tt>, <tt>fabs</tt>, mc::McCormick also defines the following functions:
- <tt>fstep(x)</tt> and <tt>bstep(x)</tt>, implementing the forward step function (switching value from 0 to 1 for x>=0) and the backward step function (switching value from 1 to 0 for x>=0). These functions can be used to model a variety of discontinuous functions as first proposed in [Wechsung & Barton, 2012].
- <tt>ltcond(x,y,z)</tt> and <tt>gtcond(x,y,z)</tt>, similar in essence to <tt>fstep(x)</tt> and <tt>bstep(x)</tt>, and implementing disjunctions of the form { y if x<=0; z otherwise } and { y if x>=0; z otherwise }, respectively.
- <tt>inter(x,y,z)</tt>, computing the intersection \f$x = y\cap z\f$ and returning true/false if the intersection is nonempty/empty.
- <tt>hull(x,y)</tt>, computing convex/concave relaxations of the union \f$x\cup y\f$.
.


\section sec_MCCORMICK_opt What are the options in mc::McCormick and how are they set?

The class mc::McCormick has a public static member called mc::McCormick::options that can be used to set/modify the options; e.g.,

\code
      MC::options.ENVEL_USE = true;
      MC::options.ENVEL_TOL = 1e-12;
      MC::options.ENVEL_MAXIT = 100;
      MC::options.MVCOMP_USE = true;
\endcode

The available options are the following:

<TABLE border="1">
<CAPTION><EM>Options in mc::McCormick::Options: name, type and description</EM></CAPTION>
     <TR><TH><b>Name</b>  <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>ENVEL_USE</tt> <TD><tt>bool</tt> <TD>true
         <TD>Whether to compute convex/concave envelopes for the neither-convex-nor-concave univariate functions such as odd power terms, sin, cos, asin, acos, tan, atan, erf, erfc. This provides tighter McCormick relaxations, but it is more time consuming. Junction points are computed using the Newton or secant method first, then the more robust golden section search method if unsuccessful.
     <TR><TH><tt>ENVEL_TOL</tt> <TD><tt>double</tt> <TD>1e-10
         <TD>Termination tolerance for determination function points in convex/concave envelopes of univariate terms.
     <TR><TH><tt>ENVEL_MAXIT</tt> <TD><tt>int</tt> <TD>100
         <TD>Maximum number of iterations for determination function points in convex/concave envelopes of univariate terms.
     <TR><TH><tt>MVCOMP_USE</tt> <TD><tt>bool</tt> <TD>false
         <TD>Whether to use Tsoukalas & Mitsos's multivariate composition result for min/max, product, and division terms; see [Tsoukalas & Mitsos, 2012]. This provides tighter McCormick relaxations, but it is more time consuming.
     <TR><TH><tt>MVCOMP_TOL</tt> <TD><tt>double</tt> <TD>1e1*machprec()
         <TD>Tolerance for equality test in subgradient propagation for product terms with Tsoukalas & Mitsos's multivariate composition result; see [Tsoukalas & Mitsos, 2012].
     <TR><TH><tt>DISPLAY_DIGITS</tt> <TD><tt>unsigned int</tt> <TD>5
         <TD>Number of digits in output stream
</TABLE>


\section sec_MC_err What Errors Can Be Encountered during the Computation of Convex/Concave Bounds?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, a class object of type mc::McCormick::Exceptions is thrown, which contains the type of error. It is the user's responsibility to test whether an exception was thrown during a McCormick relaxation, and then make the appropriate changes. Should an exception be thrown and not caught by the calling program, the execution will stop.

Possible errors encountered during the computation of a McCormick relaxation are:

<TABLE border="1">
<CAPTION><EM>Errors during Computation of a McCormick relaxation</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH><tt>1</tt> <TD>Division by zero
     <TR><TH><tt>2</tt> <TD>Inverse with zero in range
     <TR><TH><tt>3</tt> <TD>Log with negative values in range
     <TR><TH><tt>4</tt> <TD>Square-root with nonpositive values in range
     <TR><TH><tt>5</tt> <TD>Inverse sine or cosine with values outside of \f$[-1,1]\f$ range
     <TR><TH><tt>6</tt> <TD>Tangent with values outside of \f$[-\frac{\pi}{2}+k\pi,\frac{\pi}{2}+k\pi]\f$ range
     <TR><TH><tt>-1</tt> <TD>Inconsistent size of subgradient between two mc::McCormick variables
     <TR><TH><tt>-2</tt> <TD>Failed to compute the convex or concave envelope of a univariate term
     <TR><TH><tt>-3</tt> <TD>Failed to propagate subgradients for a product term with Tsoukalas & Mitsos's multivariable composition result
</TABLE>

\section sec_MC_refs References
- Bompadre, A., A. Mitsos, <A href="http://dx.doi.org/10.1007/s10898-011-9685-2">Convergence rate of McCormick relaxations</A>, <I>Journal of Global Optimization</I> <B>52</B>(1):1-28, 2012
- McCormick, G. P., <A href="http://dx.doi.org/10.1007/BF01580665">Computability of global solutions to factorable nonconvex programs: Part I. Convex underestimating problems</A>, <i>Mathematical Programming</i>, <b>10</b>(2):147-175, 1976
- Mitsos, A., B. Chachuat, and P.I. Barton, <A href="http://dx.doi.org/10.1137/080717341">McCormick-based relaxations of algorithms</A>, <i>SIAM Journal on Optimization</i>, <b>20</b>(2):573-601, 2009
- Najmana, J. L., D. Bongartza, A. Tsoukalas, A. Mitsos, Correction of closed form for the multivariate McCormick relaxation of the binary product of functions, personal communication, 2016
- Scott, J.K., M.D. Stuber, P.I. Barton, <A href="http://dx.doi.org/10.1007/s10898-011-9664-7">Generalized McCormick relaxations</A>. <i>Journal of Global Optimization</i>, <b>51</b>(4), 569-606, 2011
- Tsoukalas, A., and A. Mitsos, <A href="http://www.optimization-online.org/DB_HTML/2012/05/3473.html">Multi-variate McCormick relaxations</A>, May 2012
- Wechsung, A., and P.I. Barton, <A href="http://dx.doi.org/10.1007/s10898-013-0060-3">Global optimization of bounded factorable functions with discontinuities</A>, <i>Journal of Global Optimization</i>, <b>in press</b>, 2013
.
*/

#ifndef MC__MCCORMICK_H
#define MC__MCCORMICK_H

#include <iostream>
#include <iomanip>
#include <stdarg.h>
#include <cassert>
#include <string>

#include "mcfunc.hpp"
#include "mcop.hpp"

namespace mc
{
//! @brief C++ class for McCormick relaxation arithmetic for factorable function
////////////////////////////////////////////////////////////////////////
//! mc::McCormick is a C++ class computing the McCormick
//! convex/concave relaxations of factorable functions on a box,
//! as well as doing subgradient propagation. The template parameter
//! corresponds to the type used in the underlying interval arithmetic
//! computations.
////////////////////////////////////////////////////////////////////////
template <typename T>
class McCormick
////////////////////////////////////////////////////////////////////////
{
  template <typename U> friend class McCormick;

  template <typename U> friend McCormick<U> operator+
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> operator+
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator+
    ( const double, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator+
    ( const McCormick<U>&, const double );
  template <typename U> friend McCormick<U> operator-
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> operator-
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator-
    ( const double, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator-
    ( const McCormick<U>&, const double );
  template <typename U> friend McCormick<U> operator*
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator*
    ( const double, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator*
    ( const McCormick<U>&, const double );
  template <typename U> friend McCormick<U> operator/
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator/
    ( const double, const McCormick<U>& );
  template <typename U> friend McCormick<U> operator/
    ( const McCormick<U>&, const double );
  template <typename U> friend std::ostream& operator<<
    ( std::ostream&, const McCormick<U>& );
  template <typename U> friend bool operator==
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend bool operator!=
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend bool operator<=
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend bool operator>=
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend bool operator<
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend bool operator>
    ( const McCormick<U>&, const McCormick<U>& );

  template <typename U> friend McCormick<U> inv
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> sqr
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> exp
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> log
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> cos
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> sin
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> tan
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> acos
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> asin
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> atan
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> fabs
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> sqrt
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> xlog
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> arh
    ( const McCormick<U>&, const double );
  template <typename U> friend McCormick<U> erf
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> erfc
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> fstep
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> bstep
    ( const McCormick<U>& );
  template <typename U> friend McCormick<U> pow
    ( const McCormick<U>&, const int );
  template <typename U> friend McCormick<U> pow
    ( const McCormick<U>&, const double );
  template <typename U> friend McCormick<U> pow
    ( const double, const McCormick<U>& );
  template <typename U> friend McCormick<U> pow
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> monomial
    ( const unsigned int, const McCormick<U>*, const int* );
  template <typename U> friend McCormick<U> cheb
    ( const McCormick<U>&, const unsigned );
  template <typename U> friend McCormick<U> min
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> max
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> min
    ( const unsigned int, const McCormick<U>* );
  template <typename U> friend McCormick<U> max
    ( const unsigned int, const McCormick<U>* );
  template <typename U> friend McCormick<U> min  
    ( const McCormick<U>&,const double );
  template <typename U> friend McCormick<U> max
    ( const McCormick<U>&, const double );
  template <typename U> friend McCormick<U> min
    ( const double, const McCormick<U>& );
  template <typename U> friend McCormick<U> max
    ( const double, const McCormick<U>& );
  template <typename U> friend McCormick<U> ltcond
    ( const McCormick<U>&, const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> ltcond
    ( const U&, const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> gtcond
    ( const McCormick<U>&, const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> gtcond
    ( const U&, const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend bool inter
    ( McCormick<U>&, const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> hull
    ( const McCormick<U>&, const McCormick<U>& );
  template <typename U> friend McCormick<U> cut
    ( const McCormick<U>& );

public:

  McCormick<T>& operator=
    ( const McCormick<T>& );
  McCormick<T>& operator=
    ( const T& );
  McCormick<T>& operator=
    ( const double );
  McCormick<T>& operator+=
    ( const McCormick<T>& );
  McCormick<T>& operator+=
    ( const double );
  McCormick<T>& operator-=
    ( const McCormick<T>& );
  McCormick<T>& operator-=
    ( const double );
  McCormick<T>& operator*=
    ( const McCormick<T>& );
  McCormick<T>& operator*=
    ( const double );
  McCormick<T>& operator/=
    ( const McCormick<T>& );
  McCormick<T>& operator/=
    ( const double );

  /** @defgroup MCCORMICK McCormick Relaxation Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Options of mc::McCormick
  static struct Options
  {
    //! @brief Constructor
    Options():
      ENVEL_USE(true), ENVEL_MAXIT(100), ENVEL_TOL(1e-10), MVCOMP_USE(false),
      MVCOMP_TOL(1e1*machprec()), DISPLAY_DIGITS(5)
      {}
    //! @brief Whether to compute convex/concave envelopes for the neither-convex-nor-concave univariate functions such as odd power terms, sin, cos, asin, acos, tan, atan, erf, erfc. This provides tighter McCormick relaxations, but it is more time consuming. Junction points are computed using the Newton or secant method first, then the more robust golden section search method if unsuccessful.
    bool ENVEL_USE;
    //! @brief Maximum number of iterations for determination function points in convex/concave envelopes of univariate terms.
    unsigned int ENVEL_MAXIT;
    //! @brief Termination tolerance for determination function points in convex/concave envelopes of univariate terms.
    double ENVEL_TOL;
    //! @brief Whether to use Tsoukalas & Mitsos's multivariate composition result for min/max, product, and division terms; see [Tsoukalas & Mitsos, 2012]. This provides tighter McCormick relaxations, but it is more time consuming.
    bool MVCOMP_USE;
    //! @brief Tolerance for testing equality in subgradient propagation for product terms with Tsoukalas & Mitsos's multivariate composition result; see [Tsoukalas & Mitsos, 2012].
    double MVCOMP_TOL;
    //! @brief Number of digits displayed with << operator (default=5)
    unsigned int DISPLAY_DIGITS;
  } options;

  //! @brief Exceptions of mc::McCormick
  class Exceptions
  {
  public:
    //! @brief Enumeration type for McCormick exception handling
    enum TYPE{
      DIV=1,	//!< Division by zero
      INV,	//!< Inverse with zero in range
      LOG,	//!< Log with negative values in range
      SQRT,	//!< Square-root with nonpositive values in range
      ASIN,	//!< Inverse sine or cosine with values outside of \f$[-1,1]\f$ range
      TAN,	//!< Tangent with values outside of \f$[-\frac{\pi}{2}+k\pi,\frac{\pi}{2}+k\pi]\f$ range
      CHEB,	//!< Chebyshev basis function different from [-1,1] range
      MULTSUB=-3,	//!< Failed to propagate subgradients for a product term with Tsoukalas & Mitsos's multivariable composition result
      ENVEL, 	//!< Failed to compute the convex or concave envelope of a univariate term
      SUB	//!< Inconsistent subgradient dimension between two mc::McCormick variables
    };
    //! @brief Constructor for error <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}
    //! @brief Inline function returning the error flag
    int ierr(){ return _ierr; }
    //! @brief Return error description
    std::string what(){
      switch( _ierr ){
      case DIV:
        return "mc::McCormick\t Division by zero";
      case INV:
        return "mc::McCormick\t Inverse with zero in range";
      case LOG:
        return "mc::McCormick\t Log with negative values in range";
      case SQRT:
        return "mc::McCormick\t Square-root with nonpositive values in range";
      case ASIN:
        return "mc::McCormick\t Inverse sine with values outside of [-1,1] range";
      case TAN:
        return "mc::McCormick\t Tangent with values pi/2+k*pi in range";
      case CHEB:
        return "mc::McCormick\t Chebyshev basis outside of [-1,1] range";
      case MULTSUB:
        return "mc::McCormick\t Subgradient propagation failed";
      case ENVEL:
        return "mc::McCormick\t Convex/concave envelope computation failed";
      case SUB:
        return "mc::McCormick\t Inconsistent subgradient dimension";
      }
      return "mc::McCormick\t Undocumented error";
    }

  private:
    TYPE _ierr;
  };

  //! @brief Default constructor (needed to declare arrays of McCormick class)
  McCormick():
    _nsub(0), _cvsub(0), _ccsub(0), _const(true)
    {}
  //! @brief Constructor for a constant value <a>c</a>
  McCormick
    ( const double c ):
    _nsub(0), _cv(c), _cc(c), _cvsub(0), _ccsub(0), _const(true)
    {
      Op<T>::I(_I,c);
    }
  //! @brief Constructor for an interval I
  McCormick
    ( const T&I ):
    _nsub(0), _cvsub(0), _ccsub(0), _const(true)
    {
      Op<T>::I(_I,I);
      _cv = Op<T>::l(I); _cc = Op<T>::u(I);
    }
  //! @brief Constructor for a variable, whose range is <a>I</a> and value is <a>c</a>
  McCormick
    ( const T&I, const double c ):
    _nsub(0), _cv(c), _cc(c), _cvsub(0), _ccsub(0), _const(false)
    {
      Op<T>::I(_I,I);
    }
  //! @brief Constructor for a variable, whose range is <a>I</a> and convex and concave bounds are <a>cv</a> and <a>cc</a>
  McCormick
    ( const T&I, const double cv, const double cc ):
    _nsub(0), _cv(cv), _cc(cc), _cvsub(0), _ccsub(0), _const(false)
    {
      Op<T>::I(_I,I); cut();
    }
  //! @brief Copy constructor
  McCormick
    ( const McCormick<T>&MC ):
    _nsub(MC._nsub), _cv(MC._cv), _cc(MC._cc),
    _cvsub(_nsub>0?new double[_nsub]:0), _ccsub(_nsub>0?new double[_nsub]:0),
    _const(MC._const)
    {
      Op<T>::I(_I,MC._I);
      for ( unsigned int ip=0; ip<_nsub; ip++ ){
        _cvsub[ip] = MC._cvsub[ip];
        _ccsub[ip] = MC._ccsub[ip];
      }
    }
  //! @brief Copy constructor doing type conversion for underlying interval
  template <typename U> McCormick
    ( const McCormick<U>&MC ):
    _nsub(MC._nsub), _cv(MC._cv), _cc(MC._cc),
    _cvsub(_nsub>0?new double[_nsub]:0), _ccsub(_nsub>0?new double[_nsub]:0),
    _const(MC._const)
    {
      Op<T>::I(_I,MC._I);
      for ( unsigned int ip=0; ip<_nsub; ip++ ){
        _cvsub[ip] = MC._cvsub[ip];
        _ccsub[ip] = MC._ccsub[ip];
      }
    }

  //! @brief Destructor
  ~McCormick()
    {
      delete [] _cvsub;
      delete [] _ccsub;
    }

  //! @brief Number of subgradient components/directions
  unsigned int& nsub()
    {
      return _nsub;
    }
  unsigned int nsub() const
    {
      return _nsub;
    }
  //! @brief Interval bounds
  T& I()
    {
      return _I;
    }
  const T& I() const
    {
      return _I;
    }
  //! @brief Lower bound
  double l() const
    {
      return Op<T>::l(_I);
    }
  //! @brief Upper bound
  double u() const
    {
      return Op<T>::u(_I);
    }
  //! @brief Convex bound
  double& cv()
    {
      return _cv;
    }
  double cv() const
    {
      return _cv;
    }
  //! @brief Concave bound
  double& cc()
    {
      return _cc;
    }
  double cc() const
    {
      return _cc;
    }
  //! @brief Pointer to a subgradient of convex underestimator
  double*& cvsub()
    {
      return _cvsub;
    }
  const double* cvsub() const
    {
      return _cvsub;
    }
  //! @brief Pointer to a subgradient of concave overestimator
  double*& ccsub()
    {
      return _ccsub;
    }
  const double* ccsub() const
    {
      return _ccsub;
    }
  //! @brief <a>i</a>th component of a subgradient of convex underestimator
  double& cvsub
    ( const unsigned int i )
    {
      return _cvsub[i];
    }
  double cvsub
    ( const unsigned int i ) const
    {
      return _cvsub[i];
    }
  //! @brief <a>i</a>th component of a subgradient of concave overestimator
  double& ccsub
    ( const unsigned int i )
    {
      return _ccsub[i];
    }
  double ccsub
    ( const unsigned int i ) const
    {
      return _ccsub[i];
    }

  //! @brief Set interval bounds
  void I
    ( const T& I )
    {
      Op<T>::I(_I,I);
    }
  //! @brief Set convex bound to <a>cv</a>
  void cv
    ( const double& cv )
    {
      _cv = cv;
      _const = false;
    }
  //! @brief Set concave bound to <a>cc</a>
  void cc
    ( const double& cc )
    {
      _cc = cc;
      _const = false;
    }
  //! @brief Set both convex and concave bounds to <a>c</a>
  void c
    ( const double& c )
    {
      _cv = _cc = c;
      _const = false;
    }

  //! @brief Set dimension of subgradient to <a>nsub</a>
  McCormick<T>& sub
    ( const unsigned int nsub);
  //! @brief Set dimension of subgradient to <a>nsub</a> and variable index <a>isub</a> (starts at 0)
  McCormick<T>& sub
    ( const unsigned int nsub, const unsigned int isub );
  //! @brief Set dimension of subgradient to <a>nsub</a> and subgradient values for the convex and concave relaxations to <a>cvsub</a> and <a>ccsub</a>
  McCormick<T>& sub
    ( const unsigned int nsub, const double*cvsub, const double*ccsub );

  //! @brief Cut convex/concave relaxations at interval bound
  McCormick<T>& cut();
  //! @brief Compute affine underestimator at <a>p</a> based on a subgradient value of the convex underestimator at <a>pref</a>
  double laff
    ( const double*p, const double*pref ) const;
  //! @brief Compute lower bound on range <a>Ip</a> based on an affine underestimator of the convex underestimator at <a>pref</a>
  double laff
    ( const T*Ip, const double*pref ) const;
  //! @brief Compute affine overestimator at <a>p</a> based on a subgradient value of the concave overestimator at <a>pref</a>
  double uaff
    ( const double*p, const double*pref ) const;
  //! @brief Compute upper bound on range <a>Ip</a> based on an affine overestimator of the concave overestimator at <a>pref</a>
  double uaff
    ( const T*Ip, const double*pref ) const;
  /** @} */
  
private:

  //! @brief Number of subgradient components
  unsigned int _nsub;
  //! @brief Interval bounds
  T _I;
  //! @brief Convex bound
  double _cv;
  //! @brief Concave overestimator
  double _cc;
  //! @brief Subgradient of convex underestimator
  double *_cvsub;
  //! @brief Subgradient of concave overestimator
  double *_ccsub;
  //! @brief Whether the convex/concave bounds are constant
  bool _const;
  
  //! @brief Set subgradient size to <a>nsub</a> and specifiy if convex/concave bounds are constant with <a>cst</a>
  void _sub
    ( const unsigned int nsub, const bool cst );
  //! @brief Reset subgradient arrays
  void _sub_reset();
  //! @brief Resize subgradient arrays
  void _sub_resize
    ( const unsigned int nsub );
  //! @brief Copy subgradient arrays
  void _sub_copy
    ( const McCormick<T>&MC );

  //! @brief Compute McCormick relaxation of summation term u1+u2 with u2 constant
  McCormick<T>& _sum1
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of summation term u1+u2 with neither u1 nor u2 constant
  McCormick<T>& _sum2
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );

  //! @brief Compute McCormick relaxation of subtraction term u1-u2 with u2 constant
  McCormick<T>& _sub1
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of subtraction term u1-u2 with u1 constant
  McCormick<T>& _sub2
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of subtraction term u1-u2 with neither u1 nor u2 constant
  McCormick<T>& _sub3
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );

  //! @brief Compute McCormick relaxation of product term u1*u2 in the case u1 >= 0 and u2l >= 0, with u2 constant
  McCormick<T>& _mul1_u1pos_u2pos
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of product term u1*u2 in the case u1 >= 0 and u2l >= 0, with neither u1 nor u2 constant
  McCormick<T>& _mul2_u1pos_u2pos
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of product term u1*u2 in the case u1l >= 0 and u2l <= 0 <= u2u, with u2 constant
  McCormick<T>& _mul1_u1pos_u2mix
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of product term u1*u2 in the case u1l >= 0 and u2l <= 0 <= u2u, with u1 constant
  McCormick<T>& _mul2_u1pos_u2mix
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of product term u1*u2 in the case u1l >= 0 and u2l <= 0 <= u2u, with neither u1 nor u2 constant
  McCormick<T>& _mul3_u1pos_u2mix
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of product term u1*u2 in the case u1l <= 0 <= u1u and u2l <= 0 <= u2u, with u2 constant
  McCormick<T>& _mul1_u1mix_u2mix
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of product term u1*u2 in the case u1l <= 0 <= u1u and u2l <= 0 <= u2u, with neither u1 nor u2 constant
  McCormick<T>& _mul2_u1mix_u2mix
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );
  //! @brief Compute McCormick relaxation of product term u1*u2 using Tsoukalas & Mitsos multivariable composition result
  McCormick<T>& _mulMV
    ( const McCormick<T>&MC1, const McCormick<T>&MC2 );

  //! @brief Prototype function for finding junction points in convex/concave envelopes of univariate terms
  typedef double (puniv)
    ( const double x, const double*rusr, const int*iusr );
  //! @brief Newton method for root finding 
  static double _newton
    ( const double x0, const double xL, const double xU, puniv f,
      puniv df, const double*rusr, const int*iusr=0 );
  //! @brief Secant method for root finding 
  static double _secant
    ( const double x0, const double x1, const double xL, const double xU,
      puniv f, const double*rusr, const int*iusr );
  //! @brief Golden section search method for root finding 
  static double _goldsect
    ( const double xL, const double xU, puniv f, const double*rusr,
      const int*iusr );
  //! @brief Golden section search iterations 
  static double _goldsect_iter
    ( const bool init, const double a, const double fa, const double b,
      const double fb, const double c, const double fc, puniv f,
      const double*rusr, const int*iusr );

  //! @brief Compute convex envelope of odd power terms
  static double* _oddpowcv
    ( const double x, const int iexp, const double xL, const double xU );
  //! @brief Compute concave envelope of odd power terms
  static double* _oddpowcc
    ( const double x, const int iexp, const double xL, const double xU );
  //! @brief Compute residual value for junction points in the envelope of odd power terms
  static double _oddpowenv_func
    ( const double x, const double*rusr, const int*iusr );
  //! @brief Compute residual derivative for junction points in the envelope of odd power terms
  static double _oddpowenv_dfunc
    ( const double x, const double*rusr, const int*iusr );

  //! @brief Compute convex envelope of odd Chebyshev terms
  static double* _oddchebcv
    ( const double x, const int iord, const double xL, const double xU );
  //! @brief Compute concave envelope of odd Chebyshev terms
  static double* _oddchebcc
    ( const double x, const int iord, const double xL, const double xU );
  //! @brief Compute convex envelope of even Chebyshev terms
  static double* _evenchebcv
    ( const double x, const int iord, const double xL, const double xU );

  //! @brief Compute convex envelope of erf terms
  static double* _erfcv
    ( const double x, const double xL, const double xU );
  //! @brief Compute concave envelope of erf terms
  static double* _erfcc
    ( const double x, const double xL, const double xU );
  //! @brief Compute residual value for junction points in the envelope of erf terms
  static double _erfenv_func
    ( const double x, const double*rusr, const int*iusr );
  //! @brief Compute residual derivative for junction points in the envelope of erf terms
  static double _erfenv_dfunc
    ( const double x, const double*rusr, const int*iusr );

  //! @brief Compute convex envelope of atan terms
  static double* _atancv
    ( const double x, const double xL, const double xU );
  //! @brief Compute concave envelope of atan terms
  static double* _atancc
    ( const double x, const double xL, const double xU );
  //! @brief Compute residual value for junction points in the envelope of atan terms
  static double _atanenv_func
    ( const double x, const double*rusr, const int*iusr );
  //! @brief Compute residual derivative for junction points in the envelope of atan terms
  static double _atanenv_dfunc
    ( const double x, const double*rusr, const int*iusr );

  //! @brief Compute convex envelope of a step at 0
  static double* _stepcv
    ( const double x, const double xL, const double xU );
  //! @brief Compute concave envelope of a step at 0
  static double* _stepcc
    ( const double x, const double xL, const double xU );

  //! @brief Compute arg min & arg max for the cos envelope
  static double* _cosarg
    (  const double xL, const double xU );
  //! @brief Compute convex envelope of cos terms
  static double* _coscv
    ( const double x, const double xL, const double xU );
  //! @brief Compute concave envelope of cos terms
  static double* _coscc
    ( const double x, const double xL, const double xU );
  //! @brief Compute convex envelope of cos terms in [-PI,PI]
  static double* _coscv2
    ( const double x, const double xL, const double xU );
  //! @brief Compute residual value for junction points in the envelope of cos terms
  static double _cosenv_func
    ( const double x, const double*rusr, const int*iusr );
  //! @brief Compute residual derivative for junction points in the envelope of cos terms
  static double _cosenv_dfunc
    ( const double x, const double*rusr, const int*iusr );

  //! @brief Compute convex envelope of asin terms
  static double* _asincv
    ( const double x, const double xL, const double xU );
  //! @brief Compute concave envelope of asin terms
  static double* _asincc
    ( const double x, const double xL, const double xU );
  //! @brief Compute residual value for junction points in the envelope of asin terms
  static double _asinenv_func
    ( const double x, const double*rusr, const int*iusr );
  //! @brief Compute residual derivative for junction points in the envelope of asin terms
  static double _asinenv_dfunc
    ( const double x, const double*rusr, const int*iusr );

  //! @brief Compute convex envelope of tan terms
  static double* _tancv
    ( const double x, const double xL, const double xU );
  //! @brief Compute concave envelope of tan terms
  static double* _tancc
    ( const double x, const double xL, const double xU );
  //! @brief Compute residual value for junction points in the envelope of tan terms
  static double _tanenv_func
    ( const double x, const double*rusr, const int*iusr );
  //! @brief Compute residual derivative for junction points in the envelope of tan terms
  static double _tanenv_dfunc
    ( const double x, const double*rusr, const int*iusr );
};

////////////////////////////////////////////////////////////////////////

template <typename T> inline void
McCormick<T>::_sub_reset()
{
  delete [] _cvsub;
  delete [] _ccsub;
  _cvsub = _ccsub = 0;
}

template <typename T> inline void
McCormick<T>::_sub_resize
( const unsigned int nsub )
{
  if( _nsub != nsub ){
    delete [] _cvsub;
    delete [] _ccsub;
    _nsub = nsub;
    if( _nsub > 0 ){
      _cvsub = new double[_nsub];
      _ccsub = new double[_nsub];
    }
    else{
      _cvsub = _ccsub = 0;
      return;
    }
  }
}

template <typename T> inline void
McCormick<T>::_sub_copy
( const McCormick<T>&MC )
{
  _sub_resize( MC._nsub );
  for ( unsigned int i=0; i<_nsub; i++ ){
    _cvsub[i] = MC._cvsub[i];
    _ccsub[i] = MC._ccsub[i];
  }
  return;
}

template <typename T> inline void
McCormick<T>::_sub
( const unsigned int nsub, const bool cst )
{
  _sub_resize( nsub );
  for ( unsigned int i=0; i<nsub; i++ ){
    _cvsub[i] = _ccsub[i] = 0.;
  }
  _const = cst;
}

template <typename T> inline McCormick<T>&
McCormick<T>::sub
( const unsigned int nsub )
{
  _sub( nsub, false );
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::sub
( const unsigned int nsub, const unsigned int isub )
{
  if( isub >= nsub ) throw Exceptions( Exceptions::SUB );
  sub( nsub );
  _cvsub[isub] = _ccsub[isub] = 1.;
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::sub
( const unsigned int nsub, const double*cvsub, const double*ccsub )
{
  if( nsub && !(cvsub && ccsub) ) throw Exceptions( Exceptions::SUB );
  sub( nsub );
  for ( unsigned int i=0; i<nsub; i++ ){
    _cvsub[i] = cvsub[i];
    _ccsub[i] = ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::cut()
{
  if( _cv < Op<T>::l(_I) ){
    _cv = Op<T>::l(_I);
    for( unsigned int i=0; i<_nsub; i++ ) _cvsub[i] = 0.;
  }
  if( _cc > Op<T>::u(_I) ){
    _cc = Op<T>::u(_I);
    for( unsigned int i=0; i<_nsub; i++ ) _ccsub[i] = 0.;
  }
  return *this;
}

template <typename T> inline double
McCormick<T>::laff
( const double*p, const double*pref ) const
{
  double _laff = _cv;
  for( unsigned int i=0; i<_nsub; i++ ){
    _laff += _cvsub[i]*(p[i]-pref[i]);
  }
  return _laff;
}

template <typename T> inline double
McCormick<T>::laff
( const T*Ip, const double*pref ) const
{
  double _laff = _cv;
  for( unsigned int i=0; i<_nsub; i++ ){
    _laff += Op<T>::l(_cvsub[i]*(Ip[i]-pref[i]));
  }
  return _laff;
}

template <typename T> inline double
McCormick<T>::uaff
( const double*p, const double*pref ) const
{
  double _uaff = _cc;
  for( unsigned int i=0; i<_nsub; i++ ){
    _uaff += _ccsub[i]*(p[i]-pref[i]);
  }
  return _uaff;
}

template <typename T> inline double
McCormick<T>::uaff
( const T*Ip, const double*pref ) const
{
  double _uaff = _cc;
  for( unsigned int i=0; i<_nsub; i++ ){
    _uaff += Op<T>::u(_ccsub[i]*(Ip[i]-pref[i]));
  }
  return _uaff;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator=
( const double c )
{
  _I = c;
  _cv = _cc = c;
  _sub_reset();  
  _nsub = 0;
  _const = true; 
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator=
( const T&I )
{
  _I = I;
  _cv = Op<T>::l(I);
  _cc = Op<T>::u(I);
  _sub_reset();
  _nsub = 0;
  _const = true;
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator=
( const McCormick<T>&MC )
{
  if( this == &MC ) return *this;
  _I = MC._I;
  _cv = MC._cv;
  _cc = MC._cc;
  _sub_copy( MC );
  _const = MC._const;
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator+=
( const double a )
{ 
  _I += a;
  _cv += a;
  _cc += a; 
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator+=
( const McCormick<T> &MC )
{
  if( _const && !MC._const ) sub( MC._nsub );
  else if( !MC._const && _nsub != MC._nsub ) throw Exceptions( Exceptions::SUB );
  _I += MC._I;
  _cv += MC._cv;
  _cc += MC._cc;
  for( unsigned int i=0; i<_nsub && !MC._const; i++ ){
    _cvsub[i] += MC._cvsub[i];
    _ccsub[i] += MC._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_sum1
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I + MC2._I;
  _cv = MC1._cv + MC2._cv;
  _cc = MC1._cc + MC2._cc;
  for( unsigned int i=0; i<_nsub; i++ ){
    _cvsub[i] = MC1._cvsub[i];
    _ccsub[i] = MC1._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_sum2
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I + MC2._I;
  _cv = MC1._cv + MC2._cv;
  _cc = MC1._cc + MC2._cc;
  for( unsigned int i=0; i<_nsub; i++ ){
    _cvsub[i] = MC1._cvsub[i] + MC2._cvsub[i];
    _ccsub[i] = MC1._ccsub[i] + MC2._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator-=
( const double a )
{ 
  _I -= a;
  _cv -= a;
  _cc -= a; 
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator-=
( const McCormick<T> &MC )
{
  if( _const && !MC._const ) sub( MC._nsub );
  else if( !MC._const && _nsub != MC._nsub ) throw Exceptions( Exceptions::SUB );
  _I -= MC._I;
  double t_cv = MC._cv;
  _cv -= MC._cc;
  _cc -= t_cv;
  for( unsigned int i=0; i<_nsub && !MC._const; i++ ){
    double t_cvsub = MC._cvsub[i];
    _cvsub[i] -= MC._ccsub[i];
    _ccsub[i] -= t_cvsub;
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_sub1
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I - MC2._I;
  _cv = MC1._cv - MC2._cc;
  _cc = MC1._cc - MC2._cv;
  for( unsigned int i=0; i<_nsub; i++ ){
    _cvsub[i] = MC1._cvsub[i];
    _ccsub[i] = MC1._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_sub2
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I - MC2._I;
  _cv = MC1._cv - MC2._cc;
  _cc = MC1._cc - MC2._cv;
  for( unsigned int i=0; i<_nsub; i++ ){
    _cvsub[i] = -MC2._ccsub[i];
    _ccsub[i] = -MC2._cvsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_sub3
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I - MC2._I;
  _cv = MC1._cv - MC2._cc;
  _cc = MC1._cc - MC2._cv;
  for( unsigned int i=0; i<_nsub; i++ ){
    _cvsub[i] = MC1._cvsub[i] - MC2._ccsub[i];
    _ccsub[i] = MC1._ccsub[i] - MC2._cvsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator*=
( const double a )
{
  McCormick<T> MC2 = a * (*this);
  *this = MC2;
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator*=
( const McCormick<T>&MC )
{
  if( _const && !MC._const ) sub( MC._nsub );
  McCormick<T> MC2 = MC * (*this);
  *this = MC2;
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mul1_u1pos_u2pos
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I * MC2._I;

  double cv1 = Op<T>::u(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cv
    - Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
  double cv2 = Op<T>::l(MC2._I) * MC1._cv + Op<T>::l(MC1._I) * MC2._cv
    - Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
  if ( cv1 > cv2 ){
    _cv = cv1;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::u(MC2._I) * MC1._cvsub[i];
  }
  else{
    _cv = cv2;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::l(MC2._I) * MC1._cvsub[i];
  }

  double cc1 = Op<T>::l(MC2._I) * MC1._cc + Op<T>::u(MC1._I) * MC2._cc
    - Op<T>::u(MC1._I) * Op<T>::l(MC2._I);
  double cc2 = Op<T>::u(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cc
    - Op<T>::l(MC1._I) * Op<T>::u(MC2._I);
  if ( cc1 < cc2 ){
    _cc = cc1;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::l(MC2._I) * MC1._ccsub[i];
  }
  else{
    _cc = cc2;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::u(MC2._I) * MC1._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mul2_u1pos_u2pos
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I * MC2._I;

  double cv1 = Op<T>::u(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cv
    - Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
  double cv2 = Op<T>::l(MC2._I) * MC1._cv + Op<T>::l(MC1._I) * MC2._cv
    - Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
  if ( cv1 > cv2 ){
    _cv = cv1;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::u(MC2._I) * MC1._cvsub[i] + Op<T>::u(MC1._I) * MC2._cvsub[i];
  }
  else{
    _cv = cv2;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::l(MC2._I) * MC1._cvsub[i] + Op<T>::l(MC1._I) * MC2._cvsub[i];
  }

  double cc1 = Op<T>::l(MC2._I) * MC1._cc + Op<T>::u(MC1._I) * MC2._cc
    - Op<T>::u(MC1._I) * Op<T>::l(MC2._I);
  double cc2 = Op<T>::u(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cc
    - Op<T>::l(MC1._I) * Op<T>::u(MC2._I);
  if ( cc1 < cc2 ){
    _cc = cc1;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::l(MC2._I) * MC1._ccsub[i] + Op<T>::u(MC1._I) * MC2._ccsub[i];
  }
  else{
    _cc = cc2;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::u(MC2._I) * MC1._ccsub[i] + Op<T>::l(MC1._I) * MC2._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mul1_u1pos_u2mix
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I * MC2._I;

  double cv1 = Op<T>::u(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cv
    - Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
  double cv2 = Op<T>::l(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cv
    - Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
  if ( cv1 > cv2 ){
    _cv = cv1;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::u(MC2._I) * MC1._cvsub[i];
  }
  else{
    _cv = cv2;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::l(MC2._I) * MC1._ccsub[i];
  }

  double cc1 = Op<T>::l(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cc
    - Op<T>::u(MC1._I) * Op<T>::l(MC2._I);
  double cc2 = Op<T>::u(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cc
    - Op<T>::l(MC1._I) * Op<T>::u(MC2._I);
  if ( cc1 < cc2 ){
    _cc = cc1;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::l(MC2._I) * MC1._cvsub[i];
  }
  else{
    _cc = cc2;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::u(MC2._I) * MC1._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mul2_u1pos_u2mix
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I * MC2._I;

  double cv1 = Op<T>::u(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cv
    - Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
  double cv2 = Op<T>::l(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cv
    - Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
  if ( cv1 > cv2 ){
    _cv = cv1;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::u(MC1._I) * MC2._cvsub[i];
  }
  else{
    _cv = cv2;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::l(MC1._I) * MC2._cvsub[i];
  }

  double cc1 = Op<T>::l(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cc
    - Op<T>::u(MC1._I) * Op<T>::l(MC2._I);
  double cc2 = Op<T>::u(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cc
    - Op<T>::l(MC1._I) * Op<T>::u(MC2._I);
  if ( cc1 < cc2 ){
    _cc = cc1;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::u(MC1._I) * MC2._ccsub[i];
  }
  else{
    _cc = cc2;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::l(MC1._I) * MC2._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mul3_u1pos_u2mix
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I * MC2._I;

  double cv1 = Op<T>::u(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cv
    - Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
  double cv2 = Op<T>::l(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cv
    - Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
  if ( cv1 > cv2 ){
    _cv = cv1;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::u(MC2._I) * MC1._cvsub[i] + Op<T>::u(MC1._I) * MC2._cvsub[i];
  }
  else{
    _cv = cv2;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::l(MC2._I) * MC1._ccsub[i] + Op<T>::l(MC1._I) * MC2._cvsub[i];
  }

  double cc1 = Op<T>::l(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cc
    - Op<T>::u(MC1._I) * Op<T>::l(MC2._I);
  double cc2 = Op<T>::u(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cc
    - Op<T>::l(MC1._I) * Op<T>::u(MC2._I);
  if ( cc1 < cc2 ){
    _cc = cc1;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::l(MC2._I) * MC1._cvsub[i] + Op<T>::u(MC1._I) * MC2._ccsub[i];
  }
  else{
    _cc = cc2;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::u(MC2._I) * MC1._ccsub[i] + Op<T>::l(MC1._I) * MC2._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mul1_u1mix_u2mix
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I * MC2._I;

  double cv1 = Op<T>::u(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cv
    - Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
  double cv2 = Op<T>::l(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cc
    - Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
  if ( cv1 > cv2 ){
    _cv = cv1;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::u(MC2._I) * MC1._cvsub[i];
  }
  else{
    _cv = cv2;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::l(MC2._I) * MC1._ccsub[i];
  }

  double cc1 = Op<T>::l(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cc
    - Op<T>::u(MC1._I) * Op<T>::l(MC2._I);
  double cc2 = Op<T>::u(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cv
    - Op<T>::l(MC1._I) * Op<T>::u(MC2._I);
  if ( cc1 < cc2 ){
    _cc = cc1;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::l(MC2._I) * MC1._cvsub[i];
  }
  else{
    _cc = cc2;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::u(MC2._I) * MC1._ccsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mul2_u1mix_u2mix
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  _I = MC1._I * MC2._I;

  double cv1 = Op<T>::u(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cv
    - Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
  double cv2 = Op<T>::l(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cc
    - Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
  if ( cv1 > cv2 ){
    _cv = cv1;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::u(MC2._I) * MC1._cvsub[i] + Op<T>::u(MC1._I) * MC2._cvsub[i];
  }
  else{
    _cv = cv2;
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = Op<T>::l(MC2._I) * MC1._ccsub[i] + Op<T>::l(MC1._I) * MC2._ccsub[i];
  }

  double cc1 = Op<T>::l(MC2._I) * MC1._cv + Op<T>::u(MC1._I) * MC2._cc
    - Op<T>::u(MC1._I) * Op<T>::l(MC2._I);
  double cc2 = Op<T>::u(MC2._I) * MC1._cc + Op<T>::l(MC1._I) * MC2._cv
    - Op<T>::l(MC1._I) * Op<T>::u(MC2._I);
  if ( cc1 < cc2 ){
    _cc = cc1;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::l(MC2._I) * MC1._cvsub[i] + Op<T>::u(MC1._I) * MC2._ccsub[i];
  }
  else{
    _cc = cc2;
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = Op<T>::u(MC2._I) * MC1._ccsub[i] + Op<T>::l(MC1._I) * MC2._cvsub[i];
  }
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::_mulMV
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
 // Convex underestimator part
 {const double k = - Op<T>::diam(MC2._I) / Op<T>::diam(MC1._I);
  const double z = ( Op<T>::u(MC1._I)*Op<T>::u(MC2._I)
                   - Op<T>::l(MC1._I)*Op<T>::l(MC2._I) )
                   / Op<T>::diam(MC1._I);
  struct fct{
    static double t1
      ( const double x1, const double x2, const McCormick<T>&MC1,
        const McCormick<T>&MC2 )
      { return Op<T>::u(MC2._I) * x1 + Op<T>::u(MC1._I) * x2
	     - Op<T>::u(MC2._I) * Op<T>::u(MC1._I); }
    static double t2
      ( const double x1, const double x2, const McCormick<T>&MC1,
        const McCormick<T>&MC2 )
      { return Op<T>::l(MC2._I) * x1 + Op<T>::l(MC1._I) * x2
	     - Op<T>::l(MC2._I) * Op<T>::l(MC1._I); }
    static double t
      ( const double x1, const double x2, const McCormick<T>&MC1,
        const McCormick<T>&MC2 )
      { return std::max( t1(x1,x2,MC1,MC2), t2(x1,x2,MC1,MC2) ); }
  };

  // Modified @ AVT.SVT, Aug 23, 2016
  // After realizing that the closed form for multiplication given in 
  // Tsoukalas & Mitsos 2014 is not correct
  int imid[4] = { -1, -1, -1, -1 };
  const double x1t[6] = { MC1._cv, MC1._cc,
                          mid( MC1._cv, MC1._cc, (MC2._cv-z)/k, imid[0] ),
						  mid( MC1._cv, MC1._cc, (MC2._cc-z)/k, imid[1] ),
						  // added:
                          MC1._cv, MC1._cc
						};
  const double x2t[6] = { mid( MC2._cv, MC2._cc, k*MC1._cv+z, imid[2] ),
						  mid( MC2._cv, MC2._cc, k*MC1._cc+z, imid[3] ),
                          MC2._cv, MC2._cc,
						  // added:
                          MC2._cv, MC2._cc
						};	
  const double v[6] = { fct::t( x1t[0], x2t[0], MC1, MC2 ), fct::t( x1t[1], x2t[1], MC1, MC2 ),
                        fct::t( x1t[2], x2t[2], MC1, MC2 ), fct::t( x1t[3], x2t[3], MC1, MC2 ),
  // added the two corners (MC1._cv,MC2._cv),(MC1._cc,MC2._cc) for the convex relaxation specifically, since
  // they can be excluded by the mid() term if the envelope of the multiplication is monotone
                        fct::t( x1t[4], x2t[4], MC1, MC2 ), 
                        fct::t( x1t[5], x2t[5], MC1, MC2 )
                        };

  const unsigned int ndx = argmin( 6, v ); // 6 elements now
  _cv = v[ndx];
  if( _nsub ){
    double myalpha;
    if( isequal( fct::t1( x1t[ndx], x2t[ndx], MC1, MC2 ),
                 fct::t2( x1t[ndx], x2t[ndx], MC1, MC2 ),
		 options.MVCOMP_TOL, options.MVCOMP_TOL ) ){
      std::pair<double,double> alpha( 0., 1. );
     bool MC1thin = isequal( MC1._cv, MC1._cc, options.MVCOMP_TOL, options.MVCOMP_TOL )?
        true: false; 
      if( !MC1thin && x1t[ndx] > MC1._cv ){
      // Modified @ AVT.SVT, Aug 29, 2016
      // We had to add an additional if-statement where we question if the two values x[ndx] and MC._cv 
      // are equal since we only work with a given tolerance (MVCOMP_TOL) 
      // the if-statement is added 4 times in the convex subgradient and 4 times in the concave subgradient
        if(!isequal(x1t[ndx], MC1._cv, options.MVCOMP_TOL, options.MVCOMP_TOL)){
          alpha.second = std::min( alpha.second, -Op<T>::l(MC2._I)/Op<T>::diam(MC2._I) );
        }
      }
      if( !MC1thin && x1t[ndx] < MC1._cc ){
        if(!isequal(x1t[ndx], MC1._cc, options.MVCOMP_TOL, options.MVCOMP_TOL)){
          alpha.first = std::max( alpha.first, -Op<T>::l(MC2._I)/Op<T>::diam(MC2._I) );
        }
      }
      bool MC2thin = isequal( MC2._cv, MC2._cc, options.MVCOMP_TOL, options.MVCOMP_TOL )?
        true: false; 
      if( !MC2thin && x2t[ndx] > MC2._cv ){
        if(!isequal(x2t[ndx], MC2._cv, options.MVCOMP_TOL, options.MVCOMP_TOL)){
          alpha.second = std::min( alpha.second, -Op<T>::l(MC1._I)/Op<T>::diam(MC1._I) );
        }
      }
      if( !MC2thin && x2t[ndx] < MC2._cc ){
        if(!isequal(x2t[ndx], MC2._cc, options.MVCOMP_TOL, options.MVCOMP_TOL)){
          alpha.first = std::max( alpha.first, -Op<T>::l(MC1._I)/Op<T>::diam(MC1._I) );
        }
      }        
      bool alphathin = isequal( alpha.first, alpha.second, options.MVCOMP_TOL, options.MVCOMP_TOL )?
        true: false;
      if( !alphathin && alpha.first > alpha.second ){
        std::cout << "WARNING1: alphaL= " << alpha.first << "  alphaU= " << alpha.second
	          << std::endl;
        throw Exceptions( Exceptions::MULTSUB );
      }
      myalpha = 0.5*( alpha.first + alpha.second );
    }
    else if( fct::t1( x1t[ndx], x2t[ndx], MC1, MC2 ) > fct::t2( x1t[ndx], x2t[ndx], MC1, MC2 ) )
      myalpha = 1.;
    else
      myalpha = 0.;
    double sigma1cv = Op<T>::l(MC2._I) + myalpha * Op<T>::diam(MC2._I),
           sigma2cv = Op<T>::l(MC1._I) + myalpha * Op<T>::diam(MC1._I);
    for( unsigned int i=0; i<_nsub; i++ )
      _cvsub[i] = ( sigma1cv>=0? (MC1._const? 0.:MC1._cvsub[i]):
                                 (MC1._const? 0.:MC1._ccsub[i]) ) * sigma1cv
                + ( sigma2cv>=0? (MC2._const? 0.:MC2._cvsub[i]):
		                 (MC2._const? 0.:MC2._ccsub[i]) ) * sigma2cv;
  }
 }

 // Concave overestimator part
 {const double k = Op<T>::diam(MC2._I) / Op<T>::diam(MC1._I);
  const double z = ( Op<T>::u(MC1._I)*Op<T>::l(MC2._I)
                   - Op<T>::l(MC1._I)*Op<T>::u(MC2._I) )
                   / Op<T>::diam(MC1._I);
  struct fct{
    static double t1
      ( const double x1, const double x2, const McCormick<T>&MC1,
        const McCormick<T>&MC2 )
      { return Op<T>::l(MC2._I) * x1 + Op<T>::u(MC1._I) * x2
	     - Op<T>::l(MC2._I) * Op<T>::u(MC1._I); }
    static double t2
      ( const double x1, const double x2, const McCormick<T>&MC1,
        const McCormick<T>&MC2 )
      { return Op<T>::u(MC2._I) * x1 + Op<T>::l(MC1._I) * x2
	     - Op<T>::u(MC2._I) * Op<T>::l(MC1._I); }
    static double t
      ( const double x1, const double x2, const McCormick<T>&MC1,
        const McCormick<T>&MC2 )
      { return std::min( t1(x1,x2,MC1,MC2), t2(x1,x2,MC1,MC2) ); }
  };

  // Modified @ AVT.SVT, Aug 23, 2016
  // After realizing that the closed form for multiplication given in 
  // Tsoukalas & Mitsos 2014 is not correct
  int imid[4] = { -1, -1, -1, -1 };
  const double x1t[6] = { MC1._cv, MC1._cc,
                          mid( MC1._cv, MC1._cc, (MC2._cv-z)/k, imid[0] ),
                          mid( MC1._cv, MC1._cc, (MC2._cc-z)/k, imid[1] ),
						  // added:
                          MC1._cv, MC1._cc 
						};
  const double x2t[6] = { mid( MC2._cv, MC2._cc, k*MC1._cv+z, imid[2] ),
                          mid( MC2._cv, MC2._cc, k*MC1._cc+z, imid[3] ),
                          MC2._cv, MC2._cc,
					      // added:
                          MC2._cc, MC2._cv 
						};
  const double v[6] = { fct::t( x1t[0], x2t[0], MC1, MC2 ), fct::t( x1t[1], x2t[1], MC1, MC2 ),
                        fct::t( x1t[2], x2t[2], MC1, MC2 ), fct::t( x1t[3], x2t[3], MC1, MC2 ),
  // added the two corners (MC1._cv,MC2._cc),(MC1._cc,MC2._cv) for the concave relaxation specifically, since
  // they can be excluded by the mid() term if the envelope of the multiplication is monotone                      
                        fct::t( x1t[4], x2t[4], MC1, MC2 ),
                        fct::t( x1t[5], x2t[5], MC1, MC2 )
                        };					
						
  const unsigned int ndx = argmax( 6, v );	 // 6 elements now
  _cc = v[ndx];

  if( _nsub ){
    double myalpha;
    if( isequal( fct::t1( x1t[ndx], x2t[ndx], MC1, MC2 ),
                 fct::t2( x1t[ndx], x2t[ndx], MC1, MC2 ),
		 options.MVCOMP_TOL, options.MVCOMP_TOL ) ){
      std::pair<double,double> alpha( 0., 1. );
      bool MC1thin = isequal( MC1._cv, MC1._cc, options.MVCOMP_TOL, options.MVCOMP_TOL )?
        true: false;
      if( !MC1thin && x1t[ndx] > MC1._cv ){
      // Modified @ AVT.SVT, Aug 29, 2016
      // we had to add an additional if-statement where we question if the two values x[ndx] and MC._cv 
      // are equal since we only work with a given tolerance (MVCOMP_TOL) 
      // the if-statement is added 4 times in the convex subgradient and 4 times in the concave subgradient
        if(!isequal(x1t[ndx], MC1._cv, options.MVCOMP_TOL, options.MVCOMP_TOL)){
         alpha.first = std::max( alpha.first, -Op<T>::l(MC2._I)/Op<T>::diam(MC2._I) );
        }
      }
      if( !MC1thin && x1t[ndx] < MC1._cc ){           
        if(!isequal(x1t[ndx], MC1._cc, options.MVCOMP_TOL, options.MVCOMP_TOL)){
          alpha.second = std::min( alpha.second, -Op<T>::l(MC2._I)/Op<T>::diam(MC2._I) );
        }
      }
      bool MC2thin = isequal( MC2._cv, MC2._cc, options.MVCOMP_TOL, options.MVCOMP_TOL )?
        true: false;
      if( !MC2thin && x2t[ndx] > MC2._cv ){
        if(!isequal(x2t[ndx], MC2._cv, options.MVCOMP_TOL, options.MVCOMP_TOL)){
          alpha.second = std::min( alpha.second, Op<T>::u(MC1._I)/Op<T>::diam(MC1._I) );
        }
      }
      if( !MC2thin && x2t[ndx] < MC2._cc ){           
        if(!isequal(x2t[ndx], MC2._cc, options.MVCOMP_TOL, options.MVCOMP_TOL)){
          alpha.first = std::max( alpha.first, Op<T>::u(MC1._I)/Op<T>::diam(MC1._I) );
        }
      }

      bool alphathin = isequal( alpha.first, alpha.second, options.MVCOMP_TOL, options.MVCOMP_TOL )?
        true: false;
      if( !alphathin && alpha.first > alpha.second ){
        std::cout << "WARNING2: alphaL= " << alpha.first << "  alphaU= " << alpha.second
	          << std::endl;
        throw Exceptions( Exceptions::MULTSUB );
      }
      myalpha = 0.5*( alpha.first + alpha.second );
    }
    else if( fct::t1( x1t[ndx], x2t[ndx], MC1, MC2 ) < fct::t2( x1t[ndx], x2t[ndx], MC1, MC2 ) )
      myalpha = 0.;
    else
      myalpha = 1.;
    double sigma1cc = Op<T>::l(MC2._I) + myalpha * Op<T>::diam(MC2._I),
           sigma2cc = Op<T>::u(MC1._I) - myalpha * Op<T>::diam(MC1._I);
    for( unsigned int i=0; i<_nsub; i++ )
      _ccsub[i] = ( sigma1cc>=0? (MC1._const? 0.:MC1._ccsub[i]):
                                 (MC1._const? 0.:MC1._cvsub[i]) ) * sigma1cc
                + ( sigma2cc>=0? (MC2._const? 0.:MC2._ccsub[i]):
		                 (MC2._const? 0.:MC2._cvsub[i]) ) * sigma2cc;
  }
 }

  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator/=
( const double a )
{
  McCormick<T> MC2 = (*this) / a;
  *this = MC2;
  return *this;
}

template <typename T> inline McCormick<T>&
McCormick<T>::operator/=
( const McCormick<T>&MC )
{
  if( _const && !MC._const ) sub( MC._nsub );
  McCormick<T> MC2 = (*this) / MC;
  *this = MC2;
  return *this;
}

template <typename T> inline double*
McCormick<T>::_erfcv
( const double x, const double xL, const double xU )
{
  static double cv[2];
  if( xU <= 0. ){	 // convex part
    cv[0] = ::erf(x), cv[1] = 2./std::sqrt(PI)*std::exp(-sqr(x));
    return cv;
  }

  if( xL >= 0. ){	 // concave part
    double r = ( isequal( xL, xU )? 0.:(::erf(xU)-::erf(xL))/(xU-xL) );
    cv[0] = ::erf(xL)+r*(x-xL), cv[1] = r;
    return cv;
  }
    
  double xj;
  try{
    xj = _newton( xL, xL, 0., _erfenv_func, _erfenv_dfunc, &xU );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( xL, 0., _erfenv_func, &xU, 0 );
  }
  if( x <= xj ){	 // convex part
    cv[0] = ::erf(x), cv[1] = 2./std::sqrt(PI)*std::exp(-sqr(x));
    return cv;
  }
  double r = ( isequal( xj, xU )? 0.:(::erf(xU)-::erf(xj))/(xU-xj) );
  cv[0] = ::erf(xU)+r*(x-xU), cv[1] = r;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_erfcc
( const double x, const double xL, const double xU )
{
  static double cc[2];
  if( xU <= 0. ){	 // convex part
    double r = ( isequal( xL, xU )? 0.:(::erf(xU)-::erf(xL))/(xU-xL) );
    cc[0] = ::erf(xL)+r*(x-xL), cc[1] = r;
    return cc;
  }

  if( xL >= 0. ){	 // concave part
    cc[0] = ::erf(x), cc[1] = 2./std::sqrt(PI)*std::exp(-sqr(x));
    return cc;
  }

  double xj;
  try{
    xj = _newton( xU, 0., xU, _erfenv_func, _erfenv_dfunc, &xL );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( 0., xU, _erfenv_func, &xL, 0 );
  }
  if( x >= xj ){	 // concave part
    cc[0] = ::erf(x), cc[1] = 2./std::sqrt(PI)*std::exp(-sqr(x));
    return cc;
  }
  double r = ( isequal( xj, xL )? 0.:(::erf(xL)-::erf(xj))/(xL-xj) );
  cc[0] = ::erf(xL)+r*(x-xL), cc[1] = r;
  return cc;
}

template <typename T> inline double
McCormick<T>::_erfenv_func
( const double x, const double*rusr, const int*iusr )
{
  // f(z) = (z-a)*exp(-z^2)-sqrt(pi)/2.*(erf(z)-erf(a)) = 0
  return (x-*rusr)*std::exp(-sqr(x))-std::sqrt(PI)/2.*(::erf(x)-::erf(*rusr));
}

template <typename T> inline double
McCormick<T>::_erfenv_dfunc
( const double x, const double*rusr, const int*iusr )
{
  // f'(z) = -2*z*(z-a)*exp(-z^2)
  return -2.*x*(x-*rusr)*std::exp(-2.*sqr(x));
}

template <typename T> inline double*
McCormick<T>::_atancv
( const double x, const double xL, const double xU )
{
  static double cv[2];
  if( xU <= 0. ){	 // convex part
    cv[0] = std::atan(x), cv[1] = 1./(1.+sqr(x));
    return cv;
  }

  if( xL >= 0. ){	 // concave part
    double r = ( isequal( xL, xU )? 0.:(std::atan(xU)-std::atan(xL))/(xU-xL) );
    cv[0] = std::atan(xL)+r*(x-xL), cv[1] = r;
    return cv;
  }
    
  double xj;
  try{
    xj = _newton( xL, xL, 0., _atanenv_func, _atanenv_dfunc, &xU, 0 );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( xL, 0., _atanenv_func, &xU, 0 );
  }
  if( x <= xj ){	 // convex part
    cv[0] = std::atan(x), cv[1] = 1./(1.+sqr(x));
    return cv;
  }
  double r = ( isequal( xj, xU )? 0.:(std::atan(xU)-std::atan(xj))/(xU-xj) );
  cv[0] = std::atan(xU)+r*(x-xU), cv[1] = r;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_atancc
( const double x, const double xL, const double xU )
{
  static double cc[2];
  if( xU <= 0. ){	 // convex part
    double r = ( isequal( xL, xU )? 0.:(std::atan(xU)-std::atan(xL))/(xU-xL) );
    cc[0] = std::atan(xL)+r*(x-xL), cc[1] = r;
    return cc;
  }

  if( xL >= 0. ){	 // concave part
    cc[0] = std::atan(x), cc[1] = 1./(1.+sqr(x));
    return cc;
  }
    
  double xj;
  try{
    xj = _newton( xU, 0., xU, _atanenv_func, _atanenv_dfunc, &xL, 0 );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( 0., xU, _atanenv_func, &xL, 0 );
  }
  if( x >= xj ){	 // concave part
    cc[0] = std::atan(x), cc[1] = 1./(1.+sqr(x));
    return cc;
  }
  double r = ( isequal( xj, xL )? 0.:(std::atan(xL)-std::atan(xj))/(xL-xj) );
  cc[0] = std::atan(xL)+r*(x-xL), cc[1] = r;
  return cc;
}

template <typename T> inline double
McCormick<T>::_atanenv_func
( const double x, const double*rusr, const int*iusr )
{
  // f(z) = z-a-(1+z^2)*(asin(z)-asin(a)) = 0
  return (x-*rusr)-(1.+sqr(x))*(std::atan(x)-std::atan(*rusr));
}

template <typename T> inline double
McCormick<T>::_atanenv_dfunc
( const double x, const double*rusr, const int*iusr )
{
  // f'(z) = -2*z*(asin(z)-asin(a))
  return -2.*x*(std::atan(x)-std::atan(*rusr));
}

template <typename T> inline double*
McCormick<T>::_oddpowcv
( const double x, const int iexp, const double xL, const double xU )
{
  static double cv[2];
  if( xL >= 0. ){	 // convex part
    double v = std::pow(x,iexp-1);
    cv[0] = x*v, cv[1] = iexp*v;
    return cv;
  }

  if( xU <= 0. ){	 // concave part
    double r = ( isequal( xL, xU )? 0.:
      (std::pow(xU,iexp)-std::pow(xL,iexp))/(xU-xL) );
    cv[0] = std::pow(xL,iexp)+r*(x-xL), cv[1] = r;
    return cv;
  }
    
  double xj;
  try{
    xj = _newton( xU, 0., xU, _oddpowenv_func, _oddpowenv_dfunc, &xL, &iexp );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( 0., xU, _oddpowenv_func, &xL, &iexp );
  }
  if( x >= xj ){	 // convex part
    double v = std::pow(x,iexp-1);
    cv[0] = x*v, cv[1] = iexp*v;
    return cv;
  }
  double r = ( isequal( xL, xj )? 0.:
    (std::pow(xj,iexp)-std::pow(xL,iexp))/(xj-xL) );
  cv[0] = std::pow(xL,iexp)+r*(x-xL), cv[1] = r;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_oddpowcc
( const double x, const int iexp, const double xL, const double xU )
{
  static double cc[2];
  if( xL >= 0. ){	 // convex part
    double r = ( isequal( xL, xU )? 0.:
      (std::pow(xU,iexp)-std::pow(xL,iexp))/(xU-xL) );
    cc[0] = std::pow(xL,iexp)+r*(x-xL), cc[1] = r;
    return cc;
  }
  if( xU <= 0. ){	 // concave part
    double v = std::pow(x,iexp-1);
    cc[0] = x*v, cc[1] = iexp*v;
    return cc;
  }

  double xj;
  try{
    xj = _newton( xL, xL, 0., _oddpowenv_func, _oddpowenv_dfunc, &xU, &iexp );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( xL, 0., _oddpowenv_func, &xU, &iexp );
  }
  if( x <= xj ){	 // concave part
    double v = std::pow(x,iexp-1);
    cc[0] = x*v, cc[1] = iexp*v;
    return cc;
  }
  double r = ( isequal( xU, xj )? 0.:
    (std::pow(xj,iexp)-std::pow(xU,iexp))/(xj-xU) );
  cc[0] = std::pow(xU,iexp)+r*(x-xU), cc[1] = r;
  return cc;
}

template <typename T> inline double
McCormick<T>::_oddpowenv_func
( const double x, const double*rusr, const int*iusr )
{
  // f(z) = (p-1)*z^p - a*p*z^{p-1} + a^p = 0
  return ((*iusr-1)*x-(*rusr)*(*iusr))*std::pow(x,*iusr-1)
    + std::pow(*rusr,*iusr);
}

template <typename T> inline double
McCormick<T>::_oddpowenv_dfunc
( const double x, const double*rusr, const int*iusr )
{
  // f'(z) = p*(p-1)*z^{p-1} - a*p*(p-1)*z^{p-2}
  return ((*iusr)*(*iusr-1)*x-(*rusr)*(*iusr)*(*iusr-1))*std::pow(x,*iusr-2);
}

template <typename T> inline double*
McCormick<T>::_evenchebcv
( const double x, const int iord, const double xL, const double xU )
{
  double xjL = std::cos(PI-PI/(double)iord);
  double xjU = std::cos(PI/(double)iord);
  static double cv[2];
  if( x <= xjL || x >= xjU )
    cv[0] = mc::cheb(x,iord), cv[1] = iord*mc::cheb2(x,iord-1);
  else
    cv[0] = -1., cv[1] = 0.;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_oddchebcv
( const double x, const int iord, const double xL, const double xU )
{
  static double cv[2];
  double xj = std::cos(PI/(double)iord);
  if( x >= xj ){	 // convex part
    cv[0] = mc::cheb(x,iord), cv[1] = iord*mc::cheb2(x,iord-1);
    return cv;
  }
  cv[0] = -1., cv[1] = 0.;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_oddchebcc
( const double x, const int iord, const double xL, const double xU )
{
  static double cc[2];
  double xj = std::cos(PI-PI/(double)iord);
  if( x <= xj ){	 // concave part
    cc[0] = mc::cheb(x,iord), cc[1] = iord*mc::cheb2(x,iord-1);
    return cc;
  }
  cc[0] = 1., cc[1] = 0.;
  return cc;
}

template <typename T> inline double*
McCormick<T>::_stepcv
( const double x, const double xL, const double xU )
{
  static double cv[2];

  if( x < 0. ){
    cv[0] = cv[1] = 0.;
    return cv;
  }

  if( xL >= 0. ){
    cv[0] = 1., cv[1] = 0.;
    return cv;
  }

  cv[0] = x/xU, cv[1] = 1./xU;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_stepcc
( const double x, const double xL, const double xU )
{
  static double cc[2];

  if( x >= 0. ){
    cc[0] = 1., cc[1] = 0.;
    return cc;
  }

  else if( xU < 0. ){
    cc[0] = 0., cc[1] = 0.;
    return cc;
  }

  cc[0] = 1.-x/xL, cc[1] = -1./xL;
  return cc;
}

template <typename T> inline double*
McCormick<T>::_cosarg
( const double xL, const double xU )
{
  static double arg[2];
  const int kL = std::ceil(-(1.+xL/PI)/2.);
  const double xL1 = xL+2.*PI*kL, xU1 = xU+2.*PI*kL;
  assert( xL1 >= -PI && xL1 <= PI );
  if( xL1 <= 0 ){
    if( xU1 <= 0 ) arg[0] = xL, arg[1] = xU;
    else if( xU1 >= PI ) arg[0] = PI*(1.-2.*kL), arg[1] = -PI*2.*kL;
    else arg[0] = std::cos(xL1)<=std::cos(xU1)?xL:xU, arg[1] = -PI*2.*kL;
    return arg;
  }
  if( xU1 <= PI ) arg[0] = xU, arg[1] = xL;
  else if( xU1 >= 2.*PI ) arg[0] = PI*(1-2.*kL), arg[1] = 2.*PI*(1.-kL);
  else arg[0] = PI*(1.-2.*kL), arg[1] = std::cos(xL1)>=std::cos(xU1)?xL:xU;
  return arg;
}

template <typename T> inline double*
McCormick<T>::_coscv
( const double x, const double xL, const double xU )
{
  static double cv[2];
  const int kL = std::ceil(-(1.+xL/PI)/2.);
  if( x <= PI*(1-2*kL) ){
    const double xL1 = xL+2.*PI*kL;
    if( xL1 >= 0.5*PI ){
      cv[0] = std::cos(x), cv[1] = -std::sin(x);
      return cv;
    }
    const double xU1 = std::min(xU+2.*PI*kL,PI);
    if( xL1 >= -0.5*PI && xU1 <= 0.5*PI ){
      double r = ( isequal( xL, xU )? 0.: (std::cos(xU)-std::cos(xL))/(xU-xL) );
      cv[0] = std::cos(xL)+r*(x-xL), cv[1] = r;
      return cv;
    }
    return _coscv2( x+2.*PI*kL, xL1, xU1 );
  }

  const int kU = std::floor((1.-xU/PI)/2.);
  if( x >= PI*(-1-2*kU) ){
    const double xU2 = xU+2.*PI*kU;
    if( xU2 <= -0.5*PI ){
      cv[0] = std::cos(x), cv[1] = -std::sin(x);
      return cv;
    }
    return _coscv2( x+2.*PI*kU, std::max(xL+2.*PI*kU,-PI), xU2 );
  }

  cv[0] = -1., cv[1] = 0.;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_coscv2
( const double x, const double xL, const double xU )
{
  bool left;
  double x0, xm;
  if( std::fabs(xL)<=std::fabs(xU) )
    left = false, x0 = xU, xm = xL;
  else
    left = true, x0 = xL, xm = xU;

  double xj;
  try{
    xj = _newton( x0, xL, xU, _cosenv_func, _cosenv_dfunc, &xm, 0 );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( xL, xU, _cosenv_func, &xm, 0 );
  }
  static double cv[2];
  if(( left && x<=xj ) || ( !left && x>=xj )){
    cv[0] = std::cos(x), cv[1] = -std::sin(x);
    return cv;
  }
  double r = ( isequal( xm, xj )? 0.: (std::cos(xm)-std::cos(xj))/(xm-xj) );
  cv[0] = std::cos(xm)+r*(x-xm), cv[1] = r;
  return cv;
}

template <typename T> inline double*
McCormick<T>::_coscc
( const double x, const double xL, const double xU )
{
  static double cc[2];
  const double*cvenv = _coscv( x-PI, xL-PI, xU-PI );
  cc[0] = -cvenv[0], cc[1] = -cvenv[1];
  return cc;
}

template <typename T> inline double
McCormick<T>::_cosenv_func
( const double x, const double*rusr, const int*iusr )

{
  // f(z) = (z-a)*sin(z)+cos(z)-cos(a) = 0
  return ((x-*rusr)*std::sin(x)+std::cos(x)-std::cos(*rusr));
}

template <typename T> inline double
McCormick<T>::_cosenv_dfunc
( const double x, const double*rusr, const int*iusr )
{
  // f'(z) = (z-a)*cos(z)
  return ((x-*rusr)*std::cos(x));
}

template <typename T> inline double*
McCormick<T>::_asincv
( const double x, const double xL, const double xU )
{
  static double cv[2];
  if( xL >= 0. ){	 // convex part
    cv[0] = std::asin(x), cv[1] = 1./std::sqrt(1-x*x);
    return cv;
  }
  if( xU <= 0. ){	 // concave part
    double r = ( isequal( xL, xU )? 0.: (std::asin(xU)-std::asin(xL))/(xU-xL));
    cv[0] = std::asin(xL)+r*(x-xL), cv[1] = r;
    return cv;
  } 

  double xj;
  try{
    xj = _secant( 0., xU, 0., xU, _asinenv_func, &xL, 0 );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( 0., xU, _asinenv_func, &xL, 0 );
  }
  if( x >= xj ){	 // convex part
    cv[0] = std::asin(x), cv[1] = 1./std::sqrt(1-x*x);
    return cv;
  }
  double r = ( isequal( xL, xj )? 0.: (std::asin(xj)-std::asin(xL))/(xj-xL));
  cv[0] = std::asin(xL)+r*(x-xL), cv[1] = r;	// linear part
  return cv;
}

template <typename T> inline double*
McCormick<T>::_asincc
( const double x, const double xL, const double xU )
{
  static double cc[2];
  if( xL >= 0. ){	 // convex part
    double r = ( isequal( xL, xU )? 0.: (std::asin(xU)-std::asin(xL))/(xU-xL));
    cc[0] = std::asin(xL)+r*(x-xL), cc[1] = r;
    return cc;
  }
  if( xU <= 0. ){	 // concave part
    cc[0] = std::asin(x), cc[1] = 1./std::sqrt(1-x*x);
    return cc;
  }

  double xj;
  try{
    xj = _secant( 0., xL, xL, 0., _asinenv_func, &xU, 0 );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( xL, 0., _asinenv_func, &xU, 0 );
  }
  if( x <= xj ){	 // concave part
    cc[0] = std::asin(x), cc[1] = 1./std::sqrt(1-x*x);
    return cc;
  }
  double r = ( isequal( xU, xj )? 0.: (std::asin(xj)-std::asin(xU))/(xj-xU));
  cc[0] = std::asin(xU)+r*(x-xU), cc[1] = r;	// secant part
  return cc;
}

template <typename T> inline double
McCormick<T>::_asinenv_func
( const double x, const double*rusr, const int*iusr )
{
  // f(z) = z-a-sqrt(1-z^2)*(asin(z)-asin(a)) = 0
  return x-(*rusr)-std::sqrt(1.-x*x)*(std::asin(x)-std::asin(*rusr));
}

template <typename T> inline double
McCormick<T>::_asinenv_dfunc
( const double x, const double*rusr, const int*iusr )
{
  // f'(z) = z/sqrt(1-z^2)*(asin(z)-asin(a))
  return x/std::sqrt(1.-x*x)*(std::asin(x)-std::asin(*rusr));
}

template <typename T> inline double*
McCormick<T>::_tancv
( const double x, const double xL, const double xU )
{
  static double cv[2];
  if( xL >= 0. ){	 // convex part
    cv[0] = std::tan(x), cv[1] = 1.+sqr(std::tan(x));
    return cv;
  }
  if( xU <= 0. ){	 // concave part
    double r = ( isequal( xL, xU )? 0.: (std::tan(xU)-std::tan(xL))/(xU-xL));
    cv[0] = std::tan(xL)+r*(x-xL), cv[1] = r;
    return cv;
  } 

  double xj;
  try{
    xj = _secant( 0., xU, 0., xU, _tanenv_func, &xL, 0 );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( 0., xU, _tanenv_func, &xL, 0 );
  }
  if( x >= xj ){	 // convex part
    cv[0] = std::tan(x), cv[1] = 1.+sqr(std::tan(x));
    return cv;
  }
  double r = ( isequal( xL, xj )? 0.: (std::tan(xj)-std::tan(xL))/(xj-xL));
  cv[0] = std::tan(xL)+r*(x-xL), cv[1] = r;	// secant part
  return cv;
}

template <typename T> inline double*
McCormick<T>::_tancc
( const double x, const double xL, const double xU )
{
  static double cc[2];
  if( xL >= 0. ){	 // convex part
    double r = ( isequal( xL, xU )? 0.: (std::tan(xU)-std::tan(xL))/(xU-xL));
    cc[0] = std::tan(xL)+r*(x-xL), cc[1] = r;
    return cc;
  }
  if( xU <= 0. ){	 // concave part
    cc[0] = std::tan(x), cc[1] = 1.+sqr(std::tan(x));
    return cc;
  }

  double xj;
  try{
    xj = _secant( 0., xL, xL, 0., _tanenv_func, &xU, 0 );
  }
  catch( McCormick<T>::Exceptions ){
    xj = _goldsect( xL, 0., _tanenv_func, &xU, 0 );
  }
  if( x <= xj ){	 // concave part
    cc[0] = std::tan(x), cc[1] = 1.+sqr(std::tan(x));
    return cc;
  }
  double r = ( isequal( xU, xj )? 0.: (std::tan(xj)-std::tan(xU))/(xj-xU));
  cc[0] = std::tan(xU)+r*(x-xU), cc[1] = r;	// secant part
  return cc;
}

template <typename T> inline double
McCormick<T>::_tanenv_func
( const double x, const double*rusr, const int*iusr )
{
  // f(z) = (z-a)-(tan(z)-tan(a))/(1+tan(z)^2) = 0
  return (x-(*rusr))-(std::tan(x)-std::tan(*rusr))/(1.+sqr(std::tan(x)));
}

template <typename T> inline double
McCormick<T>::_tanenv_dfunc
( const double x, const double*rusr, const int*iusr )
{
  // f'(z) = (tan(z)-tan(a))/(1+tan(z)^2)*2*tan(z)
  return 2.*std::tan(x)/(1.+sqr(std::tan(x)))*(std::tan(x)-std::tan(*rusr));
}

template <typename T> inline double
McCormick<T>::_newton
( const double x0, const double xL, const double xU, puniv f,
  puniv df, const double*rusr, const int*iusr )
{
  double xk = std::max(xL,std::min(xU,x0));
  double fk = f(xk,rusr,iusr);
  
  for( unsigned int it=0; it<options.ENVEL_MAXIT; it++ ){
    if( std::fabs(fk) < options.ENVEL_TOL ) return xk;
    double dfk = df(xk,rusr,iusr);
    if( dfk == 0 ) throw Exceptions( Exceptions::ENVEL );
    if( isequal(xk,xL) && fk/dfk>0 ) return xk;
    if( isequal(xk,xU) && fk/dfk<0 ) return xk;
    xk = std::max(xL,std::min(xU,xk-fk/dfk));
    fk = f(xk,rusr,iusr);
  }

  throw Exceptions( Exceptions::ENVEL );
}

template <typename T> inline double
McCormick<T>::_secant
( const double x0, const double x1, const double xL, const double xU,
  puniv f, const double*rusr, const int*iusr )
{
  double xkm = std::max(xL,std::min(xU,x0));
  double fkm = f(xkm,rusr,iusr);
  double xk = std::max(xL,std::min(xU,x1));
  
  for( unsigned int it=0; it<options.ENVEL_MAXIT; it++ ){
    double fk = f(xk,rusr,iusr);
    if( std::fabs(fk) < options.ENVEL_TOL ) return xk;
    double Bk = (fk-fkm)/(xk-xkm);
    if( Bk == 0 ) throw Exceptions( Exceptions::ENVEL );
    if( isequal(xk,xL) && fk/Bk>0 ) return xk;
    if( isequal(xk,xU) && fk/Bk<0 ) return xk;
    xkm = xk;
    fkm = fk;
    xk = std::max(xL,std::min(xU,xk-fk/Bk));
  }

  throw Exceptions( Exceptions::ENVEL );
}

template <typename T> inline double
McCormick<T>::_goldsect
( const double xL, const double xU, puniv f, const double*rusr,
  const int*iusr )
{
  const double phi = 2.-(1.+std::sqrt(5.))/2.;
  const double fL = f(xL,rusr,iusr), fU = f(xU,rusr,iusr);
  if( fL*fU > 0 ) throw Exceptions( Exceptions::ENVEL );
  const double xm = xU-phi*(xU-xL), fm = f(xm,rusr,iusr);
  return _goldsect_iter( true, xL, fL, xm, fm, xU, fU, f, rusr, iusr );
}

template <typename T> inline double
McCormick<T>::_goldsect_iter
( const bool init, const double a, const double fa, const double b,
  const double fb, const double c, const double fc, puniv f,
  const double*rusr, const int*iusr )
// a and c are the current bounds; the minimum is between them.
// b is a center point
{
  static unsigned int iter;
  iter = ( init? 1: iter+1 );
  const double phi = 2.-(1.+std::sqrt(5.))/2.;
  bool b_then_x = ( c-b > b-a );
  double x = ( b_then_x? b+phi*(c-b): b-phi*(b-a) );
  if( std::fabs(c-a) < options.ENVEL_TOL*(std::fabs(b)+std::fabs(x)) 
   || iter > options.ENVEL_MAXIT ) return (c+a)/2.;
  double fx = f(x,rusr,iusr);
  if( b_then_x )
    return( fa*fx<0? _goldsect_iter( false, a, fa, b, fb, x, fx, f, rusr, iusr ):
                      _goldsect_iter( false, b, fb, x, fx, c, fc, f, rusr, iusr ) );
  return( fa*fb<0? _goldsect_iter( false, a, fa, x, fx, b, fb, f, rusr, iusr ):
                    _goldsect_iter( false, x, fx, b, fb, c, fc, f, rusr, iusr ) );
}

////////////////////////////////////////////////////////////////////////

template <typename T> inline McCormick<T>
cut
( const McCormick<T>&MC )
{
  McCormick<T> MC2( MC );
  return MC2.cut();
}

template <typename T> inline McCormick<T>
operator+
( const McCormick<T>&MC )
{
  McCormick<T> MC2( MC );
  return MC2;
}

template <typename T> inline McCormick<T>
operator+
( const double a, const McCormick<T>&MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = a + MC._I;
  MC2._cv = a + MC._cv;
  MC2._cc = a + MC._cc;
  for( unsigned int i=0; i<MC2._nsub; i++ ){
    MC2._cvsub[i] = MC._cvsub[i];
    MC2._ccsub[i] = MC._ccsub[i];
  }
  return MC2;
}

template <typename T> inline McCormick<T>
operator+
( const McCormick<T>&MC, const double a )
{
  return a + MC;
}

template <typename T> inline McCormick<T>
operator+
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  if( MC2._const ){
    McCormick<T> MC3;
    MC3._sub( MC1._nsub, MC1._const );
    return MC3._sum1( MC1, MC2 );
  }
  if( MC1._const ){
    McCormick<T> MC3;
    MC3._sub( MC2._nsub, MC2._const );
    return MC3._sum1( MC2, MC1 );
  } 
  if( MC1._nsub != MC2._nsub )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
  McCormick<T> MC3;
  MC3._sub( MC1._nsub, MC1._const||MC2._const );
  return MC3._sum2( MC1, MC2 );
}

template <typename T> inline McCormick<T>
operator-
( const McCormick<T>&MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = -MC._I;
  MC2._cv = -MC._cc;
  MC2._cc = -MC._cv;
  for( unsigned int i=0; i<MC2._nsub; i++ ){
    MC2._cvsub[i] = -MC._ccsub[i];
    MC2._ccsub[i] = -MC._cvsub[i];
  }
  return MC2;
}

template <typename T> inline McCormick<T>
operator-
( const McCormick<T>&MC, const double a )
{
  return MC + (-a);
}

template <typename T> inline McCormick<T>
operator-
( const double a, const McCormick<T>&MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = a - MC._I;
  MC2._cv = a - MC._cc;
  MC2._cc = a - MC._cv;
  for( unsigned int i=0; i<MC2._nsub; i++ ){
    MC2._cvsub[i] = -MC._ccsub[i];
    MC2._ccsub[i] = -MC._cvsub[i];
  }
  return MC2;
}

template <typename T> inline McCormick<T>
operator-
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  if( &MC1 == &MC2 ) return 0;

  if( MC2._const ){
    McCormick<T> MC3;
    MC3._sub( MC1._nsub, MC1._const );
    return MC3._sub1( MC1, MC2 );  
  }
  if( MC1._const ){
    McCormick<T> MC3;
    MC3._sub( MC2._nsub, MC2._const );
    return MC3._sub2( MC1, MC2 );
  }
  if( MC1._nsub != MC2._nsub )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
  McCormick<T> MC3;
  MC3._sub( MC1._nsub, MC1._const||MC2._const );
  return MC3._sub3( MC1, MC2 );
}

template <typename T> inline McCormick<T>
operator*
( const double a, const McCormick<T>&MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = a * MC._I;
  if ( a >= 0 ){
    MC2._cv = a * MC._cv;
    MC2._cc = a * MC._cc;
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = a * MC._cvsub[i];
      MC2._ccsub[i] = a * MC._ccsub[i];
    }
  }
  else{
    MC2._cv = a * MC._cc;
    MC2._cc = a * MC._cv;
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = a * MC._ccsub[i];
      MC2._ccsub[i] = a * MC._cvsub[i];
    }
  }
  return MC2;
}

template <typename T> inline McCormick<T>
operator*
( const McCormick<T>&MC, const double a )
{  
  return a * MC;
}

template <typename T> inline McCormick<T>
operator*
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  if( &MC1 == &MC2 ) return sqr(MC1);

  bool thin1 = isequal( Op<T>::diam(MC1._I), 0. );
  bool thin2 = isequal( Op<T>::diam(MC2._I), 0. );

  if ( McCormick<T>::options.MVCOMP_USE && !(thin1||thin2) ){
    McCormick<T> MC3;
    if( MC2._const )
      MC3._sub( MC1._nsub, MC1._const );
    else if( MC1._const )
      MC3._sub( MC2._nsub, MC2._const );
    else if( MC1._nsub != MC2._nsub )
      throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
    else
      MC3._sub( MC1._nsub, MC1._const||MC2._const );

    MC3._I = MC1._I * MC2._I;
    return MC3._mulMV( MC1, MC2 ).cut();
  }

  if ( Op<T>::l(MC1._I) >= 0. ){
    if ( Op<T>::l(MC2._I) >= 0. ){
      if( MC2._const ){
        McCormick<T> MC3;
        MC3._sub( MC1._nsub, MC1._const );
        return MC3._mul1_u1pos_u2pos( MC1, MC2 ).cut();
      }
      if( MC1._const ){
        McCormick<T> MC3;
        MC3._sub( MC2._nsub, MC2._const );
        return MC3._mul1_u1pos_u2pos( MC2, MC1 ).cut();
      }
      if( MC1._nsub != MC2._nsub )
        throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
      McCormick<T> MC3;
      MC3._sub( MC1._nsub, MC1._const||MC2._const );
      return MC3._mul2_u1pos_u2pos( MC1, MC2 ).cut();
    }
    if ( Op<T>::u(MC2._I) <= 0. ){
      return -( MC1 * (-MC2) );
    }
    if( MC2._const ){
      McCormick<T> MC3;
      MC3._sub( MC1._nsub, MC1._const );
      return MC3._mul1_u1pos_u2mix( MC1, MC2 ).cut();
    }
    if( MC1._const ){
      McCormick<T> MC3;
      MC3._sub( MC2._nsub, MC2._const );
      return MC3._mul2_u1pos_u2mix( MC1, MC2 ).cut();
    }
    if( MC1._nsub != MC2._nsub )
      throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
    McCormick<T> MC3;
    MC3._sub( MC1._nsub, MC1._const||MC2._const );
    return MC3._mul3_u1pos_u2mix( MC1, MC2 ).cut();
  }

  if ( Op<T>::u(MC1._I) <= 0. ){
    if ( Op<T>::l(MC2._I) >= 0. ){
      return -( (-MC1) * MC2);
    }
    if ( Op<T>::u(MC2._I) <= 0. ){
      return (-MC1) * (-MC2);
    }
    return -( MC2 * (-MC1) );
  }

  if ( Op<T>::l(MC2._I) >= 0. ){
    return MC2 * MC1;
  }
  if ( Op<T>::u(MC2._I) <= 0. ){
    return -( (-MC2) * MC1 );
  }
  if( MC2._const ){
    McCormick<T> MC3;
    MC3._sub( MC1._nsub, MC1._const );
    return MC3._mul1_u1mix_u2mix( MC1, MC2 ).cut();
  }
  if( MC1._const ){
    McCormick<T> MC3;
    MC3._sub( MC2._nsub, MC2._const );
    return MC3._mul1_u1mix_u2mix( MC2, MC1 ).cut();
  }
  if( MC1._nsub != MC2._nsub )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
  McCormick<T> MC3;
  MC3._sub( MC1._nsub, MC1._const||MC2._const );
  return MC3._mul2_u1mix_u2mix( MC1, MC2 ).cut();
}

template <typename T> inline McCormick<T>
operator/
( const McCormick<T>&MC, const double a )
{
  if ( isequal( a, 0. ))
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::DIV );
  return (1./a) * MC;
}

template <typename T> inline McCormick<T>
operator/
( const double a, const McCormick<T>&MC )
{
  return a * inv( MC );
}
template <typename T> inline McCormick<T>
operator/
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  if( &MC1 == &MC2 ) return 1.;
  
  bool posorthant = ( Op<T>::l(MC1._I) >= 0. && Op<T>::l(MC2._I) > 0. );
  if ( McCormick<T>::options.MVCOMP_USE && posorthant){
    McCormick<T> MC3;
    if( MC2._const )
      MC3._sub( MC1._nsub, MC1._const );
    else if( MC1._const )
	  MC3._sub( MC2._nsub, MC2._const );
    else if( MC1._nsub != MC2._nsub )
      throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
    else
      MC3._sub( MC1._nsub, MC1._const||MC2._const );

    MC3._I = MC1._I / MC2._I;

    int imidcv1 = -1, imidcv2 = -1;
    double fmidcv1 = ( mid(MC1._cv, MC1._cc, Op<T>::l(MC1._I), imidcv1)
      + std::sqrt(Op<T>::l(MC1._I) * Op<T>::u(MC1._I)) )
      / ( std::sqrt(Op<T>::l(MC1._I)) + std::sqrt(Op<T>::u(MC1._I)) );
    double fmidcv2 = mid(MC2._cv, MC2._cc, Op<T>::u(MC2._I), imidcv2);
    MC3._cv = sqr(fmidcv1) / fmidcv2;
    for( unsigned int i=0; i<MC3._nsub; i++ )
      MC3._cvsub[i] = 2. * fmidcv1 / fmidcv2
        / ( std::sqrt(Op<T>::l(MC1._I)) + std::sqrt(Op<T>::u(MC1._I)) )
        * (MC1._const? 0.: mid( MC1._cvsub, MC1._ccsub, i, imidcv1 ))
        - sqr( fmidcv1 / fmidcv2 )
        * (MC2._const? 0.: mid( MC2._cvsub, MC2._ccsub, i, imidcv2 ));

    int imidcc1 = -1, imidcc2 = -1;
    double fmidcc1 = mid(MC1._cv, MC1._cc, Op<T>::u(MC1._I), imidcc1);
    double fmidcc2 = mid(MC2._cv, MC2._cc, Op<T>::l(MC2._I), imidcc2);
    double gcc1 = Op<T>::u(MC2._I) * fmidcc1 - Op<T>::l(MC1._I) * fmidcc2
                 + Op<T>::l(MC1._I) * Op<T>::l(MC2._I);
    double gcc2 = Op<T>::l(MC2._I) * fmidcc1 - Op<T>::u(MC1._I) * fmidcc2
                 + Op<T>::u(MC1._I) * Op<T>::u(MC2._I);
    if( gcc1 <= gcc2 ){
      MC3._cc = gcc1 / ( Op<T>::l(MC2._I) * Op<T>::u(MC2._I) );
      for( unsigned int i=0; i<MC3._nsub; i++ )
        MC3._ccsub[i] = 1. / Op<T>::l(MC2._I)
          * (MC1._const? 0.: mid( MC1._cvsub, MC1._ccsub, i, imidcc1 ))
          - Op<T>::l(MC1._I) / ( Op<T>::l(MC2._I) * Op<T>::u(MC2._I) )
          * (MC2._const? 0.: mid( MC2._cvsub, MC2._ccsub, i, imidcc2 ));
    }
    else{
      MC3._cc = gcc2 / ( Op<T>::l(MC2._I) * Op<T>::u(MC2._I) );
      for( unsigned int i=0; i<MC3._nsub; i++ )
        MC3._ccsub[i] = 1. / Op<T>::u(MC2._I)
          * (MC1._const? 0.: mid( MC1._cvsub, MC1._ccsub, i, imidcc1 ))
          - Op<T>::u(MC1._I) / ( Op<T>::l(MC2._I) * Op<T>::u(MC2._I) )
          * (MC2._const? 0.: mid( MC2._cvsub, MC2._ccsub, i, imidcc2 ));
    }
    return MC3.cut();
  }

  return MC1 * inv( MC2 );
}

template <typename T> inline McCormick<T>
inv
( const McCormick<T>&MC )
{
  if ( Op<T>::l(MC._I) <= 0. && Op<T>::u(MC._I) >= 0. )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::INV );
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::inv( MC._I );

  if ( Op<T>::l(MC._I) > 0. ){
    { int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid );
      MC2._cv = 1./vmid;
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = - mid( MC._cvsub, MC._ccsub, i, imid )
          / ( vmid * vmid );
    }
    { int imid = -1;
      MC2._cc = 1. / Op<T>::l(MC._I) + 1. / Op<T>::u(MC._I) - mid( MC._cv, MC._cc,
        Op<T>::l(MC._I), imid ) / ( Op<T>::l(MC._I) * Op<T>::u(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = - mid( MC._cvsub, MC._ccsub, i, imid )
          / ( Op<T>::l(MC._I) * Op<T>::u(MC._I) );
    }
  }

  else{
    { int imid = -1;
      MC2._cv = 1. / Op<T>::l(MC._I) + 1. / Op<T>::u(MC._I) - mid( MC._cv, MC._cc,
        Op<T>::u(MC._I), imid ) / ( Op<T>::l(MC._I) * Op<T>::u(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = - mid( MC._cvsub, MC._ccsub, i, imid )
          / ( Op<T>::l(MC._I) * Op<T>::u(MC._I) );
    }
    { int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid);
      MC2._cc = 1. / vmid;
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = - mid( MC._cvsub, MC._ccsub, i, imid )
          / ( vmid * vmid );
    }
  }
  
  return MC2.cut();
}

template <typename T> inline McCormick<T>
sqr
( const McCormick<T>&MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::sqr( MC._I );
  { int imid = -1;
    double zmin = mid( Op<T>::l(MC._I), Op<T>::u(MC._I), 0., imid );
    imid = -1;
    MC2._cv = mc::sqr( mid( MC._cv, MC._cc, zmin, imid ) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._cvsub[i] = 2 * mid( MC._cvsub, MC._ccsub, i, imid )
        * mid( MC._cv, MC._cc, zmin, imid );
  }

  { int imid = -1;
    double zmax = (mc::sqr( Op<T>::l(MC._I) )>mc::sqr( Op<T>::u(MC._I) )?
      Op<T>::l(MC._I): Op<T>::u(MC._I));
    double r = ( isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) )? 0.:
      ( mc::sqr( Op<T>::u(MC._I) ) - mc::sqr( Op<T>::l(MC._I) ) )
      / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) ) );
    MC2._cc = mc::sqr( Op<T>::l(MC._I) ) + r * ( mid( MC._cv, MC._cc, zmax,
      imid ) - Op<T>::l(MC._I) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
  }

  return MC2.cut();
}

template <typename T> inline McCormick<T>
exp
( const McCormick<T>&MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::exp( MC._I );

  { int imid = -1;
    MC2._cv = std::exp( mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid ));
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * MC2._cv;
  }

  { int imid = -1;
    double r = 0.;
    if( !isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) ))
      r = ( std::exp( Op<T>::u(MC._I) ) - std::exp( Op<T>::l(MC._I) ) )
        / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) );
    MC2._cc = std::exp( Op<T>::u(MC._I) ) + r * ( mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid )
      - Op<T>::u(MC._I) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
  }

  return MC2.cut();
}

template <typename T> inline McCormick<T>
arh
( const McCormick<T>&MC, const double k )
{
  if( Op<T>::l(MC._I) <= 0. || k < 0. || ( Op<T>::u(MC._I) > 0.5*k && Op<T>::l(MC._I) >= 0.5*k ) ){
    return exp( - k * inv( MC ) );
  }

  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::arh( MC._I, k );

  if ( Op<T>::u(MC._I) <= 0.5*k ){
    { int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid );
      MC2._cv = std::exp( - k / vmid );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = k / ( vmid * vmid ) * MC2._cv
          * mid( MC._cvsub, MC._ccsub, i, imid );
    }
    { int imid = -1;
      double r = 0.;
      if( !isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) ))
        r = ( mc::arh( Op<T>::u(MC._I) ) - mc::arh( Op<T>::l(MC._I) ) )
          / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) );
      MC2._cc = mc::arh( Op<T>::l(MC._I) ) + r * ( mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid )
        - Op<T>::l(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
    }
    return MC2.cut();
  }

  else if ( Op<T>::l(MC._I) >= 0.5*k ){
    { int imid = -1;
      double r = 0.;
      if( !isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) ))
        r = ( mc::arh( Op<T>::u(MC._I) ) - mc::arh( Op<T>::l(MC._I) ) )
          / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) );
      MC2._cv = mc::arh( Op<T>::l(MC._I) ) + r * ( mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid )
        - Op<T>::l(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
    }
    { int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid );
      MC2._cc = std::exp( - k / vmid );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = k / ( vmid * vmid ) * MC2._cc
          * mid( MC._cvsub, MC._ccsub, i, imid );
    }
    return MC2.cut();
  }
}

template <typename T> inline McCormick<T>
log
( const McCormick<T>&MC )
{
  if ( Op<T>::l(MC._I) <= 0. )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::LOG );
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::log( MC._I );

  { int imid = -1;
    double scal = 0.;
    if( !isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) ))
      scal = ( std::log( Op<T>::u(MC._I) ) - std::log( Op<T>::l(MC._I) ) )
        / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) );
    MC2._cv = std::log( Op<T>::l(MC._I) ) + scal * ( mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid )
      - Op<T>::l(MC._I) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * scal;
  }

  { int imid = -1;
    double vmid = mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid );
    MC2._cc = std::log( vmid );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) / vmid;
  }

  return MC2.cut();
}

template <typename T> inline McCormick<T>
xlog
( const McCormick<T>&MC )
{
  if ( Op<T>::l(MC._I) <= 0. )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::LOG );
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::xlog( MC._I );

  { int imid = -1;
    double zmin = mid( Op<T>::l(MC._I), Op<T>::u(MC._I), std::exp(-1.), imid );
    imid = -1;
    double vmid = mid( MC._cv, MC._cc, zmin, imid );
    MC2._cv = xlog( vmid );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._cvsub[i] = (std::log( vmid ) + 1.) * mid( MC._cvsub, MC._ccsub,
        i, imid );
  }

  { int imid = -1;
    double zmax = ( xlog(Op<T>::u(MC._I))>=xlog(Op<T>::l(MC._I))? Op<T>::u(MC._I): Op<T>::l(MC._I) );
    double r = 0.;
    if( !isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) ))
      r = ( xlog(Op<T>::u(MC._I)) - xlog(Op<T>::l(MC._I)) )
        / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) );
    imid = -1;
    MC2._cc = xlog(Op<T>::l(MC._I)) + r * ( mid( MC._cv, MC._cc, zmax, imid ) - Op<T>::l(MC._I) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
  }

  return MC2.cut();
}

template <typename T> inline McCormick<T>
sqrt
( const McCormick<T>&MC )
{
  if ( Op<T>::l(MC._I) < 0. )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SQRT );
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::sqrt( MC._I );

  { double r = 0.;
    if( !isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) ))
      r = ( std::sqrt( Op<T>::u(MC._I) ) - std::sqrt( Op<T>::l(MC._I) ) )
        / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) );
    int imid = -1;
    double vmid = mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid );
    MC2._cv = std::sqrt( Op<T>::l(MC._I) ) + r * ( vmid - Op<T>::l(MC._I) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
  }

  { int imid = -1;
    double vmid = mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid );
    MC2._cc = std::sqrt( vmid );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) / (2.*MC2._cc);
  }

  return MC2.cut();
}

template <typename T> inline McCormick<T>
erfc
( const McCormick<T> &MC )
{
  return ( 1. - erf( MC ) );
}

template <typename T> inline McCormick<T>
erf
( const McCormick<T>&MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::erf( MC._I );

  if( !McCormick<T>::options.ENVEL_USE ){
     MC2._cv = Op<T>::l(MC2._I);
     MC2._cc = Op<T>::u(MC2._I);
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = MC2._ccsub[i] = 0.;
    }
    return MC2;
  }

  { int imid = -1;
    const double* cvenv = McCormick<T>::_erfcv( mid( MC._cv,
      MC._cc, Op<T>::l(MC._I), imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
    MC2._cv = cvenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
    }
  }
  { int imid = -1;
    const double* ccenv = McCormick<T>::_erfcc( mid( MC._cv,
      MC._cc, Op<T>::u(MC._I), imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
    MC2._cc = ccenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * ccenv[1];
    }
  }
  return MC2.cut();
}

template <typename T> inline McCormick<T>
pow
( const McCormick<T>&MC, const int n )
{
  if( n == 0 ){
    return 1.;
  }

  if( n == 1 ){
    return MC;
  }

  if( n >= 2 && !(n%2) ){ 
    McCormick<T> MC2;
    MC2._sub( MC._nsub, MC._const );
    MC2._I = Op<T>::pow( MC._I, n );
    { int imid = -1;
      double zmin = mid( Op<T>::l(MC._I), Op<T>::u(MC._I), 0., imid );
      imid = -1;
      MC2._cv = std::pow( mid( MC._cv, MC._cc, zmin, imid ), n );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = n * mid( MC._cvsub, MC._ccsub, i, imid )
          * std::pow( mid( MC._cv, MC._cc, zmin, imid ), n-1 );
    }
    { int imid = -1;
      double zmax = (std::pow( Op<T>::l(MC._I), n )>std::pow( Op<T>::u(MC._I), n )?
        Op<T>::l(MC._I): Op<T>::u(MC._I));
      double r = ( isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) )? 0.: ( std::pow( Op<T>::u(MC._I),
        n ) - std::pow( Op<T>::l(MC._I), n ) ) / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) ) );
      MC2._cc = std::pow( Op<T>::l(MC._I), n ) + r * ( mid( MC._cv, MC._cc, zmax,
        imid ) - Op<T>::l(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
    }
    return MC2.cut();
  }

  if( n >= 3 && McCormick<T>::options.ENVEL_USE ){
    McCormick<T> MC2;
    MC2._sub( MC._nsub, MC._const );
    MC2._I = Op<T>::pow( MC._I, n );
    { int imid = -1;
      const double* cvenv = McCormick<T>::_oddpowcv( mid( MC._cv,
        MC._cc, Op<T>::l(MC._I), imid ), n, Op<T>::l(MC._I), Op<T>::u(MC._I) );
      MC2._cv = cvenv[0];
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
      }
    }
    { int imid = -1;
      const double* ccenv = McCormick<T>::_oddpowcc( mid( MC._cv,
        MC._cc, Op<T>::u(MC._I), imid ), n, Op<T>::l(MC._I), Op<T>::u(MC._I) );
      MC2._cc = ccenv[0];
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * ccenv[1];
      }
    }
    return MC2.cut();
  }

  if( n >= 3 ){
    return pow( MC, n-1 ) * MC;
  }

  if( n == -1 ){
    return inv( MC );
  }

  if ( Op<T>::l(MC._I) <= 0. && Op<T>::u(MC._I) >= 0. )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::INV );
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::pow( MC._I, n );

  if ( Op<T>::l(MC._I) > 0. ){
    { int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid );
      MC2._cv = std::pow( vmid, n );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = n* mid( MC._cvsub, MC._ccsub, i, imid ) * std::pow( vmid, n-1 );
    }
    { double r = std::pow( Op<T>::l(MC._I), -n-1 ) + std::pow( Op<T>::u(MC._I), -n-1 );
      for( int i=1; i<=-n-2; i++ )
         r += std::pow( Op<T>::l(MC._I), i ) * std::pow( Op<T>::u(MC._I), -n-1-i );
      r /= - std::pow( Op<T>::l(MC._I), -n ) * std::pow( Op<T>::u(MC._I), -n );
      int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid );
      MC2._cc = std::pow( Op<T>::l(MC._I), n ) + r * ( vmid - Op<T>::l(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = r * mid( MC._cvsub, MC._ccsub, i, imid );
    }
    return MC2.cut();
  }

  if( (-n)%2 ){
    { double r = std::pow( Op<T>::l(MC._I), -n-1 ) + std::pow( Op<T>::u(MC._I), -n-1 );
      for( int i=1; i<=-n-2; i++ )
         r += std::pow( Op<T>::l(MC._I), i ) * std::pow( Op<T>::u(MC._I), -n-1-i );
      r /= - std::pow( Op<T>::l(MC._I), -n ) * std::pow( Op<T>::u(MC._I), -n );
      int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid );
      MC2._cv = std::pow( Op<T>::l(MC._I), n ) + r * ( vmid - Op<T>::l(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = r * mid( MC._cvsub, MC._ccsub, i, imid );
    }
    { int imid = -1;
      double vmid = mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid );
      MC2._cc = std::pow( vmid, n );
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = n* mid( MC._cvsub, MC._ccsub, i, imid ) * std::pow( vmid, n-1 );
    }
    return MC2.cut();
  }

  { int imid = -1;
    double vmid = mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid );
    MC2._cv = std::pow( vmid, n );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._cvsub[i] = n* mid( MC._cvsub, MC._ccsub, i, imid ) * std::pow( vmid, n-1 );
  }
    { double r = std::pow( Op<T>::l(MC._I), -n-1 ) + std::pow( Op<T>::u(MC._I), -n-1 );
      for( int i=1; i<=-n-2; i++ )
         r += std::pow( Op<T>::l(MC._I), i ) * std::pow( Op<T>::u(MC._I), -n-1-i );
      r /= - std::pow( Op<T>::l(MC._I), -n ) * std::pow( Op<T>::u(MC._I), -n );
    int imid = -1;
    double vmid = mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid );
    MC2._cc = std::pow( Op<T>::l(MC._I), n ) + r * ( vmid - Op<T>::l(MC._I) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = r * mid( MC._cvsub, MC._ccsub, i, imid );
  }
  return MC2.cut();
}

template <typename T> inline McCormick<T>
pow
( const McCormick<T> &MC, const double a )
{
  return exp( a * log( MC ) );
}

template <typename T> inline McCormick<T>
pow
( const McCormick<T> &MC1, const McCormick<T> &MC2 )
{
  return exp( MC2 * log( MC1 ) );
}

template <typename T> inline McCormick<T>
pow
( const double a, const McCormick<T> &MC )
{
  return exp( MC * std::log( a ) );
}

template <typename T> inline McCormick<T>
monomial
( const unsigned int n, const McCormick<T>*MC, const int*k )
{
  if( n == 0 ){
    return 1.;
  }
  if( n == 1 ){
    return pow( MC[0], k[0] );
  }
  return pow( MC[0], k[0] ) * monomial( n-1, MC+1, k+1 );
}

template <typename T> inline McCormick<T>
cheb
( const McCormick<T> &MC, const unsigned n )
{
  if ( !isequal(Op<T>::l(MC._I),-1.) || !isequal(Op<T>::u(MC._I),1.) )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::CHEB );

  switch( n ){
    case 0:  return 1.;
    case 1:  return MC;
    case 2:  return 2*sqr(MC)-1;
    default: break;
  }

  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::cheb( MC._I, n );
  if( !(n%2) ){ 
    { int imid = -1;
      const double* cvenv = McCormick<T>::_evenchebcv( mid( MC._cv,
        MC._cc, Op<T>::l(MC._I), imid ), n, Op<T>::l(MC._I), Op<T>::u(MC._I) );
      MC2._cv = cvenv[0];
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
      }
    }
    { MC2._cc = 1.;
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._ccsub[i] = 0.;
    }
  }
  else{
    { int imid = -1;
      const double* cvenv = McCormick<T>::_oddchebcv( mid( MC._cv,
        MC._cc, Op<T>::l(MC._I), imid ), n, Op<T>::l(MC._I), Op<T>::u(MC._I) );
      MC2._cv = cvenv[0];
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
      }
    }
    { int imid = -1;
      const double* ccenv = McCormick<T>::_oddchebcc( mid( MC._cv,
        MC._cc, Op<T>::u(MC._I), imid ), n, Op<T>::l(MC._I), Op<T>::u(MC._I) );
      MC2._cc = ccenv[0];
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * ccenv[1];
      }
    }
  }
  //McCormick<T> MCcheb = 2.*MC*cheb(MC,n-1)-cheb(MC,n-2);
  //return( inter( MCcheb, MCcheb, McCormick<T>(T(-1.,1.)) )? MCcheb: McCormick<T>(T(-1.,1.)) 
  return MC2.cut();
}

template <typename T> inline McCormick<T>
fabs
( const McCormick<T> &MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::fabs( MC._I );

  { int imid = -1;
    double zmin = mid( Op<T>::l(MC._I), Op<T>::u(MC._I), 0., imid );
    imid = -1;
    double vmid = mid( MC._cv, MC._cc, zmin, imid );
    MC2._cv = std::fabs( vmid );
    if( vmid >= 0. )
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid );
    else
      for( unsigned int i=0; i<MC2._nsub; i++ )
        MC2._cvsub[i] = - mid( MC._cvsub, MC._ccsub, i, imid );
  }

  { int imid = -1;
    double zmax = (std::fabs( Op<T>::l(MC._I) )>std::fabs( Op<T>::u(MC._I) )? Op<T>::l(MC._I):
      Op<T>::u(MC._I));
    double r = ( isequal( Op<T>::l(MC._I), Op<T>::u(MC._I) )? 0.: ( std::fabs( Op<T>::u(MC._I) )
      - std::fabs( Op<T>::l(MC._I) ) ) / ( Op<T>::u(MC._I) - Op<T>::l(MC._I) ) );
    MC2._cc = std::fabs( Op<T>::l(MC._I) ) + r * ( mid( MC._cv, MC._cc, zmax, imid )
      - Op<T>::l(MC._I) );
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * r;
  }

  return MC2.cut();
}

template <typename T> inline McCormick<T>
min
( const McCormick<T> &MC1, const McCormick<T> &MC2 )
{
  McCormick<T> MC3;
  if( MC2._const )
    MC3._sub( MC1._nsub, MC1._const );
  else if( MC1._const )
    MC3._sub( MC2._nsub, MC2._const );
  else if( MC1._nsub != MC2._nsub )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
  else
    MC3._sub( MC1._nsub, MC1._const||MC2._const );
  MC3._I = Op<T>::min( MC1._I, MC2._I );

  if( Op<T>::u(MC1._I) <= Op<T>::l(MC2._I) ){
    MC3._cv = MC1._cv;
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._cvsub[i] = (MC1._const? 0.: MC1._cvsub[i]);
  }
  else if( Op<T>::u(MC2._I) <= Op<T>::l(MC1._I) ){
    MC3._cv = MC2._cv;
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._cvsub[i] = (MC2._const? 0.: MC2._cvsub[i]);
  }
  else if( McCormick<T>::options.MVCOMP_USE ){
     double minL1L2 = std::min( Op<T>::l(MC1._I), Op<T>::l(MC2._I) );
     double minL1U2 = std::min( Op<T>::l(MC1._I), Op<T>::u(MC2._I) );
     double minU1L2 = std::min( Op<T>::u(MC1._I), Op<T>::l(MC2._I) );
     double minU1U2 = std::min( Op<T>::u(MC1._I), Op<T>::u(MC2._I) );

     bool thin1 = isequal( Op<T>::diam(MC1._I), 0. );
     double r11 = ( thin1?  0.: ( minU1L2 - minL1L2 ) / Op<T>::diam(MC1._I) );
     double r21 = ( thin1?  0.: ( minL1U2 - minU1U2 ) / Op<T>::diam(MC1._I) );

     bool thin2 = isequal( Op<T>::diam(MC2._I), 0. );
     double r12 = ( thin2?  0.: ( minL1U2 - minL1L2 ) / Op<T>::diam(MC2._I) );
     double r22 = ( thin2?  0.: ( minU1L2 - minU1U2 ) / Op<T>::diam(MC2._I) );

     double g1cv = minL1L2 + r11 * ( MC1._cv - Op<T>::l(MC1._I) )
                           + r12 * ( MC2._cv - Op<T>::l(MC2._I) );
     double g2cv = minU1U2 - r21 * ( MC1._cv - Op<T>::u(MC1._I) )
                           - r22 * ( MC2._cv - Op<T>::u(MC2._I) );
     if( g1cv >= g2cv ){
       MC3._cv = g1cv;
      for( unsigned int i=0; i< MC3._nsub; i++ )
        MC3._cvsub[i] = (MC1._const? 0.: r11*MC1._cvsub[i])
                      + (MC2._const? 0.: r12*MC2._cvsub[i]);
     }
     else{
       MC3._cv = g2cv;
      for( unsigned int i=0; i< MC3._nsub; i++ )
        MC3._cvsub[i] = - (MC1._const? 0.: r21*MC1._cvsub[i])
                        - (MC2._const? 0.: r22*MC2._cvsub[i]);
     }
  }
  else{
    McCormick<T> MCMin = 0.5*( MC1 + MC2 - fabs( MC2 - MC1 ) );
    MC3._cv = MCMin._cv;
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._cvsub[i] = MCMin._cvsub[i];
  }

  MC3._cc = std::min( MC1._cc, MC2._cc );
  for( unsigned int i=0; i< MC3._nsub; i++ )
    MC3._ccsub[i] = ( MC1._cc<=MC2._cc? (MC1._const? 0.: MC1._ccsub[i])
                                      : (MC2._const? 0.: MC2._ccsub[i]) );

  return  MC3.cut();
}

template <typename T> inline McCormick<T>
max
( const McCormick<T> &MC1, const McCormick<T> &MC2 )
{
  McCormick<T> MC3;
  if( MC2._const )
    MC3._sub( MC1._nsub, MC1._const );
  else if( MC1._const )
    MC3._sub( MC2._nsub, MC2._const );
  else if( MC1._nsub != MC2._nsub )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
  else
    MC3._sub( MC1._nsub, MC1._const||MC2._const );
  MC3._I = Op<T>::max( MC1._I, MC2._I );

  if( Op<T>::u(MC1._I) <= Op<T>::l(MC2._I) ){ 
    MC3._cc = MC2._cc;										
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._ccsub[i] = (MC2._const? 0.: MC2._ccsub[i]);		
  }
  else if( Op<T>::u(MC2._I) <= Op<T>::l(MC1._I) ){
    MC3._cc = MC1._cc;										
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._ccsub[i] = (MC1._const? 0.: MC1._ccsub[i]);		
  }
  else if ( McCormick<T>::options.MVCOMP_USE ){
     double maxL1L2 = std::max( Op<T>::l(MC1._I), Op<T>::l(MC2._I) );
     double maxL1U2 = std::max( Op<T>::l(MC1._I), Op<T>::u(MC2._I) );
     double maxU1L2 = std::max( Op<T>::u(MC1._I), Op<T>::l(MC2._I) );
     double maxU1U2 = std::max( Op<T>::u(MC1._I), Op<T>::u(MC2._I) );

     bool thin1 = isequal( Op<T>::diam(MC1._I), 0. );
     double r11 = ( thin1?  0.: ( maxU1L2 - maxL1L2 ) / Op<T>::diam(MC1._I) );
     double r21 = ( thin1?  0.: ( maxL1U2 - maxU1U2 ) / Op<T>::diam(MC1._I) );

     bool thin2 = isequal( Op<T>::diam(MC2._I), 0. );
     double r12 = ( thin2?  0.: ( maxL1U2 - maxL1L2 ) / Op<T>::diam(MC2._I) );
     double r22 = ( thin2?  0.: ( maxU1L2 - maxU1U2 ) / Op<T>::diam(MC2._I) );

     double g1cc = maxL1L2 + r11 * ( MC1._cc - Op<T>::l(MC1._I) )
                           + r12 * ( MC2._cc - Op<T>::l(MC2._I) );
     double g2cc = maxU1U2 - r21 * ( MC1._cc - Op<T>::u(MC1._I) )
                           - r22 * ( MC2._cc - Op<T>::u(MC2._I) );
     if( g1cc <= g2cc ){
       MC3._cc = g1cc;
      for( unsigned int i=0; i< MC3._nsub; i++ )
        MC3._ccsub[i] = (MC1._const? 0.: r11*MC1._ccsub[i])
                      + (MC2._const? 0.: r12*MC2._ccsub[i]);
     }
     else{
       MC3._cc = g2cc;
      for( unsigned int i=0; i< MC3._nsub; i++ )
        MC3._ccsub[i] = - (MC1._const? 0.: r21*MC1._ccsub[i])
                        - (MC2._const? 0.: r22*MC2._ccsub[i]);
     }
  }
  else{
    McCormick<T> MCMax = 0.5*( MC1 + MC2 + fabs( MC1 - MC2 ) );
    MC3._cc = MCMax._cc;
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._ccsub[i] = MCMax._ccsub[i];
  }

  MC3._cv = std::max( MC1._cv, MC2._cv );
  for( unsigned int i=0; i< MC3._nsub; i++ )
    MC3._cvsub[i] = ( MC1._cv>=MC2._cv? (MC1._const? 0.: MC1._cvsub[i])
                                      : (MC2._const? 0.: MC2._cvsub[i]) );

  return  MC3.cut();
}

template <typename T> inline McCormick<T>
max
( const McCormick<T> &MC, const double a  ) {
	McCormick<T> MC2 = a;
	return max( MC, MC2 );
}

template <typename T> inline McCormick<T>
max
( const double a, const McCormick<T> &MC ) {
	McCormick<T> MC2 = a;
	return max( MC, MC2 );
}

template <typename T> inline McCormick<T>
min
( const McCormick<T> &MC, const double a  ) {
	McCormick<T> MC2 = a;
	return min( MC, MC2 );
}

template <typename T> inline McCormick<T>
min
( const double a, const McCormick<T> &MC ) {
	McCormick<T> MC2 = a;
	return min( MC, MC2 );
}

template <typename T> inline McCormick<T>
min
( const unsigned int n, const McCormick<T>*MC )
{
  McCormick<T> MC2( n==0 || !MC ? 0.: MC[0] );
  for( unsigned int i=1; i<n; i++ ) MC2 = min( MC2, MC[i] );
  return MC2;
}

template <typename T> inline McCormick<T>
max
( const unsigned int n, const McCormick<T>*MC )
{
  McCormick<T> MC2( n==0 || !MC ? 0.: MC[0] );
  for( unsigned int i=1; i<n; i++ ) MC2 = max( MC2, MC[i] );
  return MC2;
}

template <typename T> inline McCormick<T>
fstep
( const McCormick<T> &MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  if( Op<T>::l( MC._I ) >= 0 )
    MC2._I = 1.;
  else if( Op<T>::u( MC._I ) < 0 )
    MC2._I = 0.;
  else
    MC2._I = Op<T>::zeroone();
  
  { int imid = -1;
    double zmin = Op<T>::l(MC._I);
    double vmid = mid( MC._cv, MC._cc, zmin, imid );
    const double* cvenv = McCormick<T>::_stepcv( vmid, Op<T>::l(MC._I),
      Op<T>::u(MC._I) );
    MC2._cv = cvenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid )*cvenv[1];
  }
  
  { int imid = -1;
    double zmax = Op<T>::u(MC._I);
    double vmid = mid( MC._cv, MC._cc, zmax, imid );
    const double* ccenv = McCormick<T>::_stepcc( vmid, Op<T>::l(MC._I),
      Op<T>::u(MC._I) );
    MC2._cc = ccenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ )
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid )*ccenv[1];
  }

  return  MC2.cut();
}

template <typename T> inline McCormick<T>
bstep
( const McCormick<T> &MC )
{
  return fstep( -MC );  
}

template <typename T> inline McCormick<T>
ltcond
( const T &I0, const McCormick<T> &MC1, const McCormick<T> &MC2 )
{
  if( Op<T>::u( I0 ) < 0. )       return MC1;
  else if( Op<T>::l( I0 ) >= 0. ) return MC2;

  McCormick<T> MC3;
  if( MC2._const )
    MC3._sub( MC1._nsub, MC1._const );
  else if( MC1._const )
    MC3._sub( MC2._nsub, MC2._const );
  else if( MC1._nsub != MC2._nsub )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );
  else
    MC3._sub( MC1._nsub, MC1._const||MC2._const );

  MC3._I = Op<T>::hull( MC1._I, MC2._I );
  McCormick<T> MCMin  = 0.5*( MC1 + MC2 - fabs( MC2 - MC1 ) );
  McCormick<T> MCMax  = 0.5*( MC1 + MC2 + fabs( MC1 - MC2 ) );
  MC3._cv = MCMin._cv;
  MC3._cc = MCMax._cc;
  for( unsigned int i=0; i< MC3._nsub; i++ ){
    MC3._cvsub[i] = MCMin._cvsub[i];
    MC3._ccsub[i] = MCMax._ccsub[i];
  }    
  return  MC3.cut();
}

template <typename T> inline McCormick<T>
ltcond
( const McCormick<T> &MC0, const McCormick<T> &MC1, const McCormick<T> &MC2 )
{
  McCormick<T> MC3 = ltcond( MC0._I, MC1, MC2 );
  McCormick<T> MCStep = fstep(-MC0)*MC1 + fstep(MC0)*MC2;
  if( MCStep._cv > MC3._cv ){
    MC3._cv = MCStep._cv;
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._cvsub[i] = MCStep._cvsub[i];
  }    
  if( MCStep._cc < MC3._cc ){
    MC3._cc = MCStep._cc;
    for( unsigned int i=0; i< MC3._nsub; i++ )
      MC3._ccsub[i] = MCStep._ccsub[i];
  }    
  return  MC3.cut();
}

template <typename T> inline McCormick<T>
gtcond
( const T &I0, const McCormick<T> &MC1, const McCormick<T> &MC2 )
{
  return ltcond( -I0, MC1, MC2 );
}
template <typename T> inline McCormick<T>
gtcond
( const McCormick<T> &MC0, const McCormick<T> &MC1, const McCormick<T> &MC2 )
{
  return ltcond( -MC0, MC1, MC2 );
}

template <typename T> inline McCormick<T>
cos
( const McCormick<T> &MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::cos( MC._I );

  if( !McCormick<T>::options.ENVEL_USE ){
     MC2._cv = Op<T>::l(MC2._I);
     MC2._cc = Op<T>::u(MC2._I);
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = MC2._ccsub[i] = 0.;
    }
    return MC2;
  }
  
  double*argbnd = McCormick<T>::_cosarg( Op<T>::l(MC._I), Op<T>::u(MC._I) );
  { int imid = -1;
    const double* cvenv = McCormick<T>::_coscv( mid( MC._cv,
      MC._cc, argbnd[0], imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
    MC2._cv = cvenv[0];
    for( unsigned int i=0; i< MC2._nsub; i++ ){
       MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
    }
  }
  { int imid = -1;
    const double* ccenv = McCormick<T>::_coscc( mid( MC._cv,
      MC._cc, argbnd[1], imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
     MC2._cc = ccenv[0];
    for( unsigned int i=0; i< MC2._nsub; i++ ){
       MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * ccenv[1];
    }
  }
  return  MC2.cut();
}

template <typename T> inline McCormick<T>
sin
( const McCormick<T> &MC )
{
  return cos( MC - PI/2. );
}

template <typename T> inline McCormick<T>
asin
( const McCormick<T> &MC )
{
  if ( Op<T>::l(MC._I) <= -1. || Op<T>::u(MC._I) >= 1. )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::ASIN );

  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::asin( MC._I );

  if( !McCormick<T>::options.ENVEL_USE ){
    { int imid = -1;
      MC2._cv = Op<T>::l(MC2._I) + ( mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid )
        - Op<T>::l(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid );
      }
    }
    { int imid = -1;
      MC2._cc = Op<T>::u(MC2._I) + ( mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid )
        - Op<T>::u(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid );
      }
    }
    return MC2.cut();
  }

  { int imid = -1;
    const double* cvenv = McCormick<T>::_asincv( mid( MC._cv,
      MC._cc, Op<T>::l(MC._I), imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
    MC2._cv = cvenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
    }
  }
  { int imid = -1;
    const double* ccenv = McCormick<T>::_asincc( mid( MC._cv,
      MC._cc, Op<T>::u(MC._I), imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
    MC2._cc = ccenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * ccenv[1];
    }
  }
  return MC2.cut();
}

template <typename T> inline McCormick<T>
acos
( const McCormick<T> &MC )
{
  return asin( -MC ) + PI/2.;
}

template <typename T> inline McCormick<T>
tan
( const McCormick<T> &MC )
{
  if ( Op<T>::diam(MC._I) >= PI )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::TAN );
  const double shift = PI*std::ceil(-Op<T>::l(MC._I)/PI-1./2.);
  const double xL1 = Op<T>::l(MC._I)+shift, xU1 = Op<T>::u(MC._I)+shift;
  if ( xL1 <= -PI/2. || xU1 >= PI/2. )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::TAN );

  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::tan( MC._I );

  if( !McCormick<T>::options.ENVEL_USE ){
    { int imid = -1;
      MC2._cv = Op<T>::l(MC2._I) + ( mid( MC._cv, MC._cc, Op<T>::l(MC._I), imid )
        - Op<T>::l(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid );
      }
    }
    { int imid = -1;
      MC2._cc = Op<T>::u(MC2._I) + ( mid( MC._cv, MC._cc, Op<T>::u(MC._I), imid )
        - Op<T>::u(MC._I) );
      for( unsigned int i=0; i<MC2._nsub; i++ ){
        MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid );
      }
    }
    return MC2.cut();
  }

  { int imid = -1;
    const double* cvenv = McCormick<T>::_tancv( mid( MC._cv+shift,
      MC._cc+shift, Op<T>::l(MC._I)+shift, imid ), Op<T>::l(MC._I)+shift,
      Op<T>::u(MC._I)+shift );
    MC2._cv = cvenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
    }
  }
  { int imid = -1;
    const double* ccenv = McCormick<T>::_tancc( mid( MC._cv+shift,
      MC._cc+shift, Op<T>::u(MC._I)+shift, imid ), Op<T>::l(MC._I)+shift,
      Op<T>::u(MC._I)+shift );
    MC2._cc = ccenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * ccenv[1];
    }
  }
  return MC2.cut();
}

template <typename T> inline McCormick<T>
atan
( const McCormick<T> &MC )
{
  McCormick<T> MC2;
  MC2._sub( MC._nsub, MC._const );
  MC2._I = Op<T>::atan( MC._I );

  if( !McCormick<T>::options.ENVEL_USE ){
     MC2._cv = Op<T>::l(MC2._I);
     MC2._cc = Op<T>::u(MC2._I);
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = MC2._ccsub[i] = 0.;
    }
    return MC2;
  }

  { int imid = -1;
    const double* cvenv = McCormick<T>::_atancv( mid( MC._cv,
      MC._cc, Op<T>::l(MC._I), imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
    MC2._cv = cvenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._cvsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * cvenv[1];
    }
  }
  { int imid = -1;
    const double* ccenv = McCormick<T>::_atancc( mid( MC._cv,
      MC._cc, Op<T>::u(MC._I), imid ), Op<T>::l(MC._I), Op<T>::u(MC._I) );
    MC2._cc = ccenv[0];
    for( unsigned int i=0; i<MC2._nsub; i++ ){
      MC2._ccsub[i] = mid( MC._cvsub, MC._ccsub, i, imid ) * ccenv[1];
    }
  }
  return MC2.cut();
}

template <typename T> inline std::ostream&
operator<<
( std::ostream&out, const McCormick<T>&MC)
{
  out << std::scientific << std::setprecision(McCormick<T>::options.DISPLAY_DIGITS) << std::right
      << "[ " << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.l() << " : "
              << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.u()
      << " ] [ "  << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.cv() << " : "
                  << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.cc() << " ]";
  if( MC._nsub ){
    out << " [ (";
    for( unsigned int i=0; i<MC._nsub-1; i++ )
      out << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.cvsub(i) << ",";
    out << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.cvsub(MC._nsub-1) << ") : (";
    for( unsigned int i=0; i<MC._nsub-1; i++ )
      out << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.ccsub(i) << ",";
    out << std::setw(McCormick<T>::options.DISPLAY_DIGITS+7) << MC.ccsub(MC._nsub-1) << ") ]";
  }
  return out;
}


template <typename T> inline McCormick<T>
hull
( const McCormick<T>&X, const McCormick<T>&Y )
{
  if( !X._const && !Y._const && (X._nsub != Y._nsub) )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );

  McCormick<T> CV = min(X,Y);
  McCormick<T> CC = max(X,Y);
  McCormick<T> XUY( Op<T>::hull(X.I(),Y.I()), CV.cv(), CC.cc() );
  if( !X._const )
    XUY._sub( X._nsub, X._const );
  else
    XUY._sub( Y._nsub, Y._const );
  for( unsigned int is=0; is<XUY._nsub; is++ ){
    XUY._cvsub[is] = CV.cvsub(is);
    XUY._ccsub[is] = CC.ccsub(is);
  }
  return XUY;
}

template <typename T> inline bool
inter
( McCormick<T>&XIY, const McCormick<T>&X, const McCormick<T>&Y )
{
  if( !X._const && !Y._const && (X._nsub != Y._nsub) )
    throw typename McCormick<T>::Exceptions( McCormick<T>::Exceptions::SUB );

  if( !Op<T>::inter( XIY._I, X._I, Y._I ) ) return false;
  McCormick<T> CV = max(X,Y);
  McCormick<T> CC = min(X,Y);
  if( CV.cv() > CC.cc() ) return false;
  XIY._cv = CV.cv();
  XIY._cc = CC.cc();
  if( !X._const )
    XIY._sub( X._nsub, X._const );
  else
    XIY._sub( Y._nsub, Y._const );
  for( unsigned int is=0; is<XIY._nsub; is++ ){
    XIY._cvsub[is] = CV.cvsub(is);
    XIY._ccsub[is] = CC.ccsub(is);
  }
  return true;
}

template <typename T> inline bool
operator==
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  return( Op<T>::eq(MC1._I,MC2._I) && MC1._cv == MC2._cv && MC1._cc == MC2._cc );
}

template <typename T> inline bool
operator!=
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  return( Op<T>::ne(MC1._I,MC2._I) || MC1._cv != MC2._cv || MC1._cc != MC2._cc );
}

template <typename T> inline bool
operator<=
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  return( Op<T>::le(MC1._I,MC2._I) && MC1._cv >= MC2._cv && MC1._cc <= MC2._cc );
}

template <typename T> inline bool
operator>=
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  return( Op<T>::ge(MC1._I,MC2._I) && MC1._cv <= MC2._cv && MC1._cc >= MC2._cc );
}

template <typename T> inline bool
operator<
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  return( Op<T>::lt(MC1._I,MC2._I) && MC1._cv > MC2._cv && MC1._cc < MC2._cc );
}

template <typename T> inline bool
operator>
( const McCormick<T>&MC1, const McCormick<T>&MC2 )
{
  return( Op<T>::gt(MC1._I,MC2._I) && MC1._cv < MC2._cv && MC1._cc > MC2._cc );
}

template <typename T> typename McCormick<T>::Options McCormick<T>::options;

} // namespace mc


#include "mcop.hpp"

namespace mc
{

//! @brief Specialization of the structure mc::Op to allow usage of the type mc::Interval for DAG evaluation or as a template parameter in other MC++ classes
template<typename T> struct Op< mc::McCormick<T> >
{
  typedef mc::McCormick<T> MC;
  static MC point( const double c ) { return MC(c); }
  static MC zeroone() { return MC( mc::Op<T>::zeroone() ); }
  static void I(MC& x, const MC&y) { x = y; }
  static double l(const MC& x) { return x.l(); }
  static double u(const MC& x) { return x.u(); }
  static double abs (const MC& x) { return mc::Op<T>::abs(x.I());  }
  static double mid (const MC& x) { return mc::Op<T>::mid(x.I());  }
  static double diam(const MC& x) { return mc::Op<T>::diam(x.I()); }
  static MC inv (const MC& x) { return mc::inv(x);  }
  static MC sqr (const MC& x) { return mc::sqr(x);  }
  static MC sqrt(const MC& x) { return mc::sqrt(x); }
  static MC log (const MC& x) { return mc::log(x);  }
  static MC xlog(const MC& x) { return mc::xlog(x); }
  static MC fabs(const MC& x) { return mc::fabs(x); }
  static MC exp (const MC& x) { return mc::exp(x);  }
  static MC sin (const MC& x) { return mc::sin(x);  }
  static MC cos (const MC& x) { return mc::cos(x);  }
  static MC tan (const MC& x) { return mc::tan(x);  }
  static MC asin(const MC& x) { return mc::asin(x); }
  static MC acos(const MC& x) { return mc::acos(x); }
  static MC atan(const MC& x) { return mc::atan(x); }
  static MC erf (const MC& x) { return mc::erf(x);  }
  static MC erfc(const MC& x) { return mc::erfc(x); }
  static MC fstep(const MC& x) { return mc::fstep(x); }
  static MC bstep(const MC& x) { return mc::bstep(x); }
  static MC hull(const MC& x, const MC& y) { return mc::Op<T>::hull(x.I(),y.I()); }
  static MC min (const MC& x, const MC& y) { return mc::min(x,y);  }
  static MC max (const MC& x, const MC& y) { return mc::max(x,y);  }
  static MC arh (const MC& x, const double k) { return mc::arh(x,k); }
  static MC cheb (const MC& x, const unsigned n) { return mc::cheb(x,n); }
  template <typename X, typename Y> static MC pow(const X& x, const Y& y) { return mc::pow(x,y); }
  static MC monomial (const unsigned int n, const MC* x, const int* k) { return mc::monomial(n,x,k); }
  static bool inter(MC& xIy, const MC& x, const MC& y) { return mc::inter(xIy,x,y); }
  static bool eq(const MC& x, const MC& y) { return x==y; }
  static bool ne(const MC& x, const MC& y) { return x!=y; }
  static bool lt(const MC& x, const MC& y) { return x<y;  }
  static bool le(const MC& x, const MC& y) { return x<=y; }
  static bool gt(const MC& x, const MC& y) { return x>y;  }
  static bool ge(const MC& x, const MC& y) { return x>=y; }
};

} // namespace mc

#endif
