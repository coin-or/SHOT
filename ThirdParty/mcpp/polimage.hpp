// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_POLYHEDRAL Polyhedral Arithmetic for Factorable Functions
\author Beno&icirc;t Chachuat

Consider the image set \f$\Gamma:\{{\bf g}({\bf x}) \,\mid\, {\bf x}^{\rm L}\leq{\bf x}\leq{\bf x}^{\rm U}\} \subseteq \mathbb R^m\f$, where \f$g_j:\mathbb{R}^n\to\mathbb R, j=1,\ldots,m\f$, are factorable, potentially nonconvex functions. The classes mc::PolImg and mc::PolVar implement an arithmetic for the construction of a polyedral enclosure \f$\Gamma\f$ of the set \f$\overline{\Gamma}\f$ in the form:
\f{align*}
\overline{\Gamma} := \left\{ {\bf G} {\bf v}\; \middle|\; \begin{array}{l} {\bf A} {\bf v} \;=\; {\bf 0}\\ {\bf B} {\bf v} \;\leq\; {\bf 0}\\ {\bf v}^{\rm L}\leq{\bf v}\leq{\bf v}^{\rm U} \end{array} \right\} \supseteq \Gamma\, ,
\f}
where the variable vector \f$v\f$ contains the original variable vector \f$x\f$

These classes build upon the DAG classes mc::FFGraph and mc::FFVar. Both are templated in the interval type used to bound the nonlinearity of the function, By default, mc::PolImg and mc::PolVar can be used with the non-verified interval type mc::Interval of MC++. For reliability, however, it is recommended to use verified interval arithmetic such as <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> (header file <tt>mcprofil.hpp</tt>) or <A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A> (header file <tt>mcfilib.hpp</tt>). Note that the implementation in mc::PolImg and mc::PolVar is <a>not verified</a> in the sense that rounding errors are not accounted for in the polyhedral cuts.


\section sec_POLIMG_THEOR What is the theory behind polyhedral relaxations?

The basic procedure follows three steps: (i) decompose the factorable functions \f$g_j\f$ into atom operations, both unary and binary operations, by introducing auxiliary variables and extra constraints (lifting); (ii) generate convex enclosures for the nonlinear operations, including products, divisions and outer-compositions with nonlinear univariates such as exp, log, sin, cos, <I>etc</I>; and (iii) outer-approximate the nonlinear parts of the convex enclosures using supporting hyperplanes at finitely many points. Each step is detailed further below.

- <B>Step 1. Decomposition/Lifting</B>
\f{align*}
\Gamma:\{{\bf g}({\bf x}) \,\mid\, {\bf x}^{\rm L}\leq{\bf x}\leq {\bf x}^{\rm U}\}
& \xrightarrow{\displaystyle\text{decomp.}} && 
\overline{\Gamma} := \left\{ {\bf G} {\bf v}\; \middle|\; \begin{array}{l} {\bf A} {\bf v} \;=\; {\bf 0}\\ v_k = v_iv_j,\ \forall (i,j,k)\in\mathcal{B}\\ v_k = \frac{v_i}{v_j},\ \forall (i,j,k)\in\mathcal{F}\\ v_k = \varphi(v_i),\ \forall (i,k)\in\mathcal{U}\\ {\bf v}^{\rm L}\leq{\bf v}\leq {\bf v}^{\rm U} \end{array} \right\}
\f}
The advantage of this decomposition, which may be applied to any factorable function, is that it accounts for common subexpressions, thus enables tighter relaxations. Its main drawback, on the other hand, is that it may introduce a large number of auxiliary variables and constraints.

- <B>Step 2. Relaxation</B>
  - The bilinear terms \f$v_k = v_iv_j\f$, \f$v_i^{\rm L}\leq v_i\leq v_i^{\rm U}\f$, \f$v_j^{\rm L}\leq v_j\leq v_j^{\rm U}\f$, can be replaced by their polyhedral envelopes as
\f{align*}
v_k=v_iv_j \quad \xrightarrow{\displaystyle\text{relax.}}\quad \left\{\begin{array}{l}
v_k \geq v_i^{\rm L}v_j+v_j^{\rm L}v_i-v_i^{\rm L}v_j^{\rm L}\\
v_k \geq v_i^{\rm U}v_j+v_j^{\rm U}v_i-v_i^{\rm U}v_j^{\rm U}\\
v_k \leq v_i^{\rm U}v_j+v_j^{\rm L}v_i-v_i^{\rm U}v_j^{\rm L}\\
v_k \leq v_i^{\rm L}v_j+v_j^{\rm U}v_i-v_i^{\rm L}v_j^{\rm U}
\end{array}\right.
\f}
  - The fractional terms \f$v_k = \frac{v_i}{v_j}\f$, \f$v_i^{\rm L}\leq v_i\leq v_i^{\rm U}\f$, \f$v_j^{\rm L}\leq v_j\leq v_j^{\rm U}\f$, \f$0\notin[v_j^{\rm L},v_j^{\rm U}]\f$, can be rewritten as bilinear terms \f$v_i = v_jv_k\f$ and relaxed as indicated above, with the following bounds for the variables \f$v_k\f$:
\f{align*}
\min\left\{\frac{v_j^{\rm L}}{v_k^{\rm L}}, \frac{v_j^{\rm L}}{v_k^{\rm U}}, \frac{v_j^{\rm U}}{v_k^{\rm L}}, \frac{v_j^{\rm U}}{v_k^{\rm U}}\right\} =: v_k^{\rm L} \leq v_k \leq v_k^{\rm U} := \max\left\{\frac{v_j^{\rm L}}{v_k^{\rm L}}, \frac{v_j^{\rm L}}{v_k^{\rm U}}, \frac{v_j^{\rm U}}{v_k^{\rm L}}, \frac{v_j^{\rm U}}{v_k^{\rm U}}\right\}
\f}
Note that, although straightforward, this approach of dealing with fractional terms does not generally yield the convex/concave envelopes (which turn out to be quite complicated nonlinear expressions -- see \ref sec_POLIMG_REFS).
\n
  - The univariate terms \f$v_k = \varphi(v_i)\f$, \f$v_i^{\rm L}\leq v_i\leq v_i^{\rm U}\f$, are relaxed differently depending on whether the function \f$\varphi\f$ is convex, concave, convexo-concave, etc., on \f$[v_i^{\rm L},v_i^{\rm U}]\f$.
\f{align*}
\text{convex case:} & \quad v_k=\varphi(v_i) \quad \xrightarrow{\displaystyle\text{relax}} \quad \left\{\begin{array}{l} v_k \geq \varphi(v_i)\\ v_k \leq \varphi(v_i^{\rm L}) + \frac{\varphi(v_i^{\rm U})-\varphi(v_i^{\rm L})}{v_i^{\rm U}-v_i^{\rm L}}(v_i-v_i^{\rm L}) \end{array}\right.\\
\text{concave case:} & \quad v_k=\varphi(v_i) \quad \xrightarrow{\displaystyle\text{relax}} \quad \left\{\begin{array}{l} v_k \geq \varphi(v_i^{\rm L}) + \frac{\varphi(v_i^{\rm U})-\varphi(v_i^{\rm L})}{v_i^{\rm U}-v_i^{\rm L}}(v_i-v_i^{\rm L})\\ v_k \leq \varphi(v_i) \end{array}\right.\\
\text{convexo-concave case:} & \quad v_k=\varphi(v_i) \quad \xrightarrow{\displaystyle\text{relax}} \quad \left\{\begin{array}{l}
v_k \geq \left\{\begin{array}{ll}
\varphi(v_i), & \text{if $v_i\leq v_{\rm m}^{\rm cv}$}\\
\varphi(v_i^{\rm U}) + \frac{\varphi(v_i^{\rm U})-\varphi(v_{\rm m}^{\rm cv})}{v_i^{\rm U}-v_{\rm m}^{\rm cv}}(v_i-v_i^{\rm U}), & \text{otherwise}
\end{array}\right.\\
v_k \leq \left\{\begin{array}{ll}
\varphi(v_i), & \text{if $v_i\geq v_{\rm m}^{\rm cc}$}\\
\varphi(v_i^{\rm L}) + \frac{\varphi(v_i^{\rm L})-\varphi(v_{\rm m}^{\rm cc})}{v_i^{\rm L}-v_{\rm m}^{\rm cc}}(v_i-v_i^{\rm L}), & \text{otherwise}
\end{array}\right.
\end{array}\right.\\
& \quad \text{with:}\ v_{\rm m}^{\rm cv},v_{\rm m}^{\rm cc}\in[v_i^{\rm L},v_i^{\rm U}]:\ \varphi'(v_{\rm m}^{\rm cv}) = \textstyle\frac{\varphi(v_i^{\rm U})-\varphi(v_{\rm m}^{\rm cv})}{v_i^{\rm U}-v_{\rm m}^{\rm cv}} \text{ and } \varphi'(v_{\rm m}^{\rm cc}) = \textstyle\frac{\varphi(v_i^{\rm L})-\varphi(v_{\rm m}^{\rm cc})}{v_i^{\rm L}-v_{\rm m}^{\rm cc}}
\f}
\image html exm_uni.png
\n
  .

- <B>Step 3. Polyhedral Outer-Approximation</B>\n
Every convex, nonlinear univariate constraint generated in Step 2 is outer-approximated by constructing supporting cuts at a number of well-chosen points. Although the resulting polyhedral relaxations are inherently weaker than the nonlinear relaxations, LP solvers are currently more robust and faster than NLP solvers.\n
An iterative scheme (a.k.a. sandwich algorithm) can be applied that adds linearization points in such a way that the maximum distance \f$\delta^{\rm max}\f$ between the nonlinear constraint and its polyhedral approximation decreases as the inverse of the square of the number \f$n\f$ of linearization points; that is, \f$\delta^{\rm max}\propto \frac{1}{n^2}\f$. This algorithm proceeds as follows:
  -# Construct cuts at both interval end-points \f$v_i^{\rm L}\f$ and \f$v_i^{\rm U}\f$
  -# <B>REPEAT</B>
    - Identify an interval \f$[v_i^\ell,v_i^{\ell+1}]\f$ with maximum outer-approximation error \f$\delta_{\rm max}\f$
    - Subdivide \f$[v_i^\ell,v_i^{\ell+1}]\f$ at a suitably chosen point \f$v_i^{\rm new}\f$
    .
    <B>UNTIL</B> \f$\delta_{\rm max} < \varepsilon^{\rm tol}\f$
  .
  In particular, several strategies have been proposed for selecting a new linearization point \f$v_i^{\rm new}\f$. The <I>interval bisection rule</I> and the <I>maximum error rule</I> are depicted below.

\image html  OAcvx_strategy.png


\section sec_POLIMG_COMP How to generate a polyhedral relaxation of a factorable function?

For illustration, suppose that we want to compute a polyhedral enclosure for the image set of function:
\f[
{\bf g}(x) := \left(\begin{array}{c}
\log(x_1)+x^2_2 \\
\sin(x_1)-\cos(x_2) \\
\end{array} \right) \qquad \text{with} \qquad 
x \in [1,5]\times [2,6].
\f]

For simplicity, the underlying interval bounds are propagated using the default interval type mc::Interval, the required header files are:
  
\code
#include "interval.hpp"
typedef mc::Interval I;

#include "polimage.hpp"
typedef mc::PolImg<I> PI;
typedef mc::PolVar<I> PV;
\endcode

First, the DAG of the vector function \f${\bf g}\f$ is defined:
\code 
  mc::FFGraph DAG;
  mc::FFVar X[2]; X[0].set( &DAG ); X[1].set( &DAG );
  mc::FFVar F[2]; F[0] = log(X[0])+pow(X[1],2); F[1] = sin(X[0])-cos(X[1]); 
  std::cout << DAG;
\endcode

The following output is displayed, including the auxiliary variables created during the DAG decomposition of the factorable functions:
\verbatim
  DAG VARIABLES:
    X0    => { Z1 Z4 }
    X1    => { Z0 Z3 }

  DAG INTERMEDIATES:
    Z0    <=  SQR( X1 )       => { Z2 }
    Z1    <=  LOG( X0 )       => { Z2 }
    Z2    <=  Z0 + Z1         => { }
    Z3    <=  COS( X1 )       => { Z5 }
    Z4    <=  SIN( X0 )       => { Z5 }
    Z5    <=  Z4 - Z3         => { }
\endverbatim

Next, the polyhedral relaxation environment is created and the main variables are initialized:
\code 
  mc::PolImg<I> Env;
  I IX[2] = { I(1,5), I(2,6) };
  mc::PolVar<I> PX[2]; PX[0].set( &Env, X[0], IX[0] ); PX[1].set( &Env, X[1], IX[1] );
\endcode

Then, the polyhedral relaxation of the image set is initialized by propagating bounds for the lifted variables through the DAG:
\code 
  mc::PolVar<I> PF[2]; DAG.eval( 2, F, PF, 2, X, PX );
  std::cout << Env;
\endcode

This produces the following output, where no cuts have been generated to define the polyhedral enclosure yet:
\verbatim
VARIABLES:
  X0	 in [  1.00000e+00 :  5.00000e+00 ]	 (DAG: X0)
  X1	 in [  2.00000e+00 :  6.00000e+00 ]	 (DAG: X1)
  X2	 in [  4.00000e+00 :  3.60000e+01 ]	 (DAG: Z0)
  X3	 in [  0.00000e+00 :  1.60944e+00 ]	 (DAG: Z1)
  X4	 in [  4.00000e+00 :  3.76094e+01 ]	 (DAG: Z2)
  X6	 in [ -1.00000e+00 :  9.60170e-01 ]	 (DAG: Z3)
  X5	 in [ -1.00000e+00 :  1.00000e+00 ]	 (DAG: Z4)
  X7	 in [ -1.96017e+00 :  2.00000e+00 ]	 (DAG: Z5)

NO AUXILIARY

NO BILINEAR OR FRACTIONAL TERM

NO CUT
\endverbatim

Finally, polyhedral cuts are generated and displayed as follows:
\code 
  Env.generate_cuts( 2, PF );
  std::cout << Env;
\endcode

\verbatim
VARIABLES:
  X0	 in [  1.00000e+00 :  5.00000e+00 ]	 (DAG: X0)
  X1	 in [  2.00000e+00 :  6.00000e+00 ]	 (DAG: X1)
  X2	 in [  4.00000e+00 :  3.60000e+01 ]	 (DAG: Z0)
  X3	 in [  0.00000e+00 :  1.60944e+00 ]	 (DAG: Z1)
  X4	 in [  4.00000e+00 :  3.76094e+01 ]	 (DAG: Z2)
  X6	 in [ -1.00000e+00 :  9.60170e-01 ]	 (DAG: Z3)
  X5	 in [ -1.00000e+00 :  1.00000e+00 ]	 (DAG: Z4)
  X7	 in [ -1.96017e+00 :  2.00000e+00 ]	 (DAG: Z5)

NO AUXILIARY

NO BILINEAR OR FRACTIONAL TERM

CUTS:
  + 1.00000e+00X2 + 1.00000e+00X3 - 1.00000e+00X4 = -0.00000e+00
  + 1.00000e+00X5 - 1.00000e+00X6 - 1.00000e+00X7 = -0.00000e+00
  + 4.00000e+00X3 - 1.60944e+00X0 >= -1.60944e+00
  + 1.00000e+00X0 - 1.00000e+00X3 >= 1.00000e+00
  + 1.00000e+00X0 - 5.00000e+00X3 >= -3.04719e+00
  + 1.00000e+00X0 - 2.75054e+00X3 >= -3.24492e-02
  + 1.00000e+00X0 - 1.80361e+00X3 >= 7.39860e-01
  + 1.00000e+00X0 - 3.81983e+00X3 >= -1.29953e+00
  + 4.00000e+00X2 - 3.20000e+01X1 <= -4.80000e+01
  + 1.00000e+00X5 - 5.40302e-01X0 <= 3.01169e-01
  + 1.00000e+00X5 + 0.00000e+00X0 <= 1.00000e+00
  + 1.00000e+00X5 - 2.73845e-01X0 <= 6.07581e-01
  + 1.00000e+00X5 - 4.08535e-01X0 <= 4.42948e-01
  + 1.00000e+00X5 - 1.37362e-01X0 <= 7.93681e-01
  + 1.00000e+00X5 + 0.00000e+00X0 <= 1.00000e+00
  + 1.00000e+00X5 + 6.31638e-01X0 <= 2.19927e+00
  + 1.00000e+00X5 + 3.22022e-01X0 <= 1.55814e+00
  + 1.00000e+00X5 + 4.79346e-01X0 <= 1.87021e+00
  + 1.00000e+00X5 + 1.61734e-01X0 <= 1.26716e+00
  + 1.00000e+00X6 - 3.44637e-01X1 <= -1.10542e+00
  + 1.00000e+00X6 - 2.79415e-01X1 <= -7.16323e-01
  + 1.00000e+00X2 - 4.00000e+00X1 >= -4.00000e+00
  + 1.00000e+00X2 - 1.20000e+01X1 >= -3.60000e+01
  + 1.00000e+00X2 - 8.00000e+00X1 >= -1.60000e+01
  + 1.00000e+00X2 - 6.00000e+00X1 >= -9.00000e+00
  + 1.00000e+00X2 - 1.00000e+01X1 >= -2.50000e+01
  + 1.00000e+00X5 + 5.35701e-01X0 >= 1.37717e+00
  + 1.00000e+00X5 + 0.00000e+00X0 >= -1.00000e+00
  + 1.00000e+00X5 + 2.71443e-01X0 >= 2.42071e-01
  + 1.00000e+00X5 + 4.04992e-01X0 >= 8.25290e-01
  + 1.00000e+00X5 + 1.36150e-01X0 >= -3.67693e-01
  + 1.00000e+00X5 + 0.00000e+00X0 >= -1.00000e+00
  + 1.00000e+00X5 - 2.83662e-01X0 >= -2.37724e+00
  + 1.00000e+00X5 - 1.42321e-01X0 >= -1.68082e+00
  + 1.00000e+00X5 - 2.13178e-01X0 >= -2.02739e+00
  + 1.00000e+00X5 - 7.12210e-02X0 >= -1.33816e+00
  + 1.00000e+00X6 + 9.09297e-01X1 >= 1.40245e+00
  + 1.00000e+00X6 + 0.00000e+00X1 >= -1.00000e+00
  + 1.00000e+00X6 + 4.78987e-01X1 >= 3.87705e-01
  + 1.00000e+00X6 + 7.05713e-01X1 >= 9.55690e-01
  + 1.00000e+00X6 + 2.41998e-01X1 >= -2.69168e-01
  + 1.00000e+00X6 + 0.00000e+00X1 >= -1.00000e+00
  + 1.00000e+00X6 - 8.07940e-01X1 >= -3.88747e+00
  + 1.00000e+00X6 - 4.18937e-01X1 >= -2.40524e+00
  + 1.00000e+00X6 - 6.19996e-01X1 >= -3.14699e+00
  + 1.00000e+00X6 - 2.11107e-01X1 >= -1.68558e+00
\endverbatim

By default, the cuts in the polyhedral relaxation of a convex univariate terms are generated according to the <I>maximum error rule</I> (see above), and a maximum of 5 cuts are generated for each nonlinear constraint. The cut generation is also controlled by the absolute and relative tolerances on the maximum outer-approximation error, which are both set to \f$10^{-3}\f$ by default. All these default values can be altered as explained below in the section \ref sec_POLIMG_OPT below.

All the cuts can be retreived by using the function mc::PolImg::Cuts, which returns a set of cuts as defined in the class mc::PolCuts.


\section sec_POLIMG_OPT What are the options in computing a polyhedral relaxation?

All the options are defined in the structure mc::PolImg::Options, and the default values can be altered via the public static member mc::PolImg::options; for example:

\code
  Env.options.AGGREG_LIN = false;
  Env.options.SANDWICH_MAXCUT = 7;
\endcode


\section sec_POLIMG_ERR What errors may be encoutered in computing a polyhedral relaxation?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, an instance of the class mc::PolImg::Exceptions is thrown, which contains the type of error. Additional exceptions may be sent by the template argument class in propagating the bounds through the DAG, as well as by the DAG class itself. It is the responsibility of a user to test whether an exception was thrown, and then make the appropriate changes. Should an exception be thrown and not caught by the calling program, execution will abort.

\section sec_POLIMG_REFS References

- Tawarmalani, M., and N.V. Sahinidis, <A href="http://dx.doi.org/10.1007/s10107-003-0467-6">Global optimization of mixed-integer nonlinear programs: A theoretical and computational study</A>, <I>Mathematical Programming</I>, <B>99</B>(3):563-591, 2004.
- Smith, E.M.B, and C.C. Pantelides, <A href="http://dx.doi.org/10.1016/S0098-1354(98)00286-5">A symbolic reformulation/spatial branch-and-bound algorithm for the global optimisation of nonconvex MINLPs</A>, <I>Computers & Chemical Engineering</I>, <B>23</B>(4-5):457-478, 1999.
.
*/

/*
TODO: 
- Semilinear relaxation of terms that are neither convex nor concave? (e.g., pow(x,3)) ==> OK
- Account for multiple occurence of variables ==> OK
- Enable DC-decomposition of bilinear terms (e.g. lists of product/division terms) ==> OK
- Fix bug in DC scaling ==> OK
- Implement alternative piecewise relaxation approaches ==> OK
- Add all remaining univariate terms (trigo, min/max)
- Implement RLT and reduction constraints
- Distinguish between binary and integer variables?
*/

#ifndef MC__POLIMAGE_H
#define MC__POLIMAGE_H

#include <assert.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <queue>
#include <map>

#include "ffunc.hpp"
#include "mcop.hpp"

#undef  MC__POLIMG_DEBUG
#undef  MC__POLIMG_DEBUG_CUTS

namespace mc
{

template< class T > class PolImg;
template< class T > class PolLinEq;
template< class T > class PolCut;
template< class T > class lt_PolVar;

//! @brief C++ class for defining polytopic image variables
////////////////////////////////////////////////////////////////////////
//! mc::PolVar is a C++ class for defining polytopic image variables.
//! The template parameter corresponds to the type used to propagate
//! variable range.
////////////////////////////////////////////////////////////////////////
template< class T >
class PolVar
////////////////////////////////////////////////////////////////////////
{
  friend class PolImg<T>;
  friend class lt_PolVar<T>;

  template< class U > friend  PolVar<U> operator+( const PolVar<U>& );
  template< class U > friend  PolVar<U> operator+( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator+( const PolVar<U>&, const double );
  template< class U > friend  PolVar<U> operator+( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator+( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator+( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>&, const double );
  template< class U > friend  PolVar<U> operator-( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator-( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator*( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator*( const PolVar<U>&, const double );			
  template< class U > friend  PolVar<U> operator*( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator*( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator*( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator/( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator/( const PolVar<U>&, const double );			
  template< class U > friend  PolVar<U> operator/( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator/( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator/( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator^( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator^( const PolVar<U>&, const double );
  template< class U > friend  PolVar<U> operator^( const double, const PolVar<U>& );
    
  template< class U > friend  PolVar<U> inv ( const PolVar<U>& );
  template< class U > friend  PolVar<U> exp ( const PolVar<U>& );
  template< class U > friend  PolVar<U> log ( const PolVar<U>& );
  template< class U > friend  PolVar<U> sqrt( const PolVar<U>& );
  template< class U > friend  PolVar<U> sqr ( const PolVar<U>& );
  template< class U > friend  PolVar<U> pow ( const PolVar<U>&, const PolVar<U>& );  
  template< class U > friend  PolVar<U> cheb( const PolVar<U>&, const PolVar<U>& );  
  template< class U > friend  PolVar<U> cos ( const PolVar<U>& );
  template< class U > friend  PolVar<U> sin ( const PolVar<U>& );
  template< class U > friend  PolVar<U> tan ( const PolVar<U>& );
  template< class U > friend  PolVar<U> acos( const PolVar<U>& );
  template< class U > friend  PolVar<U> asin( const PolVar<U>& );
  template< class U > friend  PolVar<U> atan( const PolVar<U>& );
  template< class U > friend  PolVar<U> fabs( const PolVar<U>& );
  template< class U > friend  PolVar<U> fstep( const PolVar<U>& );

  public:
    /** @ingroup POLYTOPE
     *  @{
     */
    //! @brief Enumeration type for variables in factorable program
    enum TYPE{
      VARCONT=0,//!< DAG continuous variable
      VARINT,	//!< DAG integer variable
      AUXCONT,	//!< Auxiliary continuous variable
      AUXINT,	//!< Auxiliary integer variable   
      AUXCST	//!< Auxiliary constant
    };
    //! @brief Typedef for variable identifier in factorable program
    typedef std::pair< TYPE, unsigned > t_idVar;

    //! @brief Return string with variable name for identifier <a>id</a>
    std::string name
      () const
      {
        std::ostringstream ovar;
        switch( _id.first ){
          case VARCONT: ovar << "X"; break;
          case VARINT:  ovar << "Y"; break;
          case AUXCONT: ovar << "W"; break;
          case AUXINT:  ovar << "Z"; break;
          case AUXCST:  ovar << "C"; break;
        }
        ovar << _id.second;
        return ovar.str();
     }
  /** @} */ 
							  
  private:
    //! @brief pointer to underlying polytope
    PolImg<T>* _img;
    //! @brief underlying variable in DAG
    FFVar _var;
    //! @brief variable range
    T _range;
    //! @brief variable identifier (type and index)
    t_idVar _id;
    //! @brief variable break-points
    std::set<double> _breakpts;
    //! @brief variable subdivision w.r.t. break-points
    mutable std::pair< std::vector<double>, std::vector<PolVar<T> > > _subdiv;
    //! @brief flag indicating whether cuts have already been generated
    mutable bool _hascuts;

    //! @brief set as DAG variable <a>X</a> in polytope image <a>P</a> with range <a>B</a>
    void _set
      ( PolImg<T>*P, FFVar&X, const T&B, const bool cont, const unsigned index )
      { _img = P; _var = X; _range = (!_var.cst()? B: _var.num().val());
        _id = std::make_pair( cont? VARCONT: VARINT, index );
        _breakpts.clear(); reset_subdiv(); reset_cuts(); }
    //! @brief set as auxiliary variable in polytope image <a>P</a> with range <a>B</a>
    void _set
      ( PolImg<T>*P, const T&B, const bool cont, const unsigned index )
      { _img = P; _var = 0; _range = B;
        _id = std::make_pair( cont? AUXCONT: AUXINT, index );
         _breakpts.clear(); reset_subdiv(); reset_cuts(); }
    //! @brief update variable bounds and type
    void _update
      ( const T&B, const bool cont )
      { _range = B; _id.first = (cont? VARCONT: VARINT); }

    //! @brief push variable subdivision
    void _push_subdiv
      ( double pt ) const
      { if( !_img ) return;
        auto itVar = _img->_Vars.find( const_cast<FFVar*>(&_var) );
        if( itVar == _img->_Vars.end() ) return;
        itVar->second->_subdiv.first.push_back( pt );
        _subdiv.first.push_back( pt ); }
    //! @brief push variable subdivision
    void _push_subdiv
      ( PolVar<T> var ) const
      { if( !_img ) return;
        auto itVar = _img->_Vars.find( const_cast<FFVar*>(&_var) );
        if( itVar == _img->_Vars.end() ) return;
        itVar->second->_subdiv.second.push_back( var );
        _subdiv.second.push_back( var ); }

  public:
    /** @ingroup POLYTOPE
     *  @{
     */
    //! @brief Constructor for a constant value <a>d</a> (default)
    PolVar
      ( const double d=0. )
      : _img(0), _var(FFVar(d)), _range(d), _id( AUXCST, 0 ), _hascuts(false) 
      {} 
    //! @brief Constructor for a constant value <a>n</a>
    PolVar
      ( const int n )
      : _img(0), _var(FFVar(n)), _range(n), _id( AUXCST, 0 ), _hascuts(false)
      {}
    //! @brief Constructor for DAG variable <a>X</a> in polytope image <a>P</a> with range <a>B</a>
    PolVar
      ( PolImg<T>*P, FFVar&X, const T&B=0., const bool cont=true )
      { set( P, X, B, cont ); }
    //! @brief Constructor for auxiliary variable in polytope image <a>P</a> with range <a>B</a>
    PolVar
      ( PolImg<T>*P, const T&B=0., const bool cont=true )
      { set( P, B, cont ); }
    //! @brief Copy constructor for a polytope image <a>P</a>
    PolVar
      ( const PolVar<T>&P )
      : _img(P._img), _var(P._var), _range(P._range), _id( P._id ),
        _breakpts( P._breakpts ), _subdiv( P._subdiv )
      {}

    //! @brief Destructor
    virtual ~PolVar
      ()
      {};

    //! @brief Update range <a>B</a> and type <a>cont</a> of variable in polytope image
    PolVar<T>& update
      ( const T&B=0., const bool cont=true )
      { if( !_img ) return *this;
        *this = *_img->_append_var( &_var, B, cont );
        reset_subdiv(); reset_cuts(); return *this; }
    //! @brief set as DAG variable <a>X</a> in polytope image <a>P</a> with range <a>B</a>
    PolVar<T>& set
      ( PolImg<T>*P, FFVar&X, const T&B=0., const bool cont=true )
      { *this = *P->_append_var( &X, B, cont );
        _breakpts.clear(); reset_subdiv(); reset_cuts(); return *this; }
    //! @brief set as auxiliary variable in polytope image <a>P</a> with range <a>B</a>
    PolVar<T>& set
      ( PolImg<T>*P, const T&B=0., const bool cont=true )
      { *this = *P->_append_aux( B, cont );
        _breakpts.clear(); reset_subdiv(); reset_cuts(); return *this; }

    //! @brief get variable constness
    bool cst
      () const
      { return (_var.id().first == FFVar::CINT
             || _var.id().first == FFVar::CREAL? true: false); }
    //! @brief get variable range
    T range
      () const
      { return _var.cst()? _var.num().val(): _range; }
      //{ return _range; }
    //! @brief get pointer to polytopic image
    PolImg<T>* image
      () const
      { return _img; }
    //! @brief get pointer to variable identifier
    t_idVar id
      () const
      { return _id; }
    //! @brief get reference to DAG variable
    FFVar& var
      ()
      { return _var; }
    //! @brief get const reference to DAG variable
    const FFVar& var
      () const
      { return _var; }

    //! @brief add variable break-points
    void add_breakpt
      ( const double bkpt );
    //! @brief get variable break-points
    const std::set<double>& breakpts
      () const
      { return _breakpts; }

    //! @brief get variable subdivision
    const std::pair< std::vector<double>, std::vector<PolImg<T> > >& subdiv
      () const
      { return _subdiv; }
    //! @brief reset variable subdivision
    void reset_subdiv
      () const
      { _subdiv.first.clear(); _subdiv.second.clear(); }
    //! @brief create variable subdivision
    const std::vector<double>& create_subdiv
      ( const double XL, const double XU, const bool reset=false ) const;
    //! @brief set SOS2 variable subdivision
    const std::vector< PolVar<T> >& SOS2_subdiv
      ( FFOp*pOp=0, const bool reset=false ) const;
    //! @brief set linear binary variable subdivision
    const std::vector< PolVar<T> >& BIN_subdiv
      ( FFOp*pOp=0, const bool reset=false ) const;

    //! @brief get cuts flag 
    bool cuts
      () const
      { return _hascuts; }
    //! @brief reset cuts flag to false
    void reset_cuts
      () const
      { _hascuts = false; }
    //! @brief set cuts flag to true
    void set_cuts
      () const
      { _hascuts = true; }
    //! @brief propagate cuts backwards through polyhedral image
    void generate_cuts
      () const;
    //! @brief propagate linear terms backwards through polyhedral image
    bool generate_cuts_linear
      ( PolLinEq<T>*&pLin ) const;
    //! @brief propagate default cuts backwards through polyhedral image
    void generate_cuts_default
      () const;

    //! @brief Public overloads
    PolVar<T>& operator= ( const PolVar<T>& );
    PolVar<T>& operator= ( const double );
    PolVar<T>& operator= ( const int );

    PolVar<T>& operator+=( const PolVar<T>& );
    PolVar<T>& operator-=( const PolVar<T>& );
    PolVar<T>& operator*=( const PolVar<T>& );
    PolVar<T>& operator/=( const PolVar<T>& );
  /** @} */ 
};

//! @brief C++ structure for ordering of polytopic variables
template <class T> 
struct lt_PolVar
{
  bool operator()
    ( const PolVar<T>*Var1, const PolVar<T>*Var2 ) const
    {
      // Order variables/constants w.r.t. their types first
      if( Var1->_id.first < Var2->_id.first ) return true;
      if( Var1->_id.first > Var2->_id.first ) return false;
      // If variables, order w.r.t. their index next
      switch( Var1->_id.first ){
        case PolVar<T>::VARCONT: case PolVar<T>::VARINT:
        case PolVar<T>::AUXCONT: case PolVar<T>::AUXINT:
          if( Var1->_id.second < Var2->_id.second ) return true;
          if( Var1->_id.second > Var2->_id.second ) return false;
          break;
        case PolVar<T>::AUXCST:
          lt_FFNum ltNum;
          return ltNum( &Var1->_var.num(), &Var2->_var.num() );
          break;
      }
      return false;
    }
};

template <class T> 
inline 
PolVar<T>&
PolVar<T>::operator=
( const PolVar<T>& P ) 
{
  _img = P._img;
  _var = P._var;
  _range = P._range;
  _id = P._id;
  _breakpts = P._breakpts;
  _subdiv = P._subdiv;
  return *this; 
}

template <class T> 
inline 
PolVar<T>&
PolVar<T>::operator=
( const double d )
{
  _img = 0;
  _var = FFVar(d);
  _range = d;
  _id = std::make_pair( AUXCST, 0 );
  _breakpts.clear();
  reset_subdiv();
  return *this; 
}

template <class T> 
inline 
PolVar<T>&
PolVar<T>::operator=
( const int n ) 
{
  _img = 0;
  _var = FFVar(n);
  _range = n;
  _id = std::make_pair( AUXCST, 0 );
  _breakpts.clear();
  reset_subdiv();
  return *this; 
}

template <typename T> inline PolVar<T>&
PolVar<T>::operator +=
( const PolVar<T>&P1 )
{
   PolVar<T> P2( *this );
   *this = P2 + P1;
   return *this;
}

template <typename T> inline PolVar<T>&
PolVar<T>::operator -=
( const PolVar<T>&P1 )
{
   PolVar<T> P2( *this );
   *this = P2 - P1;
   return *this;
}

template <typename T> inline PolVar<T>&
PolVar<T>::operator *=
( const PolVar<T>&P1 )
{
   PolVar<T> P2( *this );
   *this = P2 * P1;
   return *this;
}

template <typename T> inline PolVar<T>&
PolVar<T>::operator /=
( const PolVar<T>&P1 )
{
   PolVar<T> P2( *this );
   *this = P2 / P1;
   return *this;
}

template <typename T>
inline void
PolVar<T>::add_breakpt
( const double bkpt )
{
   if( !_img ) return;
   auto itVar = _img->_Vars.find( &_var );
   if( itVar == _img->_Vars.end() ) return;
   double atol = _img->options.BREAKPOINT_ATOL, rtol = _img->options.BREAKPOINT_RTOL;
   if( isequal( Op<T>::l(itVar->second->range()), bkpt, atol, rtol ) 
    || isequal( Op<T>::u(itVar->second->range()), bkpt, atol, rtol ) ) return;
   auto itL = _breakpts.lower_bound( bkpt );
   if( itL!=_breakpts.end() && isequal( *itL, bkpt, atol, rtol ) ) return;
   auto itU = _breakpts.upper_bound( bkpt );
   if( itU!=_breakpts.end() && isequal( *itU, bkpt, atol, rtol ) ) return;
   itVar->second->_breakpts.insert( bkpt );
   itVar->second->reset_subdiv();
   _breakpts.insert( bkpt );
   reset_subdiv();
}

template <typename T>
inline const std::vector<double>&
PolVar<T>::create_subdiv
( const double XL, const double XU, const bool reset ) const
{
  if( !reset && !_subdiv.first.empty() ) return _subdiv.first;
  if(  reset && !_subdiv.first.empty() ) reset_subdiv();
  _push_subdiv( XL ); 
  if( !_breakpts.empty() ){
    for( auto it = _breakpts.upper_bound( XL );
      it != _breakpts.end() && *it < XU; ++it )
      _push_subdiv( *it ); 
  }
  _push_subdiv( XU ); 
  return _subdiv.first;
}

template <typename T>
inline const std::vector< PolVar<T> >&
PolVar<T>::BIN_subdiv
( FFOp*pOp, const bool reset ) const
{
  if( !reset && !_subdiv.second.empty() ) return _subdiv.second;
  if(  reset && !_subdiv.second.empty() ) _subdiv.second.clear();
  if( !_img ) return _subdiv.second;

  const unsigned nsubint = _subdiv.first.size()-1;
  double coef[nsubint];
  for( unsigned isub=0; isub<nsubint; isub++ ){
    coef[isub] = _subdiv.first[isub] - _subdiv.first[isub+1];
    _push_subdiv( PolVar<T>( _img, Op<T>::zeroone(), true ) ); 
  }
  _img->_append_cut( pOp, PolCut<T>::EQ, _subdiv.first[0], nsubint, _subdiv.second.data(), coef, *this, 1. );

  for( unsigned isub=0; isub<nsubint-1; isub++ ){
    _push_subdiv( PolVar<T>( _img, Op<T>::zeroone(), false ) );
    _img->_append_cut( pOp, PolCut<T>::LE, 0., _subdiv.second[nsubint+isub], 1., _subdiv.second[isub], -1. );
    _img->_append_cut( pOp, PolCut<T>::GE, 0., _subdiv.second[nsubint+isub], 1., _subdiv.second[isub+1], -1. );
  }

  return _subdiv.second;
}

template <typename T>
inline const std::vector< PolVar<T> >&
PolVar<T>::SOS2_subdiv
( FFOp*pOp, const bool reset ) const
{
  if( !reset && !_subdiv.second.empty() ) return _subdiv.second;
  if(  reset && !_subdiv.second.empty() ) _subdiv.second.clear();
  if( !_img ) return _subdiv.second;

  const unsigned nsubint = _subdiv.first.size();
  double coef[nsubint];
  for( unsigned isub=0; isub<nsubint; isub++ ){
    coef[isub] = 1.;
    _push_subdiv( PolVar<T>( _img, Op<T>::zeroone(), true ) ); 
  }
  _img->_append_cut( pOp, PolCut<T>::EQ, 1., nsubint, _subdiv.second.data(), coef );

  for( unsigned isub=0; isub<nsubint; isub++ )
    coef[isub] = _subdiv.first[isub];
  _img->_append_cut( pOp, PolCut<T>::EQ, 0., nsubint, _subdiv.second.data(), coef, *this, -1. );
  _img->_append_cut( pOp, PolCut<T>::SOS2, 1., nsubint, _subdiv.second.data(), coef );

  return _subdiv.second;
}

template <typename T> inline void
PolVar<T>::generate_cuts
() const
{
  if( cuts() ) return;
  set_cuts();
#ifdef MC__POLIMG_DEBUG_CUTS
  std::cout << "CUTS FOR " << _var << ": " << *_var.ops().first << std::endl;
#endif

  PolLinEq<T>*pLin = 0;
  if( _img->options.AGGREG_LIN && generate_cuts_linear( pLin ) ){
    _img->_append_cut( _var.ops().first, PolCut<T>::EQ, -pLin->cst(),
                       pLin->size(), pLin->var(), pLin->coef(), *this, -1. );
    const PolVar<T>*pVar = pLin->var();
    for( unsigned i=0; i<pLin->size(); i++ ){
      auto itVar = _img->_Vars.find( const_cast<FFVar*>(&pVar[i]._var) );
      if( itVar != _img->_Vars.end() ) itVar->second->generate_cuts();
      //pVar[i].generate_cuts();
    }
    return;
  }

  generate_cuts_default();
  if( _var.ops().first->plop ){
    auto itVar = _img->_Vars.find( _var.ops().first->plop );
    if( itVar != _img->_Vars.end() ) itVar->second->generate_cuts();
  }
  if( _var.ops().first->prop ){
    auto itVar = _img->_Vars.find( _var.ops().first->prop );
    if( itVar != _img->_Vars.end() ) itVar->second->generate_cuts();
  }
  return;
}

template <typename T> inline void
PolVar<T>::generate_cuts_default
() const
{
   // PLUS, NEG, MINUS, TIMES, SCALE, DIV,
   // EXP, LOG, SQRT, SQR, IPOW, POW, SIN, COS, TAN, ASIN, ACOS, ATAN,
   // FABS, ERF, FSTEP, MINF, MAXF, INTER, CHEB
  
  switch( _var.ops().first->type ){
   case FFOp::CNST:
   case FFOp::VAR:
    break;

   case FFOp::PLUS:
    _img->_append_cuts_PLUS( this, _var.ops().first->plop, _var.ops().first->prop );
    break;

   case FFOp::MINUS:
    _img->_append_cuts_MINUS( this, _var.ops().first->plop, _var.ops().first->prop );
    break;

   case FFOp::NEG:
    _img->_append_cuts_NEG( this, _var.ops().first->plop );
    break;

   case FFOp::SCALE:
   case FFOp::TIMES:
    _img->_append_cuts_TIMES( this, _var.ops().first->plop, _var.ops().first->prop );
    break;

   case FFOp::DIV:
    _img->_append_cuts_DIV( this, _var.ops().first->plop, _var.ops().first->prop );
    break;

   case FFOp::IPOW:
    _img->_append_cuts_IPOW( this, _var.ops().first->plop, _var.ops().first->prop );
    break;

   case FFOp::SQR:
    _img->_append_cuts_SQR( this, _var.ops().first->plop );
    break;

   case FFOp::SQRT:
    _img->_append_cuts_SQRT( this, _var.ops().first->plop );
    break;

   case FFOp::EXP:
    _img->_append_cuts_EXP( this, _var.ops().first->plop );
    break;

   case FFOp::LOG:
    _img->_append_cuts_LOG( this, _var.ops().first->plop );
    break;

   case FFOp::FABS:
    _img->_append_cuts_FABS( this, _var.ops().first->plop );
    break;

   case FFOp::FSTEP:
    _img->_append_cuts_FSTEP( this, _var.ops().first->plop );
    break;

   case FFOp::COS:
    _img->_append_cuts_COS( this, _var.ops().first->plop );
    break;

   case FFOp::SIN:
    _img->_append_cuts_SIN( this, _var.ops().first->plop );
    break;

   case FFOp::INTER:
    _img->_append_cuts_INTER( this, _var.ops().first->plop, _var.ops().first->prop );
    break;

   case FFOp::CHEB:
//#ifndef MC__CHEB_RECURS
    _img->_append_cuts_CHEB( this, _var.ops().first->plop, _var.ops().first->prop );
//#endif
    break;

   default:
    std::cout << "  -> EXCEPTION FOR " << _var << ": " << *_var.ops().first << std::endl;
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::INTERN );
  }

  if( _var.ops().first->plop ){
    auto itVar = _img->_Vars.find( _var.ops().first->plop );
    if( itVar != _img->_Vars.end() ) itVar->second->generate_cuts();
  }
  if( _var.ops().first->prop ){
    auto itVar = _img->_Vars.find( _var.ops().first->prop );
    if( itVar != _img->_Vars.end() ) itVar->second->generate_cuts();
  }
  return;
}

template <typename T> inline bool
PolVar<T>::generate_cuts_linear
( PolLinEq<T>*&pLin ) const
{
#ifdef MC__POLIMG_DEBUG_CUTS
  std::cout << "  -> LINEAR CUTS FOR " << _var << ": " << *_var.ops().first << std::endl;
#endif

  switch( _var.ops().first->type ){
   case FFOp::PLUS:
    if( !_img->_lineq_PLUS( pLin, this, _var.ops().first->plop, _var.ops().first->prop ) )
      return false;
    break;

   case FFOp::MINUS:
    if( !_img->_lineq_MINUS( pLin, this, _var.ops().first->plop, _var.ops().first->prop ) )
      return false;
    break;

   case FFOp::NEG:
    if( !_img->_lineq_NEG( pLin, this, _var.ops().first->plop ) ) 
      return false;
    break;

   case FFOp::SCALE:
   case FFOp::TIMES:
    if( !_img->_lineq_TIMES( pLin, this, _var.ops().first->plop, _var.ops().first->prop ) )
      return false;
    break;

   case FFOp::DIV:
    if( !_img->_lineq_DIV( pLin, this, _var.ops().first->plop, _var.ops().first->prop ) )
      return false;
    break;

   case FFOp::FABS:
    if( !_img->_lineq_FABS( pLin, this, _var.ops().first->plop ) )
      return false;
    break;

   case FFOp::FSTEP:
    if( !_img->_lineq_FSTEP( pLin, this, _var.ops().first->plop ) )
      return false;
    break;

   default:
    return false;

   // MINF, MAXF, INTER?
  }

  if( _var.ops().first->plop ){
    auto itVar = _img->_Vars.find( _var.ops().first->plop );
    if( itVar != _img->_Vars.end() ) itVar->second->generate_cuts_linear( pLin );
  }
  if( _var.ops().first->prop ){
    auto itVar = _img->_Vars.find( _var.ops().first->prop );
    if( itVar != _img->_Vars.end() ) itVar->second->generate_cuts_linear( pLin );
  }
  return true;
}

//! @brief C++ structure for holding information about bilinear terms in a polytopic image
////////////////////////////////////////////////////////////////////////
//! mc::PolBilin is a C++ structure for holding information about
//! linear expressions (equalities) in a polytopic image.
////////////////////////////////////////////////////////////////////////
template< class T >
class PolLinEq
////////////////////////////////////////////////////////////////////////
{
 private:
  //! @brief Map of variables and coefficients in linear expression 
  std::map< const PolVar<T>*, double, lt_PolVar<T> > _terms;
  //! @brief Vector of coefficients in linear expression 
  std::vector< double > _coef;
  //! @brief Vector of variables in linear expression 
  std::vector< PolVar<T> > _var;
  //! @brief Constant coefficient
  double _const;

 public:
  //! @brief Constructor for linear expression
  PolLinEq<T>
    ( const PolVar<T>*Var )
    : _terms(), _coef(), _var(), _const(0)
    { _terms.insert( std::make_pair( Var, 1. ) ); }
  //! @brief Add constant term <a>cst</a>
  void add
    ( const double cst )
    { _const += cst; }
  //! @brief Add linear term <a>coef * Var</a>
  void add
    ( const double coef, const PolVar<T>*Var )
    { auto itVar = _terms.find( Var );
      if( itVar != _terms.end() )
        itVar->second += coef;
      else
        _terms.insert( std::make_pair( Var, coef ) ); }
  //! @brief Substitute variable <a>Res</a> with term <a>coef * Var</a>
  bool substitute
    ( const PolVar<T>*Res, const double coef, const PolVar<T>*Var )
    { auto itRes = _terms.find( Res );
      if( itRes == _terms.end() )
        return false;
      add( itRes->second * coef, Var );
      _terms.erase( itRes );
      return true; }
  //! @brief Substitute variable <a>Res</a> with term <a>cst</a>
  bool substitute
    ( const PolVar<T>*Res, const double cst )
    { auto itRes = _terms.find( Res );
      if( itRes == _terms.end() )
        return false;
      add( itRes->second * cst );
      _terms.erase( itRes );
      return true; }
  //! @brief Substitute variable <a>Res</a> with term <a>coef * Var + cst</a>
  bool substitute
    ( const PolVar<T>*Res, const double coef, const PolVar<T>*Var, const double cst )
    { auto itRes = _terms.find( Res );
      if( itRes == _terms.end() )
        return false;
      add( itRes->second * coef, Var );
      add( itRes->second * cst );
      _terms.erase( itRes );
      return true; }
  //! @brief Substitute variable <a>Res</a> with term <a>coef1 * Var1 + coef2 * Var2</a>
  bool substitute
    ( const PolVar<T>*Res, const double coef1, const PolVar<T>*Var1,
      const double coef2, const PolVar<T>*Var2 )
    { auto itRes = _terms.find( Res );
      if( itRes == _terms.end() )
        return false;
      add( itRes->second * coef1, Var1 );
      add( itRes->second * coef2, Var2 );
      _terms.erase( itRes );
      return true; }
  //! @brief Return constant term
  const double cst
    () const
    { return _const; }
  //! @brief Return pointer to coefficient array
  const double* coef
    ()
    { _coef.resize( _terms.size() );
      auto it = _terms.cbegin();
      for( unsigned i=0; it != _terms.cend(); ++it, i++ )
        _coef[i] = it->second;
      return _coef.data(); }
  //! @brief Return pointer to variable array
  const PolVar<T>* var
    ()
    { _var.resize( _terms.size() );
      auto it = _terms.cbegin();
      for( unsigned i=0; it != _terms.cend(); ++it, i++ )
        _var[i] = *it->first;
      return _var.data(); }
  //! @brief Return number of participating variables
  const unsigned size
    () const
    { return _terms.size(); }
};

//! @brief C++ structure for holding information about bilinear terms in a polytopic image
////////////////////////////////////////////////////////////////////////
//! mc::PolBilin is a C++ structure for holding information about
//! bilinear terms in a polytopic image. This is useful in connection to
//! DC decomposition in order to refine piecewise-linear cuts.
////////////////////////////////////////////////////////////////////////
template< class T >
struct PolBilin
////////////////////////////////////////////////////////////////////////
{
  //! @brief Constructor for bilinear term
  PolBilin<T>
    ( const PolVar<T>*Var )
    : var(Var), scal1(0), scal2(0)
    {}
  //! @brief corresponding variable in polytopic image (product result)
  const PolVar<T>* var;
  //! @brief scaling constant for sum term in DC decomposition
  PolVar<T>* scal1;
  //! @brief scaling constant for difference term in DC decomposition
  PolVar<T>* scal2;
};

//! @brief C++ template class for defining cuts in a polytopic image
////////////////////////////////////////////////////////////////////////
//! mc::PolCut is a C++ template class defining cuts in a polytopic
//! image.
////////////////////////////////////////////////////////////////////////
template< class T >
class PolCut
////////////////////////////////////////////////////////////////////////
{
  // friends of class PolCut for operator overloading
  template< class U > friend std::ostream& operator<<( std::ostream&, const PolCut<U>& );

public:
  /** @ingroup LPRELAX
   *  @{
   */
  //! @brief Enumeration type for cuts and special sets
  enum TYPE{
    EQ=0,	//!< Equality constraint Ax=b
    LE,		//!< Inequality constraint Ax<=b
    GE,		//!< Inequality constraint Ax>=b
    SOS1,	//!< SOS1-type constraint 
    SOS2	//!< SOS2-type constraint 
  };
  /** @} */

private:
  //! @brief Pointer to defining operation
  FFOp* _op;
  //! @brief Type of cut
  TYPE _type;
  //! @brief Right-hand side
  double _rhs;
  //! @brief Number of participating variables
  unsigned _nvar;
  //! @brief Participating variables
  PolVar<T>* _var;
  //! @brief Coefficients
  double* _coef;

public:
  /** @ingroup LPRELAX
   *  @{
   */
  //! @brief Retreive type of cut
  TYPE type
    () const
    { return _type; }
  //! @brief Retreive right-hand side
  double rhs
    () const
    { return _rhs; }
  //! @brief Retreive number of participating variables
 unsigned nvar
    () const
    { return _nvar; }
  //! @brief Retreive variable coefficients
  const double* coef
    () const 
    { return _coef; }
  //! @brief Retreive variable pointers
  const PolVar<T>* var
    () const 
    { return _var; }
  //! @brief Retreive pointer to corresponding operation in DAG
  FFOp*& op
    ()
    { return _op; }
  //! @brief Retreive pointer to corresponding operation in DAG
  const FFOp* op
    () const
    { return _op; }

  //! @brief Constructor for cut w/ 1 participating variable
  PolCut
    ( FFOp*op, TYPE type, const double b, const PolVar<T>&X1, const double a1 )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = 1;
      _coef = new double[_nvar];
      _var  = new PolVar<T>[_nvar];
      _coef[0] = a1;
      _var[0] = X1;
    }
  //! @brief Constructor for cut w/ 2 participating variables
  PolCut
    ( FFOp*op, TYPE type, const double b, const PolVar<T>&X1, const double a1,
      const PolVar<T>&X2, const double a2 )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = 2;
      _coef = new double[_nvar];
      _var = new PolVar<T>[_nvar];
      _coef[0] = a1;
      _coef[1] = a2;
      _var[0] = X1;
      _var[1] = X2;
    }
  //! @brief Constructor for cut w/ 3 participating variables
  PolCut
    ( FFOp*op, TYPE type, const double b, const PolVar<T>&X1, const double a1,
      const PolVar<T>&X2, const double a2, const PolVar<T>&X3, const double a3 )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = 3;
      _coef = new double[_nvar];
      _var  = new PolVar<T>[_nvar];
      _coef[0] = a1;
      _coef[1] = a2;
      _coef[2] = a3;
      _var[0] = X1;
      _var[1] = X2;
      _var[2] = X3;
    }
  //! @brief Constructor for cut w/ <a>n</a> participating variables
  PolCut
    ( FFOp*op, TYPE type, const double b, const unsigned n, const PolVar<T>*X,
      const double*a )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = n;
      _coef = new double[_nvar];
      _var  = new PolVar<T>[_nvar];
      for( unsigned ivar=0; ivar<_nvar; ivar++ ){
        _coef[ivar] = a[ivar]; _var[ivar] = X[ivar];
      }
    }
  //! @brief Constructor for cut w/ selection among <a>n</a> participating variables
  PolCut
    ( FFOp*op, TYPE type, const double b, const std::set<unsigned>&ndx,
      const PolVar<T>*X, const double*a )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = ndx.size();
      _coef = _nvar? new double[_nvar]: 0;
      _var  = _nvar? new PolVar<T>[_nvar]: 0;
      std::set<unsigned>::const_iterator it = ndx.begin();
      for( unsigned ivar=0; it!=ndx.end(); ++it, ivar++ ){
        _coef[ivar] = a[*it]; _var[ivar] = X[*it];
      }
    }
  //! @brief Constructor for cut w/ variable and coefficient maps
  template <typename U> PolCut
    ( FFOp*op, TYPE type, const double b, const std::map<U,PolVar<T>>&X,
      const std::map<U,double>&a )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = a.size();
      _coef = _nvar? new double[_nvar]: 0;
      _var  = _nvar? new PolVar<T>[_nvar]: 0;
      auto ita = a.cbegin();
      for( unsigned ivar=0; ita!=a.cend(); ++ita, ivar++ ){
        _coef[ivar] = ita->second;
        auto itX = X.find(ita->first);
        if( itX == X.cend() )
          throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::BADCUT );
        _var[ivar] = itX->second;
      }
    }
  //! @brief Constructor for cut w/ <a>n+1</a> participating variables
  PolCut
    ( FFOp*op, TYPE type, const double b, const unsigned n, const PolVar<T>*X,
      const double*a, const PolVar<T>&X1, const double a1 )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = n+1;
      _coef = new double[_nvar+1];
      _var  = new PolVar<T>[_nvar+1];
      for( unsigned ivar=0; ivar<n; ivar++ ){
        _coef[ivar] = a[ivar]; _var[ivar] = X[ivar];
      }
      _coef[n] = a1;
      _var[n] = X1;
    }
  //! @brief Constructor for cut w/ selection among <a>n</a> participating variables
  PolCut
    ( FFOp*op, TYPE type, const double b, const std::set<unsigned>&ndx,
      const PolVar<T>*X, const double*a, const PolVar<T>&X1, const double a1 )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = ndx.size()+1;
      _coef = new double[_nvar];
      _var  = new PolVar<T>[_nvar];
      std::set<unsigned>::const_iterator it = ndx.begin();
      for( unsigned ivar=0; it!=ndx.end(); ++it, ivar++ ){
        _coef[ivar] = a[*it]; _var[ivar] = X[*it];
      }
      _coef[_nvar-1] = a1;
      _var[_nvar-1] = X1;
    }
  //! @brief Constructor for cut w/ variable and coefficient maps
  template <typename U> PolCut
    ( FFOp*op, TYPE type, const double b, const std::map<U,PolVar<T>>&X,
      const std::map<U,double>&a, const PolVar<T>&X1, const double a1 )
    : _op(op), _type(type), _rhs(b)
    {
      _nvar = a.size()+1;
      _coef = new double[_nvar];
      _var  = new PolVar<T>[_nvar];
      auto ita = a.cbegin();
      for( unsigned ivar=0; ita!=a.cend(); ++ita, ivar++ ){
        _coef[ivar] = ita->second;
        auto itX = X.find(ita->first);
        if( itX == X.cend() )
          throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::BADCUT );
        _var[ivar] = itX->second;
      }
      _coef[_nvar-1] = a1;
      _var[_nvar-1] = X1;
    }
  //! @brief Destructor
  ~PolCut
    ()
    {
      delete[] _coef;
      delete[] _var;
    }
  /** @} */

private:
  //! @brief Private methods to block default compiler methods
  PolCut
    ();
};

template <class T> 
inline std::ostream&
operator <<
( std::ostream&out, const PolCut<T>&cut )
{
  const int iprec = 5;
  out << std::right << std::scientific << std::setprecision(iprec);
  
  switch( cut._type ){
    case PolCut<T>::EQ: case PolCut<T>::LE: case PolCut<T>::GE:
      for(unsigned k=0; k<cut.nvar(); k++ ){
        if( isequal( cut._coef[k], 0. ) )
          out << " + " << std::setw(iprec+6) << 0.;
        else if( cut._coef[k] > 0. )
          out << " + " << std::setw(iprec+6) << cut._coef[k];
        else
          out << " - " << std::setw(iprec+6) << -cut._coef[k];
        out << cut._var[k].name();  
      }
      break;

    case PolCut<T>::SOS1: case PolCut<T>::SOS2:
      out << " {";
      for(unsigned k=0; k<cut.nvar(); k++ )
        out << " " << cut._var[k].name();
      out << " }";
  }
  
  switch( cut._type ){
    case PolCut<T>::EQ: out << " = "; break;
    case PolCut<T>::LE: out << " <= "; break;
    case PolCut<T>::GE: out << " >= "; break;
    case PolCut<T>::SOS1: out << " SOS1"; return out;
    case PolCut<T>::SOS2: out << " SOS2"; return out;
  }
  
  out << std::setw(iprec+6) << cut._rhs;
  return out;
}

//! @brief C++ structure for ordering of polytopic cuts
template <class T> 
struct lt_PolCut
{
  bool operator()
    ( const PolCut<T>*Cut1, const PolCut<T>*Cut2 ) const
    {
      // Order cuts w.r.t. their types first
      if( Cut1->type() < Cut2->type() ) return true;
      // then w.r.t. their defining operation next
      return ( Cut1->op() && Cut2->op() ?
        lt_FFOp()( Cut1->op(), Cut2->op() ):
        false );
    }
};

//! @brief C++ structure for storing subintervals in the outer approximation of a univariate term
////////////////////////////////////////////////////////////////////////
//! mc::OAsub is a C++ structure for storing subintervals in the
//! outer approximation of a convex/concave portion of a univariate
//! function.
////////////////////////////////////////////////////////////////////////
class OAsub
////////////////////////////////////////////////////////////////////////
{

public:
  //! @brief Constructor
  OAsub( const double LB, const double UB, const double MID, 
    const double GAP ):
    _xL( LB ), _xU( UB ), _xM( MID ), _gap( GAP )
    {}
  //! @brief Retreive interval lower bound
  const double xL() const
    { return _xL; }
  //! @brief Retreive interval upper bound
  const double xU() const
    { return _xU; }
  //! @brief Retreive bisection point
  const double xM() const
    { return _xM; }
  //! @brief Retreive maximum gap
  const double gap() const
    { return _gap; }

private:
  //! @brief Interval lower bound
  double _xL;
  //! @brief Interval upper bound
  double _xU;
  //! @brief Bisection point
  double _xM;
  //! @brief Maximum gap
  double _gap;
};

//! @brief C++ structure for comparison of subintervals in the outer approximation of a univariate term
struct lt_OAsub
{
  bool operator()
    ( const OAsub&Dom1, const OAsub&Dom2 ) const
    {
      return( Dom1.gap() < Dom2.gap() );
    }
};

//! @brief C++ class for polytopic image evaluation
////////////////////////////////////////////////////////////////////////
//! mc::PolImg is a C++ class for evaluation of the polytoptic image of
//! a factorable function. Propagation of the image is via polytopic 
//! arithmetic, as implemented in mc::PolVar. The template parameter
//! corresponds to the type used to propagate variable range. Round-off
//! errors are not accounted for in the computations (non-verified
//! implementation).
////////////////////////////////////////////////////////////////////////
template< class T >
class PolImg
////////////////////////////////////////////////////////////////////////
{
  friend class PolVar<T>;
 
  template <typename U> friend std::ostream& operator<<( std::ostream&, const PolImg<U>& );

  template< class U > friend  PolVar<U> operator+( const PolVar<U>& );
  template< class U > friend  PolVar<U> operator+( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator+( const PolVar<U>&, const double );
  template< class U > friend  PolVar<U> operator+( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator+( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator+( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>&, const double );
  template< class U > friend  PolVar<U> operator-( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator-( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator-( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator*( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator*( const PolVar<U>&, const double );			
  template< class U > friend  PolVar<U> operator*( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator*( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator*( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator/( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator/( const PolVar<U>&, const double );			
  template< class U > friend  PolVar<U> operator/( const double, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator/( const PolVar<U>&, const U& );
  template< class U > friend  PolVar<U> operator/( const U&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator^( const PolVar<U>&, const PolVar<U>& );
  template< class U > friend  PolVar<U> operator^( const PolVar<U>&, const double );
  template< class U > friend  PolVar<U> operator^( const double, const PolVar<U>& );

  template< class U > friend  PolVar<U> inv ( const PolVar<U>& );
  template< class U > friend  PolVar<U> exp ( const PolVar<U>& );
  template< class U > friend  PolVar<U> log ( const PolVar<U>& );
  template< class U > friend  PolVar<U> sqrt( const PolVar<U>& );
  template< class U > friend  PolVar<U> sqr ( const PolVar<U>& );
  template< class U > friend  PolVar<U> pow ( const PolVar<U>&, const PolVar<U>& );  
  template< class U > friend  PolVar<U> cheb( const PolVar<U>&, const PolVar<U>& );  
  template< class U > friend  PolVar<U> cos ( const PolVar<U>& );
  template< class U > friend  PolVar<U> sin ( const PolVar<U>& );
  template< class U > friend  PolVar<U> tan ( const PolVar<U>& );
  template< class U > friend  PolVar<U> acos( const PolVar<U>& );
  template< class U > friend  PolVar<U> asin( const PolVar<U>& );
  template< class U > friend  PolVar<U> atan( const PolVar<U>& );
  template< class U > friend  PolVar<U> fabs( const PolVar<U>& );
  template< class U > friend  PolVar<U> fstep( const PolVar<U>& );

public:
  typedef std::map< FFVar*, PolVar<T>*, lt_FFVar > t_Vars;
  typedef std::list< PolVar<T>* > t_Aux;
  typedef std::list< PolLinEq<T>* > t_Lin;
  typedef std::map< const FFVar*, PolBilin<T>*, lt_FFVar > t_Bilin;
  typedef std::multiset< PolCut<T>*, lt_PolCut<T> > t_Cuts;
  typedef double (*p_Univ)( const double, const double*, const int* );
  typedef std::pair<double,double> (*p_dUniv)( const double, const double*, const int* );
  typedef void (PolImg<T>::*p_Cut)( FFOp*, const double, const PolVar<T>&, const double,
    const double, const PolVar<T>&, const double, const double, const double*, const int* );
  typedef void (PolImg<T>::*p_Cut2)( FFOp*, const PolVar<T>&, const double,
    const double, const PolVar<T>&, const double, const double, const double*, const int* );
  typedef std::priority_queue< OAsub, std::vector<OAsub>, lt_OAsub > t_OA;

protected:
  //! @brief Map of DAG variables in polytopic image
  t_Vars _Vars;
  //! @brief Appends new pointer to DAG variable map in polytopic image
  PolVar<T>* _append_var
    ( FFVar*var, const T&range, const bool cont );
  //! @brief Erase all entries in _Vars
  void _erase_vars
    ();
  //! @brief Reset cut-related field _subdiv in _Vars
  void _reset_vars
    ();

  //! @brief List of auxiliary variables in polytopic image
  t_Aux _Aux;
  //! @brief Appends new pointer to auxiliary variable set in polytopic image
  PolVar<T>* _append_aux
    ( const T&range, const bool cont );
  //! @brief Erase all entries in _Aux
  void _erase_aux
    ();

  //! @brief List of linear terms in polytopic image
  t_Lin _LinEq;
  //! @brief Appends new pointer to bilinear term map in polytopic image
  PolLinEq<T>* _append_lin
    ( const PolVar<T>*varR );
  //! @brief Erase all entries in _LinEq
  void _erase_lin
    ();

  //! @brief Map of bilinear terms in polytopic image
  t_Bilin _Bilin;
  //! @brief Appends new pointer to bilinear term map in polytopic image
  PolBilin<T>* _append_bilin
    ( const PolVar<T>*varR );
  //! @brief DC-decompose bilinear term in polytopic image
  void _decompose_bilin
    ( FFOp*op, PolBilin<T>* pBilin, const PolVar<T>&varR, const PolVar<T>&var1,
      const PolVar<T>&var2 );
  //! @brief Erase all entries in _Bilin
  void _erase_bilin
    ();

  //! @brief Set of cuts in polytopic image
  t_Cuts _Cuts;
  //! @brief Erase all entries in _Cuts
  void _erase_cuts
    ();
  //! @brief Erase all entries corresponding to operation <a>op</a> in _Cuts
  void _erase_cuts
    ( FFOp* op );
  //! @brief Appends new relaxation cut in _Cuts w/ 1 variable
  typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type,
      const double b, const PolVar<T>&X1, const double a1 );
  //! @brief Appends new relaxation cut in _Cuts w/ 2 variables
  typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type,
      const double b, const PolVar<T>&X1, const double a1,
      const PolVar<T>&X2, const double a2 );
  //! @brief Appends new relaxation cut in _Cuts w/ 3 variables
  typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type,
      const double b, const PolVar<T>&X1, const double a1,
      const PolVar<T>&X2, const double a2, const PolVar<T>&X3,
      const double a3 );
  //! @brief Appends new relaxation cut in _Cuts w/ <a>n</a> variables
  typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type, const double b,
      const unsigned n, const PolVar<T>*X, const double*a );
  //! @brief Appends new relaxation cut in _Cuts w/ <a>n</a> variables
  typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type, const double b,
      const std::set<unsigned>&ndx, const PolVar<T>*X, const double*a );
  //! @brief Appends new relaxation cut in _Cuts w/ variable and coefficient maps
  template <typename U> typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type, const double b,
      const std::map<U,PolVar<T>>&X, const std::map<U,double>&a );
  //! @brief Appends new relaxation cut in _Cuts w/ <a>n+1</a> variables
  typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type, const double b,
      const unsigned n, const PolVar<T>*X, const double*a,
      const PolVar<T>&X1, const double a1 );
  //! @brief Appends new relaxation cut in _Cuts w/ <a>n+1</a> variables
  typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type, const double b,
      const std::set<unsigned>&ndx, const PolVar<T>*X, const double*a,
      const PolVar<T>&X1, const double a1 );
  //! @brief Appends new relaxation cut in _Cuts w/ variable and coefficient maps
  template <typename U> typename t_Cuts::iterator _append_cut
    ( FFOp*op, const typename PolCut<T>::TYPE type, const double b,
      const std::map<U,PolVar<T>>&X, const std::map<U,double>&a,
      const PolVar<T>&X1, const double a1 );

  //! @brief Computes max distance between function and outer-approximation
  std::pair< double, double > _distmax
    ( p_dUniv f, const double xL, const double xU, const double*rpar=0,
      const int*ipar=0 ) const;
  //! @brief Computes solution of scalar nonlinear equation using the Newton method
  double _newton
    ( const double x0, const double xL, const double xU, p_dUniv f,
      const double TOL, const unsigned MAXIT, const double*rusr=0, const int*iusr=0 ) const;
  //! @brief Computes solution of scalar nonlinear equation using the secant method
  double _secant
    ( const double x0, const double x1, const double xL, const double xU, p_Univ f,
      const double TOL, const unsigned MAXIT, const double*rusr=0, const int*iusr=0 ) const;
  //! @brief Append cuts for nonlinear operation using outer-approximation (sandwich algorithm)
  void _sandwich_cuts
    ( FFOp*pOp, const PolVar<T>&X, const double XL, const double XU, const PolVar<T>&Y,
      const double YL, const double YU, const typename PolCut<T>::TYPE sense, p_dUniv f,
      const double*rpar=0, const int*ipar=0 );
  //! @brief Append linearization cut for nonlinear operation using outer-approximation
  void _linearization_cut
    ( FFOp*pOp, const double Xref, const PolVar<T>&X, const double XL, const double XU,
      const PolVar<T>&Y, const double YL, const double YU, const typename PolCut<T>::TYPE sense,
      p_dUniv f, const double*rpar, const int*ipar );

  //! @brief Append cuts for nonlinear operation using piecewise-linear approximation
  void _semilinear_cuts
    ( FFOp*pOp, const PolVar<T>&X, const double XL, const double XU, const PolVar<T>&Y,
      const typename PolCut<T>::TYPE sense, p_dUniv f, const double*rpar=0, const int*ipar=0 );
  //! @brief Form subintervals for semilinear cuts
  std::vector<double>* _semilinear_sub
    ( const PolVar<T>&X, const double XL, const double XU );

  //! @brief Propagate linear cut for binary + operation
  bool _lineq_PLUS
    ( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Propagate linear cut for unary - operation
  bool _lineq_NEG
    ( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Propagate linear cut for binary - operation
  bool _lineq_MINUS
    ( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Propagate linear cut for binary * operation
  bool _lineq_TIMES
    ( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Propagate linear cut for binary / operation
  bool _lineq_DIV
    ( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Propagate linear cut for fabs function
  bool _lineq_FABS
    ( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Propagate linear cut for fstep function
  bool _lineq_FSTEP
    ( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1 );

  //! @brief Append linear cuts for binary intersection
  void _append_cuts_INTER
    ( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Append linear cuts for binary + operation
  void _append_cuts_PLUS
    ( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Append linear cuts for unary - operation
  void _append_cuts_NEG
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for binary - operation
  void _append_cuts_MINUS
    ( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Append linear cuts for binary * operation
  void _append_cuts_TIMES
    ( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Append linear cuts for binary / operation
  void _append_cuts_DIV
    ( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Append linear cuts for pow function
  void _append_cuts_IPOW
    ( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Append linear cuts for cheb function
  void _append_cuts_CHEB
    ( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 );
  //! @brief Append linear cuts for exp function
  void _append_cuts_EXP
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for log function
  void _append_cuts_LOG
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for sqr function
  void _append_cuts_SQR
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for sqrt function
  void _append_cuts_SQRT
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for fabs function
  void _append_cuts_FABS
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for fstep function
  void _append_cuts_FSTEP
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for cos function
  void _append_cuts_COS
    ( const PolVar<T>*VarR, FFVar*pVar1 );
  //! @brief Append linear cuts for sin function
  void _append_cuts_SIN
    ( const PolVar<T>*VarR, FFVar*pVar1 );

public:
  /** @ingroup POLYTOPE
   *  @{
   */
  //! @brief Default Constructor
  PolImg
    ()
    {}

  //! @brief Destructor
  virtual ~PolImg
    ()
    { reset(); }

  //! @brief PolImg exceptions
  class Exceptions
  {
  public:
    //! @brief Enumeration type for exception handling
    enum TYPE{
      ROOT=1,        //!< Error during root search for obtaining the convex/concave envelope of a univariate term
      INTER,         //!< Error during intersection of two terms (terms do not intersect)
      DIV,           //!< Error during division operation (division by 0)
      ENVMIS=-1,     //!< Error due to an operation between variables participating in different polytopic images
      BADCUT=-2,     //!< Error due to an error during cut generation
      UNAVAIL=-3,    //!< Error due to calling a function/feature not yet implemented in MC++
      NOTALLOWED=-4, //!< Error due to calling a function/feature not yet implemented in MC++
      INTERN=-5	     //!< Internal error
    };
    //! @brief Constructor for error <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}
    //! @brief Inline function returning the error flag
    int ierr(){ return _ierr; }

  private:
    TYPE _ierr;
  };

  //! @brief PolImg options class
  struct Options
  {
    //! @brief Constructor
    Options():
      AGGREG_LIN(true), ROOT_USE(true), ROOT_MAXIT(100), ROOT_TOL(1e-10),
      SANDWICH_ATOL(1e-3), SANDWICH_RTOL(1e-3), SANDWICH_MAXCUT(5),
      SANDWICH_RULE(MAXERR), FRACTIONAL_ATOL(machprec()),
      FRACTIONAL_RTOL(machprec()), BREAKPOINT_TYPE(NONE),
      BREAKPOINT_ATOL(1e-8), BREAKPOINT_RTOL(1e-5), DCDECOMP_SCALE(false)
      {}
    //! @brief Assignment operator
    Options& operator= ( const Options&options ){
        AGGREG_LIN      = options.AGGREG_LIN;
        ROOT_USE        = options.ROOT_USE;
        ROOT_MAXIT      = options.ROOT_MAXIT;
        ROOT_TOL        = options.ROOT_TOL;
        SANDWICH_ATOL   = options.SANDWICH_ATOL;
        SANDWICH_RTOL   = options.SANDWICH_RTOL;
        SANDWICH_MAXCUT = options.SANDWICH_MAXCUT;
        SANDWICH_RULE   = options.SANDWICH_RULE;
        FRACTIONAL_ATOL = options.FRACTIONAL_ATOL;
        FRACTIONAL_RTOL = options.FRACTIONAL_RTOL;
        BREAKPOINT_TYPE     = options.BREAKPOINT_TYPE;
        BREAKPOINT_ATOL     = options.BREAKPOINT_ATOL;
        BREAKPOINT_RTOL     = options.BREAKPOINT_RTOL;
        DCDECOMP_SCALE      = options.DCDECOMP_SCALE;
        return *this;
      }
    //! @brief Enumeration type for sandwich strategy
    enum SANDWICH{
      BISECT=0,	//!< Range bisection
      MAXERR	//!< Maximum error rule
    };
    //! @brief Enumeration type for bilinear term relaxation strategy
    enum REFINE{
      NONE=0,	//!< No semi-linear cuts (use secant approximation)
      BIN,	//!< Semilinear cuts as linear binary reformulation
      SOS2	//!< Semilinear cuts as SOS2 reformulation
    };
    //! @brief Whether or not to aggregate linear expressions in cuts - Default: true
    bool AGGREG_LIN;
    //! @brief Whether or not to use root search to construct envelopes of univariate terms - Default: true
    bool ROOT_USE;
    //! @brief Maximal number of iterations in root search - Default: 100
    unsigned ROOT_MAXIT;
    //! @brief Termination tolerance in root search - Default: 1e-10
    double ROOT_TOL;
    //! @brief Absolute tolerance in outer-approximation of univariate terms - Default: 1e-3
    double SANDWICH_ATOL;
    //! @brief Relative tolerance in outer-approximation of univariate terms - Default: 1e-3
    double SANDWICH_RTOL;
    //! @brief Maximal number of cuts in outer approximation of univariate terms - Default: 5
    unsigned SANDWICH_MAXCUT;
    //! @brief Rule for outer-approximation of nonlinear convex/concave terms - Default: MAXERR
    SANDWICH SANDWICH_RULE;
    //! @brief Absolute tolerance in fractional terms to prevent division by zero - Default: EPS
    double FRACTIONAL_ATOL;
    //! @brief Relative tolerance in fractional terms to prevent division by zero - Default: EPS
    double FRACTIONAL_RTOL;
    //! @brief Rule for piecewise linear cuts of nonlinear convex/concave terms - Default: NONE
    REFINE BREAKPOINT_TYPE;
    //! @brief Absolute tolerance in adding breakpoints in piecewise linear cuts - Default: 1e-8
    double BREAKPOINT_ATOL;
    //! @brief Relative tolerance in adding breakpoints in piecewise linear cuts - Default: 1e-5
    double BREAKPOINT_RTOL;
    //! @brief Whether or not to scale variables in DC decomposition of bilinear/fractional terms - Default: false
    bool DCDECOMP_SCALE;
  };

  //! @brief PolImg options handle
  Options options;

  //! @brief Retreive reference to set of DAG variables in polytopic image
  t_Vars& Vars()
    { return _Vars; }

  //! @brief Retreive const reference to set of DAG variables in polytopic image
  const t_Vars& Vars() const
    { return _Vars; }
  
  //! @brief Retreive reference to set of auxiliary variables in polytopic image
  t_Aux& Aux()
    { return _Aux; }

  //! @brief Retreive reference to set of bilinear terms in polytopic image
  t_Bilin& Bilin()
    { return _Bilin; }

  //! @brief Retreive const reference to set of bilinear terms in polytopic image
  const t_Bilin& Bilin() const
    { return _Bilin; }

  //! @brief Retreive reference to set of cuts in polytopic image
  t_Cuts& Cuts()
    { return _Cuts; }

  //! @brief Reset polytopic image (all participating variables and cuts)
  void reset()
    { _erase_cuts(); _erase_vars(); _erase_aux(); _erase_lin(); _erase_bilin(); }

  //! @brief Append relaxation cuts for the <a>ndep</a> dependents in <a>pdep</a>
  void generate_cuts
    ( const unsigned ndep, const PolVar<T>*pdep, const bool reset=false );
  //! @brief Append relaxation cuts for the dependents in <a>pdep</a> indexed by <a>ndxdep</a>
  void generate_cuts
    ( const std::set<unsigned>&ndxdep, const PolVar<T>*pdep, const bool reset=false );
  //! @brief Append relaxation cuts for the dependents in the map <a>mdep</a>
  template <typename U> void generate_cuts
    ( const std::map<U,PolVar<T>>&mdep, const bool reset=false );

  //! @brief Append new relaxation cut w/ 1 variable
  typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const PolVar<T>&X1, const double a1 )
    { return _append_cut( 0, type, b, X1, a1 ); }
  //! @brief Append new relaxation cut w/ 2 variables
  typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const PolVar<T>&X1, const double a1,
      const PolVar<T>&X2, const double a2 )
    { return _append_cut( 0, type, b, X1, a1, X2, a2 ); }
  //! @brief Append new relaxation cut w/ 3 variables
  typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const PolVar<T>&X1, const double a1,
      const PolVar<T>&X2, const double a2,
      const PolVar<T>&X3, const double a3 )
    { return _append_cut( 0, type, b, X1, a1, X2, a2, X3, a3 ); }
  //! @brief Append new relaxation cut w/ <a>n</a> variables
  typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const unsigned n, const PolVar<T>*X, const double*a )
    { return _append_cut( 0, type, b, n, X, a ); }
  //! @brief Append new relaxation cut w/ selection amongst <a>n</a> variables
  typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const std::set<unsigned>&ndx, const PolVar<T>*X, const double*a )
    { return _append_cut( 0, type, b, ndx, X, a ); }
  //! @brief Append new relaxation cut w/ variable and coefficient maps
  template <typename U> typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const std::map<U,PolVar<T>>&X, const std::map<U,double>&a )
    { return _append_cut( 0, type, b, X, a ); }
  //! @brief Append new relaxation cut w/ <a>n+1</a> variables
  typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const unsigned n, const PolVar<T>*X, const double*a,
      const PolVar<T>&X1, const double a1 )
    { return _append_cut( 0, type, b, n, X, a, X1, a1 ); }
  //! @brief Append new relaxation cut w/ selection amongst <a>n+1</a> variables
  typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const std::set<unsigned>&ndx, const PolVar<T>*X, const double*a,
      const PolVar<T>&X1, const double a1 )
    { return _append_cut( 0, type, b, ndx, X, a, X1, a1 ); }
  //! @brief Append new relaxation cut w/ variable and coefficient maps
  template <typename U> typename t_Cuts::iterator add_cut
    ( const typename PolCut<T>::TYPE type, const double b,
      const std::map<U,PolVar<T>>&X, const std::map<U,double>&a,
      const PolVar<T>&X1, const double a1 )
    { return _append_cut( 0, type, b, X, a, X1, a1 ); }

  //! @brief Erase cut with iterator <a>itcut</a> from set of cuts
  void erase_cut
    ( typename t_Cuts::iterator itcut )
    { return _Cuts.erase( itcut ); }
  //! @brief Erase all cuts and auxiliary variables
  void reset_cuts
    ()
    { _reset_vars(); _erase_aux(); _erase_cuts(); }

/*
  //! @brief Add constraint to factorable program
  PolCut<T>* add_constraint
    ( const PolVar<T>&lhsVar, const typename PolImg<T>::CTRTYPE type,
      const PolVar<T>&rhsVar );
  //! @brief Remove constraint from factorable program
  virtual bool remove_constraint
    ( FPOp<T>* constr )
    { return _erase_operation( constr ); }
*/
};

template <typename T>
inline PolVar<T>*
PolImg<T>::_append_var
( FFVar*var, const T&range, const bool cont )
{
  auto itVar = _Vars.find( var );
  if( itVar != _Vars.end() ){
    itVar->second->_update( range, cont );
    return itVar->second;
  }
  PolVar<T>* pVar = new PolVar<T>;
  pVar->_set( this, *var, range, cont, _Vars.size() );
  _Vars.insert( std::make_pair( var, pVar ) );
  return pVar;
}

template <typename T> inline void
PolImg<T>::_erase_vars
()
{
  for( auto itv = _Vars.begin(); itv != _Vars.end(); ++itv )
    delete itv->second;
  _Vars.clear();
}

template <typename T> inline void
PolImg<T>::_reset_vars
()
{
  for( auto itv = _Vars.begin(); itv != _Vars.end(); ++itv ){
    itv->second->reset_subdiv();
    itv->second->reset_cuts();
  }
}

template <typename T>
inline PolVar<T>*
PolImg<T>::_append_aux
( const T&range, const bool cont )
{
  PolVar<T>* pAux = new PolVar<T>;
  pAux->_set( this, range, cont, _Aux.size() );
  _Aux.push_back( pAux );
  return pAux;
}

template <typename T> inline void
PolImg<T>::_erase_aux
()
{
  for( auto itv = _Aux.begin(); itv != _Aux.end(); ++itv )
    delete *itv;
  _Aux.clear();
}

template <typename T>
inline PolLinEq<T>*
PolImg<T>::_append_lin
( const PolVar<T>*varR )
{
  PolLinEq<T>* pLin = new PolLinEq<T>( varR );
  _LinEq.push_back( pLin );
  return pLin;
}

template <typename T>
inline void
PolImg<T>::_erase_lin
()
{
  for( auto itl = _LinEq.begin(); itl != _LinEq.end(); ++itl )
    delete *itl;
  _LinEq.clear();
}

template <typename T>
inline PolBilin<T>*
PolImg<T>::_append_bilin
( const PolVar<T>*varR )
{
  // Determine if bilinear term already exists
  auto itb = _Bilin.find( &varR->var() );
  if( itb != _Bilin.end() ){
    //itVar->second->_update( range, cont );
    return itb->second;
  }

  // Append bilinear or fractional term
  //const PolVar<T>*pvarR = _Vars.find( const_cast<FFVar*>(&varR->var()) )->second;
  //PolBilin<T>* pBilin = new PolBilin<T>( pvarR );
  PolBilin<T>* pBilin = new PolBilin<T>( varR );
  _Bilin.insert( std::make_pair( &varR->var(), pBilin ) );
  return pBilin;
}

template <typename T>
inline void
PolImg<T>::_decompose_bilin
( FFOp*op, PolBilin<T>* pBilin, const PolVar<T>&varR, const PolVar<T>&var1,
  const PolVar<T>&var2 )
{
  if( options.BREAKPOINT_TYPE == options.NONE ) return;

  // Add cuts for DC decomposition
  FFGraph* dag = varR._var.dag();

  if( options.DCDECOMP_SCALE ){
    double s1 = Op<T>::diam(var2.range());
    //double s1 = options.DCDECOMP_SCALE? 1./Op<T>::diam(var1.range()): 1.;
    if( pBilin->scal1 ){
      pBilin->scal1->_var.num() = s1;
      pBilin->scal1->_range = s1;
    }
    else{ 
      FFVar scal1( dag, s1 );
      FFVar* pscal1 = *dag->Vars().find(&scal1);
      PolVar<T> polscal1( this, *pscal1 );
      pBilin->scal1 = _Vars.find( &scal1 )->second;
    }

    double s2 = Op<T>::diam(var1.range());
    //double s2 = options.DCDECOMP_SCALE? 1./Op<T>::diam(var2.range()): 1.;
    if( pBilin->scal2 ){
      pBilin->scal2->_var.num() = s2;
      pBilin->scal2->_range = s2;
    }
    else{ 
      FFVar scal2( dag, s2 );
      FFVar* pscal2 = *dag->Vars().find(&scal2);
      PolVar<T> polscal2( this, *pscal2 );
      pBilin->scal2 = _Vars.find( &scal2 )->second;
    }

    FFVar scalvar1( pBilin->scal1->_var * var1._var );
    dag->curOp() = scalvar1.ops().first;
    PolVar<T> polscalvar1( *pBilin->scal1 * var1 );

    FFVar scalvar2( pBilin->scal2->_var * var2._var );
    dag->curOp() = scalvar2.ops().first;
    PolVar<T> polscalvar2( *pBilin->scal2 * var2 );

    FFVar sumvar( scalvar1 + scalvar2 );
    dag->curOp() = sumvar.ops().first;
    PolVar<T> polsumvar( polscalvar1 + polscalvar2 );

    FFVar subvar( scalvar1 - scalvar2 );
    dag->curOp() = subvar.ops().first;
    PolVar<T> polsubvar( polscalvar1 - polscalvar2 );

    FFVar sqrvar1( sqr( sumvar ) );
    dag->curOp() = sqrvar1.ops().first;
    PolVar<T> polsqrvar1( sqr( polsumvar ) );

    FFVar sqrvar2( sqr( subvar ) );
    dag->curOp() = sqrvar2.ops().first;
    PolVar<T> polsqrvar2( sqr( polsubvar ) );

    _append_cut( op, PolCut<T>::EQ, 0., varR, 4.*s1*s2, polsqrvar1, -1., polsqrvar2, 1. );
  }

  else{
    FFVar sumvar( var1._var + var2._var );
    dag->curOp() = sumvar.ops().first;
    PolVar<T> polsumvar( var1 + var2 );

    FFVar subvar( var1._var - var2._var );
    dag->curOp() = subvar.ops().first;
    PolVar<T> polsubvar( var1 - var2 );

    FFVar sqrvar1( sqr( sumvar ) );
    dag->curOp() = sqrvar1.ops().first;
    PolVar<T> polsqrvar1( sqr( polsumvar ) );

    FFVar sqrvar2( sqr( subvar ) );
    dag->curOp() = sqrvar2.ops().first;
    PolVar<T> polsqrvar2( sqr( polsubvar ) );

    _append_cut( op, PolCut<T>::EQ, 0., varR, 4., polsqrvar1, -1., polsqrvar2, 1. );
  }
  return;
}

template <typename T> inline void
PolImg<T>::_erase_bilin
()
{
  for( auto itv = _Bilin.begin(); itv != _Bilin.end(); ++itv )
    delete itv->second;
  _Bilin.clear();
}

template <typename T> inline void
PolImg<T>::_erase_cuts
()
{
  for( auto itc = _Cuts.begin(); itc != _Cuts.end(); ++itc )
    delete *itc;
  _Cuts.clear();
}
 
template <typename T>
inline void
PolImg<T>::_erase_cuts
( FFOp* op )
{
  auto itc = _Cuts.begin();
  while( itc != _Cuts.end() ){
    auto itp = itc; ++itc;
    if( (*itp)->op() == op ){ delete *itp; _Cuts.erase( itp ); }
  } 
}

template <typename T>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const PolVar<T>&X1, const double a1 )
{
  PolCut<T>* pCut = new PolCut<T>( op, type, b, X1, a1 );
  return _Cuts.insert( pCut );
}

template <typename T>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const PolVar<T>&X1, const double a1,
  const PolVar<T>&X2, const double a2 )
{
  PolCut<T>* pCut = new PolCut<T>( op, type, b, X1, a1, X2, a2 );
  return _Cuts.insert( pCut );
}

template <typename T>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const PolVar<T>&X1, const double a1,
  const PolVar<T>&X2, const double a2, const PolVar<T>&X3,
  const double a3 )
{
  PolCut<T>* pCut = new PolCut<T>( op, type, b, X1, a1, X2, a2, X3, a3 );
  return _Cuts.insert( pCut );
}

template <typename T>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const unsigned n,
  const PolVar<T>*X, const double*a )
{
  //if( !n ) throw Exceptions( Exceptions::INTERNAL );
  PolCut<T>* pCut = new PolCut<T>( op, type, b, n, X, a );
  return _Cuts.insert( pCut );
}

template <typename T>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const std::set<unsigned>&ndx,
  const PolVar<T>*X, const double*a )
{
  PolCut<T>* pCut = new PolCut<T>( op, type, b, ndx, X, a );
  return _Cuts.insert( pCut );
}

template <typename T> template <typename U>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const std::map<U,PolVar<T>>&X,
  const std::map<U,double>&a )
{
  PolCut<T>* pCut = new PolCut<T>( op, type, b, X, a );
  return _Cuts.insert( pCut );
}

template <typename T>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const unsigned n,
  const PolVar<T>*X, const double*a,
  const PolVar<T>&X1, const double a1 )
{
  //if( !n ) throw Exceptions( Exceptions::INTERNAL );
  PolCut<T>* pCut = new PolCut<T>( op, type, b, n, X, a, X1, a1 );
  return _Cuts.insert( pCut );
}

template <typename T>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const std::set<unsigned>&ndx,
  const PolVar<T>*X, const double*a,
  const PolVar<T>&X1, const double a1 )
{
  PolCut<T>* pCut = new PolCut<T>( op, type, b, ndx, X, a, X1, a1 );
  return _Cuts.insert( pCut );
}

template <typename T> template <typename U>
inline typename PolImg<T>::t_Cuts::iterator
PolImg<T>::_append_cut
( FFOp*op, const typename PolCut<T>::TYPE type,
  const double b, const std::map<U,PolVar<T>>&X,
  const std::map<U,double>&a, const PolVar<T>&X1,
  const double a1 )
{
  PolCut<T>* pCut = new PolCut<T>( op, type, b, X, a, X1, a1 );
  return _Cuts.insert( pCut );
}

template <typename T> inline void
PolImg<T>::generate_cuts
( const unsigned ndep, const PolVar<T>*pdep, const bool reset )
{
  // Reset cuts in polyhedral image?
  if( reset ) reset_cuts();

  // Propagate cuts through all dependent subtrees
  for( unsigned i=0; i<ndep; i++ ){
    auto itDep = _Vars.find( const_cast<FFVar*>(&pdep[i]._var) );
    if( itDep != _Vars.end() ) itDep->second->generate_cuts();
  }
}

template <typename T> inline void
PolImg<T>::generate_cuts
( const std::set<unsigned>&ndxdep, const PolVar<T>*pdep, const bool reset )
{
  // Reset cuts in polyhedral image?
  if( reset ) reset_cuts();

  // Propagate cuts through all dependent subtrees
  auto it = ndxdep.cbegin();
  for( ; it != ndxdep.cend(); ++it ){
    auto itDep = _Vars.find( const_cast<FFVar*>(&pdep[*it]._var) );
    if( itDep != _Vars.end() ) itDep->second->generate_cuts();
  }
}

template <typename T> template <typename U>
inline void
PolImg<T>::generate_cuts
( const std::map<U,PolVar<T>>&mdep, const bool reset )
{
  // Reset cuts in polyhedral image?
  if( reset ) reset_cuts();

  // Propagate cuts through all dependent subtrees
  auto it = mdep.cbegin();
  for( ; it != mdep.cend(); ++it ){
    auto itDep = _Vars.find( const_cast<FFVar*>(&it->second._var) );
    if( itDep != _Vars.end() ) itDep->second->generate_cuts();
  }
}

template <typename T>
inline std::ostream&
operator <<
( std::ostream&out, const PolImg<T>&P )
{
  out << ( P._Vars.empty()? "\nNO VARIABLE\n": "\nVARIABLES:\n" );
  for( auto itv=P._Vars.begin(); itv!=P._Vars.end(); ++itv ){
    out << "  " << itv->second->name() << "\t in " << itv->second->range()
        << "\t (DAG: " << itv->second->var() << ")";
    //if( (*itv)->_Op ) out << "\t:= " << *(*itv)->_Op;
    out << std::endl;
  }
  out << ( P._Aux.empty()? "\nNO AUXILIARY\n": "\nAUXILIARIES:\n" );
  for( auto itv=P._Aux.begin(); itv!=P._Aux.end(); ++itv ){
    out << "  " << (*itv)->name() << "\t in " << (*itv)->range();
    //if( (*itv)->_Op ) out << "\t" << *(*itv)->_Op;
    out << std::endl;
  }

  out << ( P._Bilin.empty()? "\nNO BILINEAR OR FRACTIONAL TERM": "\nBILINEAR AND FRACTIONAL TERMS:\n" );
  for( auto itv=P._Bilin.begin(); itv!=P._Bilin.end(); ++itv ){
    out << "  " << itv->second->var->name();
  }
  out << std::endl;

  out << ( P._Cuts.empty()? "\nNO CUT\n": "\nCUTS:\n" );
  for( auto itc=P._Cuts.begin(); itc!=P._Cuts.end(); ++itc )
    out << " " << **itc << std::endl;
  return out;
}

template <typename T>
inline PolVar<T>
operator^
( const PolVar<T>&Var1, const PolVar<T>&Var2 )
{
  if( Var1._img && Var2._img && Var1._img != Var2._img )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::ENVMIS );
  PolImg<T>* img = Var1._img? Var1._img: Var2._img;
  FFGraph* dag = Var1._var.cst()? Var2._var.dag(): Var1._var.dag();
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
  FFVar* pFFVarR = dag->curOp()->pres;
  T IVarR;
  if( !Op<T>::inter( IVarR, Var1._range, Var2._range ) )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::INTER );
  PolVar<T>* pVarR = img->_append_var( pFFVarR, IVarR, true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_INTER
( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar1->num().val(), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar1->second, -1. );
  }
  if( pVar2->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar2->num().val(), *VarR, 1. );
  else{
    auto itVar2 = _Vars.find( pVar2 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar2->second, -1. );
  }
}

template <typename T>
inline PolVar<T>
operator+
( const PolVar<T>&Var1, const PolVar<T>&Var2 )
{
  if( Var1._img && Var2._img && Var1._img != Var2._img )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::ENVMIS );
  PolImg<T>* img = Var1._img? Var1._img: Var2._img;
  FFGraph* dag = Var1._var.cst()? Var2._var.dag(): Var1._var.dag();
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = img->_append_var( pFFVarR, Var1.range() + Var2.range(), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_PLUS
( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( pVar1->cst() && pVar2->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar1->num().val()+pVar2->num().val(), *VarR, 1. );
  else if( pVar2->cst() ){
    auto itVar1 = _Vars.find( pVar1 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar2->num().val(), *VarR, 1., *itVar1->second, -1. );
  }
  else if( pVar1->cst() ){
    auto itVar2 = _Vars.find( pVar2 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar1->num().val(), *VarR, 1., *itVar2->second, -1. );
  }
  else{
    auto itVar1 = _Vars.find( pVar1 );
    auto itVar2 = _Vars.find( pVar2 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar1->second, -1., *itVar2->second, -1. );
  }
}

template <typename T> inline bool
PolImg<T>::_lineq_PLUS
( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( !pLin ) pLin = _append_lin( VarR );
  if( pVar1->cst() && pVar2->cst() )
    pLin->substitute( VarR, pVar1->num().val()+pVar2->num().val() );
  else if( pVar2->cst() ){
    auto itVar1 = _Vars.find( pVar1 );
    pLin->substitute( VarR, 1., itVar1->second, pVar2->num().val() );
  }
  else if( pVar1->cst() ){
    auto itVar2 = _Vars.find( pVar2 );
    pLin->substitute( VarR, 1., itVar2->second, pVar1->num().val() );
  }
  else{
    auto itVar1 = _Vars.find( pVar1 );
    auto itVar2 = _Vars.find( pVar2 );
    pLin->substitute( VarR, 1., itVar1->second, 1., itVar2->second );
  }
  return true;
}

template <typename T>
inline PolVar<T>
operator-
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
  PolImg<T>* img = Var1._img;
  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = img->_append_var( pFFVarR, -Var1.range(), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_NEG
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, -pVar1->num().val(), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar1->second, 1. );
  }
}

template <typename T> inline bool
PolImg<T>::_lineq_NEG
( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( !pLin ) pLin = _append_lin( VarR );
  if( pVar1->cst() )
    pLin->substitute( VarR, -pVar1->num().val() );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    pLin->substitute( VarR, -1., itVar1->second );
  }
  return true;
}

template <typename T>
inline PolVar<T>
operator-
( const PolVar<T>&Var1, const PolVar<T>&Var2 )
{
  if( Var1._img && Var2._img && Var1._img != Var2._img )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::ENVMIS );
  PolImg<T>* img = Var1._img? Var1._img: Var2._img;
  FFGraph* dag = Var1._var.cst()? Var2._var.dag(): Var1._var.dag();
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = img->_append_var( pFFVarR, Var1.range() - Var2.range(), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_MINUS
( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( pVar1->cst() && pVar2->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar1->num().val()-pVar2->num().val(), *VarR, 1. );
  else if( pVar2->cst() ){
    auto itVar1 = _Vars.find( pVar1 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, -pVar2->num().val(), *VarR, 1., *itVar1->second, -1. );
  }
  else if( pVar1->cst() ){
    auto itVar2 = _Vars.find( pVar2 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar1->num().val(), *VarR, 1., *itVar2->second, 1. );
  }
  else{
    auto itVar1 = _Vars.find( pVar1 );
    auto itVar2 = _Vars.find( pVar2 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar1->second, -1., *itVar2->second, 1. );
  }
}

template <typename T> inline bool
PolImg<T>::_lineq_MINUS
( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( !pLin ) pLin = _append_lin( VarR );
  if( pVar1->cst() && pVar2->cst() )
    pLin->substitute( VarR, pVar1->num().val()-pVar2->num().val() );
  else if( pVar2->cst() ){
    auto itVar1 = _Vars.find( pVar1 );
    pLin->substitute( VarR, 1., itVar1->second, -pVar2->num().val() );
  }
  else if( pVar1->cst() ){
    auto itVar2 = _Vars.find( pVar2 );
    pLin->substitute( VarR, -1., itVar2->second, pVar1->num().val() );
  }
  else{
    auto itVar1 = _Vars.find( pVar1 );
    auto itVar2 = _Vars.find( pVar2 );
    pLin->substitute( VarR, 1., itVar1->second, -1., itVar2->second );
  }
  return true;
}

template <typename T>
inline PolVar<T>
operator*
( const PolVar<T>&Var1, const PolVar<T>&Var2 )
{
  if( Var1._img && Var2._img && Var1._img != Var2._img )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::ENVMIS );
  PolImg<T>* img = Var1._img? Var1._img: Var2._img;
  FFGraph* dag = Var1._var.cst()? Var2._var.dag(): Var1._var.dag();
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = img->_append_var( pFFVarR, Var1.range() * Var2.range(), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_TIMES
( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( pVar1->cst() && pVar2->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar1->num().val() * pVar2->num().val(), *VarR, 1. );

  else if( pVar2->cst() ){
    auto itVar1 = _Vars.find( pVar1 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar1->second, -pVar2->num().val() );
  }

  else if( pVar1->cst() ){
    auto itVar2 = _Vars.find( pVar2 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar2->second, -pVar1->num().val() );
  }

  else{
    auto itVar1 = _Vars.find( pVar1 );
    auto itVar2 = _Vars.find( pVar2 );

    _append_cut( VarR->_var.ops().first, PolCut<T>::GE, -Op<T>::u(itVar1->second->_range)*Op<T>::u(itVar2->second->_range),
      *VarR, 1., *itVar1->second, -Op<T>::u(itVar2->second->_range), *itVar2->second, -Op<T>::u(itVar1->second->_range) );
    _append_cut( VarR->_var.ops().first, PolCut<T>::GE, -Op<T>::l(itVar1->second->_range)*Op<T>::l(itVar2->second->_range),
      *VarR, 1., *itVar1->second, -Op<T>::l(itVar2->second->_range), *itVar2->second, -Op<T>::l(itVar1->second->_range) );
    _append_cut( VarR->_var.ops().first, PolCut<T>::LE, -Op<T>::u(itVar1->second->_range)*Op<T>::l(itVar2->second->_range),
      *VarR, 1., *itVar1->second, -Op<T>::l(itVar2->second->_range), *itVar2->second, -Op<T>::u(itVar1->second->_range) );
    _append_cut( VarR->_var.ops().first, PolCut<T>::LE, -Op<T>::l(itVar1->second->_range)*Op<T>::u(itVar2->second->_range),
      *VarR, 1., *itVar1->second, -Op<T>::u(itVar2->second->_range), *itVar2->second, -Op<T>::l(itVar1->second->_range) );

    PolBilin<T>* pBilin = _append_bilin( VarR );
    _decompose_bilin( VarR->_var.ops().first, pBilin, *VarR, *itVar1->second, *itVar2->second );
  }
}

template <typename T> inline bool
PolImg<T>::_lineq_TIMES
( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( !pVar1->cst() && !pVar2->cst() ) return false;
  if( !pLin ) pLin = _append_lin( VarR );
  if( pVar1->cst() && pVar2->cst() )
    pLin->substitute( VarR, pVar1->num().val()*pVar2->num().val() );
  else if( pVar2->cst() ){
    auto itVar1 = _Vars.find( pVar1 );
    pLin->substitute( VarR, pVar2->num().val(), itVar1->second );
  }
  else{
    auto itVar2 = _Vars.find( pVar2 );
    pLin->substitute( VarR, pVar1->num().val(), itVar2->second );
  }
  return true;
}

template <typename T>
inline PolVar<T>
operator/
( const PolVar<T>&Var1, const PolVar<T>&Var2 )
{
  if( Var1._img && Var2._img && Var1._img != Var2._img )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::ENVMIS );
  PolImg<T>* img = Var1._img? Var1._img: Var2._img;
  FFGraph* dag = Var1._var.cst()? Var2._var.dag(): Var1._var.dag();
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = img->_append_var( pFFVarR, Var1.range() / Var2.range(), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_DIV
( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( pVar1->cst() && pVar2->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, pVar1->num().val(), *VarR, pVar2->num().val() );

  else if( pVar2->cst() ){
    auto itVar1 = _Vars.find( pVar1 );
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, pVar2->num().val(), *itVar1->second, -1. );
  }

  else if( pVar1->cst() ){
    auto itVar2 = _Vars.find( pVar2 );
    double Cst1 = pVar1->num().val();
    struct loc{
      static std::pair<double,double> scalinv
        ( const double x, const double*rusr, const int*iusr )
        { return std::make_pair( *rusr/x, -*rusr/(x*x) ); }
    };
    // -- Convex Case
    if( (pVar1->num().val() >= 0. && Op<T>::l(itVar2->second->_range) > 0.)
     || (pVar1->num().val() <= 0. && Op<T>::u(itVar2->second->_range) < 0.) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar2->second, Op<T>::l(itVar2->second->_range),
        Op<T>::u(itVar2->second->_range), *VarR, PolCut<T>::LE, loc::scalinv, &Cst1, 0 );
      _sandwich_cuts( VarR->_var.ops().first, *itVar2->second, Op<T>::l(itVar2->second->_range),
        Op<T>::u(itVar2->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
        PolCut<T>::GE, loc::scalinv, &Cst1, 0 );
    }
    // -- Concave Case
    else{
      _semilinear_cuts( VarR->_var.ops().first, *itVar2->second, Op<T>::l(itVar2->second->_range),
        Op<T>::u(itVar2->second->_range), *VarR, PolCut<T>::GE, loc::scalinv, &Cst1, 0 );
      _sandwich_cuts( VarR->_var.ops().first, *itVar2->second, Op<T>::l(itVar2->second->_range),
        Op<T>::u(itVar2->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
        PolCut<T>::LE, loc::scalinv, &Cst1, 0 );
    }
  }

  else{
    auto itVar1 = _Vars.find( pVar1 );
    auto itVar2 = _Vars.find( pVar2 );

    _append_cut( VarR->_var.ops().first, PolCut<T>::GE, -Op<T>::u(VarR->_range)*Op<T>::u(itVar2->second->_range),
      *itVar1->second, 1., *VarR, -Op<T>::u(itVar2->second->_range), *itVar2->second, -Op<T>::u(VarR->_range) );
    _append_cut( VarR->_var.ops().first, PolCut<T>::GE, -Op<T>::l(VarR->_range)*Op<T>::l(itVar2->second->_range),
      *itVar1->second, 1., *VarR, -Op<T>::l(itVar2->second->_range), *itVar2->second, -Op<T>::l(VarR->_range) );
    _append_cut( VarR->_var.ops().first, PolCut<T>::LE, -Op<T>::u(VarR->_range)*Op<T>::l(itVar2->second->_range),
      *itVar1->second, 1., *VarR, -Op<T>::l(itVar2->second->_range), *itVar2->second, -Op<T>::u(VarR->_range) );
    _append_cut( VarR->_var.ops().first, PolCut<T>::LE, -Op<T>::l(VarR->_range)*Op<T>::u(itVar2->second->_range),
      *itVar1->second, 1., *VarR, -Op<T>::u(itVar2->second->_range), *itVar2->second, -Op<T>::l(VarR->_range) );

    PolBilin<T>* pBilin = _append_bilin( VarR );
    _decompose_bilin( VarR->_var.ops().first, pBilin, *itVar1->second, *VarR, *itVar2->second );
  }
}

template <typename T> inline bool
PolImg<T>::_lineq_DIV
( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  if( !pVar2->cst() ) return false;
  if( !pLin ) pLin = _append_lin( VarR );
  if( pVar1->cst() )
    pLin->substitute( VarR, pVar1->num().val()/pVar2->num().val() );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    pLin->substitute( VarR, 1./pVar2->num().val(), itVar1->second );
  }
  return true;
}

template <typename T> inline std::pair< double, double >
PolImg<T>::_distmax
( p_dUniv f, const double xL, const double xU, const double*rpar,
  const int*ipar ) const
{
  std::pair<double,double> fL = f(xL,rpar,ipar), fU = f(xU,rpar,ipar);
  double Ddf = fU.second-fL.second,
         Adf = std::max(std::fabs(fL.second),std::fabs(fU.second));

  double xmid = 0.5 * ( xL + xU );
  double xmax = ( std::fabs(Ddf) - Adf*options.FRACTIONAL_RTOL > options.FRACTIONAL_ATOL?
    ( fU.second * xU - fL.second * xL - fU.first + fL.first ) / Ddf: xmid );
  std::pair<double,double> fmax = f(xmax,rpar,ipar);
  double dmax = std::fabs( fmax.first - fL.second * ( xmax - xL ) - fL.first );

  switch( options.SANDWICH_RULE ){
  case Options::BISECT:
    return std::make_pair( xmid, dmax );
  case Options::MAXERR: default:
    return std::make_pair( xmax, dmax );
  }
}

template <typename T>
inline void
PolImg<T>::_sandwich_cuts
( FFOp*pOp, const PolVar<T>&X, const double XL, const double XU, const PolVar<T>&Y,
  const double YL, const double YU, const typename PolCut<T>::TYPE sense, p_dUniv f,
  const double*rpar, const int*ipar )
{
  t_OA OA;
  unsigned NBCUTS = 0;

  switch( options.BREAKPOINT_TYPE ){
    // OA cuts @xL,xU + @breakpoints
    case Options::BIN:
    case Options::SOS2:{
      const std::vector<double>& subint = X.create_subdiv( XL, XU );
      const unsigned nsubint = subint.size();
      if( nsubint > 2 ){
        for( ; NBCUTS<nsubint; NBCUTS++ ){
          _linearization_cut( pOp, subint[NBCUTS], X, XL, XU, Y, YL, YU, sense, f, rpar, ipar );
          if( !NBCUTS ) continue;
          std::pair<double,double> worst = _distmax( f, subint[NBCUTS-1], subint[NBCUTS], rpar, ipar );
          OA.push( OAsub( subint[NBCUTS-1], subint[NBCUTS], worst.first, worst.second ) );
#ifdef MC__POLIMG_DEBUG_SANDWICH
          std::cerr << OA.top() << std::endl;
#endif
        }
        break;
      }
    }

    // OA cuts @xL,xU only
    case Options::NONE: default:{
      _linearization_cut( pOp, XL, X, XL, XU, Y, YL, YU, sense, f, rpar, ipar );
      if( mc::isequal( XL, XU ) ) return;
      _linearization_cut( pOp, XU, X, XL, XU, Y, YL, YU, sense, f, rpar, ipar );
      NBCUTS += 2;
      std::pair<double,double> worst = _distmax( f, XL, XU, rpar, ipar );
      OA.push( OAsub( XL, XU, worst.first, worst.second ) );
#ifdef MC__POLIMG_DEBUG_SANDWICH
      std::cerr << OA.top() << std::endl;
#endif
      break;
    }
  }

  const double dtol = options.SANDWICH_ATOL
    + options.SANDWICH_RTOL*std::max(std::fabs(YL),std::fabs(YU));
#ifdef MC__POLIMG_DEBUG_SANDWICH
  std::cerr << "DTOL: " << dtol << std::endl;
  std::cerr << OA.top() << std::endl;
#endif

  // OA cut @xM
  while( OA.top().gap() > dtol && NBCUTS < options.SANDWICH_MAXCUT ){
    // x - y/exp(xref) <= xref - 1, @xref=xmax
    _linearization_cut( pOp, OA.top().xM(), X, OA.top().xL(), OA.top().xU(),
      Y, YL, YU, sense, f, rpar, ipar );
    NBCUTS++;
    std::pair<double,double> worst = _distmax( f, OA.top().xL(), OA.top().xM(), rpar, ipar );
    OA.push( OAsub( OA.top().xL(), OA.top().xM(), worst.first, worst.second ) );
#ifdef MC__POLIMG_DEBUG_SANDWICH
    std::cerr << "  Pushed: "  << FPOuter( OA.top().xL(), OA.top().xM(),
      worst.first, worst.second ) << std::endl;
#endif
    worst = _distmax( f, OA.top().xM(), OA.top().xU(), rpar, ipar );
    OA.push( OAsub( OA.top().xM(), OA.top().xU(), worst.first, worst.second ) );
#ifdef MC__POLIMG_DEBUG_SANDWICH
    std::cerr << "  Pushed: " << FPOuter( OA.top().xM(), OA.top().xU(),
      worst.first, worst.second ) << std::endl;
#endif
    OA.pop();
#ifdef MC__POLIMG_DEBUG_SANDWICH
    std::cerr << OA.top() << std::endl;
#endif
  }
#ifdef MC__POLIMG_DEBUG_SANDWICH
  std::cerr << "NBCUTS: " << NBCUTS << std::endl;
#endif
}

template <typename T>
inline void
PolImg<T>::_linearization_cut
( FFOp*pOp, const double Xref, const PolVar<T>&X, const double XL, const double XU,
  const PolVar<T>&Y, const double YL, const double YU, const typename PolCut<T>::TYPE sense,
  p_dUniv f, const double*rpar, const int*ipar )
{
  // sense: f convex -> PolCut<T>::GE - f concave -> PolCut<T>::LE
  // Y - f'(Xref)·X >= f(Xref) - f'(Xref)·Xref
  const std::pair<double,double> Fref = f(Xref,rpar,ipar);
  _append_cut( pOp, sense, Fref.first-Xref*Fref.second, Y, 1., X, -Fref.second );
}

template <typename T> inline double
PolImg<T>::_newton
( const double x0, const double xL, const double xU, p_dUniv f,
  const double TOL, const unsigned MAXIT, const double*rusr, const int*iusr ) const
{
  double xk = std::max(xL,std::min(xU,x0)), dk;
  std::pair<double,double> fk = f(xk,rusr,iusr);
  
  for( unsigned int it=0; it<MAXIT; it++ ){
    if( std::fabs(fk.first) < TOL ) return xk;
    if( fk.second == 0 ) throw Exceptions( Exceptions::ROOT );
    dk = fk.first/fk.second;
    if( mc::isequal(xk,xL) && dk>0 ) return xk;
    if( mc::isequal(xk,xU) && dk<0 ) return xk;
    xk = std::max(xL,std::min(xU,xk-dk));
    fk = f(xk,rusr,iusr);
  }

  throw Exceptions( Exceptions::ROOT );
}

template <typename T> inline double
PolImg<T>::_secant
( const double x0, const double x1, const double xL, const double xU, p_Univ f,
  const double TOL, const unsigned MAXIT, const double*rusr, const int*iusr ) const
{
  double xkm = std::max(xL,std::min(xU,x0));
  double fkm = f(xkm,rusr,iusr);
  double xk = std::max(xL,std::min(xU,x1));
  
  for( unsigned int it=0; it<MAXIT; it++ ){
    double fk = f(xk,rusr,iusr);
    if( std::fabs(fk) < TOL ) return xk;
    double Bk = (fk-fkm)/(xk-xkm);
    if( Bk == 0 ) throw Exceptions( Exceptions::ROOT );
    if( isequal(xk,xL) && fk/Bk>0 ) return xk;
    if( isequal(xk,xU) && fk/Bk<0 ) return xk;
    xkm = xk;
    fkm = fk;
    xk = std::max(xL,std::min(xU,xk-fk/Bk));
  }

  throw Exceptions( Exceptions::ROOT );
}

template <typename T>
inline void
PolImg<T>::_semilinear_cuts
( FFOp*pOp, const PolVar<T>&X, const double XL, const double XU, const PolVar<T>&Y,
  const typename PolCut<T>::TYPE sense, p_dUniv f, const double*rpar, const int*ipar )
{
  switch( options.BREAKPOINT_TYPE ){
   case Options::BIN:{
    const std::vector<double>& subint = X.create_subdiv( XL, XU );
    const unsigned nsubint = subint.size()-1;
    if( nsubint > 1 ){
      // Represent variable range using linear binary transformation
      const std::vector< PolVar<T> >& subvar = X.BIN_subdiv( pOp );
      // Append semilinear cuts
      double coef[nsubint];
      double rhs = f( subint[0], rpar, ipar ).first;
      for( unsigned isub=0; isub<nsubint; isub++ )
        coef[isub] = f( subint[isub], rpar, ipar ).first - f( subint[isub+1], rpar, ipar ).first;
      _append_cut( pOp, sense, rhs, nsubint, subvar.data(), coef, Y, 1. );
    }
    break;
   }
   case Options::SOS2:{
    const std::vector<double>& subint = X.create_subdiv( XL, XU );
    const unsigned nsubint = subint.size();
    if( nsubint > 2 ){
      // Represent variable range using SOS2 transformation
      const std::vector< PolVar<T> >& subvar = X.SOS2_subdiv( pOp );
      // Append semilinear cuts
      double coef[nsubint];
      for( unsigned isub=0; isub<nsubint; isub++ )
        coef[isub] = -f( subint[isub], rpar, ipar ).first;
      _append_cut( pOp, sense, 0., nsubint, subvar.data(), coef, Y, 1. );
    }
    break;
   }
   default:
    break;
  }
  double dX = XU-XL, YL = f(XL,rpar,ipar).first, dY = f(XU,rpar,ipar).first-YL;
  _append_cut( pOp, sense, dX*YL-dY*XL, Y, dX, X, -dY );
}

template <typename T>
inline PolVar<T>
inv
( const PolVar<T>&Var1 )
{
  return pow( Var1, PolVar<T>(-1) );
}

template <typename T>
inline PolVar<T>
exp
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::exp( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_EXP
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, std::exp( pVar1->num().val() ), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    struct loc{ static std::pair<double,double> exp
      ( const double x, const double*rusr, const int*iusr )
      { return std::make_pair( std::exp(x), std::exp(x) ); }
    };
    _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
      Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::LE, loc::exp );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
      Op<T>::u(itVar1->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
      PolCut<T>::GE, loc::exp );
  }
}

template <typename T>
inline PolVar<T>
log
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::log( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_LOG
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, std::log( pVar1->num().val() ), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    struct loc{
      static std::pair<double,double> exp
        ( const double x, const double*rusr, const int*iusr )
        { return std::make_pair( std::exp(x), std::exp(x) ); }
      static std::pair<double,double> log
        ( const double x, const double*rusr, const int*iusr )
        { return std::make_pair( std::log(x), 1/x ); }
    };
    _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
      Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::GE, loc::log );
    _sandwich_cuts( VarR->_var.ops().first, *VarR, Op<T>::l(VarR->_range),
      Op<T>::u(VarR->_range), *itVar1->second, Op<T>::l(itVar1->second->_range), Op<T>::u(itVar1->second->_range),
      PolCut<T>::GE, loc::exp );
  }
}

template <typename T>
inline PolVar<T>
sqr
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::sqr( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_SQR
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, mc::sqr( pVar1->num().val() ), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    struct loc{ static std::pair<double,double> sqr
      ( const double x, const double*rusr, const int*iusr )
      { return std::make_pair( x*x, 2.*x ); }
    };
    _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
      Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::LE, loc::sqr );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
      Op<T>::u(itVar1->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
      PolCut<T>::GE, loc::sqr );
  }
}

template <typename T>
inline PolVar<T>
sqrt
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::sqrt( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_SQRT
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() ){
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, std::sqrt( pVar1->num().val() ), *VarR, 1. );
    return;
  }

  auto itVar1 = _Vars.find( pVar1 );
  struct loc{ 
    static std::pair<double,double> sqr
      ( const double x, const double*rusr, const int*iusr )
      { return std::make_pair( x*x, 2.*x ); }
    static std::pair<double,double> sqrt
      ( const double x, const double*rusr, const int*iusr )
      { return std::make_pair( std::sqrt(x), 1/(2*std::sqrt(x)) ); }
  };
  _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
    Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::GE, loc::sqrt );
  _sandwich_cuts( VarR->_var.ops().first, *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
    *itVar1->second, Op<T>::l(itVar1->second->_range), Op<T>::u(itVar1->second->_range), PolCut<T>::GE, loc::sqr );
}

template <typename T>
inline PolVar<T>
cos
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::cos( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_COS
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() ){
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, std::cos( pVar1->num().val() ), *VarR, 1. );
    return;
  }

  auto itVar1 = _Vars.find( pVar1 );
  struct loc{
    static std::pair<double,double> cos
      ( const double x, const double*rusr, const int*iusr )
      { return std::make_pair( std::cos(x), -std::sin(x) ); }
  };
  struct fct{ static std::pair<double,double> cosfunc
    ( const double x, const double*rusr, const int*iusr )
    { return std::make_pair(
        (x-*rusr)*std::sin(x)+std::cos(x)-std::cos(*rusr),
        (x-*rusr)*std::cos(x) ); }
  };

  // Convex relaxation
  T IVar1 = itVar1->second->_range;
  int kL = std::ceil( -0.5*(1.+Op<T>::l(IVar1)/PI) ); // offset for xL to be in [-pi,pi]
  double dxL = 2.*PI*kL; 
  double xL1 = Op<T>::l(IVar1)+dxL, xU1 = Op<T>::u(IVar1)+dxL;
  assert( xL1 >= -PI && xL1 <= PI );
 
  if( xU1 >= PI ){
    const int kU = std::ceil( -0.5*(1.+Op<T>::u(IVar1)/PI) );
    const double dxU = 2.*PI*kU; 
    const double xU2 = Op<T>::u(IVar1)+dxU;
    assert( xU2 >= -PI && xU2 <= PI );

    double xJcv1 = Op<T>::l(IVar1);
    if( xL1 <= PI/2. ) xJcv1 = _newton( PI-dxL, Op<T>::l(IVar1), PI-dxL, fct::cosfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv1, 0 );
    double xJcv2 = Op<T>::u(IVar1);
    if( xU2 >= -PI/2. ) xJcv2 = _newton( -PI-dxU, -PI-dxU, Op<T>::u(IVar1), fct::cosfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv2, 0 );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcv1, PI-dxL,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cos );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, -PI-dxU, xJcv2,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cos );
  }
  
  else if( xL1 >= PI/2. ){
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cos );
  }
  
  else if( xL1 >= -PI/2. && xU1 <= PI/2. ){
    _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, PolCut<T>::GE, loc::cos );
  }

  else if( xL1 >= -PI/2. ){
    double xJcv1 = Op<T>::l(IVar1);
    xJcv1 = _newton( Op<T>::u(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::cosfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv1, 0 );
    if( mc::isequal( xJcv1, Op<T>::u(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::GE, loc::cos );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcv1, Op<T>::u(IVar1), 
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cos );
    }
  }

  else{
    double xJcv1 = Op<T>::u(IVar1);
    xJcv1 = _newton( Op<T>::l(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::cosfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv1, 0 );
    if( mc::isequal( xJcv1, Op<T>::l(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::GE, loc::cos );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), xJcv1,
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cos );
    }
  }

  // Concave relaxation
  kL = std::ceil( -0.5*(2.+Op<T>::l(IVar1)/PI) );
  dxL = 2.*PI*kL;
  xL1 = Op<T>::l(IVar1)+dxL, xU1 = Op<T>::u(IVar1)+dxL;
  assert( xL1 >= -2.*PI && xL1 <= 0. );

  if( xU1 >= 0. ){
    const int kU = std::ceil( -0.5*(2.+Op<T>::u(IVar1)/PI) );
    const double dxU = 2.*PI*kU; 
    const double xU2 = Op<T>::u(IVar1)+dxU;
    assert( xU2 >= -2.*PI && xU2 <= 0. );

    double xJcc1 = Op<T>::l(IVar1);
    if( xL1 <= -PI/2. ) xJcc1 = _newton( -dxL, Op<T>::l(IVar1), -dxL, fct::cosfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcc1, 0 );
    double xJcc2 = Op<T>::u(IVar1);
    if( xU2 >= -3.*PI/2. ) xJcc2 = _newton( -2.*PI-dxU, -2.*PI-dxU, Op<T>::u(IVar1),
      fct::cosfunc, options.ROOT_TOL, options.ROOT_MAXIT, &xJcc2, 0 );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcc1, -dxL,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::cos );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, -2.*PI-dxU, xJcc2,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::cos );
  }

  else if( xL1 >= -PI/2. ){
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::cos );
  }
  
  else if( xL1 >= -3.*PI/2. && xU1 <= -PI/2. ){
    _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, PolCut<T>::LE, loc::cos );
  }

  else if( xL1 >= -3.*PI/2. ){
    double xJcc1 = Op<T>::l(IVar1);
    xJcc1 = _newton( Op<T>::u(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::cosfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcc1, 0 );
    if( mc::isequal( xJcc1, Op<T>::u(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::LE, loc::cos );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcc1, Op<T>::u(IVar1), 
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::cos );
    }
  }

  else{
    double xJcc1 = Op<T>::u(IVar1);
    xJcc1 = _newton( Op<T>::l(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::cosfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcc1, 0 );
    if( mc::isequal( xJcc1, Op<T>::l(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::LE, loc::cos );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), xJcc1,
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::cos );
    }
  }
}

template <typename T>
inline PolVar<T>
sin
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::sin( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_SIN
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() ){
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, std::sin( pVar1->num().val() ), *VarR, 1. );
    return;
  }

  auto itVar1 = _Vars.find( pVar1 );
  struct loc{
    static std::pair<double,double> sin
      ( const double x, const double*rusr, const int*iusr )
      { return std::make_pair( std::sin(x), std::cos(x) ); }
  };
  struct fct{ static std::pair<double,double> sinfunc
    ( const double x, const double*rusr, const int*iusr )
    { return std::make_pair(
        (*rusr-x)*std::cos(x)+std::sin(x)-std::sin(*rusr),
        (x-*rusr)*std::sin(x) ); }
  };

  // Convex relaxation
  T IVar1 = itVar1->second->_range;
  int kL = std::ceil( -0.5*(0.5+Op<T>::l(IVar1)/PI) ); // offset for xL to be in [-pi/2,3pi/2]
  double dxL = 2.*PI*kL; 
  double xL1 = Op<T>::l(IVar1)+dxL, xU1 = Op<T>::u(IVar1)+dxL;
  assert( xL1 >= -PI/2. && xL1 <= 3.*PI/2. );

  if( xU1 >= 3.*PI/2. ){
    const int kU = std::ceil( -0.5*(0.5+Op<T>::u(IVar1)/PI) );
    const double dxU = 2.*PI*kU; 
    const double xU2 = Op<T>::u(IVar1)+dxU;
    assert( xU2 >= -PI/2. && xU2 <= 3.*PI/2. );

    double xJcv1 = Op<T>::l(IVar1);
    if( xL1 <= PI ) xJcv1 = _newton( 3.*PI/2.-dxL, Op<T>::l(IVar1), 3.*PI/2.-dxL, fct::sinfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv1, 0 );
    double xJcv2 = Op<T>::u(IVar1);
    if( xU2 >= 0. ) xJcv2 = _newton( -PI/2.-dxU, -PI/2.-dxU, Op<T>::u(IVar1), fct::sinfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv2, 0 );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcv1, 3.*PI/2.-dxL,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::sin );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, -PI/2.-dxU, xJcv2,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::sin );
  }
 
  else if( xL1 >= PI ){
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::sin );
  }
  
  else if( xL1 >= 0. && xU1 <= PI ){
    _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, PolCut<T>::GE, loc::sin );
  }

  else if( xL1 >= 0. ){
    double xJcv1 = Op<T>::l(IVar1);
    xJcv1 = _newton( Op<T>::u(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::sinfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv1, 0 );
    if( mc::isequal( xJcv1, Op<T>::u(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::GE, loc::sin );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcv1, Op<T>::u(IVar1), 
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::sin );
    }
  }

  else{
    double xJcv1 = Op<T>::u(IVar1);
    xJcv1 = _newton( Op<T>::l(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::sinfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcv1, 0 );
    if( mc::isequal( xJcv1, Op<T>::l(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::GE, loc::sin );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), xJcv1,
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::sin );
    }
  }

  // Concave relaxation
  kL = std::ceil( -0.5*(1.5+Op<T>::l(IVar1)/PI) );
  dxL = 2.*PI*kL;
  xL1 = Op<T>::l(IVar1)+dxL, xU1 = Op<T>::u(IVar1)+dxL;
  assert( xL1 >= -3.*PI/2. && xL1 <= PI/2. );

  if( xU1 >= PI/2. ){
    const int kU = std::ceil( -0.5*(1.5+Op<T>::u(IVar1)/PI) );
    const double dxU = 2.*PI*kU; 
    const double xU2 = Op<T>::u(IVar1)+dxU;
    assert( xU2 >= -3.*PI/2. && xU2 <= PI/2. );

    double xJcc1 = Op<T>::l(IVar1);
    if( xL1 <= 0. ) xJcc1 = _newton( PI/2.-dxL, Op<T>::l(IVar1), PI/2.-dxL, fct::sinfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcc1, 0 );
    double xJcc2 = Op<T>::u(IVar1);
    if( xU2 >= -PI ) xJcc2 = _newton( -3.*PI/2.-dxU, -3.*PI/2.-dxU, Op<T>::u(IVar1),
      fct::sinfunc, options.ROOT_TOL, options.ROOT_MAXIT, &xJcc2, 0 );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcc1, PI/2.-dxL,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::sin );
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, -3.*PI/2.-dxU, xJcc2,
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::sin );
  }

  else if( xL1 >= 0. ){
    _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::sin );
  }
  
  else if( xL1 >= -PI && xU1 <= 0. ){
    _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
      *VarR, PolCut<T>::LE, loc::sin );
  }

  else if( xL1 >= -PI ){
    double xJcc1 = Op<T>::l(IVar1);
    xJcc1 = _newton( Op<T>::u(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::sinfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcc1, 0 );
    if( mc::isequal( xJcc1, Op<T>::u(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::LE, loc::sin );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcc1, Op<T>::u(IVar1), 
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::sin );
    }
  }

  else{
    double xJcc1 = Op<T>::u(IVar1);
    xJcc1 = _newton( Op<T>::l(IVar1), Op<T>::l(IVar1), Op<T>::u(IVar1), fct::sinfunc,
      options.ROOT_TOL, options.ROOT_MAXIT, &xJcc1, 0 );
    if( mc::isequal( xJcc1, Op<T>::l(IVar1) ) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), Op<T>::u(IVar1),
        *VarR, PolCut<T>::LE, loc::sin );
    }
    else{
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(IVar1), xJcc1,
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::sin );
    }
  }
}

template <typename T>
inline PolVar<T>
pow
( const PolVar<T>&Var1, const PolVar<T>&Var2 )
{
#ifdef MC__POLIMG_CHECK
  if( !Var2_.var.cst() && Var2_.var.num().t != FFNum::INT )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif
  const int iExp = Var2._var.num().n;
#ifdef MC__POLIMG_CHECK
  if( iExp == 0 || iExp == 1 || iExp == 2 )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::pow( Var1._range, iExp ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_IPOW
( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  const int iExp = pVar2->num().n;
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, std::pow( pVar1->num().val(), iExp ), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    struct loc{
      static std::pair<double,double> pow
        ( const double x, const double*rusr, const int*iusr )
        { return std::make_pair( std::pow(x,*iusr), *iusr*std::pow(x,*iusr-1) ); }
      static std::pair<double,double> powcvu
        ( const double x, const double*rusr, const int*iusr )
        { return std::make_pair( std::pow(x,*iusr), *iusr*std::pow(x,*iusr-1) ); }
    };
    // Positive even exponent term
    if( iExp > 0 && !(iExp%2) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
        Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::LE, loc::pow, 0, &iExp );
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
        Op<T>::u(itVar1->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
        PolCut<T>::GE, loc::pow, 0, &iExp );
    }

    // Positive odd exponent term
    else if( iExp > 0 && options.ROOT_USE ){
      // -- Convex Portion
      if( Op<T>::l(itVar1->second->_range) >= 0. ){
        _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::LE, loc::pow, 0, &iExp );
        _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
          PolCut<T>::GE, loc::pow, 0, &iExp );
      }
      // -- Concave Portion
      else if( Op<T>::u(itVar1->second->_range) <= 0. ){
        _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::GE, loc::pow, 0, &iExp );
        _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
          PolCut<T>::LE, loc::pow, 0, &iExp );
      }
      // -- Nonconvex/Nonconcave Portion
      else{
        switch( options.BREAKPOINT_TYPE ){
          case PolImg<T>::Options::BIN:
          case PolImg<T>::Options::SOS2:{
            const unsigned nsubint = itVar1->second->create_subdiv( Op<T>::l(itVar1->second->_range),
              Op<T>::u(itVar1->second->_range) ).size();
            if( nsubint > 2 ){
              struct dc{
                static std::pair<double,double> pow1
                  ( const double x, const double*rusr, const int*iusr )
                  { return std::make_pair( x<0?std::pow(x,*iusr):0., x<0?*iusr*std::pow(x,*iusr-1):0. ); }
                static std::pair<double,double> pow2
                  ( const double x, const double*rusr, const int*iusr )
                  { return std::make_pair( x>0?std::pow(x,*iusr):0., x>0?*iusr*std::pow(x,*iusr-1):0. ); }
              };
              PolVar<T>* Var3 = _append_aux( Op<T>::pow( itVar1->second->_range, iExp ), true );
              PolVar<T>* Var4 = _append_aux( Op<T>::pow( itVar1->second->_range, iExp ), true );
              _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
                Op<T>::u(itVar1->second->_range), *Var3, PolCut<T>::GE, dc::pow1, 0, &iExp );
              _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
                Op<T>::u(itVar1->second->_range), *Var3, Op<T>::l(VarR->_range), 0., PolCut<T>::LE, dc::pow1, 0, &iExp );
              _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
                Op<T>::u(itVar1->second->_range), *Var4, PolCut<T>::LE, dc::pow2, 0, &iExp );
              _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
                Op<T>::u(itVar1->second->_range), *Var4, 0., Op<T>::u(VarR->_range), PolCut<T>::GE, dc::pow2, 0, &iExp );
              _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *Var3, -1., *Var4, -1. );
              break;
            }
            // No break in order to append other "normal" cuts
          }
          case PolImg<T>::Options::NONE: default:{
            struct fct{ static std::pair<double,double> powoddfunc
              ( const double x, const double*rusr, const int*iusr )
              { return std::make_pair(
                  ((*iusr-1)*x-(*iusr)*(*rusr))*std::pow(x,*iusr-1) + std::pow(*rusr,*iusr),
                  (*iusr)*(*iusr-1)*(x-(*rusr))*std::pow(x,*iusr-2) ); }
            };
            double xJcc = Op<T>::u(itVar1->second->_range);
            xJcc = _newton( Op<T>::l(itVar1->second->_range), Op<T>::l(itVar1->second->_range), 0.,
              fct::powoddfunc, options.ROOT_TOL, options.ROOT_MAXIT, &xJcc, &iExp );
            if( mc::isequal( xJcc, Op<T>::l(itVar1->second->_range) ) )
              _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
                Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::LE, loc::pow, 0, &iExp );
            else
              _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range), xJcc,
                *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::pow, 0, &iExp );

            double xJcv = Op<T>::l(itVar1->second->_range);
            xJcv = _newton( Op<T>::u(itVar1->second->_range), 0., Op<T>::u(itVar1->second->_range),
              fct::powoddfunc, options.ROOT_TOL, options.ROOT_MAXIT, &xJcv, &iExp );
            if( mc::isequal( xJcv, Op<T>::u(itVar1->second->_range) ) )
              _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
                Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::GE, loc::pow, 0, &iExp );
            else
              _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJcv, Op<T>::u(itVar1->second->_range),
                *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::pow, 0, &iExp );
            break;
          }
        }
      }
    }

    // Negative exponent term
    else if( iExp < 0 ){
      // -- Convex Case
      if( !(iExp%2) || Op<T>::l(itVar1->second->_range) > 0. ){
        _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::LE, loc::pow, 0, &iExp );
        _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
          PolCut<T>::GE, loc::pow, 0, &iExp );
      }
      // -- Concave Case
      else{
        _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::GE, loc::pow, 0, &iExp );
        _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
          Op<T>::u(itVar1->second->_range), *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range),
          PolCut<T>::LE, loc::pow, 0, &iExp );
      }
    }
  }
}

template <typename T>
inline PolVar<T>
cheb
( const PolVar<T>&Var1, const PolVar<T>&Var2 )
{
#ifdef MC__POLIMG_CHECK
  if( !Var2_.var.cst() && Var2_.var.num().t != FFNum::INT )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif
  const int iOrd = Var2._var.num().n;
#ifdef MC__POLIMG_CHECK
  if( iOrd <= 2 )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
#ifdef MC__POLIMG_DEBUG_CHEB
  std::cout << "X = " << Var1._range << std::endl;
  std::cout << "Cheb(" << Var1._range << "," << iOrd << ") = " << Op<T>::cheb( Var1._range, iOrd ) << std::endl;
#endif
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::cheb( Var1._range, iOrd ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_CHEB
( const PolVar<T>*VarR, FFVar*pVar1, FFVar*pVar2 )
{
  const int iOrd = pVar2->num().n;
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, mc::cheb( pVar1->num().val(), iOrd ), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    struct loc{
      static std::pair<double,double> cheb
        ( const double x, const double*rusr, const int*iusr )
        { return std::make_pair( mc::cheb(x,*iusr), *iusr*mc::cheb2(x,*iusr-1) ); }
    };
    // Positive even order
    if( iOrd > 0 && !(iOrd%2) ){
      _semilinear_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range),
        Op<T>::u(itVar1->second->_range), *VarR, PolCut<T>::LE, loc::cheb, 0, &iOrd );
      double xJL = std::cos(mc::PI*(1.+1./(double)iOrd));
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range), xJL,
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cheb, 0, &iOrd );
      double xJU = std::cos(mc::PI/(double)iOrd);
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJU, Op<T>::u(itVar1->second->_range),
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cheb, 0, &iOrd );
    }

    // Positive odd order
    else{
      double xJL = std::cos(mc::PI*(1.+1./(double)iOrd));
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, Op<T>::l(itVar1->second->_range), xJL,
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::LE, loc::cheb, 0, &iOrd );
      double xJU = std::cos(mc::PI/(double)iOrd);
      _sandwich_cuts( VarR->_var.ops().first, *itVar1->second, xJU, Op<T>::u(itVar1->second->_range),
        *VarR, Op<T>::l(VarR->_range), Op<T>::u(VarR->_range), PolCut<T>::GE, loc::cheb, 0, &iOrd );
    }
  }
}

template <typename T>
inline PolVar<T>
fabs
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::fabs( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_FABS
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, std::fabs( pVar1->num().val() ), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    if( Op<T>::l(itVar1->second->_range) >= 0. )
      _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar1->second, -1. );
    else if( Op<T>::u(itVar1->second->_range) <= 0. )
      _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, 0., *VarR, 1., *itVar1->second,  1. );
    else{
      double XL = Op<T>::l(itVar1->second->_range),  dX = Op<T>::diam(itVar1->second->_range),
             YL = std::fabs(Op<T>::l(itVar1->second->_range)), dY = std::fabs(Op<T>::u(itVar1->second->_range))-YL;
      _append_cut( VarR->_var.ops().first, PolCut<T>::GE, dY*XL-dX*YL, *VarR, -dX, *itVar1->second, dY );
      _append_cut( VarR->_var.ops().first, PolCut<T>::GE, 0., *VarR, 1., *itVar1->second,  -1. );
      _append_cut( VarR->_var.ops().first, PolCut<T>::GE, 0., *VarR, 1., *itVar1->second,   1. );
    }
  }
}

template <typename T> inline bool
PolImg<T>::_lineq_FABS
( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1 )
{
  auto itVar1 = _Vars.find( pVar1 );
  if( !pVar1->cst() || Op<T>::l(itVar1->second->_range)*Op<T>::u(itVar1->second->_range) < 0. ) return false;
  if( !pLin ) pLin = _append_lin( VarR );
  if( pVar1->cst() )
    pLin->substitute( VarR, std::fabs(pVar1->num().val()) );
  else{
    if( Op<T>::l(itVar1->second->_range) >= 0 )
      pLin->substitute( VarR, 1., itVar1->second );
    else
      pLin->substitute( VarR, -1., itVar1->second );
  }
  return true;
}

template <typename T>
inline PolVar<T>
fstep
( const PolVar<T>&Var1 )
{
  FFGraph* dag = Var1._var.dag();
#ifdef MC__POLIMG_CHECK
  if( !dag || !dag->curOp() )
    throw typename PolImg<T>::Exceptions( PolImg<T>::Exceptions::NOTALLOWED );
#endif

  FFVar* pFFVarR = dag->curOp()->pres;
  PolVar<T>* pVarR = Var1._img->_append_var( pFFVarR, Op<T>::fstep( Var1._range ), true );
  return *pVarR;
}

template <typename T> inline void
PolImg<T>::_append_cuts_FSTEP
( const PolVar<T>*VarR, FFVar*pVar1 )
{
  if( pVar1->cst() )
    _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, mc::fstep( pVar1->num().val() ), *VarR, 1. );
  else{
    auto itVar1 = _Vars.find( pVar1 );
    if( Op<T>::l(itVar1->second->_range) >= 0. )
      _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, mc::fstep( Op<T>::l(itVar1->second->_range) ), *VarR, 1. );
    else if( Op<T>::u(itVar1->second->_range) < 0. )
      _append_cut( VarR->_var.ops().first, PolCut<T>::EQ, mc::fstep( Op<T>::u(itVar1->second->_range) ), *VarR, 1. );
    else{
      _append_cut( VarR->_var.ops().first, PolCut<T>::GE, 0., *VarR, Op<T>::u(itVar1->second->_range),
                   *itVar1->second,  -1. );
      _append_cut( VarR->_var.ops().first, PolCut<T>::GE, Op<T>::l(itVar1->second->_range),
                   *VarR, Op<T>::l(itVar1->second->_range), *itVar1->second,  1. );
    }
  }
}

template <typename T> inline bool
PolImg<T>::_lineq_FSTEP
( PolLinEq<T>*&pLin, const PolVar<T>*VarR, FFVar*pVar1 )
{
  auto itVar1 = _Vars.find( pVar1 );
  if( !pVar1->cst() || (Op<T>::l(itVar1->second->_range) < 0. && Op<T>::u(itVar1->second->_range) >= 0.) ) return false;
  if( !pLin ) pLin = _append_lin( VarR );
  if( pVar1->cst() )
    pLin->substitute( VarR, mc::fstep(pVar1->num().val()) );
  else
    pLin->substitute( VarR, mc::fstep(Op<T>::l(itVar1->second->_range)) );
  return true;
}

} // namespace mc

#include "mcop.hpp"

namespace mc
{

//! @brief Specialization of the mc::Op templated structure for use of type mc::PolVar as template type in other MC++ classes
template< typename T > struct Op< mc::PolVar<T> >
{
  typedef mc::PolVar<T> PV;
  static PV point( const double c ) { return PV(c); }
  static PV zeroone(){ throw std::runtime_error("operation not permitted"); }// { return PV( mc::Op<T>::zeroone() ); }
  static void I(PV& x, const PV&y) { x = y; }
  static double l(const PV& x) { return mc::Op<T>::l(x.range()); }
  static double u(const PV& x) { return mc::Op<T>::u(x.range()); }
  static double abs (const PV& x) { return mc::Op<T>::abs(x.range());  }
  static double mid (const PV& x) { return mc::Op<T>::mid(x.range());  }
  static double diam(const PV& x) { return mc::Op<T>::diam(x.range()); }
  static PV inv (const PV& x){ return mc::inv(x);  }
  static PV sqr (const PV& x){ return mc::sqr(x);  }
  static PV sqrt(const PV& x){ return mc::sqrt(x); }
  static PV log (const PV& x){ return mc::log(x);  }
  static PV xlog(const PV& x){ return x*mc::log(x); }
  static PV fabs(const PV& x){ return mc::fabs(x);  }
  static PV exp (const PV& x){ return mc::exp(x);  }
  static PV cos (const PV& x){ return mc::cos(x);  }
  static PV sin (const PV& x){ return mc::sin(x);  }
  static PV tan (const PV& x)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::tan(x);  }
  static PV asin(const PV& x)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::asin(x); }
  static PV acos(const PV& x)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::acos(x); }
  static PV atan(const PV& x)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::atan(x); }
  static PV erf (const PV& x)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::erf(x);  }
  static PV erfc(const PV& x)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::erfc(x); }
  static PV fstep(const PV& x){ return mc::fstep(x); }
  static PV bstep(const PV& x)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::bstep(x); }
  static PV min (const PV& x, const PV& y)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::min(x,y);  }
  static PV max (const PV& x, const PV& y)
    { throw std::runtime_error("operation not permitted"); }
//  { return mc::max(x,y);  }
  static PV arh (const PV& x, const double k){ return mc::exp(-k/x); }
  static PV pow (const PV& x, const PV& y){ return mc::pow(x,y); }
  static PV cheb(const PV& x, const PV& y){ return mc::cheb(x,y); }
  static PV hull(const PV& x, const PV& y){ return mc::Op<T>::hull(x.range(),y.range()); }
  static bool inter(PV& xIy, const PV& x, const PV& y){
    try{ xIy = x^y; return true; }
    catch(typename mc::PolImg<T>::Exceptions &eObj){ return false; }
  }
  static bool eq(const PV& x, const PV& y)
    { return mc::Op<T>::eq(x.range(),y.range()); }
  static bool ne(const PV& x, const PV& y)
    { return mc::Op<T>::ne(x.range(),y.range()); }
  static bool lt(const PV& x, const PV& y)
    { return mc::Op<T>::lt(x.range(),y.range()); }
  static bool le(const PV& x, const PV& y)
    { return mc::Op<T>::le(x.range(),y.range()); }
  static bool gt(const PV& x, const PV& y)
    { return mc::Op<T>::gt(x.range(),y.range()); }
  static bool ge(const PV& x, const PV& y)
    { return mc::Op<T>::ge(x.range(),y.range()); }
};

} // namespace mc

#endif
