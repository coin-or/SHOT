// Copyright (C) 2009-2016 Benoit Chachuat, Imperial College London.
// All Rights Reserved.
// This code is published under the Eclipse Public License.

/*!
\page page_SPECBND Eigenvalue Arithmetic for Factorable Functions
\author Nikola Peri&cacute;, Akshay Shah, Jai Rajyaguru & Beno&icirc;t Chachuat

Given a factorable, multivariate function \f$f:\mathbb{R}^n\to\mathbb{R}\f$, that is twice-continuously differentiable on a box \f$X:=[x^{\rm L},x^{\rm U}]\f$, M&ouml;nnigmann's technique [M&ouml;nnigmann, 2008; 2011] provides a means for computing spectral bounds of its Hessian matrix \f$H_f(x)\f$ at any point \f$x\in X\f$&mdash;without actually computing \f$H_f(x)\f$. Applications of this technique are in determining whether a function is convex or concave on a particular domain, as well as in constructing convex/concave relaxations for complete search approaches in global optimization. Alternative techniques for determining spectral bounds include the interval variant of Gershgorin's circle criterion [Adjiman <I>et al.</I>, 1998] as well as Hertz & Rohn's method [Hertz, 1992], which rely on an interval enclosure of the set of all possible Hessian matrices \f$[H_f] \supseteq \{H_f(x) \mid x\in X\}\f$. See also \ref page_MCCORMICK for an alternative way of constructing convex/concave relaxations. 

The class mc::Specbnd provides an implementation of eigenvalue arithmetic. We note that mc::SpecBnd is <B>not a verified implementation</B> in the sense that rounding errors are not accounted for in propagating the spectral bounds.

The implementation of mc::Specbnd relies on the operator/function overloading mechanism of C++. This makes the computation of spectral bounds both simple and intuitive, similar to computing function values in real arithmetics. Moreover, mc::Specbnd can be used as the template parameter of other available types in MC++; for instance, mc::Specbnd can be used in order to propagate spectral bounds on the remainder term of Taylor variable mc::TVar. Likewise, mc::Specbnd can be used as the template parameter of the types fadbad::F, fadbad::B and fadbad::T of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A> for computing spectral bounds of either the partial derivatives or the Taylor coefficients of a factorable function too (see \ref sec_SPECBND_fadbad).

mc:Specbnd is itself templated in the type used to propagate the necessary bounds. By default, mc::Specbnd can be used with the non-verified interval type mc::Interval of MC++. For reliability, however, it is strongly recommended to use verified interval arithmetic such as <A href="http://www.ti3.tu-harburg.de/Software/PROFILEnglisch.html">PROFIL</A> (header file <tt>mcprofil.hpp</tt>) or <A href="http://www.math.uni-wuppertal.de/~xsc/software/filib.html">FILIB++</A> (header file <tt>mcfilib.hpp</tt>). The types mc::McCormick and mc::TVar can also be used as template parameters of mc::Specbnd, thereby making it possible to compute, respectively, convex/concave bounds and Taylor models of the spectrum of \f$H_f\f$.

As well as propagating spectral bounds for factorable functions using M&ouml;nnigmann's technique, mc::Specbnd provides support for computing spectral bounds based on the interval Hessian matrix of a twice continuously-differentiable, factorable function using either Gershgorin's circle criterion or Hertz & Rohn's method. As established by Darup <I>et al.</I> [2012], the spectral bound arithmetic may or may not produce tighter spectral bounds than these latter techniques, depending on the factorable function \f$f\f$ at hand and its variable range \f$X\f$.

Results obtained for the factorable function \f$f(x,y)=1+x-\sin(2x+3y)-\cos(3x-5y)\f$  for \f$x\in [-0.5,0.5]^2\f$ are shown in the figure below. This function (red line) and corresponding interval bounds (blue line) are shown in the left plot, while the minimal and maximal eigenvalues (red and green lines) as well as the computed spectral bounds (blue line) of the Hessian matrix \f$H_f\f$ are shown on the right plot.

<CENTER><TABLE BORDER=0>
<TR>
<TD>\image html SB-2D_function.png</TD>
<TD>\image html SB-2D_spectral.png</TD>
</TR>
</TABLE></CENTER>


\section sec_SPECBND_I How do I compute (interval) bounds on the spectrum of the Hessian matrix of a factorable function?

Suppose we want to compute a spectral (interval) bound for the Hessian matrix of the real-valued function \f$f(x_1,x_2,x_3)=\exp(x_1-2x_2^2+3x_3^3)\f$ for \f$(x_1,x_2,x_3)\in [-0.3,0.2]\times[-0.1,0.6]\times[-0.4,0.5]\f$. For simplicity, this bound is calculated using the default interval type, mc::Interval:

\code
      #include "interval.hpp"
      #include "specbnd.hpp"
      typedef mc::Interval I;
      typedef mc::Specbnd<I> SB;
\endcode

First, the variables \f$x_1\f$, \f$x_2\f$ and \f$x_3\f$ are defined as follows:

\code
      SB X1( I(-0.3,0.2), 0, 3 );
      SB X2( I(-0.1,0.6), 1, 3 );
      SB X3( I(-0.4,0.5), 2, 3 );
\endcode

Essentially, the first line means that <tt>X1</tt> is a variable of class mc::Specbnd, belongs to the interval \f$[-0.3,0.2]\f$, and having index 0 out of 3 independent variables (recall that indexing in C/C++ starts at 0 by convention!). The same holds for the mc::Specbnd variable <tt>X2</tt> and <tt>X3</tt>.

Having defined the variables, spectral bounds on the Hessian matrix \f$H_f\f$ of \f$f\f$ on \f$[-0.3,0.2]\times[-0.1,0.6]\times[-0.4,0.5]\f$ are simply calculated as:

\code
      SB F = exp( X1 - 2*sqr(X2) + 3*pow(X3,3) );
\endcode

The computed spectral bounds can be retrieved as:

\code
      I specF = F.SI();
\endcode

The function and first-derivative bounds can also be retrieved using mc::Specbnd::I and mc::Specbnd::FI. The results of the spectral bound propagation can be displayed to the standard output as:

\code
      std::cout << F << std::endl;
\endcode

which produces the following display:
\verbatim
  [ -1.99039e+01 :  3.70043e+01 ]
  [  2.97601e-01 :  1.77713e+00 ]
  ( [  2.97601e-01 :  1.77713e+00 ] )
  ( [ -4.26511e+00 :  7.10852e-01 ] )
  ( [  0.00000e+00 :  3.99854e+00 ] )
\endverbatim

Note that the display is organized as follows:
\f{align*}
& [\underline{\lambda}:\overline{\lambda}]\\
& [\underline{f}:\overline{f}]\\
& ( [\underline{\partial_{x_1}f}:\overline{\partial_{x_1}f}] )\\
& ( [\underline{\partial_{x_2}f}:\overline{\partial_{x_2}f}] )\\ 
& ( [\underline{\partial_{x_3}f}:\overline{\partial_{x_3}f}] )
\f}
That is, the spectral bound is \f$[-19.904,37.004]\f$ here.

As noted earlier, other methods are available for bounding the spectrum of interval Hessian matrices, such as Gershgorin's circle criterion and Hertz & Rohn's method. mc::Specbnd also provides a means for computing these bounds, e.g. for comparison with the M&ouml;nnigmann's eigenvalue arithmetic. Interval Hessian matrices can be computed using the forward and/or reverse AD types of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A>:

\code
      #include "mcfadbad.hpp"
      typedef fadbad::F<I> FI;
      typedef fadbad::F<FI> FFI;
      typedef fadbad::B<FI> BFI;
\endcode

Then, in order to compute spectral bounds for \f$f(x_1,x_2,x_3)=\exp(x_1-2x_2^2+3x_3^3)\f$ on \f$(x_1,x_2,x_3)\in [-0.3,0.2]\times[-0.1,0.6]\times[-0.4,0.5]\f$, we proceed as follows:

\code
      FI FX1 = I(-0.3,0.2); FX1.diff(0,3);
      FI FX2 = I(-0.1,0.6); FX2.diff(1,3);
      FI FX3 = I(-0.4,0.5); FX3.diff(2,3);

      FFI FFX1 = FX1; FFX1.diff(0,3);
      FFI FFX2 = FX2; FFX2.diff(1,3);
      FFI FFX3 = FX3; FFX3.diff(2,3);
      FFI FFZ = exp( FFX1 - 2*pow(FFX2,2) + 3*pow(FFX3,3) );
      std::pair<double,double> spbndF;
      
      SB::options.HESSBND = SB::Options::GERSHGORIN;
      spbndF = SB::spectral_bound( FFZ );
      std::cout << "Spectral bound (Gershgorin, forward-forward): " << I(spbndF.first,spbndF.second) << std::endl;

      SB::options.HESSBND = SB::Options::HERTZROHN;
      spbndF = SB::spectral_bound( FFZ );
      std::cout << "Spectral bound (Hertz&Rohn, forward-forward): " << I(spbndF.first,spbndF.second) << std::endl;
\endcode

producing:
\verbatim
Spectral bound (Gershgorin, forward-forward): [ -2.63904e+01 :  3.85859e+01 ]
Spectral bound (Hertz&Rohn, forward-forward): [ -2.11973e+01 :  3.05272e+01 ]
\endverbatim

In this case, the eigenvalue arithmetic provides tighter bounds than with Gershgorin's circle criterion, yet looser than with Herz & Rohn's method.

Spectral bounds for interval Hessian matrices generated with the forward-reverse mode of AD can also be computed:

\code
      BFI BFX[3] = { FX1, FX2, FX3 };
      BFI BFZ = exp( BFX[0] - 2*pow(BFX[1],2) + 3*pow(BFX[2],3) );
      BFZ.diff(0,1);
      std::pair<double,double> spbndB;

      SB::options.HESSBND = SB::Options::GERSHGORIN;
      spbndB = SB::spectral_bound( BFX );
      std::cout << "Spectral bound (Gershgorin, forward-reverse): " << I(spbndB.first,spbndB.second) << std::endl;

      SB::options.HESSBND = SB::Options::HERTZROHN;
      spbndB = SB::spectral_bound( BFX );
      std::cout << "Spectral bound (Hertz&Rohn, forward-reverse):  " << I(spbndB.first,spbndB.second) << std::endl;
\endcode

producing:
\verbatim
Spectral bound (Gershgorin, forward-reverse): [ -2.63904e+01 :  3.85859e+01 ]
Spectral bound (Hertz&Rohn, forward-reverse): [ -2.11973e+01 :  3.05272e+01 ]
\endverbatim

In this case, the spectral bounds computed from the Hessian matrix obtained with the forward-forward and forward-reverse modes of AD are indeed identical, although such may not always be the case; see [Darup <I>et al.</I>, 2012]

\section sec_SPECBND_MC How do I compute convex/concave relaxations on the spectrum of the Hessian matrix of a factorable function?

Instead of using standard interval arithmetic for propagating spectral bounds, we can as well make use of the McCormick relaxation technique to propagate convex/concave bounds. Using MC++, this is done simply by selecting mc::McCormick as the template parameter in mc::Specbnd:

\code
      #include "mccormick.hpp"
      typedef mc::McCormick<I> MC;
      typedef mc::Specbnd<MC> SBMC;
\endcode

Then, the procedure for computing a spectral bound remains essentially the same as described in the previous section. In order to compute convex/concave spectral relaxations and subgradients at point \f$(0,0,0)\f$ of \f$f(x_1,x_2,x_3)=\exp(x_1-2x_2^2+3x_3^3)\f$ for \f$(x_1,x_2,x_3)\in [-0.3,0.2]\times[-0.1,0.6]\times[-0.4,0.5]\f$, we proceed as follows:

\code
      SBMC X1( MC(I(-0.3,0.2),0.).sub(3,0), 0, 3 );
      SBMC X2( MC(I(-0.1,0.6),0.).sub(3,1), 1, 3 );
      SBMC X3( MC(I(-0.4,0.5),0.).sub(3,2), 2, 3 );
      SBMC F = exp( X1 - 2*sqr(X2) + 3*pow(X3,3) );
      MC specF = F.SI();
      std::cout << F << std::endl;
\endcode

The only difference here concerns the initialization of the McCormick variables inside the variables X and Y, which passes the bounds for the variable as well as the point at which the convex/concave bounds and their subgradients are computed&mdash;See How do I compute McCormick relaxations of a factorable function?

More information on the spectral relaxations can again be obtained as:

\code
      std::cout << F << std::endl;
\endcode

which displays:
\verbatim
  [ -1.99039e+01 :  3.70043e+01 ] [ -1.54413e+01 :  2.81720e+01 ] [ (-9.27293e+00, 0.00000e+00,-5.21602e+00) : ( 1.72398e+01, 0.00000e+00, 1.50542e+01) ]
  [  2.97601e-01 :  1.77713e+00 ] [  8.45354e-01 :  1.37868e+00 ] [ ( 8.45354e-01,-8.45354e-01, 3.04327e-01) : ( 8.27940e-01, 0.00000e+00, 4.65716e-01) ]
  ( [  2.97601e-01 :  1.77713e+00 ] [  8.45354e-01 :  1.37868e+00 ] [ ( 8.45354e-01,-8.45354e-01, 3.04327e-01) : ( 8.27940e-01, 0.00000e+00, 4.65716e-01) ] )
  ( [ -4.26511e+00 :  7.10852e-01 ] [ -3.72711e-01 :  4.32433e-01 ] [ ( 3.38142e-01,-7.44666e+00, 1.21731e-01) : ( 3.31176e-01,-1.19041e+00, 1.86287e-01) ] )
  ( [  0.00000e+00 :  3.99854e+00 ] [  0.00000e+00 :  2.96812e+00 ] [ ( 0.00000e+00, 0.00000e+00, 0.00000e+00) : ( 1.86287e+00, 0.00000e+00, 1.31570e+00) ] )
\endverbatim

By construction, the underlying interval bounds of the McCormick relaxations are identical to their spectral interval bound counterparts. The convex/concave bounds at \f$(0,0,0)\f$ come as additional information and are seen to be tighter than the interval bounds. The corresponding subgradients can also be used to construct affine relaxations.

Similarly, a Taylor model of the spectrum can be conveniently computed by selecting mc::TVar as the template parameter in mc::Specbnd.


\section sec_SPECBND_fct Which functions are overloaded in mc::Specbnd eigenvalue arithmetic?

mc::TVar overloads the usual functions <tt>exp</tt>, <tt>log</tt>, <tt>sqr</tt>, <tt>sqrt</tt>, <tt>pow</tt>, <tt>inv</tt>, <tt>cos</tt>, <tt>sin</tt>, <tt>tan</tt>, <tt>acos</tt>, <tt>asin</tt>, <tt>atan</tt>. Unlike mc::Interval and mc::McCormick, the functions <tt>min</tt>, <tt>max</tt> and <tt>fabs</tt> are not overloaded in mc::TVar since they are not twice continuously differentiable. Also, <tt>erf</tt> and <tt>erfc</tt> are overloaded in mc::Specbnd but currently cannot be used due to a limitation in the third-party libarary <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A>, where these functions are not (yet?) overloaded.


\section sec_SPECBND_fadbad How do I compute spectral bounds of the partial derivatives or the Taylor coefficients of a factorable function using FADBAD++?

The combination of mc::Specbnd with the classes fadbad::F, and fadbad::B of <A href="http://www.fadbad.com/fadbad.html">FADBAD++</A> to compute a spectral bound for the Hessian matrix of either the partial derivatives or the Taylor coefficients of a factorable function is essentially the same as with mc::McCormick (see \ref sec_MCCORMICK_fadbad) or mc::TVar (see \ref sec_TAYLOR_fadbad).

Next, we present the case of fadbad::F only. Continuing the previous example, spectral bounds of the partial derivatvies of \f$f(x_1,x_2,x_3)=\exp(x_1-2x_2^2+3x_3^3)\f$ for \f$(x_1,x_2,x_3)\in [-0.3,0.2]\times[-0.1,0.6]\times[-0.4,0.5]\f$ can be computed as follows:

\code
      typedef fadbad::F<SB> FSB;
\endcode

\code
      FSB FSBX1 = X1; FSBX1.diff(0,3);
      FSB FSBX2 = X2; FSBX2.diff(1,3);
      FSB FSBX3 = X3; FSBX3.diff(2,3);
      FSB FSBF = exp( FSBX1 - 2*pow(FSBX2,2) + 3*pow(FSBX3,3) );
      std::cout << "Spectral bounds of df/dx1:\n" << FSBF.d(0) << std::endl;
      std::cout << "Spectral bounds of df/dx2:\n" << FSBF.d(1) << std::endl;
      std::cout << "Spectral bounds of df/dx3:\n" << FSBF.d(2) << std::endl;
\endcode

producing the output:

\verbatim
Spectral bounds of df/dx1:
  [ -1.99039e+01 :  3.70043e+01 ]
  [  2.97601e-01 :  1.77713e+00 ]
  ( [  2.97601e-01 :  1.77713e+00 ] )
  ( [ -4.26511e+00 :  7.10852e-01 ] )
  ( [  0.00000e+00 :  3.99854e+00 ] )

Spectral bounds of df/dx2:
  [ -1.16096e+02 :  8.92716e+01 ]
  [ -4.26511e+00 :  7.10852e-01 ]
  ( [ -4.26511e+00 :  7.10852e-01 ] )
  ( [ -8.81457e+00 :  9.04587e+00 ] )
  ( [ -9.59650e+00 :  1.59942e+00 ] )

Spectral bounds of df/dx3:
  [ -1.28567e+02 :  2.06229e+02 ]
  [  0.00000e+00 :  3.99854e+00 ]
  ( [  0.00000e+00 :  3.99854e+00 ] )
  ( [ -9.59650e+00 :  1.59942e+00 ] )
  ( [ -1.27953e+01 :  2.49909e+01 ] )
\endverbatim



\section sec_SPECBND_opt How are the options set for the computation of a spectral bound?

The class mc::Specbnd has a public static member called mc::Specbnd::options that can be used to set/modify the options; e.g.,

\code
      mc::Specbnd<I>::options.HESSBND = mc::Specbnd<I>::Options::HERTZROHN;
\endcode

The available options are the following:

<TABLE border="1">
<CAPTION><EM>Options in mc::Specbnd::Options: name, type and description</EM></CAPTION>
     <TR><TH><b>Name</b>  <TD><b>Type</b><TD><b>Default</b>
         <TD><b>Description</b>
     <TR><TH><tt>HESSBND</tt> <TD><tt>mc::Specbnd::Options::HESSBND_STRATEGY</tt> <TD>mc::Specbnd::Options::GERSHGORIN
         <TD>Strategy for computing spectral bounds in interval Hessian matrix using mc::Specbnd::spectral_bound
</TABLE>


\section sec_SPECBND_err Errors What errors can I encounter during computation of a spectral bound?

Errors are managed based on the exception handling mechanism of the C++ language. Each time an error is encountered, a class object of type mc::Specbnd::Exceptions is thrown, which contains the type of error. It is the user's responsibility to test whether an exception was thrown during the computation of a spectral bound, and then make the appropriate changes. Should an exception be thrown and not caught by the calling program, the execution will abort.

Possible errors encountered during the computation of a spectral bound are:

<TABLE border="1">
<CAPTION><EM>Errors during the Computation of a Spectral Bound</EM></CAPTION>
     <TR><TH><b>Number</b> <TD><b>Description</b>
     <TR><TH><tt>1</tt> <TD>Failed to compute spectrum in Specbnd::spectrum
     <TR><TH><tt>2</tt> <TD>Failed to compute spectral bound in Specbnd::spectral_bound
     <TR><TH><tt>-1</tt> <TD>Operation between variables with different numbers of dependents
     <TR><TH><tt>-33</tt> <TD>Feature not yet implemented in mc::Specbnd
</TABLE>

Moreover, exceptions may be thrown by the template parameter class itself.


\section sec_SPECBND_refs References

- Adjiman, C.S., S. Dallwig, C.A. Floudas, and A. Neumaier, <A href="http://dx.doi.org/10.1016/S0098-1354(98)00027-1">A global optimization method, \f$\rm\alpha BB\f$, for general twice-differentiable constrained NLPs-I. Theoretical advances</A>, <I>Computers & Chemical Engineering</I> <B>22</B>(9):1137-1158, 1998.
- Hertz, D., <A href="http://dx.doi.org/10.1109/9.126593">The extreme eigenvalues and stability of real symmetric interval matrices</A>, <I>IEEE Transactions on Automatic Control</I> <B>37</B>:532-535, 1992.
- M&ouml;nnigmann, M., <A href="http://dx.doi.org/10.1137/070704186">Efficient calculation of bounds on spectra of Hessian matrices</A>, <i>SIAM Journal on Scientific Computing</i>, <b>30</b>:2340-2357, 2008.
- M&ouml;nnigmann, M., <A href="http://dx.doi.org/10.1137/10078760X">Fast Calculation of Spectral Bounds for Hessian Matrices on Hyperrectangles</A>, <i>SIAM Journal on Matrix Analysis and Applications</i>, <b>32</b>:4, 1351-1366, 2011.
- Darup, M. S., M. Kastsian, S. Mross, and M. M&ouml;nnigmann, <A href="http://arxiv.org/pdf/1206.0196.pdf">Efficient Computation of Spectral Bounds for Hessian Matrices on Hyperrectangles for Global Optimization</A>, arXiv:1206.0196v1, 1 June 2012.
.

*/

#ifndef MC__SPECBND_H
#define MC__SPECBND_H

#include <iostream>
#include <cmath>

#include "mclapack.hpp"
#include "mcop.hpp"
#include "fadiff.h"
//#include "mcfadiff.hpp"
#include "badiff.h"

#undef  MC__SPECBND_DEBUG_SPECTRUM
#undef  MC__SPECBND_DEBUG_HESSBND

namespace mc
{
//! @brief C++ template class computing spectral bounds for the Hessian matrix of a factorable function on a box
////////////////////////////////////////////////////////////////////////
//! mc::Specbnd<T> is a C++ template class computing spectral bounds
//! for the Hessian matrix of a factorable function on a box. The
//! template parameter T corresponds to the type used in the underlying
//! interval arithmetic calculations.
////////////////////////////////////////////////////////////////////////
template <typename T>
class Specbnd
////////////////////////////////////////////////////////////////////////
{
  template <typename U> friend class Specbnd;

  template <class U> friend Specbnd<U> operator+(const Specbnd<U> &x );
  template <class U> friend Specbnd<U> operator+(const Specbnd<U> &x, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator+(const Specbnd<U> &y, const double c);
  template <class U> friend Specbnd<U> operator+(const double c, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator+(const int c, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator-(const Specbnd<U> &x );
  template <class U> friend Specbnd<U> operator-(const Specbnd<U> &x, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator-(const Specbnd<U> &y, const double c);
  template <class U> friend Specbnd<U> operator-(const double c, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator*(const Specbnd<U> &x, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator*(const double c, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator*(const Specbnd<U> &y, const double c);
  template <class U> friend Specbnd<U> pow(const Specbnd<U> &x, const int m);
  template <class U> friend Specbnd<U> pow(const Specbnd<U> &x, const double c);
  template <class U> friend Specbnd<U> pow(const Specbnd<U> &x, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> pow(const double c, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> inv(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator/(const Specbnd<U> &x, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> operator/(const Specbnd<U> &y, const double c);
  template <class U> friend Specbnd<U> operator/(const double c, const Specbnd<U> &y);
  template <class U> friend Specbnd<U> sqr(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> sqrt(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> cheb(const Specbnd<U>&, const unsigned );
  template <class U> friend Specbnd<U> exp(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> log(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> xlog(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> cos(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> sin(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> acos(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> asin(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> tan(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> atan(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> erf(const Specbnd<U> &y);
  template <class U> friend Specbnd<U> erfc(const Specbnd<U> &y);
  template <class U> friend std::ostream& operator<<(std::ostream&, const Specbnd<U>&);

private:
  //! @brief Number of independent variables
  unsigned int _n;
  //! @brief Gradient bounds
  fadbad::F<T> _FI;
  //! @brief Spectral bound
  T _spec;

  //! @brief Internal function for spectral bound propagation in univariate terms
  static T _LambdaS( const fadbad::F<T> &a, const unsigned int n );
  //! @brief Internal function for spectral bound propagation in product terms
  static T _LambdaT( const fadbad::F<T> &a, const fadbad::F<T> &b, const unsigned int n );

  ////! @brief Computing spectral bound of interval Hessian matrix (forward-reverse AD) using Gershgorin's circle criterion
  //static std::pair<double,double> _gershgorin_bound
  //  ( const fadbad::B< fadbad::F< T > >* D2X );
  ////! @brief Computing spectral bound of interval Hessian matrix (forward-reverse AD) using Hertz & Rohn's method
  //static std::pair<double,double> _hertzrohn_bound
  //  ( const fadbad::B< fadbad::F< T > >* D2X );
  ////! @brief Computing spectral bound of interval Hessian matrix (forward-forward AD) using Gershgorin's circle criterion
  //static std::pair<double,double> _gershgorin_bound
  //  ( const fadbad::F< fadbad::F< T > >& D2F );
  ////! @brief Computing spectral bound of interval Hessian matrix (forward-forward AD) using Hertz & Rohn's method
  //static std::pair<double,double> _hertzrohn_bound
  //  ( const fadbad::F< fadbad::F< T > >& D2F );
  //! @brief Computing spectral bound of interval Hessian matrix using Gershgorin's circle criterion
  static std::pair<double,double> _gershgorin_bound
    ( const unsigned N, const T*D2F );
  //! @brief Computing spectral bound of interval Hessian matrix using Hertz & Rohn's method
  static std::pair<double,double> _hertzrohn_bound
    ( const unsigned N, const T*D2F );

public: 
  // other operator overloadings
  Specbnd<T>& operator+=
    ( const Specbnd<T>& );
  Specbnd<T>& operator+=
    ( const double );
  Specbnd<T>& operator-=
    ( const Specbnd<T>& );
  Specbnd<T>& operator-=
    ( const double );
  Specbnd<T>& operator*=
    ( const Specbnd<T>& );
  Specbnd<T>& operator*=
    ( const double );
  Specbnd<T>& operator/=
    ( const Specbnd<T>& );
  Specbnd<T>& operator/=
    ( const double );
  Specbnd<T> & operator=
    ( const Specbnd<T> &x );
  Specbnd<T> & operator=
    ( const double c );
  Specbnd<T> & operator=
    ( const T &c );

  /** @defgroup SPECBND Eigenvalue Arithmetic for Factorable Functions
   *  @{
   */
  //! @brief Options of mc::Specbnd
  static struct Options
  {
    //! @brief Constructor
    Options():
      HESSBND(GERSHGORIN)
      {}
    //! @brief Strategy for computing spectral bounds in interval Hessian matrix
    enum HESSBND_STRATEGY{
      GERSHGORIN=0,	//!< Gershgorin circle's criterion
      HERTZROHN		//!< Hertz & Rohn's method
    };
    //! @brief Method to bound eignevalues in interval Hessian matrix using mc::Specbnd::spectral_bound
    HESSBND_STRATEGY HESSBND;
  } options;

  //! @brief Exceptions of mc::Specbnd
  class Exceptions
  {
  public:
    //! @brief Enumeration type for Specbnd exception handling
    enum TYPE{
      SPECTR=1,	//!< Failed to compute spectrum in Specbnd::spectrum
      HESSBND,	//!< Failed to compute spectral bound in Specbnd::spectral_bound
      SIZE=-1,	//!< Operation between variables with different numbers of dependents
      UNDEF=-33 //!< Feature not yet implemented in mc::Specbnd
    };
    //! @brief Constructor for error <a>ierr</a>
    Exceptions( TYPE ierr ) : _ierr( ierr ){}
    //! @brief Inline function returning the error flag
    int ierr(){ return _ierr; }
    //! @brief Error description
    std::string what(){
      switch( _ierr ){
      case SPECTR:
        return "mc::Specbnd\t Computation of Hessian spectrum failed";
      case HESSBND:
        return "mc::Specbnd\t Computation of Hessian spectral bound failed";
      case SIZE:
        return "mc::Specbnd\t Operation between variables with different numbers of dependents";
      case UNDEF:
        return "mc::Specbnd\t Feature not yet implemented in mc::Specbnd class";
      default:
        return "mc::Specbnd\t Undocumented error";
      }
    }

  private:
    TYPE _ierr;
  };

  //! @brief Default constructor (needed to declare arrays of Specbnd)
  Specbnd():
    _n(0), _FI(0), _spec(0.)
  {}

  //! @brief Constructor for real scalar <tt>c</tt>
  Specbnd( const double c ):
    _n(0), _FI(c), _spec(0.)
  {}

  //! @brief Constructor for an interval <tt>B</tt>
  Specbnd( const T &B ):
    _n(0), _FI(B), _spec(0.)
  {}

  //! @brief Constructor for a variable with range <tt>B</tt> and index <a>i</a> of <a>n</a> independent variables
  Specbnd( const T &B, const unsigned int i, const unsigned int n ):
    _n(n), _FI(B), _spec(0.)
  {
    _FI.diff(i,_n);
  }

  //! @brief Copy constructor
  Specbnd(const Specbnd<T> &x):
    _n(x._n), _FI(x._FI), _spec(x._spec)
  {}

  //! @brief Destructor
  ~Specbnd()
  {}

  //! @brief Set variable with range <tt>B</tt> and index <a>i</a> of <a>n</a> independent variables
  Specbnd<T>& set( const T &B, const unsigned int i, const unsigned int n )
  {
    _n = n;
    _FI = B;
    _FI.diff(i,_n);
    _spec = 0.;
    return *this;
  }

  //! @brief Set function and first derivative bounds as well as spectral bounds to, respectively, <tt>FB</tt> and <tt>SB</tt>
  Specbnd<T>& set( const fadbad::F<T>&FB, const T&SB );

  //! @brief Set the index of a variable (and total number of variables)
  Specbnd<T>& dep( const unsigned int i, const unsigned int n )
  {
    _n = n;
    _FI.diff( i, n );
    _spec = 0.;
    return *this;
  }

  //! @brief Return number of independent variables
  unsigned int dep() const
  {
    return _n;
  }

  //! @brief Return function bounds
  const T& I() const
  {
    return _FI.val();
  }

  //! @brief Return function and gradient bounds
  const fadbad::F<T>& FI() const
  {
    return _FI;
  }

  //! @brief Return spectral bounds for Hessian matrix
  const T& SI() const
  {
    return _spec;
  }

  //! @brief Compute spectrum of Hessian matrix <tt>D2X</tt> of type fadbad::B< fadbad::F<double> >* using LAPACK function <tt>dsyev</tt>.
  static std::pair<double,double> spectrum
    ( const fadbad::B< fadbad::F< double > >* D2X );

  //! @brief Compute spectrum of Hessian matrix <tt>D2F</tt> of type fadbad::F< fadbad::F<double> > using LAPACK function <tt>dsyev</tt>.
  static std::pair<double,double> spectrum
    ( const fadbad::F< fadbad::F< double > >& D2X );

  //! @brief Compute spectral bound of interval Hessian matrix <tt>D2X</tt> of type fadbad::B< fadbad::F<T> >*. The bounding method is selected via mc::Specbnd::Options::HESSBND.
  static std::pair<double,double> spectral_bound
    ( const fadbad::B< fadbad::F< T > >* D2X );

  //! @brief Compute spectral bound of interval Hessian matrix <tt>D2X</tt> of type fadbad::F< fadbad::F<T> >. The bounding method is selected via mc::Specbnd::Options::HESSBND.
  static std::pair<double,double> spectral_bound
    ( const fadbad::F< fadbad::F< T > >& D2X );

  //! @brief Compute spectral bound of symmetric interval matrix <tt>S</tt> of size <tt>N</tt>. The bounding method is selected via mc::Specbnd::Options::HESSBND.
  static std::pair<double,double> spectral_bound
    ( const unsigned N, const T*S );

  //! @brief Compute bound on the real part of the spectrum of (non-symmetric) interval matrix <tt>A</tt>. The bounding method is selected via mc::Specbnd::Options::HESSBND.
  static std::pair<double,double> spectral_bound_re
    ( const unsigned N, const T*A );

  //! @brief Compute bound on the imaginary part of the spectrum of (non-symmetric) interval matrix <tt>A</tt>. The bounding method is selected via mc::Specbnd::Options::HESSBND.
  static std::pair<double,double> spectral_bound_im
    ( const unsigned N, const T*A );
  /** @} */
};

////////////////////////////////////////////////////////////////////////

template <typename T> typename Specbnd<T>::Options Specbnd<T>::options;

template <class T> inline T
Specbnd<T>::_LambdaS
( const fadbad::F<T> &a, unsigned int n )
{
  if( !n ) return 0.;
  if( n == 1 ) return a.size()? Op<T>::sqr( a[0] ): 0.;
  double upbnd=0.;
  for (unsigned int i=0; i<n; i++)
    if( a.size() ) upbnd += Op<T>::u( Op<T>::sqr( a[i] ) );
  return Op<T>::zeroone() * upbnd;
}

template <class T> inline T
Specbnd<T>::_LambdaT
( const fadbad::F<T> &a, const fadbad::F<T> &b, unsigned int n )
{
  if( !n ) return 0.;
  if( n == 1 ) return (a.size() && b.size())? 2.*a[0]*b[0]: 0.;
  double upbnda=0., upbndb=0.;
  for (unsigned int i=0; i<n; i++){
    if( a.size() ) upbnda += Op<T>::u( Op<T>::sqr( a[i] ) );
    if( b.size() ) upbndb += Op<T>::u( Op<T>::sqr( b[i] ) );
  }
  T lamb = 2.*(Op<T>::zeroone()-0.5) * std::sqrt(upbnda*upbndb);
  for (unsigned int i=0; i<n; i++)
    if( a.size() && b.size() ) lamb += a[i]*b[i];
  return lamb;
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::spectrum
( const fadbad::B< fadbad::F< double > >* D2X )
{
  if( !D2X ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SPECTR );
  const unsigned int N = D2X->val().size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SPECTR );
  double*H = new double[N*N];

  for( unsigned int j=0; j<N; j++ )
    for( unsigned int i=j; i<N; i++ )
      H[j*N+i] = H[i*N+j] = ( i==j? D2X[i].deriv(0).deriv(i):
        0.5*(D2X[i].deriv(0).deriv(j)+D2X[j].deriv(0).deriv(i)) );
#ifdef MC__SPECBND_DEBUG_SPECTRUM
  mc::display( N, N, H, N, "\nMatrix H", std::cout );
#endif

  double*D = mc::dsyev_wrapper( N, H );
  delete[] H;
  if( !D ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SPECTR );
  std::pair<double,double> spbnd = std::make_pair( min(N,D), max(N,D) );
  delete[] D;
  return spbnd;
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::spectrum
( const fadbad::F< fadbad::F< double > >& D2F )
{
  const unsigned int N = D2F.size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SPECTR );
  double*H = new double[N*N];

  for( unsigned int j=0; j<N; j++ )
    for( unsigned int i=j; i<N; i++ )
      H[j*N+i] = H[i*N+j] = ( i==j? D2F.deriv(i).deriv(i):
        0.5*(D2F.deriv(i).deriv(j)+D2F.deriv(j).deriv(i)) );
#ifdef MC__SPECBND_DEBUG_SPECTRUM
  mc::display( N, N, H, N, "\nMatrix H", std::cout );
#endif

  double*D = mc::dsyev_wrapper( N, H );
  delete[] H;
  if( !D ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SPECTR );
  std::pair<double,double> spbnd = std::make_pair( min(N,D), max(N,D) );
  delete[] D;
  return spbnd;
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::spectral_bound
( const fadbad::B< fadbad::F< T > >* D2F )
{
  if( !D2F ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
  const unsigned int N = D2F->val().size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
  T pD2F[N*N];
  for( unsigned int i=0, k=0; i<N; i++ )
    for( unsigned int j=0; j<N; j++, k++ )
      pD2F[k] = D2F[i].deriv(0).deriv(j);

  switch( options.HESSBND ){
  case Options::GERSHGORIN:
    return _gershgorin_bound( N, pD2F );
  case Options::HERTZROHN: default:
    return _hertzrohn_bound( N, pD2F );
  }
}
/*
template <typename T> inline std::pair<double,double>
Specbnd<T>::_gershgorin_bound
( const fadbad::B< fadbad::F< T > >* D2X )
{
  const unsigned int N = D2X->val().size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );

  std::pair<double,double> spbnd;
  for( unsigned int i=0; i<N; i++ ){
    double ri = 0.;
    for( unsigned int j=0; j<N; j++ ){
      if( j == i ) continue;
      T D2Fij;
      if( !inter( D2Fij, D2X[i].deriv(0).deriv(j), D2X[j].deriv(0).deriv(i) ) )
        D2Fij = D2X[i].deriv(0).deriv(j); 
      ri += Op<T>::abs( D2Fij );
    }

    if( !i ){
      spbnd.first  = Op<T>::l(D2X[i].deriv(0).deriv(i)) - ri;
      spbnd.second = Op<T>::u(D2X[i].deriv(0).deriv(i)) + ri;
    }
    else{
      spbnd.first  = std::min( spbnd.first,  Op<T>::l(D2X[i].deriv(0).deriv(i)) - ri );
      spbnd.second = std::max( spbnd.second, Op<T>::u(D2X[i].deriv(0).deriv(i)) + ri );
    }
  }
  return spbnd;
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::_hertzrohn_bound
( const fadbad::B< fadbad::F< T > >* D2X )
{
  const unsigned int N = D2X->val().size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );

  // Form matrix S^{(N)} recursively
  unsigned int col_S = 2;
  short *S = new short[col_S];
  S[0] = 1; S[1] = -1;
#ifdef MC__SPECBND_DEBUG_HESSBND
  mc::display( 1, col_S, S, 1, "\nMatrix S1", std::cout );
#endif
  for( unsigned int k=1; k<N; k++, col_S*=2 ){
    short *Sprev = new short[k*col_S];
    for( unsigned int i=0; i<k; i++ )
      for( unsigned int j=0; j<col_S; j++ )
        Sprev[j*k+i] = S[j*k+i];
    delete[] S; S = new short[(k+1)*2*col_S];
    for( unsigned int i=0; i<k; i++ ){
      for( unsigned int j=0; j<col_S; j++ )
        S[j*(k+1)+i] = S[(col_S+j)*(k+1)+i] = Sprev[j*k+i];
    }
    for( unsigned int j=0; j<col_S; j++ ){
      S[j*(k+1)+k] = 1; S[(col_S+j)*(k+1)+k] = -1;
    }
    delete[] Sprev;
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( k+1, 2*col_S, S, k+1, "\nMatrix Sk", std::cout );
#endif
  }

  // Compute lower and upper bound on spectral radius
  std::pair<double,double> spbnd;
  double *Lk = new double[N*N], *Uk = new double[N*N];
  for( unsigned int k=0; k<col_S; k++ ){
    
    for( unsigned int j=0; j<N; j++ )
      for( unsigned int i=j; i<N; i++ ){
        if( i == j ){
	  Lk[i*N+i] = Op<T>::l( D2X[i].deriv(0).deriv(i) );
	  Uk[i*N+i] = Op<T>::u( D2X[i].deriv(0).deriv(i) );
          continue;
	}
        T D2Fij;
        if( !inter( D2Fij, D2X[i].deriv(0).deriv(j), D2X[j].deriv(0).deriv(i) ) )
          D2Fij = D2X[i].deriv(0).deriv(j);
        Lk[i*N+j] = Lk[j*N+i] = S[k*N+i]*S[k*N+j]==1? Op<T>::l( D2Fij ): Op<T>::u( D2Fij );
        Uk[i*N+j] = Uk[j*N+i] = S[k*N+i]*S[k*N+j]==1? Op<T>::u( D2Fij ): Op<T>::l( D2Fij );
      }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( N, N, Lk, N, "\nMatrix Lk", std::cout );
    mc::display( N, N, Uk, N, "\nMatrix Uk", std::cout );
#endif

    double*DLk = mc::dsyev_wrapper( N, Lk );
    if( !DLk ){
      delete[] Lk; delete[] Uk; delete[] S;
      throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
    }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( 1, N, DLk, 1, "\nMatrix DLk", std::cout );
#endif
    spbnd.first = k? std::min( spbnd.first, min(N,DLk) ): min(N,DLk);
    delete[] DLk;

    double*DUk = mc::dsyev_wrapper( N, Uk );
    if( !DUk ){
      delete[] Lk; delete[] Uk; delete[] S;
      throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
    }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( 1, N, DUk, 1, "\nMatrix DUk", std::cout );
#endif
    spbnd.second = k? std::max( spbnd.second, max(N,DUk) ): max(N,DUk);
    delete[] DUk;

  }
  delete[] Lk; delete[] Uk; delete[] S;
  return spbnd;
}
*/
template <typename T> inline std::pair<double,double>
Specbnd<T>::spectral_bound
( const fadbad::F< fadbad::F< T > >& D2F )
{
  const unsigned int N = D2F.size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
  T pD2F[N*N];
  for( unsigned int i=0, k=0; i<N; i++ )
    for( unsigned int j=0; j<N; j++, k++ )
      pD2F[k] = D2F.deriv(i).deriv(j);

  switch( options.HESSBND ){
  case Options::GERSHGORIN:
    return _gershgorin_bound( N, pD2F );
  case Options::HERTZROHN: default:
    return _hertzrohn_bound( N, pD2F );
  }
}
/*
template <typename T> inline std::pair<double,double>
Specbnd<T>::_gershgorin_bound
( const fadbad::F< fadbad::F< T > >& D2F )
{
  const unsigned int N = D2F.size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );

  std::pair<double,double> spbnd;
  for( unsigned int i=0; i<N; i++ ){
    double ri = 0.;
    for( unsigned int j=0; j<N; j++ ){
      if( j == i ) continue;
      T D2Fij;
      if( !inter( D2Fij, D2F.deriv(i).deriv(j), D2F.deriv(j).deriv(i) ) )
        D2Fij = D2F.deriv(i).deriv(j); 
      ri += Op<T>::abs( D2Fij );
    }

    if( !i ){
      spbnd.first  = Op<T>::l(D2F.deriv(i).deriv(i)) - ri;
      spbnd.second = Op<T>::u(D2F.deriv(i).deriv(i)) + ri;
    }
    else{
      spbnd.first  = std::min( spbnd.first,  Op<T>::l(D2F.deriv(i).deriv(i)) - ri );
      spbnd.second = std::max( spbnd.second, Op<T>::u(D2F.deriv(i).deriv(i)) + ri );
    }
  }
  return spbnd;
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::_hertzrohn_bound
( const fadbad::F< fadbad::F< T > >& D2F )
{
  const unsigned int N = D2F.size();
  if( !N ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );

  // Form matrix S^{(N)} recursively
  unsigned int col_S = 2;
  short *S = new short[col_S];
  S[0] = 1; S[1] = -1;
#ifdef MC__SPECBND_DEBUG_HESSBND
  mc::display( 1, col_S, S, 1, "\nMatrix S1", std::cout );
#endif
  for( unsigned int k=1; k<N; k++, col_S*=2 ){
    short *Sprev = new short[k*col_S];
    for( unsigned int i=0; i<k; i++ )
      for( unsigned int j=0; j<col_S; j++ )
        Sprev[j*k+i] = S[j*k+i];
    delete[] S; S = new short[(k+1)*2*col_S];
    for( unsigned int i=0; i<k; i++ ){
      for( unsigned int j=0; j<col_S; j++ )
        S[j*(k+1)+i] = S[(col_S+j)*(k+1)+i] = Sprev[j*k+i];
    }
    for( unsigned int j=0; j<col_S; j++ ){
      S[j*(k+1)+k] = 1; S[(col_S+j)*(k+1)+k] = -1;
    }
    delete[] Sprev;
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( k+1, 2*col_S, S, k+1, "\nMatrix Sk", std::cout );
#endif
  }

#ifdef MC__SPECBND_DEBUG_HESSBND
  T *H = new T[N*N];
  for( unsigned int j=0; j<N; j++ )
    for( unsigned int i=j; i<N; i++ ){
      if( i == j ){
        H[i*N+i] = D2F.deriv(i).deriv(i);
        continue;
      }
      T D2Fij;
      if( !inter( D2Fij, D2F.deriv(i).deriv(j), D2F.deriv(j).deriv(i) ) )
        D2Fij = D2F.deriv(i).deriv(j);
      H[i*N+j] = H[j*N+i] = D2Fij;
    }
  mc::display( N, N, H, N, "\nMatrix H", std::cout );
  delete[] H;
#endif

  // Compute lower and upper bound on spectral radius
  std::pair<double,double> spbnd;
  double *Lk = new double[N*N], *Uk = new double[N*N];
  for( unsigned int k=0; k<col_S; k++ ){
    
    for( unsigned int j=0; j<N; j++ )
      for( unsigned int i=j; i<N; i++ ){
        if( i == j ){
	  Lk[i*N+i] = Op<T>::l( D2F.deriv(i).deriv(i) );
	  Uk[i*N+i] = Op<T>::u( D2F.deriv(i).deriv(i) );
          continue;
	}
        T D2Fij;
        if( !inter( D2Fij, D2F.deriv(i).deriv(j), D2F.deriv(j).deriv(i) ) )
          D2Fij = D2F.deriv(i).deriv(j);
        Lk[i*N+j] = Lk[j*N+i] = S[k*N+i]*S[k*N+j]==1? Op<T>::l( D2Fij ): Op<T>::u( D2Fij );
        Uk[i*N+j] = Uk[j*N+i] = S[k*N+i]*S[k*N+j]==1? Op<T>::u( D2Fij ): Op<T>::l( D2Fij );
      }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( N, N, Lk, N, "\nMatrix Lk", std::cout );
    mc::display( N, N, Uk, N, "\nMatrix Uk", std::cout );
#endif

    double*DLk = mc::dsyev_wrapper( N, Lk );
    if( !DLk ){
      delete[] Lk; delete[] Uk; delete[] S;
      throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
    }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( 1, N, DLk, 1, "\nMatrix DLk", std::cout );
#endif
    spbnd.first = k? std::min( spbnd.first, min(N,DLk) ): min(N,DLk);
    delete[] DLk;

    double*DUk = mc::dsyev_wrapper( N, Uk );
    if( !DUk ){
      delete[] Lk; delete[] Uk; delete[] S;
      throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
    }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( 1, N, DUk, 1, "\nMatrix DUk", std::cout );
#endif
    spbnd.second = k? std::max( spbnd.second, max(N,DUk) ): max(N,DUk);
    delete[] DUk;

  }
  delete[] Lk; delete[] Uk; delete[] S;
  return spbnd;
}
*/

template <typename T> inline std::pair<double,double>
Specbnd<T>::spectral_bound_re
( const unsigned N, const T*A )
{
  if( !N || !A ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
#ifdef MC__SPECBND_DEBUG_HESSBND
  mc::display( N, N, A, N, "\nMatrix A", std::cout );
#endif
  T ARe[N*N];
  for( unsigned int i=0; i<N; i++ )
    for( unsigned int j=0; j<N; j++ )
      ARe[i+N*j] = ( A[i+N*j] + A[i*N+j] ) / 2.;
#ifdef MC__SPECBND_DEBUG_HESSBND
  mc::display( N, N, ARe, N, "\nMatrix ARe", std::cout );
#endif
  return spectral_bound( N, ARe );
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::spectral_bound_im
( const unsigned N, const T*A )
{
  if( !N || !A ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
  const unsigned N2 = 2*N;
#ifdef MC__SPECBND_DEBUG_HESSBND
  mc::display( N, N, A, N, "\nMatrix A", std::cout );
#endif
  T AIm[N2*N2];
  for( unsigned int i=0; i<N; i++ )
    for( unsigned int j=0; j<N; j++ ){
      AIm[i+N2*j] = AIm[N2*N+N+i+N2*j] = 0.;
      AIm[N+i+N2*j] = ( A[i+N*j] - A[i*N+j] ) / 2.;
      AIm[N2*N+i+N2*j] = - AIm[N+i+N2*j];
    }
#ifdef MC__SPECBND_DEBUG_HESSBND
  mc::display( N2, N2, AIm, N2, "\nMatrix AIm", std::cout );
#endif
  return spectral_bound( N2, AIm );
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::spectral_bound
( const unsigned N, const T*D2F )
{
  if( !N || !D2F ) throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
  switch( options.HESSBND ){
  case Options::GERSHGORIN:
    return _gershgorin_bound( N, D2F );
  case Options::HERTZROHN: default:
    return _hertzrohn_bound( N, D2F );
  }
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::_gershgorin_bound
( const unsigned N, const T*D2F )
{
  struct ndx{
    static unsigned cw( const unsigned i, const unsigned j, const unsigned n ) { return i+j*n; }
    static unsigned rw( const unsigned i, const unsigned j, const unsigned n ) { return i*n+j; }
  };
  std::pair<double,double> spbnd;

  for( unsigned int i=0; i<N; i++ ){
    double ri = 0.;
    for( unsigned int j=0; j<N; j++ ){
      if( j == i ) continue;
      T D2Fij;
      if( !inter( D2Fij, D2F[ndx::rw(i,j,N)], D2F[ndx::rw(j,i,N)] ) )
        D2Fij = D2F[ndx::rw(i,j,N)]; 
      ri += Op<T>::abs( D2Fij );
    }

    if( !i ){
      spbnd.first  = Op<T>::l(D2F[ndx::rw(i,i,N)]) - ri;
      spbnd.second = Op<T>::u(D2F[ndx::rw(i,i,N)]) + ri;
    }
    else{
      spbnd.first  = std::min( spbnd.first,  Op<T>::l(D2F[ndx::rw(i,i,N)]) - ri );
      spbnd.second = std::max( spbnd.second, Op<T>::u(D2F[ndx::rw(i,i,N)]) + ri );
    }
  }

  return spbnd;
}

template <typename T> inline std::pair<double,double>
Specbnd<T>::_hertzrohn_bound
( const unsigned N, const T*D2F )
{
  struct ndx{
    static unsigned cw( const unsigned i, const unsigned j, const unsigned n ) { return i+j*n; }
    static unsigned rw( const unsigned i, const unsigned j, const unsigned n ) { return i*n+j; }
  };

  // Form matrix S^{(N)} recursively
  unsigned int col_S = 2;
  short *S = new short[col_S];
  S[0] = 1; S[1] = -1;
#ifdef MC__SPECBND_DEBUG_HESSBND
  mc::display( 1, col_S, S, 1, "\nMatrix S1", std::cout );
#endif
  for( unsigned int k=1; k<N; k++, col_S*=2 ){
    short *Sprev = new short[k*col_S];
    for( unsigned int i=0; i<k; i++ )
      for( unsigned int j=0; j<col_S; j++ )
        Sprev[j*k+i] = S[j*k+i];
    delete[] S; S = new short[(k+1)*2*col_S];
    for( unsigned int i=0; i<k; i++ ){
      for( unsigned int j=0; j<col_S; j++ )
        S[j*(k+1)+i] = S[(col_S+j)*(k+1)+i] = Sprev[j*k+i];
    }
    for( unsigned int j=0; j<col_S; j++ ){
      S[j*(k+1)+k] = 1; S[(col_S+j)*(k+1)+k] = -1;
    }
    delete[] Sprev;
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( k+1, 2*col_S, S, k+1, "\nMatrix Sk", std::cout );
#endif
  }

#ifdef MC__SPECBND_DEBUG_HESSBND
  T *H = new T[N*N];
  for( unsigned int j=0; j<N; j++ )
    for( unsigned int i=j; i<N; i++ ){
      if( i == j ){
        H[i*N+i] = D2F[ndx::rw(i,i,N)];
        continue;
      }
      T D2Fij;
      if( !inter( D2Fij, D2F[ndx::rw(i,j,N)], D2F[ndx::rw(j,i,N)] ) )
        D2Fij = D2F[ndx::rw(i,j,N)];
      H[i*N+j] = H[j*N+i] = D2Fij;
    }
  mc::display( N, N, H, N, "\nMatrix H", std::cout );
  delete[] H;
#endif

  // Compute lower and upper bound on spectral radius
  std::pair<double,double> spbnd;
  double *Lk = new double[N*N], *Uk = new double[N*N];
  for( unsigned int k=0; k<col_S; k++ ){
    
    for( unsigned int j=0; j<N; j++ )
      for( unsigned int i=j; i<N; i++ ){
        if( i == j ){
	  Lk[i*N+i] = Op<T>::l( D2F[ndx::rw(i,i,N)] );
	  Uk[i*N+i] = Op<T>::u( D2F[ndx::rw(i,i,N)] );
          continue;
	}
        T D2Fij;
        if( !inter( D2Fij, D2F[ndx::rw(i,j,N)], D2F[ndx::rw(j,i,N)] ) )
          D2Fij = D2F[ndx::rw(i,j,N)];
        Lk[i*N+j] = Lk[j*N+i] = S[k*N+i]*S[k*N+j]==1? Op<T>::l( D2Fij ): Op<T>::u( D2Fij );
        Uk[i*N+j] = Uk[j*N+i] = S[k*N+i]*S[k*N+j]==1? Op<T>::u( D2Fij ): Op<T>::l( D2Fij );
      }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( N, N, Lk, N, "\nMatrix Lk", std::cout );
    mc::display( N, N, Uk, N, "\nMatrix Uk", std::cout );
#endif

    double*DLk = mc::dsyev_wrapper( N, Lk );
    if( !DLk ){
      delete[] Lk; delete[] Uk; delete[] S;
      throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
    }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( 1, N, DLk, 1, "\nMatrix DLk", std::cout );
#endif
    spbnd.first = k? std::min( spbnd.first, min(N,DLk) ): min(N,DLk);
    delete[] DLk;

    double*DUk = mc::dsyev_wrapper( N, Uk );
    if( !DUk ){
      delete[] Lk; delete[] Uk; delete[] S;
      throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::HESSBND );
    }
#ifdef MC__SPECBND_DEBUG_HESSBND
    mc::display( 1, N, DUk, 1, "\nMatrix DUk", std::cout );
#endif
    spbnd.second = k? std::max( spbnd.second, max(N,DUk) ): max(N,DUk);
    delete[] DUk;

  }
  delete[] Lk; delete[] Uk; delete[] S;
  return spbnd;
}

template <class T> inline std::ostream&
operator<<
( std::ostream &out, const Specbnd<T> &y )
{
  out << "  " << y._spec << std::endl
      << "  " << y._FI.val() << std::endl;
  for( unsigned int i=0; i<y._n; i++ )
    out << "  ( " << y._FI.deriv(i) << " )" << std::endl;
  return out;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::operator=
( const double c )
{ 
  _n = 0;
  _FI = c;
  _spec = 0.;
  return *this;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::operator=
( const T&I )
{ 
  _n = 0;
  _FI = I;
  _spec = 0.;
  return *this;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::set
( const fadbad::F<T>&FB, const T&SB )
{ 
  _n = FB.size();
  _FI = FB;
  _spec = SB;
  return *this;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::operator=
( const Specbnd<T> &x )
{ 
  _n = x._n;
  _FI = x._FI;
  _spec = x._spec;
  return *this;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::operator+=
( const double c )
{ 
  _FI += c;
  return *this;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::operator+=
( const Specbnd<T> &y )
{ 
  if( _n && y._n && _n != y._n )
    throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SIZE );
  _FI += y._FI;
  _n = _FI.size();
  _spec += y.spec;
  return *this;
}

template <class T> inline Specbnd<T>
operator+
( const Specbnd<T> &y )
{
  return y;
}

template <class T> Specbnd<T>
operator+
( const Specbnd<T> &x, const Specbnd<T> &y )
{
  if( x._n && y._n && x._n != y._n )
    throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SIZE );
  Specbnd<T> z;
  z._FI = x._FI + y._FI;
  z._n = z._FI.size();
  z._spec = x._spec + y._spec;
  return z;
}

template <class T> Specbnd<T>
operator+
( const Specbnd<T> &y, const double c )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = y._FI + c;
  z._spec = y._spec;
  return z;
}

template <class T> Specbnd<T>
operator+
( const double c, const Specbnd<T> &y )
{
  return y + c;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::operator-=
( const double c )
{ 
  _FI -= c;
  return *this;
}

template <class T> inline Specbnd<T>&
Specbnd<T>::operator-=
( const Specbnd<T> &y )
{ 
  if( _n && y._n && _n != y._n )
    throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SIZE );
  _FI -= y._FI;
  _n = _FI.size();
  _spec -= y.spec;
  return *this;
}

template <class T> inline Specbnd<T>
operator-
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = -y._FI;
  z._spec = -y._spec;
  return z;
}

template <class T> inline Specbnd<T>
operator-
( const Specbnd<T> &x, const Specbnd<T> &y )
{
  if( x._n && y._n && x._n != y._n )
    throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SIZE );
  Specbnd<T> z;
  z._FI = x._FI - y._FI;
  z._n = z._FI.size();
  z._spec = x._spec - y._spec;
  return z;
}

template <class T> inline Specbnd<T>
operator-
( const Specbnd<T> &y, const double c )
{
  return y + (-c);
}

template <class T> inline Specbnd<T>
operator-
( const double c, const Specbnd<T> &y )
{
  return (-y) + c;
}

template <typename T> inline Specbnd<T>&
Specbnd<T>::operator*=
( const double c )
{
  Specbnd<T> z = c * (*this);
  *this = z;
  return *this;
}

template <typename T> inline Specbnd<T>&
Specbnd<T>::operator*=
( const Specbnd<T> &x )
{
  Specbnd<T> z = x * (*this);
  *this = z;
  return *this;
}

template <class T> inline Specbnd<T>
operator*
( const Specbnd<T> &x, const Specbnd<T> &y )
{
  if( x._n && y._n && x._n != y._n )
    throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::SIZE );
  Specbnd<T> z;
  z._FI = x._FI * y._FI;
  z._n = z._FI.size();
  z._spec = x._FI.val() * y._spec + x._spec * y._FI.val()
          + Specbnd<T>::_LambdaT( x._FI, y._FI, z._n );
  return z;
}

template <class T> inline Specbnd<T>
operator*
( const double c, const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = c * y._FI;
  z._spec = c * y._spec;
  return z;
}

template <class T> inline Specbnd<T>
operator*
( const Specbnd<T> &y, const double c )
{
  return c * y;
}

template <class T> inline Specbnd<T>
sqr
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::sqr( y._FI );
  z._spec = 2.*( y._FI.val() * y._spec + Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
pow
( const Specbnd<T> &y, const int m )
{
  if( !m )      return 1.;
  if( m == 1 )  return y;
  if( m == 2 )  return sqr(y);
  if( m == -1 ) return inv( y );
  if( m < -1 )  return inv( pow( y, -m ) );
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::pow( y._FI, m );
  z._spec = (double)m * Op<T>::pow( y._FI.val(), m-2 )
          * ( y._FI.val() * y._spec + (m-1) * Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
inv
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = 1./y._FI;
  z._spec = Op<T>::sqr( z._FI.val() )
         * ( 2. * z._FI.val() * Specbnd<T>::_LambdaS( y._FI, z._n ) - y._spec );
  return z;
}

template <typename T> inline Specbnd<T>&
Specbnd<T>::operator/=
( const double c )
{
  return *this / c;
}

template <typename T> inline Specbnd<T>&
Specbnd<T>::operator/=
( const Specbnd<T> &x )
{
  return *this / x;
}

template <class T> inline Specbnd<T>
operator/
( const Specbnd<T> &x, const Specbnd<T> &y )
{
  return x * inv(y);
}

template <class T> inline Specbnd<T>
operator/
( const Specbnd<T> &y, const double c )
{
  return y * (1./c);
}

template <class T> inline Specbnd<T>
operator/
( const double c, const Specbnd<T> &y )
{
  return c * inv(y);
}

template <class T> inline Specbnd<T>
sqrt
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::sqrt( y._FI );
  z._spec = 1./(2.*Op<T>::sqr( z._FI.val() ))
         * ( y._spec - Specbnd<T>::_LambdaS( y._FI, z._n ) / (2. * y._FI.val()) );
  return z;
}

template <class T> inline Specbnd<T>
exp
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::exp( y._FI );
  z._spec = z._FI.val() * ( y._spec + Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
log
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::log( y._FI );
  z._spec = 1./y._FI.val() * ( y._spec - Specbnd<T>::_LambdaS( y._FI, z._n ) / y._FI.val() );
  return z;
}

template <class T> inline Specbnd<T>
xlog
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = y._FI*fadbad::log( y._FI );
  z._spec = (Op<T>::log(y._FI.val())+1.)*y._spec
    + Specbnd<T>::_LambdaS( y._FI, z._n ) / y._FI.val();
  return z;
}

template <class T> inline Specbnd<T>
pow
( const Specbnd<T> &x, const Specbnd<T> &y )
{
  return exp( y * log( x ) );
}

template <class T> inline Specbnd<T>
pow
( const double c, const Specbnd<T> &y )
{
  return exp( y * std::log( c ) );
}

template <class T> inline Specbnd<T>
pow
( const Specbnd<T> &x, const double c )
{
  return exp( c * log( x ) );
}

template <class T> inline Specbnd<T>
monomial
( const unsigned int n, const Specbnd<T>*x, const int*k )
{
  if( n == 0 ){
    return 1.;
  }
  if( n == 1 ){
    return pow( x[0], k[0] );
  }
  return pow( x[0], k[0] ) * monomial( n-1, x+1, k+1 );
}

template <typename T> inline Specbnd<T>
cheb
( const Specbnd<T> &x, const unsigned n )
{
  switch( n ){
    case 0:  return 1.;
    case 1:  return x;
    default: break;
  }
  return 2.*(x*cheb(x,n-1))-cheb(x,n-2);
}

template <class T> inline Specbnd<T>
fabs
( const Specbnd<T> &y )
{
  throw typename Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF );
}

template <class T> inline Specbnd<T>
cos
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::cos( y._FI );
  z._spec = - z._FI.val() * ( y._spec + Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
sin
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::sin( y._FI );
  z._spec = z._FI.val() * ( y._spec - Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
tan
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::tan( y._FI );
  z._spec = ( Op<T>::sqr(z._FI.val()) + 1. ) * ( y._spec
    + 2. * y._FI.val() * Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
acos
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::acos( y._FI );
  z._spec = - 1./Op<T>::sqrt(1.-Op<T>::sqr(y._FI.val())) * ( y._spec
    + y._FI.val()/(1.-Op<T>::sqr(y._FI.val()))
     *Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
asin
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::asin( y._FI );
  z._spec = 1./Op<T>::sqrt(1.-Op<T>::sqr(y._FI.val())) * ( y._spec
    + y._FI.val()/(1.-Op<T>::sqr(y._FI.val()))
     *Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
atan
( const Specbnd<T> &y )
{
  Specbnd<T> z;
  z._n = y._n;
  z._FI = fadbad::atan( y._FI );
  z._spec = 1./(Op<T>::sqr(y._FI.val())+1.) * ( y._spec
    - 2.*y._FI.val()/(Op<T>::sqr(y._FI.val())+1.)
     *Specbnd<T>::_LambdaS( y._FI, z._n ) );
  return z;
}

template <class T> inline Specbnd<T>
erf
( const Specbnd<T> &y )
{
  throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF );
//   Specbnd<T> z;
//   z._n = y._n;
//   z._FI = fadbad::erf( y._FI );
//   z._spec = 2./std::sqrt(PI)*Op<T>::exp(-Op<T>::sqr(y._FI.val())) * ( y._spec
//     - 2.*y._FI.val()*Specbnd<T>::_LambdaS( y._FI, z._n ) );
//   return z;
}

template <class T> inline Specbnd<T>
erfc
( const Specbnd<T> &y )
{
  throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF );
//   Specbnd<T> z;
//   z._n = y._n;
//   z._FI = fadbad::erfc( y._FI );
//   z._spec = - 2./std::sqrt(PI)*Op<T>::exp(-Op<T>::sqr(y._FI.val())) * ( y._spec
//     - 2.*y._FI.val()*Specbnd<T>::_LambdaS( y._FI, z._n ) );
//   return z;
}

} // namespace mc


#include "mcop.hpp"

namespace mc
{

//! @brief C++ structure for specialization of the mc::Op templated structure for use of mc::Specbnd in other MC++ classes
template< typename T > struct Op< mc::Specbnd<T> >
{
  typedef mc::Specbnd<T> SB;
  static SB point( const double c ) { return SB(c); }
  static SB zeroone() { return SB( mc::Op<T>::zeroone() ); }
  static void I(SB& x, const SB&y) { x = y; }
  static double l(const SB& x) { return Op<T>::l(x.I()); }
  static double u(const SB& x) { return Op<T>::u(x.I()); }
  static double abs (const SB& x) { return mc::Op<T>::abs(x.I());  }
  static double mid (const SB& x) { return mc::Op<T>::mid(x.I());  }
  static double diam(const SB& x) { return mc::Op<T>::diam(x.I()); }
  static SB inv (const SB& x) { return mc::inv(x);  }
  static SB sqr (const SB& x) { return mc::sqr(x);  }
  static SB sqrt(const SB& x) { return mc::sqrt(x); }
  static SB log (const SB& x) { return mc::log(x);  }
  static SB xlog(const SB& x) { return x*mc::log(x); }
  static SB fabs(const SB& x) { return mc::fabs(x); }
  static SB exp (const SB& x) { return mc::exp(x);  }
  static SB sin (const SB& x) { return mc::sin(x);  }
  static SB cos (const SB& x) { return mc::cos(x);  }
  static SB tan (const SB& x) { return mc::tan(x);  }
  static SB asin(const SB& x) { return mc::asin(x); }
  static SB acos(const SB& x) { return mc::acos(x); }
  static SB atan(const SB& x) { return mc::atan(x); }
  static SB erf (const SB& x) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static SB erfc(const SB& x) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static SB fstep(const SB& x) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static SB bstep(const SB& x) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static SB hull(const SB& x, const SB& y) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static SB min (const SB& x, const SB& y) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static SB max (const SB& x, const SB& y) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static SB arh (const SB& x, const double k) { return mc::exp(-k/x); }
  static SB cheb (const SB& x, const unsigned n) { return mc::cheb(x,n); }
  template <typename X, typename Y> static SB pow(const X& x, const Y& y) { return mc::pow(x,y); }
  static SB monomial (const unsigned int n, const T* x, const int* k) { return mc::monomial(n,x,k); }
  static bool inter(SB& xIy, const SB& x, const SB& y) { throw typename mc::Specbnd<T>::Exceptions( Specbnd<T>::Exceptions::UNDEF ); }
  static bool eq(const SB& x, const SB& y) { return x.SI()==y.SI() && x.FI()==y.FI(); }
  static bool ne(const SB& x, const SB& y) { return x.SI()!=y.SI() || x.FI()==y.FI(); }
  static bool lt(const SB& x, const SB& y) { return x.SI()<y.SI()  && x.FI()<y.FI();  }
  static bool le(const SB& x, const SB& y) { return x.SI()<=y.SI() && x.FI()<=y.FI(); }
  static bool gt(const SB& x, const SB& y) { return x.SI()>y.SI()  && x.FI()>y.FI();  }
  static bool ge(const SB& x, const SB& y) { return x.SI()>=y.SI() && x.FI()>=y.FI(); }
};

} // namespace mc

#endif

